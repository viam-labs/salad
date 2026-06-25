import * as VIAM from "@viamrobotics/sdk";
import Cookies from "js-cookie";
import type { Ingredient, SetupResult, ZonesResult } from "./types";

let robotClient: VIAM.RobotClient;
let coordinator: VIAM.GenericServiceClient;
let streamClient: VIAM.StreamClient;
let cameraStream: MediaStream;

// Captured at connect time; reused to lazily build the dashboard's ViamClient
// and resolve its org/location.
let apiKeyId = "";
let apiKeySecret = "";
let machinePartId = "";

// {orgId, robotId} the Data API needs, derived at runtime and cached.
// Populated lazily when the dashboard first opens.
interface DashboardContext {
  client: VIAM.ViamClient;
  orgId: string;
  robotId: string;
}
let dashboardCtx: DashboardContext | null = null;
let dashboardCtxPromise: Promise<DashboardContext> | null = null;

export async function initConnection(): Promise<void> {
  const machineCookieKey = window.location.pathname.split("/")[2];
  const cookie = Cookies.get(machineCookieKey);
  if (!cookie) {
    throw new Error(
      "No machine cookie found. Are you running inside the Viam app?",
    );
  }

  const parsed = JSON.parse(cookie);
  apiKeyId = parsed.apiKey.id;
  apiKeySecret = parsed.apiKey.key;
  // The cookie key is the machine part id; used to resolve location/org.
  machinePartId = machineCookieKey;

  robotClient = await VIAM.createRobotClient({
    host: parsed.hostname,
    credentials: {
      type: "api-key",
      payload: apiKeySecret,
      authEntity: apiKeyId,
    },
    signalingAddress: "https://app.viam.com",
  });

  coordinator = new VIAM.GenericServiceClient(robotClient, "salad-coordinator");
  streamClient = new VIAM.StreamClient(robotClient);
  cameraStream = await streamClient.getStream("overhead-webcam");
}

// Resolves and caches the ViamClient + {orgId, robotId} the Data API needs.
// The ViamClient is created here (not at startup) so the cloud handshake only
// happens when the dashboard opens; concurrent callers share one in-flight
// promise.
export function getDashboardContext(): Promise<DashboardContext> {
  if (dashboardCtx) return Promise.resolve(dashboardCtx);
  if (dashboardCtxPromise) return dashboardCtxPromise;
  dashboardCtxPromise = (async () => {
    if (!machinePartId) {
      throw new Error(
        "Machine part id not available; initConnection must complete first.",
      );
    }
    const client = await VIAM.createViamClient({
      credentials: {
        type: "api-key",
        payload: apiKeySecret,
        authEntity: apiKeyId,
      },
    });
    const partResp = await client.appClient.getRobotPart(machinePartId);
    const part = partResp.part;
    if (!part) {
      throw new Error(`getRobotPart(${machinePartId}) returned no part.`);
    }
    const robotId = part.robot;
    if (!robotId) {
      throw new Error("Robot part is missing robot id.");
    }
    // location_id is only needed to resolve the owning org for the Data API.
    const locationId = part.locationId;
    if (!locationId) {
      throw new Error("Robot part is missing location_id.");
    }
    const location = await client.appClient.getLocation(locationId);
    // Fall back to the first sharing org if there's no primary owner.
    const orgId =
      location?.primaryOrgIdentity?.id ??
      location?.organizations?.[0]?.organizationId ??
      "";
    if (!orgId) {
      throw new Error(`Could not resolve org id for location ${locationId}.`);
    }
    dashboardCtx = { client, orgId, robotId };
    return dashboardCtx;
  })();
  // Reset the in-flight promise on failure so a later caller can retry.
  dashboardCtxPromise.catch(() => {
    dashboardCtxPromise = null;
  });
  return dashboardCtxPromise;
}

export async function fetchTheme(): Promise<string> {
  const result = (await coordinator.doCommand({
    get_theme: true,
  })) as unknown as { theme?: string };
  return result.theme ?? "salad";
}

export async function fetchIngredients(): Promise<Ingredient[]> {
  const result = (await coordinator.doCommand({
    list_ingredients: true,
  })) as unknown as { ingredients: Ingredient[] };
  return (result.ingredients ?? []).filter(
    (ing): ing is Ingredient =>
      typeof ing?.name === "string" && ing.name.length > 0,
  );
}

export async function buildSalad(
  payload: Record<string, number>,
  customerName?: string,
): Promise<void> {
  await coordinator.doCommand({
    build_salad: payload,
    ...(customerName ? { customer_name: customerName } : {}),
  });
}

export async function getStatus(): Promise<{
  status: string;
  progress: number;
  error_msg?: string;
}> {
  return (await coordinator.doCommand({
    status: true,
  })) as unknown as { status: string; progress: number; error_msg?: string };
}

export async function setupStation(): Promise<void> {
  await coordinator.doCommand({ setup_station: true });
}

export async function stopBuild(): Promise<void> {
  await coordinator.doCommand({ stop: true });
}

export function getCameraStream(): MediaStream {
  return cameraStream;
}

export async function getSetupResult(): Promise<SetupResult> {
  const result = (await coordinator.doCommand({
    get_setup_result: true,
  })) as unknown as { pcd: string; zones: ZonesResult };

  const binary = atob(result.pcd);
  const pcd = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) {
    pcd[i] = binary.charCodeAt(i);
  }

  return { pcd, zones: result.zones };
}
