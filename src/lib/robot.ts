import * as VIAM from "@viamrobotics/sdk";
import Cookies from "js-cookie";
import type { Ingredient, SetupResult, ZonesResult } from "./types";

let robotClient: VIAM.RobotClient;
let coordinator: VIAM.GenericServiceClient;
let streamClient: VIAM.StreamClient;
let cameraStream: MediaStream;

export async function initConnection(): Promise<void> {
  let apiKeyId = "";
  let apiKeySecret = "";
  let host = "";

  const machineCookieKey = window.location.pathname.split("/")[2];
  const cookie = Cookies.get(machineCookieKey);
  if (!cookie) {
    throw new Error("No machine cookie found. Are you running inside the Viam app?");
  }

  const parsed = JSON.parse(cookie);
  apiKeyId = parsed.apiKey.id;
  apiKeySecret = parsed.apiKey.key;
  host = parsed.hostname;

  robotClient = await VIAM.createRobotClient({
    host,
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

export async function fetchIngredients(): Promise<Ingredient[]> {
  const result = (await coordinator.doCommand({
    list_ingredients: true,
  })) as unknown as { ingredients: Ingredient[] };
  return result.ingredients ?? [];
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
