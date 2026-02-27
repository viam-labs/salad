import * as VIAM from "@viamrobotics/sdk";
import Cookies from "js-cookie";
import type { Ingredient } from "./types";

let coordinator: VIAM.GenericServiceClient;

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

  const robotClient = await VIAM.createRobotClient({
    host,
    credentials: {
      type: "api-key",
      payload: apiKeySecret,
      authEntity: apiKeyId,
    },
    signalingAddress: "https://app.viam.com",
  });

  coordinator = new VIAM.GenericServiceClient(robotClient, "salad-coordinator");
}

export async function fetchIngredients(): Promise<Ingredient[]> {
  const result = (await coordinator.doCommand({
    list_ingredients: true,
  })) as unknown as { ingredients: Ingredient[] };
  return result.ingredients ?? [];
}

export async function buildSalad(
  payload: Record<string, number>,
): Promise<void> {
  await coordinator.doCommand({ build_salad: payload });
}

export async function getStatus(): Promise<{
  status: string;
  progress: number;
}> {
  return (await coordinator.doCommand({
    status: true,
  })) as unknown as { status: string; progress: number };
}

export async function stopBuild(): Promise<void> {
  await coordinator.doCommand({ stop: true });
}
