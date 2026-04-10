import * as VIAM from "@viamrobotics/sdk";
import Cookies from "js-cookie";
import type { Ingredient, QueueSnapshot } from "./types";

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
}> {
  return (await coordinator.doCommand({
    status: true,
  })) as unknown as { status: string; progress: number };
}

export async function stopBuild(): Promise<void> {
  await coordinator.doCommand({ stop: true });
}

export function getCameraStream(): MediaStream {
  return cameraStream;
}

export async function queueOrder(
  payload: Record<string, number>,
  customerName?: string,
): Promise<{ status: string; order_id: string; queue_position: number }> {
  return (await coordinator.doCommand({
    queue_order: payload,
    ...(customerName ? { customer_name: customerName } : {}),
  })) as unknown as { status: string; order_id: string; queue_position: number };
}

export async function getQueue(): Promise<QueueSnapshot> {
  return (await coordinator.doCommand({
    get_queue: true,
  })) as unknown as QueueSnapshot;
}

export async function cancelOrder(orderId: string): Promise<{ status: string; order_id: string }> {
  return (await coordinator.doCommand({
    cancel_order: orderId,
  })) as unknown as { status: string; order_id: string };
}
