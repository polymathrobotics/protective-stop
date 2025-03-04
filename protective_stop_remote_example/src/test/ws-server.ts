import { FoxgloveServer } from "@foxglove/ws-protocol";
import { AddressInfo, Data, WebSocket, WebSocketServer } from "ws";
import ConsumerQueue from "consumer-queue";

async function setupServerAndClient(server: FoxgloveServer) {
  const wss = new WebSocketServer({
    port: 0,
    handleProtocols: server.handleProtocols.bind(server),
  });
  wss.on("connection", (conn, req) => {
    server.handleConnection(conn, `${req.socket.remoteAddress!}:${req.socket.remotePort!}`);
  });
  await new Promise((resolve) => wss.on("listening", resolve));

  const msgQueue = new ConsumerQueue<Data>();
  const ws = new WebSocket(`ws://localhost:${(wss.address() as AddressInfo).port}`);
  ws.binaryType = "arraybuffer";
  ws.onmessage = (event) => {
    msgQueue.push(event.data);
  };

  const nextJsonMessage = async () => {
    const msg = await msgQueue.pop();
    if (typeof msg === "string") {
      return JSON.parse(msg) as unknown;
    }
    throw new Error("Expected string message");
  };
  const nextBinaryMessage = async () => {
    const msg = await msgQueue.pop();
    if (msg instanceof ArrayBuffer) {
      return new Uint8Array(msg);
    }
    throw new Error(`Expected binary message, got: ${typeof msg}`);
  };

  const eventQueue = new ConsumerQueue<unknown[]>();
  server.on("subscribe", (chanId) => {
    eventQueue.push(["subscribe", chanId]);
  });
  server.on("unsubscribe", (chanId) => {
    eventQueue.push(["unsubscribe", chanId]);
  });
  server.on("error", (err) => {
    eventQueue.push(["error", err]);
  });
  server.on("advertise", (event) => {
    eventQueue.push(["advertise", event]);
  });
  server.on("unadvertise", (event) => {
    eventQueue.push(["unadvertise", event]);
  });
  server.on("message", (event) => {
    eventQueue.push(["message", event]);
  });
  server.on("getParameters", (event) => {
    eventQueue.push(["getParameters", event]);
  });
  server.on("setParameters", (event) => {
    eventQueue.push(["setParameters", event]);
  });
  server.on("subscribeParameterUpdates", (event) => {
    eventQueue.push(["subscribeParameterUpdates", event]);
  });
  server.on("unsubscribeParameterUpdates", (event) => {
    eventQueue.push(["unsubscribeParameterUpdates", event]);
  });
  server.on("serviceCallRequest", (event, clientConnection) => {
    eventQueue.push(["serviceCallRequest", event, clientConnection]);
  });
  server.on("fetchAsset", (event, clientConnection) => {
    eventQueue.push(["fetchAsset", event, clientConnection]);
  });

  const nextEvent = async () => await eventQueue.pop();

  const send = (data: Data) => {
    ws.send(data);
  };
  const close = () => {
    msgQueue.cancelWait(new Error("Server was closed"));
    void msgQueue.pop().then((_msg) => {
      throw new Error("Unexpected message on close");
    });
    eventQueue.cancelWait(new Error("Server was closed"));
    void eventQueue.pop().then((event) => {
      throw new Error(`Unexpected event on close: ${event[0] as string}`);
    });
    ws.close();
    wss.close();
  };
  return { server, send, nextJsonMessage, nextBinaryMessage, nextEvent, close };
}