import { FoxgloveServer } from "@foxglove/ws-protocol";

function delay(durationSec) {
  return new Promise((resolve) => setTimeout(resolve, durationSec * 1000));
}

async function main() {
  const server = new FoxgloveServer({ name: "example-server" });
  const ws = new WebSocketServer({
    port: 8765,
    handleProtocols: (protocols) => server.handleProtocols(protocols),
  });
  ws.on("listening", () => {
    console.log("server listening on %s", ws.address());
  });
  ws.on("connection", (conn, req) => {
    const name = `${req.socket.remoteAddress}:${req.socket.remotePort}`;
    console.log("connection from %s via %s", name, req.url);
    server.handleConnection(conn, name);
  });
  server.on("subscribe", (chanId) => {
    console.log("first client subscribed to %d", chanId);
  });
  server.on("unsubscribe", (chanId) => {
    console.log("last client unsubscribed from %d", chanId);
  });
  server.on("error", (err) => {
    console.error("server error: %o", err);
  });

  const ch1 = server.addChannel({
    topic: "example_msg",
    encoding: "json",
    schemaName: "ExampleMsg",
    schema: JSON.stringify({
      type: "object",
      properties: {
        msg: { type: "string" },
        count: { type: "number" },
      },
    }),
  });

  const textEncoder = new TextEncoder();
  let i = 0;
  while (true) {
    await delay(0.2);
    server.sendMessage(
      ch1,
      BigInt(Date.now()) * 1_000_000n,
      textEncoder.encode(JSON.stringify({ msg: "Hello!", count: ++i }))
    );
  }
}

main().catch(console.error);
