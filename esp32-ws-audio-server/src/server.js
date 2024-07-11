const path = require("path");
const express = require("express");
const WebSocket = require("ws");
const app = express();

const WS_PORT = process.env.WS_PORT || 8888;
const HTTP_PORT = process.env.HTTP_PORT || 8000;
const ESP32_USER_AGENT = "arduino-WebSocket-Client";

let connectedESP32s = new Set();

const wss = new WebSocket.Server({ port: WS_PORT, clientTracking: true }, () =>
  console.log(`WS server is listening at ws://localhost:${WS_PORT}`)
);

wss.on("connection", (ws, req) => {
    const userAgent = req.headers['user-agent'];
    if (userAgent == ESP32_USER_AGENT) {
        console.log("ESP32S3 connected with ip: ", ws._socket.remoteAddress);
        connectedESP32s.add(ws._socket.remoteAddress);
        ws.on("message", (data) => {
            wss.clients.forEach((ws, i) => {
                if (ws.readyState === ws.OPEN && !connectedESP32s.has(ws._socket.remoteAddress)) {
                    ws.send(data);
                }
            });
        });
    } else {
        console.log("Browser connected with ip: ", ws._socket.remoteAddress);
    }
    // Log when disconnected in console
    ws.on("close", function() {
        console.log(req.socket.remoteAddress, " disconnected");
        if (typeof ws._socket.remoteAddress !== 'undefined') {
            connectedESP32s.delete(ws._socket.remoteAddress);
        }
    });
});

express.static.mime.types['wasm'] = 'application/wasm';
app.use("/js", express.static("js"));
app.get("/audio", (req, res) =>
  res.sendFile(path.resolve(__dirname, "./index.html"))
);
app.listen(HTTP_PORT, () =>
  console.log(`HTTP server listening at http://localhost:${HTTP_PORT}`)
);