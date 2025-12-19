#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>

// -------- HOTSPOT DETAILS --------
const char* ssid = "ESP32_DASHBOARD";
const char* password = "12345678";

// -------- SAFE RELAY PINS --------
int relayPins[] = {
  2, 4, 5,
  12, 13, 14, 15,
  16, 17,
  18, 19,
  21, 22, 23,
  25, 26, 27,
  32, 33
};
const int RELAY_COUNT = sizeof(relayPins) / sizeof(relayPins[0]);

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// -------- HTML DASHBOARD --------
const char dashboard_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 WebSocket Panel</title>

<style>
body{margin:0;font-family:Arial;background:#0f2027;color:#fff}
header{text-align:center;padding:18px;font-size:22px;background:#00000055}
.container{padding:20px;display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:18px}
.card{background:#ffffff22;border-radius:14px;padding:18px;text-align:center}
.btn{width:100%;padding:16px;border:none;border-radius:14px;font-size:17px;font-weight:bold}
.on{background:#00e676;color:#000}
.off{background:#ff5252;color:#000}
</style>
</head>

<body>
<header>ESP32 WEBSOCKET CONTROL</header>

<div class="container">
<script>
let ws;

function connectWS(){
  ws = new WebSocket("ws://" + location.hostname + ":81/");
  ws.onmessage = (e) => handleMessage(e.data);
}

function handleMessage(msg){
  let p = msg.split(":");
  if(p[0] === "STATUS"){
    let id = p[1];
    let state = p[2];
    let btn = document.getElementById("b"+id);
    if(btn){
      btn.className = "btn " + (state==="ON"?"on":"off");
      btn.innerText = state;
    }
  }
}

function press(id){
  ws.send("ON:"+id);
}
function release(id){
  ws.send("OFF:"+id);
}

window.onload = connectWS;
</script>

<script>
for(let i=1;i<=18;i++){
  document.write(`
    <div class="card">
      <h4>Relay ${i}</h4>
      <button id="b${i}" class="btn off"
        ontouchstart="press(${i})"
        ontouchend="release(${i})"
        onmousedown="press(${i})"
        onmouseup="release(${i})">
        HOLD
      </button>
    </div>
  `);
}
</script>

</body>
</html>
)rawliteral";

// -------- HTTP --------
void handleRoot(){
  server.send(200, "text/html", dashboard_html);
}

// -------- WEBSOCKET EVENTS --------
void webSocketEvent(uint8_t num, WStype_t type,
                    uint8_t * payload, size_t length) {

  if(type == WStype_TEXT){
    String msg = (char*)payload;

    if(msg.startsWith("ON:")){
      int id = msg.substring(3).toInt();
      if(id>=1 && id<=RELAY_COUNT){
        digitalWrite(relayPins[id-1], HIGH);
        webSocket.broadcastTXT("STATUS:"+String(id)+":ON");
      }
    }

    if(msg.startsWith("OFF:")){
      int id = msg.substring(4).toInt();
      if(id>=1 && id<=RELAY_COUNT){
        digitalWrite(relayPins[id-1], LOW);
        webSocket.broadcastTXT("STATUS:"+String(id)+":OFF");
      }
    }
  }
}

// -------- SETUP --------
void setup(){
  Serial.begin(115200);

  for(int i=0;i<RELAY_COUNT;i++){
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }

  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  MDNS.begin("esp32");

  server.on("/", handleRoot);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("WebSocket started on port 81");
}

// -------- LOOP --------
void loop(){
  server.handleClient();
  webSocket.loop();
}
