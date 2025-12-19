#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// -------- HOTSPOT DETAILS --------
const char* ssid = "ESP32_DASHBOARD";
const char* password = "12345678";

// -------- SAFE RELAY PINS (ESP32-WROOM-32) --------
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

// -------- HTML DASHBOARD --------
const char dashboard_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Touch Panel</title>

<style>
body{margin:0;font-family:Arial;background:#0f2027;color:#fff}
header{text-align:center;padding:18px;font-size:22px;background:#00000055}
.container{padding:20px;display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:18px}
.card{background:#ffffff22;border-radius:14px;padding:18px;text-align:center}
.btn{width:100%;padding:16px;border:none;border-radius:14px;font-size:17px;font-weight:bold;user-select:none}
.on{background:#00e676;color:#000}
.off{background:#ff5252;color:#000}
</style>
</head>

<body>
<header>ESP32 TOUCH CONTROL PANEL</header>

<div class="container">
<script>
for(let i=1;i<=18;i++){
  document.write(`
    <div class="card">
      <h4>Relay ${i}</h4>
      <button class="btn off"
        ontouchstart="press(${i},this)"
        ontouchend="release(${i},this)"
        onmousedown="press(${i},this)"
        onmouseup="release(${i},this)">
        HOLD
      </button>
    </div>
  `);
}
</script>
</div>

<script>
function press(id,btn){
  btn.className="btn on";
  btn.innerText="ON";
  fetch("/on?id="+id);
}
function release(id,btn){
  btn.className="btn off";
  btn.innerText="HOLD";
  fetch("/off?id="+id);
}
</script>

</body>
</html>
)rawliteral";

// -------- HANDLERS --------
void handleRoot(){
  server.send(200, "text/html", dashboard_html);
}

void handleOn(){
  int id = server.arg("id").toInt();
  if(id >= 1 && id <= RELAY_COUNT){
    digitalWrite(relayPins[id-1], HIGH);
  }
  server.send(200, "text/plain", "ON");
}

void handleOff(){
  int id = server.arg("id").toInt();
  if(id >= 1 && id <= RELAY_COUNT){
    digitalWrite(relayPins[id-1], LOW);
  }
  server.send(200, "text/plain", "OFF");
}

// -------- SETUP --------
void setup(){
  Serial.begin(115200);

  for(int i=0;i<RELAY_COUNT;i++){
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }

  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  MDNS.begin("esp32");

  server.on("/", handleRoot);
  server.on("/on", handleOn);
  server.on("/off", handleOff);

  server.begin();
  Serial.println("Web server started");
}

// -------- LOOP --------
void loop(){
  server.handleClient();
}
