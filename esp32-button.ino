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
<title>ESP32 Touch Control</title>

<style>
:root{
  --bg:#0f2027;
  --card:#ffffff18;
  --off:#ff4d4d;
  --on:#00e676;
}

*{box-sizing:border-box}

body{
  margin:0;
  font-family:Segoe UI,Arial;
  background:linear-gradient(135deg,#0f2027,#203a43,#2c5364);
  color:#fff;
}

header{
  text-align:center;
  padding:20px;
  font-size:24px;
  font-weight:600;
  letter-spacing:1px;
  background:rgba(0,0,0,0.35);
  backdrop-filter:blur(6px);
}

.container{
  padding:22px;
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(140px,1fr));
  gap:22px;
}

.card{
  background:var(--card);
  border-radius:18px;
  padding:18px;
  text-align:center;
  box-shadow:0 12px 30px rgba(0,0,0,.35);
  backdrop-filter:blur(8px);
  transition:transform .3s;
}
.card:hover{transform:translateY(-4px)}

.card h4{
  margin:8px 0 14px;
  font-weight:500;
}

.btn{
  width:100%;
  height:70px;
  border:none;
  border-radius:16px;
  font-size:18px;
  font-weight:700;
  cursor:pointer;
  user-select:none;
  transition:all .15s ease;
  box-shadow:
    inset 0 -6px 10px rgba(0,0,0,.3),
    0 6px 18px rgba(0,0,0,.45);
}

.btn.off{
  background:linear-gradient(145deg,#ff6b6b,#ff3b3b);
  color:#000;
}

.btn.on{
  background:linear-gradient(145deg,#00ff87,#00c853);
  color:#000;
  box-shadow:
    0 0 25px rgba(0,255,135,.9),
    inset 0 -4px 8px rgba(0,0,0,.35);
  transform:scale(.96);
}

.btn:active{
  transform:scale(.92);
}
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
  btn.classList.remove("off");
  btn.classList.add("on");
  btn.innerText="ON";
  fetch("/on?id="+id);
}

function release(id,btn){
  btn.classList.remove("on");
  btn.classList.add("off");
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
  Serial.println(WiFi.softAPIP());

  MDNS.begin("esp32");

  server.on("/", handleRoot);
  server.on("/on", handleOn);
  server.on("/off", handleOff);

  server.begin();
}

// -------- LOOP --------
void loop(){
  server.handleClient();
}
