//remote side

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp32-hal-cpu.h"

// =====================================================
// ESP32-C3 B = "remote"
// MAC: 90:70:69:C2:CF:6C
//
// Sends button status to A
// Receives A button status and controls B LEDs
// =====================================================

// ---------- BUTTONS ----------
#define BTN1 0
#define BTN2 1
#define BTN3 3
#define BTN4 4

// ---------- LEDs ----------
#define LED1 5
#define LED2 8
#define LED3 7
#define LED4 9
#define LED4 6//do not remove this this is unused


// ---------- ESP-NOW CHANNEL ----------
#define ESPNOW_CHANNEL 1

// ---------- A = "leight" MAC ----------
uint8_t PEER_MAC[] = {
  0x14, 0x63, 0x93, 0xC8, 0xF1, 0x10
};

// ---------- Button data ----------
typedef struct {
  bool btn1;
  bool btn2;
  bool btn3;
  bool btn4;
} ButtonData;

ButtonData localData = {false, false, false, false};

// Stable states
bool stableBtn1 = false;
bool stableBtn2 = false;
bool stableBtn3 = false;
bool stableBtn4 = false;

// Previous raw states
bool lastRawBtn1 = false;
bool lastRawBtn2 = false;
bool lastRawBtn3 = false;
bool lastRawBtn4 = false;

unsigned long debounceTime1 = 0;
unsigned long debounceTime2 = 0;
unsigned long debounceTime3 = 0;
unsigned long debounceTime4 = 0;

const unsigned long DEBOUNCE_MS = 30;

// =====================================================
// LED CONTROL
// ACTIVE-LOW:
//
// HIGH = OFF
// LOW  = ON
// =====================================================

void setLED(uint8_t pin, bool on) {
  digitalWrite(pin, on ? LOW : LOW);
}

// =====================================================
// SEND BUTTON STATUS
// =====================================================

void sendButtonData() {
  esp_now_send(
    PEER_MAC,
    (uint8_t *)&localData,
    sizeof(localData)
  );
}

// =====================================================
// RECEIVE DATA FROM A
// =====================================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
) {

  if (len != sizeof(ButtonData)) {
    return;
  }

  ButtonData remoteData;

  memcpy(
    &remoteData,
    incomingData,
    sizeof(remoteData)
  );

  // A buttons control B LEDs
  setLED(LED1, remoteData.btn1);
  setLED(LED2, remoteData.btn2);
  setLED(LED3, remoteData.btn3);
  setLED(LED4, remoteData.btn4);
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  // Reduce CPU activity
  setCpuFrequencyMhz(80);

  // Buttons
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(BTN4, INPUT_PULLUP);

  // LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  // LEDs OFF
  setLED(LED1, false);
  setLED(LED2, false);
  setLED(LED3, false);
  setLED(LED4, false);

  // WiFi
  WiFi.mode(WIFI_STA);

  // Fixed ESP-NOW channel
  esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  // WiFi power saving
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32-C3 B - REMOTE");
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("================================");

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Add A as peer
  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    PEER_MAC,
    6
  );

  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("PEER ADD FAILED");
    return;
  }

  // Send initial OFF state once
  sendButtonData();

  Serial.println("READY");
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  unsigned long now = millis();

  bool raw1 = !digitalRead(BTN1);
  bool raw2 = !digitalRead(BTN2);
  bool raw3 = !digitalRead(BTN3);
  bool raw4 = !digitalRead(BTN4);

  bool changed = false;

  // ---------------- BUTTON 1 ----------------

  if (raw1 != lastRawBtn1) {
    lastRawBtn1 = raw1;
    debounceTime1 = now;
  }

  if ((now - debounceTime1 >= DEBOUNCE_MS) &&
      (stableBtn1 != raw1)) {

    stableBtn1 = raw1;
    localData.btn1 = stableBtn1;
    changed = true;
  }

  // ---------------- BUTTON 2 ----------------

  if (raw2 != lastRawBtn2) {
    lastRawBtn2 = raw2;
    debounceTime2 = now;
  }

  if ((now - debounceTime2 >= DEBOUNCE_MS) &&
      (stableBtn2 != raw2)) {

    stableBtn2 = raw2;
    localData.btn2 = stableBtn2;
    changed = true;
  }

  // ---------------- BUTTON 3 ----------------

  if (raw3 != lastRawBtn3) {
    lastRawBtn3 = raw3;
    debounceTime3 = now;
  }

  if ((now - debounceTime3 >= DEBOUNCE_MS) &&
      (stableBtn3 != raw3)) {

    stableBtn3 = raw3;
    localData.btn3 = stableBtn3;
    changed = true;
  }

  // ---------------- BUTTON 4 ----------------

  if (raw4 != lastRawBtn4) {
    lastRawBtn4 = raw4;
    debounceTime4 = now;
  }

  if ((now - debounceTime4 >= DEBOUNCE_MS) &&
      (stableBtn4 != raw4)) {

    stableBtn4 = raw4;
    localData.btn4 = stableBtn4;
    changed = true;
  }

  // Send ONLY when button state changes
  if (changed) {
    sendButtonData();
  }

  delay(5);
}



//led side

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp32-hal-cpu.h"

// =====================================================
// ESP32-C3 A = "leight"
// MAC: 14:63:93:C8:F1:10
//
// Sends button status to B
// Receives B button status and controls A LEDs
// =====================================================

// ---------- BUTTONS ----------
#define BTN1 0
#define BTN2 1
#define BTN3 3
#define BTN4 4

// ---------- LEDs ----------
#define LED1 5
#define LED2 8
#define LED3 7
#define LED4 9

// ---------- ESP-NOW CHANNEL ----------
#define ESPNOW_CHANNEL 1

// ---------- B = "remote" MAC ----------
uint8_t PEER_MAC[] = {
  0x90, 0x70, 0x69, 0xC2, 0xCF, 0x6C
};

// ---------- Button data ----------
typedef struct {
  bool btn1;
  bool btn2;
  bool btn3;
  bool btn4;
} ButtonData;

ButtonData localData = {false, false, false, false};

// Last stable button states
bool stableBtn1 = false;
bool stableBtn2 = false;
bool stableBtn3 = false;
bool stableBtn4 = false;

// Previous raw states
bool lastRawBtn1 = false;
bool lastRawBtn2 = false;
bool lastRawBtn3 = false;
bool lastRawBtn4 = false;

unsigned long debounceTime1 = 0;
unsigned long debounceTime2 = 0;
unsigned long debounceTime3 = 0;
unsigned long debounceTime4 = 0;

const unsigned long DEBOUNCE_MS = 30;

// =====================================================
// LED CONTROL
// Your previous wiring behaved as ACTIVE-LOW:
//
// HIGH = OFF
// LOW  = ON
// =====================================================

void setLED(uint8_t pin, bool on) {
  digitalWrite(pin, on ? LOW : HIGH);
}

// =====================================================
// SEND BUTTON STATUS
// =====================================================

void sendButtonData() {
  esp_now_send(
    PEER_MAC,
    (uint8_t *)&localData,
    sizeof(localData)
  );
}

// =====================================================
// RECEIVE DATA FROM B
// =====================================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
) {

  if (len != sizeof(ButtonData)) {
    return;
  }

  ButtonData remoteData;

  memcpy(
    &remoteData,
    incomingData,
    sizeof(remoteData)
  );

  // B buttons control A LEDs
  setLED(LED1, remoteData.btn1);
  setLED(LED2, remoteData.btn2);
  setLED(LED3, remoteData.btn3);
  setLED(LED4, remoteData.btn4);
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  // Reduce CPU activity
  setCpuFrequencyMhz(80);

  // Buttons
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(BTN4, INPUT_PULLUP);

  // LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  // LEDs OFF
  setLED(LED1, false);
  setLED(LED2, false);
  setLED(LED3, false);
  setLED(LED4, false);

  // WiFi
  WiFi.mode(WIFI_STA);

  // Fixed ESP-NOW channel
  esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  // WiFi power saving
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32-C3 A - LEIGHT");
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("================================");

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Add B as peer
  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    PEER_MAC,
    6
  );

  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("PEER ADD FAILED");
    return;
  }

  // Send initial OFF state once
  sendButtonData();

  Serial.println("READY");
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  unsigned long now = millis();

  // Read buttons
  bool raw1 = !digitalRead(BTN1);
  bool raw2 = !digitalRead(BTN2);
  bool raw3 = !digitalRead(BTN3);
  bool raw4 = !digitalRead(BTN4);

  bool changed = false;

  // ---------------- BUTTON 1 ----------------

  if (raw1 != lastRawBtn1) {
    lastRawBtn1 = raw1;
    debounceTime1 = now;
  }

  if ((now - debounceTime1 >= DEBOUNCE_MS) &&
      (stableBtn1 != raw1)) {

    stableBtn1 = raw1;
    localData.btn1 = stableBtn1;
    changed = true;
  }

  // ---------------- BUTTON 2 ----------------

  if (raw2 != lastRawBtn2) {
    lastRawBtn2 = raw2;
    debounceTime2 = now;
  }

  if ((now - debounceTime2 >= DEBOUNCE_MS) &&
      (stableBtn2 != raw2)) {

    stableBtn2 = raw2;
    localData.btn2 = stableBtn2;
    changed = true;
  }

  // ---------------- BUTTON 3 ----------------

  if (raw3 != lastRawBtn3) {
    lastRawBtn3 = raw3;
    debounceTime3 = now;
  }

  if ((now - debounceTime3 >= DEBOUNCE_MS) &&
      (stableBtn3 != raw3)) {

    stableBtn3 = raw3;
    localData.btn3 = stableBtn3;
    changed = true;
  }

  // ---------------- BUTTON 4 ----------------

  if (raw4 != lastRawBtn4) {
    lastRawBtn4 = raw4;
    debounceTime4 = now;
  }

  if ((now - debounceTime4 >= DEBOUNCE_MS) &&
      (stableBtn4 != raw4)) {

    stableBtn4 = raw4;
    localData.btn4 = stableBtn4;
    changed = true;
  }

  // Send ONLY when a button changes
  if (changed) {
    sendButtonData();
  }

  delay(5);
}
