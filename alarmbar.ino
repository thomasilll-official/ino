#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp32-hal-cpu.h"

// =====================================================
// ESP32-C3 A = "leight" / RECEIVER
//
// Functions:
// 1. Receives button status from ESP32-C3 B using ESP-NOW
// 2. Controls A LEDs from B buttons
// 3. Reads 801S vibration sensor
// 4. Vibration turns LED2 ON for 5 seconds
//
// A MAC:
// 14:63:93:C8:F1:10
//
// B MAC:
// 90:70:69:C2:CF:6C
// =====================================================


// =====================================================
// BUTTONS
// =====================================================

#define BTN1 0
#define BTN2 1
#define BTN3 3
#define BTN4 4


// =====================================================
// LEDs
// =====================================================

#define LED1 5
#define LED2 8
#define LED3 7
#define LED4 9


// =====================================================
// 801S VIBRATION SENSOR
// =====================================================

// 801S OUT connected to GPIO 4
#define VIBRATION_PIN 4

// Vibration LED
// GPIO 8 = LED2
#define VIBRATION_LED LED2


// =====================================================
// ESP-NOW CHANNEL
// =====================================================

#define ESPNOW_CHANNEL 1


// =====================================================
// B = "remote" MAC
// =====================================================

uint8_t PEER_MAC[] = {
  0x90,
  0x70,
  0x69,
  0xC2,
  0xCF,
  0x6C
};


// =====================================================
// BUTTON DATA
// =====================================================

typedef struct {

  bool btn1;
  bool btn2;
  bool btn3;
  bool btn4;

} ButtonData;


ButtonData localData = {
  false,
  false,
  false,
  false
};


// =====================================================
// LAST STABLE BUTTON STATES
// =====================================================

bool stableBtn1 = false;
bool stableBtn2 = false;
bool stableBtn3 = false;
bool stableBtn4 = false;


// =====================================================
// PREVIOUS RAW STATES
// =====================================================

bool lastRawBtn1 = false;
bool lastRawBtn2 = false;
bool lastRawBtn3 = false;
bool lastRawBtn4 = false;


// =====================================================
// DEBOUNCE
// =====================================================

unsigned long debounceTime1 = 0;
unsigned long debounceTime2 = 0;
unsigned long debounceTime3 = 0;
unsigned long debounceTime4 = 0;

const unsigned long DEBOUNCE_MS = 30;


// =====================================================
// VIBRATION CONTROL
// =====================================================

bool vibrationActive = false;

unsigned long vibrationStartTime = 0;

const unsigned long VIBRATION_LED_TIME = 5000;


// =====================================================
// LED CONTROL
//
// Your LEDs are ACTIVE-LOW:
//
// LOW  = ON
// HIGH = OFF
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

  // Check packet size
  if (len != sizeof(ButtonData)) {
    return;
  }


  ButtonData remoteData;


  memcpy(
    &remoteData,
    incomingData,
    sizeof(remoteData)
  );


  // ===================================================
  // REMOTE BUTTONS CONTROL A LEDs
  // ===================================================

  setLED(LED1, remoteData.btn1);

  // LED2 is also used by vibration.
  // Only turn it OFF from remote if vibration is NOT active.
  if (!vibrationActive) {
    setLED(LED2, remoteData.btn2);
  }

  setLED(LED3, remoteData.btn3);
  setLED(LED4, remoteData.btn4);


  // Debug
  Serial.print("Remote: ");

  Serial.print(remoteData.btn1);
  Serial.print(" ");

  Serial.print(remoteData.btn2);
  Serial.print(" ");

  Serial.print(remoteData.btn3);
  Serial.print(" ");

  Serial.println(remoteData.btn4);

}


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);


  // ===================================================
  // CPU
  // ===================================================

  setCpuFrequencyMhz(80);


  // ===================================================
  // BUTTONS
  // ===================================================

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(BTN4, INPUT_PULLUP);


  // ===================================================
  // LEDs
  // ===================================================

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);


  // ===================================================
  // 801S VIBRATION SENSOR
  // ===================================================

  pinMode(VIBRATION_PIN, INPUT);


  // ===================================================
  // LEDs OFF
  // ===================================================

  setLED(LED1, false);
  setLED(LED2, false);
  setLED(LED3, false);
  setLED(LED4, false);


  // ===================================================
  // WIFI
  // ===================================================

  WiFi.mode(WIFI_STA);


  // Fixed ESP-NOW channel

  esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );


  // WiFi power saving

  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);


  // ===================================================
  // SERIAL INFORMATION
  // ===================================================

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32-C3 A - RECEIVER");
  Serial.println("================================");

  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println("801S Vibration Sensor: GPIO 4");
  Serial.println("Vibration LED: GPIO 8");

  Serial.println("================================");


  // ===================================================
  // ESP-NOW INIT
  // ===================================================

  if (esp_now_init() != ESP_OK) {

    Serial.println("ESP-NOW INIT FAILED");

    return;
  }


  // Register receive callback

  esp_now_register_recv_cb(OnDataRecv);


  // ===================================================
  // ADD B AS PEER
  // ===================================================

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


  // ===================================================
  // INITIAL BUTTON STATE
  // ===================================================

  sendButtonData();


  Serial.println("READY");
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  unsigned long now = millis();


  // ===================================================
  // 1. READ 801S VIBRATION SENSOR
  // ===================================================

  if (digitalRead(VIBRATION_PIN) == HIGH) {

    // Start vibration timer only once

    if (!vibrationActive) {

      vibrationActive = true;

      vibrationStartTime = now;


      Serial.println();
      Serial.println("******************************");
      Serial.println("VIBRATION DETECTED!");
      Serial.println("LED2 ON FOR 5 SECONDS");
      Serial.println("******************************");


      // LED ON

      setLED(VIBRATION_LED, true);
    }
  }


  // ===================================================
  // 2. VIBRATION TIMER
  // ===================================================

  if (vibrationActive) {

    if (now - vibrationStartTime >= VIBRATION_LED_TIME) {

      vibrationActive = false;


      // LED OFF

      // IMPORTANT:
      // After vibration ends, LED2 returns to OFF.
      // If you want remote button 2 state restored,
      // the next remote packet will restore it.

      setLED(VIBRATION_LED, false);


      Serial.println("Vibration 5 seconds completed");

    }
  }


  // ===================================================
  // 3. READ REMOTE-STYLE LOCAL BUTTONS
  // ===================================================

  bool raw1 = !digitalRead(BTN1);
  bool raw2 = !digitalRead(BTN2);
  bool raw3 = !digitalRead(BTN3);
  bool raw4 = !digitalRead(BTN4);


  bool changed = false;


  // ===================================================
  // BUTTON 1
  // ===================================================

  if (raw1 != lastRawBtn1) {

    lastRawBtn1 = raw1;

    debounceTime1 = now;
  }


  if (
    (now - debounceTime1 >= DEBOUNCE_MS) &&
    (stableBtn1 != raw1)
  ) {

    stableBtn1 = raw1;

    localData.btn1 = stableBtn1;

    changed = true;
  }


  // ===================================================
  // BUTTON 2
  // ===================================================

  if (raw2 != lastRawBtn2) {

    lastRawBtn2 = raw2;

    debounceTime2 = now;
  }


  if (
    (now - debounceTime2 >= DEBOUNCE_MS) &&
    (stableBtn2 != raw2)
  ) {

    stableBtn2 = raw2;

    localData.btn2 = stableBtn2;

    changed = true;
  }


  // ===================================================
  // BUTTON 3
  // ===================================================

  if (raw3 != lastRawBtn3) {

    lastRawBtn3 = raw3;

    debounceTime3 = now;
  }


  if (
    (now - debounceTime3 >= DEBOUNCE_MS) &&
    (stableBtn3 != raw3)
  ) {

    stableBtn3 = raw3;

    localData.btn3 = stableBtn3;

    changed = true;
  }


  // ===================================================
  // BUTTON 4
  // ===================================================

  if (raw4 != lastRawBtn4) {

    lastRawBtn4 = raw4;

    debounceTime4 = now;
  }


  if (
    (now - debounceTime4 >= DEBOUNCE_MS) &&
    (stableBtn4 != raw4)
  ) {

    stableBtn4 = raw4;

    localData.btn4 = stableBtn4;

    changed = true;
  }


  // ===================================================
  // SEND ONLY WHEN BUTTON CHANGES
  // ===================================================

  if (changed) {

    sendButtonData();
  }


  // Small delay

  delay(5);
}
