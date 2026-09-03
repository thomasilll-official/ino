//=============================================================================
// INDUSTRIAL CONTROL SYSTEM v4.0
// With Power Management & Heat Reduction
//=============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <esp_pm.h>

//=============================================================================
// POWER MANAGEMENT CONFIGURATION
//=============================================================================

#define ENABLE_POWER_SAVE 1
#define LIGHT_SLEEP_TIMEOUT_MS 3000
#define CPU_FREQ_MHZ 80
#define MIN_TX_POWER 40  // 4dBm

//=============================================================================
// GPIO DEFINITIONS
//=============================================================================

// Button GPIOs
#define B1 4
#define B2 5
#define B3 6
#define B4 7

// LED GPIOs
#define L1 8
#define L2 9
#define L3 10
#define L4 11
#define L5 12

// Timer DIP Switch GPIOs (8-bit)
#define SW1 13  // 1
#define SW2 14  // 2
#define SW3 15  // 4
#define SW4 16  // 8
#define SW5 17  // 16
#define SW6 18  // 32
#define SW7 21  // 64
#define SW8 38  // 128

// System Switching DIP Switch (4-bit)
#define D1 39  // 1
#define D2 40  // 2
#define D3 41  // 4
#define D4 42  // 8

//=============================================================================
// ESP-NOW CONFIGURATION
//=============================================================================

#define ESPNOW_CHANNEL 1
#define ROLE_MASTER 1
#define ROLE_SLEW 2

// Message Types
#define MSG_HELLO       1
#define MSG_HELLO_ACK   2
#define MSG_HEARTBEAT   3
#define MSG_BTN1_STATE  4
#define MSG_RESET       5

// MAC Addresses
uint8_t SLEW_MAC[] = {0x08, 0x3A, 0xF2, 0x09, 0xBE, 0xF8};
uint8_t MASTER_MAC[] = {0x94, 0xA9, 0x90, 0xEF, 0x98, 0x28};

//=============================================================================
// DATA STRUCTURES
//=============================================================================

typedef struct {
  uint8_t type;
  uint8_t role;
  uint8_t buttonState;
  uint32_t counter;
} DataPacket;

typedef enum {
    SYSTEM_DEFAULT = 0,
    SYSTEM_ONE = 1,
    SYSTEM_TWO = 2,
    SYSTEM_THREE = 3,
    SYSTEM_FOUR = 4,
    SYSTEM_FIVE = 5,
    SYSTEM_SIX = 6,
    SYSTEM_SEVEN = 7,
    SYSTEM_EIGHT = 8
} system_mode_t;

typedef enum {
    SEQ_IDLE = 0,
    SEQ_L1_L5 = 1,
    SEQ_L2_L5 = 2,
    SEQ_L3_L5 = 3,
    SEQ_L4_L5 = 4
} sequence_state_t;

typedef enum {
    MASTER_SEARCHING = 0,
    MASTER_CONNECT_BLINK = 1,
    MASTER_RESET_READY = 2,
    MASTER_BTN_PRESSED = 3,
    MASTER_BTN_RELEASED = 4,
    MASTER_DISCONNECTED = 5
} master_state_t;

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// System state
static system_mode_t current_system = SYSTEM_DEFAULT;
static system_mode_t last_system = SYSTEM_DEFAULT;

// Timer management
static unsigned long timer_start = 0;
static unsigned long timer_duration = 0;
static bool timer_active = false;
static int active_button = 0;
static int timer_value = 1;

// Button debouncing
static bool last_b1_state = true;
static bool last_b2_state = true;
static bool last_b3_state = true;
static bool last_b4_state = true;
static unsigned long last_debounce_time = 0;
static const int debounce_delay_ms = 50;

// Default system
static bool default_blink_state = false;
static unsigned long default_blink_timer = 0;
static const int default_blink_interval_ms = 1000;

// System 4 (Auto-running sequential)
static bool system4_active = false;
static sequence_state_t seq_state = SEQ_IDLE;
static unsigned long seq_step_start = 0;
static unsigned long seq_step_duration = 0;
static int current_led_index = 0;

// System 7 (2 AND manner)
static bool system7_active = false;
static int system7_combination = 0;

//=============================================================================
// SYSTEM 8 ESP-NOW VARIABLES
//=============================================================================

static bool espNowInitialized = false;
static unsigned long lastPacket = 0;
static unsigned long lastHello = 0;
static unsigned long lastHeartbeat = 0;

// Timings
static const unsigned long HELLO_INTERVAL = 500;
static const unsigned long HEARTBEAT_INTERVAL = 500;
static const unsigned long CONNECTION_TIMEOUT = 2500;
static const unsigned long SEARCH_BLINK_TIME = 100;
static const unsigned long CONNECT_BLINK_TIME = 200;
static const unsigned long SEQUENCE_TIME = 300;
static const int CONNECT_BLINK_COUNT = 3;
static const unsigned long DEBOUNCE_TIME = 40;

// State variables
static master_state_t masterState = MASTER_SEARCHING;
static bool slewButtonPressed = false;

// Searching state
static unsigned long lastSearchBlink = 0;
static bool searchLED1 = true;

// Connection blink state
static unsigned long lastConnectBlink = 0;
static int connectBlinkCount = 0;
static bool connectLEDState = false;

// Disconnected state
static unsigned long lastDisconnectBlink = 0;
static bool disconnectLED1 = true;

// Release sequence state
static unsigned long lastSequenceChange = 0;
static bool sequenceLED1 = true;

// Master button
static bool lastBtn1 = HIGH;
static bool stableBtn1 = HIGH;
static unsigned long btn1Debounce = 0;

//=============================================================================
// PERFORMANCE MONITORING
//=============================================================================

static unsigned long loopCount = 0;
static unsigned long lastStatsPrint = 0;
static unsigned long lastLoopTime = 0;
static unsigned long lastActivityTime = 0;

//=============================================================================
// FUNCTION PROTOTYPES
//=============================================================================

void turnOffAllLEDs(void);
void turnOnAllLEDs(void);
void setLEDs(uint8_t l1, uint8_t l2, uint8_t l3, uint8_t l4, uint8_t l5);
void readTimerDIPSwitch(void);
void readSystemDIPSwitch(void);
bool debounce(void);
void configurePowerManagement(void);
void systemDefaultTask(void);
void systemOneTask(int button);
void systemTwoTask(int button);
void systemThreeTask(int button);
void systemFourStart(void);
void systemFourTask(void);
void systemFiveTask(int button);
void systemSixTask(void);
void systemSevenTask(void);
void systemEightTask(void);
void systemEightReset(void);
void initESPNow(void);
void sendToSlew(uint8_t type, uint8_t buttonState = 0);
void onDataReceiveSystem8(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void resetSystem(void);
bool checkCombinationPress(bool b1, bool b2, bool b3, bool b4);
void setSystem8LEDs(bool led1, bool led2, bool led3, bool led4, bool led5);
void enterResetReadyMode(void);
void startConnectionBlink(void);
void connectionBlinkIndication(void);
void searchingIndication(void);
void disconnectIndication(void);
void startDisconnectMode(void);
void sequenceIndication(void);
void checkMasterButton1(void);
void enterLightSleep(void);

//=============================================================================
// POWER MANAGEMENT
//=============================================================================

void configurePowerManagement() {
  #if ENABLE_POWER_SAVE
    esp_pm_config_t pm_config = {
      .max_freq_mhz = CPU_FREQ_MHZ,
      .min_freq_mhz = CPU_FREQ_MHZ,
      .light_sleep_enable = true
    };
    esp_pm_configure(&pm_config);
    esp_wifi_set_max_tx_power(MIN_TX_POWER);
    esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_TIMEOUT_MS * 1000);
    
    Serial.println("=== POWER MANAGEMENT ===");
    Serial.printf("CPU: %d MHz\n", CPU_FREQ_MHZ);
    Serial.printf("TX Power: %d dBm\n", 4);
    Serial.printf("Sleep Timeout: %d ms\n", LIGHT_SLEEP_TIMEOUT_MS);
  #endif
}

void enterLightSleep() {
  #if ENABLE_POWER_SAVE
    if (millis() - lastActivityTime > LIGHT_SLEEP_TIMEOUT_MS) {
      delay(2);
    }
  #endif
}

//=============================================================================
// LED CONTROL FUNCTIONS
//=============================================================================

inline void turnOffAllLEDs() {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    digitalWrite(L3, LOW);
    digitalWrite(L4, LOW);
    digitalWrite(L5, LOW);
}

inline void turnOnAllLEDs() {
    digitalWrite(L1, HIGH);
    digitalWrite(L2, HIGH);
    digitalWrite(L3, HIGH);
    digitalWrite(L4, HIGH);
    digitalWrite(L5, HIGH);
}

inline void setLEDs(uint8_t l1, uint8_t l2, uint8_t l3, uint8_t l4, uint8_t l5) {
    digitalWrite(L1, l1);
    digitalWrite(L2, l2);
    digitalWrite(L3, l3);
    digitalWrite(L4, l4);
    digitalWrite(L5, l5);
}

inline void setSystem8LEDs(bool led1, bool led2, bool led3, bool led4, bool led5) {
    digitalWrite(L1, led1 ? HIGH : LOW);
    digitalWrite(L2, led2 ? HIGH : LOW);
    digitalWrite(L3, led3 ? HIGH : LOW);
    digitalWrite(L4, led4 ? HIGH : LOW);
    digitalWrite(L5, led5 ? HIGH : LOW);
}

//=============================================================================
// SYSTEM 8 MASTER STATE FUNCTIONS
//=============================================================================

void enterResetReadyMode() {
    masterState = MASTER_RESET_READY;
    slewButtonPressed = false;
    sequenceLED1 = true;
    setSystem8LEDs(false, true, false, false, false);
    lastActivityTime = millis();
    
    Serial.println("\n================================");
    Serial.println("SYSTEM 8: RESET/READY MODE");
    Serial.println("LED2 = ON | All others OFF");
    Serial.println("================================");
}

void startConnectionBlink() {
    masterState = MASTER_CONNECT_BLINK;
    connectBlinkCount = 0;
    connectLEDState = false;
    lastConnectBlink = millis();
    setSystem8LEDs(false, false, false, false, false);
    lastActivityTime = millis();
    
    Serial.println("\n================================");
    Serial.println("SYSTEM 8: SLEW CONNECTED");
    Serial.println("3 BLINK INDICATION");
    Serial.println("================================");
}

inline void connectionBlinkIndication() {
    if (masterState != MASTER_CONNECT_BLINK) return;
    
    if (millis() - lastConnectBlink >= CONNECT_BLINK_TIME) {
        lastConnectBlink = millis();
        connectLEDState = !connectLEDState;
        
        if (connectLEDState) {
            setSystem8LEDs(true, true, false, false, false);
        } else {
            setSystem8LEDs(false, false, false, false, false);
            connectBlinkCount++;
            
            if (connectBlinkCount >= CONNECT_BLINK_COUNT) {
                enterResetReadyMode();
            }
        }
    }
}

inline void searchingIndication() {
    if (masterState != MASTER_SEARCHING) return;
    
    if (millis() - lastSearchBlink >= SEARCH_BLINK_TIME) {
        lastSearchBlink = millis();
        searchLED1 = !searchLED1;
        setSystem8LEDs(searchLED1, !searchLED1, false, false, false);
    }
}

void startDisconnectMode() {
    masterState = MASTER_DISCONNECTED;
    slewButtonPressed = false;
    disconnectLED1 = true;
    lastDisconnectBlink = millis();
    setSystem8LEDs(true, false, false, false, false);
    lastActivityTime = millis();
    
    Serial.println("\n================================");
    Serial.println("SYSTEM 8: SLEW DISCONNECTED");
    Serial.println("LED1 <-> LED2 ALTERNATING");
    Serial.println("================================");
}

inline void disconnectIndication() {
    if (masterState != MASTER_DISCONNECTED) return;
    
    if (millis() - lastDisconnectBlink >= SEARCH_BLINK_TIME) {
        lastDisconnectBlink = millis();
        disconnectLED1 = !disconnectLED1;
        setSystem8LEDs(disconnectLED1, !disconnectLED1, false, false, false);
    }
}

inline void sequenceIndication() {
    if (masterState != MASTER_BTN_RELEASED) return;
    
    if (millis() - lastSequenceChange >= SEQUENCE_TIME) {
        lastSequenceChange = millis();
        sequenceLED1 = !sequenceLED1;
        setSystem8LEDs(sequenceLED1, !sequenceLED1, true, false, false);
    }
}

void checkMasterButton1() {
    if (current_system != SYSTEM_EIGHT || masterState == MASTER_RESET_READY) return;
    
    bool reading = digitalRead(B1);
    
    if (reading != lastBtn1) {
        btn1Debounce = millis();
    }
    
    if (millis() - btn1Debounce > DEBOUNCE_TIME) {
        if (reading != stableBtn1) {
            stableBtn1 = reading;
            
            if (stableBtn1 == LOW) {
                Serial.println("SYSTEM 8: MASTER RESET REQUEST");
                sendToSlew(MSG_RESET, 0);
                enterResetReadyMode();
                lastActivityTime = millis();
            }
        }
    }
    
    lastBtn1 = reading;
}

//=============================================================================
// SYSTEM 8 ESP-NOW FUNCTIONS
//=============================================================================

void initESPNow() {
    if (espNowInitialized) return;
    
    Serial.println("SYSTEM 8: Initializing ESP-NOW...");
    
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("SYSTEM 8: ESP-NOW INIT FAILED!");
        espNowInitialized = false;
        return;
    }
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, SLEW_MAC, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("SYSTEM 8: FAILED TO ADD SLEW PEER");
    } else {
        Serial.println("SYSTEM 8: SLEW PEER ADDED");
    }
    
    esp_now_register_recv_cb(onDataReceiveSystem8);
    espNowInitialized = true;
    masterState = MASTER_SEARCHING;
    
    Serial.println("SYSTEM 8: ESP-NOW READY - SEARCHING FOR SLEW");
}

void sendToSlew(uint8_t type, uint8_t buttonState) {
    if (!espNowInitialized) return;
    
    DataPacket packet;
    packet.type = type;
    packet.role = ROLE_MASTER;
    packet.buttonState = buttonState;
    packet.counter = millis();
    
    esp_err_t result = esp_now_send(SLEW_MAC, (uint8_t *)&packet, sizeof(packet));
    
    static uint32_t lastErrorLog = 0;
    if (result != ESP_OK && (millis() - lastErrorLog > 5000)) {
        lastErrorLog = millis();
        Serial.printf("ESP-NOW Send Error: %d\n", result);
    }
}

void onDataReceiveSystem8(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len != sizeof(DataPacket)) return;
    if (memcmp(info->src_addr, SLEW_MAC, 6) != 0) return;
    
    DataPacket packet;
    memcpy(&packet, data, sizeof(packet));
    lastPacket = millis();
    lastActivityTime = millis();
    
    switch(packet.type) {
        case MSG_HELLO:
            if (masterState == MASTER_SEARCHING || masterState == MASTER_DISCONNECTED) {
                startConnectionBlink();
            }
            sendToSlew(MSG_HELLO_ACK, 0);
            break;
            
        case MSG_HELLO_ACK:
            if (masterState == MASTER_SEARCHING || masterState == MASTER_DISCONNECTED) {
                startConnectionBlink();
            }
            break;
            
        case MSG_HEARTBEAT:
            if (masterState == MASTER_SEARCHING || masterState == MASTER_DISCONNECTED) {
                startConnectionBlink();
            }
            lastPacket = millis();
            break;
            
        case MSG_BTN1_STATE:
            if (packet.buttonState == 1) {
                Serial.println("SYSTEM 8: SLEW BTN1 PRESSED");
                masterState = MASTER_BTN_PRESSED;
                slewButtonPressed = true;
                setSystem8LEDs(true, false, false, true, false);
            } else {
                Serial.println("SYSTEM 8: SLEW BTN1 RELEASED");
                masterState = MASTER_BTN_RELEASED;
                slewButtonPressed = false;
                sequenceLED1 = true;
                lastSequenceChange = millis();
                setSystem8LEDs(true, false, true, false, false);
            }
            break;
            
        case MSG_RESET:
            Serial.println("SYSTEM 8: RESET FROM SLEW");
            enterResetReadyMode();
            break;
    }
}

void systemEightTask() {
    if (!espNowInitialized) {
        initESPNow();
        return;
    }
    
    switch(masterState) {
        case MASTER_SEARCHING:
            searchingIndication();
            if (millis() - lastHello >= HELLO_INTERVAL) {
                lastHello = millis();
                sendToSlew(MSG_HELLO, 0);
            }
            break;
            
        case MASTER_CONNECT_BLINK:
            connectionBlinkIndication();
            if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
                lastHeartbeat = millis();
                sendToSlew(MSG_HEARTBEAT, 0);
            }
            break;
            
        case MASTER_RESET_READY:
            setSystem8LEDs(false, true, false, false, false);
            if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
                lastHeartbeat = millis();
                sendToSlew(MSG_HEARTBEAT, 0);
            }
            if (millis() - lastPacket > CONNECTION_TIMEOUT) {
                startDisconnectMode();
            }
            break;
            
        case MASTER_BTN_PRESSED:
            setSystem8LEDs(true, false, false, true, false);
            if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
                lastHeartbeat = millis();
                sendToSlew(MSG_HEARTBEAT, 0);
            }
            if (millis() - lastPacket > CONNECTION_TIMEOUT) {
                startDisconnectMode();
            }
            break;
            
        case MASTER_BTN_RELEASED:
            sequenceIndication();
            if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
                lastHeartbeat = millis();
                sendToSlew(MSG_HEARTBEAT, 0);
            }
            if (millis() - lastPacket > CONNECTION_TIMEOUT) {
                startDisconnectMode();
            }
            break;
            
        case MASTER_DISCONNECTED:
            disconnectIndication();
            if (millis() - lastHello >= HELLO_INTERVAL) {
                lastHello = millis();
                sendToSlew(MSG_HELLO, 0);
            }
            break;
    }
}

void systemEightReset() {
    masterState = MASTER_SEARCHING;
    slewButtonPressed = false;
    searchLED1 = true;
    lastSearchBlink = millis();
    setSystem8LEDs(false, false, false, false, false);
    Serial.println("SYSTEM 8: Reset - Searching for Slew");
}

//=============================================================================
// SYSTEM TASK FUNCTIONS
//=============================================================================

void systemDefaultTask() {
    unsigned long current_time = millis();
    if (current_time - default_blink_timer >= default_blink_interval_ms) {
        default_blink_timer = current_time;
        default_blink_state = !default_blink_state;
        
        if (default_blink_state) {
            turnOnAllLEDs();
        } else {
            turnOffAllLEDs();
        }
    }
}

void systemOneTask(int button) {
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = button;
    turnOffAllLEDs();
    lastActivityTime = millis();
    
    switch(button) {
        case 1: setLEDs(0, 0, 1, 0, 1); break;
        case 2: setLEDs(0, 0, 0, 1, 1); break;
        case 3: setLEDs(1, 0, 0, 0, 1); break;
        case 4: setLEDs(0, 1, 0, 0, 1); break;
    }
    Serial.printf("SYSTEM 1: B%d → Timer %ds\n", button, timer_value);
}

void systemTwoTask(int button) {
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = button;
    turnOffAllLEDs();
    lastActivityTime = millis();
    
    switch(button) {
        case 1: setLEDs(0, 1, 1, 1, 1); break;
        case 2: setLEDs(1, 0, 1, 1, 1); break;
        case 3: setLEDs(1, 1, 0, 1, 1); break;
        case 4: setLEDs(1, 1, 1, 0, 1); break;
    }
    Serial.printf("SYSTEM 2: B%d → Timer %ds\n", button, timer_value);
}

void systemThreeTask(int button) {
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = button;
    turnOffAllLEDs();
    lastActivityTime = millis();
    
    switch(button) {
        case 1: setLEDs(1, 0, 0, 0, 1); break;
        case 2: setLEDs(0, 1, 0, 0, 1); break;
        case 3: setLEDs(0, 0, 1, 0, 1); break;
        case 4: setLEDs(0, 0, 0, 1, 1); break;
    }
    Serial.printf("SYSTEM 3: B%d → Timer %ds\n", button, timer_value);
}

void systemFourStart() {
    seq_step_duration = (unsigned long)timer_value * 1000;
    system4_active = true;
    seq_state = SEQ_L1_L5;
    current_led_index = 0;
    seq_step_start = millis();
    turnOffAllLEDs();
    setLEDs(1, 0, 0, 0, 1);
    lastActivityTime = millis();
    
    Serial.printf("SYSTEM 4: Auto-sequence started, %ds per step\n", timer_value);
}

void systemFourTask() {
    if (!system4_active) return;
    
    unsigned long current_time = millis();
    if (current_time - seq_step_start >= seq_step_duration) {
        current_led_index = (current_led_index + 1) % 4;
        seq_step_start = current_time;
        
        switch(current_led_index) {
            case 0: setLEDs(1, 0, 0, 0, 1); break;
            case 1: setLEDs(0, 1, 0, 0, 1); break;
            case 2: setLEDs(0, 0, 1, 0, 1); break;
            case 3: setLEDs(0, 0, 0, 1, 1); break;
        }
        lastActivityTime = millis();
    }
}

void systemFiveTask(int button) {
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = button;
    turnOffAllLEDs();
    turnOnAllLEDs();
    lastActivityTime = millis();
    
    Serial.printf("SYSTEM 5: B%d → ALL LEDs ON, %ds\n", button, timer_value);
}

void systemSixTask() {
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = 5;
    turnOffAllLEDs();
    turnOnAllLEDs();
    lastActivityTime = millis();
    
    Serial.printf("SYSTEM 6: ALL 4 BUTTONS → ALL LEDs ON, %ds\n", timer_value);
}

void systemSevenTask() {
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = 6;
    lastActivityTime = millis();
    
    if (system7_combination == 1) {
        setLEDs(1, 1, 0, 0, 1);
        Serial.printf("SYSTEM 7: B1+B2 → L1,L2,L5, %ds (NO LOCK)\n", timer_value);
    } else if (system7_combination == 2) {
        setLEDs(0, 0, 1, 1, 1);
        Serial.printf("SYSTEM 7: B3+B4 → L3,L4,L5, %ds (NO LOCK)\n", timer_value);
    }
}

//=============================================================================
// DIP SWITCH FUNCTIONS
//=============================================================================

void readTimerDIPSwitch() {
    int value = 0;
    if (digitalRead(SW1) == LOW) value += 1;
    if (digitalRead(SW2) == LOW) value += 2;
    if (digitalRead(SW3) == LOW) value += 4;
    if (digitalRead(SW4) == LOW) value += 8;
    if (digitalRead(SW5) == LOW) value += 16;
    if (digitalRead(SW6) == LOW) value += 32;
    if (digitalRead(SW7) == LOW) value += 64;
    if (digitalRead(SW8) == LOW) value += 128;
    
    if (value == 0) value = 1;
    
    if (value != timer_value) {
        timer_value = value;
        Serial.printf("Timer set to %d seconds\n", timer_value);
    }
}

void readSystemDIPSwitch() {
    int value = 0;
    if (digitalRead(D1) == LOW) value += 1;
    if (digitalRead(D2) == LOW) value += 2;
    if (digitalRead(D3) == LOW) value += 4;
    if (digitalRead(D4) == LOW) value += 8;
    
    system_mode_t new_system = SYSTEM_DEFAULT;
    
    switch(value) {
        case 0: new_system = SYSTEM_DEFAULT; break;
        case 1: new_system = SYSTEM_ONE; break;
        case 2: new_system = SYSTEM_TWO; break;
        case 3: new_system = SYSTEM_THREE; break;
        case 4: new_system = SYSTEM_FOUR; break;
        case 5: new_system = SYSTEM_FIVE; break;
        case 6: new_system = SYSTEM_SIX; break;
        case 7: new_system = SYSTEM_SEVEN; break;
        case 8: new_system = SYSTEM_EIGHT; break;
        default: new_system = SYSTEM_DEFAULT; break;
    }
    
    if (new_system != current_system) {
        current_system = new_system;
        timer_active = false;
        active_button = 0;
        system4_active = false;
        seq_state = SEQ_IDLE;
        system7_active = false;
        system7_combination = 0;
        
        if (current_system == SYSTEM_EIGHT) {
            systemEightReset();
        }
        
        turnOffAllLEDs();
        
        const char* system_names[] = {"DEFAULT", "SYSTEM 1", "SYSTEM 2", "SYSTEM 3", 
                                      "SYSTEM 4", "SYSTEM 5", "SYSTEM 6", "SYSTEM 7", 
                                      "SYSTEM 8 (ESP-NOW)"};
        Serial.printf("System: %s (DIP: %d)\n", system_names[current_system], value);
        
        if (current_system == SYSTEM_FOUR) {
            systemFourStart();
        }
        lastActivityTime = millis();
    }
}

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================

inline bool debounce() {
    unsigned long current_time = millis();
    if (current_time - last_debounce_time > debounce_delay_ms) {
        last_debounce_time = current_time;
        return true;
    }
    return false;
}

inline bool checkCombinationPress(bool b1, bool b2, bool b3, bool b4) {
    return (b1 && b2 && b3 && b4);
}

void resetSystem() {
    turnOffAllLEDs();
    timer_active = false;
    active_button = 0;
    default_blink_state = false;
    default_blink_timer = millis();
    system4_active = false;
    seq_state = SEQ_IDLE;
    system7_active = false;
    system7_combination = 0;
    
    if (espNowInitialized) {
        systemEightReset();
    }
    lastActivityTime = millis();
    Serial.println("System Reset");
}

//=============================================================================
// SETUP
//=============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n==========================================");
    Serial.println("   INDUSTRIAL CONTROL SYSTEM v4.0");
    Serial.println("   Power Optimized - 24/7 Operation");
    Serial.println("==========================================\n");
    
    // Configure pins
    pinMode(B1, INPUT_PULLUP);
    pinMode(B2, INPUT_PULLUP);
    pinMode(B3, INPUT_PULLUP);
    pinMode(B4, INPUT_PULLUP);
    
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(L3, OUTPUT);
    pinMode(L4, OUTPUT);
    pinMode(L5, OUTPUT);
    
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);
    pinMode(SW5, INPUT_PULLUP);
    pinMode(SW6, INPUT_PULLUP);
    pinMode(SW7, INPUT_PULLUP);
    pinMode(SW8, INPUT_PULLUP);
    
    pinMode(D1, INPUT_PULLUP);
    pinMode(D2, INPUT_PULLUP);
    pinMode(D3, INPUT_PULLUP);
    pinMode(D4, INPUT_PULLUP);
    
    // Configure power management
    configurePowerManagement();
    
    // Initial reads
    readSystemDIPSwitch();
    readTimerDIPSwitch();
    
    turnOffAllLEDs();
    default_blink_timer = millis();
    lastActivityTime = millis();
    lastLoopTime = millis();
    
    Serial.println("System Ready - Industrial Mode Active");
    Serial.printf("System: %d | Timer: %d seconds\n", current_system, timer_value);
    
    if (current_system == SYSTEM_FOUR) {
        systemFourStart();
    }
    
    if (current_system == SYSTEM_EIGHT) {
        initESPNow();
    }
}

//=============================================================================
// MAIN LOOP
//=============================================================================

void loop() {
    unsigned long currentTime = millis();
    
    // Read DIP switches (optimized - every 50ms)
    if (currentTime - lastLoopTime >= 50) {
        readSystemDIPSwitch();
        readTimerDIPSwitch();
        lastLoopTime = currentTime;
    }
    
    // ===== SYSTEM 8 =====
    if (current_system == SYSTEM_EIGHT) {
        systemEightTask();
        checkMasterButton1();
        enterLightSleep();
        return;
    }
    
    // ===== SYSTEM 4 =====
    if (current_system == SYSTEM_FOUR) {
        if (!system4_active) {
            systemFourStart();
        } else {
            systemFourTask();
        }
        enterLightSleep();
        return;
    }
    
    // ===== SYSTEM DEFAULT =====
    if (current_system == SYSTEM_DEFAULT) {
        systemDefaultTask();
        if (timer_active) {
            timer_active = false;
            active_button = 0;
        }
        enterLightSleep();
        return;
    }
    
    // ===== TIMER HANDLING =====
    if (timer_active) {
        if (currentTime - timer_start >= timer_duration) {
            turnOffAllLEDs();
            timer_active = false;
            active_button = 0;
            system7_active = false;
            system7_combination = 0;
            Serial.println("Timer expired - LEDs OFF");
            lastActivityTime = currentTime;
        }
    }
    
    // ===== BUTTON READING =====
    bool current_b1 = digitalRead(B1) == LOW;
    bool current_b2 = digitalRead(B2) == LOW;
    bool current_b3 = digitalRead(B3) == LOW;
    bool current_b4 = digitalRead(B4) == LOW;
    
    // ===== SYSTEM 6: 4 AND =====
    if (current_system == SYSTEM_SIX && !timer_active) {
        if (checkCombinationPress(current_b1, current_b2, current_b3, current_b4)) {
            if (debounce()) {
                systemSixTask();
            }
        }
    }
    
    // ===== SYSTEM 7: 2 AND =====
    if (current_system == SYSTEM_SEVEN) {
        if (current_b1 && current_b2 && !current_b3 && !current_b4) {
            if (debounce()) {
                system7_combination = 1;
                systemSevenTask();
            }
        } else if (current_b3 && current_b4 && !current_b1 && !current_b2) {
            if (debounce()) {
                system7_combination = 2;
                systemSevenTask();
            }
        }
    }
    
    // ===== INDIVIDUAL BUTTONS =====
    if (!timer_active && current_system != SYSTEM_SIX && 
        current_system != SYSTEM_SEVEN && current_system != SYSTEM_EIGHT) {
        
        if (current_b1 && !last_b1_state && debounce()) {
            switch(current_system) {
                case SYSTEM_ONE: systemOneTask(1); break;
                case SYSTEM_TWO: systemTwoTask(1); break;
                case SYSTEM_THREE: systemThreeTask(1); break;
                case SYSTEM_FIVE: systemFiveTask(1); break;
                default: break;
            }
        }
        else if (current_b2 && !last_b2_state && debounce()) {
            switch(current_system) {
                case SYSTEM_ONE: systemOneTask(2); break;
                case SYSTEM_TWO: systemTwoTask(2); break;
                case SYSTEM_THREE: systemThreeTask(2); break;
                case SYSTEM_FIVE: systemFiveTask(2); break;
                default: break;
            }
        }
        else if (current_b3 && !last_b3_state && debounce()) {
            switch(current_system) {
                case SYSTEM_ONE: systemOneTask(3); break;
                case SYSTEM_TWO: systemTwoTask(3); break;
                case SYSTEM_THREE: systemThreeTask(3); break;
                case SYSTEM_FIVE: systemFiveTask(3); break;
                default: break;
            }
        }
        else if (current_b4 && !last_b4_state && debounce()) {
            switch(current_system) {
                case SYSTEM_ONE: systemOneTask(4); break;
                case SYSTEM_TWO: systemTwoTask(4); break;
                case SYSTEM_THREE: systemThreeTask(4); break;
                case SYSTEM_FIVE: systemFiveTask(4); break;
                default: break;
            }
        }
    }
    
    // Update states
    last_b1_state = current_b1;
    last_b2_state = current_b2;
    last_b3_state = current_b3;
    last_b4_state = current_b4;
    
    // ===== PERFORMANCE MONITORING =====
    loopCount++;
    if (currentTime - lastStatsPrint > 30000) { // Every 30 seconds
        lastStatsPrint = currentTime;
        Serial.printf("Performance: %d loops/sec | System: %d\n", loopCount / 30, current_system);
        loopCount = 0;
    }
    
    // ===== ENTER LIGHT SLEEP =====
    enterLightSleep();
    
    // Small delay for heat reduction
    delay(2);
}
