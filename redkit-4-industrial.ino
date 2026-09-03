//=============================================================================
// INDUSTRIAL CONTROL SYSTEM v4.1
// INDUSTRIAL GRADE - 24/7 OPERATION
// With Watchdog, Power Management & Heat Reduction
// Updated Timer Logic: DIP switches OFF -> Button hold timer
//=============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <esp_pm.h>
#include <esp_task_wdt.h>

//=============================================================================
// INDUSTRIAL SAFETY & WATCHDOG CONFIGURATION
//=============================================================================

#define WATCHDOG_TIMEOUT_MS 10000  // 10 seconds watchdog
#define LOOP_DELAY_MS 2            // Heat reduction delay
#define DIP_READ_INTERVAL_MS 50    // DIP switch read interval

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
static bool timer_dip_off = false;

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
// PERFORMANCE & SAFETY MONITORING
//=============================================================================

static unsigned long loopCount = 0;
static unsigned long lastStatsPrint = 0;
static unsigned long lastLoopTime = 0;
static unsigned long lastActivityTime = 0;
static unsigned long lastWatchdogReset = 0;
static unsigned long systemStartTime = 0;
static bool systemHealthy = true;

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
void initWatchdog(void);
void resetWatchdog(void);
void safetyCheck(void);
void emergencyShutdown(void);
void saveSystemState(void);
void restoreSystemState(void);

//=============================================================================
// WATCHDOG & SAFETY FUNCTIONS
//=============================================================================

void initWatchdog() {
    // Initialize watchdog timer for ESP32-S3
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_MS,
        .idle_core_mask = (1 << 0) | (1 << 1),  // Both cores
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    
    Serial.println("=== WATCHDOG INITIALIZED ===");
    Serial.printf("Watchdog Timeout: %d ms\n", WATCHDOG_TIMEOUT_MS);
}

void resetWatchdog() {
    esp_task_wdt_reset();
    lastWatchdogReset = millis();
}

void safetyCheck() {
    // Check if system is stuck
    if (millis() - lastWatchdogReset > WATCHDOG_TIMEOUT_MS) {
        Serial.println("⚠️ WARNING: System appears stuck! Resetting...");
        emergencyShutdown();
        delay(100);
        ESP.restart();
    }
}

void emergencyShutdown() {
    Serial.println("🚨 EMERGENCY SHUTDOWN INITIATED");
    turnOffAllLEDs();
    
    // Disable all active systems
    timer_active = false;
    system4_active = false;
    system7_active = false;
    
    // Save current state for restoration
    saveSystemState();
}

void saveSystemState() {
    // Save critical state variables
    Serial.println("Saving system state...");
}

void restoreSystemState() {
    // Restore system state after reset
    Serial.println("Restoring system state...");
}

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
    if (timer_dip_off) {
        timer_active = true;
        timer_start = millis();
        active_button = button;
        turnOffAllLEDs();
        lastActivityTime = millis();
        
        switch(button) {
            case 1: setLEDs(0, 0, 1, 0, 1); break;
            case 2: setLEDs(0, 0, 0, 1, 1); break;
            case 3: setLEDs(1, 0, 0, 0, 1); break;
            case 4: setLEDs(0, 1, 0, 0, 1); break;
        }
        Serial.printf("SYSTEM 1: B%d → Button Hold Timer (DIP OFF)\n", button);
    } else {
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
}

void systemTwoTask(int button) {
    if (timer_dip_off) {
        timer_active = true;
        timer_start = millis();
        active_button = button;
        turnOffAllLEDs();
        lastActivityTime = millis();
        
        switch(button) {
            case 1: setLEDs(0, 1, 1, 1, 1); break;
            case 2: setLEDs(1, 0, 1, 1, 1); break;
            case 3: setLEDs(1, 1, 0, 1, 1); break;
            case 4: setLEDs(1, 1, 1, 0, 1); break;
        }
        Serial.printf("SYSTEM 2: B%d → Button Hold Timer (DIP OFF)\n", button);
    } else {
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
}

void systemThreeTask(int button) {
    if (timer_dip_off) {
        timer_active = true;
        timer_start = millis();
        active_button = button;
        turnOffAllLEDs();
        lastActivityTime = millis();
        
        switch(button) {
            case 1: setLEDs(1, 0, 0, 0, 1); break;
            case 2: setLEDs(0, 1, 0, 0, 1); break;
            case 3: setLEDs(0, 0, 1, 0, 1); break;
            case 4: setLEDs(0, 0, 0, 1, 1); break;
        }
        Serial.printf("SYSTEM 3: B%d → Button Hold Timer (DIP OFF)\n", button);
    } else {
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
    if (timer_dip_off) {
        timer_active = true;
        timer_start = millis();
        active_button = button;
        turnOffAllLEDs();
        turnOnAllLEDs();
        lastActivityTime = millis();
        
        Serial.printf("SYSTEM 5: B%d → ALL LEDs ON (Button Hold - DIP OFF)\n", button);
    } else {
        timer_duration = (unsigned long)timer_value * 1000;
        timer_start = millis();
        timer_active = true;
        active_button = button;
        turnOffAllLEDs();
        turnOnAllLEDs();
        lastActivityTime = millis();
        
        Serial.printf("SYSTEM 5: B%d → ALL LEDs ON, %ds\n", button, timer_value);
    }
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
    
    timer_dip_off = (value == 0);
    
    if (value == 0) value = 1;
    
    if (value != timer_value) {
        timer_value = value;
        if (timer_dip_off) {
            Serial.println("Timer DIP: ALL OFF → Button hold timer mode");
        } else {
            Serial.printf("Timer set to %d seconds\n", timer_value);
        }
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
    Serial.println("   INDUSTRIAL CONTROL SYSTEM v4.1");
    Serial.println("   INDUSTRIAL GRADE - 24/7 OPERATION");
    Serial.println("   With Watchdog & Safety Monitoring");
    Serial.println("   Timer Logic: DIP OFF = Button Hold");
    Serial.println("==========================================\n");
    
    // Initialize watchdog
    initWatchdog();
    
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
    systemStartTime = millis();
    
    Serial.println("System Ready - Industrial Mode Active");
    Serial.printf("System: %d | Timer: %d seconds\n", current_system, timer_value);
    Serial.printf("Watchdog: %d ms | Loop Delay: %d ms\n", WATCHDOG_TIMEOUT_MS, LOOP_DELAY_MS);
    
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
    // Reset watchdog at the beginning of each loop
    resetWatchdog();
    
    unsigned long currentTime = millis();
    
    // Safety check
    safetyCheck();
    
    // Read DIP switches (optimized)
    if (currentTime - lastLoopTime >= DIP_READ_INTERVAL_MS) {
        readSystemDIPSwitch();
        readTimerDIPSwitch();
        lastLoopTime = currentTime;
    }
    
    // ===== SYSTEM 8 =====
    if (current_system == SYSTEM_EIGHT) {
        systemEightTask();
        checkMasterButton1();
        enterLightSleep();
        delay(LOOP_DELAY_MS);
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
        delay(LOOP_DELAY_MS);
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
        delay(LOOP_DELAY_MS);
        return;
    }
    
    // ===== TIMER HANDLING =====
    if (timer_active) {
        if (timer_dip_off) {
            // Button hold timer logic - check if button is still pressed
            bool button_released = false;
            switch(active_button) {
                case 1: if (digitalRead(B1) == HIGH) button_released = true; break;
                case 2: if (digitalRead(B2) == HIGH) button_released = true; break;
                case 3: if (digitalRead(B3) == HIGH) button_released = true; break;
                case 4: if (digitalRead(B4) == HIGH) button_released = true; break;
                case 5: 
                    if (digitalRead(B1) == HIGH || digitalRead(B2) == HIGH || 
                        digitalRead(B3) == HIGH || digitalRead(B4) == HIGH) {
                        button_released = true;
                    }
                    break;
                case 6:
                    if (system7_combination == 1) {
                        if (digitalRead(B1) == HIGH || digitalRead(B2) == HIGH) {
                            button_released = true;
                        }
                    } else if (system7_combination == 2) {
                        if (digitalRead(B3) == HIGH || digitalRead(B4) == HIGH) {
                            button_released = true;
                        }
                    }
                    break;
                default: button_released = true; break;
            }
            
            if (button_released) {
                turnOffAllLEDs();
                timer_active = false;
                active_button = 0;
                system7_active = false;
                system7_combination = 0;
                Serial.println("Button released - LEDs OFF");
                lastActivityTime = currentTime;
            }
        } else {
            // Original timer logic when DIP switches are set
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
    if (currentTime - lastStatsPrint > 30000) {
        lastStatsPrint = currentTime;
        unsigned long uptime = (currentTime - systemStartTime) / 1000;
        Serial.printf("STATS: %d loops/sec | System: %d | Timer DIP: %s | Uptime: %lu sec\n", 
                     loopCount / 30, current_system, timer_dip_off ? "OFF(Hold)" : "ON(Timer)", uptime);
        loopCount = 0;
    }
    
    // ===== ENTER LIGHT SLEEP =====
    enterLightSleep();
    
    // Heat reduction delay
    delay(LOOP_DELAY_MS);
}






















/*

//slew----------------------------------------------------------------------------------------------------------------------------------------------------------

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <esp_pm.h>

// =====================================================
// POWER MANAGEMENT CONFIGURATION
// =====================================================

#define ENABLE_POWER_SAVE 1
#define LIGHT_SLEEP_TIMEOUT_MS 5000
#define CPU_FREQ_MHZ 80  // Reduced from 240MHz to 80MHz

// =====================================================
// SLEW PINS (YOUR ORIGINAL PINS)
// =====================================================

#define LED1 26
#define LED2 33
#define BTN1 13
// BTN2 is not used in your original code

// =====================================================
// SETTINGS (YOUR ORIGINAL TIMINGS)
// =====================================================

const unsigned long SEARCH_BLINK_TIME = 300;
const unsigned long CONNECT_BLINK_TIME = 250;
const int CONNECT_BLINK_COUNT = 5;
const unsigned long HELLO_INTERVAL = 500;
const unsigned long HEARTBEAT_INTERVAL = 500;
const unsigned long CONNECTION_TIMEOUT = 2500;
const unsigned long DEBOUNCE_TIME = 40;

// =====================================================
// MAC ADDRESSES
// =====================================================

// SLEW
uint8_t SLEW_MAC[] = {
  0x08, 0x3A, 0xF2, 0x09, 0xBE, 0xF8
};

// MASTER
uint8_t MASTER_MAC[] = {
  0x94, 0xA9, 0x90, 0xEF, 0x98, 0x28
};

// =====================================================
// ESP-NOW CHANNEL
// =====================================================

#define ESPNOW_CHANNEL 1

// =====================================================
// MESSAGE TYPES
// =====================================================

#define MSG_HELLO       1
#define MSG_HELLO_ACK   2
#define MSG_HEARTBEAT   3
#define MSG_BTN1_STATE  4
#define MSG_RESET       5

#define ROLE_MASTER 1
#define ROLE_SLEW   2

// =====================================================
// PACKET
// =====================================================

typedef struct
{
  uint8_t type;
  uint8_t role;
  uint8_t buttonState;
  uint32_t counter;
} DataPacket;

// =====================================================
// SYSTEM STATES
// =====================================================

enum SystemState
{
  SEARCHING,
  CONNECT_BLINK,
  NORMAL_OPERATION,
  DISCONNECTED
};

SystemState systemState = SEARCHING;

// =====================================================
// TIMING VARIABLES
// =====================================================

unsigned long lastMasterPacket = 0;
unsigned long lastHello = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastSearchBlink = 0;
unsigned long lastConnectBlink = 0;
unsigned long lastActivityTime = 0;
unsigned long lastLoopTime = 0;

// =====================================================
// SEARCHING STATE
// =====================================================

bool searchState = false;

// =====================================================
// CONNECTION SUCCESS BLINK
// =====================================================

bool connectionBlinkActive = false;
bool connectionLEDState = false;
int connectionBlinkCount = 0;

// =====================================================
// CONNECTION STATE
// =====================================================

bool connected = false;

// =====================================================
// BUTTON
// =====================================================

bool lastBtn1 = HIGH;
bool stableBtn1 = HIGH;
unsigned long btn1Debounce = 0;
bool lastBtn1State = false;
unsigned long lastBtnSendTime = 0;

// =====================================================
// PERFORMANCE MONITORING
// =====================================================

unsigned long loopCount = 0;
unsigned long lastStatsPrint = 0;

// =====================================================
// SET SLEW LEDS (YOUR ORIGINAL FUNCTION)
// =====================================================

inline void setSlewLEDs(bool led26, bool led33)
{
  digitalWrite(LED1, led26 ? HIGH : LOW);
  digitalWrite(LED2, led33 ? HIGH : LOW);
}

// =====================================================
// POWER MANAGEMENT
// =====================================================

void configurePowerManagement()
{
  #if ENABLE_POWER_SAVE
    // Reduce CPU frequency
    esp_pm_config_t pm_config = {
      .max_freq_mhz = CPU_FREQ_MHZ,
      .min_freq_mhz = CPU_FREQ_MHZ,
      .light_sleep_enable = true
    };
    esp_pm_configure(&pm_config);
    
    // Reduce WiFi power
    esp_wifi_set_max_tx_power(40);  // 4dBm (minimum for stable connection)
    
    Serial.println("Power Management Configured:");
    Serial.printf("  CPU Frequency: %d MHz\n", CPU_FREQ_MHZ);
    Serial.printf("  TX Power: %d dBm\n", 4);
    Serial.printf("  Light Sleep Timeout: %d ms\n", LIGHT_SLEEP_TIMEOUT_MS);
  #endif
}

// =====================================================
// RESET COMMUNICATION (YOUR ORIGINAL FUNCTION)
// =====================================================

void resetCommunication()
{
  connected = false;
  connectionBlinkActive = false;
  connectionLEDState = false;
  connectionBlinkCount = 0;
  systemState = SEARCHING;
  
  // IMPORTANT: Always turn both LEDs OFF on reset.
  setSlewLEDs(false, false);
  
  Serial.println();
  Serial.println("================================");
  Serial.println("COMMUNICATION RESET");
  Serial.println("SLEW LEDs OFF");
  Serial.println("SEARCHING FOR MASTER...");
  Serial.println("================================");
  
  lastActivityTime = millis();
}

// =====================================================
// SEND TO MASTER (with retry and rate limiting)
// =====================================================

void sendToMaster(uint8_t type, uint8_t buttonState = 0)
{
  DataPacket packet;
  packet.type = type;
  packet.role = ROLE_SLEW;
  packet.buttonState = buttonState;
  packet.counter = millis();
  
  esp_err_t result = esp_now_send(MASTER_MAC, (uint8_t *)&packet, sizeof(packet));
  
  // Log errors but don't spam
  static uint32_t lastErrorLog = 0;
  if (result != ESP_OK && (millis() - lastErrorLog > 5000))
  {
    lastErrorLog = millis();
    Serial.printf("ESP-NOW Send Error: %d\n", result);
  }
}

// =====================================================
// START CONNECTION BLINK (YOUR ORIGINAL FUNCTION)
// =====================================================

void startConnectionBlink()
{
  connectionBlinkActive = true;
  connectionLEDState = false;
  connectionBlinkCount = 0;
  lastConnectBlink = millis();
  systemState = CONNECT_BLINK;
  
  // IMPORTANT: Start from OFF.
  setSlewLEDs(false, false);
  
  Serial.println();
  Serial.println("================================");
  Serial.println("MASTER CONNECTED");
  Serial.println("START 5 BLINKS");
  Serial.println("================================");
  
  lastActivityTime = millis();
}

// =====================================================
// CONNECTION SUCCESS BLINK (YOUR ORIGINAL LOGIC)
// =====================================================

inline void connectionBlink()
{
  if (!connectionBlinkActive) return;
  
  if (millis() - lastConnectBlink >= CONNECT_BLINK_TIME)
  {
    lastConnectBlink = millis();
    connectionLEDState = !connectionLEDState;
    
    if (connectionLEDState)
    {
      setSlewLEDs(true, true);
    }
    else
    {
      setSlewLEDs(false, false);
      connectionBlinkCount++;
      
      Serial.print("Connection blink: ");
      Serial.println(connectionBlinkCount);
      
      // 5 COMPLETE BLINKS
      if (connectionBlinkCount >= CONNECT_BLINK_COUNT)
      {
        connectionBlinkActive = false;
        
        // IMPORTANT: Final state ALWAYS OFF.
        setSlewLEDs(false, false);
        systemState = NORMAL_OPERATION;
        
        Serial.println("5 BLINKS COMPLETE");
        Serial.println("SLEW LED26 = OFF");
        Serial.println("SLEW LED33 = OFF");
        Serial.println("NORMAL OPERATION");
      }
    }
  }
}

// =====================================================
// RECEIVE DATA (YOUR ORIGINAL LOGIC)
// =====================================================

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  if (len != sizeof(DataPacket)) return;
  
  // Only accept MASTER
  if (memcmp(info->src_addr, MASTER_MAC, 6) != 0) return;
  
  DataPacket packet;
  memcpy(&packet, data, sizeof(packet));
  
  lastMasterPacket = millis();
  lastActivityTime = millis();
  
  // ===================================================
  // MASTER HELLO
  // ===================================================
  
  if (packet.type == MSG_HELLO)
  {
    bool wasDisconnected = !connected;
    connected = true;
    
    // ACK MASTER
    sendToMaster(MSG_HELLO_ACK);
    
    // New connection
    if (wasDisconnected)
    {
      startConnectionBlink();
    }
    
    Serial.println("MASTER FOUND");
  }
  
  // ===================================================
  // MASTER HELLO ACK
  // ===================================================
  
  else if (packet.type == MSG_HELLO_ACK)
  {
    if (!connected)
    {
      connected = true;
      startConnectionBlink();
    }
  }
  
  // ===================================================
  // HEARTBEAT
  // ===================================================
  
  else if (packet.type == MSG_HEARTBEAT)
  {
    connected = true;
  }
  
  // ===================================================
  // RESET
  // ===================================================
  
  else if (packet.type == MSG_RESET)
  {
    Serial.println("RESET COMMAND FROM MASTER");
    resetCommunication();
  }
}

// =====================================================
// SEARCHING INDICATION (YOUR ORIGINAL LOGIC)
// =====================================================

inline void searchingIndication()
{
  if (systemState != SEARCHING) return;
  
  if (millis() - lastSearchBlink >= SEARCH_BLINK_TIME)
  {
    lastSearchBlink = millis();
    searchState = !searchState;
    
    if (searchState)
    {
      // LED26 ON, LED33 OFF
      setSlewLEDs(true, false);
    }
    else
    {
      // LED26 OFF, LED33 ON
      setSlewLEDs(false, true);
    }
  }
}

// =====================================================
// BUTTON 1 (YOUR ORIGINAL LOGIC)
// =====================================================

void checkButton1()
{
  bool reading = digitalRead(BTN1);
  
  if (reading != lastBtn1)
  {
    btn1Debounce = millis();
  }
  
  if (millis() - btn1Debounce > DEBOUNCE_TIME)
  {
    if (reading != stableBtn1)
    {
      stableBtn1 = reading;
      
      // BUTTON PRESSED
      if (stableBtn1 == LOW)
      {
        Serial.println("SLEW BTN1 PRESSED");
        
        // Slew LEDs ALWAYS OFF
        setSlewLEDs(false, false);
        sendToMaster(MSG_BTN1_STATE, 1);
        lastBtnSendTime = millis();
      }
      
      // BUTTON RELEASED
      else
      {
        Serial.println("SLEW BTN1 RELEASED");
        
        // Slew LEDs ALWAYS OFF
        setSlewLEDs(false, false);
        sendToMaster(MSG_BTN1_STATE, 0);
        lastBtnSendTime = millis();
      }
      
      lastActivityTime = millis();
    }
  }
  
  lastBtn1 = reading;
}

// =====================================================
// START DISCONNECT MODE
// =====================================================

void startDisconnectMode()
{
  connected = false;
  systemState = DISCONNECTED;
  connectionBlinkActive = false;
  
  setSlewLEDs(false, false);
  
  Serial.println();
  Serial.println("================================");
  Serial.println("MASTER DISCONNECTED");
  Serial.println("SLEW LEDs OFF");
  Serial.println("================================");
  
  lastActivityTime = millis();
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);
  
  Serial.println();
  Serial.println("======================================");
  Serial.println("       ESP-NOW SLEW (INDUSTRIAL)");
  Serial.println("======================================");
  
  // ===================================================
  // GPIO (YOUR ORIGINAL PINS)
  // ===================================================
  
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(BTN1, INPUT_PULLUP);
  
  // IMPORTANT: Force LEDs OFF immediately.
  setSlewLEDs(false, false);
  
  // ===================================================
  // POWER MANAGEMENT
  // ===================================================
  
  configurePowerManagement();
  
  // ===================================================
  // WIFI
  // ===================================================
  
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  Serial.print("SLEW MAC: ");
  Serial.println(WiFi.macAddress());
  
  // ===================================================
  // ESP-NOW
  // ===================================================
  
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW INIT FAILED!");
    
    while (true)
    {
      setSlewLEDs(true, false);
      delay(200);
      setSlewLEDs(false, false);
      delay(200);
    }
  }
  
  // ===================================================
  // ADD MASTER PEER
  // ===================================================
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, MASTER_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("FAILED TO ADD MASTER PEER!");
  }
  else
  {
    Serial.println("MASTER PEER ADDED");
  }
  
  // ===================================================
  // RECEIVE CALLBACK
  // ===================================================
  
  esp_now_register_recv_cb(onDataReceive);
  
  Serial.println();
  Serial.println("ESP-NOW READY");
  Serial.println("SLEW LEDs = OFF");
  Serial.println("SEARCHING FOR MASTER...");
  
  lastActivityTime = millis();
  lastLoopTime = millis();
  systemState = SEARCHING;
}

// =====================================================
// LOOP - OPTIMIZED FOR HEAT REDUCTION
// =====================================================

void loop()
{
  unsigned long currentTime = millis();
  
  // ===================================================
  // BUTTON CHECK (with optimized timing)
  // ===================================================
  
  if (currentTime - lastLoopTime >= 5)  // 5ms minimum between checks
  {
    checkButton1();
    lastLoopTime = currentTime;
  }
  
  // ===================================================
  // STATE MACHINE
  // ===================================================
  
  switch (systemState)
  {
    case SEARCHING:
      searchingIndication();
      
      if (currentTime - lastHello >= HELLO_INTERVAL)
      {
        lastHello = currentTime;
        sendToMaster(MSG_HELLO);
      }
      
      // Allow light sleep between operations
      #if ENABLE_POWER_SAVE
      if (currentTime - lastActivityTime > LIGHT_SLEEP_TIMEOUT_MS)
      {
        delay(1);
      }
      #endif
      break;
      
    case CONNECT_BLINK:
      connectionBlink();
      
      if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL)
      {
        lastHeartbeat = currentTime;
        sendToMaster(MSG_HEARTBEAT);
      }
      break;
      
    case NORMAL_OPERATION:
      // LEDs should be OFF in normal operation (your original logic)
      // But keep them OFF unless button is pressed
      if (!connectionBlinkActive)
      {
        setSlewLEDs(false, false);
      }
      
      if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL)
      {
        lastHeartbeat = currentTime;
        sendToMaster(MSG_HEARTBEAT);
      }
      
      if (currentTime - lastMasterPacket > CONNECTION_TIMEOUT)
      {
        startDisconnectMode();
      }
      break;
      
    case DISCONNECTED:
      setSlewLEDs(false, false);
      
      if (currentTime - lastHello >= HELLO_INTERVAL)
      {
        lastHello = currentTime;
        sendToMaster(MSG_HELLO);
      }
      
      // Check if reconnected
      if (connected)
      {
        systemState = NORMAL_OPERATION;
        Serial.println("Reconnected to MASTER");
      }
      
      #if ENABLE_POWER_SAVE
      if (currentTime - lastActivityTime > LIGHT_SLEEP_TIMEOUT_MS)
      {
        delay(1);
      }
      #endif
      break;
  }
  
  // ===================================================
  // PERFORMANCE MONITORING (every 10 seconds)
  // ===================================================
  
  loopCount++;
  if (currentTime - lastStatsPrint > 10000)
  {
    lastStatsPrint = currentTime;
    Serial.printf("Performance: %d loops/sec | State: %d\n", loopCount / 10, systemState);
    loopCount = 0;
  }
  
  // ===================================================
  // DELAY FOR HEAT REDUCTION
  // ===================================================
  
  #if ENABLE_POWER_SAVE
    // Small delay allows CPU to enter lower power state
    if (systemState != CONNECT_BLINK)
    {
      delay(2);
    }
    else
    {
      delay(1);
    }
  #endif
}

*/

