#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ===== GPIO Definitions =====

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

// ===== ESP-NOW System 8 Definitions =====
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
uint8_t SLEW_MAC[] = {
  0x08, 0x3A, 0xF2, 0x09, 0xBE, 0xF8
};

uint8_t MASTER_MAC[] = {
  0x94, 0xA9, 0x90, 0xEF, 0x98, 0x28
};

// ===== ESP-NOW Packet Structure =====
typedef struct {
  uint8_t type;
  uint8_t role;
  uint8_t buttonState;
  uint32_t counter;
} DataPacket;

// ===== System Definitions =====
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

// ===== System 4 States =====
typedef enum {
    SEQ_IDLE = 0,
    SEQ_L1_L5 = 1,
    SEQ_L2_L5 = 2,
    SEQ_L3_L5 = 3,
    SEQ_L4_L5 = 4
} sequence_state_t;

// ===== System 8 Master States =====
typedef enum {
    MASTER_SEARCHING = 0,
    MASTER_CONNECT_BLINK = 1,
    MASTER_RESET_READY = 2,
    MASTER_BTN_PRESSED = 3,
    MASTER_BTN_RELEASED = 4,
    MASTER_DISCONNECTED = 5
} master_state_t;

// ===== Global Variables =====
static unsigned long timer_start = 0;
static unsigned long timer_duration = 0;
static bool timer_active = false;
static int active_button = 0;
static int timer_value = 0;
static system_mode_t current_system = SYSTEM_DEFAULT;
static system_mode_t last_system = SYSTEM_DEFAULT;

// Button states for debouncing
static bool last_b1_state = true;
static bool last_b2_state = true;
static bool last_b3_state = true;
static bool last_b4_state = true;
static unsigned long last_debounce_time = 0;
static const int debounce_delay_ms = 50;

// Default system blink control
static bool default_blink_state = false;
static unsigned long default_blink_timer = 0;
static const int default_blink_interval_ms = 1000;

// System 4 variables (Auto-running sequential pattern)
static bool system4_active = false;
static sequence_state_t seq_state = SEQ_IDLE;
static unsigned long seq_step_start = 0;
static unsigned long seq_step_duration = 0;
static int current_led_index = 0; // 0=L1, 1=L2, 2=L3, 3=L4

// System 7 (2 AND manner) variables
static bool system7_active = false;
static int system7_combination = 0; // 1 = B1+B2, 2 = B3+B4

// ===== System 8 Variables =====
static bool espNowInitialized = false;
static unsigned long lastPacket = 0;
static unsigned long lastHello = 0;
static unsigned long lastHeartbeat = 0;
static const unsigned long HELLO_INTERVAL = 500;
static const unsigned long HEARTBEAT_INTERVAL = 500;
static const unsigned long CONNECTION_TIMEOUT = 2500;
static const unsigned long SEARCH_BLINK_TIME = 100;
static const unsigned long CONNECT_BLINK_TIME = 200;
static const int CONNECT_BLINK_COUNT = 3;
static const unsigned long SEQUENCE_TIME = 300;

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
static const unsigned long DEBOUNCE_TIME = 40;

// ===== Function Prototypes =====
void turnOffAllLEDs(void);
void turnOnAllLEDs(void);
void readTimerDIPSwitch(void);
void readSystemDIPSwitch(void);
bool debounce(void);
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
void setLEDs(uint8_t l1, uint8_t l2, uint8_t l3, uint8_t l4, uint8_t l5);
void setSystem8LEDs(bool led1, bool led2, bool led3, bool led4, bool led5);
void enterResetReadyMode(void);
void startConnectionBlink(void);
void connectionBlinkIndication(void);
void searchingIndication(void);
void disconnectIndication(void);
void startDisconnectMode(void);
void sequenceIndication(void);
void checkMasterButton1(void);

// ===== LED Control Functions =====

void turnOffAllLEDs(void) {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    digitalWrite(L3, LOW);
    digitalWrite(L4, LOW);
    digitalWrite(L5, LOW);
}

void turnOnAllLEDs(void) {
    digitalWrite(L1, HIGH);
    digitalWrite(L2, HIGH);
    digitalWrite(L3, HIGH);
    digitalWrite(L4, HIGH);
    digitalWrite(L5, HIGH);
}

void setLEDs(uint8_t l1, uint8_t l2, uint8_t l3, uint8_t l4, uint8_t l5) {
    digitalWrite(L1, l1);
    digitalWrite(L2, l2);
    digitalWrite(L3, l3);
    digitalWrite(L4, l4);
    digitalWrite(L5, l5);
}

void setSystem8LEDs(bool led1, bool led2, bool led3, bool led4, bool led5) {
    digitalWrite(L1, led1 ? HIGH : LOW);
    digitalWrite(L2, led2 ? HIGH : LOW);
    digitalWrite(L3, led3 ? HIGH : LOW);
    digitalWrite(L4, led4 ? HIGH : LOW);
    digitalWrite(L5, led5 ? HIGH : LOW);
}

// ===== System 8 Master State Functions =====

void enterResetReadyMode() {
    masterState = MASTER_RESET_READY;
    slewButtonPressed = false;
    sequenceLED1 = true;
    
    // LED2 ON, all others OFF
    setSystem8LEDs(false, true, false, false, false);
    
    Serial.println();
    Serial.println("================================");
    Serial.println("SYSTEM 8: RESET/READY MODE");
    Serial.println("LED1 = OFF");
    Serial.println("LED2 = ON");
    Serial.println("LED3 = OFF");
    Serial.println("LED4 = OFF");
    Serial.println("LED5 = OFF");
    Serial.println("================================");
}

void startConnectionBlink() {
    masterState = MASTER_CONNECT_BLINK;
    connectBlinkCount = 0;
    connectLEDState = false;
    lastConnectBlink = millis();
    
    setSystem8LEDs(false, false, false, false, false);
    
    Serial.println();
    Serial.println("================================");
    Serial.println("SYSTEM 8: SLEW CONNECTED");
    Serial.println("CONNECTION INDICATION: 3 BLINKS");
    Serial.println("================================");
}

void connectionBlinkIndication() {
    if (masterState != MASTER_CONNECT_BLINK) return;
    
    if (millis() - lastConnectBlink >= CONNECT_BLINK_TIME) {
        lastConnectBlink = millis();
        connectLEDState = !connectLEDState;
        
        if (connectLEDState) {
            // LED1 + LED2 ON, all others OFF
            setSystem8LEDs(true, true, false, false, false);
        } else {
            // All OFF
            setSystem8LEDs(false, false, false, false, false);
            connectBlinkCount++;
            
            if (connectBlinkCount >= CONNECT_BLINK_COUNT) {
                enterResetReadyMode();
            }
        }
    }
}

void searchingIndication() {
    if (masterState != MASTER_SEARCHING) return;
    
    if (millis() - lastSearchBlink >= SEARCH_BLINK_TIME) {
        lastSearchBlink = millis();
        searchLED1 = !searchLED1;
        
        if (searchLED1) {
            // LED1 ON, all others OFF
            setSystem8LEDs(true, false, false, false, false);
        } else {
            // LED2 ON, all others OFF
            setSystem8LEDs(false, true, false, false, false);
        }
    }
}

void startDisconnectMode() {
    masterState = MASTER_DISCONNECTED;
    slewButtonPressed = false;
    disconnectLED1 = true;
    lastDisconnectBlink = millis();
    
    // LED1 ON initially
    setSystem8LEDs(true, false, false, false, false);
    
    Serial.println();
    Serial.println("================================");
    Serial.println("SYSTEM 8: SLEW DISCONNECTED");
    Serial.println("DISCONNECT INDICATION");
    Serial.println("LED1 <-> LED2");
    Serial.println("================================");
}

void disconnectIndication() {
    if (masterState != MASTER_DISCONNECTED) return;
    
    if (millis() - lastDisconnectBlink >= SEARCH_BLINK_TIME) {
        lastDisconnectBlink = millis();
        disconnectLED1 = !disconnectLED1;
        
        if (disconnectLED1) {
            setSystem8LEDs(true, false, false, false, false);
        } else {
            setSystem8LEDs(false, true, false, false, false);
        }
    }
}

void sequenceIndication() {
    if (masterState != MASTER_BTN_RELEASED) return;
    
    // LED3 ON (constant), LED4 OFF (constant)
    if (millis() - lastSequenceChange >= SEQUENCE_TIME) {
        lastSequenceChange = millis();
        sequenceLED1 = !sequenceLED1;
        
        if (sequenceLED1) {
            // LED1 ON, LED2 OFF, LED3 ON, LED4 OFF
            setSystem8LEDs(true, false, true, false, false);
        } else {
            // LED1 OFF, LED2 ON, LED3 ON, LED4 OFF
            setSystem8LEDs(false, true, true, false, false);
        }
    }
}

void checkMasterButton1() {
    if (current_system != SYSTEM_EIGHT) return;
    if (masterState == MASTER_RESET_READY) return;
    
    bool reading = digitalRead(B1);
    
    if (reading != lastBtn1) {
        btn1Debounce = millis();
    }
    
    if (millis() - btn1Debounce > DEBOUNCE_TIME) {
        if (reading != stableBtn1) {
            stableBtn1 = reading;
            
            if (stableBtn1 == LOW) {
                Serial.println("SYSTEM 8: MASTER BTN1 PRESSED - Entering Reset Mode");
                // Send reset to Slew with default buttonState 0
                sendToSlew(MSG_RESET, 0);
                // Enter reset mode
                enterResetReadyMode();
            }
        }
    }
    
    lastBtn1 = reading;
}

// ===== System Handlers =====

void systemDefaultTask(void) {
    // Blink all LEDs every second
    unsigned long current_time = millis();
    if (current_time - default_blink_timer >= default_blink_interval_ms) {
        default_blink_timer = current_time;
        default_blink_state = !default_blink_state;
        
        if (default_blink_state) {
            turnOnAllLEDs();
            Serial.println("Default System: LEDs ON");
        } else {
            turnOffAllLEDs();
            Serial.println("Default System: LEDs OFF");
        }
    }
}

void systemOneTask(int button) {
    // System 1: Two-way with two projectors
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = button;
    
    turnOffAllLEDs();
    
    switch(button) {
        case 1:
            setLEDs(0, 0, 1, 0, 1);
            Serial.printf("SYSTEM 1: B1 → L3+L5, %d seconds\n", timer_value);
            break;
        case 2:
            setLEDs(0, 0, 0, 1, 1);
            Serial.printf("SYSTEM 1: B2 → L4+L5, %d seconds\n", timer_value);
            break;
        case 3:
            setLEDs(1, 0, 0, 0, 1);
            Serial.printf("SYSTEM 1: B3 → L1+L5, %d seconds\n", timer_value);
            break;
        case 4:
            setLEDs(0, 1, 0, 0, 1);
            Serial.printf("SYSTEM 1: B4 → L2+L5, %d seconds\n", timer_value);
            break;
    }
}

void systemTwoTask(int button) {
    // System 2: Signal Pro
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = button;
    
    turnOffAllLEDs();
    
    switch(button) {
        case 1:
            setLEDs(0, 1, 1, 1, 1);
            Serial.printf("SYSTEM 2: B1 → L2,L3,L4,L5, %d seconds\n", timer_value);
            break;
        case 2:
            setLEDs(1, 0, 1, 1, 1);
            Serial.printf("SYSTEM 2: B2 → L1,L3,L4,L5, %d seconds\n", timer_value);
            break;
        case 3:
            setLEDs(1, 1, 0, 1, 1);
            Serial.printf("SYSTEM 2: B3 → L1,L2,L4,L5, %d seconds\n", timer_value);
            break;
        case 4:
            setLEDs(1, 1, 1, 0, 1);
            Serial.printf("SYSTEM 2: B4 → L1,L2,L3,L5, %d seconds\n", timer_value);
            break;
    }
}

void systemThreeTask(int button) {
    // System 3: Two shutter with projectors
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = button;
    
    turnOffAllLEDs();
    
    switch(button) {
        case 1:
            setLEDs(1, 0, 0, 0, 1);
            Serial.printf("SYSTEM 3: B1 → L1+L5, %d seconds\n", timer_value);
            break;
        case 2:
            setLEDs(0, 1, 0, 0, 1);
            Serial.printf("SYSTEM 3: B2 → L2+L5, %d seconds\n", timer_value);
            break;
        case 3:
            setLEDs(0, 0, 1, 0, 1);
            Serial.printf("SYSTEM 3: B3 → L3+L5, %d seconds\n", timer_value);
            break;
        case 4:
            setLEDs(0, 0, 0, 1, 1);
            Serial.printf("SYSTEM 3: B4 → L4+L5, %d seconds\n", timer_value);
            break;
    }
}

void systemFourStart(void) {
    // System 4: Auto-running sequential pattern - L1+L5, L2+L5, L3+L5, L4+L5
    seq_step_duration = (unsigned long)timer_value * 1000;
    system4_active = true;
    seq_state = SEQ_L1_L5;
    current_led_index = 0;
    seq_step_start = millis();
    
    turnOffAllLEDs();
    setLEDs(1, 0, 0, 0, 1);
    
    Serial.println("SYSTEM 4: Auto-running sequential pattern started");
    Serial.printf("SYSTEM 4: Each step duration = %d seconds\n", timer_value);
    Serial.println("SYSTEM 4: Pattern: L1+L5 → L2+L5 → L3+L5 → L4+L5 (looping)");
}

void systemFourTask(void) {
    if (!system4_active) return;
    
    unsigned long current_time = millis();
    
    if (current_time - seq_step_start >= seq_step_duration) {
        current_led_index++;
        seq_step_start = current_time;
        
        if (current_led_index >= 4) {
            current_led_index = 0;
        }
        
        switch(current_led_index) {
            case 0:
                setLEDs(1, 0, 0, 0, 1);
                seq_state = SEQ_L1_L5;
                Serial.println("SYSTEM 4: L1+L5 ON");
                break;
            case 1:
                setLEDs(0, 1, 0, 0, 1);
                seq_state = SEQ_L2_L5;
                Serial.println("SYSTEM 4: L2+L5 ON");
                break;
            case 2:
                setLEDs(0, 0, 1, 0, 1);
                seq_state = SEQ_L3_L5;
                Serial.println("SYSTEM 4: L3+L5 ON");
                break;
            case 3:
                setLEDs(0, 0, 0, 1, 1);
                seq_state = SEQ_L4_L5;
                Serial.println("SYSTEM 4: L4+L5 ON");
                break;
            default:
                break;
        }
    }
}

void systemFiveTask(int button) {
    // System 5: OR manner - Any button turns on all LEDs
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = button;
    
    turnOffAllLEDs();
    turnOnAllLEDs();
    
    Serial.printf("SYSTEM 5: B%d → ALL LEDs ON, %d seconds\n", button, timer_value);
    Serial.println("SYSTEM 5: All other buttons locked during timer");
}

void systemSixTask(void) {
    // System 6: 4 AND manner - All 4 buttons must be pressed together
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = 5;
    
    turnOffAllLEDs();
    turnOnAllLEDs();
    
    Serial.printf("SYSTEM 6: B1+B2+B3+B4 → ALL LEDs ON, %d seconds\n", timer_value);
    Serial.println("SYSTEM 6: All buttons locked during timer");
}

void systemSevenTask(void) {
    // System 7: 2 AND manner - B1+B2 or B3+B4 with timer ONLY (NO LOCK)
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = 6;
    
    if (system7_combination == 1) {
        // B1+B2 → L1,L2,L5
        setLEDs(1, 1, 0, 0, 1);
        Serial.printf("SYSTEM 7: B1+B2 → L1,L2,L5, %d seconds (NO LOCK)\n", timer_value);
    } else if (system7_combination == 2) {
        // B3+B4 → L3,L4,L5
        setLEDs(0, 0, 1, 1, 1);
        Serial.printf("SYSTEM 7: B3+B4 → L3,L4,L5, %d seconds (NO LOCK)\n", timer_value);
    }
    
    Serial.println("SYSTEM 7: Timer active - Other buttons are NOT locked");
}

// ===== System 8 ESP-NOW Functions =====

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
    
    // Add Slew peer
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
    
    esp_now_send(SLEW_MAC, (uint8_t *)&packet, sizeof(packet));
}

void onDataReceiveSystem8(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len != sizeof(DataPacket)) return;
    if (memcmp(info->src_addr, SLEW_MAC, 6) != 0) return;
    
    DataPacket packet;
    memcpy(&packet, data, sizeof(packet));
    lastPacket = millis();
    
    switch(packet.type) {
        case MSG_HELLO:
            if (masterState == MASTER_SEARCHING || masterState == MASTER_DISCONNECTED) {
                startConnectionBlink();
                sendToSlew(MSG_HELLO_ACK, 0);
            } else {
                sendToSlew(MSG_HELLO_ACK, 0);
            }
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
                // Slew BTN1 Pressed
                Serial.println("SYSTEM 8: SLEW BTN1 PRESSED");
                Serial.println("MASTER: LED1 ON + LED4 ON, LED2 OFF");
                
                masterState = MASTER_BTN_PRESSED;
                slewButtonPressed = true;
                
                // LED1 ON + LED4 ON, all others OFF
                setSystem8LEDs(true, false, false, true, false);
                
            } else {
                // Slew BTN1 Released
                Serial.println("SYSTEM 8: SLEW BTN1 RELEASED");
                Serial.println("MASTER: LED3 ON + LED1/LED2 alternate blinking + LED4 OFF");
                
                masterState = MASTER_BTN_RELEASED;
                slewButtonPressed = false;
                sequenceLED1 = true;
                lastSequenceChange = millis();
                
                // LED3 ON, LED4 OFF, LED1/LED2 will alternate
                setSystem8LEDs(true, false, true, false, false);
            }
            break;
            
        case MSG_RESET:
            Serial.println("SYSTEM 8: RESET COMMAND FROM SLEW");
            enterResetReadyMode();
            break;
    }
}

void systemEightTask(void) {
    if (!espNowInitialized) {
        initESPNow();
        return;
    }
    
    // Handle Master States
    switch(masterState) {
        case MASTER_SEARCHING:
            searchingIndication();
            
            // Send hello every 500ms
            if (millis() - lastHello >= HELLO_INTERVAL) {
                lastHello = millis();
                sendToSlew(MSG_HELLO, 0);
            }
            break;
            
        case MASTER_CONNECT_BLINK:
            connectionBlinkIndication();
            
            // Send heartbeat
            if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
                lastHeartbeat = millis();
                sendToSlew(MSG_HEARTBEAT, 0);
            }
            break;
            
        case MASTER_RESET_READY:
            // LED2 ON, all others OFF (enforced)
            setSystem8LEDs(false, true, false, false, false);
            
            // Send heartbeat
            if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
                lastHeartbeat = millis();
                sendToSlew(MSG_HEARTBEAT, 0);
            }
            
            // Check connection timeout
            if (millis() - lastPacket > CONNECTION_TIMEOUT) {
                startDisconnectMode();
            }
            break;
            
        case MASTER_BTN_PRESSED:
            // LED1 ON + LED4 ON, LED2 OFF (enforced)
            setSystem8LEDs(true, false, false, true, false);
            
            // Send heartbeat
            if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
                lastHeartbeat = millis();
                sendToSlew(MSG_HEARTBEAT, 0);
            }
            
            // Check connection timeout
            if (millis() - lastPacket > CONNECTION_TIMEOUT) {
                startDisconnectMode();
            }
            break;
            
        case MASTER_BTN_RELEASED:
            sequenceIndication();
            
            // Send heartbeat
            if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
                lastHeartbeat = millis();
                sendToSlew(MSG_HEARTBEAT, 0);
            }
            
            // Check connection timeout
            if (millis() - lastPacket > CONNECTION_TIMEOUT) {
                startDisconnectMode();
            }
            break;
            
        case MASTER_DISCONNECTED:
            disconnectIndication();
            
            // Send hello to try reconnection
            if (millis() - lastHello >= HELLO_INTERVAL) {
                lastHello = millis();
                sendToSlew(MSG_HELLO, 0);
            }
            break;
    }
}

void systemEightReset(void) {
    masterState = MASTER_SEARCHING;
    slewButtonPressed = false;
    searchLED1 = true;
    lastSearchBlink = millis();
    
    // Reset all LEDs
    setSystem8LEDs(false, false, false, false, false);
    
    Serial.println("SYSTEM 8: Reset - Searching for Slew");
}

// ===== DIP Switch Reading Functions =====

void readTimerDIPSwitch(void) {
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

void readSystemDIPSwitch(void) {
    int value = 0;
    
    if (digitalRead(D1) == LOW) value += 1;
    if (digitalRead(D2) == LOW) value += 2;
    if (digitalRead(D3) == LOW) value += 4;
    if (digitalRead(D4) == LOW) value += 8;
    
    switch(value) {
        case 0: current_system = SYSTEM_DEFAULT; break;
        case 1: current_system = SYSTEM_ONE; break;
        case 2: current_system = SYSTEM_TWO; break;
        case 3: current_system = SYSTEM_THREE; break;
        case 4: current_system = SYSTEM_FOUR; break;
        case 5: current_system = SYSTEM_FIVE; break;
        case 6: current_system = SYSTEM_SIX; break;
        case 7: current_system = SYSTEM_SEVEN; break;
        case 8: current_system = SYSTEM_EIGHT; break;
        default: current_system = SYSTEM_DEFAULT; break;
    }
    
    if (current_system != last_system) {
        last_system = current_system;
        timer_active = false;
        active_button = 0;
        system4_active = false;
        seq_state = SEQ_IDLE;
        system7_active = false;
        system7_combination = 0;
        
        // Reset System 8
        if (current_system == SYSTEM_EIGHT) {
            systemEightReset();
        }
        
        turnOffAllLEDs();
        
        const char* system_names[] = {"DEFAULT", "SYSTEM 1", "SYSTEM 2", "SYSTEM 3", 
                                      "SYSTEM 4", "SYSTEM 5", "SYSTEM 6", "SYSTEM 7", 
                                      "SYSTEM 8 (ESP-NOW)"};
        Serial.printf("System changed to: %s (DIP: %d)\n", system_names[current_system], value);
        
        if (current_system == SYSTEM_FOUR) {
            systemFourStart();
        }
    }
}

// ===== Debounce Function =====

bool debounce(void) {
    unsigned long current_time = millis();
    if (current_time - last_debounce_time > debounce_delay_ms) {
        last_debounce_time = current_time;
        return true;
    }
    return false;
}

// ===== Check Combination Press =====

bool checkCombinationPress(bool b1, bool b2, bool b3, bool b4) {
    if (b1 && b2 && b3 && b4) {
        return true;
    }
    return false;
}

// ===== Reset System =====

void resetSystem(void) {
    turnOffAllLEDs();
    timer_active = false;
    active_button = 0;
    default_blink_state = false;
    default_blink_timer = millis();
    system4_active = false;
    seq_state = SEQ_IDLE;
    system7_active = false;
    system7_combination = 0;
    
    // Reset System 8 if active
    if (espNowInitialized) {
        systemEightReset();
    }
    
    Serial.println("System Reset");
}

// ===== Setup Function =====

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== INDUSTRIAL CONTROL SYSTEM v3.0 (Arduino) ===");
    Serial.println("System Starting...");
    
    // Configure Button pins with internal pull-up
    pinMode(B1, INPUT_PULLUP);
    pinMode(B2, INPUT_PULLUP);
    pinMode(B3, INPUT_PULLUP);
    pinMode(B4, INPUT_PULLUP);
    
    // Configure LED pins
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(L3, OUTPUT);
    pinMode(L4, OUTPUT);
    pinMode(L5, OUTPUT);
    
    // Configure Timer DIP Switch pins
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);
    pinMode(SW5, INPUT_PULLUP);
    pinMode(SW6, INPUT_PULLUP);
    pinMode(SW7, INPUT_PULLUP);
    pinMode(SW8, INPUT_PULLUP);
    
    // Configure System Switching DIP Switch pins
    pinMode(D1, INPUT_PULLUP);
    pinMode(D2, INPUT_PULLUP);
    pinMode(D3, INPUT_PULLUP);
    pinMode(D4, INPUT_PULLUP);
    
    // Initial system read
    readSystemDIPSwitch();
    readTimerDIPSwitch();
    
    turnOffAllLEDs();
    default_blink_timer = millis();
    
    Serial.println("System Ready! Running 24/7 Industrial Mode");
    Serial.printf("Current System: %d\n", current_system);
    Serial.printf("Timer Value: %d seconds\n", timer_value);
    
    if (current_system == SYSTEM_FOUR) {
        systemFourStart();
    }
}

// ===== Main Loop =====

void loop() {
    readSystemDIPSwitch();
    readTimerDIPSwitch();
    
    // ===== Handle System 8 ESP-NOW =====
    if (current_system == SYSTEM_EIGHT) {
        systemEightTask();
        checkMasterButton1();
        delay(10);
        return;
    }
    
    // ===== Handle System 4 Auto-running =====
    if (current_system == SYSTEM_FOUR) {
        static int last_timer_value = 0;
        if (timer_value != last_timer_value) {
            last_timer_value = timer_value;
            if (system4_active) {
                seq_step_duration = (unsigned long)timer_value * 1000;
                Serial.printf("SYSTEM 4: Timer updated to %d seconds\n", timer_value);
            }
        }
        
        if (!system4_active) {
            systemFourStart();
        } else {
            systemFourTask();
        }
        
        delay(10);
        return;
    }
    
    // ===== Handle Default System =====
    if (current_system == SYSTEM_DEFAULT) {
        systemDefaultTask();
        
        if (timer_active) {
            timer_active = false;
            active_button = 0;
            system4_active = false;
            system7_active = false;
        }
        
        delay(10);
        return;
    }
    
    // ===== Handle Timer Expiration =====
    if (timer_active) {
        unsigned long current_time = millis();
        if (current_time - timer_start >= timer_duration) {
            turnOffAllLEDs();
            timer_active = false;
            active_button = 0;
            system7_active = false;
            system7_combination = 0;
            Serial.println("Timer expired - LEDs turned off");
        }
    }
    
    // ===== Read Button States =====
    bool current_b1 = digitalRead(B1) == LOW;
    bool current_b2 = digitalRead(B2) == LOW;
    bool current_b3 = digitalRead(B3) == LOW;
    bool current_b4 = digitalRead(B4) == LOW;
    
    // ===== SYSTEM 6: 4 AND Manner =====
    if (current_system == SYSTEM_SIX && !timer_active) {
        if (checkCombinationPress(current_b1, current_b2, current_b3, current_b4)) {
            if (debounce()) {
                systemSixTask();
            }
        }
    }
    
    // ===== SYSTEM 7: 2 AND Manner (NO LOCK) =====
    if (current_system == SYSTEM_SEVEN) {
        // Check for B1+B2 combination (exactly these two)
        if (current_b1 && current_b2 && !current_b3 && !current_b4) {
            if (debounce()) {
                system7_combination = 1;
                systemSevenTask();
            }
        }
        // Check for B3+B4 combination (exactly these two)
        else if (current_b3 && current_b4 && !current_b1 && !current_b2) {
            if (debounce()) {
                system7_combination = 2;
                systemSevenTask();
            }
        }
    }
    
    // ===== Process Individual Button Presses =====
    if (!timer_active && current_system != SYSTEM_FOUR && 
        current_system != SYSTEM_SIX && current_system != SYSTEM_SEVEN &&
        current_system != SYSTEM_EIGHT) {
        
        // B1 pressed
        if (current_b1 && !last_b1_state) {
            if (debounce()) {
                switch(current_system) {
                    case SYSTEM_ONE: systemOneTask(1); break;
                    case SYSTEM_TWO: systemTwoTask(1); break;
                    case SYSTEM_THREE: systemThreeTask(1); break;
                    case SYSTEM_FIVE: systemFiveTask(1); break;
                    default: break;
                }
            }
        }
        // B2 pressed
        else if (current_b2 && !last_b2_state) {
            if (debounce()) {
                switch(current_system) {
                    case SYSTEM_ONE: systemOneTask(2); break;
                    case SYSTEM_TWO: systemTwoTask(2); break;
                    case SYSTEM_THREE: systemThreeTask(2); break;
                    case SYSTEM_FIVE: systemFiveTask(2); break;
                    default: break;
                }
            }
        }
        // B3 pressed
        else if (current_b3 && !last_b3_state) {
            if (debounce()) {
                switch(current_system) {
                    case SYSTEM_ONE: systemOneTask(3); break;
                    case SYSTEM_TWO: systemTwoTask(3); break;
                    case SYSTEM_THREE: systemThreeTask(3); break;
                    case SYSTEM_FIVE: systemFiveTask(3); break;
                    default: break;
                }
            }
        }
        // B4 pressed
        else if (current_b4 && !last_b4_state) {
            if (debounce()) {
                switch(current_system) {
                    case SYSTEM_ONE: systemOneTask(4); break;
                    case SYSTEM_TWO: systemTwoTask(4); break;
                    case SYSTEM_THREE: systemThreeTask(4); break;
                    case SYSTEM_FIVE: systemFiveTask(4); break;
                    default: break;
                }
            }
        }
    }
    
    // ===== Button Blocking Logic =====
    if (timer_active) {
        switch(current_system) {
            case SYSTEM_ONE:
                if (active_button == 1 || active_button == 2) {
                    if (current_b3 || current_b4) {
                        Serial.printf("SYSTEM 1: Button B%d blocked during B%d timer\n", 
                                    current_b3 ? 3 : 4, active_button);
                    }
                } else if (active_button == 3 || active_button == 4) {
                    if (current_b1 || current_b2) {
                        Serial.printf("SYSTEM 1: Button B%d blocked during B%d timer\n", 
                                    current_b1 ? 1 : 2, active_button);
                    }
                }
                break;
                
            case SYSTEM_TWO:
                if (active_button == 1 && (current_b2 || current_b3 || current_b4)) {
                    Serial.println("SYSTEM 2: Buttons blocked during B1 timer");
                } else if (active_button == 2 && (current_b1 || current_b3 || current_b4)) {
                    Serial.println("SYSTEM 2: Buttons blocked during B2 timer");
                } else if (active_button == 3 && (current_b1 || current_b2 || current_b4)) {
                    Serial.println("SYSTEM 2: Buttons blocked during B3 timer");
                } else if (active_button == 4 && (current_b1 || current_b2 || current_b3)) {
                    Serial.println("SYSTEM 2: Buttons blocked during B4 timer");
                }
                break;
                
            case SYSTEM_THREE:
                // No blocking in System 3
                break;
                
            case SYSTEM_FIVE:
                if (current_b1 || current_b2 || current_b3 || current_b4) {
                    Serial.println("SYSTEM 5: All buttons locked during timer");
                }
                break;
                
            case SYSTEM_SIX:
                if (current_b1 || current_b2 || current_b3 || current_b4) {
                    Serial.println("SYSTEM 6: All buttons locked during timer");
                }
                break;
                
            case SYSTEM_SEVEN:
                // NO LOCKING - All buttons allowed during timer
                if (current_b1 || current_b2 || current_b3 || current_b4) {
                    Serial.println("SYSTEM 7: Button press during timer (allowed - no lock)");
                }
                break;
                
            case SYSTEM_EIGHT:
                // System 8 doesn't use local buttons for blocking
                break;
                
            default:
                break;
        }
    }
    
    // Update last states
    last_b1_state = current_b1;
    last_b2_state = current_b2;
    last_b3_state = current_b3;
    last_b4_state = current_b4;
    
    delay(10);
}


//impliment wheelchock
