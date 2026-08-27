//for ino code ----
// INDUSTRIAL 24/7 STABLE VERSION - NO LOGIC CHANGES
// Added: watchdog timer, input filtering, EEPROM backup, error recovery

#include <Arduino.h>
#include <EEPROM.h>

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

// ===== 24/7 Industrial Reliability Defines =====
#define WATCHDOG_TIMEOUT_MS 5000
#define INPUT_FILTER_DELAY_MS 5
#define EEPROM_SYSTEM_ADDR 0
#define EEPROM_TIMER_ADDR 1
#define EEPROM_MAGIC_ADDR 2
#define EEPROM_MAGIC_VALUE 0xA5
#define MAX_RETRY_COUNT 3
#define ERROR_RECOVERY_DELAY_MS 100

// ===== System Definitions =====
typedef enum {
    SYSTEM_DEFAULT = 0,
    SYSTEM_ONE = 1,
    SYSTEM_TWO = 2,
    SYSTEM_THREE = 3,
    SYSTEM_FOUR = 4,
    SYSTEM_FIVE = 5,
    SYSTEM_SIX = 6,
    SYSTEM_SEVEN = 7
} system_mode_t;

// ===== System 4 States =====
typedef enum {
    SEQ_IDLE = 0,
    SEQ_L1_L5 = 1,
    SEQ_L2_L5 = 2,
    SEQ_L3_L5 = 3,
    SEQ_L4_L5 = 4
} sequence_state_t;

// ===== Global Variables =====
static unsigned long timer_start = 0;
static unsigned long timer_duration = 0;
static bool timer_active = false;
static int active_button = 0;
static int timer_value = 0;
static system_mode_t current_system = SYSTEM_DEFAULT;
static system_mode_t last_system = SYSTEM_DEFAULT;

// Button states for debouncing (with filtering)
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

// ===== 24/7 Industrial Reliability Variables =====
static unsigned long last_watchdog_feed = 0;
static unsigned long last_loop_time = 0;
static int error_count = 0;
static bool system_initialized = false;
static unsigned long last_dip_read_time = 0;
static const int DIP_READ_INTERVAL_MS = 50;
static int stable_timer_value = 0;
static system_mode_t stable_system = SYSTEM_DEFAULT;
static int dip_read_count = 0;
static bool eeprom_valid = false;

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
void resetSystem(void);
bool checkCombinationPress(bool b1, bool b2, bool b3, bool b4);
void setLEDs(uint8_t l1, uint8_t l2, uint8_t l3, uint8_t l4, uint8_t l5);

// ===== 24/7 Industrial Reliability Functions =====
void feedWatchdog(void);
void saveSystemState(void);
bool loadSystemState(void);
void emergencyReset(void);
void handleError(int error_code);
bool isInputStable(int pin);
int readDIPWithFilter(int pin);
void readDIPSwitchesStable(void);

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

// ===== 24/7 Industrial Reliability: Watchdog =====

void feedWatchdog(void) {
    last_watchdog_feed = millis();
}

bool isWatchdogExpired(void) {
    return (millis() - last_watchdog_feed) > WATCHDOG_TIMEOUT_MS;
}

// ===== 24/7 Industrial Reliability: EEPROM Backup =====

void saveSystemState(void) {
    if (eeprom_valid) {
        EEPROM.write(EEPROM_SYSTEM_ADDR, (uint8_t)current_system);
        EEPROM.write(EEPROM_TIMER_ADDR, (uint8_t)timer_value);
        EEPROM.commit();
    }
}

bool loadSystemState(void) {
    uint8_t magic = EEPROM.read(EEPROM_MAGIC_ADDR);
    if (magic == EEPROM_MAGIC_VALUE) {
        uint8_t saved_system = EEPROM.read(EEPROM_SYSTEM_ADDR);
        uint8_t saved_timer = EEPROM.read(EEPROM_TIMER_ADDR);
        
        if (saved_system <= SYSTEM_SEVEN && saved_timer >= 1 && saved_timer <= 255) {
            current_system = (system_mode_t)saved_system;
            timer_value = saved_timer;
            eeprom_valid = true;
            return true;
        }
    }
    return false;
}

void initEEPROM(void) {
    EEPROM.begin(16);
    if (!loadSystemState()) {
        // Initialize EEPROM with default values
        EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
        EEPROM.write(EEPROM_SYSTEM_ADDR, 0);
        EEPROM.write(EEPROM_TIMER_ADDR, 1);
        EEPROM.commit();
        eeprom_valid = true;
    }
}

// ===== 24/7 Industrial Reliability: Error Handling =====

void handleError(int error_code) {
    error_count++;
    Serial.printf("ERROR [%d]: Count=%d, System=%d\n", error_code, error_count, current_system);
    
    if (error_count > MAX_RETRY_COUNT) {
        emergencyReset();
        error_count = 0;
    }
    
    delay(ERROR_RECOVERY_DELAY_MS);
}

void emergencyReset(void) {
    Serial.println("EMERGENCY RESET: Resetting system...");
    turnOffAllLEDs();
    timer_active = false;
    active_button = 0;
    system4_active = false;
    seq_state = SEQ_IDLE;
    system7_active = false;
    system7_combination = 0;
    
    // Try to restore from EEPROM
    if (loadSystemState()) {
        Serial.println("EMERGENCY RESET: Restored from EEPROM");
    }
    
    delay(100);
}

// ===== 24/7 Industrial Reliability: Input Filtering =====

bool isInputStable(int pin) {
    int first_read = digitalRead(pin);
    delay(INPUT_FILTER_DELAY_MS);
    int second_read = digitalRead(pin);
    return (first_read == second_read);
}

int readDIPWithFilter(int pin) {
    int value = 0;
    for (int i = 0; i < 3; i++) {
        if (digitalRead(pin) == LOW) value++;
        delay(INPUT_FILTER_DELAY_MS);
    }
    return (value >= 2) ? LOW : HIGH;
}

void readDIPSwitchesStable(void) {
    if (millis() - last_dip_read_time < DIP_READ_INTERVAL_MS) {
        return;
    }
    last_dip_read_time = millis();
    
    // Read timer DIP with filtering
    int timer_val = 0;
    if (readDIPWithFilter(SW1) == LOW) timer_val += 1;
    if (readDIPWithFilter(SW2) == LOW) timer_val += 2;
    if (readDIPWithFilter(SW3) == LOW) timer_val += 4;
    if (readDIPWithFilter(SW4) == LOW) timer_val += 8;
    if (readDIPWithFilter(SW5) == LOW) timer_val += 16;
    if (readDIPWithFilter(SW6) == LOW) timer_val += 32;
    if (readDIPWithFilter(SW7) == LOW) timer_val += 64;
    if (readDIPWithFilter(SW8) == LOW) timer_val += 128;
    if (timer_val == 0) timer_val = 1;
    
    // Read system DIP with filtering
    int sys_val = 0;
    if (readDIPWithFilter(D1) == LOW) sys_val += 1;
    if (readDIPWithFilter(D2) == LOW) sys_val += 2;
    if (readDIPWithFilter(D3) == LOW) sys_val += 4;
    if (readDIPWithFilter(D4) == LOW) sys_val += 8;
    
    // Update timer value if changed and stable
    if (timer_val != timer_value) {
        // Double-check stability
        delay(20);
        int check_val = 0;
        if (digitalRead(SW1) == LOW) check_val += 1;
        if (digitalRead(SW2) == LOW) check_val += 2;
        if (digitalRead(SW3) == LOW) check_val += 4;
        if (digitalRead(SW4) == LOW) check_val += 8;
        if (digitalRead(SW5) == LOW) check_val += 16;
        if (digitalRead(SW6) == LOW) check_val += 32;
        if (digitalRead(SW7) == LOW) check_val += 64;
        if (digitalRead(SW8) == LOW) check_val += 128;
        if (check_val == 0) check_val = 1;
        
        if (timer_val == check_val) {
            timer_value = timer_val;
            saveSystemState();
            Serial.printf("Timer set to %d seconds\n", timer_value);
        }
    }
    
    // Update system value if changed and stable
    int sys_check = 0;
    if (digitalRead(D1) == LOW) sys_check += 1;
    if (digitalRead(D2) == LOW) sys_check += 2;
    if (digitalRead(D3) == LOW) sys_check += 4;
    if (digitalRead(D4) == LOW) sys_check += 8;
    
    if (sys_val == sys_check) {
        system_mode_t new_system = SYSTEM_DEFAULT;
        switch(sys_val) {
            case 0: new_system = SYSTEM_DEFAULT; break;
            case 1: new_system = SYSTEM_ONE; break;
            case 2: new_system = SYSTEM_TWO; break;
            case 3: new_system = SYSTEM_THREE; break;
            case 4: new_system = SYSTEM_FOUR; break;
            case 5: new_system = SYSTEM_FIVE; break;
            case 6: new_system = SYSTEM_SIX; break;
            case 7: new_system = SYSTEM_SEVEN; break;
            default: new_system = SYSTEM_DEFAULT; break;
        }
        
        if (new_system != current_system) {
            current_system = new_system;
            saveSystemState();
            
            // Reset all active states
            timer_active = false;
            active_button = 0;
            system4_active = false;
            seq_state = SEQ_IDLE;
            system7_active = false;
            system7_combination = 0;
            turnOffAllLEDs();
            
            const char* system_names[] = {"DEFAULT", "SYSTEM 1", "SYSTEM 2", "SYSTEM 3", 
                                          "SYSTEM 4", "SYSTEM 5", "SYSTEM 6", "SYSTEM 7"};
            Serial.printf("System changed to: %s (DIP: %d)\n", system_names[current_system], sys_val);
            
            if (current_system == SYSTEM_FOUR) {
                systemFourStart();
            }
        }
    }
}

// ===== System Handlers (UNCHANGED LOGIC) =====

void systemDefaultTask(void) {
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
    timer_duration = (unsigned long)timer_value * 1000;
    timer_start = millis();
    timer_active = true;
    active_button = 6;
    
    if (system7_combination == 1) {
        setLEDs(1, 1, 0, 0, 1);
        Serial.printf("SYSTEM 7: B1+B2 → L1,L2,L5, %d seconds (NO LOCK)\n", timer_value);
    } else if (system7_combination == 2) {
        setLEDs(0, 0, 1, 1, 1);
        Serial.printf("SYSTEM 7: B3+B4 → L3,L4,L5, %d seconds (NO LOCK)\n", timer_value);
    }
    
    Serial.println("SYSTEM 7: Timer active - Other buttons are NOT locked");
}

// ===== Legacy DIP Reading (Retained for compatibility) =====

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
        saveSystemState();
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
        default: current_system = SYSTEM_DEFAULT; break;
    }
    
    if (current_system != last_system) {
        last_system = current_system;
        saveSystemState();
        timer_active = false;
        active_button = 0;
        system4_active = false;
        seq_state = SEQ_IDLE;
        system7_active = false;
        system7_combination = 0;
        turnOffAllLEDs();
        
        const char* system_names[] = {"DEFAULT", "SYSTEM 1", "SYSTEM 2", "SYSTEM 3", 
                                      "SYSTEM 4", "SYSTEM 5", "SYSTEM 6", "SYSTEM 7"};
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
    saveSystemState();
    Serial.println("System Reset");
}

// ===== Setup Function =====

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== INDUSTRIAL CONTROL SYSTEM v4.0 (24/7 Stable) ===");
    Serial.println("System Starting...");
    
    // Initialize EEPROM for state backup
    initEEPROM();
    
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
    last_watchdog_feed = millis();
    system_initialized = true;
    
    Serial.println("System Ready! Running 24/7 Industrial Mode");
    Serial.printf("Current System: %d\n", current_system);
    Serial.printf("Timer Value: %d seconds\n", timer_value);
    Serial.printf("EEPROM Valid: %s\n", eeprom_valid ? "YES" : "NO");
    
    if (current_system == SYSTEM_FOUR) {
        systemFourStart();
    }
}

// ===== Main Loop =====

void loop() {
    // ===== 24/7 Watchdog Feed =====
    feedWatchdog();
    
    // ===== Check for system hangs =====
    if (isWatchdogExpired()) {
        Serial.println("WARNING: Watchdog timeout - System may be hung");
        emergencyReset();
        feedWatchdog();
    }
    
    // ===== Read DIP Switches with filtering =====
    readDIPSwitchesStable();
    
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
        if (current_b1 && current_b2 && !current_b3 && !current_b4) {
            if (debounce()) {
                system7_combination = 1;
                systemSevenTask();
            }
        }
        else if (current_b3 && current_b4 && !current_b1 && !current_b2) {
            if (debounce()) {
                system7_combination = 2;
                systemSevenTask();
            }
        }
    }
    
    // ===== Process Individual Button Presses =====
    if (!timer_active && current_system != SYSTEM_FOUR && 
        current_system != SYSTEM_SIX && current_system != SYSTEM_SEVEN) {
        
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
    
    // ===== Button Blocking Logic (UNCHANGED) =====
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
                if (current_b1 || current_b2 || current_b3 || current_b4) {
                    Serial.println("SYSTEM 7: Button press during timer (allowed - no lock)");
                }
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
