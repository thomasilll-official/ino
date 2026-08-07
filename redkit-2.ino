#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

// ===== GPIO Definitions =====

// Button GPIOs
#define B1 GPIO_NUM_4
#define B2 GPIO_NUM_5
#define B3 GPIO_NUM_6
#define B4 GPIO_NUM_7

// LED GPIOs
#define L1 GPIO_NUM_8
#define L2 GPIO_NUM_9
#define L3 GPIO_NUM_10
#define L4 GPIO_NUM_11
#define L5 GPIO_NUM_12

// Timer DIP Switch GPIOs (8-bit)
#define SW1 GPIO_NUM_13  // 1
#define SW2 GPIO_NUM_14  // 2
#define SW3 GPIO_NUM_15  // 4
#define SW4 GPIO_NUM_16  // 8
#define SW5 GPIO_NUM_17  // 16
#define SW6 GPIO_NUM_18  // 32
#define SW7 GPIO_NUM_21  // 64
#define SW8 GPIO_NUM_38  // 128

// System Switching DIP Switch (4-bit)
#define D1 GPIO_NUM_39  // 1
#define D2 GPIO_NUM_40  // 2
#define D3 GPIO_NUM_41  // 4
#define D4 GPIO_NUM_42  // 8

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
static const char *TAG = "INDUSTRIAL_CONTROL";
static int64_t timer_start = 0;
static int64_t timer_duration = 0;
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
static int64_t last_debounce_time = 0;
static const int debounce_delay_ms = 50;

// Default system blink control
static bool default_blink_state = false;
static int64_t default_blink_timer = 0;
static const int default_blink_interval_ms = 1000;

// System 4 variables (Auto-running sequential pattern)
static bool system4_active = false;
static sequence_state_t seq_state = SEQ_IDLE;
static int64_t seq_step_start = 0;
static int64_t seq_step_duration = 0;
static int current_led_index = 0; // 0=L1, 1=L2, 2=L3, 3=L4

// System 7 (2 AND manner) variables - UPDATED
static bool system7_active = false;
static int system7_combination = 0; // 1 = B1+B2, 2 = B3+B4

// Watchdog timer
static const int WATCHDOG_TIMEOUT_MS = 10000;

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

// ===== LED Control Functions =====

void turnOffAllLEDs(void) {
    gpio_set_level(L1, 0);
    gpio_set_level(L2, 0);
    gpio_set_level(L3, 0);
    gpio_set_level(L4, 0);
    gpio_set_level(L5, 0);
}

void turnOnAllLEDs(void) {
    gpio_set_level(L1, 1);
    gpio_set_level(L2, 1);
    gpio_set_level(L3, 1);
    gpio_set_level(L4, 1);
    gpio_set_level(L5, 1);
}

void setLEDs(uint8_t l1, uint8_t l2, uint8_t l3, uint8_t l4, uint8_t l5) {
    gpio_set_level(L1, l1);
    gpio_set_level(L2, l2);
    gpio_set_level(L3, l3);
    gpio_set_level(L4, l4);
    gpio_set_level(L5, l5);
}

// ===== System Handlers =====

void systemDefaultTask(void) {
    // Blink all LEDs every second
    int64_t current_time = esp_timer_get_time() / 1000;
    if (current_time - default_blink_timer >= default_blink_interval_ms) {
        default_blink_timer = current_time;
        default_blink_state = !default_blink_state;
        
        if (default_blink_state) {
            turnOnAllLEDs();
            ESP_LOGD(TAG, "Default System: LEDs ON");
        } else {
            turnOffAllLEDs();
            ESP_LOGD(TAG, "Default System: LEDs OFF");
        }
    }
}

void systemOneTask(int button) {
    // System 1: Two-way with two projectors
    timer_duration = (int64_t)timer_value * 1000000;
    timer_start = esp_timer_get_time();
    timer_active = true;
    active_button = button;
    
    turnOffAllLEDs();
    
    switch(button) {
        case 1:
            setLEDs(0, 0, 1, 0, 1);
            ESP_LOGI(TAG, "SYSTEM 1: B1 → L3+L5, %d seconds", timer_value);
            break;
        case 2:
            setLEDs(0, 0, 0, 1, 1);
            ESP_LOGI(TAG, "SYSTEM 1: B2 → L4+L5, %d seconds", timer_value);
            break;
        case 3:
            setLEDs(1, 0, 0, 0, 1);
            ESP_LOGI(TAG, "SYSTEM 1: B3 → L1+L5, %d seconds", timer_value);
            break;
        case 4:
            setLEDs(0, 1, 0, 0, 1);
            ESP_LOGI(TAG, "SYSTEM 1: B4 → L2+L5, %d seconds", timer_value);
            break;
    }
}

void systemTwoTask(int button) {
    // System 2: Signal Pro
    timer_duration = (int64_t)timer_value * 1000000;
    timer_start = esp_timer_get_time();
    timer_active = true;
    active_button = button;
    
    turnOffAllLEDs();
    
    switch(button) {
        case 1:
            setLEDs(0, 1, 1, 1, 1);
            ESP_LOGI(TAG, "SYSTEM 2: B1 → L2,L3,L4,L5, %d seconds", timer_value);
            break;
        case 2:
            setLEDs(1, 0, 1, 1, 1);
            ESP_LOGI(TAG, "SYSTEM 2: B2 → L1,L3,L4,L5, %d seconds", timer_value);
            break;
        case 3:
            setLEDs(1, 1, 0, 1, 1);
            ESP_LOGI(TAG, "SYSTEM 2: B3 → L1,L2,L4,L5, %d seconds", timer_value);
            break;
        case 4:
            setLEDs(1, 1, 1, 0, 1);
            ESP_LOGI(TAG, "SYSTEM 2: B4 → L1,L2,L3,L5, %d seconds", timer_value);
            break;
    }
}

void systemThreeTask(int button) {
    // System 3: Two shutter with projectors
    timer_duration = (int64_t)timer_value * 1000000;
    timer_start = esp_timer_get_time();
    timer_active = true;
    active_button = button;
    
    turnOffAllLEDs();
    
    switch(button) {
        case 1:
            setLEDs(1, 0, 0, 0, 1);
            ESP_LOGI(TAG, "SYSTEM 3: B1 → L1+L5, %d seconds", timer_value);
            break;
        case 2:
            setLEDs(0, 1, 0, 0, 1);
            ESP_LOGI(TAG, "SYSTEM 3: B2 → L2+L5, %d seconds", timer_value);
            break;
        case 3:
            setLEDs(0, 0, 1, 0, 1);
            ESP_LOGI(TAG, "SYSTEM 3: B3 → L3+L5, %d seconds", timer_value);
            break;
        case 4:
            setLEDs(0, 0, 0, 1, 1);
            ESP_LOGI(TAG, "SYSTEM 3: B4 → L4+L5, %d seconds", timer_value);
            break;
    }
}

void systemFourStart(void) {
    // System 4: Auto-running sequential pattern - L1+L5, L2+L5, L3+L5, L4+L5
    seq_step_duration = (int64_t)timer_value * 1000000;
    system4_active = true;
    seq_state = SEQ_L1_L5;
    current_led_index = 0;
    seq_step_start = esp_timer_get_time();
    
    turnOffAllLEDs();
    setLEDs(1, 0, 0, 0, 1);
    
    ESP_LOGI(TAG, "SYSTEM 4: Auto-running sequential pattern started");
    ESP_LOGI(TAG, "SYSTEM 4: Each step duration = %d seconds", timer_value);
    ESP_LOGI(TAG, "SYSTEM 4: Pattern: L1+L5 → L2+L5 → L3+L5 → L4+L5 (looping)");
}

void systemFourTask(void) {
    if (!system4_active) return;
    
    int64_t current_time = esp_timer_get_time();
    
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
                ESP_LOGD(TAG, "SYSTEM 4: L1+L5 ON");
                break;
            case 1:
                setLEDs(0, 1, 0, 0, 1);
                seq_state = SEQ_L2_L5;
                ESP_LOGD(TAG, "SYSTEM 4: L2+L5 ON");
                break;
            case 2:
                setLEDs(0, 0, 1, 0, 1);
                seq_state = SEQ_L3_L5;
                ESP_LOGD(TAG, "SYSTEM 4: L3+L5 ON");
                break;
            case 3:
                setLEDs(0, 0, 0, 1, 1);
                seq_state = SEQ_L4_L5;
                ESP_LOGD(TAG, "SYSTEM 4: L4+L5 ON");
                break;
            default:
                break;
        }
    }
}

void systemFiveTask(int button) {
    // System 5: OR manner - Any button turns on all LEDs
    timer_duration = (int64_t)timer_value * 1000000;
    timer_start = esp_timer_get_time();
    timer_active = true;
    active_button = button;
    
    turnOffAllLEDs();
    turnOnAllLEDs();
    
    ESP_LOGI(TAG, "SYSTEM 5: B%d → ALL LEDs ON, %d seconds", button, timer_value);
    ESP_LOGI(TAG, "SYSTEM 5: All other buttons locked during timer");
}

void systemSixTask(void) {
    // System 6: 4 AND manner - All 4 buttons must be pressed together
    timer_duration = (int64_t)timer_value * 1000000;
    timer_start = esp_timer_get_time();
    timer_active = true;
    active_button = 5;
    
    turnOffAllLEDs();
    turnOnAllLEDs();
    
    ESP_LOGI(TAG, "SYSTEM 6: B1+B2+B3+B4 → ALL LEDs ON, %d seconds", timer_value);
    ESP_LOGI(TAG, "SYSTEM 6: All buttons locked during timer");
}

// ===== UPDATED SYSTEM 7 =====
void systemSevenTask(void) {
    // System 7: 2 AND manner - B1+B2 or B3+B4 with timer ONLY (NO LOCK)
    timer_duration = (int64_t)timer_value * 1000000;
    timer_start = esp_timer_get_time();
    timer_active = true;
    active_button = 6;
    
    if (system7_combination == 1) {
        // B1+B2 → L1,L2,L5
        setLEDs(1, 1, 0, 0, 1);
        ESP_LOGI(TAG, "SYSTEM 7: B1+B2 → L1,L2,L5, %d seconds (NO LOCK)", timer_value);
    } else if (system7_combination == 2) {
        // B3+B4 → L3,L4,L5
        setLEDs(0, 0, 1, 1, 1);
        ESP_LOGI(TAG, "SYSTEM 7: B3+B4 → L3,L4,L5, %d seconds (NO LOCK)", timer_value);
    }
    
    ESP_LOGI(TAG, "SYSTEM 7: Timer active - Other buttons are NOT locked");
}
// ===== END OF UPDATED SYSTEM 7 =====

// ===== DIP Switch Reading Functions =====

void readTimerDIPSwitch(void) {
    int value = 0;
    
    if (gpio_get_level(SW1) == 0) value += 1;
    if (gpio_get_level(SW2) == 0) value += 2;
    if (gpio_get_level(SW3) == 0) value += 4;
    if (gpio_get_level(SW4) == 0) value += 8;
    if (gpio_get_level(SW5) == 0) value += 16;
    if (gpio_get_level(SW6) == 0) value += 32;
    if (gpio_get_level(SW7) == 0) value += 64;
    if (gpio_get_level(SW8) == 0) value += 128;
    
    if (value == 0) value = 1;
    
    if (value != timer_value) {
        timer_value = value;
        ESP_LOGI(TAG, "Timer set to %d seconds", timer_value);
    }
}

void readSystemDIPSwitch(void) {
    int value = 0;
    
    if (gpio_get_level(D1) == 0) value += 1;
    if (gpio_get_level(D2) == 0) value += 2;
    if (gpio_get_level(D3) == 0) value += 4;
    if (gpio_get_level(D4) == 0) value += 8;
    
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
        timer_active = false;
        active_button = 0;
        system4_active = false;
        seq_state = SEQ_IDLE;
        system7_active = false;
        system7_combination = 0;
        turnOffAllLEDs();
        
        const char* system_names[] = {"DEFAULT", "SYSTEM 1", "SYSTEM 2", "SYSTEM 3", 
                                      "SYSTEM 4", "SYSTEM 5", "SYSTEM 6", "SYSTEM 7"};
        ESP_LOGI(TAG, "System changed to: %s (DIP: %d)", system_names[current_system], value);
        
        if (current_system == SYSTEM_FOUR) {
            systemFourStart();
        }
    }
}

// ===== Debounce Function =====

bool debounce(void) {
    int64_t current_time = esp_timer_get_time() / 1000;
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
    default_blink_timer = esp_timer_get_time() / 1000;
    system4_active = false;
    seq_state = SEQ_IDLE;
    system7_active = false;
    system7_combination = 0;
    ESP_LOGI(TAG, "System Reset");
}// ===== Main Application =====

void app_main(void) {
    ESP_LOGI(TAG, "=== INDUSTRIAL CONTROL SYSTEM v3.0 ===");
    ESP_LOGI(TAG, "System Starting...");
    
    // Initialize Watchdog
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_MS,
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    
    // Configure Button pins with internal pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << B1) | (1ULL << B2) | (1ULL << B3) | (1ULL << B4),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Configure LED pins
    io_conf = (gpio_config_t){
        .pin_bit_mask = (1ULL << L1) | (1ULL << L2) | (1ULL << L3) | (1ULL << L4) | (1ULL << L5),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Configure Timer DIP Switch pins
    io_conf = (gpio_config_t){
        .pin_bit_mask = (1ULL << SW1) | (1ULL << SW2) | (1ULL << SW3) | (1ULL << SW4) | 
                        (1ULL << SW5) | (1ULL << SW6) | (1ULL << SW7) | (1ULL << SW8),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Configure System Switching DIP Switch pins
    io_conf = (gpio_config_t){
        .pin_bit_mask = (1ULL << D1) | (1ULL << D2) | (1ULL << D3) | (1ULL << D4),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Initial system read
    readSystemDIPSwitch();
    readTimerDIPSwitch();
    
    turnOffAllLEDs();
    default_blink_timer = esp_timer_get_time() / 1000;
    
    ESP_LOGI(TAG, "System Ready! Running 24/7 Industrial Mode");
    ESP_LOGI(TAG, "Current System: %d", current_system);
    ESP_LOGI(TAG, "Timer Value: %d seconds", timer_value);
    
    if (current_system == SYSTEM_FOUR) {
        systemFourStart();
    }
    
    // Main loop
    while (1) {
        esp_task_wdt_reset();
        
        readSystemDIPSwitch();
        readTimerDIPSwitch();
        
        // ===== Handle System 4 Auto-running =====
        if (current_system == SYSTEM_FOUR) {
            static int last_timer_value = 0;
            if (timer_value != last_timer_value) {
                last_timer_value = timer_value;
                if (system4_active) {
                    seq_step_duration = (int64_t)timer_value * 1000000;
                    ESP_LOGI(TAG, "SYSTEM 4: Timer updated to %d seconds", timer_value);
                }
            }
            
            if (!system4_active) {
                systemFourStart();
            } else {
                systemFourTask();
            }
            
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
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
            
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // ===== Handle Timer Expiration =====
        if (timer_active) {
            int64_t current_time = esp_timer_get_time();
            if (current_time - timer_start >= timer_duration) {
                turnOffAllLEDs();
                timer_active = false;
                active_button = 0;
                system7_active = false;
                system7_combination = 0;
                ESP_LOGI(TAG, "Timer expired - LEDs turned off");
            }
        }
        
        // ===== Read Button States =====
        bool current_b1 = gpio_get_level(B1) == 0;
        bool current_b2 = gpio_get_level(B2) == 0;
        bool current_b3 = gpio_get_level(B3) == 0;
        bool current_b4 = gpio_get_level(B4) == 0;
        
        // ===== SYSTEM 6: 4 AND Manner =====
        if (current_system == SYSTEM_SIX && !timer_active) {
            if (checkCombinationPress(current_b1, current_b2, current_b3, current_b4)) {
                if (debounce()) {
                    systemSixTask();
                }
            }
        }
        
        // ===== UPDATED SYSTEM 7: 2 AND Manner (NO LOCK) =====
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
        // ===== END OF UPDATED SYSTEM 7 =====
        
        // ===== Process Individual Button Presses =====
        if (!timer_active && current_system != SYSTEM_FOUR && 
            current_system != SYSTEM_SIX && current_system != SYSTEM_SEVEN) {
            
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
                            ESP_LOGW(TAG, "SYSTEM 1: Button B%d blocked during B%d timer", 
                                    current_b3 ? 3 : 4, active_button);
                        }
                    } else if (active_button == 3 || active_button == 4) {
                        if (current_b1 || current_b2) {
                            ESP_LOGW(TAG, "SYSTEM 1: Button B%d blocked during B%d timer", 
                                    current_b1 ? 1 : 2, active_button);
                        }
                    }
                    break;
                    
                case SYSTEM_TWO:
                    if (active_button == 1 && (current_b2 || current_b3 || current_b4)) {
                        ESP_LOGW(TAG, "SYSTEM 2: Buttons blocked during B1 timer");
                    } else if (active_button == 2 && (current_b1 || current_b3 || current_b4)) {
                        ESP_LOGW(TAG, "SYSTEM 2: Buttons blocked during B2 timer");
                    } else if (active_button == 3 && (current_b1 || current_b2 || current_b4)) {
                        ESP_LOGW(TAG, "SYSTEM 2: Buttons blocked during B3 timer");
                    } else if (active_button == 4 && (current_b1 || current_b2 || current_b3)) {
                        ESP_LOGW(TAG, "SYSTEM 2: Buttons blocked during B4 timer");
                    }
                    break;
                    
                case SYSTEM_THREE:
                    // No blocking in System 3
                    break;
                    
                case SYSTEM_FIVE:
                    if (current_b1 || current_b2 || current_b3 || current_b4) {
                        ESP_LOGW(TAG, "SYSTEM 5: All buttons locked during timer");
                    }
                    break;
                    
                case SYSTEM_SIX:
                    if (current_b1 || current_b2 || current_b3 || current_b4) {
                        ESP_LOGW(TAG, "SYSTEM 6: All buttons locked during timer");
                    }
                    break;
                    
                // ===== UPDATED SYSTEM 7: NO LOCK =====
                case SYSTEM_SEVEN:
                    // NO LOCKING - All buttons allowed during timer
                    if (current_b1 || current_b2 || current_b3 || current_b4) {
                        ESP_LOGD(TAG, "SYSTEM 7: Button press during timer (allowed - no lock)");
                    }
                    break;
                // ===== END OF UPDATED SYSTEM 7 =====
                    
                default:
                    break;
            }
        }
        
        // Update last states
        last_b1_state = current_b1;
        last_b2_state = current_b2;
        last_b3_state = current_b3;
        last_b4_state = current_b4;
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}






//DIP Value	System	Description	Lock Behavior
//0000	Default	All LEDs blink ON/OFF every 1 second	N/A
//0001	System 1	Two-way with two projectors	B1/B2 blocks B3/B4, B3/B4 blocks B1/B2
//0010	System 2	Signal Pro	All other buttons blocked during timer
//0011	System 3	Two shutter with projectors	NO blocking
//0100	System 4	Auto-running sequential (L1+L5→L2+L5→L3+L5→L4+L5)	Buttons ignored
//0101	System 5	OR manner - Any button → ALL LEDs ON	All buttons locked during timer
//0110	System 6	4 AND manner - All 4 buttons → ALL LEDs ON	All buttons locked during timer
//0111	System 7	2 AND manner - B1+B2 or B3+B4	NO LOCK - All buttons allowed






//for ino code ----


#include <Arduino.h>

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
        current_system != SYSTEM_SIX && current_system != SYSTEM_SEVEN) {
        
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
