//IDF
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
    SYSTEM_FOUR = 4
} system_mode_t;

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

// Watchdog timer
static const int WATCHDOG_TIMEOUT_MS = 10000;

// ===== Function Prototypes =====
void turnOffAllLEDs(void);
void turnOnAllLEDs(void);
void handleButtonPress(int button);
void readTimerDIPSwitch(void);
void readSystemDIPSwitch(void);
bool debounce(void);
void systemDefaultTask(void);
void systemOneTask(int button);
void systemTwoTask(int button);
void systemThreeTask(int button);
void resetSystem(void);

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
    // B1 → L3 + L5, Block B3,B4
    // B2 → L4 + L5, Block B3,B4
    // B3 → L1 + L5, Block B1,B2
    // B4 → L2 + L5, Block B1,B2
    
    timer_duration = (int64_t)timer_value * 1000000; // Convert seconds to microseconds
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
    // B1 → L2,L3,L4,L5, Block B2,B3,B4
    // B2 → L1,L3,L4,L5, Block B1,B3,B4
    // B3 → L1,L2,L4,L5, Block B1,B2,B4
    // B4 → L1,L2,L3,L5, Block B1,B2,B3
    
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
    // B1 → L1,L5
    // B2 → L2,L5
    // B3 → L3,L5
    // B4 → L4,L5
    
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
    
    // Map to system modes
    switch(value) {
        case 0:
            current_system = SYSTEM_DEFAULT;
            break;
        case 1:
            current_system = SYSTEM_ONE;
            break;
        case 2:
            current_system = SYSTEM_TWO;
            break;
        case 3:
            current_system = SYSTEM_THREE;
            break;
        case 4:
            current_system = SYSTEM_FOUR;
            break;
        default:
            current_system = SYSTEM_DEFAULT;
            break;
    }
    
    if (current_system != last_system) {
        last_system = current_system;
        // Reset timer when system changes
        timer_active = false;
        active_button = 0;
        turnOffAllLEDs();
        
        const char* system_names[] = {"DEFAULT", "SYSTEM 1", "SYSTEM 2", "SYSTEM 3", "SYSTEM 4"};
        ESP_LOGI(TAG, "System changed to: %s (DIP: %d)", system_names[current_system], value);
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

// ===== Reset System =====

void resetSystem(void) {
    turnOffAllLEDs();
    timer_active = false;
    active_button = 0;
    default_blink_state = false;
    default_blink_timer = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "System Reset");
}

// ===== Main Application =====

void app_main(void) {
    ESP_LOGI(TAG, "=== INDUSTRIAL CONTROL SYSTEM ===");
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
    
    // Turn off all LEDs initially
    turnOffAllLEDs();
    default_blink_timer = esp_timer_get_time() / 1000;
    
    ESP_LOGI(TAG, "System Ready! Running 24/7 Industrial Mode");
    ESP_LOGI(TAG, "Current System: %d", current_system);
    ESP_LOGI(TAG, "Timer Value: %d seconds", timer_value);
    
    // Main loop
    while (1) {
        // Feed watchdog
        esp_task_wdt_reset();
        
        // Read DIP switches
        readSystemDIPSwitch();
        readTimerDIPSwitch();
        
        // Handle default system (blink mode)
        if (current_system == SYSTEM_DEFAULT) {
            systemDefaultTask();
            
            // Reset any active timer
            if (timer_active) {
                timer_active = false;
                active_button = 0;
            }
            
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // Handle timer expiration for other systems
        if (timer_active) {
            int64_t current_time = esp_timer_get_time();
            if (current_time - timer_start >= timer_duration) {
                turnOffAllLEDs();
                timer_active = false;
                active_button = 0;
                ESP_LOGI(TAG, "Timer expired - LEDs turned off");
            }
        }
        
        // Read button states (0 = pressed due to pull-up)
        bool current_b1 = gpio_get_level(B1) == 0;
        bool current_b2 = gpio_get_level(B2) == 0;
        bool current_b3 = gpio_get_level(B3) == 0;
        bool current_b4 = gpio_get_level(B4) == 0;
        
        // Process button presses only if timer not active
        if (!timer_active) {
            // B1 pressed
            if (current_b1 && !last_b1_state) {
                if (debounce()) {
                    switch(current_system) {
                        case SYSTEM_ONE:
                            systemOneTask(1);
                            break;
                        case SYSTEM_TWO:
                            systemTwoTask(1);
                            break;
                        case SYSTEM_THREE:
                            systemThreeTask(1);
                            break;
                        case SYSTEM_FOUR:
                            // System 4 - Add custom logic
                            ESP_LOGI(TAG, "SYSTEM 4: B1 pressed - Custom logic");
                            break;
                        default:
                            break;
                    }
                }
            }
            // B2 pressed
            else if (current_b2 && !last_b2_state) {
                if (debounce()) {
                    switch(current_system) {
                        case SYSTEM_ONE:
                            systemOneTask(2);
                            break;
                        case SYSTEM_TWO:
                            systemTwoTask(2);
                            break;
                        case SYSTEM_THREE:
                            systemThreeTask(2);
                            break;
                        case SYSTEM_FOUR:
                            ESP_LOGI(TAG, "SYSTEM 4: B2 pressed - Custom logic");
                            break;
                        default:
                            break;
                    }
                }
            }
            // B3 pressed
            else if (current_b3 && !last_b3_state) {
                if (debounce()) {
                    switch(current_system) {
                        case SYSTEM_ONE:
                            systemOneTask(3);
                            break;
                        case SYSTEM_TWO:
                            systemTwoTask(3);
                            break;
                        case SYSTEM_THREE:
                            systemThreeTask(3);
                            break;
                        case SYSTEM_FOUR:
                            ESP_LOGI(TAG, "SYSTEM 4: B3 pressed - Custom logic");
                            break;
                        default:
                            break;
                    }
                }
            }
            // B4 pressed
            else if (current_b4 && !last_b4_state) {
                if (debounce()) {
                    switch(current_system) {
                        case SYSTEM_ONE:
                            systemOneTask(4);
                            break;
                        case SYSTEM_TWO:
                            systemTwoTask(4);
                            break;
                        case SYSTEM_THREE:
                            systemThreeTask(4);
                            break;
                        case SYSTEM_FOUR:
                            ESP_LOGI(TAG, "SYSTEM 4: B4 pressed - Custom logic");
                            break;
                        default:
                            break;
                    }
                }
            }
        } else {
            // Timer active - Block conflicting buttons based on system
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
                    if (active_button == 1) {
                        if (current_b2 || current_b3 || current_b4) {
                            ESP_LOGW(TAG, "SYSTEM 2: Buttons blocked during B1 timer");
                        }
                    } else if (active_button == 2) {
                        if (current_b1 || current_b3 || current_b4) {
                            ESP_LOGW(TAG, "SYSTEM 2: Buttons blocked during B2 timer");
                        }
                    } else if (active_button == 3) {
                        if (current_b1 || current_b2 || current_b4) {
                            ESP_LOGW(TAG, "SYSTEM 2: Buttons blocked during B3 timer");
                        }
                    } else if (active_button == 4) {
                        if (current_b1 || current_b2 || current_b3) {
                            ESP_LOGW(TAG, "SYSTEM 2: Buttons blocked during B4 timer");
                        }
                    }
                    break;
                    
                case SYSTEM_THREE:
                    // No blocking in System 3 (all buttons work independently)
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
        
        // Small delay to reduce CPU usage and prevent watchdog timeout
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


// connections:-

// Button	GPIO-
// B1	GPIO4
// B2	GPIO5
// B3	GPIO6
// B4	GPIO7

// LED	GPIO
// L1	GPIO8
// L2	GPIO9
// L3	GPIO10
// L4	GPIO11
// L5	GPIO12

// DIP Switch (8-bit Timer)-

// DIP Bit	Value	GPIO
// SW1	1	GPIO13
// SW2	2	GPIO14
// SW3	4	GPIO15
// SW4	8	GPIO16
// SW5	16	GPIO17
// SW6	32	GPIO18
// SW7	64	GPIO21
// SW8	128	GPIO38


// system switching dip switch
// 4-bit DIP Switch
// Bit	Value	GPIO
// D1	1	GPIO39
// D2	2	GPIO40
// D3	4	GPIO41
// D4	8	GPIO42

// system switching logic
// 0000 = default system
// 0001 = system one
// 0010 = system two
// 0011 = system three
// 0100 = system four


// timer logic-
// 1,2,4,8,16,32,64
// example
// 00000011 = 3 sec
// 00000101 = 5 sec
// 00011110 = 30 sec


// default system:-
// turn on all led for 1 second
// turn off all led for 1 second
// again on
// again off
// loop is continue of system set on default system


// system 1:-(two way with two projector)

// if i click on b1 then turn on l3,l5 for 5 second during that timer do not accept b3,b4
// if i click on b2 then turn on l4,l5 for 5 second during that timer do not accept b3,b4
// if i click on b3 then turn on l1,l5 for 5 second during that timer do not accept b1,b2
// if i click on b4 then turn on l2,l5 for 5 second during that timer do not accept b1,b2
// and
// i use one 8 ch dip switch for timer adjestment as per timer logic




// system 2:-(signal pro)

// if i click on b1 then turn on l2,l3,l4,l5 for 5 second during that timer do not accept b2,b3,b4
// if i click on b2 then turn on l1,l3,l4,l5 for 5 second during that timer do not accept b1,b3,b4
// if i click on b3 then turn on l1,l2,l4,l5 for 5 second during that timer do not accept b1,b2,b4
// if i click on b4 then turn on l1,l2,l3,l5 for 5 second during that timer do not accept b1,b2,l3
// and
// use also same timer dip switch


// system 3:-(two shutur with projectors)

// if i click on b1 then turn on l1,l5 for 5 second
// if i click on b2 then turn on l2,l5 for 5 second 
// if i click on b3 then turn on l3,l5 for 5 second
// if i click on b4 then turn on l4,l5 for 5 second
// and
// use also same timer dip switch


// note-do not forgot system logic,i want to make it for industrial use 24,7 turn on


