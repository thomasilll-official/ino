// ESP32-C3 Super Mini
// 4 Push Buttons + 4 Blinking LEDs

const int buttons[] = {0, 1, 3, 4};
const int leds[]    = {5, 6, 7, 10};

unsigned long previousMillis[4] = {0, 0, 0, 0};

const unsigned long blinkInterval = 50; // 500 ms

bool ledState[4] = {false, false, false, false};

void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(buttons[i], INPUT_PULLUP);
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }
}

void loop() {

  unsigned long currentMillis = millis();

  for (int i = 0; i < 4; i++) {

    // Button pressed = LOW
    if (digitalRead(buttons[i]) == LOW) {

      // Blink LED
      if (currentMillis - previousMillis[i] >= blinkInterval) {
        previousMillis[i] = currentMillis;

        ledState[i] = !ledState[i];
        digitalWrite(leds[i], ledState[i]);
      }

    } else {

      // Button released = LED OFF
      ledState[i] = false;
      digitalWrite(leds[i], LOW);

      // Reset timer
      previousMillis[i] = currentMillis;
    }
  }
}
