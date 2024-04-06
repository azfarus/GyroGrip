// Include necessary libraries
#include <Arduino.h>

#define DIG_READ_FAST(pin) (*(volatile uint32_t *)(GPIO_IN_REG) & (1 << pin))

// Define the GPIO pin number you want to read
const int inputPin = 4;  // Example pin

void setup() {
  // Set the pin mode to input
  pinMode(14, INPUT_PULLDOWN);
  pinMode(27, INPUT_PULLDOWN);
  pinMode(26, INPUT_PULLDOWN);
  pinMode(25, INPUT_PULLDOWN);
  // Initialize Serial for debugging
  Serial.begin(115200);
}

void loop() {
  // Read the pin state using direct port manipulation
  uint32_t pin_state = REG_READ(GPIO_IN_REG); // Reads the value of all GPIOs at once
  bool isHigh14 = DIG_READ_FAST(14);
  bool isHigh27 = DIG_READ_FAST(27);
  bool isHigh26 = DIG_READ_FAST(26);
  bool isHigh25 = DIG_READ_FAST(25);

  // Print the result
  if(isHigh14) Serial.println("SW1");
  if(isHigh27) Serial.println("SW4");
  if(isHigh26) Serial.println("SW2");
  if(isHigh25) Serial.println("SW3");

  // Add some delay to avoid spamming Serial output
  delay(100);
}
