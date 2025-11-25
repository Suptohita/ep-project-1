#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

const int fanPin = 18;
const int potPin = 4;
const int redLed = 17;
const int greenLed = 19;
const int BUTTON_PIN = 15;

// BMP280
Adafruit_BMP280 bmp;
bool bmpReady = false;

// System state
volatile bool system_enabled = false;
volatile bool button_pressed_flag = false;
volatile unsigned long last_interrupt_time = 0;

void IRAM_ATTR handleButtonInterrupt() {
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 50) {
        button_pressed_flag = true;
        last_interrupt_time = interrupt_time;
    }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- ESP32 Temperature Control System ---");
  
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  analogWrite(fanPin, 0);

  if (bmp.begin(0x76)) {
    bmpReady = true;
    Serial.println("BMP280 sensor initialized successfully!");
  } else {
    Serial.println("Could not find a valid BMP280 sensor!");
    while (1);
  }
  
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonInterrupt, FALLING);
  Serial.println("System ready. Press button to start/stop.");
}

void loop() {
  // Handle button press
  if (button_pressed_flag) {
    button_pressed_flag = false;
    system_enabled = !system_enabled;
    Serial.printf("System %s\n", system_enabled ? "ENABLED" : "DISABLED");
    
    // Turn off everything when system is disabled
    if (!system_enabled) {
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, LOW);
      analogWrite(fanPin, 0);
    }
  }

  // Only run temperature control if system is enabled
  if (system_enabled) {
    float temp = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100;
    float altitude = bmp.readAltitude(1013.25);
    int potValue = analogRead(potPin);
    int pwmValue = map(potValue, 200, 4095, 0, 255);

    pwmValue = constrain(pwmValue, 0, 255);

    Serial.printf("Temperature: %.2f °C\n", temp);
    Serial.printf("Pressure: %.2f Pa\n", pressure);
    Serial.printf("Altitude: %.2f m\n", altitude);
    Serial.printf("Pot Value: %d, PWM: %d\n", potValue, pwmValue);
    Serial.println();

    if (temp > 23) {
      digitalWrite(redLed, HIGH);
      digitalWrite(greenLed, LOW);
      analogWrite(fanPin, pwmValue);
    } else {
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, HIGH);
      analogWrite(fanPin, 0);
    }
  }

  delay(2000);
}