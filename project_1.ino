#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

const int fanPin = 18;
const int greenLed = 19;
const int potPin = 4;
const int redLed = 17;
const int tempPin = 34;

Adafruit_BMP280 bmp;
bool bmpReady = false;

void setup() {
  Serial.begin(115200);
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(fanPin, OUTPUT);

  if (bmp.begin(0x76)) {
    bmpReady = true;
  } else {
    Serial.println("Could not find a valid BMP280 sensor!");
    while (1);
  }
}

void loop() {
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100;
  float altitude = bmp.readAltitude(1013.25);
  int potValue = analogRead(potPin);
  int pwmValue = map(potValue, 100, 4095, 0, 255);

  pwmValue = constrain(pwmValue, 0, 255);


  Serial.printf("Temperature: %.2f °C\n", temp);
  Serial.printf("Pressure: %.2f Pa\n", pressure);
  Serial.printf("Altitude: %.2f m\n", altitude);
  Serial.printf("Pot Value: %d, PWM: %d\n", potValue, pwmValue);
  Serial.println();

  if (temp > 25) {
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
    analogWrite(fanPin, pwmValue);
  } else {
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, HIGH);
    analogWrite(fanPin, 0);
  }

  delay(5000);
}