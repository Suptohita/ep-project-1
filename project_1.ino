// // fan
// const int fanPin = 18;
// const int tempPin = 34;  // Analog pin for temperature sensor

// void setup() {
//   pinMode(fanPin, OUTPUT);
//   Serial.begin(115200);
// }

// void loop() {
//   int sensorValue = analogRead(tempPin);
//   float voltage = sensorValue * (3.3 / 4095.0);
//   float temperature = (voltage - 0.5) * 100;  // LM35 sensor example
  
//   int fanSpeed = map(temperature, 20, 50, 50, 255);  // 20°C=low, 50°C=full
//   fanSpeed = constrain(fanSpeed, 50, 255);  // Minimum 50 to ensure startup
  
//   analogWrite(fanPin, 255);
  
//   Serial.printf("Temp: %.1f°C, Fan: %d\n", temperature, fanSpeed);
//   delay(1000);
// }

// // pot
// // Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
// const int potPin = 4;

// // variable for storing the potentiometer value
// int potValue = 0;

// void setup() {
//   Serial.begin(115200);
//   delay(1000);
// }

// void loop() {
//   // Reading potentiometer value
//   potValue = analogRead(potPin);
//   Serial.println(potValue);
//   delay(500);
// }


// // tem; sensor
// #include <Wire.h>
// #include <SPI.h>
// #include <Adafruit_BMP280.h>

// #define BMP_SCK  (13)
// #define BMP_MISO (12)
// #define BMP_MOSI (11)
// #define BMP_CS   (10)

// Adafruit_BMP280 bmp; // I2C
// //Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
// //Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

// void setup() {
//   Serial.begin(9600);
//   while ( !Serial ) delay(100);   // wait for native usb
//   Serial.println(F("BMP280 test"));
//   unsigned status;
//   //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
//   status = bmp.begin(0x76);
//   if (!status) {
//     Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
//                       "try a different address!"));
//     Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
//     Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
//     Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
//     Serial.print("        ID of 0x60 represents a BME 280.\n");
//     Serial.print("        ID of 0x61 represents a BME 680.\n");
//     while (1) delay(10);
//   }

//   /* Default settings from datasheet. */
//   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
//                   Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
//                   Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
//                   Adafruit_BMP280::FILTER_X16,      /* Filtering. */
//                   Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
// }

// void loop() {
//     Serial.print(F("Temperature = "));
//     Serial.print(bmp.readTemperature());
//     Serial.println(" *C");

//     Serial.print(F("Pressure = "));
//     Serial.print(bmp.readPressure());
//     Serial.println(" Pa");

//     Serial.print(F("Approx altitude = "));
//     Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
//     Serial.println(" m");

//     Serial.println();
//     delay(2000);
// }

// // LIGHTS
// // the setup function runs once when you press reset or power the board
// void setup() {
//   // initialize digital pin LED_BUILTIN as an output.
//   pinMode(17, OUTPUT);
//   pinMode(19, OUTPUT);
// }

// // the loop function runs over and over again forever
// void loop() {
//   digitalWrite(19, HIGH);  // turn the LED on (HIGH is the voltage level)
//   digitalWrite(17, LOW);  // turn the LED on (HIGH is the voltage level)
//   delay(1000);                      // wait for a second
//   digitalWrite(19, LOW);   // turn the LED off by making the voltage LOW
//   digitalWrite(17, HIGH);   // turn the LED off by making the voltage LOW
//   delay(1000);                      // wait for a second
// }




#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

// ----- Pins -----
const int fanPin = 18;
const int tempPin = 34;  // LM35 analog temp sensor
const int potPin = 4;
const int led1 = 17;
const int led2 = 19;

// ----- BMP280 -----
Adafruit_BMP280 bmp; // I2C
bool bmpReady = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Fan, LED, sensors setup
  pinMode(fanPin, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  // BMP280 setup
  if (bmp.begin(0x76)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    bmpReady = true;
  } else {
    Serial.println("BMP280 not found");
  }
}

void loop() {
  // ----- LM35 Temperature -----
  int sensorValue = analogRead(tempPin);
  float voltage = sensorValue * (3.3 / 4095.0);
  float lm35Temp = (voltage - 0.5) * 100.0;  // for LM35

  // ----- Fan speed control -----
  int fanSpeed = map(lm35Temp, 20, 50, 50, 255);
  fanSpeed = constrain(fanSpeed, 50, 255);
  analogWrite(fanPin, fanSpeed);

  // ----- Potentiometer -----
  int potValue = analogRead(potPin);

  // ----- BMP280 readings -----
  float bmpTemp = NAN, pressure = NAN, altitude = NAN;
  if (bmpReady) {
    bmpTemp = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude(1013.25);
  }

  // ----- LED toggle -----
  static bool ledState = false;
  digitalWrite(led1, ledState ? HIGH : LOW);
  digitalWrite(led2, ledState ? LOW : HIGH);
  ledState = !ledState;

  // ----- Serial Output -----
  Serial.println("=== Readings ===");
  Serial.printf("LM35 Temp: %.2f °C\n", lm35Temp);
  Serial.printf("Fan Speed: %d\n", fanSpeed);
  Serial.printf("Pot Value: %d\n", potValue);
  if (bmpReady) {
    Serial.printf("BMP Temp: %.2f °C\n", bmpTemp);
    Serial.printf("Pressure: %.2f Pa\n", pressure);
    Serial.printf("Altitude: %.2f m\n", altitude);
  }
  Serial.println();

  delay(1000);
}


// === Readings ===
// LM35 Temp: -50.00 °C
// Fan Speed: 50
// Pot Value: 240
// BMP Temp: 25.39 °C
// Pressure: 101314.97 Pa
// Altitude: 0.84 m
