/*
Version Management
13.02.2025 V04 final (regular) version
12.02.2025 V03 development
12.02.2025 V02 development
11.02.2025 V01 First version 
*/

/**
* Please uncomment just one hardware definition file that reflects your hardware combination
* for Heltec WiFi LoRa 32 V2 boards use HELTEC_V2
* for Heltec WiFi LoRa 32 V3 boards use HELTEC_V3
* for LilyGo T3S3 LoRa boards use LILYGO_T3S3_SX1262
* for ESP32 Development boards with attached LoRa module SX1276 module and OLED use ESP32_SX1276_OLED
* for ESP32 Development boards with attached LoRa module SX1276 module and TFT use ESP32_SX1276_TFT
* for all other boards and hardware combination you should consider to modify an existing one to your needs
*
* Don't forget to change the Board in Arduino:
* for Heltec V2: Heltec WiFi LoRa 32(V2)
* for Heltec V3: Heltec WiFi LoRa 32(V3) / Wireless shell (V3) / ...
* for LilyGo T3S3 LoRa: ESP32S3 Dev Module
* or ESP32 Development Boards: ESP32-WROOM-DA Module
*
* - or in Tools menue:
* for Heltec V2: Tools - Board - esp32 - Heltec WiFi LoRa 32(V2)
* for Heltec V3: Tools - Board - esp32 - Heltec WiFi LoRa 32(V3) / Wireless shell (V3) / ...
* for LilyGo T3S3 LoRa: Tools - Board - esp32 - ESP32S3 Dev Module
* for ESP32 Development Boards: Tools - Board - esp32 - ESP32-WROOM-DA Module
*
*/

//#define HELTEC_V2
//#define HELTEC_V3
#define LILYGO_T3S3_SX1262
//#define ESP32_SX1276_OLED
//#define ESP32_SX1276_TFT

// ------------------------------------------------------------------
// include the hardware definition files depending on the uncommenting
#ifdef HELTEC_V2
#include "Heltec_V2_Hardware_Settings.h"
#endif

#ifdef HELTEC_V3
#include "Heltec_V3_Hardware_Settings.h"
#endif

#ifdef LILYGO_T3S3_SX1262
#include "LilyGo_T3S3_LoRa_SX1262_Hardware_Settings.h"
#endif

#ifdef ESP32_SX1276_OLED
#include "ESP32_SX1276_OLED_Hardware_Settings.h"
#endif

#ifdef ESP32_SX1276_TFT
#include "ESP32_SX1276_TFT_Hardware_Settings.h"
#endif

// ------------------------------------------------------------------

// when using the (default) OLED display SSD1306 128 * 64 px the maximum length is 26 chars
const String PROGRAM_VERSION = "BME280 Transmitter    V04";

// ------------------------------------------------------------------
// internal or external OLED SSD1306 128 * 64 px display

#ifdef IS_OLED
#include "FONT_MONOSPACE_9.h"
// For a connection via I2C using the Arduino Wire include:
#include <Wire.h>
#include "SSD1306.h"  // https://github.com/ThingPulse/esp8266-oled-ssd1306
SSD1306Wire display(OLED_I2C_ADDRESS, OLED_I2C_SDA_PIN, OLED_I2C_SCL_PIN);
#endif

#ifdef IS_TFT
// ------------------------------------------------------------------
// TFT display ST7735 1.8' 128 * 160 RGB
#include "FONT_MONOSPACE_9.h"
#include <SPI.h>
#include <Adafruit_GFX.h>                                        // Core graphics library, https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_ST7735.h>                                     // Hardware-specific library for ST7735, https://github.com/adafruit/Adafruit-ST7735-Library
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);  // hardware SPI
#endif

// vars for displaying line 1 to 5 to display in a loop
String display1 = "";
String display2 = "";
String display3 = "";
String display4 = "";
String display5 = "";
// for TFT only
String display6, display7, display8, display9, display10, display11, display12, display13;

bool showDisplay = false;
bool isDisplayOn = false;  // this bool is needed for switching the display off after timer exceeds

bool isDisplayTransmissionData = false;  // if true the transmission data are displayed instead of sensor data

// -----------------------------------------------------------------------
// BME280 sensor

#ifdef IS_BME280
//#include <Wire.h>
#include <Adafruit_Sensor.h>  // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_BME280.h>  // https://github.com/adafruit/Adafruit_BME280_Library
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;  // I2C
#endif

#ifdef IS_SECOND_I2C_BUS
TwoWire i2cOne = TwoWire(1);
#endif

float temperature = -99;
float humidity = -99;
float pressure = -99;
float altitude = -99;

// -----------------------------------------------------------------------
// Battery management

// note: this is just a dummy value for transmission
//  A voltage management is NOT included in this sketch
float batteryVoltage = 3.75;

// -----------------------------------------------------------------------
// https://github.com/StuartsProjects/SX12XX-LoRa

#include <SPI.h>

#ifdef HELTEC_V2
#include <SX127XLT.h>  //include the appropriate library
SX127XLT LT;           //create a library class instance called LT
#endif

#ifdef HELTEC_V3
#include <SX126XLT.h>  //include the appropriate library
SX126XLT LT;           //create a library class instance called LT
#endif

#ifdef LILYGO_T3S3_SX1262
#include <SX126XLT.h>  //include the appropriate library
SX126XLT LT;           //create a library class instance called LT
#endif

#ifdef ESP32_SX1276_OLED
#include <SX127XLT.h>  //include the appropriate library
SX127XLT LT;           //create a library class instance called LT
#endif

#ifdef ESP32_SX1276_TFT
#include <SX127XLT.h>  //include the appropriate library
SX127XLT LT;           //create a library class instance called LT
#endif

#include <arrayRW.h>        // routines for reading and writing varaibles to an array
#include "LoRa_Settings.h"  // include the setttings file, LoRa frequencies, txpower etc
#include "Node_Settings.h"  // include the node/sketch specific settings

uint32_t TXPacketCounter;
uint8_t TXPacketL;

// vars for sending
// prepairing the buffer
const uint8_t TXBUFFER_SIZE = 16;
uint8_t TXBuffer[TXBUFFER_SIZE];
long lastTransmissionMillis = 0;
long lastDisplayUpdateMillis = 0;

// measure the transmission time in milliseconds
long startTransmissionMillis = 0;
long endTransmissionMillis = 0;
long durationTransmissionMillis = 0;

// -----------------------------------------------------------------------
// PRG/Boot button
// #define BOOT_BUTTON_PIN 0 // see settings or hardware settings
#include <HotButton.h>  // https://github.com/ropg/HotButton
HotButton bootButton(BOOT_BUTTON_PIN);

boolean isSettingsMode = false;
uint8_t modeCounter = 0;  // just a counter

// LilyGo T3S3 support for battery mode
#include "LilyGoLoRaBoard.h"

void loop() {

  bootButton.update();
  if (bootButton.isSingleClick()) {
    singleDataTransmitting();
    lastTransmissionMillis = millis();
  }

  // the transmission will not start directly
  if (millis() - lastTransmissionMillis > TRANSMISSION_INTERVAL_MILLIS) {
    singleDataTransmitting();
    lastTransmissionMillis = millis();
  }

  if (millis() - lastDisplayUpdateMillis > DISPLAY_UPDATE_INTERVAL_MILLIS) {
    if (isDisplayTransmissionData) {
      isDisplayTransmissionData = false;
    } else {
      displaySensorData();
    }
    lastDisplayUpdateMillis = millis();
  }
}

void singleDataTransmitting() {
  uint8_t len;
  uint16_t IRQStatus;

  TXPacketCounter++;
  Serial.print(TXPacketCounter);  //print the numbers of sends
  Serial.print(F(" Sending > "));

  // get the sensor data
  getBme280Values();
  printBme280Values();
  displaySensorData();

  // prepairing an array for transmitting data
  /*
    Battery Voltage for transmission is a calculated value to fit in a uint8_t var:
    Usually a Lithium Ion battery has a nominal voltage of 3.7 volts, a maximum of 
    4.2 volts and a minimum of 3 volts. As we have one byte only available we have 
    to adjust the voltage in the range of 0..255 (range of an uint8_t variable).
    We multiply the battery voltage (which has usually 2 decimals like „3.78“) 
    with 100 and get a range of 300 .. 420. Now we are subtracting 200 from the 
    value and get a range of 100..220 which is perfect for our uint8_t variable. 
    On receiver side we are working the opposite way. If we receive e.g. a value 
    of „207“ we add „200“ and the sum is „407“, divided by 100.0 we get our initial 
    value of 4.07 volts.
  */
  uint8_t batteryVoltageTransmission = (uint8_t)((batteryVoltage * 100.0) - 200.0);
  float humidityTransmission = humidity * 100.0;
  beginarrayRW(TXBuffer, 0);                         // start writing to controlarray array at location 0
  arrayWriteUint8(NODE_NUMBER);                      // 1 byte sender
  arrayWriteUint8(RECEIVER_NUMBER);                  // 1 byte receiver
  arrayWriteUint32(TXPacketCounter);                 // 4 bytes
  arrayWriteFloat(temperature);                      // 4 bytes
  arrayWriteUint16((uint16_t)humidityTransmission);  // 2 bytes // not a float value as this would extend our defined array size of 16
  arrayWriteUint16((uint16_t)pressure);              // 2 bytes // not a float value as this would extend our defined array size of 16
  arrayWriteUint8(batteryVoltageTransmission);       // 1 byte battery voltage, see formula below
  arrayWriteUint8(0);                                // 1 byte, RFU - reserved for future usage
  uint8_t dataArraySize = endarrayRW() + 1;          // this returns the number of array bytes written, is 16 bytes
  Serial.print(F("The buffer is filled with "));
  Serial.print(dataArraySize);
  Serial.println(F(" bytes"));
  LT.printHEXPacket(TXBuffer, dataArraySize);  // print the sent array as HEX
  Serial.println();
  Serial.flush();

  LT.startWriteSXBuffer(0);  // initialise buffer write at address 0
  LT.writeBuffer(TXBuffer, dataArraySize);
  uint8_t txBufferSize = LT.endWriteSXBuffer();  // close buffer write
  
  // measure the transmission time
  startTransmissionMillis = millis();

  TXPacketL = LT.transmitSXBuffer(0, TXBUFFER_SIZE, 10000, TX_POWER, WAIT_TX);

  endTransmissionMillis = millis();
  durationTransmissionMillis = endTransmissionMillis - startTransmissionMillis;
  Serial.print(F("TX duration: "));
  Serial.print(durationTransmissionMillis) + Serial.println(F(" ms"));  // print the numbers of seconds

  if (TXPacketL) {
    ledFlash(1, 125);
    Serial.println(" SUCCESS");
    clearDisplayData();
    display1 = PROGRAM_VERSION;
    display2 = "Frequency: " + (String)(FREQUENCY / 1000) + " Khz";
    display3 = "TXCounter: " + (String)TXPacketCounter + " | Size: " + (String)txBufferSize;
    display4 = "  * SUCCESS *";
    display5 = "TX duration: " + (String)durationTransmissionMillis + " ms";
    displayData();
    lastDisplayUpdateMillis = millis();
    isDisplayTransmissionData = true;
  } else {
    ledFlash(3, 125);
    Serial.println(" FAILURE");
    display1 = PROGRAM_VERSION;
    display2 = "Frequency: " + (String)(FREQUENCY / 1000) + " Khz";
    display3 = "TXCounter: " + (String)TXPacketCounter + " | Size: " + (String)txBufferSize;
    display4 = "  * FAILURE *";
    display5 = "TX duration: " + (String)durationTransmissionMillis + " ms";
    displayData();
    lastDisplayUpdateMillis = millis();
    isDisplayTransmissionData = true;
  }
}

void getBme280Values() {
#ifdef IS_BME280
  bme.takeForcedMeasurement();
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
#else
  // dummy values
  temperature = 29.12;
  humidity = 70.23;
  pressure = 1234.5;
#endif
}

void printBme280Values() {
  Serial.print(F("Temperature: "));
  Serial.print(temperature, 1);
  Serial.print(F("c, Humidity: "));
  Serial.print(humidity);
  Serial.print(F("%, Pressure: "));
  Serial.print(pressure, 0);
  Serial.print(F("hPa, Altitude: "));
  Serial.print(altitude, 1);
  Serial.println(F("m"));
  Serial.flush();
}

void displaySensorData() {
  uint16_t remainingSeconds = (TRANSMISSION_INTERVAL_MILLIS - (millis() - lastTransmissionMillis)) / 1000;
  display1 = PROGRAM_VERSION;
  display2 = "Node:" + String(NODE_NUMBER, HEX) + " RECV:" + String(RECEIVER_NUMBER, HEX) + " TXCnt:" + (String)TXPacketCounter;
  display3 = "T:" + String(temperature) + " *C    | H:" + String(humidity) + " %";
  display4 = "P:" + String(pressure) + " hPa | V: " + String(batteryVoltage, 2) + " V";
  display5 = "Next transmiss.in " + (String)remainingSeconds + " sec.";
  displayData();
}

void displayTransmissionData(bool success) {
  display1 = PROGRAM_VERSION;
  display2 = "T:" + String(temperature) + " *C H:" + String(humidity) + "%";
  display3 = "P:" + String(pressure) + " hPa";
  display4 = "Battery Volt: " + String(batteryVoltage, 2) + " V";
  display5 = "Battery Volt: " + String(batteryVoltage, 2) + " V";
  displayData();
}

void ledFlash(uint16_t flashes, uint16_t delaymS) {
  // run only if a LED is connected
  if (LED_PIN >= 0) {
    uint16_t index;
    for (index = 1; index <= flashes; index++) {
      digitalWrite(LED_PIN, HIGH);
      delay(delaymS);
      digitalWrite(LED_PIN, LOW);
      delay(delaymS);
    }
  }
}

void setup() {
#ifdef LILYGO_T3S3_SX1262
  setupLilyGoBoard();
#else
  Serial.begin(115200);
  while (!Serial) {}
#endif

  Serial.println(F("ESP32 MD LoRa BME280 Transmitter V04"));

  // if we have a power control for devices put it on
#ifdef IS_VEXT_CONTROL
  setVextControl(true);
#endif

  if (LED_PIN >= 0) {
    pinMode(LED_PIN, OUTPUT);  // setup pin as output for indicator LED
    ledFlash(1, 125);          // two quick LED flashes to indicate program start
  }

  // this is necessary as the LilyGo T3S3 board requires a different setup
#ifdef LILYGO_T3S3_SX1262
  SPI.begin(SX_SCK, SX_MISO, SX_MOSI);
#else
  SPI.begin();
#endif

  // setup display
#ifdef IS_OLED
  if (OLED_I2C_RST_PIN >= 0) {
    pinMode(OLED_I2C_RST_PIN, OUTPUT);
    digitalWrite(OLED_I2C_RST_PIN, LOW);  // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(OLED_I2C_RST_PIN, HIGH);
    delay(50);
  }
  clearDisplayData();
  display.init();
#ifdef DISPLAY_ORIENTATION_FLIPPED
  // do nothing
#else
  display.flipScreenVertically();  // Landscape 90 degrees right rotated
#endif
  display.setFont(ArialMT_Plain_10);
  delay(50);
  display1 = PROGRAM_VERSION;
  displayData();
  delay(500);
#endif

  // init TFT display
#ifdef IS_TFT
  tft.initR(INITR_BLACKTAB);     // den ST7735S Chip initialisieren, schwarz
  tft.fillScreen(ST77XX_BLACK);  // und den Schirm mit Schwarz füllen
  tft.setTextWrap(false);        // automatischen Zeilenumbruch ausschalten
#ifdef DISPLAY_ORIENTATION_FLIPPED
  tft.setRotation(1);  // Landscape 270 degrees right rotated
#else
  tft.setRotation(3);              // Landscape 90 degrees right rotated
#endif
  Serial.println(F("Display init done"));
  display1 = PROGRAM_VERSION;
  displayData();
  delay(500);
#endif

  display2 = "Display init done";
  displayData();
  delay(1000);

  // init BME280 sensor
#ifdef IS_BME280
#ifndef IS_SECOND_I2C_BUS
  bool bme_status = bme.begin(BME280_I2C_ADDRESS);  // address either 0x76 or 0x77
#else
  bool wireStatus = Wire1.begin(BME280_I2C_SDA_PIN, BME280_I2C_SCL_PIN);
  if (!wireStatus) {
    Serial.println(F("Wire1 failed to init"));
    display3 = "Wire FAILURE";
    display4 = "System is halting";
    displayData();
    while (1)
      ;
  } else {
    display3 = "BME280/Wire1 ACTIVE";
    displayData();
    Serial.println(F("Wire1 initialized"));
  }
  bool bme_status = bme.begin(BME280_I2C_ADDRESS, &Wire1);  //address either 0x76 or 0x77
#endif
  if (!bme_status) {
    ledFlash(100, 15);  // long very fast speed flash indicates BME280 device error
    display4 = "BME280 not found";
    display5 = "System is halting";
    displayData();
    Serial.println(F("No valid BME280 found"));
    while (1)
      ;
  } else {
    display4 = "BME280 ACTIVE";
    displayData();
    Serial.println(F("BME280 found"));
  }
  // the preferred way to get correct indoor temperatures
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X16,  // temperature
                  Adafruit_BME280::SAMPLING_X1,   // pressure
                  Adafruit_BME280::SAMPLING_X1,   // humidity
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_0_5);
#endif

  // setup the LoRa device
#ifdef HELTEC_V2
  if (LT.begin(SX_NSS, SX_NRESET, SX_DIO0, LORA_DEVICE)) {
    Serial.println(F("LoRa Device Heltec V2 found"));
    display3 = "LoRa Device HV2 found";
    displayData();
    ledFlash(2, 125);
  } else {
    Serial.println(F("Device error"));
    while (1) {
      Serial.println(F("No device responding"));
      display3 = "No LoRa Device found";
      displayData();
      ledFlash(50, 50);  // long fast speed flash indicates LoRa device error
    }
  }
#endif


#ifdef HELTEC_V3
  if (LT.begin(SX_NSS, SX_NRESET, SX_RFBUSY, SX_DIO1, LORA_DEVICE)) {
    Serial.println(F("LoRa Device Heltec V3 found"));
    display3 = "LoRa Device HV3 found";
    displayData();
    ledFlash(1, 125);
  } else {
    Serial.println(F("Device error"));
    while (1) {
      Serial.println(F("No device responding"));
      display3 = "No LoRa Device found";
      displayData();
      ledFlash(50, 50);  // long fast speed flash indicates LoRa device error
    }
  }
  // The Heltec V3 board uses an unusual crystal voltage. Somme errors came up
  // when using Reliable communication so I'm setting the value here.
  LT.setDIO3AsTCXOCtrl(TCXO_CTRL_1_8V);
#endif

#ifdef LILYGO_T3S3_SX1262
  if (LT.begin(SX_NSS, SX_NRESET, SX_RFBUSY, SX_DIO1, LORA_DEVICE)) {
    Serial.println(F("LoRa Device LilyGo T3S3 SX1262 found"));
    display3 = "LoRa Dev LilyGoT3S3 found";
    displayData();
    ledFlash(1, 125);
  } else {
    Serial.println(F("Device error"));
    Serial.println(F("LoRa Device LilyGo T3S3 SX1262"));
    while (1) {
      Serial.println(F("No device responding"));
      display3 = "No LoRa Device found";
      displayData();
      ledFlash(50, 50);  // long fast speed flash indicates LoRa device error
    }
  }
  // The LilyGo T3S3 board uses an unusual crystal voltage. Somme errors came up
  // when using Reliable communication so I'm setting the value here.
  LT.setDIO2AsRfSwitchCtrl();
  LT.setDIO3AsTCXOCtrl(TCXO_CTRL_1_8V);
#endif

#ifdef ESP32_SX1276_OLED
  if (LT.begin(SX_NSS, SX_NRESET, SX_DIO0, LORA_DEVICE)) {
    Serial.println(F("LoRa Device ESP32/SX1276 found"));
    display3 = "LoRa Dev.ESP32+SX1276";
    displayData();
    ledFlash(2, 125);
  } else {
    Serial.println(F("Device error"));
    while (1) {
      Serial.println(F("No device responding"));
      display3 = "No LoRa Device found";
      displayData();
      ledFlash(50, 50);  // long fast speed flash indicates LoRa device error
    }
  }
#endif

#ifdef ESP32_SX1276_TFT
  if (LT.begin(SX_NSS, SX_NRESET, SX_DIO0, LORA_DEVICE)) {
    Serial.println(F("LoRa Device ESP32/SX1276 found"));
    display3 = "LoRa Dev.ESP32+SX1276";
    displayData();
    ledFlash(2, 125);
  } else {
    Serial.println(F("Device error"));
    while (1) {
      Serial.println(F("No device responding"));
      display3 = "No LoRa Device found";
      displayData();
      ledFlash(50, 50);  // long fast speed flash indicates LoRa device error
    }
  }
#endif

  // just to be for sure - use the default syncword
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);  // this is the default value
  // set the high sensitive mode
  // Sets LoRa device for the highest sensitivity at expense of slightly higher LNA current.
  // The alternative is setLowPowerReceive() for lower sensitivity with slightly reduced current.
  LT.setHighSensitivity();
  // start the device with default parameters
  LT.setupLoRa(FREQUENCY, OFFSET, SPREADING_FACTOR, BANDWIDTH, CODE_RATE, OPTIMISATION);
  Serial.println(F("LoRa setup is complete"));

  // debug information
  Serial.println();
  LT.printModemSettings();
  Serial.println();
  LT.printOperatingSettings();
  Serial.println();

  display1 = PROGRAM_VERSION;
  display2 = "Fqcy: " + (String)(FREQUENCY / 1000) + " kHz | SF: " + (String)SPREADING_FACTOR;
  display3 = "   Press BOOT button";
  display4 = "once:   Transmitting";
  display5 = "        a packet";
  displayData();
  delay(2000);

  getBme280Values();                                                 // initial data
  lastTransmissionMillis = millis() - TRANSMISSION_INTERVAL_MILLIS;  // the first transmission willstart directly in loop
}

void displayData() {
#ifdef IS_TFT
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setFont(NULL);  // Pass NULL to revert to 'classic' fixed-space bitmap font.
  tft.setCursor(0, 0);
  tft.print(display1);
  tft.setCursor(0, 10);
  tft.print(display2);
  tft.setCursor(0, 20);
  tft.print(display3);
  tft.setCursor(0, 30);
  tft.print(display4);
  tft.setCursor(0, 40);
  tft.print(display5);
  tft.setCursor(0, 50);
  tft.print(display6);
  tft.setCursor(0, 60);
  tft.print(display7);
  tft.setCursor(0, 70);
  tft.print(display8);
  tft.setCursor(0, 80);
  tft.print(display9);
  tft.setCursor(0, 90);
  tft.print(display10);
  tft.setCursor(0, 100);
  tft.print(display11);
  tft.setCursor(0, 110);
  tft.print(display12);
  tft.setCursor(0, 120);
  tft.print(display13);
#endif

#ifdef IS_OLED
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(Monospaced_plain_9);
  display.drawString(0, 0, display1);
  display.drawString(0, 12, display2);
  display.drawString(0, 24, display3);
  display.drawString(0, 36, display4);
  display.drawString(0, 48, display5);
  display.display();
#endif
}

void clearDisplayData() {
  display1 = "";
  display2 = "";
  display3 = "";
  display4 = "";
  display5 = "";
  display6 = "";
  display7 = "";
  display8 = "";
  display9 = "";
  display10 = "";
  display11 = "";
  display12 = "";
  display13 = "";
}

void setVextControl(boolean trueIsOn) {
#ifdef IS_VEXT_CONTROL
  if (trueIsOn) {
    pinMode(VEXT_POWER_CONTROL_PIN, OUTPUT);
    digitalWrite(VEXT_POWER_CONTROL_PIN, LOW);
  } else {
    // pulled up, no need to drive it
    pinMode(VEXT_POWER_CONTROL_PIN, INPUT);
  }
#endif
}
