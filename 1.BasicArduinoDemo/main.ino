// --------------- LIBRARIES --------------- //
#include <Wire.h>                              // Library used for the I2C protocol
#include <OneWire.h>                           // Library used for the 1Wire protocol
#include <Adafruit_BME280.h>                   // Library we use for the BME280 (see https://github.com/adafruit/Adafruit_BME280_Library)
#include <DallasTemperature.h>                 // Library we use for the Dallas temperature sensor (see https://github.com/milesburton/Arduino-Temperature-Control-Library)
#include <SparkFunCCS811.h>                    // Library we use for the CJMCU811 sensor (see https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library)
#include <Adafruit_INA219.h>                   // Library we use for the INA219 sensor (see https://github.com/adafruit/Adafruit_INA219)
#include <SparkFun_ADS1015_Arduino_Library.h>  // Library we use for the ADS1015 sensor (see https://github.com/sparkfun/SparkFun_ADS1015_Arduino_Library)

// Define known addresses for the I2C bus
// Look at https://i2cdevices.org/addresses for more information.
#define NEXUS_INA219_ADDRESS 0x42
#define NEXUS_ADS1015_ADDRESS 0x48
#define NEXUS_CCS811_ADDRESS 0x5A
#define NEXUS_BME280_ADDRESS 0x76

// --------------- PINS --------------- //
// Pins used for the LED light
#define LED_R_PIN 19
#define LED_G_PIN 18
#define LED_B_PIN 5
// Pins used for I2C
#define SDA_PIN 21
#define SCL_PIN 22
// Pins used for OneWire
#define ONEWIRE_PIN 33
// Pins for other sensors
#define PIR_PIN 39
#define LUX_PIN 4                // pin LOW sets the measurement to max 100.000lux (outside), pin HIGH max 1030lux (indoor)
#define NEXUS_CJMCU_WAKE_PIN 23  // enables or disables (deep sleep) the sensor

// --------------- INSTANCES --------------- //
// define a simple array to easily switch between colors
enum LedColor {
  WHITE,
  RED,
  GREEN,
  BLUE,
  OFF
};

// BME 280
bool bme280Found = false;
Adafruit_BME280 bme280;
Adafruit_Sensor *bme280_temp;
Adafruit_Sensor *bme280_pressure;
Adafruit_Sensor *bme280_humidity;

// Dallas temperature
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature dallasLib(&oneWire);                  // pass a reference of the Onewire bus with the configured pin for the library to communicate on
unsigned long millisTimestampDallasSensors = millis();  // a high resulotion requires a delay for the sensor to calculate the new value
bool dallasIsCalculating = false;
int dallasPreferredResolution = 12;  // use the highest possible accuracy

// CJMCU811 sensor
bool cjmcuFound = false;
CCS811 ccs811Lib(NEXUS_CCS811_ADDRESS);

// INA219 sensor
bool ina219Found = false;
Adafruit_INA219 ina219Lib(NEXUS_INA219_ADDRESS);

// ads1015 sensor
bool ads1015Found = false;
ADS1015 adcSensor;

// PIR sensor
bool pirActive = false;

// global timers
unsigned long millisTimestampGlobalSensors = millis();
unsigned long millisTimestampPirSensor = millis();

void setup() {
  // Activate the Serial Comm
  Serial.begin(115200);

  // Set All Pins to the desired mode
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(LUX_PIN, OUTPUT);
  pinMode(NEXUS_CJMCU_WAKE_PIN, OUTPUT);

  // Activate the I2C bus
  Wire.begin(SDA_PIN, SCL_PIN);

  // Activate Dallas temp sensor(s)
  dallasLib.begin();
  dallasLib.setResolution(dallasPreferredResolution);

  // Activate BME280
  if (checkI2CDevice(NEXUS_BME280_ADDRESS)) {
    bool bme280StartStatus = bme280.begin(NEXUS_BME280_ADDRESS, &Wire);
    if (bme280StartStatus) {
      bme280Found = true;
      Serial.println("--OK-- BME280 found.");
    }
    bme280_temp = bme280.getTemperatureSensor();
    bme280_pressure = bme280.getPressureSensor();
    bme280_humidity = bme280.getHumiditySensor();
  }

  // Activate CJMCU811
  if (checkI2CDevice(NEXUS_CCS811_ADDRESS)) {
    CCS811Core::CCS811_Status_e startStatus = ccs811Lib.beginWithStatus(Wire);
    if (startStatus == CCS811Core::CCS811_Stat_SUCCESS) {
      cjmcuFound = true;
      Serial.println("--OK-- CJMCU811 found.");
      // Configure the CJMCU811 sensor
      // 1 = every 1s, 2 = every 10s, 3 = every 60s, 4 = RAW mode
      ccs811Lib.setDriveMode(1);
    } else {
      Serial.printf("--WARNING-- CJMCU could not start, errorcode: %s.\n", ccs811Lib.statusString(startStatus));
    }
  }

  // Activate INA219
  if (checkI2CDevice(NEXUS_INA219_ADDRESS)) {
    bool ina219Status = ina219Lib.begin();
    if (ina219Status) {
      ina219Found = true;
      Serial.println("--OK-- INA219 found.");

    } else {
      Serial.printf("--WARNING-- INA219 could not start.\n");
    }
  }

  // Activate ADS1015
  if (checkI2CDevice(NEXUS_ADS1015_ADDRESS)) {
    if (adcSensor.begin(NEXUS_ADS1015_ADDRESS)) {
      adcSensor.setGain(ADS1015_CONFIG_PGA_TWOTHIRDS);      // max ~6,144 volts (needed for battery)
      adcSensor.setMode(0);                                 // measure continuously (preferred mode)
      adcSensor.setSampleRate(ADS1015_CONFIG_RATE_3300HZ);  // max sample mode, causes a minimal delay of 400us, but requires more power.
      adcSensor.useConversionReady(false);                  // this causes a delay of max 16 milliseconds, based on sampling rate. For the sake of this example code, no multi threading is implemented.
      ads1015Found = true;
      Serial.println("--OK-- INA219 found.");
    } else {
      Serial.printf("--WARNING-- ADS1015 could not start.\n");
    }
  }

  delay(3000);  // delay a few seconds for the user to read the initial statuses.
}

void loop() {
  digitalWrite(NEXUS_CJMCU_WAKE_PIN, LOW);  // we always want the CJMCU811 active
  digitalWrite(LUX_PIN, HIGH);              // we assume you start testing in a home(office) environment, therefore set the measuring to INDOOR mode.


  // always check the PIR sensor, this should react 'realtime'
  processPirSensor();

  // Do the sensor readout / update routine every second.
  if (millis() - millisTimestampGlobalSensors < 1000) {
    return;
  }

  // When 'idle', just flash the led.
  if (!pirActive) {
    toggleLed();
  }

  printBME280Values();

  printDallasValues();

  printCJMCU811Values();

  printIna219Values();

  printAds1015Values();  
  
  // TODO LUX

  millisTimestampGlobalSensors = millis();
}

void processPirSensor() {
  // Check the motion sensor every 20ms for a new state.
  if (millis() - millisTimestampGlobalSensors >= 20) {
    if (pirActive && digitalRead(PIR_PIN) == LOW) {
      Serial.printf("---PIR SENSOR--- Motion Stopped\n");
      pirActive = false;
    }
    if (!pirActive && digitalRead(PIR_PIN) == HIGH) {
      Serial.printf("---PIR SENSOR--- Motion Detected\n");
      setLedColor(LedColor::BLUE);
      pirActive = true;
    }
    millisTimestampPirSensor = millis();
  }
}

void printBME280Values() {
  if (!bme280Found) {
    return;  // do nothing to prevent errors.
  }

  // read BME280 sensor and print values
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme280_temp->getEvent(&temp_event);
  bme280_pressure->getEvent(&pressure_event);
  bme280_humidity->getEvent(&humidity_event);
  Serial.printf("BME280 values - temperature: %fC, air pressure: %fhPa, air humidity: %f\%\n", temp_event.temperature, pressure_event.pressure, humidity_event.relative_humidity);
}

void printDallasValues() {
  // read the connected Dallas temperature sensors and print values
  int dallasSensorsFound = dallasLib.getDS18Count();
  if (dallasSensorsFound > 0) {
    if (millis() - millisTimestampDallasSensors >= dallasLib.millisToWaitForConversion(dallasPreferredResolution)) {
      for (int i = 0; i < dallasSensorsFound; i++) {
        Serial.printf("Dallas sensor %i value: %f\n", i, dallasLib.getTempCByIndex(i) + 1);  // add one to make it more human readable
      }
      // after printing the values, request new temperatures.
      dallasLib.requestTemperatures();
      millisTimestampDallasSensors = millis();
    } else {
      Serial.printf("Dallas sensors are calculating... Time to new reading: %i ms\n", dallasLib.millisToWaitForConversion(dallasPreferredResolution) - (millis() - millisTimestampDallasSensors));
    }
  }
}

// Please note that this sensor needs approx 500 hours to properly burn in before advanced features like the baseline should be used.
// The sensor needs 20 minutes of warmup time before accurate readings are given.
// Read more about using and programming the baseline feature here: https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library/blob/master/examples/Example4_SetBaseline/Example4_SetBaseline.ino
void printCJMCU811Values() {
  if (!cjmcuFound) {
    return;  // do nothing to prevent errors.
  }

  // we need environmental data to make the CCS811 more precise
  sensors_event_t temp_event, humidity_event;
  if (bme280Found) {
    bme280_temp->getEvent(&temp_event);
    bme280_humidity->getEvent(&humidity_event);
  }
  if (ccs811Lib.dataAvailable()) {
    CCS811Core::CCS811_Status_e readoutStatus = ccs811Lib.readAlgorithmResults();
    ccs811Lib.setEnvironmentalData(humidity_event.relative_humidity, temp_event.temperature);

    Serial.printf("CJMCU811 status: %s | Values - tVOC: %i, eCO2: %i\n", ccs811Lib.statusString(readoutStatus), ccs811Lib.getTVOC(), ccs811Lib.getCO2());
  } else {
    Serial.printf("CJMCU811 busy is calculating...\n");
  }
}

void printIna219Values() {
  if (!ina219Found) {
    return;  // do nothing to prevent errors.
  }

  float curA = 0.0;
  // Read voltage and current from INA219.
  float shuntvoltage = ina219Lib.getShuntVoltage_mV();
  float busvoltage = ina219Lib.getBusVoltage_V();
  curA = ina219Lib.getCurrent_mA();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);
  float power_mW = loadvoltage * curA;

  Serial.printf("INA219 (Power supply) values - mAh: %f, V: %f, usage in mW: %f\n", curA, loadvoltage, power_mW);
}

void printAds1015Values() {
  if (!ads1015Found) {
    return;  // do nothing to prevent errors.
  }

  // the private variable _multiplierToVolts is auto-updated each time setGain is called
  float multiplier = adcSensor.getMultiplier();  // used to convert readings to actual voltages (in mV units)
  uint16_t channel0 = adcSensor.getSingleEnded(0);
  uint16_t channel1 = adcSensor.getSingleEnded(1);
  uint16_t channel2 = adcSensor.getSingleEnded(2);
  uint16_t channel3 = adcSensor.getSingleEnded(3);

  float channel0MilliVolts = channel0 * multiplier;
  float channel1MilliVolts = channel1 * multiplier;
  float channel2MilliVolts = channel2 * multiplier;
  float channel3MilliVolts = channel3 * multiplier;

  Serial.printf("ADS1015 values - channel 0 (batt voltage): %fmV, channel 1 (LUX voltage): %fmV, channel 2 (free): %fmV, channel 3 (free): %fmV\n", channel0MilliVolts, channel1MilliVolts, channel2MilliVolts, channel3MilliVolts);

  uint16_t channel_A0 = adcSensor.getSingleEnded(0);
}

// Simple method toggling the RGB LED on and off.
void toggleLed() {
  if (digitalRead(LED_R_PIN) == LOW) {
    digitalWrite(LED_R_PIN, HIGH);
    digitalWrite(LED_G_PIN, HIGH);
    digitalWrite(LED_B_PIN, HIGH);
  } else {
    digitalWrite(LED_R_PIN, LOW);
    digitalWrite(LED_G_PIN, LOW);
    digitalWrite(LED_B_PIN, LOW);
  }
}

// Method used to set the LED in the desired state
// Please be aware of the fact that the LED used has a shared cathode, meaning it will light up when the corresponding pin is pulled LOW
void setLedColor(LedColor color) {
  // reset all.
  digitalWrite(LED_R_PIN, HIGH);
  digitalWrite(LED_G_PIN, HIGH);
  digitalWrite(LED_B_PIN, HIGH);

  switch (color) {
    case LedColor::WHITE:
      {
        break;
      }
    case LedColor::RED:
      {
        digitalWrite(LED_R_PIN, LOW);
        break;
      }
    case LedColor::GREEN:
      {
        digitalWrite(LED_G_PIN, LOW);
        break;
      }
    case LedColor::BLUE:
      {
        digitalWrite(LED_B_PIN, LOW);
        break;
      }
    default:
      {
        // default OFF.
        break;
      }
  }
}

// Method used to determine a given device is present on the Nexus. Useful to prevent errors in the program.
bool checkI2CDevice(byte address) {
  uint8_t error;
  Wire.beginTransmission(address);
  error = Wire.endTransmission();

  if (error == 0) {
    return true;
  } else {
    Serial.printf("--WARNING-- I2C Device at address 0x%02X NOT found! errorcode: %d\n", address, error);
  }

  return false;
}
