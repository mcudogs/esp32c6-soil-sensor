// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief This example demonstrates Zigbee temperature and humidity sensor Sleepy device.
 *
 * The example demonstrates how to use Zigbee library to create an end device temperature and humidity sensor.
 * The sensor is a Zigbee end device, which is reporting data to the Zigbee network.
 *
 * Proper Zigbee mode must be selected in Tools->Zigbee mode
 * and also the correct partition scheme must be selected in Tools->Partition Scheme.
 *
 * Please check the README.md for instructions and more detailed description.
 *
 * Created by Jan Procházka (https://github.com/P-R-O-C-H-Y/)
 */

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"

#define USE_GLOBAL_ON_RESPONSE_CALLBACK 1  // Set to 0 to use local callback specified directly for the endpoint.

/* Zigbee temperature + humidity sensor configuration */
#define TEMP_SENSOR_ENDPOINT_NUMBER 10

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  55         /* Sleep for 55s will + 5s delay for establishing connection => data reported every 1 minute */
#define REPORT_TIMEOUT 1000       /* Timeout for response from coordinator in ms */

uint8_t button = BOOT_PIN;

ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);

uint8_t dataToSend = 3;  // Temperature and humidity values are reported in same endpoint, so 2 values are reported
bool resend = false;
const int BatteryPin = A0; 
const int SoilResistancePin = A1; 
struct BatteryCalibrationPoint {
  int raw;
  float voltage;
};

// Raw ADC readings measured against DMM voltage.
// Table must be ordered from lowest raw/voltage to highest raw/voltage.
const BatteryCalibrationPoint BATTERY_CAL[] = {
  {1324, 3.40f},
  {1608, 3.60f},
  {1680, 3.70f},
  {1840, 4.00f},
  {2028, 4.20f}
};

const int BATTERY_CAL_COUNT = sizeof(BATTERY_CAL) / sizeof(BATTERY_CAL[0]);

float batteryVoltageFromRaw(int raw) {
  // Clamp below lowest calibration point
  if (raw <= BATTERY_CAL[0].raw) {
    return BATTERY_CAL[0].voltage;
  }

  // Clamp above highest calibration point
  if (raw >= BATTERY_CAL[BATTERY_CAL_COUNT - 1].raw) {
    return BATTERY_CAL[BATTERY_CAL_COUNT - 1].voltage;
  }

  // Interpolate between the two calibration points surrounding raw
  for (int i = 0; i < BATTERY_CAL_COUNT - 1; i++) {
    int rawLow = BATTERY_CAL[i].raw;
    int rawHigh = BATTERY_CAL[i + 1].raw;

    if (raw >= rawLow && raw <= rawHigh) {
      float voltageLow = BATTERY_CAL[i].voltage;
      float voltageHigh = BATTERY_CAL[i + 1].voltage;

      float fraction = (float)(raw - rawLow) / (float)(rawHigh - rawLow);
      return voltageLow + fraction * (voltageHigh - voltageLow);
    }
  }

  // Should never get here, but return lowest voltage as a safe fallback
  return BATTERY_CAL[0].voltage;
}

uint8_t liIonPercentFromVoltage(float voltage) {
  // Clamp full and empty limits
  if (voltage >= 4.20f) return 100;
  if (voltage <= 3.40f) return 0;

  // Piecewise approximation for a single-cell Li-ion/LiPo battery.
  // This is intentionally conservative near the bottom.
  if (voltage >= 4.00f) {
    return 80 + (uint8_t)roundf((voltage - 4.00f) * (20.0f / 0.20f));
  }

  if (voltage >= 3.85f) {
    return 60 + (uint8_t)roundf((voltage - 3.85f) * (20.0f / 0.15f));
  }

  if (voltage >= 3.75f) {
    return 40 + (uint8_t)roundf((voltage - 3.75f) * (20.0f / 0.10f));
  }

  if (voltage >= 3.65f) {
    return 20 + (uint8_t)roundf((voltage - 3.65f) * (20.0f / 0.10f));
  }

  if (voltage >= 3.50f) {
    return 10 + (uint8_t)roundf((voltage - 3.50f) * (10.0f / 0.15f));
  }

  // 3.40V to 3.50V = 0% to 10%
  return (uint8_t)roundf((voltage - 3.40f) * (10.0f / 0.10f));
}

/************************ Callbacks *****************************/
#if USE_GLOBAL_ON_RESPONSE_CALLBACK
void onGlobalResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status, uint8_t endpoint, uint16_t cluster) {
  Serial.printf("Global response command: %d, status: %s, endpoint: %d, cluster: 0x%04x\r\n", command, esp_zb_zcl_status_to_name(status), endpoint, cluster);
  if ((command == ZB_CMD_REPORT_ATTRIBUTE) && (endpoint == TEMP_SENSOR_ENDPOINT_NUMBER)) {
    switch (status) {
      case ESP_ZB_ZCL_STATUS_SUCCESS: dataToSend--; break;
      case ESP_ZB_ZCL_STATUS_FAIL:    resend = true; break;
      default:                        break;  // add more statuses like ESP_ZB_ZCL_STATUS_INVALID_VALUE, ESP_ZB_ZCL_STATUS_TIMEOUT etc.
    }
  }
}
#else
void onResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status) {
  Serial.printf("Response command: %d, status: %s\r\n", command, esp_zb_zcl_status_to_name(status));
  if (command == ZB_CMD_REPORT_ATTRIBUTE) {
    switch (status) {
      case ESP_ZB_ZCL_STATUS_SUCCESS: dataToSend--; break;
      case ESP_ZB_ZCL_STATUS_FAIL:    resend = true; break;
      default:                        break;  // add more statuses like ESP_ZB_ZCL_STATUS_INVALID_VALUE, ESP_ZB_ZCL_STATUS_TIMEOUT etc.
    }
  }
}
#endif

int readBatteryRawAveraged(int pin) {
  const int samples = 20;
  long total = 0;

  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delay(2);
  }

  return total / samples;
}

/************************ Temp sensor *****************************/
static void meausureAndSleep(void *arg) {
  // Measure temperature sensor value
  float temperature = temperatureRead();
  int soilrawvalue = analogRead(SoilResistancePin);
  float humidity = map(soilrawvalue, 2646, 920, 0, 100);
  int battrawvalue = readBatteryRawAveraged(BatteryPin);
  //int battrawvalue = analogRead(BatteryPin);
  float voltage = batteryVoltageFromRaw(battrawvalue);
  uint8_t battery = liIonPercentFromVoltage(voltage);
  //float voltage = (battrawvalue / 3218.0) * 2 * 3.3;
  //float battery = map(voltage * 100, 350, 420, 0, 100);
  // Use temperature value as humidity value to demonstrate both temperature and humidity
  zbTempSensor.setBatteryPercentage(battery);
  zbTempSensor.setTemperature(temperature);
  zbTempSensor.setHumidity(humidity);

  // Send battery and sensor reports
  zbTempSensor.reportBatteryPercentage();
  zbTempSensor.report();
  Serial.printf(
  "Reported temperature: %.2f°C, Battery: %u%%, Voltage: %.3fV, Humidity: %.2f%%, Raw Battery: %d, soil raw value: %d\r\n",
  temperature,
  battery,
  voltage,
  humidity,
  battrawvalue,
  soilrawvalue
  );
  //Serial.printf("Reported temperature: %.2f°C, Battery: %.2f%%, Humidity: %.2f%%, Raw Battery: %u, soil raw value: %u\r\n", temperature, battery, humidity, battrawvalue, soilrawvalue);

  unsigned long startTime = millis();
  const unsigned long timeout = REPORT_TIMEOUT;

  Serial.printf("Waiting for data report to be confirmed \r\n");
  // Wait until data was successfully sent
  int tries = 0;
  const int maxTries = 3;
  while (dataToSend != 0 && tries < maxTries) {
    if (resend) {
      Serial.println("Resending data on failure!");
      resend = false;
      dataToSend = 3;
      zbTempSensor.reportBatteryPercentage();
      zbTempSensor.report();
    }
    if (millis() - startTime >= timeout) {
     Serial.println("\nReport timeout! Report Again");
      dataToSend = 3;

      zbTempSensor.reportBatteryPercentage();
      zbTempSensor.report();

      startTime = millis();
      tries++;
    }
    
    Serial.printf(".");
    delay(50);  // 50ms delay to avoid busy-waiting
  }

  // Put device to deep sleep after data was sent successfully or timeout
  delay(50);  // 50ms delay to avoid busy-waiting
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
}

/********************* Arduino functions **************************/
void setup() {
  Serial.begin(115200);

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  // Configure the wake up source and set to wake up every 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // Optional: set Zigbee device name and model
  zbTempSensor.setManufacturerAndModel("Espressif", "SleepyZigbeeTempSensor");

  // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
  zbTempSensor.setMinMaxValue(10, 50);

  // Set default (initial) value for the temperature sensor to 10.0°C to match the minimum temperature measurement value (default value is 0.0°C)
  zbTempSensor.setDefaultValue(10.0);

  // Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
  zbTempSensor.setTolerance(1);

  // Set power source to battery, battery percentage and battery voltage (now 100% and 3.5V for demonstration)
  // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) or zbTempSensor.setBatteryVoltage(voltage) anytime after Zigbee.begin()
  zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100, 42);
  
  // Add humidity cluster to the temperature sensor device with min, max, tolerance and default values
  zbTempSensor.addHumiditySensor(0, 100, 1, 0.0);

  // Set callback for default response to handle status of reported data, there are 2 options.

#if USE_GLOBAL_ON_RESPONSE_CALLBACK
  // Global callback for all endpoints with more params to determine the endpoint and cluster in the callback function.
  Zigbee.onGlobalDefaultResponse(onGlobalResponse);
#else
  // Callback specified for endpoint
  zbTempSensor.onDefaultResponse(onResponse);
#endif

  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbTempSensor);

  // Create a custom Zigbee configuration for End Device with keep alive 10s to avoid interference with reporting data
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
  zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;

  // For battery powered devices, it can be better to set timeout for Zigbee Begin to lower value to save battery
  // If the timeout has been reached, the network channel mask will be reset and the device will try to connect again after reset (scanning all channels)
  //Zigbee.setTimeout(10000);  // Set timeout for Zigbee Begin to 10s (default is 30s)
    delay(5000);

  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();
  }

  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }

  Serial.println();
  Serial.println("Successfully connected to Zigbee network");

  // Keep this sleepy end device awake long enough for Zigbee2MQTT to
  // complete the interview. Without this, Z2M may fail while requesting
  // the node descriptor.
  //Serial.println("Keeping device awake for Zigbee2MQTT interview...");
  //delay(60000);

  // Start sensor reading task after the interview window.
  xTaskCreate(meausureAndSleep, "temp_sensor_update", 4096, NULL, 10, NULL);
}


void loop() {
  // Checking button for factory reset
  if (digitalRead(button) == LOW) {  // Push button pressed
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(button) == LOW) {
      delay(50);
      if ((millis() - startTime) > 10000) {
        // If key pressed for more than 10secs, factory reset Zigbee and reboot
        Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
        delay(1000);
        // Optional set reset in factoryReset to false, to not restart device after erasing nvram, but set it to endless sleep manually instead
        Zigbee.factoryReset();
        Serial.println("Going to endless sleep, press RESET button or power off/on the device to wake up");
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        esp_deep_sleep_start();
      }
    }
  }
  delay(100);
}
