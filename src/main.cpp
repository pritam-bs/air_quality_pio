#include <Arduino.h>
#include <MQUnifiedsensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

struct Settings
{
  char gatewayIP[16];
  char port[5];
  char section[7];
  int delay;
  float R0;
} settings;

// Button press counter to enable setup
bool isSetup = false;
bool isCalibration = false;
size_t setupEnableCount = 5;

#define SETUP_BUTTON_PIN 12
#define CALIBRATION_BUTTON_PIN 14

struct Button
{
  const uint8_t PIN;
  uint32_t numberKeyPresses;
};

Button setupButton = {SETUP_BUTTON_PIN, 0};
Button calibrationButton = {CALIBRATION_BUTTON_PIN, 0};

void sendRequest(float airQuality, float temperature, float humidity)
{
  if (WiFi.status() == WL_CONNECTED)
  {

    WiFiClient client;
    HTTPClient http;

    String host = settings.gatewayIP;
    int port = String(settings.port).toInt();
    String uri = "/environment";

    String httpRequestData;
    StaticJsonDocument<96> doc;

    doc["air_quality"] = airQuality;
    doc["temperature"] = temperature;
    doc["humidity"] = humidity;
    doc["section"] = "female";

    if (!serializeJson(doc, httpRequestData))
    {
      Serial.println("Json serialize failed");
    }

    http.begin(client, host, port, uri, false);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(httpRequestData);

    // httpResponseCode will be negative on error
    if (httpResponseCode > 0)
    {
      // HTTP header has been send and Server response header has been handled
      Serial.printf("[HTTP] POST... code: %d\n", httpResponseCode);

      // file found at server
      if (httpResponseCode == HTTP_CODE_OK)
      {
        const String &payload = http.getString();
        Serial.println("Received response:\n<<");
        Serial.println(payload);
        Serial.println(">>");
      }
    }
    else
    {
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    http.end();
  }
}

void IRAM_ATTR setupButtonPressed()
{
  setupButton.numberKeyPresses = setupButton.numberKeyPresses + 1;
  Serial.println("Setup button press count: ");
  Serial.println(setupButton.numberKeyPresses);
  if (setupButton.numberKeyPresses >= setupEnableCount)
  {
    isSetup = true;
  }
}

void IRAM_ATTR calibrationButtonPressed()
{
  calibrationButton.numberKeyPresses = calibrationButton.numberKeyPresses + 1;
  Serial.println("Calibration button press count: ");
  Serial.println(calibrationButton.numberKeyPresses);
  if (calibrationButton.numberKeyPresses >= setupEnableCount)
  {
    isCalibration = true;
  }
}

void resetSetup()
{
  isSetup = false;
  isCalibration = false;
  setupButton.numberKeyPresses = 0;
  calibrationButton.numberKeyPresses = 0;
}

/************************Hardware Related Macros************************************/
#define Board ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.
#define Pin (25)         // IO25 for your ESP32 WeMos Board, pinout here: https://i.pinimg.com/originals/66/9a/61/669a618d9435c702f4b67e12c40a11b8.jpg
/***********************Software Related Macros************************************/
#define Type ("MQ-3")            // MQ3 or other MQ Sensor, if change this verify your a and b values.
#define Voltage_Resolution (3.3) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define ADC_Bit_Resolution (12)  // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define RatioMQ135CleanAir (3.6) // RS / R0 = 3.6 ppm

#define MQ135_DEFAULTPPM 399            // default ppm of CO2 for calibration
#define MQ135_DEFAULTRO 68550           // default Ro for MQ135_DEFAULTPPM ppm of CO2
#define MQ135_SCALINGFACTOR 116.6020682 // CO2 gas value
#define MQ135_EXPONENT -2.769034857     // CO2 gas value
#define MQ135_MAXRSRO 2.428             // for CO2
#define MQ135_MINRSRO 0.358             // for CO2

/// Parameters for calculating ppm of CO2 from sensor resistance
#define PARA 116.6020682
#define PARB 2.769034857

/// Parameters to model temperature and humidity dependence
#define CORA 0.00035
#define CORB 0.02718
#define CORC 1.39538
#define CORD 0.0018

#define DHTPIN 2 // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
#define DHTTYPE DHT11 // DHT 11
//#define DHTTYPE DHT22 // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

/*****************************Globals***********************************************/
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
/*****************************Globals***********************************************/

/**************************************************************************/
/*!
@brief  Get the correction factor to correct for temperature and humidity
@param[in] t  The ambient air temperature
@param[in] h  The relative humidity
@return The calculated correction factor
*/
/**************************************************************************/
float getCorrectionFactor(float t, float h)
{
  return CORA * t * t - CORB * t + CORC - (h - 33.) * CORD;
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value corrected
        for temp/hum
@param[in] t  The ambient air temperature
@param[in] h  The relative humidity
@return The corrected sensor resistance kOhm
*/
/**************************************************************************/
float getCorrectedResistance(long resvalue, float t, float h)
{
  return resvalue / getCorrectionFactor(t, h);
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected
        for temp/hum
@param[in] t  The ambient air temperature
@param[in] h  The relative humidity
@return The ppm of CO2 in the air
*/
/**************************************************************************/
float getCorrectedPPM(long resvalue, float t, float h, long ro)
{
  return PARA * pow((getCorrectedResistance(resvalue, t, h) / ro), -PARB);
}

void setup()
{
  // Init the serial port communication - to debug the library
  Serial.begin(115200);

  resetSetup();

  pinMode(setupButton.PIN, INPUT_PULLUP);
  pinMode(calibrationButton.PIN, INPUT_PULLUP);
  attachInterrupt(setupButton.PIN, setupButtonPressed, FALLING);
  attachInterrupt(calibrationButton.PIN, calibrationButtonPressed, FALLING);

  // Delay to push SETUP button
  Serial.println("Press setup or calibration button for 5 times");
  for (int sec = 5; sec > 0; sec--)
  {
    Serial.print(sec);
    Serial.print("..");
    delay(1000);
  }

  Serial.println("");
  Serial.println("Setup button press count: ");
  Serial.println(setupButton.numberKeyPresses);
  Serial.println("Calibration button press count: ");
  Serial.println(calibrationButton.numberKeyPresses);

  EEPROM.begin(512);
  EEPROM.get(0, settings);
  Serial.println("Settings loaded");

  settings.section[6] = '\0';
  settings.gatewayIP[15] = '\0';
  settings.port[4] = '\0';

  if (isCalibration)
  {
    /*****************************  MQ Calibration ********************************************/
    // Explanation:
    // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
    // and on clean air (Calibration conditions), setting up R0 value.
    // We recomend executing this routine only on setup in laboratory conditions.
    // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
    // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor

    Serial.print("Calibrating please wait.");
    float calcR0 = 0;
    for (int i = 1; i <= 10; i++)
    {
      MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
      calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
      Serial.print(".");
    }
    float R0 = calcR0 / 10.0;
    MQ135.setR0(R0);
    Serial.println("  done!.");

    if (isinf(calcR0))
    {
      Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
      while (1)
        ;
    }
    if (calcR0 == 0)
    {
      Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
      while (1)
        ;
    }
    settings.R0 = R0;
    /*****************************  MQ CAlibration ********************************************/
  }

  if (isSetup)
  {
    Serial.println("Setup Mode");
    WiFiManager wm;
    WiFiManagerParameter gatewayIP("gatewayIP", "Gateway IP", "192.168.10.104", 15);
    WiFiManagerParameter gatewayPort("gatewayPort", "Gateway Port", "5454", 4);
    WiFiManagerParameter facilitySection("facilitySection", "Facility Section", "female", 6);
    WiFiManagerParameter updateFrequency("updateFrequency", "Update Frequency", "15", 2);

    wm.addParameter(&gatewayIP);
    wm.addParameter(&gatewayPort);
    wm.addParameter(&facilitySection);
    wm.addParameter(&updateFrequency);

    // SSID & password parameters already included
    wm.startConfigPortal("MLBD_Air_Quality", "Mlbd@1234");

    strncpy(settings.gatewayIP, gatewayIP.getValue(), 16);
    strncpy(settings.port, gatewayPort.getValue(), 5);
    strncpy(settings.section, facilitySection.getValue(), 7);
    settings.delay = String(updateFrequency.getValue()).toInt();

    settings.section[6] = '\0';
    settings.gatewayIP[15] = '\0';
    settings.port[4] = '\0';

    Serial.print("Gateway IP: ");
    Serial.println(settings.gatewayIP);
    Serial.print("Gateway Port: ");
    Serial.println(settings.port);
    Serial.print("Section: ");
    Serial.println(settings.section);
  }

  EEPROM.put(0, settings);
  if (EEPROM.commit())
  {
    Serial.println("Settings saved");
  }
  else
  {
    Serial.println("EEPROM error");
  }

  resetSetup();

  // Init serial port
  dht.begin();

  // Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setA(102.2);
  MQ135.setB(-2.473); // Configure the equation to to calculate NH4 concentration

  /*
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937
  Alcohol  | 77.255 | -3.18
  CO2      | 110.47 | -2.862
  Toluen  | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Aceton  | 34.668 | -3.369
  */

  /*****************************  MQ Init ********************************************/
  // Remarks: Configure the pin of arduino as input.
  /************************************************************************************/
  MQ135.init();

  // If the RL value is different from 10K please assign your RL value with the following method:
  MQ135.setRL(1);
  MQ135.serialDebug(true);

  // Set delay between sensor readings based on sensor details.
  if (!isnan(settings.delay))
  {
    delayMS = 5 * 60 * 1000;
  }
  else
  {
    delayMS = String(settings.delay).toInt() * 60 * 1000;
  }
}

void loop()
{
  delay(delayMS); // Sampling frequency

  // if you want to apply corelation factor, you will add in this program the temperature and humidity sensor
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  float temperature = event.temperature;
  float humidity = event.relative_humidity;

  float cFactor = 0;
  if (!isnan(temperature) && !isnan(humidity))
    cFactor = getCorrectionFactor(event.temperature, event.relative_humidity);
  Serial.print("Correction Factor: ");
  Serial.println(cFactor);
  MQ135.update();                                      // Update data, the arduino will read the voltage from the analog pin
  float airQuality = MQ135.readSensor(false, cFactor); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  MQ135.serialDebug();                                 // Will print the table on the serial port

  sendRequest(airQuality, temperature, humidity);
}