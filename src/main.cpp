#include <Arduino.h>
#include <MQUnifiedsensor.h>
#include <DHT.h>
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
bool isCalibration = false;
size_t setupEnableCount = 5;

#define CALIBRATION_BUTTON_PIN 0
#define LED_PIN 2

struct Button
{
  const uint8_t PIN;
  uint32_t numberKeyPresses;
};

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
    doc["section"] = settings.section;

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

void IRAM_ATTR calibrationButtonPressed()
{
  calibrationButton.numberKeyPresses = calibrationButton.numberKeyPresses + 1;
  Serial.println("Calibration button press count: ");
  Serial.println(calibrationButton.numberKeyPresses);
  if (calibrationButton.numberKeyPresses >= setupEnableCount)
  {
    isCalibration = true;
    digitalWrite(LED_PIN, HIGH);
  }
}

void resetSetup()
{
  isCalibration = false;
  calibrationButton.numberKeyPresses = 0;
  digitalWrite(LED_PIN, LOW);
}

/************************Hardware Related Macros************************************/
#define Board ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.
#define Pin (34)         // IO25 for your ESP32 WeMos Board, pinout here: https://i.pinimg.com/originals/66/9a/61/669a618d9435c702f4b67e12c40a11b8.jpg
/***********************Software Related Macros************************************/
#define Type "MQ-135"            // MQ3 or other MQ Sensor, if change this verify your a and b values.
#define Voltage_Resolution (3.3) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define ADC_Bit_Resolution (12)  // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define RatioMQ135CleanAir (3.6) // RS / R0 = 3.6 ppm

#define DHTPIN 25 // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
#define DHTTYPE DHT11 // DHT 11
// #define DHTTYPE DHT22 // DHT 22 (AM2302)
// #define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

/*****************************Globals***********************************************/
DHT dht(DHTPIN, DHTTYPE);
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
/// Parameters to model temperature and humidity dependence
#define CORA 0.00035
#define CORB 0.02718
#define CORC 1.39538
#define CORD 0.0018
float getCorrectionFactor(float t, float h)
{
  return CORA * t * t - CORB * t + CORC - (h - 33.0) * CORD;
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

/*****************************  MQ Calibration ********************************************/
void startCalibration()
{
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
    digitalWrite(LED_PIN, HIGH);
  }
  if (calcR0 == 0)
  {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    digitalWrite(LED_PIN, HIGH);
  }
  settings.R0 = R0;
  EEPROM.put(0, settings);
  if (EEPROM.commit())
  {
    Serial.println("Settings saved");
  }
  else
  {
    Serial.println("EEPROM error");
    digitalWrite(LED_PIN, HIGH);
  }
}
/*****************************  MQ CAlibration ********************************************/

void setup()
{
  // Init the serial port communication - to debug the library
  Serial.begin(115200);

  resetSetup();
  pinMode(calibrationButton.PIN, INPUT_PULLUP);
  attachInterrupt(calibrationButton.PIN, calibrationButtonPressed, FALLING);

  pinMode(LED_PIN, OUTPUT);

  EEPROM.begin(512);
  EEPROM.get(0, settings);
  Serial.println("Settings loaded");

  settings.section[6] = '\0';
  settings.gatewayIP[15] = '\0';
  settings.port[4] = '\0';

  WiFiManager wm;
  WiFiManagerParameter gatewayIP("gatewayIP", "Gateway IP", settings.gatewayIP, 15);
  WiFiManagerParameter gatewayPort("gatewayPort", "Gateway Port", settings.port, 4);
  WiFiManagerParameter facilitySection("facilitySection", "Facility Section", settings.section, 6);
  WiFiManagerParameter updateFrequency("updateFrequency", "Update Frequency", String(settings.delay).c_str(), 2);
  WiFiManagerParameter valueOfR0("valueOfR0", "R0 Value(00.00)", String(settings.R0).c_str(), 5);

  wm.addParameter(&gatewayIP);
  wm.addParameter(&gatewayPort);
  wm.addParameter(&facilitySection);
  wm.addParameter(&updateFrequency);
  wm.addParameter(&valueOfR0);

  // SSID & password parameters already included
  wm.setConfigPortalTimeout(120);
  wm.autoConnect("MLBD_Air_Quality", "Mlbd@1234");

  strncpy(settings.gatewayIP, gatewayIP.getValue(), 16);
  strncpy(settings.port, gatewayPort.getValue(), 5);
  strncpy(settings.section, facilitySection.getValue(), 7);
  settings.delay = String(updateFrequency.getValue()).toInt();
  settings.R0 = String(valueOfR0.getValue()).toFloat();

  settings.section[6] = '\0';
  settings.gatewayIP[15] = '\0';
  settings.port[4] = '\0';

  Serial.print("Gateway IP: ");
  Serial.println(settings.gatewayIP);
  Serial.print("Gateway Port: ");
  Serial.println(settings.port);
  Serial.print("Section: ");
  Serial.println(settings.section);
  Serial.print("R0: ");
  Serial.println(settings.R0);

  EEPROM.put(0, settings);
  if (EEPROM.commit())
  {
    Serial.println("Settings saved");
  }
  else
  {
    Serial.println("EEPROM error");
  }

  // Init serial port
  dht.begin();

  // Set math model to calculate the PPM concentration and the value of constants
  Serial.println("Saved R0 value: ");
  Serial.println(settings.R0);
  if (isnan(String(settings.R0).toFloat()))
  {
    MQ135.setR0(10.00);
    Serial.println("Default R0 is set: ");
    Serial.println(10.00);
  }
  else
  {
    MQ135.setR0(settings.R0);
    Serial.println("R0 from settings is set: ");
    Serial.println(settings.R0);
  }

  Serial.println("Update frequency: ");
  Serial.println(delayMS);
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
  Serial.println("Saved delay: ");
  Serial.println(settings.delay);
  if (isnan(String(settings.delay).toInt()))
  {
    delayMS = 5 * 60 * 1000;
    Serial.println("Default delay is set: ");
    Serial.println(delayMS);
  }
  else
  {
    delayMS = String(settings.delay).toInt() * 60 * 1000;
    Serial.println("Delay from settings is set: ");
    Serial.println(delayMS);
  }

  Serial.println("Update frequency: ");
  Serial.println(delayMS);
}

void loop()
{
  Serial.println("Looping....");

  if (isCalibration == true)
  {
    resetSetup();
    startCalibration();
  }

  delay(delayMS); // Sampling frequency
  Serial.println("Going to measure air quality....");
  // if you want to apply corelation factor, you will add in this program the temperature and humidity sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  Serial.println("Temperature");
  Serial.println(temperature);
  Serial.println("Humidity");
  Serial.println(humidity);
  float cFactor = 0;
  if (!isnan(temperature) && !isnan(humidity))
    cFactor = getCorrectionFactor(temperature, humidity);
  Serial.print("Correction Factor: ");
  Serial.println(cFactor);
  MQ135.update();                                      // Update data, the arduino will read the voltage from the analog pin
  float airQuality = MQ135.readSensor(false, cFactor); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  MQ135.serialDebug();                                 // Will print the table on the serial port

  sendRequest(airQuality, temperature, humidity);
}