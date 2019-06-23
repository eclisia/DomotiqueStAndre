#include <HTTPClient.h>

#include <WiFi.h>
#include "InfluxArduino.hpp"
#include "InfluxCert.hpp"
#include "Adafruit_BME680.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <OneWire.h>
#include <DallasTemperature.h>


//#define LED_PIN 13
InfluxArduino influx;
Adafruit_BME680 bme;

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

float hum_weighting = 0.25; // so hum effect is 25% of the total air quality score
float gas_weighting = 0.75; // so gas effect is 75% of the total air quality score

float hum_score, gas_score;
float gas_reference = 250000;
float hum_reference = 40;
int   getgasreference_count = 0;




//Information to setup DS18B20+ sensor
// Data wire is connected to GPIO15
#define ONE_WIRE_BUS 15
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

DeviceAddress sensor1 = { 0x28, 0x4D, 0x12, 0x4F, 0x07, 0x00, 0x00, 0xDE };

//192.168.1.99:8086

//connection/ database stuff that needs configuring
const char WIFI_NAME[] = "Bbox-Florelie";
const char WIFI_PASS[] = "mot de passe wifi";
const char INFLUX_DATABASE[] = "grafana_florelie_weather";
const char INFLUX_IP[] = "192.168.1.99";
const char INFLUX_USER[] = "florent";
const char INFLUX_PASS[] = "florent";
const char INFLUX_MEASUREMENT[] = "DS1820_meas";
const char INFLUX_MEASUREMENT2[] = "BME680";

unsigned long DELAY_TIME_US = 5 * 1000 * 1000; //how frequently to send data, in microseconds
unsigned long count = 0;                       //a variable that we gradually increase in the loop

void setup()
{
  Serial.begin(115200);

  while (!Serial);
  Serial.println(F("BME680 test"));

  
  Wire.begin();
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  } else Serial.println("Found a sensor");

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_2X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_2X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms
  // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
//  GetGasReference();


  


  //Start Temperature sensor
  sensors.begin();


  WiFi.begin(WIFI_NAME, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected!");
  influx.configure(INFLUX_DATABASE, INFLUX_IP); //third argument (port number) defaults to 8086
  influx.authorize(INFLUX_USER, INFLUX_PASS);   //if you have set the Influxdb .conf variable auth-enabled to true, uncomment this
 //influx.addCertificate(ROOT_CERT);             //uncomment if you have generated a CA cert and copied it into InfluxCert.hpp
  Serial.print("Using HTTPS: ");
  Serial.println(influx.isSecure()); //will be true if you've added the InfluxCert.hpp file.
}

void loop()
{
  unsigned long startTime = micros();

  char tags[16];
  char fields[128];
  char formatString[] = "temperature_DS1820=%0.3f,pressure_DS1820=%0.3f,humidity_DS1820=%0.3f,gas_resistance_DS1820=%0.3f";
  char formatStringBME[] = "temperature_BME=%0.3f,pressure_BME=%0.3f,humidity_BME=%0.3f,gas_resistance_BME=%0.3f";

 if (bme.performReading())
  {
    sprintf(tags, "read_ok=true");
    //sprintf(fields, formatStringBME, bme.temperature, bme.pressure / 100000.0, bme.humidity, bme.gas_resistance / 1000.0);
  
//     pressure /100 pour donner des HectoPascal (valeur tunée pour coller avec des autres station météo)
    sprintf(fields, formatStringBME, bme.temperature, bme.pressure / 100.0, bme.humidity, bme.gas_resistance / 1000.0);
  }
  else
  {
    sprintf(tags, "read_ok=false");
    sprintf(fields, formatStringBME, -1.0, -1.0, -1.0, -1.0);
    Serial.println("Failed to perform reading :(");
  }
  bool writeSuccessful = influx.write(INFLUX_MEASUREMENT2, tags, fields);

  if (!writeSuccessful)
  {
    Serial.print("error: ");
    Serial.println(influx.getResponse());
  }
  else
  {
    delay(50);
    //digitalWrite(LED_PIN, LOW);
  }

  while ((micros() - startTime) < DELAY_TIME_US)
  {
    //kill time until we're ready for the next reading
  }

  

  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");

   Serial.println(".");
  Serial.print("Sensor 1(*C): ");
  Serial.print(sensors.getTempC(sensor1));
   
   Serial.println(".");
  sprintf(tags, "read_ok=true");
  sprintf(fields, formatString, sensors.getTempC(sensor1),0,0,0);
  


  
  bool writeSuccessfulDS = influx.write(INFLUX_MEASUREMENT, tags, fields);
  if (!writeSuccessfulDS)
  {
    Serial.print("error: ");
    Serial.println(influx.getResponse());
  }
  else
  {
    delay(50);
  }

  while ((micros() - startTime) < DELAY_TIME_US)
  {
    //kill time until we're ready for the next reading
  }
}
