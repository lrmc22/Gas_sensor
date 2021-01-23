#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//Sensors headers
#include "Adafruit_SGP30.h"
#include "DHT.h"
#include "DHT_U.h"

//generated pb interface
#include <pb_encode.h>
#include "interface.pb.h"

#include <time.h>


#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN 2 // From esp8266 pin diagram

//Router parameters
const char* ssid        = "Bbox-8884FC0B"; // Enter your WiFi name
const char* password    =  "A5C5CCF55EA9EA45C2ACCF259E21EC"; // Enter WiFi password
const char* mqttServer  = "192.168.1.78"; //RPI MQTT Server IP

//PB variables
bool status;

//Our two communicating objects in MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Topics
const char* temperatureTopic = "/senso-care/sensors/temperature-superesp8266";
const char* humidityTopic    = "/senso-care/sensors/humidity-superesp8266";
const char* tvocTopic        = "/senso-care/sensors/TVOC-superesp8266";
const char* eCO2Topic        = "/senso-care/sensors/eCO2-superesp8266";
const char* rawH2Topic       = "/senso-care/sensors/rawH2-superesp8266";
const char* rawEthanolTopic  = "/senso-care/sensors/rawEthanol-superesp8266";

//sgp object
Adafruit_SGP30 sgp;

//dht object
DHT dht(DHTPIN, DHTTYPE);


/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

//Sent parameters
void wifiSetup()
{
  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  //Init wifi connection to router Bbox in our case
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

//We wait until connection is done
while (WiFi.status() != WL_CONNECTED)
{
  delay(500);
  Serial.print(".");
}

  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  configTime(2*3600, 0, mqttServer);
}
void reconnect() {

  // Loop until we're reconnected
  while (!client.connected())
  {

    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
     {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    }else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void sendSerialised(float value, const char* topic)
{
  uint8_t buffer[sensocare_messages_Measure_size];
  sensocare_messages_Measure measure = sensocare_messages_Measure_init_zero;
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  //Set protobuf structure values
  measure.timestamp = (uint64_t) time(NULL);
  measure.value = value;

//Encoding Measure fields
  status = pb_encode(&stream, sensocare_messages_Measure_fields, &measure);

  if (!status)
  {
    Serial.println("Encoding failed"); // Fail
  }
  //Prinf buffer in serial monitor
  for(int i = 0; i < sensocare_messages_Measure_size; i++ )
  {
    Serial.print(buffer[i]);
  }
  Serial.print(" ");
  // Envoyer le buffer
  client.publish(topic, (char*)buffer);
}
void setup() {

  //Init serial communication with 115200 baudrate
  Serial.begin(115200);
  Serial.println(F("DHTxx test!"));

  // init dht 
  dht.begin();

  while (!Serial) { delay(10); } // Wait for serial console to open!

  Serial.println(" SGP30 test ! ");
  
  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1); 
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);

  // If you have a baseline measurement from before you can assign it to start, to 'self-calibrate'
  //sgp.setIAQBaseline(0x8E68, 0x8F41);  // Will vary for each sensor!
}

int counter = 0;
void loop() {
 
  delay(1000);
  // Read humidity 
  float humidity    = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float temperature = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

 // print temperature and humidity
  Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.print(F("%  Temperature: "));
  Serial.println(temperature);

  //We set sgp humidity to get more accurate values 
  sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\t");
  Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");

  if (! sgp.IAQmeasureRaw()) {
    Serial.println("Raw Measurement failed");
    return;
  }
  Serial.print("Raw H2 "); Serial.print(sgp.rawH2); Serial.print(" \t");
  Serial.print("Raw Ethanol "); Serial.print(sgp.rawEthanol); Serial.println("");
 
 
  delay(1000);

  counter++;
  if (counter == 30) {
    counter = 0;

    uint16_t TVOC_base, eCO2_base;
    if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
      Serial.println("Failed to get baseline readings");
      return;
    }
    Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
    Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
  }

// We send serialised data to the specific mqtt server
  sendSerialised(temperature,temperatureTopic);
  sendSerialised(humidity,humidityTopic);
  sendSerialised(sgp.TVOC,tvocTopic);
  sendSerialised(sgp.eCO2,eCO2Topic);
  sendSerialised(sgp.rawH2,rawH2Topic);
  sendSerialised(sgp.rawEthanol,rawEthanolTopic);
}