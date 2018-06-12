

typedef struct  {
  float temp;  // temperature
  int heartbeat;
  int alert;
} RemoteSensorData_t;

typedef union float2bytes_t   // union consists of one variable represented in a number of different ways 
{ 
  float f; 
  char b[sizeof(float)]; 
}; 

#include <Wire.h>          // For I2C communication with panStamp, http://www.arduino.cc/en/Reference/Wire
#define PACKET_SIZE 8     // I2C packet size, slave will send 6 bytes to master
#define addrSlaveI2C 21    // ID of i@C slave
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_SSD1306.h>
 
#define OLED_Address 0x3C
Adafruit_SSD1306 oled(1);

#include <WifiLocation.h>

const char* ssid = "AndroidAPq";
const char* password = "abc123456";
//const char* mqtt_server = "34.208.26.153";
const char* mqtt_server = "35.165.59.45";
const char* googleApiKey = "AIzaSyAkeZw0sSp5b4xLBioa1xDLyw6Tn0jEKrM";
WifiLocation location(googleApiKey);


WiFiClient espClient;
PubSubClient client(espClient);

// Function prototype
bool getData(RemoteSensorData_t* sensorInfo);


void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
  //oled.begin(SSD1306_SWITCHCAPVCC, OLED_Address);
  //oled.clearDisplay();
  //oled.setTextSize(1);  
  Wire.begin(0,2);//Change to Wire.begin() for non ESP.
  setup_wifi();
   //location API Call
  location_t loc = location.getGeoFromWiFi();
  Serial.println("Location request data");
  Serial.println(location.getSurroundingWiFiJson());
  Serial.println("Latitude: " + String(loc.lat, 7));
  Serial.println("Longitude: " + String(loc.lon, 7));
  Serial.println("Accuracy: " + String(loc.accuracy));
  

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  reconnect();

}

void loop() {

 
 RemoteSensorData_t sensor1; 
  if( getData(&sensor1))
  { 
    
  Serial.print(sensor1.temp);
  Serial.print(',');
  Serial.print(sensor1.heartbeat);
  Serial.println();
  String payload = "{\"temperature\":";
  payload += sensor1.temp;
  payload+=",";
  payload += "\"BPM\":";
  payload+=sensor1.heartbeat;
  payload+=",";
  payload += "\"alert\":";
  payload+=sensor1.alert;
  payload += "}";
  //oled.writeFillRect(0,50,128,16,BLACK);
  //oled.setCursor(0,10);
  //oled.clearDisplay();
  //oled.print(sensor1.heartbeat);
  //oled.print(" BPM");
  //oled.display();
   if (client.connected()){
    Serial.print("Sending payload: ");
    Serial.println(payload);
    delay(1000);
    if (client.publish("temp", (char*) payload.c_str())) {
      Serial.println("Publish ok");
    }
    else {
      Serial.println("Publish failed");
    }
  }
   

  }
  else
  {
    Serial.println("No packet received");
  }
 
 
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}



// I2C Request data from slave
bool getData(RemoteSensorData_t* sensorInfo)
{
  float2bytes_t b2f;
  int beats=0;
  int alert=0;
  bool gotI2CPacket = false;    
  byte i=0;
  byte i2CData[PACKET_SIZE];  // don't use char data type
  
  Wire.requestFrom(addrSlaveI2C, PACKET_SIZE);    // request data from I2C slave
  
  while(Wire.available())    // Wire.available() will return the number of bytes available to read
  { 
    delay(100);
    i2CData[i++] = Wire.read(); // receive a byte of data
    gotI2CPacket = true;  // Flag to indicate sketch received I2C packet
  }

  // If we got an I2C packet, we can extract the values
  if(gotI2CPacket)
  {
    gotI2CPacket = false;  // Reset flag
 // Get floating temperature    
    b2f.b[0] = i2CData[0];
    b2f.b[1] = i2CData[1];
    b2f.b[2] = i2CData[2];
    b2f.b[3] = i2CData[3];
    beats|=((int)i2CData[4])<<8;
    beats|=((int)i2CData[5]);
     alert|=((int)i2CData[6])<<8;
    alert|=((int)i2CData[7]);
    sensorInfo->temp = b2f.f; 
    sensorInfo->heartbeat=beats;
    sensorInfo->alert=alert;
    return true;
  }  // end got packet
  else
  { return false; } // No Packet received
  
} // end getData



void setup_wifi(){

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
 float2bytes_t b2f;

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
   b2f.b[i]= payload[i];
  }
  
 Serial.println(b2f.f);
}

void reconnect() {
  // Loop until we're reconnected
   while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("demo", "Connected!");
      // ... and resubscribe
      client.subscribe("demo");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
