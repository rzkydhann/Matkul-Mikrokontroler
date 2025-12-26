#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>

int ledPin = 2;  
char myData = 0;

int motor1Pin1 = 27;
int motor1Pin2 = 26;
int enable1Pin = 12;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

const byte pin_rpm = 13;
int volatile rev = 0;
//int rpm = 0;

const char* ssid = "DANERA 5G"; // Enter your WiFi name
const char* password =  "danera1234"; // Enter WiFi password

#define mqttServer "broker.emqx.io"
#define mqttPort 1883

WiFiServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);

String Topic;
String Payload;

// constants
const int baud = 115200;       // serial baud rate


void setup()
{
  pinMode(ledPin, OUTPUT);  
  Serial.begin(115200);
  // sets the pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(pin_rpm, INPUT_PULLUP);
  //  pinMode(pin_rpm, INPUT);
  //attachInterrupt(digitalPinToInterrupt(pin_rpm), isr, RISING);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);

  // testing
  Serial.print("Testing DC Motor...");

  // Connect to WiFi network
  Serial.println();
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
 
  // Connect to Server IoT (CloudMQTT)
  client.setServer(mqttServer, mqttPort);
  client.setCallback(receivedCallback);
 
  while (!client.connected()) {
    Serial.println("Connecting to CLoud IoT ...");
 
//    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
     if (client.connect("iMCLab On/Off")) { 

      Serial.println("connected");
      Serial.print("Message received: ");
   
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
    client.subscribe("ImcRizky");
  }
}

void MotorOn()
{
  // Move DC motor forward with increasing speed
  dutyCycle = 205;
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  ledcWrite(pwmChannel, dutyCycle);
}

void MotorOff()
{
  dutyCycle = 0;
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  ledcWrite(pwmChannel, dutyCycle);
}


void receivedCallback(char* topic, byte* payload, unsigned int length) {

/* we got '1' -> MotorOn() */
  if ((char)payload[0] == '1') {
        MotorOn();
        Serial.println("Motor On");
  } 
  
/* we got '0' -> Motoroff */
  if ((char)payload[0] == '0') {
        MotorOff(); 
        Serial.println("Motor Off");  
  } 
}

void loop()
{

client.loop();

myData = int(Serial.read());

if (myData == '1'){  
  Serial.println("LED is on !!!");  
  digitalWrite(ledPin, HIGH);
  MotorOn();
}
else if (myData == '0'){  
  Serial.println("LED is off !!!");  
  digitalWrite(ledPin, LOW);
  MotorOff();
}


}