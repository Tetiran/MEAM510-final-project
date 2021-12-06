
#include <WiFi.h>
#include <WiFiUdp.h>
#include <tgmath.h>

#define MAX_VAL 32769.0

void update_servos();

char packetBuffer[255]; //buffer to hold incoming packet
const char* ssid     = "da_baby";
const char* password = "47543454";

WiFiUDP udp;
unsigned int UDPPort = 5005; // port for cans is 1510, port for robots is 2510
IPAddress ipTarget(192, 168, 1, 255); // 255 is a broadcast address to everyone at 192.168.1.xxx
IPAddress ipLocal(192, 168, 1, 6);  // replace with your IP address


//intiialize varaibles
double xVal, yVal;
int leftD, rightD; //0 is forward, 1 is backward

const int enablePin1 = 10;
const int enablePin2 = 5;
const int motor1Pin1 = 23;
const int motor1Pin2 = 18;
const int motor2Pin1 = 21;
const int motor2Pin2 = 22;

const int motor1Channel = 0;
const int motor2Channel = 1;
const int resolution = 13;
const int freq = 50;
long heartbeat = 0;

float scaler(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void update_servos (float angle_cmd, float vel_cmd) {

  float cmd_scale = abs(angle_cmd) + abs(vel_cmd);
  if (cmd_scale < 1.0) {
    cmd_scale = 1.0;
  }

  float ctrl_l = (vel_cmd - angle_cmd) / cmd_scale;
  float ctrl_r = (vel_cmd + angle_cmd) / cmd_scale;

  double sig_l = scaler(abs(ctrl_l), 0.0, 1.0, 0.0, 8191.0);
  double sig_r = scaler(abs(ctrl_r), 0.0, 1.0, 0.0, 8191.0);

  if(abs(angle_cmd) + abs(vel_cmd)<.15){
    sig_l = 0;
    sig_r = 0;
  }

  if(ctrl_l< 0){
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  } else{
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }

  if (ctrl_r< 0){
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }

  ledcWrite(motor1Channel, sig_l);
  ledcWrite(motor2Channel, sig_r);
  Serial.println(sig_l);
  Serial.println(sig_r);
  
}

void setup() {                  
  Serial.begin(115200);  
 
  WiFi.softAP(ssid);
  WiFi.softAPConfig(IPAddress(192, 168, 1, 6),  IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0)); 

  udp.begin(UDPPort); // strange bug needs to come after WiFi.begin but before connect

  ledcSetup(motor1Channel, freq, resolution);
  ledcSetup(motor2Channel, freq, resolution);
  ledcAttachPin(enablePin1, motor1Channel);
  ledcAttachPin(enablePin2, motor2Channel);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  
}

void loop(){
  int packetSize = udp.parsePacket();

  if (packetSize) {

    int len = udp.read(packetBuffer, 255);

    if (len > 0) {

      packetBuffer[len] = 0;

    }

    int val = atoi(packetBuffer+2);
    
    if (packetBuffer[0] == 'y'){
      yVal = val / MAX_VAL;
    }
    
    if (packetBuffer[0] == 'x'){
      xVal = val / MAX_VAL;
    }
    update_servos(xVal, yVal);
  }
}
