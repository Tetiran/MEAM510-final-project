
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
const int motor2Pina = 19;
const int motor2Pinb = 23;
const int motor1Pina = 21;
const int motor1Pinb = 22;

const int motor1Channela = 0;
const int motor1Channelb = 1;
const int motor2Channela = 2;
const int motor2Channelb = 3;

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
    ledcWrite(motor1Channelb, 0);
    ledcWrite(motor1Channela, sig_l);
  } else{
    ledcWrite(motor1Channela, 0);
    ledcWrite(motor1Channelb, sig_l);
  }

  if (ctrl_r< 0){
    ledcWrite(motor2Channelb, 0);
    ledcWrite(motor2Channela, sig_r);
  } else {
    ledcWrite(motor2Channela, 0);
    ledcWrite(motor2Channelb, sig_r);
  }
  
  Serial.println(sig_l);
  Serial.println(sig_r);
  
}

void setup() {                  
  Serial.begin(115200);  
 
  WiFi.softAP(ssid);
  WiFi.softAPConfig(IPAddress(192, 168, 1, 6),  IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0)); 

  udp.begin(UDPPort); // strange bug needs to come after WiFi.begin but before connect

  ledcSetup(motor1Channela, freq, resolution);
  ledcSetup(motor1Channelb, freq, resolution);
  ledcSetup(motor2Channela, freq, resolution);
  ledcSetup(motor2Channelb, freq, resolution);
  ledcAttachPin(motor1Pina, motor1Channela);
  ledcAttachPin(motor1Pinb, motor1Channelb);
  ledcAttachPin(motor2Pina, motor2Channela);
  ledcAttachPin(motor2Pinb, motor2Channelb);
  
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
