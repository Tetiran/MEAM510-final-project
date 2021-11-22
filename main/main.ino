
#include <WiFi.h>
#include <WiFiUdp.h>
#include <tgmath.h>

char packetBuffer[255]; //buffer to hold incoming packet
const char* ssid     = "da_baby";
const char* password = "47543454";

WiFiUDP udp;
unsigned int UDPPort = 5005; // port for cans is 1510, port for robots is 2510
IPAddress ipTarget(192, 168, 1, 255); // 255 is a broadcast address to everyone at 192.168.1.xxx
IPAddress ipLocal(192, 168, 1, 6);  // replace with your IP address


//intiialize varaibles
double r, duty_cycle, theta, xVal, yVal, DCL, DCR, theta_deg;
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

void update_servos() {
  
  //find radius of point
  r = sqrt(xVal * xVal + yVal * yVal);
  
  duty_cycle = map(r, 0, 100, 0, 8191); //map radius to duty cycle for primary motor (can adjust resolution)

  if (r < 15){
    duty_cycle=0;
  }

  //find angle to determine driving mode
  if (xVal == 0){
    xVal = 0.0001; 
  }

  theta = atan2(yVal, xVal);
  theta_deg = theta * 180.0 / M_PI;
  if(theta_deg < 0)
  {
    theta_deg = theta_deg + 360;
  }

  //Figure out Section of Graph

  //PIVOT RIGHT
  if (theta_deg <  45 or theta_deg > 315) {
    leftD = 0;
    rightD = 1;
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    DCL = duty_cycle;
    DCR = duty_cycle;
  }

  //STRAIGHT AHEAD
  else if (theta_deg > 45 and theta_deg < 135) {
    leftD = 0;
    rightD = 0;
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    DCL = duty_cycle;
    DCR = duty_cycle;
  }

  //PIVOT LEFT
  else if (theta_deg > 135 and theta_deg < 225) {
    leftD = 1;
    rightD = 0;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    DCL = duty_cycle;
    DCR = duty_cycle;
  }

  //STRAIGHT BACKWARDS
  else if (theta_deg > 225 and theta_deg < 315) {
    leftD = 1;
    rightD = 1;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    DCL = duty_cycle;
    DCR = duty_cycle;
  }
  Serial.println(xVal);
  Serial.println(yVal);
  Serial.println("cycles");
  Serial.println(DCL);
  Serial.println(DCR);
  Serial.println(theta_deg);
  //Based on leftD and rightD, set motor directions
  //Send DCL and DCR to motors
  ledcWrite(motor1Channel, DCL);
  ledcWrite(motor2Channel, DCR);
  
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
      yVal = val / 327.69;
    }
    
    if (packetBuffer[0] == 'x'){
      xVal = val / 327.69;
    }
    
  }
  update_servos();
}
