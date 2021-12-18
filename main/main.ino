
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include "vive510.h"

#define MAX_VAL 32769.0


char packetBuffer[255]; //buffer to hold incoming packet
//const char* ssid     = "TP-Link_05AF";
const char* ssid     = "TP-Link_E0C8";
//const char* password = "47543454";
const char* password = "52665134";

WiFiUDP UdpCommand;
WiFiUDP UdpRobots;
WiFiUDP UdpCans;
unsigned int UDPPortCommands = 5005;
unsigned int UDPPortCans = 1510;
unsigned int UDPPortRobots = 2510;

IPAddress ipTarget(192, 168, 1, 255); // 255 is a broadcast address to everyone at 192.168.1.xxx
IPAddress ipLocal(192, 168, 1, 6);  // replace with your IP address

//intiialize varaibles
double xVal, yVal;

const int motor2Pina = 22;
const int motor2Pinb = 21;
const int motor1Pina = 23;
const int motor1Pinb = 19;

const int servo1PIN = 18;
const int servo2PIN = 5;
//const int servo3PIN = 10;

const int motor1Channela = 0;
const int motor1Channelb = 1;
const int motor2Channela = 2;
const int motor2Channelb = 3;
const int servo1Channel = 4;
const int servo2Channel = 5;
const int servo3Channel = 6;

const int resolution = 13;
const int freq = 50;

bool GripState = false;
bool WallFollow = false;
bool MoveTo = false;
bool BeaconTrack = false;

long heartbeat = 0;
int RobotNum = 0;

int Gip1Open = 0;
int Grip1Closed = 0;
int Grip2Open = 0;
int Grip2Closed = 0;

const int Trig1 = 25;
const int Echo1 = 34;

const int Trig2 = 10;
const int Echo2 = 38;

const int Trig3 = 9;
const int Echo3 = 35;

int FollowRotation = 0; // -1 is follow right 1 is follow left

int FollowState = 0; // 0 is uninitialized, 1 is tracking

float FollowDistance = 1;
float FollowPGain = 0;
float FollowIGain = 0;
int FollowControlFreq =1;
long FollowLastUpdate = 0;
float FollowVelocity =0;
float FollowIError=0;
float TurnDistance = 1;
float TurnPGain = 0;
int RotationDelay =0;

int MoveToX = 0;
int MoveToY = 0;
int MoveToControlFreq = 0;
long MoveToLastUpdate = 0;
float MoveToPGain = 0;
int MoveToFinalDistance = 0;
float MoveToForwardVelocity = 0;

long BeaconLastUpdate = 0;
int BeaconTrackFreq =10;

int RobotX = 0;
int RobotY = 0;

float BeaconGain = .1;

double RobotAngle = 0;

#define VIVEPIN1 26
#define VIVEPIN2 32

#define IR_PIN_1 33
#define IR_PIN_2 27

#define TELEM_BUF_LEN 13

Vive510 vive1(VIVEPIN1);
Vive510 vive2(VIVEPIN2);

long LastVive = 0;

long LastLocationSend = 0;


float scaler(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

float angleToDC(int angle){
   int DC = map(angle, 0, 180, 410, 1640);
   return DC;
}

void update_grip(){
  if (GripState){
     ledcWrite(servo1Channel, angleToDC(Grip1Closed));
     ledcWrite(servo2Channel, angleToDC(Grip2Closed));
  }
  else{
    ledcWrite(servo1Channel, angleToDC(Gip1Open));
    ledcWrite(servo2Channel, angleToDC(Grip2Open));
  }
}

void update_servos(float angle_cmd, float vel_cmd, bool deadzone = false) {

  float cmd_scale = abs(angle_cmd) + abs(vel_cmd);
  if (cmd_scale < 1.0) {
    cmd_scale = 1.0;
  }

  float ctrl_l = (vel_cmd - angle_cmd) / cmd_scale;
  float ctrl_r = (vel_cmd + angle_cmd) / cmd_scale;

  double sig_l = scaler(abs(ctrl_l), 0.0, 1.0, 0.0, 8191.0);
  double sig_r = scaler(abs(ctrl_r), 0.0, 1.0, 0.0, 8191.0);

  if (deadzone){
    if(abs(angle_cmd) + abs(vel_cmd)<.15){
    sig_l = 0;
    sig_r = 0;
    }
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
}

void setup() {                  
  Serial.begin(115200);  
 
  WiFi.config(IPAddress(192, 168, 1, 6),  IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password); 

  UdpCommand.begin(UDPPortCommands);
  UdpRobots.begin(UDPPortRobots);
  UdpCans.begin(UDPPortCans);

  ledcSetup(motor1Channela, freq, resolution);
  ledcSetup(motor1Channelb, freq, resolution);
  ledcSetup(motor2Channela, freq, resolution);
  ledcSetup(motor2Channelb, freq, resolution);

  ledcSetup(servo1Channel, freq, resolution);
  ledcSetup(servo2Channel, freq, resolution);
  ledcSetup(servo3Channel, freq, resolution);

  ledcAttachPin(motor1Pina, motor1Channela);
  ledcAttachPin(motor1Pinb, motor1Channelb);
  ledcAttachPin(motor2Pina, motor2Channela);
  ledcAttachPin(motor2Pinb, motor2Channelb);

  ledcAttachPin(servo1PIN, servo1Channel);
  ledcAttachPin(servo2PIN, servo2Channel);
  //ledcAttachPin(servo3PIN, servo3Channel);

  pinMode(Trig1, OUTPUT);
  pinMode(Trig2, OUTPUT);
  pinMode(Trig3, OUTPUT);

  pinMode(Echo1, INPUT);
  pinMode(Echo2, INPUT);
  pinMode(Echo3, INPUT);

  pinMode(IR_PIN_1, INPUT);
  pinMode(IR_PIN_2, INPUT);

  vive1.begin();
  vive2.begin();

  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.println("no wifi");
  }
  
}
void recieve_commands(){
  int packetSize = UdpCommand.parsePacket();

  if (packetSize) {

    int len = UdpCommand.read(packetBuffer, 255);

    if (len > 0) {
      packetBuffer[len] = 0;
    }

    if (!WallFollow && !MoveTo){
      int val = atoi(packetBuffer+2);
      if (packetBuffer[0] == 'y'){
        float t =  millis() / 1000.0;
        Serial.print("Recieved Y time");
        Serial.println(t);
        yVal = val / MAX_VAL;
        update_servos(xVal, yVal, true);
      }
      
      if (packetBuffer[0] == 'x'){
        xVal = val / MAX_VAL;
        update_servos(xVal, yVal, true);
      }

      if (packetBuffer[0] == 'g'){
        if (GripState != val){
          GripState = val;
          update_grip();
        }
      }
    }
    if (packetBuffer[0] == 'w' && !MoveTo){
      int val = atoi(packetBuffer+2);
      if (WallFollow != val){
        WallFollow = val;
        if (WallFollow){
            update_servos(0,0);
        } else{
          FollowState=0;
        }
      }
    }
    

    if (packetBuffer[0] == 'm' && !WallFollow){
      char *token;
      int val = atoi(strtok(packetBuffer+2, "_"));
      MoveToX = atoi(strtok(NULL, "_"));
      MoveToY = atoi(strtok(NULL, "_"));
      if (MoveTo != val){
        MoveTo = val;
        if (MoveTo){
          update_servos(0,0);
        } 
      }
    }

    if (packetBuffer[0] == 't' && !WallFollow){
      char *token;
      int val = atoi(strtok(packetBuffer+2, "_"));
      if (BeaconTrack != val){
        BeaconTrack = val;
        if (BeaconTrack){
          update_servos(0,0);
          attach_interrupts();
        } else{
          deattach_interrupts();
        } 
      }
    }

    // parse config
    if (packetBuffer[0] == 'c'){
      char *token;
      Serial.println(packetBuffer);
      // discard 'c' character
      Gip1Open = atoi(strtok(packetBuffer+2, "_"));
      Grip1Closed = atoi(strtok(NULL, "_"));
      Grip2Open = atoi(strtok(NULL, "_"));
      Grip2Closed = atoi(strtok(NULL, "_"));
      RobotNum = atoi(strtok(NULL, "_"));
      FollowDistance = atof(strtok(NULL, "_"));
      FollowPGain = atof(strtok(NULL, "_"));
      FollowIGain = atof(strtok(NULL, "_"));
      FollowControlFreq = atoi(strtok(NULL, "_"));
      FollowVelocity = atof(strtok(NULL, "_"));
      TurnDistance = atof(strtok(NULL, "_"));
      TurnPGain = atof(strtok(NULL, "_"));
      RotationDelay = atoi(strtok(NULL, "_"));
      MoveToControlFreq = atoi(strtok(NULL, "_"));
      MoveToPGain = atof(strtok(NULL, "_"));
      MoveToFinalDistance = atoi(strtok(NULL, "_"));
      MoveToForwardVelocity = atof(strtok(NULL, "_"));
      BeaconGain = atof(strtok(NULL, "_"));
    }
  }
}

float clamp(float val, float min, float max){
  if (val<min){
    return min;
  } else if (val> max){
    return max;
  }
  return val;
}

void wall_follow(){
  if (FollowState==0){
    // compute inital distances and choose follow direction

    float distance1 = get_distance(Trig1, Echo1);
    float distance3 = get_distance(Trig3, Echo3);

    // setup direction, state and last update
    if (distance1 > 8 && distance1 < 30){
      FollowRotation = 1;
    } else{
      FollowRotation = -1;
    }
    FollowState = 1;
    FollowIError = 0;
    FollowLastUpdate = millis();

  } else if (FollowState == 1){
    // update at control frequency
    if (millis()> FollowLastUpdate + 1000 / FollowControlFreq){
      
      float WallDistance = 0;

      if (FollowRotation == 1){
        WallDistance = get_distance(Trig1, Echo1);
      } else{
        WallDistance = get_distance(Trig3, Echo3);
      }

      float ForwardDistance = get_distance(Trig2, Echo2);

      Serial.print("Forward distance");
      Serial.println(ForwardDistance);

      float error = FollowDistance - WallDistance;
      float P_Ctrl = FollowPGain * error;
      FollowIError = clamp(FollowIError+error * ((millis() - FollowLastUpdate) /1000.0), -10, 10);
      float I_Ctrl = FollowIGain * FollowIError;

      float error_F = ForwardDistance - TurnDistance;
      float P_Ctrl_F = 0;
      if (error_F < 0 && ForwardDistance > 0) {
         P_Ctrl_F = TurnPGain * error_F; 
         FollowState = 2;
      }

      float update_wall = clamp((P_Ctrl + I_Ctrl) * FollowRotation, -.2, .2);
      //float update_turn = clamp(P_Ctrl_F * -1* FollowRotation, -0.4, 0.4);

      float update = update_wall; //+ update_turn; 

      update_servos(update, FollowVelocity);
      FollowLastUpdate = millis();
    }
  } else if (FollowState == 2){
    update_servos(.5 * FollowRotation, 0);
    delay(RotationDelay);
    FollowLastUpdate = millis();
    FollowState = 1;
  }
}

float get_distance(int trig, int echo){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 10000);
  float distance = duration * 0.034 / 2;
  return distance;
}


void update_vive(){
  int V1X = 0;
  int V1Y = 0; 
  int V2X = 0;
  int V2Y = 0;
  if (vive1.status() == VIVE_LOCKEDON) {
     V1X = vive1.xCoord();
     V1Y = vive1.yCoord();
  }
  else 
     vive1.sync(15); // try to resync (nonblocking);

  if (vive2.status() == VIVE_LOCKEDON) {
     V2X = vive2.xCoord();
     V2Y = vive2.yCoord();
  }
  else 
     vive2.sync(15); // try to resync (nonblocking);

  if (V1X !=0 && V2X !=0){
    RobotX = (V1X+V2X)/2;
    RobotY = (V1Y+V2Y)/2;
    RobotAngle = atan2(V2Y - V1Y, V2X - V1X);
  }
}

void move_to(){

  if (millis()> MoveToLastUpdate + 1000 / MoveToControlFreq){
  int distance = sqrt(pow((RobotY - MoveToY),2) + pow(RobotX - MoveToX,2));

  if (distance< MoveToFinalDistance){

    update_servos(0,0);

  } else{

    double TargetAngle = atan2(MoveToY - RobotY, MoveToX - RobotX);
    if(TargetAngle< PI/2){
      TargetAngle+= PI/2;
    } else{
      TargetAngle = -2 * PI + (TargetAngle + PI/2); 
    }

    double AngleError = TargetAngle - RobotAngle;
    if (AngleError < -PI) {
      AngleError += 2*PI;
    } else if (AngleError > PI){
      AngleError -= 2*PI;
    }
    

    double UpdateAngle = AngleError * MoveToPGain;
    update_servos(UpdateAngle, .5);

  }

  MoveToLastUpdate = millis();
  }
}

volatile long LastIR1 = 0;
volatile long NewestIR1 = 0;
volatile long LastIR2 = 0;
volatile long NewestIR2 = 0;
volatile float IRFreq1 = 0;
volatile float IRFreq2 = 0;


void IRAM_ATTR IR_ISR_1(){
  LastIR1 = NewestIR1;
  NewestIR1 = micros();
}

void IRAM_ATTR IR_ISR_2(){
  LastIR2 = NewestIR2;
  NewestIR2 = micros();

}

void attach_interrupts(){
  attachInterrupt(digitalPinToInterrupt(IR_PIN_1), IR_ISR_1, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_PIN_2), IR_ISR_2, RISING);
}

void deattach_interrupts(){
  detachInterrupt(digitalPinToInterrupt(IR_PIN_1));
  detachInterrupt(digitalPinToInterrupt(IR_PIN_2));
}

void beacon_track(){

  static long BeaconLastUpdate = 0;

  if (millis()> BeaconLastUpdate + 1000/ BeaconTrackFreq){
    long currtime = micros();
    float freq_1 = 0;
    float freq_2 = 0;

    if (NewestIR1 && NewestIR2){
        freq_1 = 1.0/((NewestIR1 - LastIR1)/1000000.0);
        freq_2 = 1.0/((NewestIR2 - LastIR2)/1000000.0);
    }

    if (NewestIR1< micros() - 100000){
      freq_1 = -1;
    }
    if (NewestIR2< micros() - 100000){
      freq_2 = -1;
    }
    bool LeftDetected = false;
    bool RightDetected = false;

    if ((freq_1> 20 && freq_1 < 23) || (freq_1> 690 && freq_1 < 710)){
      LeftDetected = true;
    }

    if ((freq_2 > 20 && freq_2 < 23) || (freq_2> 690 && freq_2 < 710)){
      RightDetected = true;
    }


    if (LeftDetected && RightDetected){
      update_servos(0, .5);

    } else if (LeftDetected){
      update_servos(BeaconGain, 0);

    } else if (RightDetected){
      update_servos(-BeaconGain, 0);

    } else{
      update_servos(.1, 0);
    }
    
  }
}

void send_telemetry(){
  char s[TELEM_BUF_LEN];
  // store into a string with format #:####,####, which is robotid, x, y
  sprintf(s,"%1d:%4d,%4d",RobotNum, RobotX, RobotY); 
  UdpRobots.beginPacket(ipTarget, UDPPortRobots);
  UdpRobots.write((uint8_t *)s, TELEM_BUF_LEN);
  UdpRobots.endPacket();
}

void loop(){
  recieve_commands();
  float t =  millis() / 1000.0;
  Serial.print("curr time");
  Serial.println(t);

  if (WallFollow){
    wall_follow();
  }

  if (MoveTo){
    move_to();
  }

  if (BeaconTrack){
    beacon_track();
  }

  if (millis()> LastVive + 100){
    update_vive();
    LastVive = millis();
    t =  millis() / 1000.0;
    Serial.print("post vive time");
    Serial.println(t);
  }
  
  

  long time = millis();
  if (time> LastLocationSend +1000){
    LastLocationSend = millis();
    send_telemetry();
  }
}
