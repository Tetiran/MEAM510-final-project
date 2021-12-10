const int servo1PIN = 21;  //Pin used for LED
const int servo2PIN = 22;  //Pin used for LED
const int servo3PIN = 19;  //Pin used for LED
uint16_t DC; //Variable for keeping track of the duty cycle
int i;
// setting PWM properties
const int freq = 50; //frequency (10Hz)
const int servo1Channel = 0;
const int servo2Channel = 1;
const int servo3Channel = 2;

const int resolution = 13;

const int MINANGLE = 0;
const int MAXANGLE = 180 + MINANGLE; 

bool rightSignal, leftSignal; 
int beaconAngle;



void setup() {
  Serial.begin(115200);
  // configure LED PWM functionalitites
  ledcSetup(servo1Channel, freq, resolution);
  ledcSetup(servo2Channel, freq, resolution);
  ledcSetup(servo3Channel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(servo1PIN, servo1Channel);
  ledcAttachPin(servo2PIN, servo2Channel);
  ledcAttachPin(servo3PIN, servo3Channel);

}


//map angle to duty_cycle
float angleToDC(int angle){
   int mid = map(angle, 0, 180, 30, 130);
   int DC = map(mid, 0, 1000, 0, 8191);
   return DC;
}

void openGripper(){
  ledcWrite(servo1Channel, angleToDC(0));
  ledcWrite(servo2Channel, angleToDC(0));
}

void closeGripper(){
  ledcWrite(servo1Channel, angleToDC(170));
  ledcWrite(servo2Channel, angleToDC(170));
}

void findBeacon() {
  //read signals from two phototransistors

  //do logic on which are high 

  //if both detect a signal, move the car in the current angle
  if(rightSignal && leftSignal){
    Serial.println("Signal Detected");
  }

  //if just right servo sees signal
  else if (rightSignal) {
    //if we are at the edge, we need to turn the car 120 degrees
    if (beaconAngle == 0){
      //TURN CAR
      beaconAngle = 90;
    }
    else {
      //update angle and move servo right
      beaconAngle = beaconAngle - 5;
    }
    ledcWrite(servo3Channel, angleToDC(beaconAngle));
  }

  //if just left servo sees signal
  else if (leftSignal) {
    //if we are at the edge, we need to turn the car 120 degrees
    if (beaconAngle == 180){
      //TURN CAR
      beaconAngle = 90;
    }
    else {
      //update angle and move servo right
      beaconAngle = beaconAngle + 5;
    }
    ledcWrite(servo3Channel, angleToDC(beaconAngle));
  }
  
}


void loop() {
//   Code for sweeping beacon finder servo 
//  int i; 
//  for (i = MINANGLE; i <= MAXANGLE; i = i + 5 ) {
//    Serial.print("Angle: ");
//    Serial.println(i);
//    DC = angleToDC(i); 
//    ledcWrite(servo3Channel, DC);
//    Serial.print("Duty Cycle: ");
//    Serial.println(DC);
//    delay(500);
//    }
  openGripper();
  delay(1000);
  closeGripper();
  delay(1000);
}
