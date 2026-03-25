#include <Servo.h>                


Servo servoLeft;                    
Servo servoRight;


signed long time;
signed long beginTime;
signed long firstRun;
signed long secondRun;
signed long runTime;
const int redLedPinL = A0;
const int redLedPinC = A1;
const int redLedPinR = A2;


const int irLedPinC=6, irReceiverPinC=7;
const int irLedPinR=2, irReceiverPinR=3;
const int irLedPinL=10, irReceiverPinL=11;


const int offsetC = 0;
const int offsetR = 1000;
const int offsetL = 1000;


float smoothError = 0;
float smoothPrevError = 0;


const int redLedPin = A1;      
int error = 0;
int preverror = 0;


int turns;


int Rspin;
int Lspin;


//Tune manually, I dont think Ziegler Nicholas applies
//Start with exclusively proportion factor and tune that for error correction that makes sense
float proportion_factor = 50;
float integral_factor = 0;
//Possibly disable differential factor as it is not essential and could prove difficult to tune
float differential_factor = 5;


int limitcorrection = 200;
bool endReached = false;
float DistTravelled = 0;
int direction = 0;


bool firstTurn = 1;


int periodCount = 0;
int cycles = 0;
float distOld = 0;


//postiioning values for maze finishing
int x = 0;
int y = 0;
int xmax = 0;
int ymax = 0;
int xmin = 0;
int ymin = 0;


bool flag = false;


//Averaging array
int mem1 = 0;
int mem2 = 0;
int mem3 = 0;
int mem4 = 0;


float period = 1 / 40;


float PIDcalc(float error, int period){
if (cycles > 4){
  float smoothError = error + preverror + mem1 + mem2;
  float smoothPrevError = preverror + mem1 + mem2 + mem3;
}
else {
  int smoothError = error;
  int smoothPrevError = preverror;
}
float integral =+ error;
float integration_term = integral * integral_factor * period;


float differential = (smoothError - smoothPrevError)/5;
float differential_term = differential_factor * differential;


float error_term = proportion_factor * error;
 float adjustment = error_term + differential_term + integration_term;


preverror = error;
mem1 = preverror;
mem2 = mem1;
mem3 = mem2;
mem4 = mem3;


cycles++;


if (adjustment > limitcorrection){
adjustment = limitcorrection;
}


if (adjustment < -limitcorrection){
adjustment = -limitcorrection;
}
return adjustment;
}




void setup() {                           // Built in initialization block
   Serial.begin(9600);
   servoRight.attach(12);
   servoLeft.attach(13);
   pinMode(irReceiverPinL, INPUT);
   pinMode(irLedPinL, OUTPUT);
   pinMode(irReceiverPinR, INPUT);
   pinMode(irLedPinR, OUTPUT);
   pinMode(irReceiverPinC, INPUT);
   pinMode(irLedPinC, OUTPUT);
   delay(4000);
   beginTime = millis();
   //Moving Forward:
   //servoRight.writeMicroseconds(1300);   // Attach left signal to pin 13
   //servoLeft.writeMicroseconds(1300);    // 1.3ms = full speed clockwise
}








void loop() {                             // Main loop auto-repeats
time = millis();      
  if (endReached){
    runTime = time - firstRun;
    if ((runTime-3000) >= firstRun){
      servoRight.writeMicroseconds(1491);  
      servoLeft.writeMicroseconds(1500);
      delay(5000);
    }
  }
 int irDistC = irDistance(irLedPinC,irReceiverPinC, 0);
 int irDistL = irDistance(irLedPinL, irReceiverPinL, offsetL);
 int irDistR = irDistance(irLedPinR, irReceiverPinR, offsetR);


//  Serial.print("L:");
//  Serial.println(irDistL);                     // Display 1/0 no detect/detect
//  Serial.print("R:");
//  Serial.println(irDistR);
//  Serial.print("C:");
//  Serial.println(irDistC);  


 error = irDistL-irDistR - 1;  
 float adjust = PIDcalc(error,period);
 Serial.println(firstRun);
 Serial.println(millis());


 
  if (irDistL < 5 && irDistR < 5 && irDistC < 5) {        // Stopping Mechanism changed 3 to 5
    firstRun= time - beginTime;
    servoRight.writeMicroseconds(1650); // reverse
    servoLeft.writeMicroseconds(1300);
    delay(1400);
    servoRight.writeMicroseconds(1700); // spin
    servoLeft.writeMicroseconds(1700);
    delay(4770);


    endReached = true;
    secondRun = millis();




    servoRight.writeMicroseconds(1291);
    servoLeft.writeMicroseconds(1700);
    delay(200);
    error = 0;
  }
 
   if (irDistC < 5) {  
    if (error > 0) {


      servoRight.writeMicroseconds(1291);
      servoLeft.writeMicroseconds(1475);
      delay(725);


      preverror = 0;
      mem1 = 0;
      mem2 = 0;
      mem3 = 0;
      mem4 = 0;


    }
   
    else if (error < 0) {
      servoRight.writeMicroseconds(1525);
      servoLeft.writeMicroseconds(1700);
       delay(725);
    }




    preverror = 0;
    mem1 = 0;
    mem2 = 0;
    mem3 = 0;
    mem4 = 0;




  } else {
   servoRight.writeMicroseconds(1291- adjust);
   servoLeft.writeMicroseconds(1700 - adjust);
   Lspin=0;
   Rspin=0;
  }
}




int irDetect(int irLedPin,int irReceiverPin, long frequency) {
 tone(irLedPin, frequency);                 // Turn on the IR LED square wave
 delay(1);                                  // Wait 1 ms
 noTone(irLedPin);                          // Turn off the IR LED
 int ir = digitalRead(irReceiverPin);       // IR receiver -> ir variable// Down time before recheck
 return ir;                                 // Return 0 detect, 1 no detect
}




int irDistance(int irLedPin, int irReceivePin, int offset) {
  int distance = 0;
  for(long f = (37000+offset); f <= (41000+offset); f += 1000)
  {
     distance += irDetect(irLedPin, irReceivePin, f);
  }
  return distance;
}




