#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#define ENCA 18 // YELLOW
#define ENCB 19 // WHITE
#define PWMr 5 //speed - right
#define IN2r 7//direction 2 - right
#define IN1r 6 //direction 1 - right
#define PWMl 28 //speed - left
#define IN2l 30 //direction 2 - left
#define IN1l 31 //direction 1 - left
int m = 47; //error

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int clickPerRot = 630; // how many marks in one rotation
double circ = 262.1;
double target;

 

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWMl,OUTPUT);
  pinMode(IN1l,OUTPUT);
  pinMode(IN2l,OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCB),readEncoderR,RISING);
  
  pinMode(PWMr,OUTPUT);
  pinMode(IN1r,OUTPUT);
  pinMode(IN2r,OUTPUT);

  Serial.println("target pos");
 
}

void loop() {
  forward(500);

  // set target position
 


  //int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = .5;
  float kd = 0.055;
  float ki = 0.008;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor1(dir,pwr,PWMl,IN1l,IN2l); //left
  setMotor2(dir,pwr,PWMr,IN1r,IN2r); //right

  // store previous error
  eprev = e;

  Serial.print("target:" );
  Serial.println(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  if (target != pos){
    target++;
  }

}

void setMotor1(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}
void setMotor2(int dir, int pwmVal, int pwm, int in3, int in4){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
  }
  else if(dir == -1){
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
  }
  else{
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
    
  }
}
static double forward(int distance){
  Serial.print("circ:  ");
  Serial.println(circ);
  Serial.print("clickPerRot: ");
  Serial.println(clickPerRot);
  //target = distance / circ * clickPerRot;
  target = 5000;
  return(target);
  Serial.print("target: ");
  Serial.println(target);

}
void readEncoderR(){
  int a = digitalRead(ENCA);
  if(a>0){
    posi--;
  }
  else{
    posi++;
  }
  }
  

