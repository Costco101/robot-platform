#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWMr 24 //speed - right
#define IN2r 22 //direction 2 - right
#define IN1r 26 //direction 1 - right
#define PWMl 5 //speed - left
#define IN2l 7 //direction 2 - left
#define IN1l 6 //direction 1 - left




#define ENCAL 18 // YELLOW
#define ENCBL 19 // WHITE
//#define PWMr 24 //speed - right
//#define IN2r 22 //direction 2 - right
//#define IN1r 26 //direction 1 - right




//int m = 47; //error




volatile int posi1 = 0;
volatile int posi2 = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT1 = 0;
long prevT2 = 0;
float eprev1 = 0;
float eprev2 = 0;
float eintegral1 = 0;
float eintegral2 = 0;
int clickPerRot = 630; // how many marks in one rotation
double circ = 262.1;
double target1;
double target2;
int pos1 = 0;
int pos2 = 0;








void setup() {
Serial.begin(9600);
pinMode(ENCA,INPUT);
pinMode(ENCB,INPUT);
pinMode(ENCAL,INPUT);
pinMode(ENCBL,INPUT);
attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder1L,RISING);
pinMode(PWMl,OUTPUT);
pinMode(IN1l,OUTPUT);
pinMode(IN2l,OUTPUT);
attachInterrupt(digitalPinToInterrupt(ENCB),readEncoder1R,RISING);
pinMode(PWMr,OUTPUT);
pinMode(IN1r,OUTPUT);
pinMode(IN2r,OUTPUT);
attachInterrupt(digitalPinToInterrupt(ENCBL),readEncoder2R,RISING);
attachInterrupt(digitalPinToInterrupt(ENCAL),readEncoder2L,RISING);








Serial.println("target pos");
}




void loop() {
forward(500);
delay(500);
forward(500);
// set target position








//int target = 250*sin(prevT/1e6);




// PID constants
float kp = .09;
float kd = 0.025;
float ki = 0.002;




// time difference
long currT1 = micros();
long currT2 = micros();


float deltaT1 = ((float) (currT1 - prevT1))/( 1.0e6 );
float deltaT2 = ((float) (currT2 - prevT2))/( 1.0e6 );


prevT1 = currT1;
prevT2 = currT2;




// Read the position in an atomic block to avoid a potential
// misread if the interrupt coincides with this code running
// see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/


ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
pos1 = posi1;
pos2 = posi2;
}
// error
int e1 = pos1 - target1;
int e2 = pos2 - target2;




// derivative
float dedt1 = (e1-eprev1)/(deltaT1);
float dedt2 = (e2-eprev2)/(deltaT2);






// integral
eintegral1 = eintegral1 + e1*deltaT1;
eintegral2 = eintegral2 + e2*deltaT2;


// control signal
float u1= kp*e1 + kd*dedt1 + ki*eintegral1;
float u2 = kp*e2 + kd*dedt2 + ki*eintegral2;




// motor power
float pwr1 = fabs(u1);
if( pwr1 > 255 ){
pwr1 = 255;
}
float pwr2 = fabs(u2);
if( pwr2 > 255 ){
pwr2 = 255;
}




// motor direction
int dir1 = 1;
if(u1<0){
dir1 = -1;
}
int dir2 = 1;
if(u2<0){
dir2 = -1;
}




// signal the motor
setMotor1(dir1,pwr1,PWMl,IN1l,IN2l); //left
setMotor2(dir2,pwr2,PWMr,IN1r,IN2r); //right




// store previous error
eprev1 = e1;
eprev2 = e2;




Serial.print("target1:" );
Serial.println(target1);
Serial.print(" ");
Serial.print("pos1: ");
Serial.println(pos1);
Serial.print("pos2: ");
Serial.println(pos2);
if (target1 != pos1){
target1++;
}
Serial.print("target2:" );
Serial.println(target2);
Serial.print(" ");
Serial.print(pos2);
Serial.println();
if (target2 != pos2){
target2++;
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
void readEncoder1L(){
int b1 = digitalRead(ENCB);
if(b1 > 0){
posi1++;
}
else{
posi1--;
}
}
void readEncoder2L(){
int b2 = digitalRead(ENCBL);
if(b2 > 0){
posi2++;
}
else{
posi2--;
}
}
static double forward(int distance){
Serial.print("circ: ");
Serial.println(circ);
Serial.print("clickPerRot: ");
Serial.println(clickPerRot);
//target = distance / circ * clickPerRot;
target1 = 5000;
target2 = 5000;
return(target1);
return(target2);
Serial.print("target1: ");
Serial.println(target1);
Serial.print("pos1: ");
Serial.println(pos1);
Serial.print("target2: ");
Serial.println(target2);
Serial.print("pos2 : ");
Serial.println(pos2);




}
void readEncoder1R(){
int a1 = digitalRead(ENCA);
if(a1>0){
posi1--;
}
else{
posi1++;
}
}
void readEncoder2R(){
int a2 = digitalRead(ENCAL);
if(a2>0){
posi2--;
}
else{
posi2++;
}
}




