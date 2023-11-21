void setup() {
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(2, 200);
  analogWrite(5, 200);
  digitalWrite(3, LOW);
  digitalWrite(6,LOW);
  digitalWrite(4,HIGH);
  digitalWrite(7,HIGH);
  delay(2000);
  digitalWrite(3, HIGH);
  digitalWrite(6,HIGH);
  digitalWrite(4,LOW);
  digitalWrite(7,LOW);
  delay(2000);


}
