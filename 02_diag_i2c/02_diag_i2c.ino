#include <Wire.h>
void scan(const char* t){
  Serial.print(t); Serial.print(": ");
  int n=0;
  for(uint8_t a=1;a<127;a++){ Wire.beginTransmission(a); if(Wire.endTransmission()==0){ Serial.print("0x"); Serial.print(a,HEX); Serial.print(' '); n++; } }
  if(!n) Serial.print("rien");
  Serial.println();
}
void setup(){
  Serial.begin(9600);
  Wire.begin(); Wire.setWireTimeout(3000,true);
  Serial.println(F("--- diag i2c ---"));
  Serial.print(F("A4 repos=")); Serial.print(digitalRead(A4)); Serial.print(F(" A5 repos=")); Serial.println(digitalRead(A5));
  for(int i=2;i<=4;i++){ pinMode(i,OUTPUT); digitalWrite(i,LOW);} delay(10);
  scan("tous CE bas");
  for(int i=2;i<=4;i++){ pinMode(i,INPUT); delay(10); char b[24]; sprintf(b,"CE D%d libere",i); scan(b); pinMode(i,OUTPUT); digitalWrite(i,LOW); }
  for(int i=2;i<=4;i++) pinMode(i,INPUT); delay(10);
  scan("tous CE libres");
}
void loop(){ Serial.print(F("millis=")); Serial.println(millis()); delay(1000); }
