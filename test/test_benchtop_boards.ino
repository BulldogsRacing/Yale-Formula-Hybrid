
#include <Servo.h>

int rpm_in =      0; // analog
int rad_temp =    1; //analog
int batt_low =   22; // digital
int fuel_in =     3; // Analog
int throttle_in = 4; //Analog
int bms_fault =  23; //Digital
int servo_out =   2; //Digital (PWM)
int kelly_out =   3; //Digital (PWM)
int regen_out =   4; //Digital (PWM)
int clutch_in =   24; //Digital
int assist_in =   25; //Digital (PULLED LOW)
int brake_in =    26; //Digital
int engineEnable = 27; //Digital
Servo servo, kelly, regen;


void setup() {
  Serial.begin(9600);
  pinMode(batt_low,INPUT);
  pinMode(bms_fault,INPUT);
  pinMode(clutch_in,INPUT);
  pinMode(assist_in, INPUT);
  pinMode(brake_in, INPUT);
  pinMode(engineEnable, INPUT);
  servo.attach(servo_out);
  kelly.attach(kelly_out);
  regen.attach(regen_out);
  servo.write(90);
  kelly.write(0);
  regen.write(180);
  
}

void loop() {
  
  Serial.print("RPM ");
  Serial.println(analogRead(rpm_in));
  Serial.print("Temp ");
  Serial.println(analogRead(rad_temp));
  Serial.print("BattLow ");
  Serial.println(digitalRead(batt_low));
  Serial.print("FuelIn ");
  Serial.println(analogRead(fuel_in));
  Serial.print("ThrottleIn ");
  Serial.println(analogRead(throttle_in));
  Serial.print("BMS_Fault ");
  Serial.println(digitalRead(bms_fault));
  Serial.print("Clutch ");
  Serial.println(digitalRead(clutch_in));
  Serial.print("AssistIn ");
  Serial.println(digitalRead(assist_in));
  Serial.print("BrakeIn ");
  Serial.println(digitalRead(brake_in));
  Serial.print("EngineEnable ");
  Serial.println(digitalRead(engineEnable));
  
  delay(10);
}
