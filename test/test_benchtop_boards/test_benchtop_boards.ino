
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
int assist_in =   25; //Digital (PULLED HIGH)
int brake_in =    26; //Digital
int engineEnable= 27; //Digital
int hvEnable =    28; //Digital
int speed_in =    29; //Digital
int regen_mode =  30; //Digital
int critical_out= 33; //Digital
int engine_kill = 36; //Digital
int hvtoggle_out= 37; //Digital
int car_on_led=51;//Digital OUT
int auto_endur_led=52;//Digital OUT
int auto_endur_in=39;//Digital IN

int endurance_dial=6;//Analog in

Servo servo;


void setup() {
  Serial.begin(9600);
  pinMode(batt_low,INPUT);
  pinMode(bms_fault,INPUT);
  pinMode(clutch_in,INPUT);
  pinMode(assist_in, INPUT);
  digitalWrite(assist_in,HIGH); // Pull ASSIST_IN high
  pinMode(brake_in, INPUT);
  pinMode(engineEnable, INPUT);
  pinMode(hvEnable, INPUT);
  pinMode(speed_in, INPUT);
  pinMode(regen_mode, INPUT);
  pinMode(auto_endur_in,INPUT);

  pinMode(critical_out, OUTPUT);
  pinMode(engine_kill, OUTPUT);
  pinMode(hvtoggle_out, OUTPUT);
  pinMode(car_on_led, OUTPUT);
  pinMode(auto_endur_led,OUTPUT);
  servo.attach(servo_out);
  servo.write(180);

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
  Serial.print("EngineEnable ");
  Serial.println(digitalRead(engineEnable));
  Serial.print("HVEnable ");
  Serial.println(digitalRead(hvEnable));
  Serial.print("SpeedIn ");
  Serial.println(digitalRead(speed_in));
  Serial.print("RegenMode ");
  Serial.println(digitalRead(regen_mode));
  Serial.print("AutoEndurMode ");
  Serial.println(digitalRead(auto_endur_in));
  Serial.print("EnduranceDial ");
  Serial.println(analogRead(endurance_dial));

  digitalWrite(critical_out,HIGH);
  digitalWrite(engine_kill,HIGH);
  digitalWrite(hvtoggle_out,LOW);
  digitalWrite(car_on_led,LOW);
  digitalWrite(auto_endur_led,HIGH);



  analogWrite(kelly_out,20);
  analogWrite(regen_out,255);
  delay(10);
}
