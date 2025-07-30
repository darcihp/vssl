/*
Encoder 1 = Pino 2 = 5V
Encoder 2 = Pino 3 = 5V
PWMA = Pino 9
INA2 = Pino A3
INA1 = Pino A2
PWMB = Pino 10
INB2 = Pino A0
INB1 = Pino A1
STNDBY = Pino 6
Chave 1 = Pino A4
Chave 2 = Pino A5
LED = Pino 4
*/

/*
Right frente
digitalWrite(inr_1, HIGH);
digitalWrite(inr_2, LOW);

Left frente
digitalWrite(inl_1, LOW);
digitalWrite(inl_2, HIGH);
*/

/*
48 cortes em cada roda
*/

/*
https://aleksandarhaber.com/clear-and-detailed-explanation-of-kinematics-equations-and-geometry-of-motion-of-differential-wheeled-robot-differential-drive-robot/
*/

#include <Arduino.h>
#include <MsTimer2.h>
#include <ArduPID.h>

ArduPID myController_l;
double input_l;
double output_l;
double setpoint_l = 220;
double p_l = 100;
double i_l = 50;
double d_l = 5;

ArduPID myController_r;
double input_r;
double output_r;
double setpoint_r = 220;
double p_r = 80;
double i_r = 50;
double d_r = 5;

const byte led_pin = 4;
const byte encoder_l = 3;
const byte encoder_r = 2;
const byte standby = 6;
const byte pwm_r = 9;
const byte inr_1 = A2;
const byte inr_2 = A3;
const byte pwm_l = 10;
const byte inl_1 = A1;
const byte inl_2 = A0;

boolean direction_l = false;
boolean direction_r = false;

int ticks_encoder_l;
int ticks_encoder_r;

float wheel_radius_l = 0.031;
float wheel_radius_r = 0.031;
float baseline = 0.060;

float interruption_time = 100.0;
//360degrees /48ppr
const float speed_constant = 7.5;

float last_x = 0;
float last_y = 0;
float last_theta = 0;


void move(boolean motor, int pwm)
{
  if(motor == 0)
  {
    if(direction_l == 0)
    {
      digitalWrite(inl_1, LOW);
      digitalWrite(inl_2, HIGH);
    }
    else
    {
      digitalWrite(inl_1, HIGH);
      digitalWrite(inl_2, LOW);
    }
    analogWrite(pwm_l, pwm);
  }

  if(motor == 1)
  {
    if( direction_r == 0)
    {
      digitalWrite(inr_1, HIGH);
      digitalWrite(inr_2, LOW);
    }
    else
    {
      digitalWrite(inr_1, LOW);
      digitalWrite(inr_2, HIGH);
    }
    analogWrite(pwm_r, pwm);
  }
}

void interruption()
{
  float last_degrees_l = ticks_encoder_l * speed_constant;
  //float last_degrees_l = (ticks_encoder_l * 360)/48;
  float last_degrees_r = ticks_encoder_r * speed_constant;
  //float last_degrees_r = (ticks_encoder_r * 360)/48;
  //°/s
  float last_speed_deg_s_l = last_degrees_l / (interruption_time/1000);
  float last_speed_deg_s_r = last_degrees_r / (interruption_time/1000);
  //rad/s
  float last_speed_rad_s_l = last_speed_deg_s_l * 0.017453292519943;
  float last_speed_rad_s_r = last_speed_deg_s_r * 0.017453292519943;

  /*
  Serial.print("last_speed_deg_s_l: ");
  Serial.println(last_speed_deg_s_l);
  Serial.print("last_speed_deg_s_r: ");
  Serial.println(last_speed_deg_s_r);

  Serial.print("last_speed_rad_s_l: ");
  Serial.println(last_speed_rad_s_l);
  Serial.print("last_speed_rad_s_r: ");
  Serial.println(last_speed_rad_s_r);
  */

  ticks_encoder_l = 0;
  ticks_encoder_r = 0;

  input_l = last_speed_deg_s_l;
  myController_l.compute();
  direction_l = 0;
  //move(0, output_l);
  move(0, 50);
  
  input_r = last_speed_deg_s_r;
  myController_r.compute();
  direction_r = 0;
  //move(1, output_r);
  move(1, 40);
  
  //wheel velocity l
  //float vl = wheel_radius_l * (last_speed_l*0.01745329251994);
  float vl = wheel_radius_l * last_speed_rad_s_l;
  //wheel velocity r
  //float vr = wheel_radius_r * (last_speed_r*0.01745329251994);
  float vr = wheel_radius_r * last_speed_rad_s_r;

  Serial.print("vl: ");
  Serial.println(vl);
  Serial.print("vr: ");
  Serial.println(vr);


  //float x = ((vl/2)*cos(last_theta)) +  ((vr/2)*cos(last_theta));
  float x = (((vl/2)*cos(last_theta)) +  ((vr/2)*cos(last_theta))) * (interruption_time/1000);
  float y = (((vl/2)*sin(last_theta)) +  ((vr/2)*sin(last_theta))) * (interruption_time/1000);
  //float theta = (-(wheel_radius_l/baseline)*(last_speed_l*0.01745329251994)) + ((wheel_radius_r/baseline)*(last_speed_r*0.01745329251994));
  //float theta = (-(wheel_radius_l/baseline)*last_speed_rad_s_l) + ((wheel_radius_r/baseline)*last_speed_rad_s_r);
  float theta = 0;

  last_x += x;
  last_y += y;
  last_theta += theta;

  Serial.print(">last_x:");
  Serial.println(last_x);

  Serial.print(">last_y:");
  Serial.println(last_y);

  Serial.print(">last_theta:");
  Serial.println(last_theta);

  
  if(last_x >= 1)
  {
     myController_l.stop();
     myController_r.stop();
      digitalWrite(standby, LOW);
  }
  

}

void count_encoder_l()
{
  if (direction_l == 0)
    ticks_encoder_l ++;
  else
    ticks_encoder_l --;
}

void count_encoder_r()
{
  if (direction_r == 0)
    ticks_encoder_r ++;
  else
    ticks_encoder_r --;
}

void setup()
{
  pinMode(led_pin, OUTPUT);

  pinMode(pwm_l, OUTPUT);
  pinMode(inl_1, OUTPUT);
  pinMode(inl_2, OUTPUT);

  pinMode(pwm_r, OUTPUT);
  pinMode(inr_1, OUTPUT);
  pinMode(inr_2, OUTPUT);

  pinMode(standby, OUTPUT);
  digitalWrite(standby, HIGH);

  pinMode(encoder_l, INPUT_PULLUP);
  pinMode(encoder_r, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_l), count_encoder_l, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_r), count_encoder_r, RISING);

  ticks_encoder_l = 0;
  ticks_encoder_r = 0;

  Serial.begin(9600);

  MsTimer2::set(interruption_time, interruption);
  MsTimer2::start();

  myController_l.begin(&input_l, &output_l, &setpoint_l, p_l, i_l, d_l);
  myController_l.setSampleTime(interruption_time);
  myController_l.setOutputLimits(0, 140);
  //myController.setBias(255.0 / 2.0);
  myController_l.setWindUpLimits(-50, 50);
  myController_l.start();

  myController_r.begin(&input_r, &output_r, &setpoint_r, p_r, i_r, d_r);
  myController_r.setSampleTime(interruption_time);
  myController_r.setOutputLimits(0, 140);
  //myController.setBias(255.0 / 2.0);
  myController_r.setWindUpLimits(-50, 50);
  myController_r.start();

}
void loop()
{
  //setpoint = 0.56;
  //direction_r = 0;
  //move(1, 30);
  //delay(2000);
  

  /*
  move(0, 30);
  direction_l = 1;
  move(1, 30);
  direction_r = 1;
  delay(2000);

  move(0, 30);
  direction_l = 0;
  move(1, 30);
  direction_r = 1;
  delay(2000);

  move(0, 30);
  direction_l = 1;
  move(1, 30);
  direction_r = 0;
  delay(2000);
  */
}