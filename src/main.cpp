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

#include <Arduino.h>
#include <MsTimer2.h>

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

//int last_ticks_encoder_l;
//int last_ticks_encoder_r;

int interruption_time = 100;
//360degrees /48ppr
const float speed_constant = 7.5;

/*
motor = 0 = left
motor = 1 = right

direction = 0 = front
direction = 1 = back
*/
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
  float last_degrees_r = ticks_encoder_r * speed_constant;

  //°/s
  float last_speed_l = last_degrees_l * (speed_constant/1000);
  float last_speed_r = last_degrees_r * (speed_constant/1000);

  Serial.print(ticks_encoder_l);
  Serial.print(" - ");
  Serial.print(last_degrees_l);
  Serial.print(" - ");
  Serial.print(last_speed_l);
  Serial.print(" - ");
  Serial.print(ticks_encoder_r);
  Serial.print(" - ");
  Serial.print(last_degrees_r);
  Serial.print(" - ");
  Serial.print(last_speed_r);
  Serial.println("");

  ticks_encoder_l = 0;
  ticks_encoder_r = 0;
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
}
void loop()
{
  
  direction_l = 0;
  move(0, 30);
  direction_r = 0;
  move(1, 30);
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