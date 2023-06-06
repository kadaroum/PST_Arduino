#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip

int ServoPortePin = 2;
char message = -1;
char mstemp = 0;

int PF = 98;
int PN = 90;
int PB = 83;

int GF = 180;
int GN = 90;
int GB = 0;

// température
int tmpPin = A1;

// LCD et LEDRGB TEMPERATURE
const int LCD_COLS = 16;
const int LCD_ROWS = 2;
const int LED_Temp_red = 8;
const int LED_Temp_blue = 5;
double tempe;
int input;
int blueTemp = 0;
int redTemp = 0;

// LED RGB ENTREE
const int LED_Entree_red = 11;   /* connected to PWM pin 11 */
const int LED_Entree_green = 13; /* connected to PWM pin 10 */
const int LED_Entree_blue = 12;  /* connected to PWM pin 9 */
int r = 255;                     /* red led value is temporally 255 and it will be the first led to light up */
int b;                           /* blue led value is temporally 0 */
int g;                           /* green led value is temporally 0 */
int t = 1;                       /* "t" (time) 1000 milliseconds, feel free to change it */

// LED_MAISON

const int LED_Entree = A7;
const int LED_Salon = A6;
const int LED_Garage = A8;
const int LED_Cuisine = A9;

// FAN

int Motor_Pin1 = 6; // pin 2 on L293D
int Motor_Pin2 = 7; // pin 7 on L293D
int Enable = 4;     // pin 1 on L293D

int Motor_Pin3 = 24; // pin 2 on L293D
int Motor_Pin4 = 26; // pin 7 on L293D
int Enable2 = 22;    // pin 1 on L293D

Servo myservo;
Servo myservo2;
SoftwareSerial bluetooth(10, 11); // (RX, TX) (pin Rx BT, pin Tx BT)

void setup()
{
  // Ouvre la voie série avec l'ordinateur
  Serial.begin(9600);
  // Ouvre la voie série avec le module BT
  bluetooth.begin(9600);
  // The servo control wire is connected to Arduino D2 pin.
  myservo.attach(2);
  myservo2.attach(3);
  // Servo is stationary
  myservo.write(PN);
  myservo2.write(GN);
  myservo.detach();
  myservo2.detach();

  digitalWrite(LED_Entree_red, LOW);
  digitalWrite(LED_Entree_green, LOW);
  digitalWrite(LED_Entree_blue, LOW);

  digitalWrite(53, LOW);
  digitalWrite(4, LOW); /// ???
  digitalWrite(7, LOW); /// ???

  ///////////////////////////////////////////////////////

  // RGB

  pinMode(LED_Entree_red, OUTPUT);
  pinMode(LED_Entree_blue, OUTPUT);
  pinMode(LED_Entree_green, OUTPUT);

  pinMode(LED_Temp_blue, OUTPUT);
  pinMode(LED_Temp_red, OUTPUT);

  // FAN

  pinMode(52, OUTPUT); // PETIT VENTILATEUR

  pinMode(Motor_Pin1, OUTPUT);
  pinMode(Motor_Pin2, OUTPUT);
  pinMode(Enable, OUTPUT);

  pinMode(Motor_Pin3, OUTPUT);
  pinMode(Motor_Pin4, OUTPUT);
  pinMode(Enable2, OUTPUT);

  // LED_MAISON

  pinMode(LED_Entree, OUTPUT);
  pinMode(LED_Salon, OUTPUT);
  pinMode(LED_Garage, OUTPUT);
  pinMode(LED_Cuisine, OUTPUT);

  // LCD

  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
  lcd.print("Temperature : ");
  lcd.setCursor(0, 1);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//FIN SETUP

// FONCTION IMPORTANTE

void servo(Servo servo, int power, int pin)
{
  servo.attach(pin);
  servo.write(power);
  delay(10);
}

/////////////////////////////////////// ENTREE

void LED_RGB_ENTREE(void)
{
  for (/* no initialization */; r >= 0, b < 255; b++, r--) /*red -> blue*/
  {
    analogWrite(LED_Entree_red, r); // (PIN, Valeur)
    analogWrite(LED_Entree_blue, b);
    delay(t);
  }
  for (/* no initialization */; b >= 0, g < 255; g++, b--) /*blue -> green*/
  {
    analogWrite(LED_Entree_blue, b);
    analogWrite(LED_Entree_green, g);
    delay(t);
  }
  for (/* no initialization */; g >= 0, r < 255; r++, g--) /*green -> red*/
  {
    analogWrite(LED_Entree_green, g);
    analogWrite(LED_Entree_red, r);
    delay(t);
  }
}

void PorteVerte()
{
  analogWrite(LED_Entree_green, 50); //POURQUOI 50 ?
  analogWrite(LED_Entree_red, 0);
  analogWrite(LED_Entree_blue, 0);
}
void PorteRouge()
{
  analogWrite(LED_Entree_red, 50);   //POURQUOI 50 ?
  analogWrite(LED_Entree_green, 0);
  analogWrite(LED_Entree_blue, 0);
}

/////////////////////////////////// TEMPERATURE

void temper()
{
  input = analogRead(tmpPin);
  tempe = ((input * 5.0) / 1024) / 0.01;

  if (tempe < 23)
  {
    analogWrite(LED_Temp_blue, 255);
  }
  else if (tempe > 23 && tempe <= 27)
  {
    blueTemp = map(tempe, 35, 75, 255, 0);
    analogWrite(LED_Temp_blue, blueTemp);
  }
  if (tempe < 23)
  {
    analogWrite(LED_Temp_red, 0);
  }
  else if (tempe >= 27)
  {
    redTemp = map(tempe, 45, 90, 1, 255);
    analogWrite(LED_Temp_red, redTemp);
  }
  else if (tempe > 29)
  {
    analogWrite(LED_Temp_red, 255);
  }
  delay(200);
}

void updateLCD()
{
  input = analogRead(tmpPin);
  tempe = (((input * 5.0) / 1024) / 0.01) - 3;
  unsigned long lcdInterval = 2000; // update 2 times per second
  lcd.setCursor(8, 1);
  lcd.print("       "); // overwrite old data
  lcd.setCursor(8, 1);  // reset the cursor
  lcd.print(tempe);
  lcd.print("C");
}

////////////////////// LED_MAISON

void TelevisionON()
{
}

void TelevisionOFF()
{
}

void CuissonON()
{
}

void CuissonOFF()
{
}
///////////////////////// FAN

void FAN_ON()
{
  analogWrite(Enable, 255);       // 0% PWM duty cycle
  digitalWrite(Motor_Pin1, LOW);  // To drive the motor in a particular direction
  digitalWrite(Motor_Pin2, HIGH); // To drive the motor in a particular direction
}

void FAN_OFF()
{
  analogWrite(Enable, 0); // 0% PWM duty cycle
  digitalWrite(Motor_Pin2, LOW);
  digitalWrite(Motor_Pin1, LOW);
}

void FAN_ON2()
{
  analogWrite(Enable2, 255);      // 0% PWM duty cycle
  digitalWrite(Motor_Pin3, LOW);  // To drive the motor in a particular direction
  digitalWrite(Motor_Pin4, HIGH); // To drive the motor in a particular direction
}

void FAN_OFF2()
{
  analogWrite(Enable2, 0); // 0% PWM duty cycle
  digitalWrite(Motor_Pin3, LOW);
  digitalWrite(Motor_Pin4, LOW);
}

////////////////////////////////////////::

void loop() // run over and over
{
  // TEMP ET LCD
  updateLCD();
  temper();

  ////////////////////////////////////////////////////////////
  if (bluetooth.available())
  {
    message = bluetooth.read();
    Serial.print(message);
    bluetooth.print(tempe);
  }

  if (Serial.available())
  {
    message = Serial.read();
    bluetooth.print(message);
  }

  Serial.print(message);
  Serial.print(" ");
  Serial.print(mstemp);
  Serial.print("\n");

  if (message != mstemp)
  {
    // Porte
    if (message == -1)
    {
      LED_RGB_ENTREE();
    }
    if (message == 1)
    {
      servo(myservo, PB, 2);
    }
    if (message == 2)
    {
      myservo.detach();
    }
    if (message == 3)
    {
      servo(myservo, PF, 2);
    }

    // Garage
    if (message == 4)
    {
      servo(myservo2, GF, 3);
    }
    if (message == 5)
    {
      myservo2.detach();
    }
    if (message == 6)
    {
      servo(myservo2, GB, 3);
    }

    // PorteColor
    if (message == 7)
    {
      PorteVerte();
    }
    if (message == 8)
    {
      PorteRouge();
    }

    //LUMIERE SALON
    if (message == 9)
    {
      digitalWrite(LED_Salon, HIGH);
    }
    if (message == 10)
    {
      digitalWrite(LED_Salon, LOW);
    }

    // CUISINE - PROBLEME
    if (message == 11)
    {
      digitalWrite(LED_Cuisine, HIGH);
    }
    if (message == 12)
    {
      digitalWrite(LED_Cuisine, LOW);
    }

    if (message == 13)
    {
      TelevisionON();
    }
    if (message == 14)
    {
      TelevisionOFF();
    }

    if (message == 15)
    {
      CuissonON();
    }
    if (message == 16)
    {
      CuissonOFF();
    }

    //VENTILATEUR SALON
    if (message == 17)
    {
      FAN_ON();
    }
    if (message == 18)
    {
      FAN_OFF();
    }
    
    //VENTILATEUR CUISINE
    if (message == 19)
    {
      FAN_ON2();
    }
    if (message == 20)
    {
      FAN_OFF2();
    }

    // PROBLEME
    if (message == 21)
    {
      digitalWrite(LED_Garage, HIGH);
    }
    if (message == 22)
    {
      digitalWrite(LED_Garage, LOW);
    }

    if (message == 23)
    {
      digitalWrite(LED_Entree, HIGH);
    }
    if (message == 24)
    {
      digitalWrite(LED_Entree, LOW);
    }
  }
  mstemp = message;
  message = 0;
}
/* A FAIRE PETIT VENTILATEUR
if(message == 9)
{digitalWrite(52, HIGH);}
if(message == 10)
{digitalWrite(52, LOW);}
*/
