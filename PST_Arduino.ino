#include <Servo.h>
#include <SoftwareSerial.h>

int ServoPortePin = 2;
char message = -1;
char mstemp = 0;


int PF = 98;
int PN = 90;
int PB = 83;

int GF = 180;
int GN = 90;
int GB = 0;

//temp
int temp = 0; //A15

//led
const int red = 11; /* connected to PWM pin 11 */
const int green = 13; /* connected to PWM pin 10 */
const int blue = 12; /* connected to PWM pin 9 */
int r = 255; /* red led value is temporally 255 and it will be the first led to light up */
int b; /* blue led value is temporally 0 */
int g; /* green led value is temporally 0 */
int t = 1; /* "t" (time) 1000 milliseconds, feel free to change it */

Servo myservo;
Servo myservo2;
SoftwareSerial bluetooth (10, 11); // (RX, TX) (pin Rx BT, pin Tx BT)
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
  digitalWrite(red,LOW);
  digitalWrite(green,LOW);
  digitalWrite(blue,LOW);
  digitalWrite(53,LOW);
  digitalWrite(4,LOW);
  digitalWrite(7,LOW);
}

void led(void){
  for (/* no initialization */; r>=0, b<255; b++, r--) /*red -> blue*/
  {
  analogWrite(red, r);
  analogWrite(blue, b);
  delay(t);
  }
for (/* no initialization */; b>=0, g<255; g++, b--) /*blue -> green*/
{
  analogWrite(blue, b);
  analogWrite(green, g);
  delay(t);
}
for (/* no initialization */; g>=0, r<255; r++, g--) /*green -> red*/
{
  analogWrite(red, r);
  analogWrite(green, g);
  delay(t);
}
}
void PorteVerte(){
  analogWrite(green, 50);
  analogWrite(red, 0);
  analogWrite(blue, 0);
  }
 void PorteRouge(){
  analogWrite(red, 50);
  analogWrite(green, 0);
  analogWrite(blue, 0);
  }
void servo(Servo servo,int power,int pin)
{
  servo.attach(pin);
  servo.write(power);
  delay(10);
 
}

void loop() // run over and over 
{
  
if (bluetooth.available()) 
  {
    message = bluetooth.read(); 
    Serial.print (message);
  }

if (Serial.available()) {
  message = Serial.read();
  bluetooth.print (message);
  }
  
  Serial.print(message);
  Serial.print(" ");
  Serial.print(mstemp);
  Serial.print("\n");
  
  if(message != mstemp)
  {
    //Porte
    if(message == -1)
    {led();}
    if(message == 1)
    {servo(myservo,PB,2);}
    if(message == 2)
    {myservo.detach();}
    if(message == 3)
    {servo(myservo,PF,2);}

    //Garage
    if(message == 4)
    {servo(myservo2,GF,3);}
    if(message == 5)
    {myservo2.detach(); }
    if(message == 6)
    {servo(myservo2,GB,3);}

    //PorteColor
    if(message == 7)
    {PorteVerte();}
    if(message == 8)
    {PorteRouge();}
    //digitalWrite(4,LOW);
    digitalWrite(7,LOW);
  }
  mstemp = message;
  message = 0;
  

}
