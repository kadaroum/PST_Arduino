#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip

// LCD geometry
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

const int tmpPin = A0;
const int ledPinR = 9;
const int ledPinB = 8;
double tempe;
int input;
int blueTemp = 0;
int redTemp = 0;


void setup()
{
   lcd.begin(LCD_COLS, LCD_ROWS);
   lcd.clear();
   lcd.print("Temp : ");
   lcd.setCursor(0, 1);
}

void loop()
{
   updateLCD();
   temper();
  Serial.print("Temp : ");
  Serial.print(tempe);
  Serial.println(" C");
}

void updateLCD()
{
  input = analogRead(tmpPin);
  tempe = (input * 5.0 / 1024) * 10;
   unsigned long lcdInterval = 2000;  // update 2 times per second
      lcd.setCursor(8, 1);
      lcd.print("       "); // overwrite old data
      lcd.setCursor(8, 1);  // reset the cursor
      lcd.print(tempe);
      lcd.print("C");
   
}

void temper()
{
  input = analogRead(tmpPin);
  tempe = (input * 5.0 / 1024) * 100;

  if (tempe < 23) 
  {
    analogWrite(ledPinB, 255);
  }
  else if (tempe > 23 && tempe <= 27)
  {
    blueTemp = map (tempe, 35, 75, 255, 0);
    analogWrite (ledPinB, blueTemp);
  }
  if (tempe < 23)
  {
    analogWrite (ledPinR, 0);
  }
  else if (tempe >= 27)
  {
    redTemp = map (tempe, 45, 90, 1, 255);
    analogWrite (ledPinR, redTemp);
  }
  else if (tempe > 29)
  {
    analogWrite (ledPinR, 255);
  }
  delay(200);
  

}
