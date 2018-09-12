#include <Wire.h>                 // "Wire" Library für die Kommunikation zum Real Time Clock Chip
#include <Adafruit_NeoPixel.h>    // "Adafruit NeoPixel" Library für die LEDs
// https://github.com/adafruit/Adafruit_NeoPixel
#include "TimerOne.h"             // "TimerOne" Library für den Drehencoder
// https://playground.arduino.cc/Code/Timer1

#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIN_LED 6  // LEDs sind am Pin D6 angeschlossen
#define NUM_PIXELS 60  // Anzahl LEDs = 60
#define LED_OFFSET 41  // Anpassung der 12 Uhr Ausrichtung

// Encoder Pins
#define PIN_ENCODER_A 3         // Interrupt Pin
#define PIN_ENCODER_B 4
#define PIN_BUTTON 5
#define PIN_LIGHT_SENSOR 3

// Real Time Clock (RTC) DS1307
#define DS1307_I2C_ADDRESS 0b1101000

// Button States
#define IS_PRESSED LOW
#define IS_RELEASED HIGH

// Color definitions
#define RED pixels.Color(225,0,0)
#define GREEN pixels.Color(0,255,0)
#define BLUE pixels.Color(0,0,255)
#define PURPLE pixels.Color(255,5,255)
#define LIGHT_RED pixels.Color(150,0,0)
#define LIGHT_WHITE pixels.Color(205,205,65)
#define LIGHT_BLUE pixels.Color(0,58,238)
#define DARK_BLUE pixels.Color(0, 71, 186)
#define MEINE_FARBE 0x50AB00

// Hier können die Farben eingestellt werden
#define HCOLOR DARK_BLUE  // color for hours
#define MCOLOR LIGHT_WHITE   // color for minutes
#define SCOLOR RED    // color for seconds

//Die zwei LEDs auf der Frontseite:
#define LED1 A1
#define LED2 A2
//digitalWrite(LED1,1);

#define COUNTER_BUTTON_PRESSED 10
#define TIME_TO_SWITCH_TO_NEXT_STATE 5

volatile signed int ec_pos = 0;
volatile bool encoder_en = true;
int sekunden = 0;
int second_ds = 0;
signed int stonde = 0;
unsigned  int clock_state = 0;
unsigned int time_h = 0;
unsigned int time_m = 0;
unsigned int time_s = 0;

unsigned int counter = 0;

float brightnessFilter = 0;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS, PIN_LED, NEO_GRB + NEO_KHZ800);

/*
   Setup
   -> the setup function runs once when you press reset or power the board
*/
void setup() {
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), doEncoderA, CHANGE);

  Wire.begin();                             //i2c (serielle Schnittstelle) starten

  Timer1.initialize(50000);                 // initialize timer1 to 50ms
  Timer1.attachInterrupt(timer1callback);   // attaches callback() as a timer overflow interrupt

  pixels.begin();                           //LED Starten
  // rtcset(30,3);           //(fkt setzt stunden und Minuten des RTC--> integer)

  Serial.begin(9600);
  int brightness = analogRead(PIN_LIGHT_SENSOR);
  //  Serial.print("Brightness Test: ");
  //  Serial.print(brightness);
  //  Serial.print("\n");
  Serial.print("Started...\n");
}

/*
   Main Loop as endless loop
*/
void loop() {
  // Clear LEDs
  pixels.clear();

  // Check if button is pressed
  boolean button_state = checkButtonState();

  // Check for clock state transition based upon button state
  clock_state = recalculateClockState(clock_state, button_state);

  // Move the hour, minute and second hand if necessary
  setClock();

  // Wait for 50ms
  delay(50);
}

boolean checkButtonState() {
  return digitalRead(PIN_BUTTON);
}

int recalculateClockState(int old_state, boolean button) {
  int new_state = old_state;
  switch (old_state) {
    case 0:
      time_h = rtchours();
      time_m = rtcminutes();
      time_s = rtcseconds();

      if (button == IS_PRESSED) {
        // Switch to clock setting
        new_state = 1;
      }
      break;
    case 1:
      counter++;
      if ((button == IS_PRESSED) && (counter > COUNTER_BUTTON_PRESSED)) {
        // Show animation to the user
        animation(1);

        ec_pos = 60 + time_h;
        new_state = 2;
        counter = 0;
      } else if (button == IS_RELEASED) {
        new_state = 0;
        counter = 0;
      }
      break;
    case 2:
      time_m = ec_pos % 60;
      if ((button == IS_PRESSED) && (counter > TIME_TO_SWITCH_TO_NEXT_STATE)) {
        ec_pos = 120 + time_m;
        animation(2);
        new_state = 3;
        counter = 0;
      } else if (button == IS_RELEASED) {
        counter++;
      }
      break;
    case 3:
      time_h = ec_pos % 12;
      if ((button == IS_PRESSED) && (counter > TIME_TO_SWITCH_TO_NEXT_STATE)) {
        rtcset(time_m, time_h);
        animation(3);
        new_state = 0;
        counter = 0;
      } else if (button == IS_RELEASED) {
        counter++;
      }
      break;
    default:
      new_state = 0;
  }
  return new_state;
}

void setClock() {
  // set hour clockhand moving every 5 minutes
  pixels.setPixelColor(((time_h + NUM_PIXELS) * 5 + LED_OFFSET-2 + (time_m / 12)) % NUM_PIXELS, HCOLOR);
  pixels.setPixelColor(((time_h + NUM_PIXELS) * 5 + LED_OFFSET-1 + (time_m / 12)) % NUM_PIXELS, HCOLOR);
  pixels.setPixelColor(((time_h + NUM_PIXELS) * 5 + LED_OFFSET   + (time_m / 12)) % NUM_PIXELS, HCOLOR);
  pixels.setPixelColor(((time_h + NUM_PIXELS) * 5 + LED_OFFSET+1 + (time_m / 12)) % NUM_PIXELS, HCOLOR);
  pixels.setPixelColor(((time_h + NUM_PIXELS) * 5 + LED_OFFSET+2 + (time_m / 12)) % NUM_PIXELS, HCOLOR);
  // set minute clockhand
  pixels.setPixelColor((time_m + LED_OFFSET) % NUM_PIXELS, MCOLOR);
  // set second clockhand
  pixels.setPixelColor((time_s + LED_OFFSET) % NUM_PIXELS, SCOLOR);

  // check brightness
  brightnessFilter = (29 * brightnessFilter + getBright()) / 30; // Moving Average Filter
  pixels.setBrightness(brightnessFilter);

  pixels.show();
}

/*
   Encoder Interrupt Routine
*/
void doEncoderA() {
  // look for a low-to-high on channel A
  if (encoder_en) {
    encoder_en = false;
    if (digitalRead(PIN_ENCODER_A) == HIGH) {
      // check channel B to see which way encoder is turning
      if (digitalRead(PIN_ENCODER_B) == LOW) {
        ec_pos = ec_pos + 1;         // CW
      }
      else {
        ec_pos = ec_pos - 1;         // CCW
       
      }
    } else {
      // must be a high-to-low edge on channel A
      // check channel B to see which way encoder is turning
      if (digitalRead(PIN_ENCODER_B) == HIGH) {
        ec_pos = ec_pos + 1;          // CW
      } else {
        ec_pos = ec_pos - 1;          // CCW
      }
    }
    Timer1.start();
  }
}

/*
   Timer1 Interrupt Routine (Used for encoder debouncing)
*/

void timer1callback() {
  encoder_en = true;
  Timer1.stop();
}

/*
   Sekunden von RTC Abfragen
   Input: --
   Output: Integer Sekunden (0-59)
*/
unsigned int rtcseconds(void) {
  Wire.beginTransmission(DS1307_I2C_ADDRESS);      // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x00);                       //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(DS1307_I2C_ADDRESS, 1);         // Tell slave we need to read 1byte from the current register
  int sec = Wire.read();                      // read that byte into variable
  Wire.endTransmission();                 // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
  return sec % 16 + (10 * (sec >> 4));    //BCD zu dezimal umrechen
}

/*
   Minuten von RTC Abfragen
   Input: --
   Output: Integer Minuten (0-59)
*/
unsigned int rtcminutes(void) {
  Wire.beginTransmission(DS1307_I2C_ADDRESS);      // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x01);                       //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(DS1307_I2C_ADDRESS, 1);         // Tell slave we need to read 1byte from the current register
  int m = Wire.read();                    // read that byte into variable
  Wire.endTransmission();                 // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
  return m % 16 + (10 * (m >> 4));        //BCD zu dezimal umrechen
}

/*
   Stunden von RTC Abfragen
   Input: --
   Output: Integer Stunden
*/
unsigned int rtchours(void) {
  Wire.beginTransmission(DS1307_I2C_ADDRESS);      // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x02);                       // Set the address to the hour register
  Wire.endTransmission();
  Wire.requestFrom(DS1307_I2C_ADDRESS, 1);         // Tell slave we need to read 1byte from the current register
  int h = Wire.read();                    // read that byte into variable
  Wire.endTransmission();                 // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
  return h % 16 + (10 * (h >> 4));        // BCD zu dezimal umrechen
}

/*
   RTC Zeit einstellen                     TODO:Erwartet BCD Werte (--> noch anpassen)
   Input: Integer Minuten
          Integer Stunden
   Output: --
*/
void rtcset(int m, int h) {
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0x00);                       // set next input to start at the seconds register
  Wire.write(0);                          // set seconds

  m = m % 10 + ((m / 10) << 4);
  Wire.write(m);
  // set min
  h = h % 10 + ((h / 10) << 4);
  Wire.write(h);                         // set hours
  Wire.endTransmission();
}

/*
  Helligkeit abfragen
  Input: --
  Output: Integer mit Helligkeitswert. Gefiltert und skaliert 0-255 (dunkel-hell)
*/
int getBright(void) {
  signed int brt = 0;
  brt = analogRead(PIN_LIGHT_SENSOR);
  if (brt >= 512) {
    brt = (234 - (brt - 512) / 2);
  }
  else {
    brt = 255;
  }
  if (brt < 3) {
    brt = 3;
  }
  return brt;
}

/*
  Animation für den Uebergang zum Zeit-Programmiermodus
*/
void animation(int sel) {
  switch (sel) {
    case 1:
      for (int i = 0; i < NUM_PIXELS; i++) {
        int red = 0 + i * 4;
        pixels.setPixelColor(59 - i, pixels.Color(red, 0, 0));
        // from 0 (off) to 255 (max brightness)
        pixels.setBrightness(i*4);
        pixels.show();
        delay(20);
      }
      break;
    case 2:
      for (int loop = 0; loop < 22; loop++) {
        int green = loop * 10;
        for (int i = 0; i < NUM_PIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(0, green, 0));
        }
        // from 0 (off) to 255 (max brightness)
        pixels.setBrightness(loop*10);
        pixels.show();
        delay(30);
      }
      break;
    case 3:
      for (int i = 0; i < NUM_PIXELS; i++) {
        pixels.clear();
        int green = 240 - i * 4;
        pixels.setPixelColor(i, pixels.Color(0, green, 0));
        // from 0 (off) to 255 (max brightness)
        pixels.setBrightness(255 - i*4);
        pixels.show();
        delay(20);
      }
  }
}
