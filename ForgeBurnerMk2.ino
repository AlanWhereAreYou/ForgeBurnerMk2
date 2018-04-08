/* Pins:
// D6 = TM1637 CLK
// D5 = TM1637 DIO
// A5 = BMP280 SCL
// A4 = BMP280 SDA
// 5V = BMP 5V + Pot 5V
// GND = BMP GND + Pot GND
// A1 = Pot Feedback
// D2 / INT0 = Fan Tach Feedback
// D9 / 0C1A = Fan PWM
*/ 

/*
 *  Tach Program Concept: 
 *  We use Timer1 to control the PWM to the fan.
 *  We count the tach pulses manually using an interrupt.
 *  We use the main loop to read the tach pulse data and calculate rpms, etc.
 *  The main loop will do that calc slowly and relatively infrequently. While it is calculating rpm, we don't want the
 *   tach interrupt to modify the data, so we use a simple lock to prevent it. We do this instead of disabling interrupts.
 *   This means that we might miss a count or two here and there. No big deal. We don't care.
 *  
 */

// BMP Library redistribution text
/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMEP280 Breakout 
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required 
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
 
#include <TM1637_6D.h> // Display
#include <TimerOne.h> // Fan control
#include <Wire.h> // I2C for Pressure Sensor (PT)
#include <SPI.h> // I2C for Pressure Sensor (PT)
#include <Adafruit_Sensor.h> // I2C for Pressure Sensor (PT)
#include <Adafruit_BMP280.h> // I2C for Pressure Sensor (PT)

// Pins - Fan
const int Pin_PWMFan = 9; // Arduino 9, this is D9/PB1/OC1A
const int Pin_Pot = A1; // Arduino A1, PC1/ADC1
const int Pin_Tach = 2; // Arduino 2, INT0/PD2

// Pins - Display
#define Pin_CLK 6
#define Pin_DIO 5

// BMP280 Pressure Transmitter (PT) Definitions
Adafruit_BMP280 bme; // I2C
const int PtAddress = 0x76;

// Fan Definitions
// On NMB Fan: Red = +12VDC, Black = GND, Brown = PWM, White = Tach
struct TachFeedbackData{
  long pulses;
  float millis_Duration;
  float freq_Hz;
  float RPM;
  unsigned long micros_Prev;
  unsigned long micros_Current;
};
TachFeedbackData TachData;
#define millis_TachCaptureInterval 500; // How frequently do we want to calculate the rpm.
volatile int pulsesCountedByInterrupt = 0;
volatile long microsCapturedByInterrupt = 0;
volatile bool interrupt_Lock;
int PotIn = 0;

// Display Definitions
#define segA 0b01110111
#define segP 0b01110011
#define segS 0b01101101
#define segC 0b00111001
TM1637_6D tm1637_6D(Pin_CLK,Pin_DIO);

// Screen Control
static int screenCurrent = -1;
static long millisChangeScreenTime = millis();
int screenTopIndex = 4;
const long millisScreenToggleRate = 2000;

void setup()
{
  Serial.begin(9600);
  SetupDisplay;
  SetupPT;
  SetupFan;
}

void SetupDisplay()
{
  tm1637_6D.init();
  tm1637_6D.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  // tm1637_6D.displayFloat(0.0);  
}

void SetupPT()
{
  bme.begin(PtAddress);
}

void SetupFan()
{
  Timer1.initialize(40);  // 40 us = 25 kHz

  // Attach the tach interrupt
  pinMode(Pin_Tach, INPUT_PULLUP);
  attachInterrupt(0, TachPulseInterrupt, FALLING);  
}

void loop()
{ 
  manageScreens();
  manageFanSpeed();
  managePT();
  TachCalculateData();
  delay(200);
}

void managePT()
{
  Serial.println("3");
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
   
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure()/100.0);
  Serial.println(" hPa");
}

void SetScreenCurrent(int screen, long duration)
{
  tm1637_6D.clearDisplay();
  screenCurrent = screen;
  millisChangeScreenTime = millis() + duration;

  Serial.print(millis());
  Serial.print(" Changed Screen to ");
  Serial.print(screenCurrent);
  Serial.print(" until ");
  Serial.println(millisChangeScreenTime);
}

void manageScreens()
{

  if(millis() > millisChangeScreenTime)
  {
    if(screenCurrent>=screenTopIndex){SetScreenCurrent(0, millisScreenToggleRate);}
    else {SetScreenCurrent(++screenCurrent, millisScreenToggleRate);}
  }

  switch(screenCurrent)
  {
    case 0: // Setpoint Screen
      showSetpointScreen();
      break;
    case 1: // Actual Screen
      showActualScreen();
      break;
    case 2: // Relative Pressure Screen
      showRelativePressureScreen();
      break;
    case 3: // Temperature Screen
      showTemperatureScreen();
      break;
    case 4: // Pressure Zeroing Screen
      showPressureZeroingScreen();
      break;
    default: // Error Screen
      showErrorScreen();
      break;
  }

}

int ReadPot()
{
  return analogRead(Pin_Pot);
}

void manageFanSpeed()
{
  PotIn = ReadPot();
  Timer1.pwm(Pin_PWMFan, PotIn);
}

void ZeroPressureTransmitter()
{

}

void showSetpointScreen()
{
  Serial.println("showSetpointScreen");
  tm1637_6D.displayByte(5,segS);
}
void showActualScreen()
{
  Serial.println("showActualScreen");
  tm1637_6D.displayByte(5,segA);
}
void showRelativePressureScreen()
{
  Serial.println("showRelativePressureScreen");
  tm1637_6D.displayByte(5,segP);
}
void showTemperatureScreen()
{
  Serial.println("showTemperatureScreen");
  tm1637_6D.displayByte(5,segC);
}
void showPressureZeroingScreen()
{
  Serial.println("showPressureZeroingScreen");
  tm1637_6D.displayInteger(123456,0);
  
}
void showErrorScreen()
{
  Serial.println("showErrorScreen");
  tm1637_6D.displayError();
}

void TachPulseInterrupt(){
  static int  pulses_missed = 0;
  
  if(!interrupt_Lock){
    pulsesCountedByInterrupt = pulsesCountedByInterrupt + pulses_missed + 1;
    pulses_missed = 0;
    microsCapturedByInterrupt = micros();
  }
  else {
    pulses_missed++;
  }
}

String TachDebugString(TachFeedbackData data){
  return String(data.pulses) + " pulses in " + String(data.millis_Duration, 4) + "ms, " + String(data.freq_Hz,2) + "Hz, " + String (data.RPM) + " RPM";
}

void TachCalculateData(){
    static unsigned long millis_next_run = 0;

    // Only calculate the rpm if we have exceeded the minimum interval
    if(millis() < millis_next_run) {return;}

    // We do not want to interfere with our interrupt here, so we exert a lock.
    interrupt_Lock = true;
    TachData.micros_Current = microsCapturedByInterrupt;
    TachData.pulses = pulsesCountedByInterrupt;
    pulsesCountedByInterrupt = 0;
    interrupt_Lock = false;

    millis_next_run = millis() + millis_TachCaptureInterval;

    TachData.millis_Duration = (TachData.micros_Current-TachData.micros_Prev) / 1000.0;
    // There is a special case when the fan is stopped: Our pulses will be zero, and our interval can also be zero since it's updated in the
    // tach interrupt. Handle that case here by ignoring any data with a small duration.
    if (TachData.millis_Duration < 50){
        TachData.freq_Hz = 0;
        TachData.RPM = 0;
    }
    else {
      TachData.freq_Hz = (TachData.pulses / TachData.millis_Duration) * 1000;
      TachData.RPM = TachData.freq_Hz * 30; // (2 pulses per rev, so instead of Hz*60/2 just hardcode Hz*30)
    }
    
    TachData.micros_Prev = TachData.micros_Current;
    Serial.println(TachDebugString(TachData));
}

