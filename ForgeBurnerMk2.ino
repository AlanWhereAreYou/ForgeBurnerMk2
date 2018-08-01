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
const int pin_PWMFan = 9; // Arduino 9, this is D9/PB1/OC1A
const int pin_Pot = A1; // Arduino A1, PC1/ADC1
const int pin_Tach = 2; // Arduino 2, INT0/PD2

// Pins - Display
#define pin_CLK 6
#define pin_DIO 5

// BMP280 Pressure Transmitter (PT) Definitions
Adafruit_BMP280 Pt_Bme; // I2C
const int Pt_Address = 0x76;
float Pt_Hpa = 0.0;
float Pt_TempC = 0.0;

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
TachFeedbackData fan_TachData;
#define fan_millisTachCaptureInterval 500; // How frequently do we want to calculate the rpm.
#define fan_microsTachDebounceInterval 200;
volatile int fan_pulsesCountedByInterrupt = 0;
volatile long fan_microsCapturedByInterrupt = 0;
volatile bool fan_interrupt_Lock;
// volatile long fan_InterruptBounces = 0;

int fan_potInRaw = 0;
int fan_SetpointIntPct = 0;
const int fan_SpeedChangeRawDelta = 10;
const long fan_millisSpeedChangeFlagWindow = 5000;

// Display Definitions
#define segA 0b01110111
#define segP 0b01110011
#define segS 0b01101101
#define segC 0b00111001
#define segTest 0b11111111

TM1637_6D tm1637_6D(pin_CLK,pin_DIO);

// Screen Control
#define screen_SetpointLabel 0
#define screen_Setpoint 1
#define screen_ActualLabel 2
#define screen_Actual 3
#define screen_RelativePressureLabel 4
#define screen_RelativePressure 5
#define screen_TemperatureLabel 6
#define screen_Temperature 7

static int screen_Current = -1;
static long screen_millisChangeScreenTime = millis();
int screen_TopIndex = 7;
const long screen_millisToggleRate = 2000;

void setup()
{
  Fan_Setup();
  Serial.begin(9600);
  Serial.println("Setup");
  Display_Setup();
  Pt_Setup();
}

void loop()
{ 
  Screen_Manage();
  Fan_Manage();
  Pt_Manage();
  delay(200);
}

void Display_Setup()
{
  Serial.println("Display_Setup");
  tm1637_6D.init();
  tm1637_6D.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  for(int i=0;i<6;i++)
  {
    tm1637_6D.displayCodedByte(i,segTest);
  }
   delay(200);
   tm1637_6D.clearDisplay();
}

void Pt_Setup()
{
  Serial.println("PtSetup Starting BME");
  if (!Pt_Bme.begin(Pt_Address)) 
  {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }
  else
  {
    Serial.println("BME Started");
  }
}

void Pt_PressurePrint(float hPa)
{
  Serial.print("Pressure = ");
  Serial.print(hPa);
  Serial.println(" hPa");  
}

void Pt_TemperaturePrint(float degC)
{
  Serial.print("Temperature = ");
  Serial.print(degC);
  Serial.println(" *C");
}

void Fan_Setup()
{
  Timer1.initialize(40);  // 40 us = 25 kHz

  // Attach the tach interrupt
  pinMode(pin_Tach, INPUT_PULLUP);
  attachInterrupt(0, Fan_TachPulseInterrupt, RISING);  
  Fan_SetSpeed(0);
}

void Pt_Manage()
{
  static int loops = 0;
  Pt_Hpa = Pt_Bme.readPressure()/100.0;
  Pt_TempC = Pt_Bme.readTemperature();

  loops++;

  if (loops > 10)
  {
    Pt_TemperaturePrint(Pt_TempC);
    Pt_PressurePrint(Pt_Hpa); 
    loops = 0;
  }  
}

void Fan_Manage()
{
  static int last_fan_potInRaw = 0;
  static int fan_ChangeRaw = 0;

  Fan_TachCalculateData();
  
  fan_potInRaw = analogRead(pin_Pot);
  fan_SetpointIntPct = map(fan_potInRaw, 5, 1020, 0, 100);

  // Calculate ABS change. Can't do math within the ABS function.
  fan_ChangeRaw = last_fan_potInRaw - fan_potInRaw;
  fan_ChangeRaw = abs(fan_ChangeRaw);
      
  if (fan_ChangeRaw > fan_SpeedChangeRawDelta)
  {
    Serial.print("Pot: ");
    Serial.print(fan_potInRaw);
    Serial.print("   -   Pct: ");
    Serial.println(fan_SetpointIntPct);
    Screen_SetCurrent(screen_Setpoint, fan_millisSpeedChangeFlagWindow);
  }

  last_fan_potInRaw = fan_potInRaw;
  Fan_SetSpeed(fan_potInRaw);
}

void Fan_SetSpeed(int RawValue)
{
  Timer1.pwm(pin_PWMFan, RawValue);
  Serial.print("PWM Raw: ");
  Serial.println(RawValue);
}

/*
 *  Tach Program Concept: 
 *  We use Timer1 to control the PWM to the fan.
 *  We count the tach pulses manually using an interrupt.
 *  We use the main loop to read the tach pulse data and calculate rpms, etc.
 *  The main loop will do that calc slowly and relatively infrequently. While it is calculating rpm, we don't want the
 *   tach interrupt to modify the data, so we use a simple lock to prevent it. We do this instead of disabling interrupts.
 *   If the interrupt gets called while the calculation routine holds the lock, then it'll keep track of that missed pulse and add it
 *   in when it gets the lock back. 
 */
void Fan_TachPulseInterrupt(){
  static int  pulses_missed = 0;
  static long micros_DebounceEnd = 0;
  if(micros()>micros_DebounceEnd)
  { 
    if(!fan_interrupt_Lock){
      fan_pulsesCountedByInterrupt = fan_pulsesCountedByInterrupt + pulses_missed + 1;
      pulses_missed = 0;
      fan_microsCapturedByInterrupt = micros();
    }
    else {
      pulses_missed++;
    }
  }
  micros_DebounceEnd = micros()+fan_microsTachDebounceInterval;
}

String Fan_TachDebugString(TachFeedbackData data){
  return String(data.pulses) + " pulses in " + String(data.millis_Duration, 4) + "ms, " + String(data.freq_Hz,2) + "Hz, " + String (data.RPM) + " RPM";
}

void Fan_TachCalculateData(){
    static unsigned long millis_next_run = 0;

    // Only calculate the rpm if we have exceeded the minimum interval
    if(millis() < millis_next_run) {return;}

    // We do not want to interfere with our interrupt here, so we exert a lock.
    fan_interrupt_Lock = true;
    fan_TachData.micros_Current = fan_microsCapturedByInterrupt;
    fan_TachData.pulses = fan_pulsesCountedByInterrupt;
    fan_pulsesCountedByInterrupt = 0;
    fan_interrupt_Lock = false;

    millis_next_run = millis() + fan_millisTachCaptureInterval;

    fan_TachData.millis_Duration = (fan_TachData.micros_Current-fan_TachData.micros_Prev) / 1000.0;
    // There is a special case when the fan is stopped: Our pulses will be zero, and our interval can also be zero since it's updated in the
    // tach interrupt. Handle that case here by ignoring any data with a small duration.
    if (fan_TachData.millis_Duration < 50){
        fan_TachData.freq_Hz = 0;
        fan_TachData.RPM = 0;
    }
    else {
      fan_TachData.freq_Hz = (fan_TachData.pulses / fan_TachData.millis_Duration) * 1000;
      fan_TachData.RPM = fan_TachData.freq_Hz * 30; // (2 pulses per rev, so instead of Hz*60/2 just hardcode Hz*30)
    }
    
    fan_TachData.micros_Prev = fan_TachData.micros_Current;
    Serial.println(Fan_TachDebugString(fan_TachData));
}

void Screen_SetCurrent(int screen, long duration)
{
  if(screen != screen_Current) // Prevent flicker
  {
    tm1637_6D.clearDisplay();
  }
  screen_Current = screen;
  screen_millisChangeScreenTime = millis() + duration;
}

void Screen_Manage()
{
  if(millis() > screen_millisChangeScreenTime)
  {
    if(screen_Current>=screen_TopIndex){Screen_SetCurrent(0, screen_millisToggleRate);}
    else {Screen_SetCurrent(++screen_Current, screen_millisToggleRate);}
  }
  switch(screen_Current)
  {
    case screen_SetpointLabel: // Setpoint Screen
      ScreenRun_Label(segS);
      break;
    case screen_Setpoint: // Setpoint Screen
      ScreenRun_SetpointScreen();
      break;
    case screen_ActualLabel: // Actual Screen
      ScreenRun_Label(segA);
      break;
    case screen_Actual: // Actual Screen
      ScreenRun_ActualScreen();
      break;
    case screen_RelativePressureLabel: // Relative Pressure Screen
      ScreenRun_Label(segP);
      break;
    case screen_RelativePressure: // Relative Pressure Screen
      ScreenRun_RelativePressureScreen();
      break;
    case screen_TemperatureLabel: // Temperature Screen
      ScreenRun_Label(segC);
      break;    
    case screen_Temperature: // Temperature Screen
      ScreenRun_TemperatureScreen();
      break;
    default: // Error Screen
      ScreenRun_ErrorScreen();
      break;
  }
}

void ScreenRun_Label(int8_t digitByte)
{
  int8_t tempListDisp[6] = {digitByte,0,0,0,0,0}; // fill array with blank characters(10th)
  tm1637_6D.displayCodedByte(tempListDisp);
}
void ScreenRun_SetpointScreen()
{
  // Serial.println("showSetpointScreen");
  tm1637_6D.displayInteger(fan_SetpointIntPct,false);
}
void ScreenRun_ActualScreen()
{
  // Serial.println("showActualScreen");
  tm1637_6D.displayInteger((int)fan_TachData.RPM,0);
}
void ScreenRun_RelativePressureScreen()
{
  // Serial.println("showRelativePressureScreen");
  tm1637_6D.displayFloat(Pt_Hpa);
}
void ScreenRun_TemperatureScreen()
{
  // Serial.println("showTemperatureScreen");
  tm1637_6D.displayFloat(Pt_TempC);
}

void ScreenRun_ErrorScreen()
{
  // Serial.println("showErrorScreen");
  tm1637_6D.displayError();
}
