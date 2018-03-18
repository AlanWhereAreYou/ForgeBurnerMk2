#include "TM1637_6D.h"

#define CLK 3 //pins definitions for TM1637 and can be changed to other ports
#define DIO 2

#define segA 0b01110111
#define segP 0b01110011
#define segS 0b01101101
#define segC 0b00111001

const long millisScreenToggleRate = 2000;

TM1637_6D tm1637_6D(CLK,DIO);


int dispPressure;
int dispRPM;
int dispSP;
int dispTemperature;

static int currentScreen = -1;
static long millisChangeScreenTime = millis();
int screenTopIndex=4;

void setup()
{
  tm1637_6D.init();
  // You can set the brightness level from 0(darkest) till 7(brightest) or use one
  // of the predefined brightness levels below
  tm1637_6D.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  tm1637_6D.displayFloat(0.0);
  Serial.begin(9600);
  ZeroPressureTransmitter();
}

void loop()
{ 
  manageScreens();
  manageSetpoint();
  
  delay(200);
}

void SetCurrentScreen(int screen, long duration)
{
  tm1637_6D.clearDisplay();
  currentScreen = screen;
  millisChangeScreenTime = millis() + duration;

  Serial.print(millis());
  Serial.print(" Changed Screen to ");
  Serial.print(currentScreen);
  Serial.print(" until ");
  Serial.println(millisChangeScreenTime);
}

void manageScreens()
{

  if(millis() > millisChangeScreenTime)
  {
    if(currentScreen>=screenTopIndex){SetCurrentScreen(0, millisScreenToggleRate);}
    else {SetCurrentScreen(++currentScreen, millisScreenToggleRate);}
  }

  switch(currentScreen)
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

void manageSetpoint()
{
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

