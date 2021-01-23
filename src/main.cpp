#include <Arduino.h>
#include <PS3USB.h>
#include <PS4USB.h>


#define USB_HOST_RESET_PIN 9
#define ARDUINO_LED_PIN 17
#define MAX_INPUT_ANGLE 45

USB UsbHost;
PS3USB PS3Wired(&UsbHost);
PS4USB PS4Wired(&UsbHost);

const float PS3zeroG = 511.5f;

float accXraw;
float accYraw;
float accZraw;
float accXval;
float accYval;
float accZval;
float m_accXval;
float m_accYval;
float m_accZval;

float atan_m_accXVal;
float atan_m_accYVal;
float atan_m_accXVal_pi;
float atan_m_accYVal_pi;
float atan_m_accXVal_pi_rtd;
float atan_m_accYVal_pi_rtd;

float libXAngle;
float libYAngle;

int16_t castLibXAngle;
int16_t castLibYAngle;

float calcRoll;
float calcPitch;

uint16_t initRoll = 180;
uint16_t initPitch = 180;

uint16_t maxRoll = initRoll + MAX_INPUT_ANGLE; // 180 + 45 = 225
uint16_t minRoll = initRoll - MAX_INPUT_ANGLE; // 180 - 45 = 135
uint16_t maxPitch = initPitch + MAX_INPUT_ANGLE; // 180 + 45 = 225
uint16_t minPitch = initPitch - MAX_INPUT_ANGLE; // 180 - 45 = 135

uint16_t limitedRoll;
uint16_t limitedPitch;

int16_t zeroedRoll;
int16_t zeroedPitch;

float rollProportion = 0;
float pitchProportion = 0;

float invRollProportion = 0;
float invPitchProportion = 0;

int16_t intInvRollProportion = 0;
int16_t intInvPitchProportion = 0;

int16_t rightStickX;
int16_t rightStickY;

uint8_t controllerConnected();

int main(void)
{
  init();

  // Initialise the Serial Port
  Serial1.begin(500000);

  digitalWrite(USB_HOST_RESET_PIN, LOW);
  delay(20); //wait 20ms to reset the IC. Reseting at startup improves reliability in my experience.
  digitalWrite(USB_HOST_RESET_PIN, HIGH);
  delay(20); //Settle
  while (UsbHost.Init() == -1)
  {
      digitalWrite(ARDUINO_LED_PIN, !digitalRead(ARDUINO_LED_PIN));
      delay(500);
  }

  while (1) 
  {
    UsbHost.busprobe();
    UsbHost.Task();

    if (controllerConnected() == 1)
    {
      accXraw = (float)PS3Wired.getSensor(aX);
      accYraw = (float)PS3Wired.getSensor(aY);
      accZraw = (float)PS3Wired.getSensor(aZ);

      accXval = accXraw - PS3zeroG;
      accYval = accYraw - PS3zeroG;
      accZval = accZraw - PS3zeroG;

      m_accXval = -accXval;
      m_accYval = -accYval;
      m_accZval = -accZval;

      atan_m_accXVal = atan2f(m_accXval, m_accZval);
      atan_m_accYVal = atan2f(m_accYval, m_accZval);

      atan_m_accXVal_pi = atan_m_accXVal + PI;
      atan_m_accYVal_pi = atan_m_accYVal + PI;

      atan_m_accXVal_pi_rtd = atan_m_accXVal_pi * RAD_TO_DEG;
      atan_m_accYVal_pi_rtd = atan_m_accYVal_pi * RAD_TO_DEG;

      // libXAngle = PS3Wired.getAngle(Pitch);
      // libYAngle = PS3Wired.getAngle(Roll);

      castLibXAngle = (int16_t)PS3Wired.getAngle(Pitch);
      castLibYAngle = (int16_t)PS3Wired.getAngle(Roll);

      calcRoll = atan_m_accXVal_pi_rtd;
      calcPitch = atan_m_accYVal_pi_rtd;

      limitedRoll = calcRoll;
      limitedPitch = calcPitch;

      limitedRoll = (limitedRoll > maxRoll) ? maxRoll : limitedRoll;
      limitedRoll = (limitedRoll < minRoll) ? minRoll : limitedRoll;
      limitedPitch = (limitedPitch > maxPitch) ? maxPitch : limitedPitch;
      limitedPitch = (limitedPitch < minPitch) ? minPitch : limitedPitch;

      zeroedRoll = limitedRoll - 180;
      zeroedPitch = limitedPitch - 180;

      rollProportion = (float)zeroedRoll / MAX_INPUT_ANGLE;
      pitchProportion = (float)zeroedPitch / MAX_INPUT_ANGLE;

      invRollProportion = rollProportion * -1;
      invPitchProportion = pitchProportion * -1;
      
      intInvRollProportion = rollProportion * -1;
      intInvPitchProportion = pitchProportion * -1;

      rightStickX = invRollProportion * 32767;
      rightStickY = invPitchProportion * 32767;
    }

    Serial1.print("Raw X: ");
    Serial1.println(accXraw);
    Serial1.print("Raw Y: ");
    Serial1.println(accYraw);
    Serial1.print("Raw Z: ");
    Serial1.println(accZraw);
    Serial1.print("Zeroed X: ");
    Serial1.println(accXval);
    Serial1.print("Zeroed Y: ");
    Serial1.println(accYval);
    Serial1.print("Zeroed Z: ");
    Serial1.println(accZval);
    Serial1.print("Minus Zeroed X: ");
    Serial1.println(m_accXval);
    Serial1.print("Minus Zeroed Y: ");
    Serial1.println(m_accYval);
    Serial1.print("Minus Zeroed Z: ");
    Serial1.println(m_accZval);

    Serial1.print("Atan Minus Zeroed X: ");
    Serial1.println(atan_m_accXVal);
    Serial1.print("Atan Minus Zeroed Y: ");
    Serial1.println(atan_m_accYVal);

    Serial1.print("Atan Minus Zeroed X + PI: ");
    Serial1.println(atan_m_accXVal_pi);
    Serial1.print("Atan Minus Zeroed Y + PI: ");
    Serial1.println(atan_m_accYVal_pi);

    Serial1.print("Atan Minus Zeroed X + PI & RTD: ");
    Serial1.println(atan_m_accXVal_pi_rtd);
    Serial1.print("Atan Minus Zeroed Y + PI & RTD: ");
    Serial1.println(atan_m_accYVal_pi_rtd);

    // Serial1.print("Lib pitch: ");
    // Serial1.println(libXAngle);
    // Serial1.print("Lib roll: ");
    // Serial1.println(libYAngle);

    Serial1.print("Cast lib pitch: ");
    Serial1.println(castLibXAngle);
    Serial1.print("Cast lib roll: ");
    Serial1.println(castLibYAngle);

    Serial1.print("CalcRoll: ");
    Serial1.println(calcRoll);
    Serial1.print("CalcPitch: ");
    Serial1.println(calcPitch);

    Serial1.print("MaxRoll: ");
    Serial1.println(maxRoll);
    Serial1.print("MinRoll: ");
    Serial1.println(minRoll);    
    Serial1.print("MaxPitch: ");
    Serial1.println(maxPitch);
    Serial1.print("MinPitch: ");
    Serial1.println(minPitch);

    Serial1.print("Limited Roll: ");
    Serial1.println(limitedRoll);
    Serial1.print("Limited Pitch: ");
    Serial1.println(limitedPitch);

    Serial1.print("Zeroed Roll: ");
    Serial1.println(zeroedRoll);
    Serial1.print("Zeroed Pitch: ");
    Serial1.println(zeroedPitch);

    Serial1.print("Roll Proportion: ");
    Serial1.println(rollProportion);
    Serial1.print("Pitch Proportion: ");
    Serial1.println(pitchProportion);

    Serial1.print("Inverted Roll Proportion: ");
    Serial1.println(invRollProportion);
    Serial1.print("Inverted Pitch Proportion: ");
    Serial1.println(invPitchProportion);

    Serial1.print("Inverted Roll Proportion (int): ");
    Serial1.println(intInvRollProportion);
    Serial1.print("Inverted Pitch Proportion (int): ");
    Serial1.println(intInvPitchProportion);

    Serial1.print("Right Stick X: ");
    Serial1.println(rightStickX);
    Serial1.print("Right Stick Y: ");
    Serial1.println(rightStickY);    

    delay(100);
  }

}


uint8_t controllerConnected()
{
    uint8_t controllerType = 0;

    if (PS3Wired.PS3Connected)
		controllerType =  1;

	if (PS4Wired.connected())
		controllerType =  2;

    return controllerType;
}