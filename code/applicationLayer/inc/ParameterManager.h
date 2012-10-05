/**
* \file ParameterManager.h
*
*/

#ifndef _PARAMETERMANAGER_H_
#define _PARAMETERMANAGER_H_

#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"

struct floatParameters{
  static const uint8_t maxNumberOfParameters = 50;
  static const uint8_t maxTextIdLength = 16;
  float* pointers[maxNumberOfParameters];
  char textIds[maxNumberOfParameters][maxTextIdLength+1];
  uint8_t textIdLengths[maxNumberOfParameters];
  uint16_t numberOfParameters; // Indicates how many parameters have been put into the array.
  float safetyFloat;  // The pointers in the parameter array point to this float by default to reduce the 
                            // negative effects of erroneously writing to a pointer that has not checked in.
};

class ParameterManager{
public :
  ParameterManager();
  ~ParameterManager();
  void checkInFloat(float* parameterPointer, char* textID, uint8_t textIDLength);
  void getFloat(uint16_t index, float* value, char* textID);
  void setFloat(uint16_t index, float value);
  uint16_t setFloat(float value, char* textID);
  uint16_t getNumberOfFloats();

private:
  floatParameters floats;
};

#endif  // _PARAMETERMANAGER_H_
