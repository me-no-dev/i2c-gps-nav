// sonar,cc.h

#ifndef _SONAR,CC_h
#define _SONAR,CC_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class sonar,cc
{
 private:


 public:
	void init();
};

extern sonar,cc SONAR,CC;

#endif

