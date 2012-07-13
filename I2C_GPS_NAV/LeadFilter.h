#ifndef LeadFilter_h
#define LeadFilter_h

#include "LeadFilter.h"

class LeadFilter{
  public:
       LeadFilter() :
		_last_velocity(0) {}
	int32_t 	get_position(int32_t pos, int16_t vel);
  private:
	int16_t		_last_velocity;

};

#endif
