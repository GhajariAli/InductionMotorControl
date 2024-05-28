#include "MotorSequencer.h"

void MotorSequence(ST_SineWave* Wave,int State){
	switch (State){
	  case 1:
		  Wave->PhaseA_t=1;
		  Wave->PhaseAN_t=0;
		  break;
	  case 2:
		  Wave->PhaseC_t=0;
		  Wave->PhaseCN_t=1;
		  break;
	  case 3:
		  Wave->PhaseB_t=1;
		  Wave->PhaseBN_t=0;
		  break;
	  case 4:
		  Wave->PhaseA_t=0;
		  Wave->PhaseAN_t=1;
		  break;
	  case 5:
		  Wave->PhaseC_t=1;
		  Wave->PhaseCN_t=0;
		  break;
	  case 6:
		  Wave->PhaseB_t=0;
		  Wave->PhaseBN_t=1;
		  break;
	  }
}
