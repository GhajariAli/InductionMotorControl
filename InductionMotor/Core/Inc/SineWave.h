#ifndef INC_SINEWAVE_H_
#define INC_SINEWAVE_H_


#include "stdint.h"
#include "arm_math.h"

#define PWM_MAX_VALUE 1000

#define MAX_FREQUENCY 60
#define MIN_FREQUENCY 1

typedef struct {
	uint32_t PhaseA;
	uint32_t PhaseA_t;
	uint32_t PhaseB;
	uint32_t PhaseB_t;
	uint32_t PhaseC;
	uint32_t PhaseC_t;
	uint32_t WaveFrequency;
	uint32_t VoltageAmplitude;
}ST_SineWave;

void GenerateSine(ST_SineWave* SineWave, int* HundredMicroSecond);



#endif
