#ifndef INC_SINEWAVE_H_
#define INC_SINEWAVE_H_


#include "stdint.h"
#include "arm_math.h"

#define PWM_MAX_VALUE 1000

#define MAX_FREQUENCY 60
#define MIN_FREQUENCY 20

#define ZERO_THRESHOLD 20

typedef struct {
	int32_t PhaseA;
	int32_t PhaseB;
	int32_t PhaseC;
	uint32_t Time;
	uint32_t WaveFrequency;
	uint32_t VoltageAmplitude;
	uint32_t FrequencyA;
	uint32_t FrequencyB;
	uint32_t FrequencyC;
}ST_SineWave;

void GenerateSine(ST_SineWave* SineWave, int* HundredMicroSecond);



#endif
