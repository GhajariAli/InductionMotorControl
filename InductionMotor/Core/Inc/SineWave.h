#include "stdint.h"
#include "arm_math.h"

#define PWM_MAX_VALUE 1000

#define MAX_FREQUENCY 60
#define MIN_FREQUENCY 5

typedef struct {
	uint32_t PhaseA;
	uint32_t PhaseA_t;
	uint32_t PhaseAN;
	uint32_t PhaseAN_t;
	uint32_t PhaseB;
	uint32_t PhaseB_t;
	uint32_t PhaseBN;
	uint32_t PhaseBN_t;
	uint32_t PhaseC;
	uint32_t PhaseC_t;
	uint32_t PhaseCN;
	uint32_t PhaseCN_t;
	uint32_t WaveFrequency;
	uint32_t VoltageAmplitude;
}ST_SineWave;

void GenerateSine(ST_SineWave* SineWave, int* HundredMicroSecond);
