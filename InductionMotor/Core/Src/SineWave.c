#include "SineWave.h"

void GenerateSine(ST_SineWave* SineWave, int* MicroSecond50){
	if (!*MicroSecond50){
		return;
	}
	static uint32_t FrequencyA = MIN_FREQUENCY,FrequencyB = MIN_FREQUENCY,FrequencyC = MIN_FREQUENCY;

	if (FrequencyA != SineWave->WaveFrequency && SineWave->PhaseA < ZERO_THRESHOLD) FrequencyA= SineWave->WaveFrequency;
	if (FrequencyB != SineWave->WaveFrequency && SineWave->PhaseB < ZERO_THRESHOLD) FrequencyB= SineWave->WaveFrequency;
	if (FrequencyC != SineWave->WaveFrequency && SineWave->PhaseC < ZERO_THRESHOLD)	FrequencyC= SineWave->WaveFrequency;

	SineWave->PhaseA = trunc(fabs(arm_sin_f32( 				 ((2*PI*SineWave->PhaseA_t*FrequencyA)/20000.0)) 	* SineWave->VoltageAmplitude));
	SineWave->PhaseB = trunc(fabs(arm_sin_f32(  2.0*PI/3.0 	+((2*PI*SineWave->PhaseB_t*FrequencyB)/20000.0)) 	* SineWave->VoltageAmplitude));
	SineWave->PhaseC = trunc(fabs(arm_sin_f32( -2.0*PI/3.0	+((2*PI*SineWave->PhaseC_t*FrequencyC)/20000.0)) 	* SineWave->VoltageAmplitude));

	SineWave->PhaseA_t = (SineWave->PhaseA_t + 1) % 40000;
	SineWave->PhaseB_t = (SineWave->PhaseB_t + 1) % 40000;
	SineWave->PhaseC_t = (SineWave->PhaseC_t + 1) % 40000;

	*MicroSecond50=0;


	return;
}
