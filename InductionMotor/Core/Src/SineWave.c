#include "SineWave.h"

void GenerateSine(ST_SineWave* SineWave, int* MicroSecond50){
	if (!*MicroSecond50){
		return;
	}

	SineWave->PhaseA = trunc(fabs(arm_sin_f32( 				 ((2*PI*SineWave->PhaseA_t*SineWave->WaveFrequency)/20000.0)) 	* SineWave->VoltageAmplitude));
	SineWave->PhaseB = trunc(fabs(arm_sin_f32(  2.0*PI/3.0 	+((2*PI*SineWave->PhaseB_t*SineWave->WaveFrequency)/20000.0)) 	* SineWave->VoltageAmplitude));
	SineWave->PhaseC = trunc(fabs(arm_sin_f32( -2.0*PI/3.0	+((2*PI*SineWave->PhaseC_t*SineWave->WaveFrequency)/20000.0)) 	* SineWave->VoltageAmplitude));

	SineWave->PhaseA_t++;
	SineWave->PhaseB_t++;
	SineWave->PhaseC_t++;

	if (SineWave->PhaseA_t>=40000) SineWave->PhaseA_t=0;
	if (SineWave->PhaseB_t>=40000) SineWave->PhaseB_t=0;
	if (SineWave->PhaseC_t>=40000) SineWave->PhaseC_t=0;

	*MicroSecond50=0;
	return;
}
