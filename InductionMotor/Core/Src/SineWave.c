#include "SineWave.h"
void GenerateSine(ST_SineWave* SineWave, int* MicroSecond50){
	if (!*MicroSecond50){
		return;
	}
	if (SineWave->PhaseA_t){
		SineWave->PhaseA = trunc(arm_sin_f32( (2*PI*SineWave->PhaseA_t*SineWave->WaveFrequency)/20000.0) 	* SineWave->VoltageAmplitude);
		if (SineWave->PhaseA_t<20000) 	SineWave->PhaseA_t++;
	}
	else SineWave->PhaseA = 0;

	if (SineWave->PhaseAN_t){
		SineWave->PhaseAN = trunc(arm_sin_f32( (2*PI*SineWave->PhaseAN_t*SineWave->WaveFrequency)/20000.0) 	* SineWave->VoltageAmplitude);
		if (SineWave->PhaseAN_t<20000) 	SineWave->PhaseAN_t++;
	}
	else SineWave->PhaseAN = 0;

	if (SineWave->PhaseB_t){
		SineWave->PhaseB = trunc(arm_sin_f32( (2*PI*SineWave->PhaseB_t*SineWave->WaveFrequency)/20000.0) 	* SineWave->VoltageAmplitude);
		if (SineWave->PhaseB_t<20000) 	SineWave->PhaseB_t++;
	}
	else SineWave->PhaseB = 0;

	if (SineWave->PhaseBN_t){
		SineWave->PhaseBN = trunc(arm_sin_f32( (2*PI*SineWave->PhaseBN_t*SineWave->WaveFrequency)/20000.0) 	* SineWave->VoltageAmplitude);
		if (SineWave->PhaseBN_t<20000) 	SineWave->PhaseBN_t++;
	}
	else SineWave->PhaseBN = 0;

	if (SineWave->PhaseC_t){
		SineWave->PhaseC = trunc(arm_sin_f32( (2*PI*SineWave->PhaseC_t*SineWave->WaveFrequency)/20000.0) 	* SineWave->VoltageAmplitude);
		if (SineWave->PhaseC_t<20000) 	SineWave->PhaseC_t++;
	}
	else SineWave->PhaseC = 0;

	if (SineWave->PhaseCN_t){
		SineWave->PhaseCN = trunc(arm_sin_f32( (2*PI*SineWave->PhaseCN_t*SineWave->WaveFrequency)/20000.0)	* SineWave->VoltageAmplitude);
		if (SineWave->PhaseCN_t<20000) 	SineWave->PhaseCN_t++;
	}
	else SineWave->PhaseCN=0;

	*MicroSecond50=0;
	return;
}
