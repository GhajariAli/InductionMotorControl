#include "SineWave.h"
void GenerateSine(ST_SineWave* SineWave, int* MicroSecond50){
	if (!*MicroSecond50){
		return;
	}
	uint32_t Value=0;

	if (SineWave->PhaseA_t){
		Value= arm_sin_f32( (2*M_PI*SineWave->PhaseA_t*SineWave->WaveFrequency)/5000.0) 	* SineWave->VoltageAmplitude;
		if (Value<1) SineWave->PhaseA = 1;
		else SineWave->PhaseA = trunc(Value);

		if (SineWave->PhaseA_t<5000.0) 	SineWave->PhaseA_t++;
	}
	else SineWave->PhaseA = 0;

	if (SineWave->PhaseAN_t){
		Value= arm_sin_f32( (2*M_PI*SineWave->PhaseAN_t*SineWave->WaveFrequency)/5000.0) 	* SineWave->VoltageAmplitude;
		if (Value<1) SineWave->PhaseAN = 1;
		else SineWave->PhaseAN = trunc(Value);

		if (SineWave->PhaseAN_t<5000.0) SineWave->PhaseAN_t++;
	}
	else SineWave->PhaseAN = 0;

	if (SineWave->PhaseB_t){
		Value = arm_sin_f32( (2*M_PI*SineWave->PhaseB_t*SineWave->WaveFrequency)/5000.0) 	* SineWave->VoltageAmplitude;
		if (Value<1) SineWave->PhaseB = 1;
		else SineWave->PhaseB = trunc(Value);

		if (SineWave->PhaseB_t<5000.0) 	SineWave->PhaseB_t++;
	}
	else SineWave->PhaseB = 0;

	if (SineWave->PhaseBN_t){
		Value =	arm_sin_f32( (2*M_PI*SineWave->PhaseBN_t*SineWave->WaveFrequency)/5000.0) 	* SineWave->VoltageAmplitude;
		if (Value<1) SineWave->PhaseBN = 1;
		else SineWave->PhaseBN = trunc(Value);

		if (SineWave->PhaseBN_t<5000.0) SineWave->PhaseBN_t++;
	}
	else SineWave->PhaseBN = 0;

	if (SineWave->PhaseC_t){
		Value =	arm_sin_f32( (2*M_PI*SineWave->PhaseC_t*SineWave->WaveFrequency)/5000.0) 	* SineWave->VoltageAmplitude;
		if (Value<1) SineWave->PhaseC = 1;
		else SineWave->PhaseC = trunc(Value);

		if (SineWave->PhaseC_t<5000.0) 	SineWave->PhaseC_t++;
	}
	else SineWave->PhaseC = 0;

	if (SineWave->PhaseCN_t){
		Value =	arm_sin_f32( (2*M_PI*SineWave->PhaseCN_t*SineWave->WaveFrequency)/5000.0) 	* SineWave->VoltageAmplitude;
		if (Value<1) SineWave->PhaseCN = 1;
		else SineWave->PhaseCN = trunc(Value);

		if (SineWave->PhaseCN_t<5000.0) SineWave->PhaseCN_t++;
	}
	else SineWave->PhaseCN=0;

	*MicroSecond50=0;
	return;
}
