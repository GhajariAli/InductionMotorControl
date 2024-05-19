#include "SineWave.h"

void GenerateSine(ST_SineWave* SineWave, int* MicroSecond50){
	if (!*MicroSecond50){
		return;
	}
	if (SineWave->PhaseA_t){
		SineWave->PhaseA=trunc( sin( (2*M_PI*SineWave->PhaseA_t*SineWave->WaveFrequency)/5000.0) * SineWave->VoltageAmplitude);
		if (SineWave->PhaseA_t<5000.0) SineWave->PhaseA_t++;
	}
	else SineWave->PhaseA=0;
	if (SineWave->PhaseAN_t){
		SineWave->PhaseAN=trunc( sin( (2*M_PI*SineWave->PhaseAN_t*SineWave->WaveFrequency)/5000.0) * SineWave->VoltageAmplitude);
		if (SineWave->PhaseAN_t<5000.0) SineWave->PhaseAN_t++;
	}
	else SineWave->PhaseAN=0;
	if (SineWave->PhaseB_t){
		SineWave->PhaseB=trunc( sin( (2*M_PI*SineWave->PhaseB_t*SineWave->WaveFrequency)/5000.0) * SineWave->VoltageAmplitude);
		if (SineWave->PhaseB_t<5000.0) SineWave->PhaseB_t++;
	}
	else SineWave->PhaseB=0;
	if (SineWave->PhaseBN_t){
		SineWave->PhaseBN=trunc( sin( (2*M_PI*SineWave->PhaseBN_t*SineWave->WaveFrequency)/5000.0) * SineWave->VoltageAmplitude);
		if (SineWave->PhaseBN_t<5000.0) SineWave->PhaseBN_t++;
	}
	else SineWave->PhaseBN=0;
	if (SineWave->PhaseC_t){
		SineWave->PhaseC=trunc( sin( (2*M_PI*SineWave->PhaseC_t*SineWave->WaveFrequency)/5000.0) * SineWave->VoltageAmplitude);
		if (SineWave->PhaseC_t<5000.0) SineWave->PhaseC_t++;
	}
	else SineWave->PhaseC=0;
	if (SineWave->PhaseCN_t){
		SineWave->PhaseCN=trunc( sin( (2*M_PI*SineWave->PhaseCN_t*SineWave->WaveFrequency)/5000.0) * SineWave->VoltageAmplitude);
		if (SineWave->PhaseCN_t<5000.0) SineWave->PhaseCN_t++;
	}
	else SineWave->PhaseCN=0;
	*MicroSecond50=0;
	return;
}
