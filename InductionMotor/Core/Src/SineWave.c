#include "SineWave.h"

void GenerateSine(ST_SineWave* SineWave, int* HundredMicroSecond){
	if (!*HundredMicroSecond){
		return;
	}
	if (SineWave->PhaseA_t){
		SineWave->PhaseA=trunc( sin( (2*M_PI*SineWave->PhaseA_t*SineWave->WaveFrequency)/10000.0) * SineWave->VoltageAmplitude);
		if (SineWave->PhaseA_t<10000.0) SineWave->PhaseA_t++;
	}
	else SineWave->PhaseA=0;
	if (SineWave->PhaseB_t){
		SineWave->PhaseB=trunc( sin( (2*M_PI*SineWave->PhaseB_t*SineWave->WaveFrequency)/10000.0) * SineWave->VoltageAmplitude);
		if (SineWave->PhaseB_t<10000.0) SineWave->PhaseB_t++;
	}
	else SineWave->PhaseB=0;
	if (SineWave->PhaseC_t){
		SineWave->PhaseC=trunc( sin( (2*M_PI*SineWave->PhaseC_t*SineWave->WaveFrequency)/10000.0) * SineWave->VoltageAmplitude);
		if (SineWave->PhaseC_t<10000.0) SineWave->PhaseC_t++;
	}
	else SineWave->PhaseC=0;
	*HundredMicroSecond=0;
	return;
}
