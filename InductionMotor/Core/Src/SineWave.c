#include "SineWave.h"

void GenerateSine(ST_SineWave* SineWave, int* MicroSecond50){
	if (!*MicroSecond50){
		return;
	}

	if ((SineWave->FrequencyA != SineWave->WaveFrequency) && (SineWave->PhaseA <= ZERO_THRESHOLD && SineWave->PhaseA >= -1*ZERO_THRESHOLD)) SineWave->FrequencyA= SineWave->WaveFrequency;
	if ((SineWave->FrequencyB != SineWave->WaveFrequency) && (SineWave->PhaseB <= ZERO_THRESHOLD && SineWave->PhaseB >= -1*ZERO_THRESHOLD)) SineWave->FrequencyB= SineWave->WaveFrequency;
	if ((SineWave->FrequencyC != SineWave->WaveFrequency) && (SineWave->PhaseC <= ZERO_THRESHOLD && SineWave->PhaseC >= -1*ZERO_THRESHOLD)) SineWave->FrequencyC= SineWave->WaveFrequency;

	SineWave->PhaseA = SineWave->VoltageAmplitude * arm_sin_f32((2*PI*SineWave->Time*SineWave->FrequencyA/20000.0)				  );
	SineWave->PhaseB = SineWave->VoltageAmplitude * arm_sin_f32((2*PI*SineWave->Time*SineWave->FrequencyB/20000.0)	+ (2.0*PI/3.0));
	SineWave->PhaseC = SineWave->VoltageAmplitude * arm_sin_f32((2*PI*SineWave->Time*SineWave->FrequencyC/20000.0)	- (2.0*PI/3.0));

	SineWave->Time = (SineWave->Time + 1) % 40000;

	*MicroSecond50=0;


	return;
}
