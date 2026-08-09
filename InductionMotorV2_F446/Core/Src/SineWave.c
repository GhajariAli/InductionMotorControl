#include "SineWave.h"

#define TWO_PI 6.28318530718f
#define PHASE_120 (TWO_PI / 3.0f)
#define PHASE_240 (2.0f * TWO_PI / 3.0f)
#define ANGLE_THRESHOLD 0.05f  // ~3 degrees

void GenerateSine(ST_SineWave* SineWave, int* MicroSecond50) {
    if (!*MicroSecond50)
        return;

    // --- Update angles ---
    float delta = TWO_PI * SineWave->WaveFrequency / 20000.0f;  // 20 kHz (50 us) update rate: delta = 2*pi*F / 20000

    SineWave->AngleA += delta;
    SineWave->AngleB += delta;
    SineWave->AngleC += delta;

    // Wrap angles
    if (SineWave->AngleA >= TWO_PI) SineWave->AngleA -= TWO_PI;
    if (SineWave->AngleB >= TWO_PI) SineWave->AngleB -= TWO_PI;
    if (SineWave->AngleC >= TWO_PI) SineWave->AngleC -= TWO_PI;

    // --- Calculate new phase voltages ---
    float prevA = SineWave->PhaseA;

    SineWave->PhaseA = SineWave->VoltageAmplitude * arm_sin_f32(SineWave->AngleA);
    SineWave->PhaseB = SineWave->VoltageAmplitude * arm_sin_f32(SineWave->AngleB + PHASE_120);
    SineWave->PhaseC = SineWave->VoltageAmplitude * arm_sin_f32(SineWave->AngleC + PHASE_240);

    // --- Check conditions for frequency update ---
    // AngleA/B/C all track identically (offsets applied at sin-calc time),
    // so only the PhaseA rising zero-cross is needed to confirm a clean cycle boundary.
    int phaseA_zero_cross = (prevA < 0.0f && SineWave->PhaseA >= 0.0f);

    if (SineWave->FrequencyA != SineWave->WaveFrequency && phaseA_zero_cross)
    {
        SineWave->FrequencyA = SineWave->WaveFrequency;
        SineWave->FrequencyB = SineWave->WaveFrequency;
        SineWave->FrequencyC = SineWave->WaveFrequency;
    }

    // Reset the timer tick
    *MicroSecond50 = 0;
}
