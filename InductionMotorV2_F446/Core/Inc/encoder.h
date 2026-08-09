#include <stdint.h>
enum E_direction {
  CW=1,
  CCW=2,
};

typedef struct{
	int InputGrayCode;
	int PreviusGrayDecode;
	enum E_direction direction;
	volatile int32_t EncoderValue ;	//written by TIM3 capture ISR
	int32_t PreviousEncoderValue;
	int32_t SpeedRPM;
}encoder_data;

void GetEncoderValue(encoder_data *encoder);
