#include <stdint.h>

#define PWM_MAX_VALUE 120


typedef struct {
	int R;
	int G;
	int B;
}ST_RGBValues;

typedef enum {
	off		=0,
	Red 	=1,
	Green	=2,
	Blue	=3,
	White	=4,
	Yellow	=5,
	Cyan	=6,
	Magenta	=7,
	Orange	=8

}E_Colors;


int ReverseByte (uint8_t Number_int);
void ColorCodeGenerator(ST_RGBValues RGB, uint32_t ColorCode[24]);
ST_RGBValues RGBandIntensityGenerator(E_Colors color,uint8_t intensity);
void LEDPutC (E_Colors color,uint8_t intensity,char character,uint32_t CharOnLED[360]);
