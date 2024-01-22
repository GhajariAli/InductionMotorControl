#include "LED.h"

int ReverseByte (uint8_t Number_int){
	int ReversedNumber=0;
	int sizeOfNumber= 8;
	for (int i=0;i<sizeOfNumber;i++){
		if(Number_int & (1<<i)){
			ReversedNumber|= 1<< ((sizeOfNumber-1)-i);
		}
	}
	return ReversedNumber;
}

ST_RGBValues RGBandIntensityGenerator(E_Colors color, uint8_t intensity){
	if (intensity>255) intensity = 255;
	else if (intensity<0) intensity = 0;
	ST_RGBValues RGB;
	switch (color){
		case (off) :
			RGB.G=0;RGB.R=0;RGB.B=0;
			break;
		case (Blue) :
			RGB.B=intensity;
			RGB.G=0;RGB.R=0;
			break;
		case Red :
			RGB.R=intensity;
			RGB.G=0;RGB.B=0;
			break;
		case Green:
			RGB.G=intensity;
			RGB.R=0;RGB.B=0;
			break;
		case White:
			RGB.G=intensity;
			RGB.R=intensity;
			RGB.B=intensity;
			break;
		case Yellow:
			RGB.G=intensity;
			RGB.R=intensity;
			RGB.B=0;
			break;
		case Cyan:
			RGB.G=intensity;
			RGB.B=intensity;
			RGB.R=0;
			break;
		case Magenta:
			RGB.R=intensity;
			RGB.B=intensity;
			RGB.G=0;
			break;
		case Orange:
			RGB.R=intensity;
			RGB.G=intensity/5;
			RGB.B=0;
			break;
	}
	return RGB;
}
void ColorCodeGenerator(ST_RGBValues RGB, uint32_t ColorCode[24]){

	uint32_t RGBData= ( ((ReverseByte(RGB.B))<<16) | ((ReverseByte(RGB.R))<<8) | (ReverseByte(RGB.G)));
	for (int i=23 ; i>=0 ; i--){
		if ( RGBData & (1<<i) ) {
			ColorCode[i]= PWM_MAX_VALUE *2/3;
		}
		else {
			ColorCode[i]= PWM_MAX_VALUE *1/3;
		}
	}

}

const int Leter_A[]={
1,1,1,1,0,
0,0,1,0,1,
1,1,1,1,0
};
