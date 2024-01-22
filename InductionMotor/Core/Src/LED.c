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

void LEDPutC (E_Colors color,uint8_t intensity,char character,uint32_t CharOnLED[360]){
	const int CharacterTable[]={
			0b111100010111110,0b010101010111111,0b100011000111111,0b011101000111111,0b101011010111111,//ABCDE
			0b000010010111111,0b011011010101110,0b0111110010011111,0b000001111100000,0b111111000011000,//FGHIJ
			0b100010101011111,0b100001000011111,0b111110011011111,0b111100010011111,0b011101000101110,//KLMNO
			0b001110010111111,0b111101100101110,0b100100110111111,0b010011010110010,0b000011111100001,//PQRST
			0b111111000011111,0b011111000001111,0b111110110011111,0b110110010011011,0b000111110000011,//UVWXY
			0b100111010111001
	  };
	uint32_t LEDData[24];
	for (int i=360-24;i>=0;i-=24){
		  if((CharacterTable[(int)character-65] & 1<<(i/24))){
			  ColorCodeGenerator(RGBandIntensityGenerator(color, intensity),LEDData);
		  }
		  else{
			  ColorCodeGenerator(RGBandIntensityGenerator(off, 0),LEDData);
		  }
		  for (int j=23;j>=0;j--){
			  CharOnLED[i+j]=LEDData[j];
		  }
	}
}

