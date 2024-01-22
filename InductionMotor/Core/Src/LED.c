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


void ColorCodeGenerator(int R, int G, int B, uint32_t ColorCode[24]){

	uint32_t RGBData= ( ((ReverseByte(B))<<16) | ((ReverseByte(R))<<8) | (ReverseByte(G)));
	for (int i=23 ; i>=0 ; i--){
		if ( RGBData & (1<<i) ) {
			ColorCode[i]= 80;
		}
		else {
			ColorCode[i]= 40;
		}
	}

}
