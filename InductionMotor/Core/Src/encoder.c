#include "encoder.h"

void GetEncoderValue(encoder_data *encoder){
	  int EncoderGrayConvert[4] ={0,1,3,2};
	  int GrayDecode = EncoderGrayConvert[encoder->InputGrayCode];
	  if (encoder->PreviusGrayDecode != GrayDecode){
		  int EncoderDeltaValue = GrayDecode-encoder->PreviusGrayDecode;
		  if (EncoderDeltaValue<0) {EncoderDeltaValue+=4;}
		  if (EncoderDeltaValue ==1) {
			  encoder->direction=CW;
			  encoder->EncoderValue++;
		  }
		  else if (EncoderDeltaValue ==3){
			  encoder->direction=CCW;
			  encoder->EncoderValue--;
		  }
		  encoder->PreviusGrayDecode=GrayDecode;
	 }

}

