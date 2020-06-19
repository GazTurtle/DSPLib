/*
 * 	AudioDriver.h
 *
 *  Author: Gary Grutzek
 * 	gary@ib-gru.de
 */

#ifndef AUDIODRIVER_H_
#define AUDIODRIVER_H_

#include "Arduino.h"
// include espressif hw files
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "soc/syscon_reg.h"
#include "driver/adc.h"




// TODO: abstraction layer for ADC / DAC / PDM / CODEC Shields


uint16_t float2Int(float sample);
float int2Float(int16_t sample);

class AudioDriver {

public:
	static const int BufferCount=2; //started at 2 in YummyDSP lib -must be at least 2
	
	// make it a power of two for best DMA performance
	static const int BufferSize=16;//32; // increase to 64 samples to avoid drop outs (at 192 kHz)
	static const float ScaleFloat2Int;
	static const float ScaleInt2Float;

    //no pins defs needed for builtin adc and dac on the esp32
    int setupBuiltin(int fs=11200, int channelCount=1, i2s_port_t i2s_port=(i2s_port_t)(0));

	uint readBlock();
	uint readBlock(uint p_bytesToRead);
	uint writeBlock(uint p_bytesToWrite);
	uint writeBlock();

	inline float readSample(int n, int channel) {
        
        //does the sampling from builtin ADC return data in the MSB portion? not sure how that would work with 12 bits!?
		return int2Float(i2sReadBuffer[(channelCount * n ) + channel]);
	}


	inline void writeSample(float sample, int n, int channel) {

        int16_t l_sampleAsInt = float2Int(sample);
        
        uint16_t l_msbFirtSampleInt = l_sampleAsInt & 0x00ff;
        l_msbFirtSampleInt = l_msbFirtSampleInt << 8;
        
        //swap the LSB to MSB
        
        i2sWriteBuffer[channelCount * n + channel] = l_msbFirtSampleInt;

	}

protected:

	int fs;
	int channelCount;

	i2s_port_t i2sPort;
	int i2sBufferSize;
    
	uint16_t* i2sReadBuffer;
	uint16_t* i2sWriteBuffer;

};



#endif /* AUDIODRIVER_H_ */
