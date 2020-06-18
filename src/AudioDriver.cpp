/*
 *  AudioDriver.cpp
 *
 *  Author: Gary Grutzek
 * 	gary@ib-gru.de
 *
 * AKM AK4556 Audio Serial Interface Format / Timing
 *
 * Operating modes are selected by the CKS3-0 pins as shown in the table
 *
 * | Mode 	| CKS3 	| CKS2 	| CKS1 	| CKS0 	| HPF 	| M/S 	| MCLK **	| Format|
 * |	0	|	L	|	L	|	L	|	L	|	ON	| SLAVE | 	1		| LJ/RJ	|
 * |	1	|	L	|	L	|	L	|	H	|	ON	| SLAVE |	2		| LJ/RJ	|
 * |	2	|	L	|	L	|	H	|	L	|	OFF	| SLAVE |	1		| LJ/RJ	|
 * |	3	|	L	|	L	|	H	|	H	|	OFF	| SLAVE |	2		| LJ/RJ	|
 * |	4	|	L	|	H	|	L	|	L	|	ON	| SLAVE |	1		|	I2S	|
 * |	5	|	L	|	H	|	L	|	H	|	ON	| SLAVE |	2		|	I2S	|
 * |	6	|	L	|	H	|	H	|	L	|	OFF	| SLAVE |	1		|	I2S	|
 * |	7	|	L	|	H	|	H	|	H	|	OFF	| SLAVE |	2		|	I2S	|
 * |	8	|	L	|	L	|	L	|	L	|	ON	| SLAVE |	1		|	LJ	|
 * |	9	|	H	|	L	|	L	|	H	|	ON	| SLAVE |	2		|	LJ	|
 * |	10	|	H	|	L	|	H	|	L	|	OFF	| SLAVE |	1		|	LJ	|
 * |	11	|	H	|	L	|	H	|	H	|	OFF	| SLAVE |	2		|	LJ	|
 * |	12	|	H	|	H	|	L	|	L	|	ON	| MASTER|	256fs	|	I2S	|
 * |	13	|	H	|	H	|	L	|	H	|	ON	| MASTER|	512fs	|	I2S	|
 * |	14	|	H	|	H	|	H	|	L	|	ON	| MASTER|	128fs	|	I2S	|
 * |	15	|	H	|	H	|	H	|	H	|	ON	| MASTER|	256fs	|	I2S	|
 *
 * ** MCLK rate
 *
 * 1:
 * 	128/192fs (Quad Speed)
 * 	256/384fs (Double Speed)
 * 	512/768fs (Normal Speed)
 *
 * 2:  256/384/512/768fs (Normal Speed)
 *
 */


#include "AudioDriver.h"

//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0 //A4 on adafruit feather huzzah32




//I assume these are driven by a value -1.0f to +1.0f, so they return a signed int of -127 to +127 etc.

// the esp32 DAC only outputs positive voltages, it needs the integer
// shifting into unsigned, using 128 as it's zero value
const float AudioDriver::ScaleFloat2Int = 128.0f; // 2^7(builtin dac is 8 bit), zero is 128 decimal
const float AudioDriver::ScaleInt2Float = float(1.0f/2048.0f); // 1 / 2^11 (builtin adc is 12bit, 4095 max value, 0 min)

//convert 12 bit adc to float [-1 to 1]
float int2Float(int16_t sample) {

    float l_sample = float(sample  * AudioDriver::ScaleInt2Float);

	return (float)(l_sample-1);
}

//convert float to int
uint16_t float2Int(float sample) {
    
    //this scales it but it is still a signed value
    sample *= AudioDriver::ScaleFloat2Int;
    
    //notice Y is still signed!
    int16_t y = (int16_t)(sample >= 0.5)? sample+1 : sample; //rounding
    
    //makes sure we are still in the right range after rounding
	y = constrain(y, -AudioDriver::ScaleFloat2Int, AudioDriver::ScaleFloat2Int-1);
    
    //now for our built-in dac, we need to offset zero to 128 decimal.
    y = y + AudioDriver::ScaleFloat2Int;
    
    //only care about the LSB
    y = y & 0x00ff;
    
    return (uint16_t)y;
    
    //not sure what this was doing in the original code...
	//return y << 8; //do we need this shift?

}


//
// the setup routine for using the builtin dac and adc
//
int AudioDriver::setupBuiltin(int fs, int channelCount, i2s_port_t i2s_port) 
{

	this->fs = fs;
	this->channelCount = channelCount;
	this->i2sPort = constrain(i2s_port, I2S_NUM_0, I2S_NUM_MAX);

	//int mclk_rate = fs * 384; // FIXME: factor depends on fs and CKS pins settings

	i2s_config_t i2s_config;
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN | I2S_MODE_ADC_BUILT_IN),
    i2s_config.sample_rate = fs;
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT; //I2S_BITS_PER_SAMPLE_24BIT; //should be 16 for builtin stuff? - I2S_BITS_PER_SAMPLE_16BIT
    i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT; // switch to left only?
    i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB);
    i2s_config.intr_alloc_flags = 0;//ESP_INTR_FLAG_LEVEL1; // high interrupt priority
    i2s_config.dma_buf_count = AudioDriver::BufferCount;
    i2s_config.dma_buf_len = AudioDriver::BufferSize;
    i2s_config.use_apll = false; //true;
    //i2s_config.fixed_mclk = mclk_rate;


    //install and start i2s driver
    esp_err_t err =i2s_driver_install(i2s_port, &i2s_config, 0, NULL);

    err += i2s_set_sample_rates(i2s_port, fs);

    err += i2s_zero_dma_buffer(i2s_port);

    //init DAC pad
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
    //init ADC pad
    i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL);
    
    i2s_adc_enable(i2s_port);
    
    // wait for stable clock
    delay(500);

    // I2S buffers
    i2sBufferSize =  channelCount * AudioDriver::BufferSize;    //there are 16 bits per sample, adc returns 
                                                                //there are 16 bits per sample, stored 0 first then byte for 8 bit dac

    i2sReadBuffer = (uint16_t*) calloc(i2sBufferSize, sizeof(uint16_t));
    i2sWriteBuffer= (uint16_t*) calloc(i2sBufferSize, sizeof(uint16_t));

    return err;
}





int AudioDriver::readBlock() {
	uint bytesRead = 0;

	int err = i2s_read(i2sPort, (void*) i2sReadBuffer, i2sBufferSize * sizeof(uint16_t), &bytesRead, portMAX_DELAY); //not port max timeout?, was 500

	if (err || bytesRead < (i2sBufferSize * sizeof(int16_t))) {
		Serial.print("I2S read error: ");
		Serial.println(bytesRead);
	}
	return err;
}


int AudioDriver::writeBlock() {
    
	uint bytesWritten = 0;
	int err = i2s_write(i2sPort, (const char *) i2sWriteBuffer, channelCount * AudioDriver::BufferSize * sizeof(uint16_t), &bytesWritten, portMAX_DELAY); //was 500

	if (bytesWritten < 1) {
		Serial.print("I2S write error: ");
		Serial.println(bytesWritten);
	}
	return err;
}





