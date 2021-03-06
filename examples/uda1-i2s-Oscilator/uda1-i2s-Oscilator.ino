/* I2S Example

    This example code will output 100Hz sine wave and triangle wave to 2-channel of I2S driver
    Every 5 seconds, it will change bits_per_sample [16, 24, 32] for i2s data

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/
#include "arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"
#include <math.h>

#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */



#define SAMPLE_RATE     (36000) 
#define I2S_NUM         (0)

uint16_t WAVE_FREQ_HZ = 80;

#define PI              (3.14159265)

#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)


void setup(void)
{
    //for 36Khz sample rates, we create 100Hz sine wave, every cycle need 36000/100 = 360 samples (4-bytes or 8-bytes each sample)
  //depend on bits_per_sample
  //using 6 buffers, we need 60-samples per buffer
  //if 2-channels, 16-bit each channel, total buffer is 360*4 = 1440 bytes
  //if 2-channels, 24/32-bit each channel, total buffer is 360*8 = 2880 bytes
  i2s_config_t i2s_config;
  
  i2s_config.mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX);                                  // Only TX
  i2s_config.sample_rate = SAMPLE_RATE;
  i2s_config.bits_per_sample = (i2s_bits_per_sample_t)I2S_BITS_PER_SAMPLE_16BIT;
  i2s_config.channel_format =  I2S_CHANNEL_FMT_RIGHT_LEFT;                         //2-channels
  i2s_config.communication_format =(i2s_comm_format_t)( I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
  i2s_config.dma_buf_count = 2;
  i2s_config.dma_buf_len = SAMPLE_PER_CYCLE*2;
  i2s_config.use_apll = 0;//APLL_DISABLE;
  i2s_config.fixed_mclk=-1;
  i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1    ;                            //Interrupt level 1


  i2s_driver_install((i2s_port_t)I2S_NUM, (i2s_config_t *) &i2s_config, 0, NULL);


  i2s_pin_config_t pins = {
    .bck_io_num = 26, //A0
    .ws_io_num =  25, // A1            //  wclk,
    .data_out_num = 27, //A10
    .data_in_num = I2S_PIN_NO_CHANGE
  };
        
  i2s_set_pin((i2s_port_t)I2S_NUM, (i2s_pin_config_t *) &pins);

  //i2s_set_clk((i2s_port_t)I2S_NUM, SAMPLE_RATE, (i2s_bits_per_sample_t)16, (i2s_channel_t)I2S_CHANNEL_STEREO);

    
  ads.begin();
}

uint16_t g_wavefold = 0;

static void setup_triangle_sine_waves(int bits)
{

    //2 channels, stereo, two bytes each, times the buffer size.


    //an int is 4 bytes  
    int *samples_data = (int *)malloc(((bits+8)/16)*SAMPLE_PER_CYCLE*4); //((bits+8)/16)  = 1 for 16bit and 2 for 24 bit and 32 bit, int maths
    
    
    //32 bits = 4 bytes
    unsigned int i, sample_val; 
    
    byte DAC_bits = bits;
    
    //8 bytes!
    double sin_float, triangle_float, triangle_step = (double) pow(2, DAC_bits) / SAMPLE_PER_CYCLE;
    
    size_t i2s_bytes_write = 0;

    //printf("\r\nTest bits=%d free mem=%d, written data=%d\n", bits, esp_get_free_heap_size(), ((bits+8)/16)*SAMPLE_PER_CYCLE*4);



    triangle_float = -(pow(2, DAC_bits)/2 - 1); //start mid-way

    for(i = 0; i < SAMPLE_PER_CYCLE; i++) 
    {
        sin_float = sin(i * 2 * PI / SAMPLE_PER_CYCLE);
        if(sin_float >= 0)
            triangle_float += triangle_step;
        else
            triangle_float -= triangle_step;

        sin_float *= g_wavefold;//(pow(2, DAC_bits)/2 - 1);

        //I think this is cleverly sending a triangle to one DAC channel (LEFT) and a sing wave to the right DAC channel!!!!

        //this code writes the left and right channel at the same time. so 32 bits in total.

        //I proved that putting FF in these locations sent that dac to max output... (3v)
        //notice the 8 bits of MSB is used by the built in dac

        //the endianess of esp32 puts the MSB into the memory first, then lsb, no need to swap them round.
        //0x0000ff00 = DAC on pin A0
        //0xFF000000 = DAC on pin A1
        
        sample_val = 0x00000000;
        sample_val = uint16_t(((uint16_t)triangle_float));// + (pow(2, DAC_bits)/2 - 1)); //a short is 2 bytes on esp32
        sample_val = sample_val << 16; //remember unsigned int is 4 bytes!, so this shifts (partway, next bit of shove below) towards first byte for channel A1
        
        sample_val += uint16_t(((uint16_t)sin_float));// + (pow(2, DAC_bits)/2 - 1)); //this is2 bytes, but needs swapping to little endian so the MSB is the 8 bits the dac wants
        //sample_val = sample_val << 8; //shift while thing one byte left, notice this moves the 8bits LSB of the 16 bit channel A1 value to the MSB position, first byte.
        samples_data[i] = sample_val;


    }


    i2s_write((i2s_port_t)I2S_NUM, samples_data, ((bits+8)/16)*SAMPLE_PER_CYCLE*4, &i2s_bytes_write, 100/*portMAX_DELAY*/); //was 100 timeout

    //printf("bytes written:%d\r\n",i2s_bytes_write);

    free(samples_data);
}

void loop(void)
{
   int16_t adc0, adc1, adc2, adc3;

    while (1) {
      adc0 = ads.readADC_SingleEnded(0);
      //adc1 = ads.readADC_SingleEnded(1);
      //adc2 = ads.readADC_SingleEnded(2);
      //adc3 = ads.readADC_SingleEnded(3);

      //WAVE_FREQ_HZ = float(adc0)/65535.0f * 3000; //random(30)*100;

      //float pot = float(adc0)/65535.0f;

     g_wavefold =  uint16_t(adc0+32760 ); //uint16_t (pot * 65535.0f);

//      WAVE_FREQ_HZ = uint8_t (pot * 50.0f)+100;
      
      setup_triangle_sine_waves(16);


      //vTaskDelay(500/portTICK_RATE_MS);
      //delay(125 * (random(3)+1));
      //delay(2000);

    }
}
