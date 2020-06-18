#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"


//https://github.com/espressif/esp-idf/blob/master/examples/peripherals/i2s_adc_dac/main/app_main.c



//i2s number
#define EXAMPLE_I2S_NUM           (i2s_port_t)(0)

#define BUFFER_LEN                800 //1024

#define EXAMPLE_I2S_SAMPLE_BITS   (16)

//I2S read buffer length
#define EXAMPLE_I2S_READ_LEN      (EXAMPLE_I2S_SAMPLE_BITS * BUFFER_LEN)

//https://www.esp32.com/viewtopic.php?t=5574

//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0 //A4 on adafruit feather huzzah32



/**
 * @brief I2S ADC/DAC mode init.
 */
void example_i2s_init(void)
{
     i2s_port_t i2s_num = EXAMPLE_I2S_NUM;
    

    i2s_config_t i2s_config;
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN | I2S_MODE_ADC_BUILT_IN);
              i2s_config.sample_rate = 11025;            
              i2s_config.dma_buf_len = BUFFER_LEN;                   
              i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
              i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;  
              i2s_config.use_apll = true, //false
              i2s_config.communication_format = I2S_COMM_FORMAT_I2S_MSB; //I2S_COMM_FORMAT_PCM;
              i2s_config.intr_alloc_flags = 0;
              i2s_config.dma_buf_count = 16;

     
     //install and start i2s driver
     i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
     //init DAC pad
     i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
     //init ADC pad
     i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL);
}

void setup()
{
    //Serial.begin(115200);
    example_i2s_init();
}



/**
 * @brief Scale data to 8bit for data from ADC.
 *        Data from ADC are 12bit width by default.
 *        DAC can only output 8 bit data.
 *        Scale each 12bit ADC data to 8bit DAC data.
 */
void example_i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint32_t dac_value = 0;
#if (EXAMPLE_I2S_SAMPLE_BITS == 16)
    for (int i = 0; i < len; i += 2) {
        dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 4096;


        //Serial.print("Read:");
        //Serial.println(dac_value);
        //Serial.print(" Write:");
        //Serial.println(d_buff[j-1]);
        
    }
#else
    for (int i = 0; i < len; i += 4) {
        dac_value = ((((uint16_t)(s_buff[i + 3] & 0xf) << 8) | ((s_buff[i + 2]))));

        //Serial.println(dac_value);
        
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 4096;
    }
#endif
}


void loop()
{

    // make a wave and play it out...
    // https://github.com/espressif/esp-idf/blob/7d75213674b4572f90c68162ad6fe9b16dae65ad/examples/peripherals/i2s/main/i2s_example_main.c

  
    int i2s_read_len = EXAMPLE_I2S_READ_LEN;

    size_t bytes_read, bytes_written;

    uint8_t* i2s_read_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    uint8_t* i2s_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));

    
    i2s_adc_enable(EXAMPLE_I2S_NUM);
    
    while (true)
    {
        //read data from I2S bus, in this case, from ADC.
        i2s_read(EXAMPLE_I2S_NUM, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
       
        example_i2s_adc_data_scale(i2s_write_buff, i2s_read_buff, bytes_read);
        
        i2s_write(EXAMPLE_I2S_NUM, i2s_write_buff, bytes_read, &bytes_written, portMAX_DELAY);
        //i2s_write(EXAMPLE_I2S_NUM, i2s_read_buff, bytes_read, &bytes_written, portMAX_DELAY);

        if (bytes_written != bytes_read)
        {
          //ooh heck probably not enough DMA buffer available for writting! Eeeek, don't worry, wait and try again with 
          //what ever data was unsent.
//          Serial.println("mismatch between what we read and what we wrote");
        }

    }

        
    i2s_adc_disable(EXAMPLE_I2S_NUM);
    free(i2s_read_buff);
    i2s_read_buff = NULL;
    free(i2s_write_buff);
    i2s_write_buff = NULL;
}
