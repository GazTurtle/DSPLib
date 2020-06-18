#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"


//https://github.com/espressif/esp-idf/blob/master/examples/peripherals/i2s_adc_dac/main/app_main.c



//i2s number
#define I2S_NUM           (i2s_port_t)(0)

#define BUFFER_LEN        1024

#define I2S_SAMPLE_BITS   (16)

//I2S read buffer length
#define I2S_READ_LEN      (I2S_SAMPLE_BITS/8) * BUFFER_LEN

//https://www.esp32.com/viewtopic.php?t=5574

//I2S built-in ADC unit
#define I2S_ADC_UNIT      ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL   ADC1_CHANNEL_0 //A4 on adafruit feather huzzah32



/**
 * @brief I2S ADC/DAC mode init.
 */
void i2s_init(void)
{
     i2s_port_t i2s_num = I2S_NUM;
    

    i2s_config_t i2s_config;
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN | I2S_MODE_ADC_BUILT_IN);
    i2s_config.sample_rate = 11025;//48000; //11025; // 96000;//44100;           
                       
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;  //I2S_CHANNEL_FMT_ONLY_LEFT
    i2s_config.use_apll = true;
    i2s_config.communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_PCM | I2S_COMM_FORMAT_I2S_MSB); //I2S_COMM_FORMAT_PCM;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL3; //ESP_INTR_FLAG_LEVEL3; // high interrupt priority //0;
    i2s_config.dma_buf_count = 12;//2;
    i2s_config.dma_buf_len = BUFFER_LEN;
    //i2s_config.tx_desc_auto_clear = true;
    
     //install and start i2s driver
     i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
     //init DAC pad
     i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
     //init ADC pad
     i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL);
}

void audioTask(void *);
void oscilatorTask(void *);

void setup()
{
    //Serial.begin(115200);
    i2s_init();

    i2s_adc_enable(I2S_NUM);

    // run audio in dedicated task on cpu core 1
    //xTaskCreatePinnedToCore(audioTask, "audioTask", 10000, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(oscilatorTask, "oscilatorTask", 10000, NULL, 10, NULL, 1);
}



/**
 * @brief Scale data to 8bit for data from ADC.
 *        Data from ADC are 12bit width by default.
 *        DAC can only output 8 bit data.
 *        Scale each 12bit ADC data to 8bit DAC data.
 */
void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint32_t dac_value = 0;
#if (I2S_SAMPLE_BITS == 16)
    for (int i = 0; i < len; i += 2) {
        dac_value = ((((uint16_t) (s_buff[i + 1] & 0x0f) << 8) | ((s_buff[i + 0]))));
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 4096;
 
    }
#else
    for (int i = 0; i < len; i += 4) {
        dac_value = ((((uint16_t)(s_buff[i + 3] & 0x0f) << 8) | ((s_buff[i + 2]))));        
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 4096;
    }
#endif
}

    int i2s_read_len = I2S_READ_LEN;

    size_t bytes_read, bytes_written;

    uint8_t* i2s_read_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    uint8_t* i2s_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));


float g_amount = 0.25f;

float l_direction = g_amount;

float counter = 0.0f;

void oscilatorTask(void *)
{

        
    while (true)
    {

      
      for (int i=0; i < i2s_read_len;)
      {
        i2s_write_buff[i++] = byte(0);

        float calc = (float(counter));// / 256.0f);
        
        i2s_write_buff[i++] = byte( int(calc) & 0xff);

        counter += l_direction;
  
        if (counter >= 0xff)
          l_direction = 0-g_amount;

         if (counter <= 0)
          l_direction = g_amount;
      }

      i2s_write(I2S_NUM, i2s_write_buff, i2s_read_len, &bytes_written, portMAX_DELAY);

    }
    vTaskDelete(NULL);
}

void audioTask(void *) {

  while(true)
  {
        //read data from I2S bus, in this case, from ADC.
        i2s_read(I2S_NUM, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY); //no timeout
       
        i2s_adc_data_scale(i2s_write_buff, i2s_read_buff, bytes_read);

        i2s_write(I2S_NUM, i2s_write_buff, bytes_read, &bytes_written, portMAX_DELAY);
  }
  vTaskDelete(NULL);
}

void loop()
{

   g_amount = float(analogRead(A5))/1023.0f;
//    g_amount = rand();

    delay(random(16)*16);
    // make a wave and play it out...
    // https://github.com/espressif/esp-idf/blob/7d75213674b4572f90c68162ad6fe9b16dae65ad/examples/peripherals/i2s/main/i2s_example_main.c



}
