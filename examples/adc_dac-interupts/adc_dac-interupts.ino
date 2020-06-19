#include "arduino.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s.h"
#include "driver/adc.h"
#include "driver/dac.h"


#include "soc/syscon_reg.h"

//for fast adc sampling via timer interupts
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

//https://github.com/espressif/esp-idf/blob/master/examples/peripherals/i2s_adc_dac/main/app_main.c

//https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Timer/RepeatTimer/RepeatTimer.ino

//i2s number
#define I2S_NUM                         (i2s_port_t)(0)

#define BUFFER_LEN                      6000

#define CHANNELS                        2

//https://www.esp32.com/viewtopic.php?t=5574



//uint16_t* i2s_write_buff = (uint16_t*) calloc(BUFFER_LEN * CHANNELS, sizeof(uint16_t));

#define NUMBER_OF_BUFFERS   3

portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED; 
TaskHandle_t complexHandlerTask;
hw_timer_t * adcTimer = NULL; // our timer

uint16_t abuf[NUMBER_OF_BUFFERS][BUFFER_LEN];
uint16_t abufPos = 0;

bool SampleBufferReady = false;

uint8_t SampleBufferToWrite= 0;
uint8_t SampleBufferToProcess= 1;
uint8_t SampleBufferToRead = 2;


void complexHandler(void *param) 
{
  while (true)
  {
    // Sleep until the ISR gives us something to do, or for 1 second
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));  

    if (SampleBufferReady==true) 
    {
      SampleBufferReady=false;

      //lets hope we can process this before the next buffer load is ready!
          
      for(int i = 0; i < BUFFER_LEN; i++)
      { 

          //this is our simple DSP process, scale the samples to the output, 12 bit adc, 8 bit dac

          //eventually this will grow to a proper filter and stuff :)
        
          //convert from 12 bit adc reading to 8 bit for dac output
          float sample = float(abuf[SampleBufferToProcess][i]) / 4095.0f;

          uint16_t uintSample = uint16_t(sample * 255.0f); 

          //store the sample back, and move on
          abuf[SampleBufferToProcess][i] = uintSample;
          
      }

      if (SampleBufferReady!= false)
      {
          //eeeek we ran out of time, may need to do less processing, or slow down the sample rate!
      }

    }
  }
}
//void complexHandler(void *param) 
//{
//  while (true)
//  {
//    // Sleep until the ISR gives us something to do, or for 1 second
//    uint32_t tcount = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));  
//
//    if (SampleBufferReady==true) 
//    {
//      SampleBufferReady=false;
//      
//      size_t bytes_written=0;
//      
//      uint16_t j=0;
//    
//      for(int i = 0; i < BUFFER_LEN; i++)
//      { 
//          //convert from 12 bit adc reading to 8 bit for dac output
//          float sample = float(abuf[SampleBufferToRead][i]) / 4095.0f;
//
//          uint16_t uintSample = uint16_t(sample * 255.0f) << 8; // the DAC reads 8 bits of MSB only, so we take the 0-256 value and shove it up top
//
//          //turn this mono signal into stereo
//          //send to right and left channels
//          i2s_write_buff[j++] = uint16_t(uintSample);//endian-ness is msb written to lowest memory location first.
//          i2s_write_buff[j++] = uint16_t(uintSample);
//      }
//  
//      //i2s_write(I2S_NUM, i2s_write_buff, BUFFER_LEN * CHANNELS, &bytes_written, portMAX_DELAY);
//    }
//  }
//}



#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
void feedTheDog(){
  // feed dog 0
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG0.wdt_feed=1;                       // feed dog
  TIMERG0.wdt_wprotect=0;                   // write protect
  // feed dog 1
  TIMERG1.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG1.wdt_feed=1;                       // feed dog
  TIMERG1.wdt_wprotect=0;                   // write protect
}

uint16_t IRAM_ATTR local_adc1_read(int channel)
{

    uint16_t adc_value;
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
    while (SENS.sar_slave_addr1.meas_status != 0)
      feedTheDog();
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0)
      feedTheDog();
    adc_value = SENS.sar_meas_start1.meas1_data_sar;
 
    return adc_value;
}



void IRAM_ATTR onTimer() {
  
  portENTER_CRITICAL_ISR(&timerMux);

  
  //store sampled value to the write buffer  
  abuf[SampleBufferToWrite][abufPos] = uint16_t(local_adc1_read(ADC1_CHANNEL_0)); //A4 on feather huzzah 32

  //play sampled value from read buffer
  dac_output_voltage(DAC_CHANNEL_1, abuf[SampleBufferToRead][abufPos++]);
      
  if (abufPos >= BUFFER_LEN)
  { 
    abufPos = 0;

    //swap over the buffers
    uint8_t l_currentBuffer =  SampleBufferToWrite;
    
    SampleBufferToWrite = SampleBufferToRead;
    SampleBufferToRead = SampleBufferToProcess;
    SampleBufferToProcess = l_currentBuffer;

    // Notify adcTask that the buffer is full.
    SampleBufferReady = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(complexHandlerTask, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();

    }
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}


void setupInterupts()
{
  //Serial.println("setupInterupts...");
  //run the dsp on the other processor.
  //xTaskCreatePinnedToCore(complexHandler, "Handler Task", 8192, NULL, 1, &complexHandlerTask, 0);

  xTaskCreate(complexHandler, "Handler Task", 1000/*8192*/, NULL, 1, &complexHandlerTask);
  adcTimer = timerBegin(3, 80, true); // 80 MHz timer clock / 80 = 1 MHz hardware clock for easy figuring, means counter is microseconds
  timerAttachInterrupt(adcTimer, &onTimer, true); // Attaches the handler function to the timer 
  timerAlarmWrite(adcTimer, 200, true); // Interrupts when counter == 50 microseconds, frequency is 1 / (50 * 10^-6) , i.e. 20,000 times a second (20khz)
  timerAlarmEnable(adcTimer);
}


/**
 * @brief I2S ADC/DAC mode init.
 */
void i2s_init(void)
{
//     i2s_port_t i2s_num = I2S_NUM;
//    
//
//    i2s_config_t i2s_config;
//    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN);
//    i2s_config.sample_rate = 20000;//BUFFER_LEN * 16;//11025;//48000; //11025; // 96000;//44100;           
//                       
//    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
//    i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT; //I2S_CHANNEL_FMT_RIGHT_LEFT;  //I2S_CHANNEL_FMT_ONLY_LEFT
//    i2s_config.use_apll = false;
//    i2s_config.communication_format = (i2s_comm_format_t) ( /*I2S_COMM_FORMAT_PCM |*/ I2S_COMM_FORMAT_I2S_MSB); //I2S_COMM_FORMAT_PCM;
//    i2s_config.intr_alloc_flags = 0;//ESP_INTR_FLAG_LEVEL3; //ESP_INTR_FLAG_LEVEL3; // high interrupt priority //0;
//    i2s_config.dma_buf_count = 64;
//    i2s_config.dma_buf_len = BUFFER_LEN * CHANNELS;
//    //i2s_config.tx_desc_auto_clear = true;
//    
//     //install and start i2s driver
//     i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
//     //init DAC pad, the above line does this anyway
//     i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
//
//
//    i2s_set_sample_rates(i2s_num, 20000);
//
//    SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV); //MADE NO DIFFERENCE
//
//    i2s_zero_dma_buffer(i2s_num);
//
//    //init DAC pad
//    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
     

}


void setup()
{

//  Serial.begin(250000);
//  Serial.println("Begin...");

  dac_output_enable(DAC_CHANNEL_1);

  //this sets up the ADC CHANNEL 0 on UNIT 1
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
  int val = adc1_get_raw(ADC1_CHANNEL_0);
  
  //i2s_init();
  
  setupInterupts();

}


void loop()
{

}
