#include "arduino.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "driver/dac.h"


//#include "soc/syscon_reg.h"

//for fast adc sampling via timer interupts
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

#include <YummyDSP.h>

YummyDSP dsp;
FilterNode lp;
FilterNode hp;
WaveShaperNode sat;
DelayNode dly;

#define POT_PIN A5
float pot=0.f;

//https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Timer/RepeatTimer/RepeatTimer.ino


#define BUFFER_LEN                      32 //6000


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
          float sample = float(abuf[SampleBufferToProcess][i]) / 2047.0f;

          sample -= 1; //offset down around actual zero

          //process the sample
          uint16_t ch = 0; //only one channel for now
          sample = dsp.process(sample, ch);

          //now scale to 8bits for output to DAC
          sample += 1; //offset back up for output to DAC
          uint16_t uintSample = uint16_t(sample * 120.0f); 

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
  xTaskCreatePinnedToCore(complexHandler, "Handler Task", 8192, NULL, 1, &complexHandlerTask, 1);

  //xTaskCreate(complexHandler, "Handler Task", 1000/*8192*/, NULL, 1, &complexHandlerTask);
  adcTimer = timerBegin(3, 80, true); // 80 MHz timer clock / 80 = 1 MHz hardware clock for easy figuring, means counter is microseconds
  timerAttachInterrupt(adcTimer, &onTimer, true); // Attaches the handler function to the timer 
  timerAlarmWrite(adcTimer, 50, true); // Interrupts when counter == 50 microseconds, frequency is 1 / (50 * 10^-6) , i.e. 20,000 times a second (20khz)
  timerAlarmEnable(adcTimer);
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

  uint16_t fs = 40000;
  uint16_t channelCount = 1;

  // setup audio lib
  dsp.begin(fs);

  // setup some filter nodes
  lp.begin(fs, channelCount);
  lp.setupFilter(FilterNode::LPF, 2000, 0.7);

  hp.begin(fs, channelCount);
  hp.setupFilter(FilterNode::HPF, 30, 1.0);

  dly.begin(fs, channelCount);
  dly.setDelayMs(dly.maxDelayMs());

  sat.begin(fs, channelCount);
  sat.setDrive(0.2f);

  // add nodes to audio processing tree
  // I2S in => hp => delay => saturation => lp => I2S out
  dsp.addNode(&hp);
  dsp.addNode(&dly);
  //dsp.addNode(&sat);
  dsp.addNode(&lp);

  
  setupInterupts();


}


void loop()
{

  // sample the potentiometer multiple times, adc is too noisy 
  int readCnt = 50;
  float p = analogRead(POT_PIN);
  for (int i=0; i<readCnt; i++)
  {
    p += analogRead(POT_PIN);
  }
  
  

  
  // low pass filtered and scaled between 0-1
  pot = pot * 0.99 + (float)p/4096/(1+readCnt) * 0.01;

 
  //lp.updateFilter(pot * 10000.f);
  //sat.setDrive(pot);
  dly.setDelayMs(pot * dly.maxDelayMs());
  //hp.updateFilter(pot * 10000.f);
  
}
