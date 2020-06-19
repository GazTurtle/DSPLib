/*
    yummyDSP – Basic example

    Author: Gary Grutzek

*/

#include "Arduino.h"
#include "driver/gpio.h"
#include <YummyDSP.h>
#include <AudioDriver.h>
#include "esp_adc_cal.h"
#include "driver/adc.h"



AudioDriver i2sCodec;
YummyDSP dsp;
FilterNode lp;
FilterNode hp;
WaveShaperNode sat;
DelayNode dly;

// I2S
const int fs = 22222;//1024 * 4;// 96000;//11200;//96000;// 11200;//48000;
const int channelCount = 2;


//struggling to read using analogRead while we are doing i2s directly from ADC 
//might be better to use the ideas in this article to go direct!

//https://www.toptal.com/embedded/esp32-audio-sampling

//or wait until we are using an external DAC (need more than eight bits anyway, and 16 bits of ADC would be good too)
#define POT_PIN A5
float pot=0.f;


void audioTask(void *);


void setup() {

  //Serial.begin(250000 * 2);
  //Serial.print("\n\nSetup ");

  // setup audio codec
  i2sCodec.setupBuiltin(fs, channelCount);
  //i2sCodec.enable(true);

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

  // run audio in dedicated task on cpu core 1
  xTaskCreatePinnedToCore(audioTask, "audioTask", 10000, NULL, 10, NULL, 1); //task priority  was 10 //configMAX_PRIORITIES-1

  
  // run control task on another cpu  core with lower priority

  //Serial.print("\nSetup done ");
}


bool g_doneIt = false;
uint16_t g_countSampler = 0;

//void audioTask(void) {
void audioTask(void *) {

  //Serial.print("\nAudio task");

  float sample=0;

  while (true) {
    
      uint16_t l_bytesRead=0;

      l_bytesRead = i2sCodec.readBlock();
      
      for (int i = 0; i < (AudioDriver::BufferSize); i++) {
  
        for (int ch = 0; ch < channelCount; ch++) 
        {
  
          sample = i2sCodec.readSample(i, ch);

          //Serial.println(sample);
          sample = dsp.process(sample, ch);
  
          i2sCodec.writeSample(sample, i, ch);
        }
      }
  
      //while (true) - this does not exhibit the problem! Seems to be reading is the issue!
      //i2sCodec.writeBlock(l_bytesRead);
      i2sCodec.writeBlock();

  }
  vTaskDelete(NULL);
}


//useful code example...
//https://github.com/espressif/esp-idf/blob/master/examples/peripherals/i2s_adc_dac/main/app_main.c



// control stuff here
void loop() 
{

//
//  // sample the potentiometer multiple times, adc is too noisy 
//  int readCnt = 50;
//  float p = analogRead(POT_PIN);
//  for (int i=0; i<readCnt; i++)
//  {
//    p += analogRead(POT_PIN);
//  }
//  
//  
//
//  
//  // low pass filtered and scaled between 0-1
//  pot = pot * 0.99 + (float)p/4096/(1+readCnt) * 0.01;
//
// 
//  lp.updateFilter(pot * 10000.f);
////  //sat.setDrive(pot);
////  //dly.setDelayMs(1 * dly.maxDelayMs());

}
