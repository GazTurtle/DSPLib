/*
    yummyDSP – Basic example

    Author: Gary Grutzek

*/

#include "Arduino.h"
#include "driver/gpio.h"
#include <YummyDSP.h>
#include <AudioDriver.h>

AudioDriver i2sCodec;
YummyDSP dsp;
WaveSynth synth;
FilterNode lp;
FilterNode hp;
WaveShaperNode sat;
DelayNode dly;


// I2S
const int fs = 48000;
const int channelCount = 2;

// a potentiometer at 
#define POT_PIN1 ADC2_CHANNEL_7 //GPIO27 = ADC2 channel 7 - A10
#define POT_PIN2 ADC1_CHANNEL_5 //GPIO33 = ADC1 channel 5 - A9
#define POT_PIN3 ADC2_CHANNEL_3 //GPIO15 = ADC2 channel 3 - A8
#define POT_PIN4 ADC1_CHANNEL_4 //GPIO32 = ADC2 channel 7 - A7

//a jack input at
#define JACK_PIN1 ADC1_CHANNEL_6 //A2
#define JACK_PIN2 ADC1_CHANNEL_3 //A3
#define JACK_PIN3 ADC1_CHANNEL_0 //A4

float pot1 = 0.f;
float pot2 = 0.f;
float pot3 = 0.f;
float pot4 = 0.f;

float cv1 = 0.f;
float cv2 = 0.f;
float cv3 = 0.f;

int note = 0;
int arpCnt = 0;

void audioTask(void *);


void setup() {

  Serial.begin(115200);
  Serial.print("\n\nSetup ");

  // setup audio codec
  i2sCodec.setupBuiltin(fs, channelCount);

  adc1_config_width(ADC_WIDTH_BIT_12);
  
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  float p = adc1_get_raw(ADC1_CHANNEL_0);
  
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
  p = adc1_get_raw(ADC1_CHANNEL_3);
    
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
  p = adc1_get_raw(ADC1_CHANNEL_4);
    
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
  p = adc1_get_raw(ADC1_CHANNEL_5);
    
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  p = adc1_get_raw(ADC1_CHANNEL_6);

  //adc2_config_width(ADC_WIDTH_BIT_12);

  int p2 =0;
      
  adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_11);
  adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_BIT_12, &p2);
  
  adc2_config_channel_atten(ADC2_CHANNEL_3, ADC_ATTEN_DB_11);
  adc2_get_raw(ADC2_CHANNEL_3,ADC_WIDTH_BIT_12, &p2 );
  

  // setup audio lib
  dsp.begin(fs);

  synth.begin(fs);
  synth.note(48);
  synth.setWaveform(SAW);//SINE);//SAW);
  synth.setGlide(0);
  synth.setAttack(0);
  synth.setSustain(0.0);
  synth.setDecay(75);

  // setup some filter nodes
  lp.begin(fs, channelCount);
  lp.setupFilter(FilterNode::LPF, 5000, 1.7);

  hp.begin(fs, channelCount);
  hp.setupFilter(FilterNode::HPF, 30, 1.0);

  dly.begin(fs, channelCount);
  dly.setDelayMs(150.f);

  sat.begin(fs, channelCount);
  sat.setDrive(0.2f);

  // add nodes to audio processing tree
  // Synth => hp => lp => I2S out
  dsp.addNode(&hp);
  dsp.addNode(&lp);
  dsp.addNode(&dly);
  dsp.addNode(&sat);


  // run audio in dedicated task on cpu core 1
  xTaskCreatePinnedToCore(audioTask, "audioTask", 10000, NULL, 10, NULL, 1);
  // run control task on another cpu  core with lower priority

  Serial.print("\nSetup done ");
}


void audioTask(void *) {

  Serial.print("\nAudio task");

  float sample = 0;

  while (true) {

    for (int i = 0; i < AudioDriver::BufferSize; i++) {

      float sampleMono = synth.getSample();

      for (int ch = 0; ch < channelCount; ch++) {

        // upmix to stereo
        sample = sampleMono;

        sample = dsp.process(sample, ch);

        i2sCodec.writeSample(sample, i, ch);
      }
    }

    i2sCodec.writeBlock();
  }
  vTaskDelete(NULL);
}


//for fast adc sampling via timer interupts
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
uint16_t IRAM_ATTR local_adc1_read(int channel)
{

    uint16_t adc_value;
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
    while (SENS.sar_slave_addr1.meas_status != 0);
//      feedTheDog();
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0);
//      feedTheDog();
    adc_value = SENS.sar_meas_start1.meas1_data_sar;
 
    return adc_value;
}

uint16_t IRAM_ATTR local_adc2_read(int channel)
{
    uint16_t adc_value;
        SENS.sar_meas_start2.sar2_en_pad = (1 << channel); //only one channel is selected.    
        SENS.sar_meas_start2.meas2_start_sar = 0; //start force 0
        SENS.sar_meas_start2.meas2_start_sar = 1; //start force 1
        while (SENS.sar_meas_start2.meas2_done_sar == 0) {}; //read done
        adc_value = SENS.sar_meas_start2.meas2_data_sar;   
    return adc_value; 

}



float readAnalog1ForPin(uint16_t p_pin)
{

  
  
  // sample the potentiometer multiple times, adc is too noisy
  int readCnt = 20;
  
  float p =  float(local_adc1_read(p_pin));

  for (int i = 0; i < readCnt; i++)
  {
    p += float(local_adc1_read(p_pin));
  }
  
  return ((float)p / 4096.0f) / float(1.0f + float(readCnt));
}

float readAnalog2ForPin(uint16_t p_pin)
{

  
  
  // sample the potentiometer multiple times, adc is too noisy
  int readCnt = 20;
  
  float p =  float(local_adc2_read(p_pin));

  for (int i = 0; i < readCnt; i++)
  {
    p += float(local_adc2_read(p_pin));
  }
  
  return ((float)p / 4096.0f) / float(1.0f + float(readCnt));
}

#define TRIGGER_VALUE 0.5f

bool noteTriggered = false;

// control stuff here
void loop()
{


  pot1 = readAnalog2ForPin(POT_PIN1);
  
  pot2 = readAnalog1ForPin(POT_PIN2);
  
  pot3 = readAnalog2ForPin(POT_PIN3);
  
  pot4 = readAnalog1ForPin(POT_PIN4);
  

  cv1 = readAnalog1ForPin(JACK_PIN1);
  cv2 = readAnalog1ForPin(JACK_PIN2);
  cv3 = readAnalog1ForPin(JACK_PIN3);


  //  uncomment to change low pass filter frequency with a pot
  lp.updateFilter((cv1) * 10000.f);

  sat.setDrive(pot2);
  dly.setDelayMs(pot3 * dly.maxDelayMs());

  synth.setDecay(400 * pot4);

  //get note from pot4
  //get trigger from cv3
  if (cv3 > TRIGGER_VALUE)
  {
    if (noteTriggered==false)
    {
      noteTriggered = true;
      synth.note(cv2 * 400);
      
    }
  }
  else
  {
    noteTriggered=false;
  }
  

}
