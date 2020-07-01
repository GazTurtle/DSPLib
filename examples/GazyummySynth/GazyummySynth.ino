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
#define POT_PIN A5
float pot = 0.f;

int note = 0;
int arpCnt = 0;

void audioTask(void *);


void setup() {

  Serial.begin(115200);
  Serial.print("\n\nSetup ");

  // setup audio codec
  i2sCodec.setupBuiltin(fs, channelCount);

  // setup audio lib
  dsp.begin(fs);

  synth.begin(fs);
  synth.note(48);
  synth.setWaveform(SAW);//SINE);//SAW);
  synth.setGlide(0);
  synth.setAttack(30);
  synth.setSustain(0.6);

  // setup some filter nodes
  lp.begin(fs, channelCount);
  lp.setupFilter(FilterNode::LPF, 5000, 1.7);

  hp.begin(fs, channelCount);
  hp.setupFilter(FilterNode::HPF, 30, 1.0);

  dly.begin(fs, channelCount);
  dly.setDelayMs(dly.maxDelayMs());

  //sat.begin(fs, channelCount);
  //sat.setDrive(0.2f);

  // add nodes to audio processing tree
  // Synth => hp => lp => I2S out
//  dsp.addNode(&hp);
  dsp.addNode(&lp);
  dsp.addNode(&dly);
  //dsp.addNode(&sat);
  
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


// control stuff here
void loop()
{

  // sample the potentiometer multiple times, adc is too noisy
  int readCnt = 20;
  float p = analogRead(POT_PIN);
  for (int i = 0; i < readCnt; i++) {
    p += analogRead(POT_PIN);
  }
  // low pass filtered and scaled between 0-1
  pot = pot * 0.99 + (float)p / 4096 / (1 + readCnt) * 0.01;

  //only interested in the positive range.

  

  delay(1);

  //  uncomment to change low pass filter frequency with a pot
    lp.updateFilter(((pot -0.5)*2)* 10000.f);


  // playin some (midi) notes
  int len = 200;
  arpCnt++;
  switch (note) {
    case 0: {
        if (arpCnt > len) {
          arpCnt = 0;
          synth.note(40);
          note++;
        }
      }
    case 1: {
        if (arpCnt > len) {
          arpCnt = 0;
          synth.note(47);
          note++;
        }
      }
    case 2: {
        if (arpCnt > len) {
          arpCnt = 0;
          synth.note(43);
          note++;
        }
      }
    case 3: {
        if (arpCnt > len) {
          arpCnt = 0;
          synth.note(47);
          note = 0;
        }
      }
  }


}
