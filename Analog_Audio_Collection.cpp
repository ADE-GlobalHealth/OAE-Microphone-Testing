/*This code is the initial Audio_Collection written for the MAX4466 Analog microphone
There is also a resistor network and capacitor circuit which you can search up
alongside the MAX4466. This code can also be used for any other analog microphones in theory
Pin Configuration: 
GND -> GND
VCC -> 3.3v
OUT -> pin 16 (left input CODEC) */
#include "daisy_seed.h"
#include "daisy_core.h"

using namespace daisy;
DaisySeed hw;

const int sample_size = 65536;
static float buffer[sample_size];
static volatile int sample_count = 0;
static volatile bool done = false;

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
  if(done)
    return;

  for(size_t i = 0; i < size; i++) {
    if(sample_count < sample_size) {
      buffer[sample_count++] = in[0][i];

      if(sample_count >= sample_size) {
        done = true;
        break;
      }
    }
  }
}

int main() {
  hw.Init();
  hw.StartLog(false);
  System::Delay(1000);
  hw.PrintLine("START");
  hw.SetAudioBlockSize(256);
  hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
  hw.StartAudio(AudioCallback);

  while(true) {
    if(done) {
      hw.PrintLine("BEGIN");

      for(int i = 0; i < sample_size; i++) {
        hw.PrintLine("%ld", (long)(buffer[i] * 1000000.0f));
      }

      hw.PrintLine("END");
      while(1) {}
    }
  }
}