/*
Audio Playback Test for I2C mics using the Daisy Seed

 SPH0645 Pin Connections (mic input):
 VDD -> 3V3D (pin 38)
 GND -> GND
 BCLK -> Pin 35 (SAI2 SCK  - PA2)
 LRCL -> Pin 34 (SAI2 FS   - PG9)
 DOUT -> Pin 32 (SAI2 SD_B - PA0)
 SEL -> GND (LEFT channel)

 Audio Jack Connections (TRRS breakout, CTIA pinout):
 Tip (L ch audio out) -> Pin 18 (codec AUDIO OUT L)
 Ring1 (R ch audio out) -> Pin 19 (codec AUDIO OUT R)
 Ring2 (Ground/common) -> GND
 Sleeve - mic input, not used for playback tied to gnd
*/

#include "daisy_seed.h"
#include "per/sai.h"

using namespace daisy;
DaisySeed hw;

// DC blocking filter
static float prev_x = 0.0f;
static float prev_y = 0.0f;
const float R = 0.9999f;

inline float RemoveDC(float x) {
  float y = x - prev_x + R * prev_y;
  prev_x = x;
  prev_y = y;
  return y;
}

static const size_t BLOCK_SIZE = 256;

// DMA buffers live in non-cached D2 SRAM (.sram1_bss).
static int32_t DMA_BUFFER_MEM_SECTION dma_rx[BLOCK_SIZE * 2]; // stereo interleaved: L, R, L, R...
static int32_t DMA_BUFFER_MEM_SECTION dma_tx[BLOCK_SIZE * 2]; // dummy TX buffer (required by API)

// Ring buffer bridging the SAI2 (mic) and SAI1 (codec) clock domains.
//two independently-clocked peripherals.
const unsigned RING_SIZE = 2048;
static int16_t ring[RING_SIZE];
static volatile unsigned write_idx = RING_SIZE / 2; // start ahead for latency headroom
static volatile unsigned read_idx  = 0;

// void(int32_t* rx, int32_t* tx, size_t size)
void SAI2_Callback(int32_t* rx, int32_t* tx, size_t size) {
  for(size_t i = 0; i < size / 2; i++) { // /2 because stereo interleaved
    // SPH0645: 18-bit data in standard I2S 32-bit frame (SAI_32BIT).
    // Bits [31:14] = 18-bit value (MSB first). Arithmetic >> 14 sign-extends.
    // Normalize by 2^17 = 131072 (18-bit signed full-scale).
    int32_t raw = rx[i * 2]; // left channel (SEL = GND)
    float sample = (float)(raw >> 14) / 131072.0f;
    sample = RemoveDC(sample);

    float scaled = sample * 32767.0f;
    if(scaled > 32767.0f) scaled = 32767.0f;
    if(scaled < -32768.0f) scaled = -32768.0f;

    ring[write_idx % RING_SIZE] = (int16_t)scaled;
    write_idx++;
  }
}

void PlaybackCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
  for(size_t i = 0; i < size; i++) {
    float sample = (float)ring[read_idx % RING_SIZE] / 32768.0f;
    read_idx++;
    out[0][i] = sample;
    out[1][i] = sample;
  }
}

int main() {
  hw.Init();
  hw.StartLog(false);
  System::Delay(2000);

  SaiHandle sai2;
  SaiHandle::Config sai_cfg;
  sai_cfg.periph = SaiHandle::Config::Peripheral::SAI_2;
  sai_cfg.sr = SaiHandle::Config::SampleRate::SAI_48KHZ;
  sai_cfg.bit_depth = SaiHandle::Config::BitDepth::SAI_32BIT; // SAI_24BIT uses MSB-justified; SPH0645 needs standard I2S
  // All active pins (PA0=SD_B, PA1=MCLK_B, PA2=SCK_B, PG9=FS_B) are Block_B.
  // Block_B must be MASTER so it drives BCLK and LRCL as outputs.
  // Block_A is a dummy slave (required by libDaisy API; SA pin not connected to anything).
  sai_cfg.a_sync = SaiHandle::Config::Sync::SLAVE;
  sai_cfg.b_sync = SaiHandle::Config::Sync::MASTER;
  sai_cfg.a_dir = SaiHandle::Config::Direction::TRANSMIT;
  sai_cfg.b_dir = SaiHandle::Config::Direction::RECEIVE;

  sai_cfg.pin_config.mclk = {DSY_GPIOA, 1};  // Pin 31
  sai_cfg.pin_config.sb = {DSY_GPIOA, 0};  // Pin 32 - mic DOUT
  sai_cfg.pin_config.sa = {DSY_GPIOD, 11}; // Pin 33 - dummy TX
  sai_cfg.pin_config.fs = {DSY_GPIOG, 9};  // Pin 34
  sai_cfg.pin_config.sck = {DSY_GPIOA, 2};  // Pin 35

  SaiHandle::Result init_result = sai2.Init(sai_cfg);
  if(init_result != SaiHandle::Result::OK) {
    hw.PrintLine("ERROR: SAI init failed");
    while(1) {}
  }

  SaiHandle::Result dma_result = sai2.StartDma(dma_rx, dma_tx, BLOCK_SIZE * 2, SAI2_Callback);
  if(dma_result != SaiHandle::Result::OK) {
    hw.PrintLine("ERROR: StartDma failed");
    while(1) {}
  }

  // Start onboard codec (SAI1); drains the ring buffer through
  // OUT_L/OUT_R (pins 18/19) jack breakout board headphones.
  hw.SetAudioBlockSize(BLOCK_SIZE);
  hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
  hw.StartAudio(PlaybackCallback);

  hw.PrintLine("Live passthrough running...");
  while(1) {}
}
