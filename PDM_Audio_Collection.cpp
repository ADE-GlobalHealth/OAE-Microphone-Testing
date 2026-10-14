/*
 This is AI slop! idk what im doing so I asked claude and it doesn't know what its doing either lol
Audio Collection for DMM-3526-3-B PDM MEMS Microphone - Daisy Seed
 Drives SPI1 as a clock-generating master, RX-only, to capture the RAW
 1-bit PDM bitstream. This firmware does NOT decimate to PCM - it dumps
 the raw bytes unmodified over serial. Decimating/resampling to 48kHz PCM is a PC-side
 Python step (not yet written) - do not point the existing .m scripts
 at this CSV directly until that step exists.

 DMM-3526-3-B Pin Connections: 
 Pin 3 (datasheet pin 4): VDD -> 3V3D (pin 38)
 Pin 5: GND -> GND
 Pin 4 (datasheet pin 3): Clock input CLK -> Pin 29 (SPI1 SCK  - PA5)  [Daisy is SPI master; generates this clock]
 Pin 1: Output -> Pin 26 (SPI1 MISO - PA6)
 Pin 2: Channel Select L/R -> GND (single mic; arbitrary choice, same convention as the SPH0645's SEL->GND)

Warning: Breakout board has pin 3 and 4 flipped from pads

 Clock rate: SPI1's kernel clock is PLL2Q = 100MHz (libDaisy src/sys/system.cpp,
 Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2). BaudPrescaler::PS_128 ->
 100MHz/128 = 781.25kHz, which lands in the mic's "Lower Power Mode" range
 (380-980kHz typ 768kHz, datasheet pg.3) - NOT Standard Mode's 3.072MHz,
 which isn't reachable from a 100MHz/power-of-2 divider without exceeding
 its 3.1MHz max, and would produce a raw-bit buffer too large for a
 capture-duration-matched buffer to fit in SRAM/RAM_D2 in one shot anyway.
 Per the datasheet's own spec table this only costs ~1dB SNR (65 vs 64 dB typ).

 781.25kHz is NOT a clean multiple of 48kHz (781250/48000 = 16.276...), so
 the not-yet-written PC-side step must do a rational resample (e.g. scipy
 resample_poly, up=192/down=3125) rather than a simple decimate-by-16, to
 land on exactly 48000Hz for the existing .m scripts (which hardcode
 Fs = 48000).

 libDaisy's SpiHandle hardcodes DMA_NORMAL, not circular (src/per/spi.cpp
 InitDma()) - there is no continuous RX mode like the SPH0645's SAI2
 capture. HAL_SPI_Receive_DMA's size parameter is also uint16_t (max
 65535), so this capture cannot be one DMA transfer regardless. Both
 constraints mean the capture is chunked: each chunk's end-callback
 re-arms the next one back-to-back. This introduces a few chunk-boundary
 gaps (interrupt/re-arm latency) not present in the SPH0645's single
 continuous SAI2 DMA capture - kept to 3 boundaries total (4 chunks) to
 minimize how often that happens.

 SPI clock polarity/phase (CPOL=LOW, CPHA=ONE_EDGE / SPI Mode 0) is a
 starting guess, not verified against a scope - the datasheet's timing
 diagram (tDD, tDV) specifies edge-relative timing but not explicitly
 which edge is safe to sample on. If the dominant-frequency/SNR sanity
 check (same role as Single_Frequency_FFT.m plays for the SPH0645) comes
 back garbled, try flipping clock_phase first.
*/

#include "daisy_seed.h"
#include "per/spi.h"

using namespace daisy;
DaisySeed hw;
SpiHandle spi1;

// 131072 bytes ~= 1.342s of raw PDM bits at 781.25kHz - close to (within
// ~2% of) the 1.365s / 65536-sample window used for the SPH0645 captures.
// Split into 4 chunks because HAL_SPI_Receive_DMA's size arg is uint16_t
// (max 65535) and DMA_NORMAL requires re-arming between chunks anyway.
static const size_t CHUNK_BYTES = 32768;
static const size_t NUM_CHUNKS  = 4;
static const size_t TOTAL_BYTES = CHUNK_BYTES * NUM_CHUNKS; // 131072

static uint8_t DMA_BUFFER_MEM_SECTION pdm_buf[TOTAL_BYTES];
static volatile size_t chunks_done = 0;
static volatile bool   done        = false;

void SpiRxDone(void* context, SpiHandle::Result result);

void StartNextChunk() {
  if(chunks_done >= NUM_CHUNKS) {
    done = true;
    return;
  }
  spi1.DmaReceive(&pdm_buf[chunks_done * CHUNK_BYTES], CHUNK_BYTES, nullptr, SpiRxDone, nullptr);
}

void SpiRxDone(void* context, SpiHandle::Result result) {
  chunks_done++;
  StartNextChunk();
}

int main() {
  hw.Init();
  hw.StartLog(false);
  System::Delay(2000);

  SpiHandle::Config spi_cfg;
  spi_cfg.periph         = SpiHandle::Config::Peripheral::SPI_1;
  spi_cfg.mode           = SpiHandle::Config::Mode::MASTER;
  spi_cfg.direction      = SpiHandle::Config::Direction::TWO_LINES_RX_ONLY;
  spi_cfg.datasize       = 8;
  spi_cfg.clock_polarity = SpiHandle::Config::ClockPolarity::LOW;
  spi_cfg.clock_phase    = SpiHandle::Config::ClockPhase::ONE_EDGE;
  spi_cfg.nss            = SpiHandle::Config::NSS::SOFT;
  spi_cfg.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_128; // 100MHz/128 = 781.25kHz

  spi_cfg.pin_config.sclk = {DSY_GPIOA, 5}; // Pin 29 - mic CLK (Daisy drives this)
  spi_cfg.pin_config.miso = {DSY_GPIOA, 6}; // Pin 26 - mic Output
  spi_cfg.pin_config.mosi = {DSY_GPIOX, 0}; // unused (RX only)
  spi_cfg.pin_config.nss  = {DSY_GPIOX, 0}; // unused (NSS = SOFT)

  SpiHandle::Result init_result = spi1.Init(spi_cfg);
  if(init_result != SpiHandle::Result::OK) {
    hw.PrintLine("ERROR: SPI init failed");
    while(1) {}
  }

  StartNextChunk();

  hw.PrintLine("SPI OK - waiting for capture...");

  for(int t = 0; t < 5; t++) {
    System::Delay(1000);
    hw.PrintLine("t=%d chunks=%d/%d", t, (int)chunks_done, (int)NUM_CHUNKS);
    if(done) break;
  }

  if(!done) {
    hw.PrintLine("ERROR: capture timed out - chunks_done=%d", (int)chunks_done);
    while(1) {}
  }

  // Raw bytes, unmodified - each printed line is one byte (0-255), reusing
  // Audio_Capture's existing *1e6 encode/decode round-trip exactly (it
  // just divides back to the same integer). A separate, not-yet-written
  // PC-side script must unpack bits and resample to 48kHz before any of
  // the existing .m scripts can read the result.
  hw.PrintLine("BEGIN");
  for(size_t i = 0; i < TOTAL_BYTES; i++) {
    hw.PrintLine("%ld", (long)pdm_buf[i] * 1000000L);
  }
  hw.PrintLine("END");
  while(1) {}
}
