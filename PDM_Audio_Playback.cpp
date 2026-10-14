/*
 Audio Playback Test for the DMM-3526-3-B PDM MEMS Microphone - Daisy Seed
 (SPI1 clock-master path) Edited Heavily by Claude

 How a PDM mic actually works, since everything below follows from it:
 the mic is a 1-bit sigma-delta modulator. Every clock edge it emits one
 bit, and the audio is encoded in the *local density* of 1s - 50% density
 is silence, higher/lower density is positive/negative pressure. Two
 consequences drive this whole file:
   1. The bitstream must be captured unbroken and replayed at a constant
      rate. A gap, a dropped run of bits, or a repeated run of bits is a
      discontinuity in the density function - i.e. audible garbage - even
      though every individual bit is "valid".
   2. The modulator deliberately pushes its quantization noise up into
      the hundreds of kHz (noise shaping). It must be low-passed hard
      before decimation or that noise folds straight back into the audio
      band, so the decimation filter is not optional dressing.

 --- What was wrong with the previous version (the buzzing) ---
 CHUNK_BYTES = 65535 -> 524280 bits -> 32767 PCM samples, delivered as
 one burst every 0.671s. The ring buffer was 2048 samples (42ms). So on
 each chunk the processing loop dumped 32767 samples into a 2048-slot
 ring and the overflow guard discarded ~30000 of them; then for the
 remaining ~0.65s nothing wrote to the ring at all while the codec
 callback kept advancing read_idx unconditionally, lapping the buffer and
 replaying the same 2048 samples ~14 times. 48000/2048 = 23.4Hz of
 periodic waveform, re-seeded 1.5x/sec. That is the buzz - it would
 happen with a perfectly working mic.

 Three fixes, in order of importance:

 (1) The pipeline is now *paced*, not bursty. Rather than waiting for a
     whole 65535-byte DMA chunk to complete, the main loop drains the
     buffer while the DMA is still filling it, by reading the DMA
     stream's remaining-transfer counter (NDTR). libDaisy's SpiHandle
     hardcodes DMA2_Stream2 for SPI RX (src/per/spi.cpp InitDma()), so
     that counter is at a known address; this is a read-only peek at a
     libDaisy internal, not a reimplementation of it. Large chunks are
     kept (rare clock gaps, per ST's guidance that PDM mics want a
     continuous clock) while latency drops from ~0.7s to ~30ms.

 (2) The CIC integrators are uint32_t, not int32_t. A CIC *relies* on its
     integrators wrapping modulo 2^32 - the comb stage cancels the wrap
     exactly, which is the whole trick that lets it use fixed-point
     accumulators. Signed overflow is undefined behaviour in C++, and
     with any DC bias in the bitstream (the normal case) the higher-order
     integrators overflow within seconds. Unsigned wraparound is defined
     and is what the algorithm actually requires.

 (3) The 781250/16 = 48828.125Hz CIC output is resampled to the codec's
     48kHz by a fractional-index linear interpolator instead of being
     dropped-on-overflow. Both clocks are crystal-derived and fixed
     (SPI1 <- PLL2Q = 100MHz, SAI1 <- PLL3P), so the ratio is a constant;
     a very slow proportional servo on the buffer fill absorbs the small
     residual (the SAI's real rate is ~48014Hz, not exactly 48000).

 Also: all PrintLine/SetLed diagnostics moved out of the per-byte loop.
 Blocking USB I/O inside the DSP path was costing far more time than the
 DSP itself.

 DMM-3526-3-B Pin Connections (unchanged - no rewiring needed):
 VDD    -> 3V3D (pin 38)
 GND    -> GND
 CLK    -> Pin 35 (SPI1 SCK  - PA5)  [Daisy is SPI master; generates this]
 Output -> Pin 32 (SPI1 MISO - PA6)
 L/R    -> GND

 Audio Jack Connections (TRRS breakout, CTIA pinout):
 Tip   (L ch out) -> Pin 18 (codec AUDIO OUT L)
 Ring1 (R ch out) -> Pin 19 (codec AUDIO OUT R)
 Ring2 (ground)   -> GND
 Sleeve           -> GND (mic input unused for playback)

 --- Diagnostics ---
 Once a second this prints the ones-density of the raw bitstream. That is
 the single most useful number here: a live PDM stream sits near 50.00%
 and wanders by a few percent with sound. 0.00% or 100.00% means the mic
 is not modulating at all (dead, wrong pin, or asleep because the clock
 never started). ~50% but stone-dead audio would instead point at the
 decimator. Check this before debugging anything downstream.

 --- Known remaining limitation of this SPI approach ---
 libDaisy's SpiHandle DMA is DMA_NORMAL, not circular, so the clock still
 stops briefly (~10-20us) at each chunk re-arm - about 1.5x/second here.
 That is one lost output sample per re-arm (a faint tick), and if this
 particular mic drops into standby on any clock gap it will be worse than
 that. PDM_SAI_Playback.cpp avoids the problem entirely by clocking the
 mic from SAI2 (circular DMA, genuinely gapless, and 3.072MHz = the mic's
 higher-SNR Standard Mode) - prefer that file if you are willing to move
 two wires.
*/

#include "daisy_seed.h"
#include "per/spi.h"

using namespace daisy;
DaisySeed hw;
SpiHandle spi1;

// ---------------------------------------------------------------------
// Rates
// ---------------------------------------------------------------------
// SPI1 kernel clock = PLL2Q = 100MHz (libDaisy src/sys/system.cpp).
// BaudPrescaler::PS_128 -> 781.25kHz, inside the mic's Lower Power Mode
// range (380-980kHz). Standard Mode's 3.072MHz is not reachable from a
// 100MHz power-of-2 divider without exceeding the mic's 3.1MHz max.
static const float PDM_CLOCK_HZ = 781250.0f;

// Decimation. OSR is only 781250/48828 = 16 here, which is low for PDM -
// the CIC's alias bands sit close to the audio band, so its order does
// real work. Order 4 (vs the previous 3) buys ~6dB more alias rejection
// at the cost of ~2.5dB extra droop at 20kHz. Passband droop over the
// DPOAE bins of interest (<= 6kHz) is under 1dB, which is what matters
// for this project.
static const int      CIC_ORDER = 4;
static const uint32_t CIC_DECIM = 16;
static const float    CIC_GAIN  = 65536.0f; // CIC_DECIM^CIC_ORDER = 16^4

static const float PCM_RATE_HZ  = PDM_CLOCK_HZ / (float)CIC_DECIM; // 48828.125
static const float CODEC_RATE_HZ = 48000.0f;
// Read pointer advance per codec sample. > 1 because the CIC produces
// samples faster than the codec consumes them.
static const float NOMINAL_STEP = PCM_RATE_HZ / CODEC_RATE_HZ; // ~1.017253

// ---------------------------------------------------------------------
// CIC decimator (order 4, R = 16)
// ---------------------------------------------------------------------
// uint32_t, deliberately. See fix (2) in the header comment - the
// integrators are *supposed* to wrap, and unsigned wraparound is the only
// way to say that without invoking undefined behaviour.
static uint32_t cic_integ[CIC_ORDER];
static uint32_t cic_comb[CIC_ORDER];
static uint32_t cic_count;

static inline bool CicPushBit(uint32_t bit, float* out_sample)
{
  // Map the 1-bit stream to +-1. A 0 bit becomes 0xFFFFFFFF, which is -1
  // in two's complement - so the integrator chain below is plain modular
  // arithmetic with no branching.
  uint32_t v = bit ? 1u : 0xFFFFFFFFu;

  for(int i = 0; i < CIC_ORDER; i++)
  {
    cic_integ[i] += v;
    v = cic_integ[i];
  }

  if(++cic_count < CIC_DECIM)
    return false;
  cic_count = 0;

  uint32_t c = v;
  for(int i = 0; i < CIC_ORDER; i++)
  {
    uint32_t d  = c - cic_comb[i];
    cic_comb[i] = c;
    c           = d;
  }

  // The comb stage has undone the integrators' wrapping; the true result
  // fits in 18 bits (CIC_ORDER*log2(CIC_DECIM) + 1 = 17, plus sign), so
  // reinterpreting as signed is exact.
  *out_sample = (float)(int32_t)c * (1.0f / CIC_GAIN);
  return true;
}

// ---------------------------------------------------------------------
// DC blocking filter
// ---------------------------------------------------------------------
// A PDM mic's output has a large, arbitrary DC offset (the modulator's
// idle density is never exactly 50%), so this is load-bearing, not
// cosmetic. Cutoff ~= (1-R)*fs/2pi ~= 0.8Hz at 48.8kHz.
static float       dc_prev_x = 0.0f;
static float       dc_prev_y = 0.0f;
static const float DC_R      = 0.9999f;

static inline float RemoveDC(float x)
{
  float y   = x - dc_prev_x + DC_R * dc_prev_y;
  dc_prev_x = x;
  dc_prev_y = y;
  return y;
}

// ---------------------------------------------------------------------
// PCM ring buffer (CIC output -> codec)
// ---------------------------------------------------------------------
// Power of two so the wrap is a mask. Sized for ~170ms; the servo holds
// the fill near RING_TARGET, so this is jitter headroom, not latency.
static const uint32_t RING_SIZE   = 8192;
static const uint32_t RING_MASK   = RING_SIZE - 1;
static const uint32_t RING_TARGET = 1536; // ~32ms of latency

static float             ring[RING_SIZE];
static volatile uint32_t ring_w = 0; // monotonic write count
static volatile uint32_t ring_r = 0; // monotonic read count (integer part)
static float             ring_r_frac = 0.0f;

// Diagnostics (read from the main loop only; never printed from the DSP path)
static volatile uint32_t stat_ones      = 0;
static volatile uint32_t stat_bits      = 0;
static volatile uint32_t stat_overflow  = 0;
static volatile uint32_t stat_underflow = 0;

// Output samples to discard while the CIC and the DC blocker settle, and
// while the mic itself stabilises after its clock first appears. The DC
// blocker's time constant is ~0.2s, so half a second is ~2.5 tau.
static uint32_t settle_remaining = (uint32_t)(PCM_RATE_HZ * 0.5f);

// ---------------------------------------------------------------------
// Raw PDM capture (SPI1 + DMA)
// ---------------------------------------------------------------------
// One DMA transfer is capped at 65535 bytes (HAL_SPI_Receive_DMA's Size
// parameter is uint16_t). Kept at the maximum so the clock gap at each
// re-arm happens as rarely as possible. Lowering this reduces worst-case
// latency but multiplies how often the mic's clock stops.
static const size_t CHUNK_BYTES = 65535;

static uint8_t DMA_BUFFER_MEM_SECTION pdm_rx[2][CHUNK_BYTES];
static volatile int  dma_active  = 0;
static volatile bool dma_done[2] = {false, false};

void SpiRxDone(void* context, SpiHandle::Result result)
{
  int finished       = dma_active;
  dma_done[finished] = true;
  dma_active         = finished ^ 1;
  spi1.DmaReceive(pdm_rx[dma_active], CHUNK_BYTES, nullptr, SpiRxDone, nullptr);
}

// How many bytes of the in-flight DMA transfer have definitely landed in
// memory. NDTR counts down from CHUNK_BYTES. The stream runs with its
// FIFO enabled at a full threshold, so NDTR can lead the actual memory
// writes by up to the FIFO depth - stay well behind it.
static const uint32_t DMA_FIFO_MARGIN = 64;

static inline size_t DmaBytesSettled()
{
  uint32_t ndtr = DMA2_Stream2->NDTR;
  if(ndtr > CHUNK_BYTES)
    return 0; // between transfers; nothing safe to read
  uint32_t filled = (uint32_t)CHUNK_BYTES - ndtr;
  return (filled > DMA_FIFO_MARGIN) ? (size_t)(filled - DMA_FIFO_MARGIN) : 0;
}

// ---------------------------------------------------------------------
// Bitstream -> PCM
// ---------------------------------------------------------------------
static void ProcessBytes(const uint8_t* data, size_t len)
{
  uint32_t ones = 0;

  for(size_t i = 0; i < len; i++)
  {
    uint8_t byte = data[i];
    ones += (uint32_t)__builtin_popcount(byte);

    // MSB first, matching SPI_FIRSTBIT_MSB.
    for(int b = 7; b >= 0; b--)
    {
      float sample;
      if(!CicPushBit((uint32_t)((byte >> b) & 0x01), &sample))
        continue;

      sample = RemoveDC(sample);

      if(settle_remaining)
      {
        settle_remaining--; // let the filters converge, emit nothing
        continue;
      }

      if(sample > 1.0f)
        sample = 1.0f;
      if(sample < -1.0f)
        sample = -1.0f;

      // Leave 2 slots of headroom: the interpolator in the codec
      // callback reads ring[r] and ring[r+1].
      if((uint32_t)(ring_w - ring_r) < (RING_SIZE - 2))
      {
        ring[ring_w & RING_MASK] = sample;
        ring_w++;
      }
      else
      {
        stat_overflow++;
      }
    }
  }

  stat_ones += ones;
  stat_bits += (uint32_t)len * 8u;
}

// Drains whatever the DMA has produced. Called from main() both while
// priming and in the run loop - never from an interrupt.
static int      proc_buf = 0;
static size_t   proc_pos = 0;

static void PumpPdm()
{
  if(dma_done[proc_buf])
  {
    // This buffer is complete. Finish whatever we had not already taken
    // in-flight, then move to the other one.
    dma_done[proc_buf] = false;
    if(proc_pos < CHUNK_BYTES)
      ProcessBytes(&pdm_rx[proc_buf][proc_pos], CHUNK_BYTES - proc_pos);
    proc_buf ^= 1;
    proc_pos = 0;
  }
  else if(proc_buf == dma_active)
  {
    // Still filling - take what has settled so far.
    size_t settled = DmaBytesSettled();
    if(settled > proc_pos)
    {
      ProcessBytes(&pdm_rx[proc_buf][proc_pos], settled - proc_pos);
      proc_pos = settled;
    }
  }
}

// ---------------------------------------------------------------------
// Codec output (SAI1) - resamples 48828.125Hz -> 48kHz
// ---------------------------------------------------------------------
// Proportional servo on the buffer fill. Authority is deliberately tiny:
// the nominal ratio is already correct to ~0.03%, so this only has to
// track the difference between the SAI's real rate (~48014Hz) and the
// 48000Hz assumed above. At this gain a full-scale fill error moves the
// pitch by well under a cent.
static const float SERVO_GAIN     = 1.0e-5f;
static const float SERVO_MAX_DEV  = 0.02f; // clamp to +-2%

void PlaybackCallback(AudioHandle::InputBuffer  in,
                      AudioHandle::OutputBuffer out,
                      size_t                    size)
{
  uint32_t fill = ring_w - ring_r;

  float dev = SERVO_GAIN * ((float)fill - (float)RING_TARGET);
  if(dev > SERVO_MAX_DEV)
    dev = SERVO_MAX_DEV;
  if(dev < -SERVO_MAX_DEV)
    dev = -SERVO_MAX_DEV;
  const float step = NOMINAL_STEP * (1.0f + dev);

  static float last_out = 0.0f;

  for(size_t i = 0; i < size; i++)
  {
    // Underrun: hold the last sample rather than reading past the write
    // pointer. Replaying stale buffer contents is exactly the failure
    // this rewrite exists to remove.
    if((uint32_t)(ring_w - ring_r) < 2)
    {
      stat_underflow++;
      out[0][i] = last_out;
      out[1][i] = last_out;
      continue;
    }

    float a = ring[ring_r & RING_MASK];
    float b = ring[(ring_r + 1) & RING_MASK];
    last_out = a + (b - a) * ring_r_frac;

    ring_r_frac += step;
    uint32_t advance = (uint32_t)ring_r_frac; // 1 or 2 for step ~1.017
    ring_r_frac -= (float)advance;
    ring_r += advance;

    out[0][i] = last_out;
    out[1][i] = last_out;
  }
}

// ---------------------------------------------------------------------
int main()
{
  hw.Init();
  hw.StartLog(false);
  System::Delay(2000);

  SpiHandle::Config spi_cfg;
  spi_cfg.periph    = SpiHandle::Config::Peripheral::SPI_1;
  spi_cfg.mode      = SpiHandle::Config::Mode::MASTER;
  spi_cfg.direction = SpiHandle::Config::Direction::TWO_LINES_RX_ONLY;
  spi_cfg.datasize  = 8;
  // Which SCK edge to latch on: a PDM mic drives its data during one half
  // of the clock period and tri-states the other half (that is how two
  // mics share one data line), so in principle the wrong edge samples a
  // floating pin. At 781kHz the half-period is 640ns against a data delay
  // of well under 100ns, so both edges have generous margin and this is
  // unlikely to be the problem - but if the ones-density printout looks
  // healthy (~50%) and the audio is still hash, flip clock_phase to
  // TWO_EDGE, or move the mic's L/R pin to the other rail. Either change
  // swaps which half-cycle is sampled.
  spi_cfg.clock_polarity = SpiHandle::Config::ClockPolarity::LOW;
  spi_cfg.clock_phase    = SpiHandle::Config::ClockPhase::ONE_EDGE;
  spi_cfg.nss            = SpiHandle::Config::NSS::SOFT;
  spi_cfg.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_128;

  spi_cfg.pin_config.sclk = {DSY_GPIOA, 5}; // Pin 29 - mic CLK
  spi_cfg.pin_config.miso = {DSY_GPIOA, 6}; // Pin 26 - mic Output
  spi_cfg.pin_config.mosi = {DSY_GPIOX, 0}; // unused (RX only)
  spi_cfg.pin_config.nss  = {DSY_GPIOX, 0}; // unused (NSS = SOFT)

  if(spi1.Init(spi_cfg) != SpiHandle::Result::OK)
  {
    hw.PrintLine("ERROR: SPI init failed");
    while(1) {}
  }

  spi1.DmaReceive(pdm_rx[dma_active], CHUNK_BYTES, nullptr, SpiRxDone, nullptr);

  // The mic needs the clock to be running before it leaves its idle
  // state, and needs time to settle once it has. Pump (and discard, via
  // settle_remaining) during that window.
  uint32_t warmup_start = System::GetNow();
  while(System::GetNow() - warmup_start < 250)
    PumpPdm();

  // Prime the ring so the codec never starts into an underrun.
  uint32_t prime_start = System::GetNow();
  while(ring_w < RING_TARGET && System::GetNow() - prime_start < 3000)
    PumpPdm();

  if(ring_w < RING_TARGET)
    hw.PrintLine("WARNING: ring under-primed (%u) - is the mic clocked?",
                 (unsigned)ring_w);

  hw.SetAudioBlockSize(256);
  hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
  hw.StartAudio(PlaybackCallback);

  hw.PrintLine("PDM passthrough running (SPI1 @ %uHz, CIC order %d /%u)",
               (unsigned)PDM_CLOCK_HZ,
               CIC_ORDER,
               (unsigned)CIC_DECIM);

  uint32_t last_report = System::GetNow();
  bool     led_state   = false;

  while(1)
  {
    PumpPdm();

    uint32_t now = System::GetNow();
    if(now - last_report >= 1000)
    {
      last_report = now;

      uint32_t ones = stat_ones;
      uint32_t bits = stat_bits;
      stat_ones     = 0;
      stat_bits     = 0;

      // Hundredths of a percent, integer maths - no float formatting.
      uint32_t density = bits ? (uint32_t)(((uint64_t)ones * 10000u) / bits) : 0;

      hw.PrintLine("ones=%u.%02u%%  fill=%u  over=%u  under=%u",
                   (unsigned)(density / 100),
                   (unsigned)(density % 100),
                   (unsigned)(ring_w - ring_r),
                   (unsigned)stat_overflow,
                   (unsigned)stat_underflow);

      stat_overflow  = 0;
      stat_underflow = 0;

      led_state = !led_state;
      hw.SetLed(led_state);
    }
  }
}
