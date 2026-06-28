// =====================================================================
//  Harmonic Tree — Step 2: Wavetable Synthesis Engine
//  ESP32-S3 DevKitC-1 N16R8
//
//  Goal of this step: prove the wavetable oscillator engine renders
//  multiple voices cleanly through I2S, with the audio render task
//  pinned to Core 1 and Core 0 left free for control/display later.
//
//  Validated hardware approaches reused here:
//    - Raw legacy ESP-IDF I2S (driver/i2s.h), 44100Hz, 16-bit stereo
//    - I2S_COMM_FORMAT_STAND_I2S, 8 DMA buffers of 256
//    - DAC pins: DIN=11, BCK=12, LCK=13  (SCK tied to GND)
//
//  Test you should hear: a sustained harmonic stack (root 110Hz plus
//  integer harmonics at falling amplitude). It should be steady and
//  glitch-free. Nothing is interactive yet — that arrives in later steps.
// =====================================================================

#include "driver/i2s.h"
#include <math.h>

// ----------------------- Audio configuration ------------------------
static const uint32_t SAMPLE_RATE = 44100;
static const int      I2S_NUM_PORT = I2S_NUM_0;
static const int      DMA_BUF_COUNT = 8;
static const int      DMA_BUF_LEN   = 256;   // frames per DMA buffer
static const int      BLOCK_FRAMES  = 256;   // frames we render per pass

// DAC pins (your tested wiring)
static const int PIN_BCK = 12;   // bit clock
static const int PIN_LCK = 13;   // word/LR clock
static const int PIN_DIN = 11;   // data in

// ----------------------- Wavetable engine ---------------------------
// 1024-sample tables. Phase is a 32-bit accumulator; the top
// TABLE_BITS bits index the table, the rest is the interpolation
// fraction. This is the core trick that keeps pitch clean and cheap.
static const int      TABLE_BITS = 10;                 // 2^10 = 1024
static const uint32_t TABLE_SIZE = 1u << TABLE_BITS;   // 1024
static const uint32_t TABLE_MASK = TABLE_SIZE - 1;
static const int      FRAC_BITS  = 32 - TABLE_BITS;    // 22 fractional bits

// Waveform IDs — starts at 4, grows toward the spec's 12 later.
enum Waveform {
  WAVE_SINE = 0,
  WAVE_TRIANGLE,
  WAVE_SAW,
  WAVE_SQUARE,
  WAVE_COUNT
};

// Tables are full int16 range. One extra guard sample at the end holds
// a copy of table[0] so linear interpolation never wraps mid-read.
static int16_t gWaveTables[WAVE_COUNT][TABLE_SIZE + 1];

static void buildWaveTables() {
  for (uint32_t i = 0; i < TABLE_SIZE; i++) {
    float t = (float)i / (float)TABLE_SIZE;   // 0..1 phase

    // Sine
    gWaveTables[WAVE_SINE][i] = (int16_t)lroundf(sinf(2.0f * (float)M_PI * t) * 32767.0f);

    // Triangle: rises 0->1 in first half, falls 1->0 in second
    float tri = (t < 0.5f) ? (4.0f * t - 1.0f) : (3.0f - 4.0f * t);
    gWaveTables[WAVE_TRIANGLE][i] = (int16_t)lroundf(tri * 32767.0f);

    // Sawtooth: -1 -> +1 ramp
    gWaveTables[WAVE_SAW][i] = (int16_t)lroundf((2.0f * t - 1.0f) * 32767.0f);

    // Square (naive; band-limiting comes later if aliasing bites)
    gWaveTables[WAVE_SQUARE][i] = (t < 0.5f) ? 32767 : -32768;
  }
  // Guard sample = first sample, so interpolation at the wrap is smooth.
  for (int w = 0; w < WAVE_COUNT; w++) {
    gWaveTables[w][TABLE_SIZE] = gWaveTables[w][0];
  }
}

// ----------------------------- Voices -------------------------------
// A voice is just a phase accumulator + increment + amplitude + table.
// Q15 amplitude (0..32767) keeps the mix in fixed point.
struct Voice {
  uint32_t phase;
  uint32_t phaseInc;
  int32_t  amplitude;   // Q15, 0..32767
  uint8_t  waveform;
  bool     active;
};

static const int NUM_VOICES = 8;
static Voice gVoices[NUM_VOICES];

// Convert a frequency in Hz to a phase increment for the accumulator.
static inline uint32_t freqToInc(float hz) {
  return (uint32_t)((hz * 4294967296.0) / (double)SAMPLE_RATE);
}

static void voiceSet(int i, float hz, int32_t ampQ15, uint8_t wave) {
  gVoices[i].phaseInc  = freqToInc(hz);
  gVoices[i].amplitude = ampQ15;
  gVoices[i].waveform  = wave;
  gVoices[i].active    = (ampQ15 > 0);
  // leave phase as-is so we don't click when retuning
}

// Render one voice's contribution, with linear interpolation, summed
// into the accumulator buffer (int32 to avoid overflow across voices).
static inline int16_t voiceNextSample(Voice& v) {
  uint32_t idx  = v.phase >> FRAC_BITS;            // table index
  uint32_t frac = v.phase & ((1u << FRAC_BITS)-1); // interpolation fraction
  const int16_t* tbl = gWaveTables[v.waveform];

  int32_t s0 = tbl[idx];
  int32_t s1 = tbl[idx + 1];                        // guard sample covers idx==MASK
  // linear interp: s0 + (s1-s0)*frac
  int32_t sample = s0 + (((s1 - s0) * (int32_t)(frac >> (FRAC_BITS - 15))) >> 15);

  v.phase += v.phaseInc;
  // apply Q15 amplitude
  return (int16_t)((sample * v.amplitude) >> 15);
}

// ----------------------- Mix one audio block ------------------------
// Fills a stereo int16 buffer (L,R interleaved) with BLOCK_FRAMES frames.
static void renderBlock(int16_t* out) {
  for (int n = 0; n < BLOCK_FRAMES; n++) {
    int32_t mix = 0;
    for (int v = 0; v < NUM_VOICES; v++) {
      if (gVoices[v].active) {
        mix += voiceNextSample(gVoices[v]);
      }
    }
    // Headroom: this stack of 8 voices is pre-scaled by amplitude, but
    // clip hard as a safety net so we never wrap to garbage.
    if (mix >  32767) mix =  32767;
    if (mix < -32768) mix = -32768;

    int16_t s = (int16_t)mix;
    out[2*n]   = s;   // left
    out[2*n+1] = s;   // right (mono for now)
  }
}

// --------------------------- I2S setup ------------------------------
static void i2sSetup() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pins = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = PIN_BCK,
    .ws_io_num  = PIN_LCK,
    .data_out_num = PIN_DIN,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install((i2s_port_t)I2S_NUM_PORT, &cfg, 0, NULL);
  i2s_set_pin((i2s_port_t)I2S_NUM_PORT, &pins);
  i2s_zero_dma_buffer((i2s_port_t)I2S_NUM_PORT);
}

// --------------------- Core 1 audio render task ---------------------
static int16_t gBlockBuf[BLOCK_FRAMES * 2];   // stereo interleaved

static void audioTask(void* param) {
  size_t bytesWritten;
  for (;;) {
    renderBlock(gBlockBuf);
    // Blocking write paces the loop to the DAC clock — this is what
    // keeps timing tight and the buffer from under/overruning.
    i2s_write((i2s_port_t)I2S_NUM_PORT, gBlockBuf,
              sizeof(gBlockBuf), &bytesWritten, portMAX_DELAY);
  }
}

// ------------------------------ Setup -------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Harmonic Tree — Step 2: wavetable engine");

  buildWaveTables();

  // Test patch: a harmonic stack on root 110Hz (A2). Integer harmonics
  // at falling amplitude, with a few different waveforms so you can hear
  // the tables and the multi-voice mix at once.
  const float root = 110.0f;
  voiceSet(0, root * 1.0f, 12000, WAVE_SINE);     // fundamental
  voiceSet(1, root * 2.0f,  7000, WAVE_SINE);     // octave
  voiceSet(2, root * 3.0f,  4500, WAVE_TRIANGLE); // fifth above octave
  voiceSet(3, root * 4.0f,  3000, WAVE_TRIANGLE);
  voiceSet(4, root * 5.0f,  2200, WAVE_SAW);
  voiceSet(5, root * 6.0f,  1600, WAVE_SAW);
  voiceSet(6, root * 7.0f,  1100, WAVE_SQUARE);
  voiceSet(7, root * 8.0f,   900, WAVE_SQUARE);

  i2sSetup();

  // Pin audio render to Core 1; Core 0 stays free for control/display.
  xTaskCreatePinnedToCore(audioTask, "audio", 4096, NULL,
                          configMAX_PRIORITIES - 1, NULL, 1);

  Serial.println("Audio task started on Core 1. You should hear a drone.");
}

void loop() {
  // Core 0 is intentionally idle this step. Heartbeat so you know it's alive.
  static uint32_t last = 0;
  if (millis() - last > 2000) {
    last = millis();
    Serial.println("running...");
  }
  delay(50);
}
