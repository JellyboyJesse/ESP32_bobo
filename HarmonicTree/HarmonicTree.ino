// =====================================================================
//  Harmonic Tree — Step 2: Wavetable Synthesis Engine
//  ESP32-S3 DevKitC-1 N16R8
//
//  Per architecture doc §4.1 and §11 step 2:
//    - 12 wavetables generated at startup (additive synthesis)
//    - 2048-sample float32 tables, linear interpolation
//    - single voice playing the root
//
//  Test for this step: one voice cycles through all 12 waveforms
//  (~2.5s each, name printed over serial). Listen that each table is
//  clean and that the timbre matches the recipe. Multi-voice mixing
//  and the tree itself arrive in step 3.
//
//  Hardware approaches reused (validated):
//    - Raw legacy ESP-IDF I2S (driver/i2s.h), 44100Hz, 16-bit stereo
//    - I2S_COMM_FORMAT_STAND_I2S, 8 DMA buffers of 256
//    - Audio render task pinned to Core 1; Core 0 free for control
//
//  NOTE ON PINS: the doc's section 2 lists DAC on GPIO 25/26/27, which
//  do NOT exist on this board. Real tested wiring (used below):
//    DAC  DIN->11  BCK->12  LCK->13   (SCK tied to GND)
// =====================================================================

#include "driver/i2s.h"
#include <math.h>

// ----------------------- Audio configuration ------------------------
static const uint32_t SAMPLE_RATE   = 44100;
static const int      I2S_NUM_PORT  = I2S_NUM_0;
static const int      DMA_BUF_COUNT = 8;
static const int      DMA_BUF_LEN   = 256;   // frames per DMA buffer
static const int      BLOCK_FRAMES  = 256;   // frames rendered per pass

// DAC pins (tested wiring — see note above)
static const int PIN_BCK = 12;   // bit clock
static const int PIN_LCK = 13;   // word/LR clock
static const int PIN_DIN = 11;   // data in

// ----------------------- Wavetable engine ---------------------------
// Phase is a 32-bit accumulator. The top TABLE_BITS bits index the
// table; the remaining bits are the interpolation fraction. This gives
// full sub-sample resolution independent of table size.
static const int      TABLE_BITS = 11;                 // 2^11 = 2048
static const uint32_t TABLE_SIZE = 1u << TABLE_BITS;   // 2048
static const int      FRAC_BITS  = 32 - TABLE_BITS;    // 21 fractional bits
static const uint32_t FRAC_MASK  = (1u << FRAC_BITS) - 1;
static const float    FRAC_SCALE = 1.0f / (float)(1u << FRAC_BITS);

// 12 waveforms — see architecture doc §4.1.
enum Waveform {
  WAVE_SINE = 0,    // 01 Sine        : 1 only
  WAVE_SINE_PLUS,   // 02 Sine+       : 1, 2 subtle
  WAVE_TRIANGLE,    // 03 Triangle    : odd, 1/n^2
  WAVE_SOFT_SAW,    // 04 Soft Saw    : all, 1/n^2
  WAVE_SQUARE,      // 05 Square      : odd, 1/n
  WAVE_SAWTOOTH,    // 06 Sawtooth    : all, 1/n
  WAVE_PULSE25,     // 07 Pulse 25%   : duty-shaped
  WAVE_SOFTCLIP,    // 08 Soft Clip   : tanh-shaped
  WAVE_FORM_3_2,    // 09 3:2 Form    : harmonics 3 + 2
  WAVE_FORM_7_4,    // 10 7:4 Form    : harmonics 7 + 4
  WAVE_PRIME,       // 11 Prime Series: 2,3,5,7,11,13
  WAVE_INHARMONIC,  // 12 Inharmonic  : x1.0, x2.756, x4.1
  WAVE_COUNT
};

static const char* kWaveNames[WAVE_COUNT] = {
  "01 Sine", "02 Sine+", "03 Triangle", "04 Soft Saw",
  "05 Square", "06 Sawtooth", "07 Pulse 25%", "08 Soft Clip",
  "09 3:2 Form", "10 7:4 Form", "11 Prime Series", "12 Inharmonic"
};

// Tables live in internal SRAM (fast per-sample reads, ~98KB total).
// One guard sample at the end mirrors table[0] so interpolation at the
// wrap point never reads past the array.
static float gWaveTables[WAVE_COUNT][TABLE_SIZE + 1];

// --- additive synthesis helper: add a (possibly inharmonic) partial ---
static void addPartial(float* tbl, float ratio, float amp, float phase01) {
  for (uint32_t i = 0; i < TABLE_SIZE; i++) {
    float t = (float)i / (float)TABLE_SIZE;
    tbl[i] += amp * sinf(2.0f * (float)M_PI * (ratio * t + phase01));
  }
}

static void normalizeTable(float* tbl) {
  float peak = 1e-9f;
  for (uint32_t i = 0; i < TABLE_SIZE; i++) {
    float a = fabsf(tbl[i]);
    if (a > peak) peak = a;
  }
  float g = 0.98f / peak;
  for (uint32_t i = 0; i < TABLE_SIZE; i++) tbl[i] *= g;
  tbl[TABLE_SIZE] = tbl[0];   // guard sample
}

static void buildWaveTables() {
  // Cap harmonics so single-cycle tables stay reasonably tame when
  // played at higher pitches. Multi-table-per-octave is a future option
  // if aliasing on outer branches is audible.
  const int HMAX = 48;

  for (int w = 0; w < WAVE_COUNT; w++)
    for (uint32_t i = 0; i <= TABLE_SIZE; i++)
      gWaveTables[w][i] = 0.0f;

  // 01 Sine
  addPartial(gWaveTables[WAVE_SINE], 1.0f, 1.0f, 0.0f);

  // 02 Sine+ : fundamental + subtle 2nd
  addPartial(gWaveTables[WAVE_SINE_PLUS], 1.0f, 1.0f, 0.0f);
  addPartial(gWaveTables[WAVE_SINE_PLUS], 2.0f, 0.15f, 0.0f);

  // 03 Triangle : odd harmonics, 1/n^2, alternating sign
  for (int n = 1; n <= HMAX; n += 2) {
    float sign = (((n - 1) / 2) & 1) ? -1.0f : 1.0f;
    addPartial(gWaveTables[WAVE_TRIANGLE], (float)n, sign / (float)(n * n), 0.0f);
  }

  // 04 Soft Saw : all harmonics, 1/n^2 (mellow rolloff)
  for (int n = 1; n <= HMAX; n++)
    addPartial(gWaveTables[WAVE_SOFT_SAW], (float)n, 1.0f / (float)(n * n), 0.0f);

  // 05 Square : odd harmonics, 1/n
  for (int n = 1; n <= HMAX; n += 2)
    addPartial(gWaveTables[WAVE_SQUARE], (float)n, 1.0f / (float)n, 0.0f);

  // 06 Sawtooth : all harmonics, 1/n
  for (int n = 1; n <= HMAX; n++)
    addPartial(gWaveTables[WAVE_SAWTOOTH], (float)n, 1.0f / (float)n, 0.0f);

  // 07 Pulse 25% : amplitudes ~ sin(n*pi*duty)/n
  {
    const float duty = 0.25f;
    for (int n = 1; n <= HMAX; n++) {
      float a = sinf((float)n * (float)M_PI * duty) / (float)n;
      addPartial(gWaveTables[WAVE_PULSE25], (float)n, a, 0.0f);
    }
  }

  // 08 Soft Clip : tanh-driven sine (generate then shape)
  {
    const float drive = 3.0f;
    float* t = gWaveTables[WAVE_SOFTCLIP];
    for (uint32_t i = 0; i < TABLE_SIZE; i++) {
      float ph = (float)i / (float)TABLE_SIZE;
      t[i] = tanhf(drive * sinf(2.0f * (float)M_PI * ph)) / tanhf(drive);
    }
  }

  // 09 3:2 Form : harmonics at 3 and 2
  addPartial(gWaveTables[WAVE_FORM_3_2], 3.0f, 1.0f,  0.0f);
  addPartial(gWaveTables[WAVE_FORM_3_2], 2.0f, 0.7f,  0.0f);

  // 10 7:4 Form : harmonics at 7 and 4
  addPartial(gWaveTables[WAVE_FORM_7_4], 7.0f, 1.0f,  0.0f);
  addPartial(gWaveTables[WAVE_FORM_7_4], 4.0f, 0.7f,  0.0f);

  // 11 Prime Series : 2,3,5,7,11,13 with gentle rolloff
  {
    const int primes[] = {2, 3, 5, 7, 11, 13};
    for (int k = 0; k < 6; k++)
      addPartial(gWaveTables[WAVE_PRIME], (float)primes[k],
                 1.0f / (float)primes[k], 0.0f);
  }

  // 12 Inharmonic : non-integer partials (won't perfectly loop — that's
  // the point; gives a metallic/leaf texture). Approximate single cycle.
  addPartial(gWaveTables[WAVE_INHARMONIC], 1.0f,   1.0f,  0.0f);
  addPartial(gWaveTables[WAVE_INHARMONIC], 2.756f, 0.6f,  0.0f);
  addPartial(gWaveTables[WAVE_INHARMONIC], 4.1f,   0.4f,  0.0f);

  for (int w = 0; w < WAVE_COUNT; w++)
    normalizeTable(gWaveTables[w]);
}

// ----------------------------- Voices -------------------------------
struct Voice {
  uint32_t phase;
  uint32_t phaseInc;
  float    amplitude;   // 0..1
  uint8_t  waveform;
  bool     active;
};

static const int NUM_VOICES = 32;   // pool ceiling per §4.2
static Voice gVoices[NUM_VOICES];

static inline uint32_t freqToInc(float hz) {
  return (uint32_t)((double)hz * 4294967296.0 / (double)SAMPLE_RATE);
}

// Linear-interpolated wavetable read for one voice.
// Takes a voice *index* (not Voice&) so the Arduino IDE's auto-generated
// prototypes — emitted above the struct definition — still compile.
static inline float voiceNextSample(int vi) {
  Voice& v = gVoices[vi];
  uint32_t idx  = v.phase >> FRAC_BITS;
  float    frac = (float)(v.phase & FRAC_MASK) * FRAC_SCALE;
  const float* t = gWaveTables[v.waveform];
  float s = t[idx] + (t[idx + 1] - t[idx]) * frac;   // guard sample covers idx==MASK
  v.phase += v.phaseInc;
  return s * v.amplitude;
}

// ----------------------- Mix one audio block ------------------------
static void renderBlock(int16_t* out) {
  for (int n = 0; n < BLOCK_FRAMES; n++) {
    float mix = 0.0f;
    for (int v = 0; v < NUM_VOICES; v++)
      if (gVoices[v].active) mix += voiceNextSample(v);

    // soft clip / safety (full mix bus shaping comes in step 7)
    if (mix >  1.0f) mix =  1.0f;
    if (mix < -1.0f) mix = -1.0f;

    int16_t s = (int16_t)(mix * 32767.0f);
    out[2 * n]     = s;   // left
    out[2 * n + 1] = s;   // right
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
    .mck_io_num   = I2S_PIN_NO_CHANGE,
    .bck_io_num   = PIN_BCK,
    .ws_io_num    = PIN_LCK,
    .data_out_num = PIN_DIN,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install((i2s_port_t)I2S_NUM_PORT, &cfg, 0, NULL);
  i2s_set_pin((i2s_port_t)I2S_NUM_PORT, &pins);
  i2s_zero_dma_buffer((i2s_port_t)I2S_NUM_PORT);
}

// --------------------- Core 1 audio render task ---------------------
static int16_t gBlockBuf[BLOCK_FRAMES * 2];

static void audioTask(void* param) {
  size_t bytesWritten;
  for (;;) {
    renderBlock(gBlockBuf);
    i2s_write((i2s_port_t)I2S_NUM_PORT, gBlockBuf,
              sizeof(gBlockBuf), &bytesWritten, portMAX_DELAY);
  }
}

// ------------------------------ Setup -------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Harmonic Tree - Step 2: 12-table wavetable engine");

  buildWaveTables();
  Serial.println("12 wavetables built.");

  // Single voice on the root (110Hz, A2). Waveform is swapped in loop().
  for (int i = 0; i < NUM_VOICES; i++) gVoices[i].active = false;
  gVoices[0].phase     = 0;
  gVoices[0].phaseInc  = freqToInc(110.0f);
  gVoices[0].amplitude = 0.6f;
  gVoices[0].waveform  = WAVE_SINE;
  gVoices[0].active    = true;

  i2sSetup();
  xTaskCreatePinnedToCore(audioTask, "audio", 4096, NULL,
                          configMAX_PRIORITIES - 1, NULL, 1);

  Serial.println("Audio on Core 1. Cycling through all 12 waveforms...");
}

void loop() {
  // Step through all 12 waveforms so each table can be heard/verified.
  static uint32_t last = 0;
  static int idx = 0;
  if (millis() - last > 2500) {
    last = millis();
    gVoices[0].waveform = idx;   // single writer; audio reads atomically
    Serial.printf("Waveform -> %s\n", kWaveNames[idx]);
    idx = (idx + 1) % WAVE_COUNT;
  }
  delay(20);
}
