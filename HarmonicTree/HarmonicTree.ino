// =====================================================================
//  Harmonic Tree v2 — Step 1: Dual-Core + Fast Display
//  ESP32-S3 DevKitC-1 N16R8
//
//  Per architecture doc v2 §3 / §10 step 1 (THE priority):
//    - audio render in a FreeRTOS task pinned to CORE 1, high priority
//      (only i2s_write blocks, and only on that core)
//    - SH1106 display + control on CORE 0, free to run 30+ fps
//
//  This folds in the already-validated wavetable engine (12 tables) and
//  static tree/voice pool so there's a real audio load while we prove the
//  display runs fast and the audio stays clean side by side.
//
//  Test for this step:
//    - OLED lights up (finally!) showing FPS + a sweeping bar + voice count
//    - FPS should read ~30+ (likely much higher) and the bar should sweep
//      smoothly while the tree drone plays with no clicks/glitches
//    - serial also prints FPS once a second
//  If the display is smooth AND audio is clean simultaneously, dual-core
//  is proven and we build gardening (step 4) on top.
//
//  Library: ThingPulse SH1106Wire ("ESP8266 and ESP32 OLED Driver").
//  CONFIRMED PINS: OLED SDA->8 SCL->9 | DAC DIN->11 BCK->12 LCK->13
//                  ENC A->4 B->5 SW->6   (GPIO 25/26/27 do NOT exist here)
// =====================================================================

#include "driver/i2s.h"
#include <Wire.h>
#include "SH1106Wire.h"
#include <math.h>

// ----------------------- Pins ---------------------------------------
static const int PIN_OLED_SDA = 8;
static const int PIN_OLED_SCL = 9;
static const int PIN_BCK = 12;
static const int PIN_LCK = 13;
static const int PIN_DIN = 11;

// ----------------------- Audio configuration ------------------------
static const uint32_t SAMPLE_RATE   = 44100;
static const int      I2S_NUM_PORT  = I2S_NUM_0;
static const int      DMA_BUF_COUNT = 8;
static const int      DMA_BUF_LEN   = 256;
static const int      BLOCK_FRAMES  = 256;

// ----------------------- Tree configuration -------------------------
static const float ROOT_FREQ      = 110.0f;   // A2 (doc default)
static const int   TREE_DEPTH      = 4;
static const float BRANCH_DENSITY  = 0.60f;
static const float FREQ_CEILING    = 5000.0f;

// ----------------------- Wavetable engine ---------------------------
static const int      TABLE_BITS = 11;                 // 2^11 = 2048
static const uint32_t TABLE_SIZE = 1u << TABLE_BITS;
static const int      FRAC_BITS  = 32 - TABLE_BITS;
static const uint32_t FRAC_MASK  = (1u << FRAC_BITS) - 1;
static const float    FRAC_SCALE = 1.0f / (float)(1u << FRAC_BITS);

enum Waveform {
  WAVE_SINE = 0, WAVE_SINE_PLUS, WAVE_TRIANGLE, WAVE_SOFT_SAW,
  WAVE_SQUARE, WAVE_SAWTOOTH, WAVE_PULSE25, WAVE_SOFTCLIP,
  WAVE_FORM_3_2, WAVE_FORM_7_4, WAVE_PRIME, WAVE_INHARMONIC, WAVE_COUNT
};

static const char* kWaveNames[WAVE_COUNT] = {
  "Sine", "Sine+", "Triangle", "SoftSaw", "Square", "Sawtooth",
  "Pulse25", "SoftClip", "3:2Form", "7:4Form", "Prime", "Inharm"
};

static float gWaveTables[WAVE_COUNT][TABLE_SIZE + 1];

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
  tbl[TABLE_SIZE] = tbl[0];
}

static void buildWaveTables() {
  const int HMAX = 48;
  for (int w = 0; w < WAVE_COUNT; w++)
    for (uint32_t i = 0; i <= TABLE_SIZE; i++) gWaveTables[w][i] = 0.0f;

  addPartial(gWaveTables[WAVE_SINE], 1.0f, 1.0f, 0.0f);

  addPartial(gWaveTables[WAVE_SINE_PLUS], 1.0f, 1.0f, 0.0f);
  addPartial(gWaveTables[WAVE_SINE_PLUS], 2.0f, 0.15f, 0.0f);

  for (int n = 1; n <= HMAX; n += 2) {
    float sign = (((n - 1) / 2) & 1) ? -1.0f : 1.0f;
    addPartial(gWaveTables[WAVE_TRIANGLE], (float)n, sign / (float)(n * n), 0.0f);
  }

  for (int n = 1; n <= HMAX; n++)
    addPartial(gWaveTables[WAVE_SOFT_SAW], (float)n, 1.0f / (float)(n * n), 0.0f);

  for (int n = 1; n <= HMAX; n += 2)
    addPartial(gWaveTables[WAVE_SQUARE], (float)n, 1.0f / (float)n, 0.0f);

  for (int n = 1; n <= HMAX; n++)
    addPartial(gWaveTables[WAVE_SAWTOOTH], (float)n, 1.0f / (float)n, 0.0f);

  {
    const float duty = 0.25f;
    for (int n = 1; n <= HMAX; n++) {
      float a = sinf((float)n * (float)M_PI * duty) / (float)n;
      addPartial(gWaveTables[WAVE_PULSE25], (float)n, a, 0.0f);
    }
  }

  {
    const float drive = 3.0f;
    float* t = gWaveTables[WAVE_SOFTCLIP];
    for (uint32_t i = 0; i < TABLE_SIZE; i++) {
      float ph = (float)i / (float)TABLE_SIZE;
      t[i] = tanhf(drive * sinf(2.0f * (float)M_PI * ph)) / tanhf(drive);
    }
  }

  addPartial(gWaveTables[WAVE_FORM_3_2], 3.0f, 1.0f, 0.0f);
  addPartial(gWaveTables[WAVE_FORM_3_2], 2.0f, 0.7f, 0.0f);

  addPartial(gWaveTables[WAVE_FORM_7_4], 7.0f, 1.0f, 0.0f);
  addPartial(gWaveTables[WAVE_FORM_7_4], 4.0f, 0.7f, 0.0f);

  {
    const int primes[] = {2, 3, 5, 7, 11, 13};
    for (int k = 0; k < 6; k++)
      addPartial(gWaveTables[WAVE_PRIME], (float)primes[k], 1.0f / (float)primes[k], 0.0f);
  }

  addPartial(gWaveTables[WAVE_INHARMONIC], 1.0f,   1.0f, 0.0f);
  addPartial(gWaveTables[WAVE_INHARMONIC], 2.756f, 0.6f, 0.0f);
  addPartial(gWaveTables[WAVE_INHARMONIC], 4.1f,   0.4f, 0.0f);

  for (int w = 0; w < WAVE_COUNT; w++) normalizeTable(gWaveTables[w]);
}

// ----------------------- Seeded PRNG (xorshift32) -------------------
static uint32_t gRng = 0x1234ABCDu;
static inline uint32_t rngNext() {
  uint32_t x = gRng;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  gRng = x;
  return x;
}

// ----------------------- Just-intonation ratios ---------------------
struct Ratio { uint8_t num, den; };
static const Ratio kRatios[] = {
  {2,1}, {3,2}, {4,3}, {5,4}, {7,4}, {6,5}, {9,8}, {11,8}
};
static const int NUM_RATIOS = sizeof(kRatios) / sizeof(kRatios[0]);

// ----------------------------- Voices -------------------------------
struct Voice {
  uint32_t phase;
  uint32_t phaseInc;
  float    baseAmp;     // depth-based level
  float    env;         // grow/prune envelope (1.0 = fully grown). Used by gardening later.
  uint8_t  waveform;
  bool     active;
  float    freq;
  uint8_t  depth;
  uint8_t  rNum, rDen;
  int8_t   parent;
};

static const int NUM_VOICES = 32;
static Voice gVoices[NUM_VOICES];
static volatile int gNodeCount  = 0;
static float gMasterGain = 0.2f;

static inline uint32_t freqToInc(float hz) {
  return (uint32_t)((double)hz * 4294967296.0 / (double)SAMPLE_RATE);
}

static inline float voiceNextSample(int vi) {
  Voice& v = gVoices[vi];
  uint32_t idx  = v.phase >> FRAC_BITS;
  float    frac = (float)(v.phase & FRAC_MASK) * FRAC_SCALE;
  const float* t = gWaveTables[v.waveform];
  float s = t[idx] + (t[idx + 1] - t[idx]) * frac;
  v.phase += v.phaseInc;
  return s * v.baseAmp * v.env;
}

static uint8_t waveformForVoice(uint8_t depth, uint8_t num, uint8_t den, uint32_t r) {
  switch (depth) {
    case 0: return WAVE_SINE;
    case 1: { const uint8_t o[] = {WAVE_SINE_PLUS, WAVE_TRIANGLE, WAVE_SOFT_SAW}; return o[r % 3]; }
    case 2: { const uint8_t o[] = {WAVE_SQUARE, WAVE_SAWTOOTH}; return o[r % 2]; }
    case 3:
      if (num == 3 && den == 2) return WAVE_FORM_3_2;
      if (num == 7 && den == 4) return WAVE_FORM_7_4;
      { const uint8_t o[] = {WAVE_PULSE25, WAVE_FORM_3_2, WAVE_FORM_7_4}; return o[r % 3]; }
    default: return (r & 1) ? WAVE_PRIME : WAVE_INHARMONIC;
  }
}

static int allocVoice() {
  for (int i = 0; i < NUM_VOICES; i++)
    if (!gVoices[i].active) return i;
  return -1;
}

static void setupVoice(int i, uint8_t depth, float freq,
                       uint8_t num, uint8_t den, int parent) {
  Voice& v = gVoices[i];
  v.depth    = depth;
  v.freq     = freq;
  v.rNum     = num;
  v.rDen     = den;
  v.parent   = (int8_t)parent;
  v.waveform = waveformForVoice(depth, num, den, rngNext());
  v.phaseInc = freqToInc(freq);
  v.phase    = rngNext();
  v.baseAmp  = powf(0.65f, (float)depth);
  v.env      = 1.0f;                          // fully grown (gardening fades come in step 4)
  v.active   = true;
  gNodeCount++;
}

static void buildTree(uint8_t maxDepth, float rootFreq, float density) {
  for (int i = 0; i < NUM_VOICES; i++) gVoices[i].active = false;
  gNodeCount = 0;
  gRng = 0x1234ABCDu;

  int queue[NUM_VOICES];
  int qh = 0, qt = 0;

  int root = allocVoice();
  setupVoice(root, 0, rootFreq, 1, 1, -1);
  queue[qt++] = root;

  while (qh < qt) {
    int p = queue[qh++];
    if (gVoices[p].depth >= maxDepth) continue;

    int nChildren = 2 + (((rngNext() % 100) < (uint32_t)(density * 100)) ? 1 : 0);
    for (int c = 0; c < nChildren; c++) {
      uint32_t r = rngNext();
      Ratio rr = kRatios[r % NUM_RATIOS];
      float cf = gVoices[p].freq * (float)rr.num / (float)rr.den;
      if (cf > FREQ_CEILING) continue;

      int v = allocVoice();
      if (v < 0) { qh = qt; break; }
      setupVoice(v, gVoices[p].depth + 1, cf, rr.num, rr.den, p);
      queue[qt++] = v;
    }
  }

  gMasterGain = 1.2f / sqrtf((float)(gNodeCount > 0 ? gNodeCount : 1));
}

// ----------------------- Mix one audio block ------------------------
static void renderBlock(int16_t* out) {
  for (int n = 0; n < BLOCK_FRAMES; n++) {
    float mix = 0.0f;
    for (int v = 0; v < NUM_VOICES; v++) {
      if (!gVoices[v].active) continue;
      mix += voiceNextSample(v);
    }
    mix *= gMasterGain;
    if (mix >  1.0f) mix =  1.0f;
    if (mix < -1.0f) mix = -1.0f;
    int16_t s = (int16_t)(mix * 32767.0f);
    out[2 * n]     = s;
    out[2 * n + 1] = s;
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

// ------------------------- Display (Core 0) -------------------------
// ThingPulse SH1106Wire: constructor (i2c_addr, sda, scl).
static SH1106Wire display(0x3c, PIN_OLED_SDA, PIN_OLED_SCL);

static volatile int gFps = 0;   // updated by core 0, shown on screen + serial

static void drawScreen() {
  display.clear();

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Harmonic Tree v2");

  char buf[24];
  snprintf(buf, sizeof(buf), "FPS %d", gFps);
  display.drawString(0, 12, buf);

  snprintf(buf, sizeof(buf), "Voices %d", gNodeCount);
  display.drawString(64, 12, buf);

  // Sweeping bar so the refresh rate is visible to the eye. Position is
  // time-based, so smooth motion == steady frame pacing.
  int x = (int)((millis() / 4) % 128);
  display.drawVerticalLine(x, 28, 12);
  display.drawRect(0, 28, 128, 12);

  // A little "alive" footer.
  display.drawString(0, 44, "dual-core: audio C1 / disp C0");

  display.display();   // pushes the frame buffer over I2C
}

// ------------------------------ Setup -------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Harmonic Tree v2 - Step 1: dual-core + fast display");

  // ---- Display on core 0 ----
  display.init();
  display.flipScreenVertically();             // correct orientation (validated)
  Wire.setClock(400000);                       // 400kHz I2C (doc §3 checklist)
  display.setContrast(255);
  display.clear();
  display.drawString(0, 24, "booting...");
  display.display();

  // ---- Audio engine ----
  buildWaveTables();
  buildTree(TREE_DEPTH, ROOT_FREQ, BRANCH_DENSITY);
  Serial.printf("Tree: %d voices, masterGain=%.3f\n", gNodeCount, gMasterGain);

  i2sSetup();
  // Audio render pinned to CORE 1, high priority — only i2s_write blocks,
  // and only on that core, so the display on core 0 never stalls.
  xTaskCreatePinnedToCore(audioTask, "audio", 4096, NULL,
                          configMAX_PRIORITIES - 1, NULL, 1);

  Serial.println("Audio on Core 1, display on Core 0. Watch the FPS.");
}

// --------------------------- Loop (Core 0) --------------------------
void loop() {
  // Draw as fast as we can and measure FPS over 1s windows.
  static uint32_t frames = 0;
  static uint32_t lastFpsMs = 0;

  drawScreen();
  frames++;

  uint32_t now = millis();
  if (now - lastFpsMs >= 1000) {
    gFps = (int)frames;
    frames = 0;
    lastFpsMs = now;
    Serial.printf("FPS: %d   (voices %d)\n", gFps, gNodeCount);
  }
}
