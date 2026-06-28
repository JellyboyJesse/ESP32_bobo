// =====================================================================
//  Harmonic Tree v2 — Step 4: Gardening Interaction
//  ESP32-S3 DevKitC-1 N16R8
//
//  Per architecture doc v2 §5 / §10 step 4. The core playable loop:
//    - ROTATE  -> move a selection cursor through the tree, in visual
//                 (spatial) order, visiting existing branches AND empty
//                 grow-slots (one slot per node)
//    - CLICK   -> empty slot   : GROW a new branch (fades in, ratio by
//                                weighted random, starts as a simple wave)
//                 existing branch: PRUNE it + all its children (fade out)
//    Root is the seed and cannot be pruned.
//
//  Core split (the v2 priority): audio render task on CORE 1; a dedicated
//  control task (encoder + envelopes + gardening + display, all the I2C)
//  on CORE 0. Arduino's loop() (which runs on core 1 by default) is left
//  idle so it never competes with audio.
//
//  Encoder: quadrature full-step state machine, interrupts on BOTH A & B,
//  rests at 0b11, one count per detent. Button: 30ms debounce, fire once.
//
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
static const int PIN_ENC_A = 4;
static const int PIN_ENC_B = 5;
static const int PIN_ENC_SW = 6;

// ----------------------- Audio configuration ------------------------
static const uint32_t SAMPLE_RATE   = 44100;
static const int      I2S_NUM_PORT  = I2S_NUM_0;
static const int      DMA_BUF_COUNT = 10;    // a little extra cushion against transient spikes
static const int      DMA_BUF_LEN   = 256;
static const int      BLOCK_FRAMES  = 256;

// ----------------------- Tree / gardening config --------------------
static const float ROOT_FREQ            = 110.0f;   // A2
static const int   MAX_GROW_DEPTH        = 5;        // children may reach depth 5
static const int   MAX_CHILDREN_PER_NODE = 3;        // branch limit per node (§4.3)
static const float FREQ_CEILING          = 5000.0f;
static const float GROW_T  = 1.5f;                   // grow fade-in (s)
static const float PRUNE_T = 1.8f;                   // prune fade-out (s)
// Mix auto-gain: keep the worst-case summed peak inside a headroom budget
// so the signal stays in tanh's gentle region regardless of branch count.
static const float GAIN_CAP     = 0.40f;             // max gain (few/quiet voices)
static const float MIX_HEADROOM = 0.85f;             // peak budget into the soft clip
static const float GAIN_TAU     = 0.40f;             // smoothing time constant (s) — no pumping
static volatile float gMixGain  = GAIN_CAP;          // written by control core, read by audio core
// Rhythmic pulse / polyrhythm layer
static volatile float gPulseAmount   = 0.80f;        // dial: 0 = pure drone, 1 = fully rhythmic
static const float PULSE_BASE_HZ      = 1.0f;        // base pulse rate (× node ratio)
static const float PULSE_NODE_CHANCE  = 0.70f;       // chance a (non-root) node pulses
static const float PULSE_RANDOM_CHANCE = 0.12f;      // chance a pulsing node takes a random tempo
static volatile float gTempoBase  = 1.0f;            // user tempo (set on the TEMPO page)
static volatile float gTempoScale = 1.0f;            // live tempo = base × seasonal drift
// Evolving filter: slow per-voice cutoff wander (control-rate, free on audio core)
static const float WANDER_DEPTH = 0.35f;             // ± fraction the cutoff drifts
// Ratio-shift glides (layer 2, 'one-off events')
static volatile float gRetuneLiveliness   = 1.0f;    // scales how often branches retune
static const float RETUNE_CHANCE_PER_SEC  = 0.018f;  // ~ once per 55s per branch (× liveliness)
// Per-voice pitch drift (living detune)
static volatile float gPitchDrift   = 1.0f;          // scales the drift amount
static const float MAX_DRIFT_CENTS  = 7.0f;
// Rare 'bloom' gestures (a branch flings its filter open, then closes)
static const float BLOOM_DEPTH = 3.0f;               // extra cutoff multiplier at full bloom
static const float BLOOM_TIME  = 3.5f;               // bloom decay time (s)
static const float BLOOM_CHANCE_PER_SEC = 0.012f;    // per idle branch
// Seasons: one very slow master LFO that modulates several params (weather)
static const float SEASON_PERIOD_S      = 200.0f;    // ~3.3 min cycle
static const float PULSE_AMOUNT_BASE     = 0.70f;
static const float MORPH_LIVELINESS_BASE = 1.0f;
static const float EROSION_BASE          = 0.5f;
static float gSeasonPhase = 0.0f;
static volatile float gBrightness = 1.0f;            // global filter brightness (driven by seasons)

// ----------------------- Wavetable engine ---------------------------
static const int      TABLE_BITS = 11;
static const uint32_t TABLE_SIZE = 1u << TABLE_BITS;
static const int      FRAC_BITS  = 32 - TABLE_BITS;
static const uint32_t FRAC_MASK  = (1u << FRAC_BITS) - 1;
static const float    FRAC_SCALE = 1.0f / (float)(1u << FRAC_BITS);

enum Waveform {
  WAVE_SINE = 0, WAVE_SINE_PLUS, WAVE_TRIANGLE, WAVE_SOFT_SAW,
  WAVE_SQUARE, WAVE_SAWTOOTH, WAVE_PULSE25, WAVE_SOFTCLIP,
  WAVE_FORM_3_2, WAVE_FORM_7_4, WAVE_PRIME, WAVE_INHARMONIC, WAVE_COUNT
};

static float gWaveTables[WAVE_COUNT][TABLE_SIZE + 1];

// Wavetable erosion ("DNA decay"): the live tables are slowly smoothed
// while in use and heal back toward this pristine copy when idle. Pristine
// lives in PSRAM (read-only, occasional) to keep internal RAM for audio.
static float EXT_RAM_BSS_ATTR gPristine[WAVE_COUNT][TABLE_SIZE + 1];
static volatile float gErosionAmount = 0.5f;     // 0 = DNA stable, 1 = ages fast
static const int   EROSION_WINDOW   = 96;         // samples smoothed per event
static const float EROSION_MAX_SCAR = 0.55f;      // max |deviation| from pristine (no runaway)
static const float HEAL_RATE        = 0.05f;      // idle-table healing per heal tick

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
  { const float duty = 0.25f;
    for (int n = 1; n <= HMAX; n++) {
      float a = sinf((float)n * (float)M_PI * duty) / (float)n;
      addPartial(gWaveTables[WAVE_PULSE25], (float)n, a, 0.0f);
    } }
  { const float drive = 3.0f; float* t = gWaveTables[WAVE_SOFTCLIP];
    for (uint32_t i = 0; i < TABLE_SIZE; i++) {
      float ph = (float)i / (float)TABLE_SIZE;
      t[i] = tanhf(drive * sinf(2.0f * (float)M_PI * ph)) / tanhf(drive);
    } }
  addPartial(gWaveTables[WAVE_FORM_3_2], 3.0f, 1.0f, 0.0f);
  addPartial(gWaveTables[WAVE_FORM_3_2], 2.0f, 0.7f, 0.0f);
  addPartial(gWaveTables[WAVE_FORM_7_4], 7.0f, 1.0f, 0.0f);
  addPartial(gWaveTables[WAVE_FORM_7_4], 4.0f, 0.7f, 0.0f);
  { const int primes[] = {2, 3, 5, 7, 11, 13};
    for (int k = 0; k < 6; k++)
      addPartial(gWaveTables[WAVE_PRIME], (float)primes[k], 1.0f / (float)primes[k], 0.0f); }
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

// ----------------------- Just-intonation ratios (§4.4) --------------
struct Ratio { uint8_t num, den; };
static const Ratio   kRatios[]     = { {2,1},{3,2},{4,3},{5,4},{7,4},{6,5},{9,8},{11,8} };
static const uint8_t kRatioWeight[] = {  3,    3,    3,    2,    1,    1,    1,    1   };
static const int NUM_RATIOS = sizeof(kRatios) / sizeof(kRatios[0]);

// ----------------------------- Voices -------------------------------
// Biquad coefficients (normalized, a0 = 1). Double-buffered per voice so
// the audio core never reads a half-written (potentially unstable) set.
struct Biquad { float b0, b1, b2, a1, a2; };

struct Voice {
  uint32_t phase;
  uint32_t phaseInc;
  float    baseAmp;
  float    env;          // grow/prune envelope 0..1
  float    envTarget;    // 0 = pruning/dead, 1 = alive
  uint8_t  waveA, waveB; // morph endpoints (evolution layer 1)
  float    morphPos;     // 0 = waveA, 1 = waveB (read per-sample by audio)
  float    morphRate;    // waveforms per second (per-voice)
  // rhythmic pulse (polyrhythm layer)
  float    pulsePhase;   // 0..1 within the pulse period (audio-core state)
  float    pulseInc;     // per-sample phase increment = rateHz × tempo / SR
  float    pulseBaseHz;  // un-scaled ratio-derived pulse rate (Hz)
  float    pulseAtk;     // attack fraction of the period
  float    pulseDepth;   // per-node participation: 0 = sustain, 1 = pulses
  float    wanderPhase;  // slow filter-cutoff wander 0..1 (evolving filter)
  float    wanderRate;   // wander cycles per second
  float    glideFrom, glideTo;  // ratio-shift glide endpoints (Hz, layer 2)
  float    glideProg;    // 0..1 along the glide; >=1 = idle
  float    glideInc;     // progress per second = 1 / glide duration
  float    driftCents;   // living micro-detune random-walk (pitch drift)
  float    bloom;        // 0..1 rare filter-swell gesture
  float    panL, panR;   // stereo pan gains (equal-power)
  bool     active;       // node exists in pool (set true LAST when growing)
  float    freq;
  uint8_t  depth;
  uint8_t  rNum, rDen;
  int8_t   parent;
  // per-voice filter (architecture §4.5)
  Biquad          coef[2];     // double buffer
  volatile uint8_t coefSel;    // which buffer the audio core reads (atomic byte)
  float           z1, z2;      // filter state (audio core only)
};

// Pool ceiling. Render cost is ~180us/voice + ~560us fixed; the 5805us/block
// budget runs out near 29 voices, so 28 keeps a safe margin and stays
// click-free at full quality (all evolution layers + stereo).
static const int NUM_VOICES = 28;
static Voice gVoices[NUM_VOICES];
static int   gRootIndex = -1;

static inline uint32_t freqToInc(float hz) {
  return (uint32_t)((double)hz * 4294967296.0 / (double)SAMPLE_RATE);
}

static inline float __attribute__((always_inline)) voiceNextSample(int vi) {
  Voice& v = gVoices[vi];
  uint32_t idx  = v.phase >> FRAC_BITS;
  float    frac = (float)(v.phase & FRAC_MASK) * FRAC_SCALE;
  // crossfade between waveform A and B (evolution layer 1)
  const float* tA = gWaveTables[v.waveA];
  const float* tB = gWaveTables[v.waveB];
  float sA = tA[idx] + (tA[idx + 1] - tA[idx]) * frac;
  float sB = tB[idx] + (tB[idx + 1] - tB[idx]) * frac;
  float s = sA + (sB - sA) * v.morphPos;
  v.phase += v.phaseInc;

  // per-voice biquad (transposed direct form II); coeffs from the buffer
  // the control core last published — always a complete, stable set.
  const Biquad& c = v.coef[v.coefSel];
  float x = s * v.baseAmp;
  float y = c.b0 * x + v.z1;
  v.z1 = c.b1 * x - c.a1 * y + v.z2;
  v.z2 = c.b2 * x - c.a2 * y;

  float out = y * v.env;   // birth/death envelope scales the filtered signal

  // rhythmic pulse: retriggering attack/decay envelope (polyrhythm layer).
  // Skipped entirely for sustained nodes (saves CPU at high voice counts).
  if (v.pulseDepth > 0.0f) {
    v.pulsePhase += v.pulseInc;
    if (v.pulsePhase >= 1.0f) v.pulsePhase -= 1.0f;
    float pe;
    if (v.pulsePhase < v.pulseAtk) {
      pe = v.pulsePhase / v.pulseAtk;                      // attack
    } else {
      float r = 1.0f - (v.pulsePhase - v.pulseAtk) / (1.0f - v.pulseAtk);
      pe = r * r;                                          // decay to 0 by period end
    }
    out *= 1.0f - gPulseAmount * (1.0f - pe);              // lerp(sustain, pulse)
  }
  return out;
}

static int allocVoice() {
  for (int i = 0; i < NUM_VOICES; i++)
    if (!gVoices[i].active) return i;
  return -1;
}

static int countChildren(int v) {
  int n = 0;
  for (int i = 0; i < NUM_VOICES; i++)
    if (gVoices[i].active && gVoices[i].parent == v) n++;
  return n;
}

static int activeCount() {
  int n = 0;
  for (int i = 0; i < NUM_VOICES; i++) if (gVoices[i].active) n++;
  return n;
}

// A node can offer a grow-slot if it's alive (not being pruned) and has
// room for another child.
static bool canGrow(int v) {
  return gVoices[v].active &&
         gVoices[v].envTarget > 0.5f &&
         gVoices[v].depth < MAX_GROW_DEPTH &&
         countChildren(v) < MAX_CHILDREN_PER_NODE &&
         activeCount() < NUM_VOICES;          // hide grow-slots when the pool is full
}

// Weighted ratio pick among ratios whose child stays under the ceiling.
static int chooseRatio(float parentFreq) {
  int total = 0;
  for (int i = 0; i < NUM_RATIOS; i++)
    if (parentFreq * kRatios[i].num / kRatios[i].den <= FREQ_CEILING)
      total += kRatioWeight[i];
  if (total == 0) return -1;
  int r = rngNext() % total;
  for (int i = 0; i < NUM_RATIOS; i++) {
    if (parentFreq * kRatios[i].num / kRatios[i].den > FREQ_CEILING) continue;
    if (r < kRatioWeight[i]) return i;
    r -= kRatioWeight[i];
  }
  return -1;
}

// ----------------------- Waveform morphing (layer 1) ----------------
static float gMorphLiveliness = 1.0f;     // single tuning knob (§6 layer 1)

// Pick the next morph target, weighted by depth: deeper branches reach
// toward the more complex tables (root drifts among the simple ones).
static uint8_t pickNextWave(uint8_t depth) {
  int lo = depth * 2;
  int hi = lo + 4;                         // a 4-wide window that climbs with depth
  if (hi > WAVE_COUNT) { hi = WAVE_COUNT; lo = hi - 4; }
  if (lo < 0) lo = 0;
  return (uint8_t)(lo + (int)(rngNext() % (uint32_t)(hi - lo)));
}

// Set up a voice's rhythmic pulse. Tempo is ratio-derived (rate = base ×
// n/d) so the polyrhythm mirrors the harmony; rarely a node takes a random
// tempo. The root never pulses — it's the steady drone bed.
static void initPulse(int i) {
  Voice& v = gVoices[i];
  bool pulses = (i != gRootIndex) &&
                ((rngNext() % 100) < (uint32_t)(PULSE_NODE_CHANCE * 100));
  v.pulseDepth = pulses ? 1.0f : 0.0f;

  float rateHz;
  if (pulses && (rngNext() % 100) < (uint32_t)(PULSE_RANDOM_CHANCE * 100)) {
    rateHz = 0.4f + (rngNext() % 2600) * 0.001f;          // rare wild node: 0.4..3.0 Hz
  } else {
    rateHz = PULSE_BASE_HZ * (float)v.rNum / (float)v.rDen;
  }
  v.pulseBaseHz = rateHz;
  v.pulseInc    = rateHz * gTempoScale / (float)SAMPLE_RATE;
  v.pulseAtk   = 0.03f + (rngNext() % 220) * 0.001f;       // 0.03..0.25 of period
  v.pulsePhase = (rngNext() % 1000) * 0.001f;              // random start phase

  // slow per-voice filter wander (evolving filter, control-rate)
  v.wanderPhase = (rngNext() % 1000) * 0.001f;
  v.wanderRate  = 0.03f + (rngNext() % 120) * 0.001f;      // 0.03..0.15 Hz

  // ratio-shift glide starts idle
  v.glideFrom = v.freq; v.glideTo = v.freq; v.glideProg = 1.0f; v.glideInc = 0.0f;

  v.driftCents = 0.0f;
  v.bloom      = 0.0f;

  // equal-power stereo pan: root centered, others spread across the field
  float pan = (i == gRootIndex) ? 0.5f
              : 0.5f + 0.45f * (((int)(rngNext() % 2001) - 1000) * 0.001f);
  v.panL = cosf(pan * 1.5708f);
  v.panR = sinf(pan * 1.5708f);
}

// ----------------------- Grow / prune -------------------------------
static volatile bool gTreeDirty = true;   // rebuild layout when structure changes

static int growBranch(int parent) {
  if (!canGrow(parent)) return -1;
  int gi = chooseRatio(gVoices[parent].freq);
  if (gi < 0) return -1;
  int v = allocVoice();
  if (v < 0) return -1;

  Ratio rr = kRatios[gi];
  Voice& nv = gVoices[v];
  nv.depth     = gVoices[parent].depth + 1;
  nv.freq      = gVoices[parent].freq * (float)rr.num / (float)rr.den;
  nv.rNum      = rr.num;
  nv.rDen      = rr.den;
  nv.parent    = (int8_t)parent;
  nv.waveA     = WAVE_SINE;                  // starts simple, morphs from here
  nv.waveB     = pickNextWave(nv.depth);
  nv.morphPos  = 0.0f;
  nv.morphRate = 1.0f / (8.0f + (rngNext() % 2200) * 0.01f);   // one wave per 8..30s
  nv.phaseInc  = freqToInc(nv.freq);
  nv.phase     = rngNext();
  nv.baseAmp   = powf(0.65f, (float)nv.depth);
  nv.env       = 0.0f;                       // fade in
  nv.envTarget = 1.0f;
  nv.z1 = 0.0f; nv.z2 = 0.0f; nv.coefSel = 0;   // start with a clean passthrough filter
  nv.coef[0].b0 = 1.0f; nv.coef[0].b1 = 0.0f; nv.coef[0].b2 = 0.0f;
  nv.coef[0].a1 = 0.0f; nv.coef[0].a2 = 0.0f;
  nv.coef[1] = nv.coef[0];
  initPulse(v);                              // ratio-derived rhythmic pulse
  __sync_synchronize();                      // publish fields before active=true
  nv.active    = true;                       // Core 1 only renders once this is set
  gTreeDirty   = true;
  return v;
}

// Mark a branch and all its descendants to fade out; envelope update
// frees them once silent. Root is protected.
static void pruneBranch(int v) {
  if (v == gRootIndex) return;
  for (int i = 0; i < NUM_VOICES; i++)
    if (gVoices[i].active && gVoices[i].parent == v) pruneBranch(i);
  gVoices[v].envTarget = 0.0f;
}

static void plantSeed() {
  for (int i = 0; i < NUM_VOICES; i++) {
    gVoices[i].active = false;
    gVoices[i].env = 0.0f;
    gVoices[i].envTarget = 0.0f;
  }
  gRng = 0x1234ABCDu;
  int v = allocVoice();
  Voice& r = gVoices[v];
  r.depth = 0; r.freq = ROOT_FREQ; r.rNum = 1; r.rDen = 1; r.parent = -1;
  r.waveA = WAVE_SINE; r.waveB = pickNextWave(0); r.morphPos = 0.0f;
  r.morphRate = 1.0f / 18.0f;
  r.phaseInc = freqToInc(ROOT_FREQ); r.phase = 0;
  r.baseAmp = 1.0f; r.env = 0.0f; r.envTarget = 1.0f;
  r.z1 = 0.0f; r.z2 = 0.0f; r.coefSel = 0;       // root filter is bypass
  r.coef[0].b0 = 1.0f; r.coef[0].b1 = 0.0f; r.coef[0].b2 = 0.0f;
  r.coef[0].a1 = 0.0f; r.coef[0].a2 = 0.0f;
  r.coef[1] = r.coef[0];
  gRootIndex = v;                                 // set before initPulse (root won't pulse)
  initPulse(v);
  r.active = true;
  gTreeDirty = true;
}

// Advance grow/prune envelopes (control rate, Core 0). Frees pruned
// voices once they reach silence.
// Compute this voice's biquad (RBJ cookbook) into the spare buffer, then
// publish it. Runs at control rate on Core 0. Depth sets the filter type;
// env ('openness') opens the filter as the branch grows / closes on prune.
static void computeVoiceFilter(int i) {
  Voice& v = gVoices[i];
  uint8_t nx = v.coefSel ^ 1;
  Biquad& nb = v.coef[nx];

  if (v.depth == 0) {                 // root: flat / bypass
    nb.b0 = 1.0f; nb.b1 = 0.0f; nb.b2 = 0.0f; nb.a1 = 0.0f; nb.a2 = 0.0f;
  } else {
    float open = v.env;               // 0 = dark/closed, 1 = fully open
    int type; float f0, Q;
    if (v.depth == 1) {               // gentle low-pass that opens to bright
      type = 0; Q = 0.707f;
      float bright = v.freq * 7.0f;
      if (bright > 14000.0f) bright = 14000.0f;
      if (bright < 2000.0f)  bright = 2000.0f;
      f0 = v.freq * 1.5f + (bright - v.freq * 1.5f) * open;
    } else if (v.depth == 2) {        // band-pass on the voice's ratio freq
      type = 1; Q = 1.5f;
      f0 = v.freq * (0.6f + 0.4f * open);
    } else if (v.depth == 3) {        // band-pass, narrower
      type = 1; Q = 3.0f;
      f0 = v.freq * (0.6f + 0.4f * open);
    } else {                          // depth 4+: high-pass, thins twigs
      type = 2; Q = 0.707f;
      f0 = v.freq * (3.0f - 1.7f * open);   // born thin (high), opens lower
    }
    // evolving filter: global brightness (seasons) × per-voice wander × bloom swell
    f0 *= gBrightness
          * (1.0f + WANDER_DEPTH * sinf(2.0f * (float)M_PI * v.wanderPhase))
          * (1.0f + BLOOM_DEPTH * v.bloom);

    if (f0 < 20.0f) f0 = 20.0f;
    float maxf = SAMPLE_RATE * 0.45f;
    if (f0 > maxf) f0 = maxf;

    float w0 = 2.0f * (float)M_PI * f0 / (float)SAMPLE_RATE;
    float cw = cosf(w0), sw = sinf(w0);
    float alpha = sw / (2.0f * Q);
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cw;
    float a2 = 1.0f - alpha;
    float b0, b1, b2;
    if (type == 0)      { b0 = (1 - cw) * 0.5f; b1 = 1 - cw;  b2 = (1 - cw) * 0.5f; }
    else if (type == 1) { b0 = alpha;           b1 = 0;       b2 = -alpha;          }
    else                { b0 = (1 + cw) * 0.5f; b1 = -(1 + cw); b2 = (1 + cw) * 0.5f; }
    nb.b0 = b0 / a0; nb.b1 = b1 / a0; nb.b2 = b2 / a0;
    nb.a1 = a1 / a0; nb.a2 = a2 / a0;
  }

  __sync_synchronize();               // publish coeffs before the select flips
  v.coefSel = nx;
}

// Advance one voice's waveform morph. When the blend completes, A takes
// over from B and a new B is chosen. The order (A<-B, zero the blend,
// THEN change B) makes B's weight zero exactly when it changes, so the
// swap is seamless across cores.
static void advanceMorph(int i, float dt) {
  Voice& v = gVoices[i];
  v.morphPos += v.morphRate * gMorphLiveliness * dt;
  if (v.morphPos >= 1.0f) {
    v.waveA = v.waveB;
    v.morphPos = 0.0f;
    __sync_synchronize();
    v.waveB = pickNextWave(v.depth);
  }
}

// Begin a ratio-shift glide: pick a new (weighted) interval relative to the
// parent and slide there over 2-4s. Root has no parent and never retunes.
static void startRetune(int i) {
  Voice& v = gVoices[i];
  if (v.parent < 0) return;
  float pf = gVoices[v.parent].freq;
  int gi = chooseRatio(pf);
  if (gi < 0) return;
  v.glideFrom = v.freq;
  v.glideTo   = pf * (float)kRatios[gi].num / (float)kRatios[gi].den;
  v.rNum = kRatios[gi].num;
  v.rDen = kRatios[gi].den;
  v.glideProg = 0.0f;
  v.glideInc  = 1.0f / (2.0f + (rngNext() % 2000) * 0.001f);   // 2..4s glide
}

// Advance a glide (exponential = constant-rate in pitch), or — when idle —
// occasionally trigger a new one. Only gliding voices do the powf.
static void advanceRetune(int i, float dt) {
  Voice& v = gVoices[i];
  if (v.glideProg < 1.0f) {
    v.glideProg += v.glideInc * dt;
    if (v.glideProg >= 1.0f) { v.glideProg = 1.0f; v.freq = v.glideTo; }
    else v.freq = v.glideFrom * powf(v.glideTo / v.glideFrom, v.glideProg);
    // phaseInc is recomputed from v.freq (with drift) once per voice below
  } else if (i != gRootIndex && v.envTarget > 0.5f) {
    float p = RETUNE_CHANCE_PER_SEC * gRetuneLiveliness * dt;
    if ((float)rngNext() / 4294967296.0f < p) startRetune(i);
  }
}

// One slow master LFO (minutes) nudging several params together: the
// instrument's 'weather'. Different phase offsets so they don't all peak
// at once. All cheap, control-rate.
static void advanceSeasons(float dt) {
  gSeasonPhase += dt / SEASON_PERIOD_S;
  if (gSeasonPhase >= 1.0f) gSeasonPhase -= 1.0f;
  float a = sinf(2.0f * (float)M_PI * gSeasonPhase);
  float b = sinf(2.0f * (float)M_PI * (gSeasonPhase + 0.33f));
  float c = sinf(2.0f * (float)M_PI * (gSeasonPhase + 0.66f));

  float pa = PULSE_AMOUNT_BASE * (1.0f + 0.45f * a);
  gPulseAmount = pa < 0.0f ? 0.0f : (pa > 1.0f ? 1.0f : pa);
  float ml = MORPH_LIVELINESS_BASE * (1.0f + 0.6f * b);
  gMorphLiveliness = ml < 0.1f ? 0.1f : ml;
  gBrightness = 1.0f + 0.35f * c;
  float er = EROSION_BASE * (1.0f + 0.7f * b);
  gErosionAmount = er < 0.0f ? 0.0f : er;
  gTempoScale = gTempoBase * (1.0f + 0.15f * a);   // gentle seasonal tempo drift
}

static void updateEnvelopes(float dt) {
  advanceSeasons(dt);
  float sumAmp = 0.0f;
  for (int i = 0; i < NUM_VOICES; i++) {
    Voice& v = gVoices[i];
    if (!v.active) continue;
    if (v.env < v.envTarget) {
      v.env += dt / GROW_T;
      if (v.env > v.envTarget) v.env = v.envTarget;
    } else if (v.env > v.envTarget) {
      v.env -= dt / PRUNE_T;
      if (v.env <= 0.0015f && v.envTarget == 0.0f) {
        v.env = 0.0f;
        v.active = false;                    // returned to pool
        gTreeDirty = true;
      }
    }
    sumAmp += v.baseAmp * v.env;             // current worst-case in-phase peak
    if (v.active) {
      v.wanderPhase += v.wanderRate * dt;    // advance evolving-filter wander
      if (v.wanderPhase >= 1.0f) v.wanderPhase -= 1.0f;
      advanceRetune(i, dt);                  // layer 2: occasional ratio-shift glide

      // living pitch drift: bounded random walk, gently centered
      float rnd = ((int)(rngNext() % 2001) - 1000) * 0.001f;   // -1..1
      v.driftCents += rnd * gPitchDrift * dt * 8.0f;
      v.driftCents *= (1.0f - 0.3f * dt);
      if (v.driftCents >  MAX_DRIFT_CENTS) v.driftCents =  MAX_DRIFT_CENTS;
      if (v.driftCents < -MAX_DRIFT_CENTS) v.driftCents = -MAX_DRIFT_CENTS;
      v.phaseInc = freqToInc(v.freq * exp2f(v.driftCents * (1.0f / 1200.0f)));
      if (v.pulseDepth > 0.0f)
        v.pulseInc = v.pulseBaseHz * gTempoScale / (float)SAMPLE_RATE;   // live tempo

      // rare bloom gesture: filter swells open, then decays closed
      if (v.bloom > 0.0f) {
        v.bloom -= dt / BLOOM_TIME;
        if (v.bloom < 0.0f) v.bloom = 0.0f;
      } else if (i != gRootIndex && v.envTarget > 0.5f) {
        if ((float)rngNext() / 4294967296.0f < BLOOM_CHANCE_PER_SEC * dt) v.bloom = 1.0f;
      }

      computeVoiceFilter(i);                 // refresh filter (brightness/wander/bloom/glide)
      advanceMorph(i, dt);                   // evolution layer 1: waveform morph
    }
  }

  // Auto-gain toward the headroom budget, smoothed so grow/prune doesn't pump.
  float target = GAIN_CAP;
  if (sumAmp > 1e-4f) {
    float g = MIX_HEADROOM / sumAmp;
    if (g < target) target = g;
  }
  float a = dt / GAIN_TAU;
  if (a > 1.0f) a = 1.0f;
  gMixGain += (target - gMixGain) * a;
}

// ----------------------- Wavetable erosion / healing (Core 0) -------
static bool tableInUse(int w) {
  for (int i = 0; i < NUM_VOICES; i++)
    if (gVoices[i].active && (gVoices[i].waveA == w || gVoices[i].waveB == w)) return true;
  return false;
}

// Smooth (low-pass) a random window of a random in-use table. Smoothing is
// continuity-preserving, so it 'scars' the timbre without injecting clicks;
// the clamp keeps it bounded (no runaway / DC drift). Single float writes
// are atomic, so the audio core just reads old-or-new, never garbage.
static void erodeStep() {
  int inuse[WAVE_COUNT], n = 0;
  for (int w = 0; w < WAVE_COUNT; w++) if (tableInUse(w)) inuse[n++] = w;
  if (n == 0) return;
  int w = inuse[rngNext() % n];
  float* t = gWaveTables[w];
  const float* p = gPristine[w];
  float strength = 0.5f * gErosionAmount;
  int start = rngNext() & (TABLE_SIZE - 1);
  for (int k = 0; k < EROSION_WINDOW; k++) {
    int j  = (start + k) & (TABLE_SIZE - 1);
    int jm = (j - 1) & (TABLE_SIZE - 1);
    int jp = (j + 1) & (TABLE_SIZE - 1);
    float nv = t[j] + strength * (0.5f * (t[jm] + t[jp]) - t[j]);
    float d = nv - p[j];
    if (d >  EROSION_MAX_SCAR) nv = p[j] + EROSION_MAX_SCAR;
    if (d < -EROSION_MAX_SCAR) nv = p[j] - EROSION_MAX_SCAR;
    t[j] = nv;
  }
  t[TABLE_SIZE] = t[0];                    // keep interpolation guard in sync
}

// Tables no voice is currently using drift back toward pristine (never
// heard, since nothing reads them). One table per call, round-robin.
static int gHealCursor = 0;
static void healStep() {
  int w = gHealCursor;
  gHealCursor = (gHealCursor + 1) % WAVE_COUNT;
  if (tableInUse(w)) return;
  float* t = gWaveTables[w];
  const float* p = gPristine[w];
  for (uint32_t j = 0; j < TABLE_SIZE; j++) t[j] += (p[j] - t[j]) * HEAL_RATE;
  t[TABLE_SIZE] = t[0];
}

// ----------------------- Mix one audio block (Core 1) ---------------
// Forced to -O2 (Arduino defaults to -Os) and pinned in IRAM: this is the
// hot loop, and the math runs much faster compiled for speed without
// flash-cache stalls. If your toolchain rejects the optimize attribute,
// delete the __attribute__((optimize("O2"))) token and it still builds.
// Cheap cubic soft clip (tanh-like smooth knee, no libm): slope 0 at ±1 so
// it meets the flat ceiling with no hard corner -> click-free, output in [-1,1].
static inline int16_t __attribute__((always_inline)) softClip16(float x) {
  if (x > 1.0f) x = 1.0f; else if (x < -1.0f) x = -1.0f;
  x = 1.5f * x - 0.5f * x * x * x;
  return (int16_t)(x * 32767.0f);
}

static void IRAM_ATTR __attribute__((optimize("O2"))) renderBlock(int16_t* out) {
  for (int n = 0; n < BLOCK_FRAMES; n++) {
    float mixL = 0.0f, mixR = 0.0f;
    for (int v = 0; v < NUM_VOICES; v++) {
      if (!gVoices[v].active) continue;
      float o = voiceNextSample(v);          // mono voice output
      mixL += o * gVoices[v].panL;           // equal-power stereo spread
      mixR += o * gVoices[v].panR;
    }
    out[2 * n]     = softClip16(mixL * gMixGain);
    out[2 * n + 1] = softClip16(mixR * gMixGain);
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

static int16_t gBlockBuf[BLOCK_FRAMES * 2];
static volatile uint32_t gRenderMaxUs = 0;            // worst block render time (vs ~5805us budget)

static void audioTask(void* param) {
  size_t bytesWritten;
  for (;;) {
    uint32_t t0 = micros();
    renderBlock(gBlockBuf);
    uint32_t us = micros() - t0;
    if (us > gRenderMaxUs) gRenderMaxUs = us;         // CPU headroom meter
    i2s_write((i2s_port_t)I2S_NUM_PORT, gBlockBuf,
              sizeof(gBlockBuf), &bytesWritten, portMAX_DELAY);
  }
}

// ====================================================================
//  Encoder — self-healing quarter-step decoder (ISR on A and B)
//  Each detent = 4 quarter-steps. We accumulate quarter-steps and the
//  control task consumes them in groups of 4, KEEPING the remainder, so
//  an occasionally-missed edge heals on the next detent instead of
//  dropping a whole step. Illegal (jumped) transitions count as 0.
// ====================================================================
static const int8_t kQuadTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

static volatile uint8_t gEncPrev = 0x3;   // rest state (both HIGH)
static volatile int     gEncSub  = 0;     // quarter-steps, consumed by control task

static void IRAM_ATTR encISR() {
  uint8_t s = (uint8_t)((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B));
  gEncSub += kQuadTable[((gEncPrev << 2) | s) & 0x0f];
  gEncPrev = s;
}

// Button: 30ms debounce, single event on press (active low).
static bool buttonClicked() {
  static bool lastStable = true;     // pull-up: HIGH = released
  static bool lastRead = true;
  static uint32_t lastChange = 0;
  bool now = digitalRead(PIN_ENC_SW);
  if (now != lastRead) { lastRead = now; lastChange = millis(); }
  if (millis() - lastChange > 30 && now != lastStable) {
    lastStable = now;
    if (now == false) return true;   // just pressed
  }
  return false;
}

// ====================================================================
//  Display + layout + cursor (Core 0)
// ====================================================================
static SH1106Wire display(0x3c, PIN_OLED_SDA, PIN_OLED_SCL);

static const int SCREEN_W = 128, SCREEN_H = 64;
static const int LEFT_MARGIN = 6, COL_W = 23, TOP = 14;

static int gNodeX[NUM_VOICES], gNodeY[NUM_VOICES];
static bool gIsLeaf[NUM_VOICES];              // childless tips (drawn with a leaf)
static bool gTuneMode = false;                // tune mode (click root to enter)
static int  gTunePage = 0;                    // 0 = root note, 1 = tempo
static const float MIN_ROOT = 33.0f, MAX_ROOT = 220.0f;   // root-note range (Hz)

// Cursor targets: a node, or an empty grow-slot belonging to a parent.
enum TargetType { T_NODE, T_SLOT };
struct Target { uint8_t type; int8_t voice; int x, y; };
static Target gTargets[NUM_VOICES * 2];
static int gNumTargets = 0;
static int gCursor = 0;

// keep cursor near this after a structural rebuild
static int8_t gFocusVoice = -1;
static uint8_t gFocusType = T_NODE;

// --- tree layout (recompute only when structure changes) ---
static int gColW = 23;              // adaptive horizontal spacing (per depth)
static int gLeafTotal, gLeafIndex;  // proportional vertical placement

static int countLeaves() {
  int n = 0;
  for (int i = 0; i < NUM_VOICES; i++) {
    if (!gVoices[i].active) continue;
    if (countChildren(i) == 0) n++;
  }
  return n;
}

static int layoutNode(int v) {
  int sum = 0, nc = 0;
  for (int i = 0; i < NUM_VOICES; i++) {
    if (gVoices[i].active && gVoices[i].parent == v) { sum += layoutNode(i); nc++; }
  }
  int top = TOP + 2, bot = SCREEN_H - 2;
  int y;
  if (nc == 0) {
    // leaves spread proportionally across the tree area -> always fits, no overflow
    y = (gLeafTotal > 1) ? top + (gLeafIndex * (bot - top)) / (gLeafTotal - 1)
                         : (top + bot) / 2;
    gLeafIndex++;
  } else {
    y = sum / nc;                              // parents centered on their children
  }
  gIsLeaf[v] = (nc == 0);
  gNodeX[v] = LEFT_MARGIN + gVoices[v].depth * gColW;
  gNodeY[v] = y;
  return y;
}

static void rebuildLayout() {
  // adaptive horizontal spacing: span the full width for the current depth
  int maxDepth = 0;
  for (int i = 0; i < NUM_VOICES; i++)
    if (gVoices[i].active && gVoices[i].depth > maxDepth) maxDepth = gVoices[i].depth;
  gColW = (SCREEN_W - LEFT_MARGIN - 8) / (maxDepth > 0 ? maxDepth : 1);
  if (gColW > 40) gColW = 40;
  if (gColW < 14) gColW = 14;

  gLeafTotal = countLeaves();
  gLeafIndex = 0;
  if (gRootIndex >= 0) layoutNode(gRootIndex);

  // Build cursor target list (nodes + one slot per growable node).
  gNumTargets = 0;
  for (int i = 0; i < NUM_VOICES; i++) {
    if (!gVoices[i].active) continue;
    gTargets[gNumTargets++] = { T_NODE, (int8_t)i, gNodeX[i], gNodeY[i] };
    if (canGrow(i)) {
      int sx = LEFT_MARGIN + (gVoices[i].depth + 1) * gColW;
      if (sx > SCREEN_W - 4) sx = SCREEN_W - 4;
      gTargets[gNumTargets++] = { T_SLOT, (int8_t)i, sx, gNodeY[i] };
    }
  }

  // Spatial order: sort by x, then y (root->tip, top->bottom).
  for (int a = 0; a < gNumTargets - 1; a++)
    for (int b = 0; b < gNumTargets - 1 - a; b++) {
      Target& p = gTargets[b]; Target& q = gTargets[b + 1];
      if (p.x > q.x || (p.x == q.x && p.y > q.y)) { Target t = p; p = q; q = t; }
    }

  // Restore cursor near the focused target.
  int found = -1;
  for (int i = 0; i < gNumTargets; i++)
    if (gTargets[i].voice == gFocusVoice && gTargets[i].type == gFocusType) { found = i; break; }
  if (found >= 0) gCursor = found;
  if (gCursor >= gNumTargets) gCursor = gNumTargets - 1;
  if (gCursor < 0) gCursor = 0;
}

// Nearest note name for a frequency (A4 = 440Hz).
static void noteName(float hz, char* out, int n) {
  static const char* names[12] =
    {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  int m = (int)lroundf(69.0f + 12.0f * log2f(hz / 440.0f));
  snprintf(out, n, "%s%d", names[((m % 12) + 12) % 12], m / 12 - 1);
}

// Transpose the whole tree by a frequency factor. Because every voice is
// root × (product of ratios), one scale factor moves all of them and keeps
// the just-intonation relationships intact.
static void transposeTree(float factor) {
  for (int i = 0; i < NUM_VOICES; i++) {
    if (!gVoices[i].active) continue;
    gVoices[i].freq      *= factor;
    gVoices[i].glideFrom *= factor;
    gVoices[i].glideTo   *= factor;
  }
}

static void drawTuneScreen() {
  display.clear();
  display.setColor(WHITE);
  char big[12], sub[20];

  if (gTunePage == 0) {                          // ---- ROOT NOTE ----
    float hz = gVoices[gRootIndex].freq;
    noteName(hz, big, sizeof(big));
    snprintf(sub, sizeof(sub), "%.1f Hz", hz);
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "ROOT NOTE");
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(SCREEN_W, 0, "1/2");
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 53, "rotate=pitch  click=tempo");
  } else {                                        // ---- TEMPO ----
    int bpm = (int)lroundf(gTempoBase * PULSE_BASE_HZ * 60.0f);
    snprintf(big, sizeof(big), "%d", bpm);
    snprintf(sub, sizeof(sub), "BPM");
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "TEMPO");
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(SCREEN_W, 0, "2/2");
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 53, "rotate=tempo  click=done");
  }

  display.drawHorizontalLine(0, 11, SCREEN_W);
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 16, big);
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 42, sub);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.display();
}

static void drawScreen(int fps) {
  if (gTuneMode && gRootIndex >= 0) { drawTuneScreen(); return; }

  display.clear();
  display.setColor(WHITE);
  Target& cur = gTargets[gCursor];

  // --- status bar: action (with context) + voices/cap, framed ---
  char buf[24];
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  if (cur.type == T_SLOT)
    snprintf(buf, sizeof(buf), "GROW");
  else if (cur.voice == gRootIndex)
    snprintf(buf, sizeof(buf), "ROOT %dHz", (int)gVoices[cur.voice].freq);
  else
    snprintf(buf, sizeof(buf), "PRUNE %u:%u", gVoices[cur.voice].rNum, gVoices[cur.voice].rDen);
  display.drawString(0, 0, buf);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  snprintf(buf, sizeof(buf), "%d/%d", activeCount(), NUM_VOICES);
  display.drawString(SCREEN_W, 0, buf);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawHorizontalLine(0, 11, SCREEN_W);

  // --- trunk stub entering from the left ---
  if (gRootIndex >= 0)
    display.drawLine(0, gNodeY[gRootIndex], gNodeX[gRootIndex], gNodeY[gRootIndex]);

  // --- branches (thicker near the trunk, thin at the tips) ---
  for (int i = 0; i < NUM_VOICES; i++) {
    if (!gVoices[i].active) continue;
    int p = gVoices[i].parent;
    if (p < 0) continue;
    display.drawLine(gNodeX[p], gNodeY[p], gNodeX[i], gNodeY[i]);
    if (gVoices[i].depth <= 1)
      display.drawLine(gNodeX[p], gNodeY[p] + 1, gNodeX[i], gNodeY[i] + 1);
  }

  // --- nodes: small dots; root larger; pulsing ones flash on the beat ---
  for (int i = 0; i < NUM_VOICES; i++) {
    if (!gVoices[i].active) continue;
    int r = (i == gRootIndex) ? 2 : 1;
    if (gVoices[i].pulseDepth > 0.5f) {
      float ph = gVoices[i].pulsePhase, atk = gVoices[i].pulseAtk, pe;
      if (ph < atk) pe = ph / atk;
      else { float q = 1.0f - (ph - atk) / (1.0f - atk); pe = q * q; }
      if (pe > 0.6f) r += 1;                    // brief throb on the beat
    }
    display.fillCircle(gNodeX[i], gNodeY[i], r);

    // a little leaf at the tips (childless nodes, depth 2+)
    if (gIsLeaf[i] && gVoices[i].depth >= 2) {
      int lx = gNodeX[i], ly = gNodeY[i];
      int dir = (i & 1) ? 1 : -1;
      display.drawLine(lx + 1, ly, lx + 4, ly + 3 * dir);
    }
  }

  // --- empty grow-slots drawn as '+' marks ---
  for (int i = 0; i < gNumTargets; i++) {
    if (gTargets[i].type != T_SLOT) continue;
    int x = gTargets[i].x, y = gTargets[i].y;
    display.drawHorizontalLine(x - 2, y, 5);
    display.drawVerticalLine(x, y - 2, 5);
  }

  // --- cursor: blinking bracket around the selection ---
  if ((millis() / 300) & 1)
    display.drawRect(cur.x - 3, cur.y - 3, 7, 7);

  display.display();
}

// ====================================================================
//  Control task (Core 0): encoder, gardening, envelopes, display
// ====================================================================
static void controlTask(void* param) {
  // All I2C lives on this core.
  display.init();
  display.flipScreenVertically();
  Wire.setClock(400000);
  display.setContrast(255);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  delay(2);
  gEncPrev = (uint8_t)((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B));
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encISR, CHANGE);

  rebuildLayout();

  uint32_t lastDrawMs = 0, lastFpsMs = 0, frames = 0;
  uint32_t lastHealMs = 0, lastErodeMs = 0;
  int fps = 0;
  uint32_t lastEnvUs = micros();

  for (;;) {
    uint32_t now = millis();

    // --- envelopes ---
    uint32_t nowUs = micros();
    float dt = (nowUs - lastEnvUs) * 1e-6f;
    lastEnvUs = nowUs;
    if (dt > 0.05f) dt = 0.05f;
    updateEnvelopes(dt);

    // --- encoder rotate: 1 detent (4 quarter-steps) = 1 cursor move ---
    // Consume inside a tiny interrupts-off window so no count is lost to
    // the ISR; keep the remainder so missed edges self-heal.
    noInterrupts();
    int sub = gEncSub;
    int steps = sub / 4;          // truncates toward zero
    gEncSub -= steps * 4;         // keep the remainder
    interrupts();
    if (steps != 0) {
      if (gTuneMode && gTunePage == 0) {
        // transpose the whole tree in semitone steps, clamped to root range
        float f  = powf(2.0f, (float)steps / 12.0f);
        float nr = gVoices[gRootIndex].freq * f;
        if (nr < MIN_ROOT) f = MIN_ROOT / gVoices[gRootIndex].freq;
        if (nr > MAX_ROOT) f = MAX_ROOT / gVoices[gRootIndex].freq;
        transposeTree(f);
      } else if (gTuneMode && gTunePage == 1) {
        // adjust master tempo (multiplicative steps), clamped
        gTempoBase *= powf(2.0f, (float)steps / 12.0f);
        if (gTempoBase < 0.25f) gTempoBase = 0.25f;
        if (gTempoBase > 4.0f)  gTempoBase = 4.0f;
      } else if (gNumTargets > 0) {
        gCursor = (gCursor + steps) % gNumTargets;
        if (gCursor < 0) gCursor += gNumTargets;
        gFocusVoice = gTargets[gCursor].voice;
        gFocusType  = gTargets[gCursor].type;
      }
    }

    // --- click: grow or prune the selected target ---
    if (buttonClicked() && gNumTargets > 0) {
      if (gTuneMode) {
        if (gTunePage == 0) gTunePage = 1;         // root note -> tempo
        else { gTuneMode = false; gTunePage = 0; } // tempo -> exit
      } else {
        Target t = gTargets[gCursor];
        if (t.type == T_SLOT) {
          int nv = growBranch(t.voice);
          if (nv >= 0) { gFocusVoice = nv; gFocusType = T_NODE; }
        } else if (t.voice == gRootIndex) {
          gTuneMode = true; gTunePage = 0;         // click the root to enter tune
        } else {
          gFocusVoice = gVoices[t.voice].parent;   // cursor falls back to parent
          gFocusType  = T_NODE;
          pruneBranch(t.voice);
        }
      }
    }

    // --- rebuild layout only when structure changed ---
    if (gTreeDirty) { gTreeDirty = false; rebuildLayout(); }

    // --- wavetable DNA: erode in-use tables, heal idle ones ---
    if (now - lastHealMs  >= 25)   { lastHealMs  = now; healStep();  }
    if (now - lastErodeMs >= 2500) { lastErodeMs = now; erodeStep(); }

    // --- draw ~60 fps ---
    if (now - lastDrawMs >= 16) {
      lastDrawMs = now;
      drawScreen(fps);
      frames++;
    }
    if (now - lastFpsMs >= 1000) {
      fps = frames; frames = 0; lastFpsMs = now;
      Serial.printf("FPS:%d heap:%u voices:%d render:%uus/5805us mix:%.2f\n",
                    fps, (unsigned)ESP.getFreeHeap(), activeCount(),
                    (unsigned)gRenderMaxUs, gMixGain);
      gRenderMaxUs = 0;
    }

    vTaskDelay(1);   // yield (control task is the only thing on core 0)
  }
}

// ------------------------------ Setup -------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.setTxTimeoutMs(0);
  Serial.println("Harmonic Tree v2 - Step 4: gardening");

  buildWaveTables();
  memcpy(gPristine, gWaveTables, sizeof(gWaveTables));   // pristine 'DNA' for healing
  plantSeed();

  i2sSetup();
  // Audio on CORE 1 (high priority); control+display on CORE 0.
  xTaskCreatePinnedToCore(audioTask,   "audio",   4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
  xTaskCreatePinnedToCore(controlTask, "control", 8192, NULL, 2,                          NULL, 0);

  Serial.println("Planted. Rotate to move cursor, click to grow/prune.");
}

void loop() {
  // Unused: everything runs in the two pinned tasks. Keep core 1's
  // Arduino loopTask idle so it never competes with audio.
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
