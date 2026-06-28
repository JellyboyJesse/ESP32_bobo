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
static const int      DMA_BUF_COUNT = 8;
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
  float    pulseInc;     // per-sample phase increment = rateHz / SR
  float    pulseAtk;     // attack fraction of the period
  float    pulseDepth;   // per-node participation: 0 = sustain, 1 = pulses
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

static const int NUM_VOICES = 32;
static Voice gVoices[NUM_VOICES];
static int   gRootIndex = -1;

static inline uint32_t freqToInc(float hz) {
  return (uint32_t)((double)hz * 4294967296.0 / (double)SAMPLE_RATE);
}

static inline float voiceNextSample(int vi) {
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

  // rhythmic pulse: retriggering attack/decay envelope (polyrhythm layer).
  // pe = 0..1 within the period; the dial + per-node depth set how deeply
  // it ducks the amplitude. gPulseAmount 0 -> pure sustain (drone).
  v.pulsePhase += v.pulseInc;
  if (v.pulsePhase >= 1.0f) v.pulsePhase -= 1.0f;
  float pe;
  if (v.pulsePhase < v.pulseAtk) {
    pe = v.pulsePhase / v.pulseAtk;                       // attack
  } else {
    float r = 1.0f - (v.pulsePhase - v.pulseAtk) / (1.0f - v.pulseAtk);
    pe = r * r;                                           // decay to 0 by period end
  }
  float k = gPulseAmount * v.pulseDepth;
  float pulseMod = 1.0f - k * (1.0f - pe);                // lerp(sustain, pulse)

  return y * v.env * pulseMod;   // envelope + pulse scale the filtered signal
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
         countChildren(v) < MAX_CHILDREN_PER_NODE;
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
  v.pulseInc   = rateHz / (float)SAMPLE_RATE;
  v.pulseAtk   = 0.03f + (rngNext() % 220) * 0.001f;       // 0.03..0.25 of period
  v.pulsePhase = (rngNext() % 1000) * 0.001f;              // random start phase
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

static void updateEnvelopes(float dt) {
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
      computeVoiceFilter(i);                 // refresh filter (env may have moved)
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

// ----------------------- Mix one audio block (Core 1) ---------------
static void renderBlock(int16_t* out) {
  for (int n = 0; n < BLOCK_FRAMES; n++) {
    float mix = 0.0f;
    for (int v = 0; v < NUM_VOICES; v++) {
      if (!gVoices[v].active) continue;
      mix += voiceNextSample(v);
    }
    // Global soft clip (tanh, architecture §4.5): rounds peaks smoothly
    // instead of hard-clipping them into clicky corners, and gently
    // self-limits as more branches are added. tanh output is in (-1,1)
    // so the int16 conversion can never overflow.
    mix = tanhf(mix * gMixGain);
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

static int16_t gBlockBuf[BLOCK_FRAMES * 2];

static void audioTask(void* param) {
  size_t bytesWritten;
  for (;;) {
    renderBlock(gBlockBuf);
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
static int gLeafStartY, gLeafSpacing, gLeafCursor;

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
  int y;
  if (nc == 0) { y = gLeafStartY + gLeafCursor * gLeafSpacing; gLeafCursor++; }
  else         { y = sum / nc; }
  if (y < TOP + 2)       y = TOP + 2;          // keep nodes on-screen
  if (y > SCREEN_H - 3)  y = SCREEN_H - 3;
  gNodeX[v] = LEFT_MARGIN + gVoices[v].depth * COL_W;
  gNodeY[v] = y;
  return y;
}

static void rebuildLayout() {
  int leaves = countLeaves();
  int usable = SCREEN_H - TOP - 4;
  gLeafSpacing = (leaves > 1) ? usable / leaves : usable / 2;
  if (gLeafSpacing > 14) gLeafSpacing = 14;
  if (gLeafSpacing < 5)  gLeafSpacing = 5;
  int totalH = gLeafSpacing * (leaves > 1 ? leaves - 1 : 0);
  gLeafStartY = TOP + (usable - totalH) / 2 + 2;
  gLeafCursor = 0;
  if (gRootIndex >= 0) layoutNode(gRootIndex);

  // Build cursor target list (nodes + one slot per growable node).
  gNumTargets = 0;
  for (int i = 0; i < NUM_VOICES; i++) {
    if (!gVoices[i].active) continue;
    gTargets[gNumTargets++] = { T_NODE, (int8_t)i, gNodeX[i], gNodeY[i] };
    if (canGrow(i)) {
      int sx = LEFT_MARGIN + (gVoices[i].depth + 1) * COL_W;
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

static void drawScreen(int fps) {
  display.clear();
  display.setColor(WHITE);
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  // status bar
  Target& cur = gTargets[gCursor];
  const char* action = (cur.type == T_SLOT) ? "GROW"
                       : (cur.voice == gRootIndex ? "ROOT" : "PRUNE");
  display.drawString(0, 0, action);
  char buf[20];
  snprintf(buf, sizeof(buf), "%d voices", activeCount());
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(SCREEN_W, 0, buf);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  // edges (parent -> child)
  for (int i = 0; i < NUM_VOICES; i++) {
    if (!gVoices[i].active) continue;
    int p = gVoices[i].parent;
    if (p < 0) continue;
    display.drawLine(gNodeX[p], gNodeY[p], gNodeX[i], gNodeY[i]);
    if (gVoices[i].depth <= 1)   // thicken trunk/primary branches
      display.drawLine(gNodeX[p], gNodeY[p] + 1, gNodeX[i], gNodeY[i] + 1);
  }

  // nodes: pulsing ones throb with their rhythm; others size with envelope
  for (int i = 0; i < NUM_VOICES; i++) {
    if (!gVoices[i].active) continue;
    int r;
    if (gVoices[i].pulseDepth > 0.5f) {
      float ph = gVoices[i].pulsePhase;          // racy read, fine for a visual
      float atk = gVoices[i].pulseAtk;
      float pe;
      if (ph < atk) pe = ph / atk;
      else { float q = 1.0f - (ph - atk) / (1.0f - atk); pe = q * q; }
      r = 1 + (int)lroundf(pe * 2.0f);           // 1..3, throbs in time
    } else {
      r = 1 + (int)lroundf(gVoices[i].env);      // 1..2
    }
    display.fillCircle(gNodeX[i], gNodeY[i], r);
  }

  // empty grow-slots (hollow markers)
  for (int i = 0; i < gNumTargets; i++)
    if (gTargets[i].type == T_SLOT)
      display.drawCircle(gTargets[i].x, gTargets[i].y, 2);

  // cursor: blinking box around the selected target
  if ((millis() / 350) & 1) {
    int cx = cur.x, cy = cur.y;
    display.drawRect(cx - 4, cy - 4, 9, 9);
  }

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
    if (steps != 0 && gNumTargets > 0) {
      gCursor = (gCursor + steps) % gNumTargets;
      if (gCursor < 0) gCursor += gNumTargets;
      gFocusVoice = gTargets[gCursor].voice;
      gFocusType  = gTargets[gCursor].type;
    }

    // --- click: grow or prune the selected target ---
    if (buttonClicked() && gNumTargets > 0) {
      Target t = gTargets[gCursor];
      if (t.type == T_SLOT) {
        int nv = growBranch(t.voice);
        if (nv >= 0) { gFocusVoice = nv; gFocusType = T_NODE; }
      } else if (t.voice != gRootIndex) {
        gFocusVoice = gVoices[t.voice].parent;   // cursor falls back to parent
        gFocusType  = T_NODE;
        pruneBranch(t.voice);
      }
    }

    // --- rebuild layout only when structure changed ---
    if (gTreeDirty) { gTreeDirty = false; rebuildLayout(); }

    // --- draw ~60 fps ---
    if (now - lastDrawMs >= 16) {
      lastDrawMs = now;
      drawScreen(fps);
      frames++;
    }
    if (now - lastFpsMs >= 1000) {
      fps = frames; frames = 0; lastFpsMs = now;
      Serial.printf("FPS:%d heap:%u voices:%d targets:%d cursor:%d\n",
                    fps, (unsigned)ESP.getFreeHeap(), activeCount(), gNumTargets, gCursor);
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
