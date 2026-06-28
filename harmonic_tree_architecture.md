> **HARDWARE CORRECTIONS — addendum (not part of original v1.0 spec text)**
>
> The DAC pin assignments in §2 below are stale. GPIO 25/26/27 do **not**
> exist on this ESP32-S3 DevKitC-1 N16R8 board. The real, tested wiring is:
>
> | Signal | Doc §2 (wrong) | Tested (use these) |
> |--------|----------------|--------------------|
> | DAC DIN | GPIO25 | **GPIO11** |
> | DAC BCK | GPIO26 | **GPIO12** |
> | DAC LCK | GPIO27 | **GPIO13** |
> | DAC SCK | — | tied to GND |
>
> OLED (SDA→GPIO8, SCL→GPIO9) and Encoder (A→GPIO4, B→GPIO5, SW→GPIO6)
> are correct as written. The code uses the tested pins above.
>
> ---

# Harmonic Tree — Architecture Document
### ESP32-S3 · SH1106 OLED · PCM5102 · Rotary Encoder
---
## 1. Concept
A generative drone instrument built around a harmonic tree. A root tone spawns branch voices whose frequencies are integer ratio multiples of their parent node — not the root. Branches are born, mature, and die autonomously. The tree grows, reaches density, then collapses back to silence and regrows. The single encoder controls the global filter sweep in performance mode and navigates the menu system otherwise.
The sound is a living harmonic object — never the same twice, always internally coherent.
---
## 2. Hardware
| Component | Part | Connection |
|-----------|------|------------|
| MCU | ESP32-S3 DevKitC-1 N16R8 | — |
| Display | SH1106 1.3" OLED 128×64 blue | SDA→GPIO8, SCL→GPIO9 |
| DAC | PCM5102 PHAT breakout | BCK→GPIO26, DIN→GPIO25, LCK→GPIO27 |
| Encoder | EN2424-6T25 24ppr with push | A→GPIO4, B→GPIO5, SW→GPIO6 |
**Key specs:**
- ESP32-S3: dual-core LX7 240MHz, 8MB PSRAM (octal), 16MB flash
- PCM5102: I2S, 32-bit, up to 384kHz, 3.5mm jack onboard
- OLED: I2C 0x3C, 1-bit, 128×64px, blue phosphor
---
## 3. System Architecture
```
CORE 0 — control                    CORE 1 — audio (dedicated)
─────────────────────               ──────────────────────────
Encoder ISR                         I2S DMA interrupt
Menu state machine                  Audio render loop:
Display driver (I2C)                  for each voice:
Tree lifecycle manager                  wavetable lookup
Modulation parameter updates            amplitude envelope
                                        per-depth biquad filter
         shared memory                pitch LFO
    (double-buffered params)          amp LFO
                                    sum all voices
                                    global biquad filter
                                    soft clip
                                    → DMA buffer → PCM5102
```
Core 1 never calls malloc, never touches I2C, never blocks. Audio is interrupt-driven via I2S DMA. Core 0 updates shared parameter structs; Core 1 reads them atomically at buffer boundaries.
---
## 4. Audio Engine
### 4.1 Wavetable Synthesis
All oscillators use wavetable lookup — no sin() at runtime. Tables pre-computed at startup and stored in PSRAM.
- Table size: 2048 samples, float32 → 8KB per table
- Phase accumulator: uint32, wraps naturally
- Phase increment: `(freq / sample_rate) * TABLE_SIZE`
- Interpolation: linear between adjacent samples
12 waveforms in the bank, organised by harmonic complexity:
| # | Name | Harmonics | Tree depth |
|---|------|-----------|------------|
| 01 | Sine | 1 only | Root |
| 02 | Sine+ | 1, 2 subtle | Root |
| 03 | Triangle | Odd, 1/n² | Depth 1 |
| 04 | Soft Saw | All, 1/n² | Depth 1 |
| 05 | Square | Odd, 1/n | Depth 2 |
| 06 | Sawtooth | All, 1/n | Depth 2 |
| 07 | Pulse 25% | All, duty-shaped | Depth 3 |
| 08 | Soft Clip | All, tanh-shaped | Depth 3 |
| 09 | 3:2 Form | 3rd + 2nd | Depth 3–4 |
| 10 | 7:4 Form | 7th + 4th | Depth 4 |
| 11 | Prime Series | 2,3,5,7,11,13 | Outer branches |
| 12 | Inharmonic | ×1.0, ×2.756, ×4.1 | Outer leaves |
**Ratio-derived waveforms (09, 10):** The waveform is constructed from the same integers as the pitch ratio. A 3:2 voice has harmonics at positions 3 and 2. Timbre and pitch become the same mathematical object.
**Waveform assignment per depth:**
- Depth 0 (root): 01 Sine — pure, unchanging
- Depth 1: morphs 02 → 03 → 04 over voice lifetime
- Depth 2: morphs 05 → 06 over voice lifetime
- Depth 3: morphs 07 → 09 → 10 over voice lifetime
- Depth 4+: 11 Prime Series or 12 Inharmonic
Morph speed is tied to growth speed parameter.
### 4.2 Voice Pool
- Maximum voices: 32 (practical ceiling with full filter + modulation pipeline)
- Voice stealing: steal quietest dying voice if pool exhausted
- Each voice carries:
  - Phase accumulator (uint32)
  - Wavetable index A and B + morph position (for crossfading)
  - Amplitude (float, 0..1)
  - Birth/death envelope state
  - Pitch LFO phase (float)
  - Amp LFO phase (float)
  - Depth index (uint8, 0..5)
  - Ratio (numerator, denominator)
  - Parent voice ID
### 4.3 Harmonic Tree Structure
Ratios used (just intonation):
| Ratio | Interval | Character |
|-------|----------|-----------|
| 2:1 | Octave | Stable |
| 3:2 | Perfect fifth | Open, hollow |
| 4:3 | Perfect fourth | Ancient, stable |
| 5:4 | Major third | Warm |
| 7:4 | Subminor seventh | Alien, no Western equivalent |
| 6:5 | Minor third | Dark |
| 9:8 | Major second | Tense |
| 11:8 | Augmented fourth | Unsettled |
Each branch frequency = parent frequency × (ratio n/d).
Depth 0 root = root note (default 110Hz, A2).
Branch voices spawn from parent nodes, not always from root.
**Branching probability:** at each depth, two children always spawn. A third middle shoot spawns with configurable probability (default 60%). This is the branch density parameter.
### 4.4 Filtering
**Architecture:** per-depth biquad filters, one global output filter.
Biquad filter — second-order IIR, 5 coefficients (b0, b1, b2, a1, a2), 2 state values per instance. Fixed-point implementation.
**Per-depth filter mapping:**
| Depth | Filter type | Character |
|-------|-------------|-----------|
| 0 (root) | Flat / bypass | Uncoloured |
| 1 | Lowpass, high cutoff | Gentle warmth |
| 2 | Bandpass, centred on ratio freq | Focused, present |
| 3 | Bandpass, narrower Q | More defined |
| 4+ | Highpass | Thins outer branches |
Filter cutoff opens on voice birth and closes on voice death — the voice emerges from and returns to darkness tonally, not just in amplitude.
**Global filter:** resonant lowpass on the final mix. This is the primary encoder destination in performance mode. Slow sweep across the harmonic series causes individual partials to be amplified as cutoff passes through them — audible spectral blooming.
**Filter spread macro:** single parameter that increases per-depth differentiation. At 0 all depths sound similar. At 100 root is dark, outer branches are bright and focused.
### 4.5 Signal Flow
```
Voice 1..N:
  wavetable oscillator
  → wavetable morph (crossfade A→B)
  → pitch LFO (±0.3 cents max)
  → amplitude envelope (birth/death)
  → amp LFO (±8% max)
  → per-depth biquad filter
  → contribute to mix
Mix:
  sum all voices (normalised by voice count)
  → global resonant lowpass (encoder controlled)
  → soft clip tanh (prevents DAC overload, adds warmth)
  → I2S DMA buffer
  → PCM5102 → 3.5mm output
```
---
## 5. Modulation
### 5.1 LFO Implementation
All LFOs are wavetable sine oscillators running at control rate (every 64 samples). Same table infrastructure as audio oscillators — zero extra code.
Control rate update = 44100 / 64 ≈ 689Hz. More than adequate for any modulation below ~100Hz.
### 5.2 Modulation Sources
**Master LFO:** 0.01–2Hz (set by LFO Rate parameter), drives overall system breathing.
**Per-depth LFO:** each depth has its own LFO phase. Rate increases with depth:
- Depth 0: master_rate × 1.0
- Depth 1: master_rate × 1.4
- Depth 2: master_rate × 2.0
- Depth 3: master_rate × 2.8
- Depth 4: master_rate × 3.8
Outer branches have faster metabolic rate than the root.
**Per-voice LFOs:** pitch LFO and amp LFO, each with unique phase offset seeded from voice ID.
### 5.3 Breathing Modes
Three modes, cycled via long press on encoder:
**Unified breathing:**
All depth LFOs at the same phase. The whole tree inhales and exhales together.
```
depth_phase[n] = master_lfo_phase
```
**Wave breathing:**
Each depth offset by 72° (360°/5). Motion propagates root → outer branches in a slow wave.
```
depth_phase[n] = master_lfo_phase + n × 72°
```
**Chaotic motion:**
Cross-modulated LFOs. Each depth's LFO rate is modulated by the output of the adjacent depth's LFO. Chaos emerges from interdependency, not from random noise — consistent with the tree's relational harmonic logic.
```
depth[0].rate = base_rate
depth[n].rate = base_rate + depth[n-1].output × chaos_amount
```
Chaos amount (0–100) controls feedback intensity. At 0 = five independent LFOs. At 100 = complex emergent behaviour.
### 5.4 Modulation Destinations
| Source | Destination | Depth |
|--------|-------------|-------|
| Per-depth LFO | Filter cutoff (per depth) | ±20% of cutoff |
| Per-depth LFO | Filter Q (mid depths) | ±15% |
| Per-voice pitch LFO | Voice frequency | ±0.3 cents |
| Per-voice amp LFO | Voice amplitude | ±8% |
| Birth envelope | Filter cutoff | Closed → open over 2–4s |
| Death envelope | Filter cutoff + amplitude | Open → closed over 1–3s |
| Encoder (performance) | Global filter cutoff | 20Hz → 8kHz |
| Encoder (menu) | Selected parameter | — |
---
## 6. Tree Lifecycle
Six stages, autonomous progression:
| Stage | Duration | Max depth | Character |
|-------|----------|-----------|-----------|
| Seed | 2s | 1 | Root alone, silence |
| Sapling | 3s | 2 | First branches appear |
| Growing | 3.5s | 3 | Tree filling out |
| Mature | 5s | 4 | Full and stable |
| Dense | 6s | 5 | Maximum complexity |
| Dying | 4.5s | 5 | Branches dissolving from tips |
After Dying, resets to Seed and regrows. Tree shape is deterministic (seeded PRNG) — same structure each growth cycle, but modulation and voice timing ensure it never sounds identical.
Growth speed parameter scales all durations. At 50% everything takes twice as long. Waveform morph speed is tied to this same parameter.
Voice birth/death within stages: voices fade in over ~80 frames on birth, fade out over ~120 frames on death.
---
## 7. Display
### 7.1 Three Visual Panels
Cycled by single press on encoder:
**Tree view (default):**
- Horizontal fractal tree, trunk enters left at y=32
- Segments drawn left to right, trunk 3px, primary branches 2px, rest 1px
- Leaves at tips from depth 2+, fall during dying stage
- Status bar: stage name | segment count | max depth
**Waveform view:**
- Composite waveform of all live voices mixed
- Scrolls continuously
- Complexity visible as more voices add harmonic content
**Ratio info view:**
- Cycles through active voices every ~3.5s
- Shows: ratio fraction, calculated frequency, waveform type assigned
- Miniature waveform preview strip in bordered box
### 7.2 Menu Display
Full screen takeover. Tree continues running and audio continues — display only changes.
```
┌────────────────────────────┐
│ MODULATION                 │
├────────────────────────────┤
│ ► TYPE        BREATHING    │
│   LFO RATE    ████░░  64   │
│   PITCH DRIFT ██░░░░  32   │
└────────────────────────────┘
```
Selected row: `►` marker, inverted (white bg, black text).
Bar graph for 0–100 params (6 segments).
Text cycling for type params.
1 second idle timeout → returns to tree view automatically.
---
## 8. Menu Structure
```
SOUND
  ├── Volume           (0–100)
  └── Growth Speed     (0–100, also drives waveform morph speed)
TREE
  ├── Root Note        (A0–A4, or Hz: 27.5–440)
  └── Branch Density   (0–100, probability of third middle shoot)
MODULATION
  ├── Type             (BREATHING / UNIFIED / CHAOS)
  ├── LFO Rate         (0–100)
  └── Pitch Drift      (0–100)
ADVANCED  ──────────────── (further folder, third level)
  ├── Filter Resonance (0–100, global filter Q)
  ├── Filter Spread    (0–100, per-depth differentiation)
  ├── Max Depth        (1–5, caps tree complexity)
  ├── Chaos Amount     (0–100, cross-mod feedback in chaotic mode)
  ├── Display View     (TREE / WAVE / RATIO)
  └── Brightness       (0–100, OLED contrast)
```
---
## 9. Encoder Interaction Map
```
PERFORMANCE MODE (default)
  Rotate              → global filter cutoff sweep (20Hz–8kHz)
  Single press        → cycle visual panel (tree → wave → ratio → tree)
  Long press          → cycle breathing mode (unified → wave → chaos)
  Double press        → enter menu
MENU — LEVEL 1 (categories)
  Rotate              → scroll through categories
  Single press        → enter category
  Double press        → exit menu → return to performance
MENU — LEVEL 2 (parameters)
  Rotate              → scroll through parameters
  Single press        → select parameter for editing
  Double press        → back to level 1
MENU — EDIT STATE
  Rotate              → change value
  Single press        → confirm, return to level 2
  Double press        → cancel, return to level 2
ALL MENU STATES
  1 second idle       → close menu, return to tree view
```
---
## 10. Libraries Required
| Library | Purpose | Install |
|---------|---------|---------|
| ThingPulse SH1106Wire | OLED display driver | Arduino Library Manager: "ESP8266 and ESP32 OLED Driver" |
| Arduino ESP32 I2S | I2S audio output | Built into ESP32 Arduino core |
No audio library required — synthesis is implemented from scratch.
---
## 11. Build Order for Claude Code
Suggested incremental build sequence:
1. **I2S tone test** — single sine wave through PCM5102, verify hardware wiring
2. **Wavetable engine** — 12 tables generated at startup, single voice playing root
3. **Tree builder** — iterative branch generator, voice pool, ratio assignments
4. **Voice lifecycle** — birth/death envelopes, amplitude management, stage machine
5. **Per-depth filters** — biquad implementation, depth-to-filter mapping
6. **Modulation** — per-voice LFOs, per-depth LFOs, three breathing modes
7. **Global filter** — output stage, encoder control
8. **Display** — three views, pixel engine, status bar
9. **Menu system** — encoder state machine, two-level navigation, edit mode
10. **Integration + tuning** — balance levels, tune LFO rates, adjust stage durations
Each step is independently testable before moving to the next.
---
*Document version 1.0 — ready for Claude Code*
