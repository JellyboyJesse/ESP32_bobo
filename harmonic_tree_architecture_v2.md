# Harmonic Tree — Architecture Document v2
### ESP32-S3 · SH1106 OLED · PCM5102 · Rotary Encoder
### Revised: gardening interaction model + autonomous evolution

> **What changed from v1:** Corrected pin assignments to real board pins.
> Replaced the autonomous-lifecycle + complex menu model with a **gardening**
> interaction (rotate to select, click to grow/prune). Removed the menu system
> entirely for v1. Added **three layers of autonomous audio evolution**.
> Made **dual-core architecture the #1 priority** to fix display refresh.

---

## 1. Concept

A generative drone instrument you **tend like a garden**. A root tone spawns
branch voices whose frequencies are integer ratio multiples of their parent
node. The tree has its own life — branches morph in timbre, occasionally
retune, and breathe with wandering modulation — but **you shape its structure
by hand**, growing and pruning branches with a single encoder.

You plant; it grows in ways you don't fully control. The result is a living
harmonic object that is never quite where you left it.

---

## 2. Hardware

| Component | Part | Notes |
|-----------|------|-------|
| MCU | ESP32-S3 DevKitC-1 N16R8 | 16MB flash, 8MB OPI PSRAM |
| Display | SH1106 1.3" OLED 128×64 blue | I2C, addr 0x3C |
| DAC | PCM5102 PHAT breakout | I2S, SCK tied to GND (self-clock) |
| Encoder | EN2424 24-detent + push | rests at state 0b11 |

### CONFIRMED PIN ASSIGNMENTS (tested working — these are correct)

```
OLED   SDA → GPIO 8     SCL → GPIO 9
DAC    DIN → GPIO 11    BCK → GPIO 12    LCK → GPIO 13
ENC    A   → GPIO 4     B   → GPIO 5     SW  → GPIO 6

Grounds: OLED GND, DAC GND, ENC common, ENC switch  → GND
Power:   OLED VCC, DAC VIN → 3V3
DAC SCK → GND (self-clock mode)
```

> **CRITICAL:** This board does NOT expose GPIO 25/26/27 (those were wrong in
> v1 — they belong to the older ESP32, not the S3). I2S on ESP32-S3 is fully
> remappable, so pins 11/12/13 are used and confirmed working.

---

## 3. System Architecture — DUAL CORE (priority #1)

The single most important structural decision. The v1 single-loop approach is
why the display was unusably slow — audio buffer writes block the display.

```
CORE 1 — audio (dedicated, never blocks on anything else)
────────────────────────────────────────────────────────
  FreeRTOS task pinned to core 1, high priority
  Audio render loop:
    for each voice:
      wavetable lookup (with morph crossfade)
      pitch LFO + drift
      amplitude envelope (grow/prune fades)
      per-depth biquad filter
    sum voices, global soft clip
    i2s_write()  ← this blocks, but ONLY this core

CORE 0 — control + display (stays responsive)
────────────────────────────────────────────────────────
  Main loop:
    read encoder (interrupt-driven, already done)
    handle grow/prune actions
    update tree structure
    advance autonomous evolution (morph, retune, mod drift)
    render display  ← now runs free, no audio blocking it
    target 30+ fps
```

Shared state between cores: the voice array and tree structure. Core 0 writes
structural changes (grow/prune), Core 1 reads them each buffer. Use a simple
mutex or atomic flags around structural edits — keep the critical section tiny.

### Display performance checklist
- I2C at 400kHz (`Wire.setClock(400000)`)
- Audio on core 1 so `i2s_write` never stalls the draw
- Only recompute tree geometry when structure changes, not every frame
- Cache branch screen positions; redraw from cache each frame
- Avoid per-pixel loops where a line/rect primitive works

---

## 4. Audio Engine

### 4.1 Wavetable Synthesis
- 12 wavetables, 2048 samples, float32, built at startup, stored in PSRAM
- Phase accumulator per voice; linear interpolation
- Phase increment = freq / sample_rate * TABLE_SIZE
- Confirmed working: raw `driver/i2s.h`, 44100Hz, 16-bit stereo,
  `I2S_COMM_FORMAT_STAND_I2S`, 8 DMA buffers × 256

### 4.2 The 12 Waveforms (simple → complex)
01 Sine · 02 Sine+ · 03 Triangle · 04 Soft Saw · 05 Square · 06 Sawtooth ·
07 Pulse 25% · 08 Soft Clip · 09 Ratio 3:2 form · 10 Ratio 7:4 form ·
11 Prime series · 12 Inharmonic

Ratio-derived forms (09,10) build their harmonics from the same integers as the
pitch ratio — timbre and pitch as one mathematical object.

### 4.3 Voice Pool
- Max 24–32 voices
- Each voice: phase, wavetable A + B + morph position, amplitude, envelope
  state, pitch LFO phase, depth (0–5), ratio (n/d), parent ID, age,
  current-ratio + target-ratio (for retuning glides)

### 4.4 Harmonic Tree
Ratios (just intonation): 2:1, 3:2, 4:3, 5:4, 7:4, 6:5, 9:8, 11:8.
Branch frequency = parent frequency × ratio. Root default 110Hz (A2).

### 4.5 Filtering
Per-depth biquad filters (one per depth level, not per voice — chosen for
simplicity). Filter cutoff opens on branch birth, closes on prune/death.
Depth mapping: root flat; depth 1 gentle LP; depth 2–3 bandpass on ratio freq;
depth 4+ highpass to thin outer branches. Global soft-clip on the mix.

---

## 5. Gardening Interaction (THE control model)

**One encoder. Rotate and single click. Nothing else.** No long press, no
double click, no menu.

```
ROTATE  → move selection cursor through the tree
          cursor visits: existing branches AND empty grow-slots
          (an empty slot = a place where a new branch could sprout
           from an existing node)

CLICK   → context-aware toggle on the selected position:
            • empty slot   → GROW a new branch here
                             (new voice fades in, ratio chosen by
                              weighted probability)
            • existing branch → PRUNE it and its children
                             (those voices fade out)
```

That's the entire control surface. Learnable in seconds, impossible to get
lost. The cursor is shown on screen as a highlighted/blinking node.

### Grow behaviour
- New branch sprouts from the parent node of the selected slot
- Ratio chosen by weighted random (stable ratios favoured, occasional exotic)
- Voice fades in over ~1–2s, filter opens
- Waveform starts simple (sine-ish), will morph over time on its own

### Prune behaviour
- Selected branch + all its children fade out over ~1–3s
- Voices return to pool
- Cursor moves to nearest remaining branch

### No menu in v1
Root note, speed, breathing mode etc. are baked-in sensible defaults. The
playing experience is pure gardening. Settings access can be added later only
if genuinely needed — the goal is agency, not configurability.

---

## 6. Autonomous Evolution (THREE layers, all "moderate" pace)

The structure is yours; the *behaviour within it* evolves on its own. Timescale
roughly 10s – 2min: clearly perceptible if listening, never jumpy. This is what
keeps the instrument alive between your gardening actions.

### Layer 1 — Waveform morphing
Each branch slowly crossfades through the 12 waveforms rather than being fixed.
- Each voice has waveform A, waveform B, and a morph position 0..1
- Morph position driven by a slow per-voice LFO (each voice slightly different rate)
- When morph completes, target advances to next waveform (weighted by depth —
  deeper branches reach toward more complex forms)
- Crossfade = two table lookups + linear blend (cheap)
- Result: timbre is never static, each branch its own slow evolution

### Layer 2 — Ratio shifting (occasional retuning)
Occasionally a branch retunes to a new interval.
- Low probability per branch per unit time (e.g. a branch might retune
  every 30s–2min on average)
- New ratio chosen by weighted probability (stable favoured)
- Transition is a GLIDE not a jump — frequency slides over ~2–4s
- You hear the harmony reorganise itself smoothly
- Keeps the chord from settling permanently

### Layer 3 — Wandering modulation
The breathing isn't a fixed cycle — it drifts.
- LFO rates and depths slowly random-walk within bounds
- The system gently drifts between unified / wave / chaotic breathing on its
  own (cross-fades between modes, doesn't snap)
- The "weather" of the tree changes over time
- Implemented as slow drunk-walk on the modulation parameters with soft bounds

### Tuning the randomness
All three layers should have a single "liveliness" constant each that's easy to
turn up or down during development. Start moderate, tune by ear on hardware.
Use a seeded PRNG so behaviour is reproducible during debugging, then free-run
in normal use.

---

## 7. Display

Primary view: the tree, with the selection cursor highlighted. Given the
gardening model, the display's job is to show **structure + where the cursor
is**, clearly and responsively.

- Horizontal tree, trunk left, branches spreading right
- Selected node: blinking or brightened
- Empty grow-slots: shown as small hollow markers the cursor can land on
- Branch visual state can hint at its waveform/age (optional, subtle)
- Must run 30+ fps — this is why dual-core matters

Secondary views (waveform scope, ratio info) are optional for v1. Get the tree
view responsive and legible first.

---

## 8. Encoder (confirmed working approach)

- Quadrature state-table decoder, interrupts on BOTH A and B pins
- Detent-based counting: encoder rests at state 0b11 (both HIGH); emit one
  count per detent. Confirmed: exactly one count per physical click.
- Button: simple 30ms debounce, fire once on press
- Use the tested decoder from the integration sketch — don't reinvent it

---

## 9. Libraries

| Library | Purpose |
|---------|---------|
| ThingPulse SH1106Wire | OLED (NOT Adafruit, NOT SSD1306) |
| driver/i2s.h | I2S audio (built into ESP32 core) |

`display.flipScreenVertically()` needed for correct orientation.

---

## 10. Build Order for Claude Code

Step 1 (tone), display basics, and encoder are DONE and confirmed on hardware.
Reuse the tested approaches. Build order from here:

1. **Dual-core refactor FIRST** — move audio to a core-1 task, display to
   core 0. Confirm display is now fast (30+fps) and audio still clean. This is
   the foundation everything else sits on. Test before proceeding.
2. **Wavetable engine** — 12 tables in PSRAM, single root voice playing.
3. **Tree + voice pool** — data structures, ratios, multiple voices.
4. **Gardening interaction** — cursor select (rotate), grow/prune (click),
   with fade in/out. This is the core playable loop — get it feeling good.
5. **Per-depth filters** — biquads, depth mapping, open-on-grow/close-on-prune.
6. **Evolution layer 1** — waveform morphing per voice.
7. **Evolution layer 2** — ratio shifting with glides.
8. **Evolution layer 3** — wandering modulation.
9. **Display polish** — cursor clarity, grow-slot markers, branch state hints.
10. **Tuning** — balance the three liveliness constants by ear; tune fade times,
    glide times, grow/prune feel.

Each step independently testable. Flash and verify on hardware before moving on.

---

## 11. Design priorities (quick reference)

- Dual-core is non-negotiable — it's why v1 felt broken
- Gardening = rotate-select, click-grows-or-prunes, no menu, no defaults exposed
- Three evolution layers keep it alive: morph, retune, wander — all moderate pace
- Autonomous but steerable: you shape structure, it animates behaviour
- Agency was the missing thing, not features. Keep the control surface tiny.

---

*Document version 2.0 — gardening model + autonomous evolution*
