# Harmonic Tree

A generative drone / polyrhythm synthesizer for the **ESP32-S3** that you tend
like a garden. A root tone spawns branch voices whose frequencies are integer
**ratio multiples of their parent** (just intonation). You shape the tree's
*structure* by hand with a single encoder; the *behaviour within it* — timbre,
tuning, rhythm, filtering — evolves on its own across several slow timescales.

The result is a living harmonic object: never the same twice, always internally
coherent.

---

## Hardware

| Component | Part | Notes |
|-----------|------|-------|
| MCU | ESP32-S3 DevKitC-1 N16R8 | 16 MB flash, 8 MB OPI PSRAM, dual-core 240 MHz |
| Display | SH1106 1.3" OLED 128×64 | I²C, address `0x3C` |
| DAC | PCM5102 | I²S, SCK tied to GND (self-clock) |
| Encoder | EN2424 24-detent + push button | rests at state `0b11` |

### Wiring (tested, correct for this board)

```
OLED   SDA → GPIO 8     SCL → GPIO 9
DAC    DIN → GPIO 11    BCK → GPIO 12    LCK → GPIO 13     (SCK → GND)
ENC    A   → GPIO 4     B   → GPIO 5     SW  → GPIO 6      (common → GND)

Power: OLED VCC, DAC VIN → 3V3
Gnd:   OLED GND, DAC GND, ENC common, ENC switch → GND
```

> **Note:** Do **not** use GPIO 25/26/27 — they don't exist on this board. (An
> earlier spec listed them for the DAC; the pins above are the corrected, tested
> assignments.)

---

## Build & flash (Arduino IDE 2.x)

**Library:** install **"ESP8266 and ESP32 OLED Driver"** by ThingPulse
(provides `SH1106Wire`). I²S audio uses the built-in legacy `driver/i2s.h` — no
audio library needed.

**Board settings** (Tools menu):

- Board: **ESP32S3 Dev Module**
- PSRAM: **OPI PSRAM**
- USB CDC On Boot: **Enabled** (for the Serial monitor)
- Flash Size: **16MB (128Mb)**
- Port: your `/dev/ttyACM*` (or `ttyUSB*`)

Open `HarmonicTree/HarmonicTree.ino`, **Verify** (✓) to compile, **Upload** (→)
to flash. Serial monitor at **115200** for diagnostics. If upload won't connect:
hold **BOOT**, tap **RESET**, release **BOOT**, hit Upload.

---

## Controls — *gardening*

One encoder. Rotate + single click. No menu.

| Gesture | Action |
|---------|--------|
| **Rotate** | Move the selection cursor through the tree (branches **and** empty grow-slots), in visual order |
| **Click a `+` slot** | **Grow** a new branch here (fades in; ratio chosen by weighted random) |
| **Click a branch** | **Prune** it and all its children (fades out, returns to the pool) |
| **Click the root** | Enter **Tune** mode (root is never pruned) |

The tree starts as just the root — you plant it. It fills to **28 voices**, then
grow-slots stop appearing (the voice budget is full).

### Tune mode (reached by clicking the root)

A two-page contextual control — no menu, just the encoder:

| Page | Rotate | Click |
|------|--------|-------|
| **1 · ROOT NOTE** | Transpose the whole tree in semitones (note name shown) | → page 2 |
| **2 · TEMPO** | Master tempo, shown as BPM (scales all pulse rates together) | Exit |

Transposing keeps every just-intonation relationship intact; tempo keeps the
polyrhythm structure intact.

---

## What's alive in it

The structure is yours; the behaviour evolves autonomously across timescales:

- **Per-voice filters** — root flat, depth 1 low-pass, depths 2–3 band-pass on
  each voice's own ratio frequency, depth 4+ high-pass. Opens as a branch grows,
  closes as it's pruned (emerges *tonally* from darkness, not just in volume).
- **Waveform morphing** — each branch slowly crossfades through the 12 wavetables
  at its own rate; deeper branches reach toward more complex timbres.
- **Polyrhythm pulse** — most branches pulse with a retriggering attack/decay
  envelope; the tempo is **ratio-derived** (a 3:2 branch pulses 3-against-2), so
  the rhythm mirrors the harmony. The root stays a steady drone bed. Pulsing
  nodes throb on the display in time.
- **Ratio-shift glides** — occasionally a branch retunes to a new interval,
  sliding there over 2–4 s; the harmony quietly reorganises itself.
- **Pitch drift** — a tiny per-voice random walk (±cents) turns exact unisons
  into a living, breathing choir.
- **Filter wander** + rare **blooms** — cutoffs breathe slowly; now and then a
  branch flings its filter wide open and eases shut.
- **Seasons** — one very slow master LFO (~3 min) nudges pulse amount, brightness,
  morph speed, erosion and tempo together: the instrument's slow "weather."
- **Wavetable erosion ("DNA decay")** — the shared wavetables slowly smooth/scar
  while in use and **heal back to pristine when idle**. The tree accumulates sonic
  wear as it ages and recovers when you tend (prune) it.

Audio is **stereo** — voices spread across the field, root centred.

---

## Architecture

```
CORE 1 — audio (FreeRTOS task, high priority, IRAM, -O2)
  render 28 voices: wavetable morph → per-voice biquad → pulse → pan
  sum L/R → cubic soft-clip → I²S DMA → PCM5102

CORE 0 — control + display + all I²C
  encoder (quarter-step decoder) · gardening · envelopes
  evolution layers · wavetable erosion/healing · SH1106 draw (~60 fps)
```

Core 1 only reads shared state; Core 0 writes it. New voices publish their fields
before `active = true` (with a barrier); biquad coefficients are double-buffered;
single-float wavetable writes are atomic — so the audio core never reads a
half-built voice or coefficient set.

**Capacity:** the per-voice pipeline costs ~180 µs/voice; the audio block budget
is 5805 µs, so the pool is capped at **28 voices** for guaranteed click-free
playback at full quality. The Serial log reports `render:____us/5805us` so you can
watch the headroom.

---

## Tuning knobs

All near the top of `HarmonicTree.ino`. The `g…` ones are live globals (some are
also driven by Seasons around a base); the `…_BASE` / `…_CHANCE` ones are the
centres/rates. Tune by ear on hardware.

| Constant | Default | Effect |
|----------|---------|--------|
| `NUM_VOICES` | 28 | Voice-pool ceiling (CPU-bound — raise with care) |
| `ROOT_FREQ` | 110 Hz | Starting root note (A2) |
| `BRANCH_DENSITY` | 0.60 | Chance of a third "middle shoot" when growing |
| `GROW_T` / `PRUNE_T` | 1.5 / 1.8 s | Grow / prune fade times |
| `MORPH_LIVELINESS_BASE` | 1.0 | Waveform morph speed |
| `PULSE_AMOUNT_BASE` | 0.70 | Drone ↔ rhythmic balance (0 = pure drone) |
| `PULSE_BASE_HZ` | 1.0 | Base pulse rate (× ratio × tempo) |
| `PULSE_NODE_CHANCE` | 0.70 | Fraction of branches that pulse |
| `gTempoBase` | 1.0 | Master tempo (also set live on the TEMPO page) |
| `WANDER_DEPTH` | 0.35 | Filter-cutoff wander amount |
| `RETUNE_CHANCE_PER_SEC` | 0.018 | How often branches retune (~once/55 s each) |
| `gPitchDrift` | 1.0 | Detune-shimmer amount (cap ±7 cents) |
| `BLOOM_CHANCE_PER_SEC` / `BLOOM_DEPTH` | 0.012 / 3.0 | Bloom frequency / intensity |
| `SEASON_PERIOD_S` | 200 | Length of the slow "weather" cycle |
| `EROSION_BASE` / `HEAL_RATE` | 0.5 / 0.05 | DNA decay rate / idle-table healing rate |

---

## Just-intonation ratios

`2:1 · 3:2 · 4:3 · 5:4 · 7:4 · 6:5 · 9:8 · 11:8` — stable intervals weighted
higher; the occasional `7:4` / `11:8` adds an alien, microtonal colour. Each
branch's frequency is `parent × ratio`.

---

*Built incrementally on hardware, layer by layer. Plant it, tend it, let it grow.* 🌳
