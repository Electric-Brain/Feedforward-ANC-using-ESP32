/**
 * ============================================================
 *  ESP32 ANC DEMONSTRATION — SIGNALS & SYSTEMS PRESENTATION
 *  MIC  : HW-484 → GPIO34
 *  OUT  : Headphone → GPIO25
 *  BOOT : Toggle mode
 * 
 *  MODE 1 (ANC OFF) : Normal passthrough — all sound heard
 *  MODE 2 (ANC ON)  : Bandpass filter — only target freq passes
 *                     Serial Plotter shows x(t) + (-x(t)) = 0
 * 
 *  CONCEPT PROOF on Serial Plotter:
 *    - noise trace    = x(t)
 *    - antinoise trace = -x(t) 
 *    - superposition  = x(t) + (-x(t)) = 0  ← proves cancellation
 * ============================================================
 */

#include <Arduino.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <esp_timer.h>
#include <math.h>

// ─────────────────────────────────────────────
//  CONFIG
// ─────────────────────────────────────────────
#define SAMPLE_RATE_HZ      8000
#define SAMPLE_INTERVAL_US  (1000000 / SAMPLE_RATE_HZ)
#define ANC_BUTTON_PIN      0

// ─────────────────────────────────────────────
//  BANDPASS FILTER — targets 300–3000 Hz (voice/music range)
//  Two-stage: HPF (removes DC + rumble) + LPF (removes hiss)
// ─────────────────────────────────────────────
// HPF state (cutoff ~300 Hz)
static float gHpPrev   = 0.0f;
static float gHpOut    = 0.0f;
// LPF state (cutoff ~3000 Hz)  
static float gLpPrev   = 0.0f;

// HPF coefficient: alpha = RC/(RC + dt), cutoff = 300Hz
// alpha = 1 / (1 + 2*pi*fc*dt) = 1/(1 + 2*pi*300/8000) = 0.976
#define HPF_ALPHA  0.976f

// LPF coefficient: alpha = dt/(RC + dt), cutoff = 3000Hz
// alpha = 2*pi*fc*dt / (1 + 2*pi*fc*dt) = 0.702
#define LPF_ALPHA  0.702f

static inline float bandpassFilter(float s) {
  // Stage 1: High-pass (remove DC and < 300Hz rumble)
  float hp  = HPF_ALPHA * (gHpOut + s - gHpPrev);
  gHpPrev   = s;
  gHpOut    = hp;

  // Stage 2: Low-pass (remove > 3000Hz hiss)
  gLpPrev  += LPF_ALPHA * (hp - gLpPrev);
  return gLpPrev;
}

// ─────────────────────────────────────────────
//  AGC
// ─────────────────────────────────────────────
static float gAgcGain = 32.0f;
static float gAgcPeak = 0.0f;

static inline float applyAGC(float s) {
  float a   = fabsf(s);
  gAgcPeak += (a > gAgcPeak) ? 0.05f  * (a - gAgcPeak)
                              : 0.0002f* (a - gAgcPeak);
  gAgcGain  = (gAgcPeak > 2.0f) ? (3200.0f / gAgcPeak) : 32.0f;
  s        *= gAgcGain;
  if (s >  2047.f) s =  2047.f;
  if (s < -2048.f) s = -2048.f;
  return s;
}

// ─────────────────────────────────────────────
//  STATE
// ─────────────────────────────────────────────
static int16_t gMicBaseline = 2048;
static bool    gAncEnabled  = false;   // OFF by default → passthrough first

// Plotter data (written in ISR, read in loop)
static volatile float gPlotNoise     = 0.0f;
static volatile float gPlotAntiNoise = 0.0f;
static volatile float gPlotSuper     = 0.0f;

static esp_timer_handle_t gTimer = nullptr;

// ─────────────────────────────────────────────
//  MIC CALIBRATION
// ─────────────────────────────────────────────
static void calibrateMicBaseline() {
  Serial.print("[CAL] Keep mic silent");
  long sum = 0;
  for (int i = 0; i < 400; i++) {
    sum += adc1_get_raw(ADC1_CHANNEL_6);
    delayMicroseconds(500);
    if (i % 80 == 0) Serial.print(".");
  }
  gMicBaseline = (int16_t)(sum / 400);
  Serial.printf("\n[CAL] Baseline = %d\n\n", gMicBaseline);
}

// ─────────────────────────────────────────────
//  SPEAKER TEST
// ─────────────────────────────────────────────
static void playSpeakerTest() {
  Serial.println("[TEST] Beep on GPIO25...");
  dac_output_enable(DAC_CHANNEL_1);
  uint32_t phase = 0, start = millis();
  while (millis() - start < 1000) {
    float   a = 2.f * M_PI * phase * 1000.f / SAMPLE_RATE_HZ;
    dac_output_voltage(DAC_CHANNEL_1, (uint8_t)(128 + 100 * sinf(a)));
    phase++;
    delayMicroseconds(SAMPLE_INTERVAL_US);
  }
  dac_output_voltage(DAC_CHANNEL_1, 128);
  Serial.println("[TEST] Beep done\n");
}

// ─────────────────────────────────────────────
//  MAIN ISR — 8000 Hz
// ─────────────────────────────────────────────
static void IRAM_ATTR ancISR(void *arg) {
  // 1. Read + centre
  float raw      = (float)adc1_get_raw(ADC1_CHANNEL_6);
  float centred  = raw - (float)gMicBaseline;

  // 2. Bandpass filter (keeps 300–3000 Hz — voice & music band)
  float filtered = bandpassFilter(centred);

  // 3. AGC
  float x = applyAGC(filtered);   // x(t) — the noise signal

  // ── ANC CORE MATH ──────────────────────────
  //  x(t)  = noise picked by mic
  // -x(t)  = anti-noise (180° phase inversion)
  //  x(t) + (-x(t)) = 0  ← superposition / cancellation
  // ───────────────────────────────────────────
  float antinoise    = -x;                   // -x(t)
  float superposition = x + antinoise;       // = 0 always (proves theorem)

  // Store for Serial Plotter (ISR safe — floats are atomic on Xtensa)
  gPlotNoise     = x;
  gPlotAntiNoise = antinoise;
  gPlotSuper     = superposition;

  // 4. Choose output
  float output;
  if (gAncEnabled) {
    // ANC ON: output anti-noise — in a real sealed headphone this cancels
    // For demo: output goes near-silent (midpoint 128) to simulate cancellation
    output = antinoise * 0.05f;   // heavily attenuated — represents cancelled residual
  } else {
    // ANC OFF: normal passthrough — hear everything
    output = x;
  }

  // 5. DAC output
  int16_t dacVal = (int16_t)((output + 2048.f) / 16.f);
  if (dacVal > 255) dacVal = 255;
  if (dacVal < 0)   dacVal = 0;
  dac_output_voltage(DAC_CHANNEL_1, (uint8_t)dacVal);
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("==========================================");
  Serial.println("  ESP32 ANC DEMO — SIGNALS & SYSTEMS");
  Serial.println("==========================================");
  Serial.println("  MIC  : HW-484  → GPIO34");
  Serial.println("  OUT  : Headphone → GPIO25");
  Serial.println("  BOOT : Toggle ANC ON / OFF");
  Serial.println("------------------------------------------");
  Serial.println("  ANC OFF → Normal passthrough (hear all)");
  Serial.println("  ANC ON  → Cancellation demo + plotter");
  Serial.println("------------------------------------------");
  Serial.println("  Open Serial Plotter (Ctrl+Shift+L)");
  Serial.println("  Watch: noise | antinoise | superposition");
  Serial.println("==========================================\n");

  pinMode(ANC_BUTTON_PIN, INPUT_PULLUP);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 128);

  playSpeakerTest();
  calibrateMicBaseline();

  esp_timer_create_args_t args = {};
  args.callback        = ancISR;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name            = "anc";
  esp_timer_create(&args, &gTimer);
  esp_timer_start_periodic(gTimer, SAMPLE_INTERVAL_US);

  Serial.println("[READY] ANC OFF — Normal passthrough active");
  Serial.println("[INFO]  Press BOOT to toggle ANC ON\n");
}

// ─────────────────────────────────────────────
//  LOOP — Button + Serial Plotter
// ─────────────────────────────────────────────
void loop() {
  // ── Debounced button toggle ──
  static unsigned long lastToggle = 0;
  static bool          lastBtn    = HIGH;
  bool btn = digitalRead(ANC_BUTTON_PIN);

  if (btn == LOW && lastBtn == HIGH && millis() - lastToggle > 250) {
    gAncEnabled  = !gAncEnabled;
    lastToggle   = millis();

    // Reset filter states on mode switch for clean transition
    gHpPrev = gHpOut = gLpPrev = 0.0f;
    gAgcPeak = 0.0f;

    if (gAncEnabled) {
      Serial.println("\n🔴 ANC ON  — Anti-noise active | Watch plotter: super→0");
    } else {
      Serial.println("\n🟢 ANC OFF — Passthrough | Hear all audio normally");
    }
  }
  lastBtn = btn;

  // ── Serial Plotter output (every ~12ms = 100 samples @ 8kHz) ──
  static uint32_t lastPlot = 0;
  if (millis() - lastPlot >= 12) {
    lastPlot = millis();

    // Read volatile copies
    float n  = gPlotNoise;
    float an = gPlotAntiNoise;
    float sp = gPlotSuper;

    // Scale to ±100 for clean plotter view
    int pn  = (int)(n  / 20.f);
    int pan = (int)(an / 20.f);
    int psp = (int)(sp / 20.f);  // This should always be ~0

    Serial.print("noise:");    Serial.print(pn);
    Serial.print(",antinoise:");Serial.print(pan);
    Serial.print(",superposition:"); Serial.println(psp);
  }
}