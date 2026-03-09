# Auto-Tune Analysis & Improvement Plan

## Current Implementation Summary

The auto-tune uses a **Ziegler-Nichols relay method**: oscillate the system around a setpoint by toggling heater output between `bias + amplitude` and `bias - amplitude`, measure the resulting oscillation period (Pu) and amplitude (a), then compute PID gains from the ultimate gain (Ku) and period.

**State machine:** IDLE → HEATING → STABILIZING → RUNNING → ANALYZING → COMPLETE/FAILED

**Current config:** 50% bias, ±25% amplitude swing, 1.0°C relay hysteresis, 8 oscillation steps, 45s stabilization, 30-minute max duration.

---

## Identified Weaknesses

### 1. Peak/Valley Recorded at Crossing, Not Extremum (Critical)

**File:** `autotune.cpp:412-434`

When the temperature crosses the setpoint ± hysteresis, the *current* temperature is recorded as the peak or valley. But the actual extremum (maximum or minimum) occurred *before* the crossing — the temperature was still rising/falling past the hysteresis boundary.

**Impact:** Systematic underestimation of oscillation amplitude → overestimation of Ku → overly aggressive PID gains. The error increases with thermal lag.

**Fix:** Track the running max/min temperature between crossings. Record the extremum when the relay toggles, not the crossing temperature.

```cpp
// Track running extremum between relay toggles
static float autoTuneRunningMax = -1e9f;
static float autoTuneRunningMin = 1e9f;

// In RUNNING phase, continuously update:
if (autoTuneOutputHigh) {
    if (state.beanTemperature > autoTuneRunningMax)
        autoTuneRunningMax = state.beanTemperature;
} else {
    if (state.beanTemperature < autoTuneRunningMin)
        autoTuneRunningMin = state.beanTemperature;
}

// On toggle high→low: record autoTuneRunningMax as peak, reset to -1e9
// On toggle low→high: record autoTuneRunningMin as valley, reset to 1e9
```

### 2. No Oscillation Consistency Validation (Critical)

**File:** `autotune.cpp:467-550`

The analysis averages *all* collected peaks/valleys with no consistency check. Industry standard is to validate that the last 3 peaks are within 5% of each other, confirming the system has reached a limit cycle. Without this:
- Early transient oscillations (not yet at steady-state) corrupt the average
- A drifting setpoint or changing thermal conditions go undetected

**Fix:** Discard the first 2 oscillation cycles as transient. Validate that the last 3 peak-to-peak amplitudes are within 10% of each other before accepting results.

```cpp
// After collecting peaks, validate consistency:
if (autoTunePeakCount >= 4) {
    float lastThreeAmps[3];
    for (int i = 0; i < 3; i++) {
        int pi = autoTunePeakCount - 3 + i;
        // Find nearest valley for each peak
        lastThreeAmps[i] = autoTunePeaks[pi].temperature - nearestValley(pi);
    }
    float mean = (lastThreeAmps[0] + lastThreeAmps[1] + lastThreeAmps[2]) / 3.0f;
    for (int i = 0; i < 3; i++) {
        if (fabs(lastThreeAmps[i] - mean) / mean > 0.10f) {
            // Oscillation not converged - continue or fail
        }
    }
}
```

### 3. Overly Permissive Fallback (High)

**File:** `autotune.cpp:497-526`

When primary peak/valley analysis fails, the fallback uses a 20-sample temperature history buffer with extremely loose thresholds: `period > 0.5s` and `amplitude > 0.2°C`. These accept sensor noise as valid oscillation data.

**Impact:** The fallback can produce PID parameters from noise rather than actual system dynamics, resulting in unpredictable control behavior.

**Fix:**
- Minimum period should be ≥ 2× the expected thermal time constant (~20s for small roasters)
- Minimum amplitude should be ≥ 2× sensor noise floor (~1.0°C for MAX6675)
- If fallback criteria aren't met, fail honestly rather than returning garbage

```cpp
// Tightened fallback thresholds
computed = (avgPeriod > 10.0f && avgAmplitude > 1.0f);
```

### 4. Z-N Classic Formulas Are Overly Aggressive (High)

**File:** `autotune.cpp:536-539`

Ziegler-Nichols classic tuning rules (Kp = 0.6·Ku, Ti = Pu/2, Td = Pu/8) were designed for quarter-decay-ratio response in 1942 — they produce ~25% overshoot by design. For a coffee roaster where temperature overshoot can ruin a batch, this is problematic.

**Better alternatives (same Ku/Pu inputs, no additional measurement needed):**

| Method | Kp | Ti | Td | Overshoot |
|--------|-----|------|------|-----------|
| Z-N Classic | 0.6·Ku | Pu/2 | Pu/8 | ~25% |
| Tyreus-Luyben | Ku/3.2 | 2.2·Pu | Pu/6.3 | ~5% |
| Z-N Some Overshoot | 0.33·Ku | Pu/2 | Pu/3 | ~10% |
| Z-N No Overshoot | 0.2·Ku | Pu/2 | Pu/3 | ~0% |

**Recommendation:** Default to Tyreus-Luyben. Optionally let the user choose aggressiveness via MQTT parameter. Tyreus-Luyben is the standard conservative alternative used in industrial process control.

```cpp
// Tyreus-Luyben (less aggressive, less overshoot)
autoTuneRecommendedKp = Ku / 3.2f;
autoTuneRecommendedKi = autoTuneRecommendedKp / (2.2f * avgPeriod);
autoTuneRecommendedKd = autoTuneRecommendedKp * avgPeriod / 6.3f;
```

### 5. No Noise Filtering on Temperature Input (Medium)

**File:** `autotune.cpp:281-289`

Raw `state.beanTemperature` is used directly in the relay comparisons and peak detection. MAX6675 has ±1.5°C typical noise, which can cause:
- Premature relay toggling (if noise exceeds hysteresis)
- False peak/valley detection
- Noisy period measurements

**Fix:** Apply a simple exponential moving average (EMA) filter to the temperature used *only* for auto-tune decisions. Don't modify the actual temperature reading — just filter the auto-tune's view.

```cpp
static float autoTuneFilteredTemp = 0;
static bool autoTuneFilterInitialized = false;
const float alpha = 0.3f;  // Lower = more smoothing

if (!autoTuneFilterInitialized) {
    autoTuneFilteredTemp = state.beanTemperature;
    autoTuneFilterInitialized = true;
} else {
    autoTuneFilteredTemp = alpha * state.beanTemperature + (1.0f - alpha) * autoTuneFilteredTemp;
}
// Use autoTuneFilteredTemp for all relay comparisons and peak detection
```

### 6. No Asymmetry Correction (Medium)

The heater-only system has asymmetric dynamics: heating is active (heater on), but cooling is passive (ambient dissipation only). The relay method assumes symmetric response, which biases the measured Ku.

**Impact:** The ultimate gain calculation uses `outputSwing * 4 / (amplitude * π)`, which assumes equal heating and cooling power. In reality, cooling is much slower, making the measured amplitude larger than it would be with symmetric actuation. This makes Ku appear lower and the resulting PID gains less aggressive than optimal.

**Fix for current architecture:** Measure heating half-period and cooling half-period separately. Apply the asymmetric relay correction factor:

```
Ku_corrected = (4 * d) / (π * a) * sqrt(1 + (d_h/d_c)²) / sqrt(2)
```

where `d` is the relay amplitude, `a` is the oscillation amplitude, `d_h` is heating half-cycle output, `d_c` is cooling half-cycle output.

**Simpler alternative:** Since fan provides active cooling, consider using asymmetric relay output — increase fan speed during "cool" phase to make the cooling rate closer to the heating rate.

### 7. HEATING Phase Has Too Many Transition Pathways (Low)

**File:** `autotune.cpp:291-386`

The HEATING state has 5 different ways to transition to STABILIZING:
1. Within tolerance + time-stable (line 329)
2. Within tolerance + RoR-stable (line 327-328)
3. Within fast-track band + low RoR (line 350-354)
4. Equilibrium detection at any temp ≥120°C (line 366-374)
5. Heating timeout force (line 377-385)

While each pathway has rationale, the complexity makes behavior hard to predict and debug. The equilibrium detection (#4) and timeout (#5) can accept wildly different operating points than the requested target temperature.

**Fix:** Simplify to 3 paths:
1. **Normal:** Reach setpoint ± tolerance with RoR < threshold for N seconds
2. **Equilibrium:** If temperature stabilizes (RoR ≈ 0 for 30s) at ≥75% of target, accept that as operating point
3. **Timeout:** After max heating time, fail (don't silently accept a bad operating point)

### 8. Temperature History Buffer Too Small (Low)

**File:** `autotune.cpp:43-45`

The 20-sample circular buffer at ~1Hz gives only 20 seconds of history. With oscillation periods potentially 30-60 seconds for a thermal system, the buffer can't even hold one full cycle for the fallback analysis.

**Fix:** Increase to 120 samples (2 minutes of history at 1Hz). This is only ~960 bytes — trivial on ESP32.

---

## Improvement Roadmap

### Phase 1: Quick Wins (Same Algorithm, Better Execution)

These changes improve accuracy without changing the fundamental approach:

1. **Track actual extrema** instead of crossing temperatures
2. **Apply EMA filter** (α=0.3) to auto-tune temperature readings
3. **Switch to Tyreus-Luyben** tuning formulas (drop-in replacement)
4. **Tighten fallback thresholds** (period > 10s, amplitude > 1.0°C)
5. **Add oscillation consistency check** (last 3 amplitudes within 10%)
6. **Increase temp history buffer** to 120 samples
7. **Simplify HEATING transitions** to 3 clear pathways
8. **Report quality metrics** in results (consistency %, asymmetry ratio)

**Estimated effort:** 2-3 focused sessions. No MQTT protocol changes needed.

### Phase 2: Tuning Method Selection

Add support for multiple tuning formulas, selectable via MQTT start command:

```json
{
    "target_temperature": 200.0,
    "tuning_method": "tyreus_luyben"
}
```

Supported methods:
- `zn_classic` — Ziegler-Nichols classic (aggressive, quarter-decay)
- `tyreus_luyben` — Tyreus-Luyben (conservative, default)
- `zn_some_overshoot` — Z-N modified for ~10% overshoot
- `zn_no_overshoot` — Z-N modified for minimal overshoot

**Estimated effort:** 1 session. Backward-compatible (defaults to `tyreus_luyben`).

### Phase 3: Open-Loop Step Response Method (Alternative to Relay)

Add a second auto-tune mode: **step response identification**. Instead of oscillating, apply a step change in heater output and measure the response curve to identify the FOPDT (First Order Plus Dead Time) model parameters:
- **K** (process gain): how much temperature changes per unit of heater change
- **τ** (time constant): how fast the system responds
- **θ** (dead time): delay before the system starts responding

From K, τ, θ, compute PID gains using **SIMC** (Skogestad's Internal Model Control):
```
Kp = (1/K) * τ / (τc + θ)
Ti = min(τ, 4*(τc + θ))
Td = 0.5 * θ
```
where τc is the desired closed-loop time constant (tunable aggressiveness parameter).

**Advantages over relay method:**
- No oscillation → safer (temperature only rises, then falls once)
- Faster (one step response vs. 8 oscillation cycles)
- Provides a process model, not just PID gains
- SIMC tuning is more robust than Z-N for thermal systems

**Estimated effort:** 3-4 sessions. New state machine states (STEP_UP, STEP_SETTLE, STEP_ANALYZE).

### Phase 4: Adaptive Gain Scheduling (Future/Generic Platform)

For the generic temperature control platform vision, different devices and temperature ranges need different PID gains. Gain scheduling runs auto-tune at multiple operating points and interpolates:

- Coffee roaster: tune at 150°C (drying), 200°C (development) — endothermic→exothermic transition
- Smoker: tune at 110°C (low-and-slow), 150°C (hot-and-fast)
- Kiln: tune at multiple ramp points across 100-1200°C

Store multiple PID parameter sets in NVS, switch based on current temperature zone.

**Estimated effort:** Significant. Best deferred to the generic platform refactor.

---

## Recommended Action Plan

**Immediate (before generic platform work):**
1. Implement Phase 1 quick wins — these are pure improvements with no API changes
2. Implement Phase 2 tuning method selection — minimal effort, high value

**During generic platform refactor:**
3. Implement Phase 3 step response method as an alternative auto-tune mode
4. Design gain scheduling infrastructure for Phase 4

**Key principle:** The auto-tune stays on the ESP32. Network latency and reliability make server-side relay control unreliable for accurate oscillation measurement. The server's role is to *initiate*, *monitor*, and *store results* — not to run the control loop.

---

## Appendix: Describing Function Error in Relay Auto-Tune

The relay method uses a describing function approximation to estimate Ku from the relay output amplitude and measured oscillation amplitude. This approximation assumes:
- Pure sinusoidal oscillation (real thermal systems are more triangular)
- Symmetric relay (heating ≠ cooling in heater-only systems)
- No harmonics (hysteresis introduces phase shift)

The describing function error for a relay with hysteresis is approximately:
```
Ku_actual = Ku_measured * correction_factor
correction_factor ≈ 1 / sqrt(1 - (hysteresis/amplitude)²)
```

For our system: 1.0°C hysteresis with typical 3-5°C amplitude gives a 2-6% correction — small but worth applying in Phase 1.
