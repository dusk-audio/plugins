/*
  ==============================================================================

    SSLSaturation.h

    Console saturation emulation for British EQ mode.
    Two console types with distinct harmonic profiles:
    - E-Series: warmer, predominantly 2nd harmonic
    - G-Series: tighter, predominantly 3rd harmonic

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include <chrono>
#include <cmath>
#include <random>

class SSLSaturation
{
public:
    enum class ConsoleType
    {
        ESeries,    // E-Series (Brown knobs) - warmer, more 2nd harmonic
        GSeries     // G-Series (Black knobs) - cleaner, more 3rd harmonic
    };

    /** Construct with optional seed for reproducibility.
        seed == 0 (default): non-deterministic (random_device / clock fallback).
        seed != 0: deterministic — useful for unit tests. */
    explicit SSLSaturation(uint32_t seed = 0)
    {
        if (seed == 0)
        {
            // Each plugin instance gets unique "hardware" characteristics
            try {
                std::random_device rd;
                seed = rd();
            } catch (...) {
                seed = static_cast<uint32_t>(
                    std::chrono::steady_clock::now().time_since_epoch().count());
            }
        }

        std::mt19937 gen(seed);
        std::uniform_real_distribution<float> dist(-0.05f, 0.05f);

        // Different tolerances for different circuit stages
        transformerTolerance = 1.0f + dist(gen);     // Input transformer variation
        opAmpTolerance = 1.0f + dist(gen);           // Op-amp gain variation
        outputTransformerTolerance = 1.0f + dist(gen); // Output transformer variation

        noiseGen_L = std::mt19937(gen());
        noiseGen_R = std::mt19937(gen());
    }

    void setConsoleType(ConsoleType type)
    {
        consoleType = type;
    }

    void setSampleRate(double newSampleRate)
    {
        if (newSampleRate <= 0.0)
            return;

        sampleRate = newSampleRate;

        // High-pass at ~5Hz to remove any DC offset from saturation
        const float cutoffFreq = 5.0f;
        const float RC = 1.0f / (juce::MathConstants<float>::twoPi * cutoffFreq);
        dcBlockerCoeff = RC / (RC + 1.0f / static_cast<float>(sampleRate));
    }

    void reset()
    {
        dcBlockerX1_L = dcBlockerY1_L = 0.0f;
        dcBlockerX1_R = dcBlockerY1_R = 0.0f;
        lastSample_L = lastSample_R = 0.0f;
        highFreqEstimate_L = highFreqEstimate_R = 0.0f;
    }

    // Main processing function with drive amount (0.0 to 1.0)
    float processSample(float input, float drive, bool isLeftChannel)
    {
        // Suppress denormals for performance (prevents CPU spikes on older processors)
        juce::ScopedNoDenormals noDenormals;

        // NaN/Inf protection - return silence (0.0f) if input is invalid
        if (!std::isfinite(input))
            return 0.0f;

        if (drive < 0.001f)
            return input;

        // Gentle pre-saturation limiting to prevent extreme aliasing
        float limited = input;
        float absInput = std::abs(input);
        if (absInput > 0.95f)
        {
            // Soft knee limiting using tanh for smooth transition
            float excess = absInput - 0.95f;
            float compressed = 0.95f + std::tanh(excess * 3.0f) * 0.05f;
            limited = (input > 0.0f) ? compressed : -compressed;
        }

        // Frequency-dependent saturation: reduce HF drive for anti-aliasing
        float highFreqContent = estimateHighFrequencyContent(limited, isLeftChannel);

        // Progressive HF drive reduction for anti-aliasing
        // Scaling increases with both drive amount and frequency content
        float hfReduction = highFreqContent * (0.25f + drive * 0.35f);  // 25-60% reduction based on drive
        float effectiveDrive = drive * (1.0f - hfReduction);

        // Stage 1: Input transformer saturation
        float transformed = processInputTransformer(limited, effectiveDrive * transformerTolerance);

        // Stage 2: Op-amp gain stage
        float opAmpOut = processOpAmpStage(transformed, effectiveDrive * opAmpTolerance);

        // Stage 3: Output transformer (if applicable)
        // E-Series has output transformers, G-Series is transformerless
        float output = (consoleType == ConsoleType::ESeries)
            ? processOutputTransformer(opAmpOut, drive * 0.7f * outputTransformerTolerance)
            : opAmpOut;

        // Console noise floor (-90dB RMS base, increases with drive)
        float noiseLevel = 0.00003162f * (1.0f + drive * 0.5f); // -90dB base, increases with drive
        auto& noiseGen = isLeftChannel ? noiseGen_L : noiseGen_R;
        output += noiseDist(noiseGen) * noiseLevel;

        // DC blocking filter to prevent DC offset buildup
        output = processDCBlocker(output, isLeftChannel);

        // Mix with dry signal based on drive amount
        // At 100% drive, use 100% wet for maximum saturation effect
        float wetMix = juce::jlimit(0.0f, 1.0f, drive * 1.4f);  // Linear ramp, full wet at high drive
        float result = input * (1.0f - wetMix) + output * wetMix;

        // NaN/Inf protection - return clean input if saturation produced invalid output
        if (!std::isfinite(result))
            return input;

        return result;
    }

private:
    ConsoleType consoleType = ConsoleType::ESeries;
    double sampleRate = 44100.0;

    // DC blocker state
    float dcBlockerX1_L = 0.0f, dcBlockerY1_L = 0.0f;
    float dcBlockerX1_R = 0.0f, dcBlockerY1_R = 0.0f;
    float dcBlockerCoeff = 0.999f;

    // Frequency content estimation state
    float lastSample_L = 0.0f, lastSample_R = 0.0f;
    float highFreqEstimate_L = 0.0f, highFreqEstimate_R = 0.0f;

    // Configurable high-frequency scaling factor
    // Can be tuned or exposed to tests/parameters for extreme test signals
    // Reduced from 4.0f to 3.0f to prevent saturation on very dynamic material
    float highFreqScale = 3.0f;

    // Component tolerance variation (±5% per instance)
    // Simulates real hardware component tolerances for unique analog character
    float transformerTolerance = 1.0f;
    float opAmpTolerance = 1.0f;
    float outputTransformerTolerance = 1.0f;

    // Noise generation for console noise floor (per-channel, but not safe for
    // concurrent cross-thread access — callers must ensure single-threaded use)
    std::mt19937 noiseGen_L;
    std::mt19937 noiseGen_R;
    std::uniform_real_distribution<float> noiseDist{-1.0f, 1.0f};

    // Estimate high-frequency content using simple differentiator
    // without requiring full FFT or filter bank analysis
    float estimateHighFrequencyContent(float input, bool isLeftChannel)
    {
        float& lastSample = isLeftChannel ? lastSample_L : lastSample_R;
        float& estimate = isLeftChannel ? highFreqEstimate_L : highFreqEstimate_R;

        // First-order difference approximates high-frequency content
        // Large differences = high frequency, small differences = low frequency
        float difference = std::abs(input - lastSample);
        lastSample = input;

        // Smooth the estimate with a simple one-pole lowpass (RC filter)
        const float smoothing = 0.95f;  // Higher = more smoothing
        estimate = estimate * smoothing + difference * (1.0f - smoothing);

        // Normalize to 0-1 range (typical difference range is 0-0.5 for normalized audio)
        // Scale so that typical music content gives reasonable values
        // Use configurable highFreqScale instead of hardcoded value
        float normalized = juce::jlimit(0.0f, 1.0f, estimate * highFreqScale);

        return normalized;
    }

    // Input transformer saturation (even-order harmonics)
    float processInputTransformer(float input, float drive)
    {
        // Linear at normal levels, only saturates when driven hard
        const float transformerDrive = 1.0f + drive * 7.0f;  // Max 8x gain at full drive
        float driven = input * transformerDrive;

        // Soft saturation curve with even-order emphasis
        float abs_x = std::abs(driven);

        // Progressive saturation
        float saturated;
        if (abs_x < 0.9f)
        {
            // Linear region
            saturated = driven;
        }
        else if (abs_x < 1.5f)
        {
            // Gentle compression region - 2nd harmonic emerges
            float excess = abs_x - 0.9f;
            float compressed = 0.9f + excess * (1.0f - excess * 0.15f);
            saturated = (driven > 0.0f) ? compressed : -compressed;
        }
        else
        {
            // Hard saturation region - more harmonics
            float excess = abs_x - 1.5f;
            float compressed = 1.5f + std::tanh(excess * 1.5f) * 0.3f;
            saturated = (driven > 0.0f) ? compressed : -compressed;
        }

        // Console-specific harmonic coloration
        float threshold = (consoleType == ConsoleType::ESeries) ? 0.6f : 0.05f;

        if (abs_x > threshold)
        {
            // Scale harmonic generation based on how hard we're driving
            float saturationAmount = (abs_x - threshold) / (1.2f - threshold);
            saturationAmount = juce::jlimit(0.0f, 1.0f, saturationAmount);

            if (consoleType == ConsoleType::ESeries)
            {
                // E-Series: 2nd harmonic dominant
                saturated += saturated * saturated * (0.12f * saturationAmount);
            }
            else
            {
                // G-Series: 3rd harmonic dominant
                saturated += saturated * saturated * (0.025f * saturationAmount);
                saturated += saturated * saturated * saturated * (0.050f * saturationAmount);
            }
        }

        return saturated / transformerDrive;
    }

    // Op-amp stage: asymmetric clipping with 2nd and 3rd harmonics
    float processOpAmpStage(float input, float drive)
    {
        const float opAmpDrive = 1.0f + drive * 9.0f;
        float driven = input * opAmpDrive;

        float output;

        // Positive half-cycle
        if (driven > 0.0f)
        {
            if (driven < 1.0f)
            {
                output = driven;
            }
            else if (driven < 1.8f)
            {
                // Soft saturation region
                float excess = driven - 1.0f;
                output = 1.0f + excess * (1.0f - excess * 0.2f);
            }
            else
            {
                // Hard clipping region (supply rail)
                // E-Series clips softer, G-Series clips harder
                float clipHardness = (consoleType == ConsoleType::ESeries) ? 1.5f : 2.0f;
                output = 1.5f + std::tanh((driven - 1.8f) * clipHardness) * 0.3f;
            }
        }
        // Negative half-cycle (asymmetric)
        else
        {
            if (driven > -1.0f)
            {
                // Linear region
                output = driven;
            }
            else if (driven > -1.9f)
            {
                // Soft saturation region (slightly different than positive)
                float excess = -driven - 1.0f;
                output = -1.0f - excess * (1.0f - excess * 0.18f);
            }
            else
            {
                // Hard clipping region
                float clipHardness = (consoleType == ConsoleType::ESeries) ? 1.5f : 2.0f;
                output = -1.55f + std::tanh((driven + 1.9f) * clipHardness) * 0.3f;
            }
        }

        // Console-specific harmonic shaping
        float threshold = (consoleType == ConsoleType::ESeries) ? 0.6f : 0.05f;

        if (std::abs(driven) > threshold)
        {
            // Scale harmonic generation based on drive level
            float saturationAmount = (std::abs(driven) - threshold) / (1.5f - threshold);
            saturationAmount = juce::jlimit(0.0f, 1.0f, saturationAmount);

            if (consoleType == ConsoleType::ESeries)
            {
                // E-Series: 2nd harmonic dominant
                output += output * output * std::copysign(0.10f * saturationAmount, output);
            }
            else
            {
                // G-Series: 3rd harmonic dominant
                output += output * output * std::copysign(0.022f * saturationAmount, output);
                output += output * output * output * (0.040f * saturationAmount);
            }
        }

        return output / opAmpDrive;
    }

    // Output transformer saturation (E-Series only)
    // Similar to input transformer but with less drive
    float processOutputTransformer(float input, float drive)
    {
        const float transformerDrive = 1.0f + drive * 2.0f;
        float driven = input * transformerDrive;

        // Output transformer saturates less than input transformer
        // Adds final touch of even-order harmonics
        float abs_x = std::abs(driven);

        float saturated;
        if (abs_x < 0.5f)
        {
            saturated = driven;
        }
        else if (abs_x < 0.9f)
        {
            float excess = abs_x - 0.5f;
            float compressed = 0.5f + excess * (1.0f - excess * 0.25f);
            saturated = (driven > 0.0f) ? compressed : -compressed;
        }
        else
        {
            float excess = abs_x - 0.9f;
            float compressed = 0.9f + std::tanh(excess * 1.5f) * 0.15f;
            saturated = (driven > 0.0f) ? compressed : -compressed;
        }

        // Subtle 2nd harmonic emphasis
        saturated += saturated * saturated * 0.05f;

        return saturated / transformerDrive;
    }

    // DC blocking filter to prevent DC offset accumulation
    float processDCBlocker(float input, bool isLeftChannel)
    {
        // Simple first-order high-pass filter at ~5Hz
        float& x1 = isLeftChannel ? dcBlockerX1_L : dcBlockerX1_R;
        float& y1 = isLeftChannel ? dcBlockerY1_L : dcBlockerY1_R;

        float output = input - x1 + dcBlockerCoeff * y1;
        x1 = input;
        y1 = output;

        return output;
    }

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(SSLSaturation)
};
