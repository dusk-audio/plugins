#pragma once

#include <juce_dsp/juce_dsp.h>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <numeric>

//==============================================================================
/**
    EBU R128 / ITU-R BS.1770-4 LUFS Meter

    Implements:
    - K-weighting pre-filter
    - Momentary loudness (400ms sliding window)
    - Short-term loudness (3s sliding window)
    - Integrated loudness (gated program loudness)
    - Loudness range (LRA)
*/
class LUFSMeter
{
public:
    LUFSMeter() = default;

    void prepare(double sampleRate, int numChannels);
    void reset();
    void process(const float* left, const float* right, int numSamples);

    float getMomentaryLUFS() const { return momentaryLUFS; }
    float getShortTermLUFS() const { return shortTermLUFS; }
    float getIntegratedLUFS() const { return integratedLUFS; }
    float getLoudnessRange() const { return loudnessRange; }

    float getMaxMomentary() const { return maxMomentary; }
    float getMaxShortTerm() const { return maxShortTerm; }

    void resetIntegrated();

private:
    void initKWeighting(double sampleRate);
    float applyKWeighting(float sample, int channel);
    float calculateMeanSquare(const std::vector<float>& buffer) const;
    static float meanSquareToLUFS(float meanSquare);
    void updateIntegratedLoudness();
    void updateLoudnessRange();

    //==========================================================================
    double sampleRate = 48000.0;
    int channels = 2;

    struct BiquadState
    {
        float x1 = 0.0f, x2 = 0.0f;
        float y1 = 0.0f, y2 = 0.0f;
    };

    struct BiquadCoeffs
    {
        float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
        float a1 = 0.0f, a2 = 0.0f;
    };

    BiquadCoeffs highShelfCoeffs;
    BiquadCoeffs highPassCoeffs;
    std::array<BiquadState, 2> highShelfState;
    std::array<BiquadState, 2> highPassState;

    std::vector<float> momentaryBuffer;
    int momentaryWritePos = 0;
    int momentarySamples = 0;

    std::vector<float> shortTermBuffer;
    int shortTermWritePos = 0;
    int shortTermSamples = 0;

    std::deque<float> gatedBlocks;
    std::vector<float> blockBuffer;
    int blockWritePos = 0;
    int blockSamples = 0;

    float momentaryLUFS = -100.0f;
    float shortTermLUFS = -100.0f;
    float integratedLUFS = -100.0f;
    float loudnessRange = 0.0f;

    float maxMomentary = -100.0f;
    float maxShortTerm = -100.0f;

    static constexpr float ABSOLUTE_GATE = -70.0f;  // LUFS
    static constexpr float RELATIVE_GATE = -10.0f;  // LU below ungated mean
};
