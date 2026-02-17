#pragma once

#include <cmath>
#include <algorithm>

//==============================================================================
/**
    Stereo correlation meter.

    Calculates Pearson correlation coefficient between L and R channels.
    Range: -1 (out of phase) to +1 (mono/in phase)
*/
class CorrelationMeter
{
public:
    CorrelationMeter() = default;

    void prepare(double sampleRate)
    {
        int windowSamples = static_cast<int>(sampleRate * 0.3);  // 300ms integration
        decayCoeff = 1.0f - (1.0f / static_cast<float>(windowSamples));
        smoothingCoeff = 0.95f;

        reset();
    }

    void reset()
    {
        sumLR = 0.0f;
        sumL2 = 0.0f;
        sumR2 = 0.0f;
        smoothedCorrelation = 0.0f;
    }

    void process(const float* left, const float* right, int numSamples)
    {
        for (int i = 0; i < numSamples; ++i)
        {
            float L = left[i];
            float R = right[i];

            sumLR = sumLR * decayCoeff + L * R;
            sumL2 = sumL2 * decayCoeff + L * L;
            sumR2 = sumR2 * decayCoeff + R * R;
        }

        float correlation = calculateCorrelation();
        smoothedCorrelation = smoothedCorrelation * smoothingCoeff +
                              correlation * (1.0f - smoothingCoeff);
    }

    float getCorrelation() const
    {
        return calculateCorrelation();
    }

    float getSmoothedCorrelation() const
    {
        return smoothedCorrelation;
    }

    static const char* getCorrelationLabel(float correlation)
    {
        if (correlation > 0.9f) return "Mono";
        if (correlation > 0.5f) return "Good";
        if (correlation > 0.0f) return "Wide";
        if (correlation > -0.5f) return "Very Wide";
        return "Out of Phase";
    }

private:
    float calculateCorrelation() const
    {
        float denominator = std::sqrt(sumL2 * sumR2);

        if (denominator < 1e-10f)
            return 0.0f;

        float correlation = sumLR / denominator;
        return std::clamp(correlation, -1.0f, 1.0f);
    }

    float sumLR = 0.0f;
    float sumL2 = 0.0f;
    float sumR2 = 0.0f;

    float decayCoeff = 0.999f;
    float smoothingCoeff = 0.95f;
    float smoothedCorrelation = 0.0f;
};
