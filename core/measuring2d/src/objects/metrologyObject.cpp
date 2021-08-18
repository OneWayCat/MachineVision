// Methods for MetrologyObject.
#include "metrologyObject.hpp"
#include "fuzzyMeasuring.hpp"

using namespace cv;

void MetrologyObject::validateMeasureParams() const {
    if (measureParams.perpLength < 1 or measureParams.tangLength < 0)
        throw std::invalid_argument("Measure handle height and width must be positive");

    if (measureParams.separation <= 0)
        throw std::invalid_argument("Separation between measures must be positive");

    if (measureParams.sigma < 0.4 or measureParams.sigma > 100)
        throw std::invalid_argument("Sigma must be between 0.4 and 100");

    if (measureParams.threshold < 0 or measureParams.threshold > 255)
        throw std::invalid_argument("Threshold must be between 0 and 255");

    if (measureParams.transition == TransitionType::ALL)
        throw std::invalid_argument("Transition must be either POSITIVE or NEGATIVE");
}

void MetrologyObject::setFuzzyThresh(double score) {
    if (score < 0 or score > 1)
        throw std::invalid_argument("fuzzyThresh must be between 0 and 1");

    fuzzyThresh = score;
}

void MetrologyObject::initMeasuresFuzzy() {
    for (MeasureHandle &measure : measures)
        for (const auto &func : fuzzySet)
            measure.setFuzzyFunc(func.second, func.first);
}

void MetrologyObject::applyMeasures(const Mat &img) {
    bool useFuzzy = !fuzzySet.empty();

    double sigma = measureParams.sigma;
    double ampThresh = measureParams.threshold;
    TransitionType transition = measureParams.transition;
    SelectType select = measureParams.select;

    measureResults.clear();

    for (const auto &mHandle : measures) {
        if (useFuzzy)
            measureResults.push_back(fuzzyMeasurePos(img, mHandle, sigma, ampThresh, fuzzyThresh, 250, 250, transition));

        else
            measureResults.push_back(measurePos(img, mHandle, sigma, ampThresh, transition, select));
    }
}

void ArcProjectionObject::applyMeasures(const Mat &img) {
    bool useFuzzy = !fuzzySet.empty();

    double sigma = measureParams.sigma;
    double ampThresh = measureParams.threshold;
    TransitionType transition = measureParams.transition;
    SelectType select = measureParams.select;

    measureResults.clear();

    for (const auto &mHandle : measures) {
        if (useFuzzy)
            measureResults.push_back(fuzzyMeasurePos(img, mHandle, sigma, ampThresh, fuzzyThresh, 250, 250, transition));

        else
            measureResults.push_back(measurePos(img, mHandle, sigma, ampThresh, transition, select));
    }
}