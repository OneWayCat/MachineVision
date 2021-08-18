#ifndef BEAD_MEASURING_HPP
#define BEAD_MESURING_HPP

#include "fuzzyMeasuring.hpp"

struct FuzzyMeasuringFunctor {
	double sigma{};
	double ampThresh{};
	double fuzzyThresh{};
	TransitionType transition{};

	explicit FuzzyMeasuringFunctor() = default;

	/**
	 * @brief Construct a set of parameters for edge detection. See 1D measuring for more information.
	 * @param sig Sigma for gaussian smoothing.
	 * @param ampT Minimum edge amplitude.
	 * @param fuzzyT Minimum fuzzy score.
	 * @param trans Positive for dark->light->dark edges, Negative for light->dark->light edges. ALL for either.
	*/
	FuzzyMeasuringFunctor(double sig, double ampT, double fuzzyT, TransitionType trans = TransitionType::ALL)
		: sigma{ sig },
		ampThresh{ ampT },
		fuzzyThresh{ fuzzyT },
		transition{ trans } {}
	
	FuzzyMeasurePairsResult operator()(const cv::Mat& img, const MeasureRectangle& rect) const;
};

struct MeasureRectTemplate {
	int halfWidth{};
	int halfHeight{};
	std::unordered_map<FuzzyType, FuzzyFunc, EnumClassHash> fuzzySet;

	explicit MeasureRectTemplate() = default;

	/**
	 * @brief Provide a set of parameters to construct measure handles for edge detection.
	 * @param hw Half-width of the measure handles (Direction parallel to profile line).
	 * @param hh Half-height of the measure handles.
	*/
	MeasureRectTemplate(int hw, int hh)
		: halfWidth{ hw }, halfHeight{ hh } {}

	MeasureRectangle operator()(int row, int col, double phi) const;

	/**
	 * @brief Add a fuzzy function to set of fuzzy functions.
	 * @param func 
	 * @param type 
	*/
	void addFuzzy(const FuzzyFunc& func, const FuzzyType& type);
};

#endif
