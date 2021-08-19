#include "fuzzyMeasuring.hpp"
#include "mutils.hpp"

using namespace cv;



//template<typename T>
//FuzzyMeasurePairsResult
//fuzzyMeasurePairs(const Mat &img, const T &measureHandle, double sigma, double ampThresh,
//                  double fuzzyThresh, TransitionType transition)
//
//template FuzzyMeasurePairsResult
//fuzzyMeasurePairs<MeasureRectangle>(const cv::Mat &img, const MeasureRectangle &measureHandle, double sigma,
//                                    double ampThresh, double fuzzyThresh, TransitionType transition);
//
//template FuzzyMeasurePairsResult
//fuzzyMeasurePairs<MeasureArc>(const cv::Mat &img, const MeasureArc &measureHandle, double sigma, double ampThresh,
//                              double fuzzyThresh, TransitionType transition);

///* I could have wrote helper methods to avoid duplicate code between fuzzyMeasurePairs and this method,
// * but I've decided that it would decrease readability too much for it to be beneficial. */
//template<typename T>
//FuzzyMeasurePairingsResult
//fuzzyMeasurePairings(const cv::Mat &img, const T &measureHandle, double sigma, double ampThresh,
//                     double fuzzyThresh, TransitionType transition, unsigned int numPairs)
//
//template FuzzyMeasurePairingsResult
//fuzzyMeasurePairings<MeasureRectangle>(const cv::Mat &img, const MeasureRectangle &measureHandle, double sigma,
//                                       double ampThresh, double fuzzyThresh, TransitionType transition,
//                                       unsigned int numPairs);
//
//template FuzzyMeasurePairsResult
//fuzzyMeasurePairings<MeasureArc>(const cv::Mat &img, const MeasureArc &measureHandle, double sigma, double ampThresh,
//                                 double fuzzyThresh, TransitionType transition, unsigned int numPairs);