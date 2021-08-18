#include <random>
#include <algorithm>
#include "metrologyObject.hpp"

// The number of sides in a rectangle.
constexpr int NUM_SIDES = 4;

using namespace cv;

// Validate the parameters for applyMetrologyModel
void validateModelParams(double minScore, double distThresh, unsigned int numInstances) {
    if (minScore < 0 or minScore > 1)
        throw std::invalid_argument("minScore must be between 0.0 and 1.0");
    
    if (distThresh < 0)
        throw std::invalid_argument("distThresh must not be negative");

    if (numInstances < 1 or numInstances > 4)
        throw std::invalid_argument("numInstances must be between 1 and 4");
}

// For removing empty vectors
bool isEmpty(const std::vector<Point2d> &v) {
    return v.empty();
}

// Given the measureResults of a metrologyObject, get the points that we will use for RANSAC fitting. 
RansacPoints
getRansacPoints(const std::vector<MeasurePosResult> &measureResults, TransitionType transition) {
    RansacPoints allPoints(measureResults.size());

    for (size_t i = 0; i < measureResults.size(); i++) {
        const MeasurePosResult &result = measureResults[i];
        std::vector<Point2d> &pointsPerMeasure = allPoints[i];

        for (size_t j = 0; j < result.amplitudes.size(); j++) {
            if ((transition == TransitionType::POSITIVE and result.amplitudes[j] >= 0)
                or (transition == TransitionType::NEGATIVE and result.amplitudes[j] < 0))
                pointsPerMeasure.push_back(result.pos[j]);
        }
    }

    // Remove measures without points
    allPoints.erase(std::remove_if(allPoints.begin(), allPoints.end(), isEmpty), allPoints.end());
    return allPoints;
}

// Get the total number of points in the RansacPoints variable.
int getNumPoints(const RansacPoints& allPoints) {
    int count = 0;

    for (const std::vector<Point2d>& pts : allPoints)
        count += (int) pts.size();

    return count;
}

int calculateMaxIters(const RansacPoints &allPoints, unsigned int instancesLeft, unsigned int numInstances, int minNumMeasures) {
    // Chance of a measure region containing an inlier.
    constexpr double INLIER_CHANCE = 0.7;

	// Desired probability for RANSAC.
	constexpr double CONFIDENCE = 0.999;

    int numPoints = getNumPoints(allPoints);

    // Estimated proportion of inliers
    double inlierProportion = INLIER_CHANCE * allPoints.size() / numPoints;

    double k = log(1 - CONFIDENCE) / log(1 - instancesLeft * pow(inlierProportion, minNumMeasures));
    return int(ceil(k));
}

// Randomly choose (numPoints) points from allPoints.
std::vector<Point2d>
chooseFitPoints(const RansacPoints &allPoints, int numPoints, std::default_random_engine &rng) {
    // Use a pseudo-random number algorithm so that we avoid points that are too close to each other.
    size_t n = allPoints.size();

    // Indices of measures to use to pick fitting points
    std::vector<int> fitMeasureInds;

    /* Divide the number of total points by number of indicies to generate, like this:
     *
     * |1      4 | 5      8 | 9      12 |
     *    pick       pick       pick
     *
     * This doesn't eliminate the chances of consecutive points, but it greatly reduces them */

    int minimum = 0;
    int maximum;

    for (int pointsLeft = numPoints; pointsLeft > 0; pointsLeft--) {
        maximum = minimum + int(round(double(n) / pointsLeft));
        std::uniform_int_distribution<int> distribution(minimum, maximum - 1);

        int index = distribution(rng);
        fitMeasureInds.push_back(index);

        n -= size_t(maximum) - minimum;
        minimum = maximum;
    }

    // Now pick one point from each measure
    std::vector<Point2d> fitPoints;

    for (int i : fitMeasureInds) {
        const std::vector<Point2d>& pointsInMeasure = allPoints[i];
        std::uniform_int_distribution<int> distribution(0, (int) pointsInMeasure.size() - 1);

        int index = distribution(rng);
        fitPoints.push_back(pointsInMeasure[index]);
    }

    return fitPoints;
}

// Append inliers to the inliers vector.
void
appendInliers(const std::vector<int> &inlierInds, std::vector<Point2d> &inliers,
              const RansacPoints &allPoints) {
    for (int i = 0; i < inlierInds.size(); i++) {
        int index = inlierInds[i];

        if (index != -1)
            inliers.push_back(allPoints[i][index]);
    }
}

// Remove all inliers from the list of points.
void removeInliers(RansacPoints &allPoints, const std::vector<int> &inlierInds) {
    for (int i = 0; i < allPoints.size(); i++) {
        std::vector<Point2d>& pointsOnMeasure = allPoints[i];
        int indToRemove = inlierInds[i];

        if (indToRemove != -1)
            pointsOnMeasure.erase(pointsOnMeasure.begin() + indToRemove);
    }

    allPoints.erase(std::remove_if(allPoints.begin(), allPoints.end(), isEmpty), allPoints.end());
}

// Return true if the measure handle has an inlier, which means that the index is not -1.
bool hasInlier(int i) {
    return i != -1;
}

void MetrologyObject::applyMetrologyModel(const Mat &img, double minScore, unsigned int numInstances,
                                          double distThresh, int maxIters, unsigned int randSeed,
                                          bool allowOutsideRegion) {

    if (randSeed == 0)
        randSeed = (unsigned int) time(nullptr);

    std::default_random_engine rng(randSeed);

    validateModelParams(minScore, distThresh, numInstances);
    initMeasures();
    applyMeasures(img);

    RansacPoints allPoints = getRansacPoints(measureResults, measureParams.transition);

    for (unsigned int instances = numInstances; instances > 0; instances--) {
        // Calculate max number of iterations needed
        int maxItersCalculated = maxIters < 0 ? calculateMaxIters(allPoints, instances, numInstances, minNumMeasures) : maxIters;

        for (int i = 0; i < maxItersCalculated; i++) {
            if (allPoints.size() < minNumMeasures)
                return;  // Not enough points remaining

            std::vector<Point2d> fitPoints = chooseFitPoints(allPoints, minNumMeasures, rng);
            std::vector<int> inlierInds = getInliers(fitPoints, allPoints, distThresh);

            // Sufficient number of inliers
            int numInliers = (int) std::count_if(inlierInds.begin(), inlierInds.end(), hasInlier);
            double score = double(numInliers) / measures.size();

            if (score >= minScore) {
                std::vector<Point2d> inliers;
                appendInliers(inlierInds, inliers, allPoints);

                bool validShape = addResult(inliers, allowOutsideRegion);

                if (validShape) {
                   removeInliers(allPoints, inlierInds);
                    break;
                }
            }

        }
    }
}


std::vector<RansacPoints>
getRansacPointsRect(const std::vector<MeasurePosResult> &measureResults, TransitionType transition) {
    int measuresPerSide = int(measureResults.size()) / NUM_SIDES;
    std::vector<RansacPoints> allPoints;

    for (int i = 0; i < NUM_SIDES; i++)
        allPoints.push_back(RansacPoints(measuresPerSide));

    for (size_t i = 0; i < measureResults.size(); i++) {
        int side = (int) i / measuresPerSide;
        const MeasurePosResult &result = measureResults[i];
        std::vector<Point2d> &pointsPerMeasure = allPoints[side][i % measuresPerSide];
        
        for (size_t j = 0; j < result.amplitudes.size(); j++) {
            if ((transition == TransitionType::POSITIVE and result.amplitudes[j] >= 0)
                or (transition == TransitionType::NEGATIVE and result.amplitudes[j] < 0))
                pointsPerMeasure.push_back(result.pos[j]);
        }
    }
    
    // Remove measures without points
    for (int i = 0; i < NUM_SIDES; i++) {
        RansacPoints& pointsOnSide = allPoints[i];
        pointsOnSide.erase(std::remove_if(pointsOnSide.begin(), pointsOnSide.end(), isEmpty), pointsOnSide.end());
    }

    return allPoints;
}

void RectObject::applyMetrologyModel(const Mat &img, double minScore, unsigned int numInstances,
                                     double distThresh, int maxIters, unsigned int randSeed,
                                     bool allowOutsideRegion) {

    if (randSeed == 0)
        randSeed = (unsigned int) time(nullptr);

    std::default_random_engine rng(randSeed);

    validateModelParams(minScore, distThresh, numInstances);
    initMeasures();
    applyMeasures(img);

    std::vector<RansacPoints> allPoints = getRansacPointsRect(measureResults, measureParams.transition);
    int minPtsPerSide = minNumMeasures / NUM_SIDES;
    int measuresPerSide = int(measures.size()) / NUM_SIDES;

    for (unsigned int instances = numInstances; instances > 0; instances--) {
        // Calculate max number of iterations needed (per side)
        std::vector<Point2d> allInliers;
        std::vector<size_t> inliersPerSide;
        
        for (int side = 0; side < NUM_SIDES; side++) {  // One side at a time
            RansacPoints &pointsOnSide = allPoints[side];
			int maxItersCalculated = maxIters < 0 ? calculateMaxIters(pointsOnSide, instances, numInstances, minPtsPerSide) : maxIters;
            bool foundLine = false;  // This is true if we found a line with sufficient inliers

            for (int i = 0; i < maxItersCalculated; i++) {
                if (pointsOnSide.size() < minPtsPerSide)  // Not enough points remaining
                    return;

                std::vector<Point2d> fitPoints = chooseFitPoints(pointsOnSide, minPtsPerSide, rng);
                std::vector<int> inlierInds = getInliers(fitPoints, pointsOnSide, distThresh);

                // Sufficient number of inliers
                int numInliers = (int) std::count_if(inlierInds.begin(), inlierInds.end(), hasInlier);
                double score = double(numInliers) / measuresPerSide;

                if (score >= minScore) {
                    appendInliers(inlierInds, allInliers, pointsOnSide);

                    // Now remove all inliers from the list of points
                    removeInliers(pointsOnSide, inlierInds);
                    foundLine = true;
                    inliersPerSide.push_back(numInliers);

                    break;
                }
            }

            if (!foundLine)
                return;
        }

        addResult(allInliers, inliersPerSide, allowOutsideRegion);
    }
}
