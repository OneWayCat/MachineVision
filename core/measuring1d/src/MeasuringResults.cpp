#include "arrutils.hpp"
#include "MeasuringResults.hpp"
#include "FuzzyStructs.hpp"

// Constructors for measuring result structs

EdgeResults::EdgeResults(std::vector<cv::Point2d> &pos_, std::vector<double> &distances_) :
        edgePositions{std::move(pos_)},
        distances{std::move(distances_)} {
}

EdgeResults::EdgeResults(EdgeResults &&obj) noexcept:
        edgePositions{std::move(obj.edgePositions)},
        distances{std::move(obj.distances)} {
}

EdgePositionResults::EdgePositionResults(EdgeResults &pos, std::vector<double> &amplitudes_) :

        EdgeResults{std::move(pos)},
        amplitudes{std::move(amplitudes_)} {
}

EdgePairsResults::EdgePairsResults(std::vector<cv::Point2d> &posFirst_, std::vector<double> &amplitudesFirst_,
                                   std::vector<cv::Point2d> &posSecond_, std::vector<double> &amplitudesSecond_,
                                   std::vector<double> &intraDistance_, std::vector<double> &interDistance_) :

        posFirst{std::move(posFirst_)},
        amplitudesFirst{std::move(amplitudesFirst_)},

        posSecond{std::move(posSecond_)},
        amplitudesSecond{std::move(amplitudesSecond_)},

        intraDistance{std::move(intraDistance_)},
        interDistance{std::move(interDistance_)} {
}

EdgePairsResults EdgePairsResults::front() const {
    auto pf = getFirst(posFirst);
    auto af = getFirst(amplitudesFirst);

    auto ps = getFirst(posSecond);
    auto as = getFirst(amplitudesSecond);

    auto intra = getFirst(intraDistance);
    auto inter = getFirst(interDistance);

    return EdgePairsResults{pf, af, ps, as, intra, inter};
}

EdgePairsResults EdgePairsResults::back() const {
    auto pf = getLast(posFirst);
    auto af = getLast(amplitudesFirst);

    auto ps = getLast(posSecond);
    auto as = getLast(amplitudesSecond);

    auto intra = getLast(intraDistance);
    auto inter = getLast(interDistance);

    return EdgePairsResults{pf, af, ps, as, intra, inter};
}

FuzzyMeasurePosResult::FuzzyMeasurePosResult(EdgeResults &pos, std::vector<double> &amplitudes_,
                                             std::vector<double> &scores_) :

        EdgePositionResults(pos, amplitudes_),
        fuzzyScores{std::move(scores_)} {
}

FuzzyMeasurePairsResult::FuzzyMeasurePairsResult(std::vector<cv::Point2d> &posFirst, std::vector<double> &ampFirst,
                                                 std::vector<cv::Point2d> &posSecond, std::vector<double> &ampSecond,
                                                 std::vector<double> &intraDist, std::vector<double> &interDist,
                                                 std::vector<cv::Point2d> &posCenter, std::vector<double> &scores) :

        EdgePairsResults{posFirst, ampFirst, posSecond, ampSecond, intraDist, interDist},
        posCenter{std::move(posCenter)},
        fuzzyScores{std::move(scores)} {
}