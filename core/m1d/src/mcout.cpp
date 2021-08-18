#include "mcout.hpp"
#include "arrutils.hpp"

#include <iterator>

// Printing MeasurePosResult
std::ostream& operator<<(std::ostream& o, const MeasurePosResult& p) {
    std::vector<cv::Point2d> pos = p.pos;
    std::vector<double> amps = p.amplitudes;
    std::vector<double> dists = p.distances;

    auto pi = pos.begin();
    auto ai = amps.begin();
    auto di = dists.begin();

    std::cout << "[\n";
    while (pi < pos.end() && ai < amps.end()) {
        std::cout << "\t(" << (*pi).x << ", " << (*pi).y << ", " << *ai << ')';
        std::cout << "\n";
        std::advance(pi, 1);
        std::advance(ai, 1);
    }
    std::cout << "]\n";

    std::cout << "[ ";
    while (di < dists.end()) {
        std::cout << *di;
        std::cout << ", ";
        std::advance(di, 1);
    }
    std::cout << "]\n";

    return o;
}

// Printing MeasurePairsResult
std::ostream& operator<<(std::ostream& o, const MeasurePairsResult& p) {
    std::vector<cv::Point2d> posFirst = p.posFirst;
    std::vector<double> ampsFirst = p.amplitudesFirst;

    std::vector<cv::Point2d> posSecond = p.posSecond;
    std::vector<double> ampsSec = p.amplitudesSecond;

    auto p1 = posFirst.begin();
    auto a1 = ampsFirst.begin();

    auto p2 = posSecond.begin();
    auto a2 = ampsSec.begin();

    std::cout << "[\n";
    while (p1 < posFirst.end() && a1 < ampsFirst.end() &&
           p2 < posSecond.end() && a2 < ampsSec.end()) {
        std::cout << "\t(" << (*p1).x << ", " << (*p1).y << ", " << *a1 << ')';
        std::cout << "\t---";
        std::cout << "\t(" << (*p2).x << ", " << (*p2).y << ", " << *a2 << ')';
        std::cout << "\n";
        std::advance(p1, 1);
        std::advance(a1, 1);
        std::advance(p2, 1);
        std::advance(a2, 1);
    }
    std::cout << "]\n";

    std::vector<double> intraDist = p.intraDistance;
    std::vector<double> interDist = p.interDistance;
    auto intraIter = intraDist.begin();
    auto interIter = interDist.begin();

    std::cout << "[ ";
    while (intraIter < intraDist.end()) {
        std::cout << *intraIter;
        std::cout << ", ";
        std::advance(intraIter, 1);
    }
    std::cout << "]\n";

    std::cout << "[ ";
    while (interIter < interDist.end()) {
        std::cout << *interIter;
        std::cout << ", ";
        std::advance(interIter, 1);
    }
    std::cout << "]\n";

    return o;
}

// Printing FuzzyMeasurePosResult
std::ostream& operator<<(std::ostream& o, const FuzzyMeasurePosResult& p) {
    std::vector<cv::Point2d> pos = p.pos;
    std::vector<double> amps = p.amplitudes;
    std::vector<double> dists = p.distances;
    std::vector<double> scores = p.fuzzyScores;

    auto pi = pos.begin();
    auto ai = amps.begin();
    auto di = dists.begin();
    auto si = scores.begin();

    std::cout << "[\n";
    while (pi < pos.end() && ai < amps.end() && si < scores.end()) {
        std::cout << "\t(" << (*pi).x << ", " << (*pi).y << ", " << *ai << ", " << *si << ')';
        std::cout << "\n";
        std::advance(pi, 1);
        std::advance(ai, 1);
        std::advance(si, 1);
    }
    std::cout << "]\n";

    std::cout << "[ ";
    while (di < dists.end()) {
        std::cout << *di;
        std::cout << ", ";
        std::advance(di, 1);
    }
    std::cout << "]\n";

    return o;
}

// Printing FuzzyMeasurePairsResult
std::ostream& operator<<(std::ostream& o, const FuzzyMeasurePairsResult& p) {
    std::vector<cv::Point2d> posFirst = p.posFirst;
    std::vector<double> ampsFirst = p.amplitudesFirst;

    std::vector<cv::Point2d> posSecond = p.posSecond;
    std::vector<double> ampsSec = p.amplitudesSecond;

    std::vector<double> scores = p.fuzzyScores;

    auto p1 = posFirst.begin();
    auto a1 = ampsFirst.begin();

    auto p2 = posSecond.begin();
    auto a2 = ampsSec.begin();

    auto si = scores.begin();

    std::cout << "[\n";
    while (p1 < posFirst.end() && a1 < ampsFirst.end() &&
           p2 < posSecond.end() && a2 < ampsSec.end() && si < scores.end()) {
        std::cout << "\t(" << (*p1).x << ", " << (*p1).y << ", " << *a1 << ')';
        std::cout << "\t---";
        std::cout << "\t(" << (*p2).x << ", " << (*p2).y << ", " << *a2 << ')';
        std::cout << "\t" << *si;
        std::cout << "\n";
        std::advance(p1, 1);
        std::advance(a1, 1);
        std::advance(p2, 1);
        std::advance(a2, 1);
        std::advance(si, 1);
    }
    std::cout << "]\n";

    std::vector<double> intraDist = p.intraDistance;
    std::vector<double> interDist = p.interDistance;
    auto intraIter = intraDist.begin();
    auto interIter = interDist.begin();

    std::cout << "[ ";
    while (intraIter < intraDist.end()) {
        std::cout << *intraIter;
        std::cout << ", ";
        std::advance(intraIter, 1);
    }
    std::cout << "]\n";

    std::cout << "[ ";
    while (interIter < interDist.end()) {
        std::cout << *interIter;
        std::cout << ", ";
        std::advance(interIter, 1);
    }
    std::cout << "]\n";

    return o;
}
