#include "beadModel.hpp"
#include "drawBead.hpp"
#include "drawHandles.hpp"
#include <chrono>
#include "pickContour.hpp"

using namespace std::chrono;
using namespace cv;

Contour generateRefContour() {
	Contour refContour;

	// All beads
	refContour.emplace_back(701, 319);
	refContour.emplace_back(626, 336);
	refContour.emplace_back(538, 367);
	refContour.emplace_back(443, 431);
	refContour.emplace_back(390, 489);
	refContour.emplace_back(360, 546);
	refContour.emplace_back(354, 646);
	refContour.emplace_back(363, 722);
	refContour.emplace_back(400, 776);
	refContour.emplace_back(458, 826);
	refContour.emplace_back(509, 869);
	refContour.emplace_back(588, 919);
	refContour.emplace_back(659, 934);
	refContour.emplace_back(696, 929);

	//refContour.emplace_back(699, 318);
	//refContour.emplace_back(627, 330);
	//refContour.emplace_back(562, 347);
	//refContour.emplace_back(506, 380);
	//refContour.emplace_back(471, 416);
	//refContour.emplace_back(437, 465);
	//refContour.emplace_back(402, 493);
	//refContour.emplace_back(361, 512);
	//refContour.emplace_back(339, 561);
	//refContour.emplace_back(336, 610);
	//refContour.emplace_back(349, 668);
	//refContour.emplace_back(369, 745);
	//refContour.emplace_back(399, 794);
	//refContour.emplace_back(462, 837);
	//refContour.emplace_back(550, 882);
	//refContour.emplace_back(637, 920);
	//refContour.emplace_back(699, 934);

	// OMG1
//	refContour.emplace_back(765, 75);
//	refContour.emplace_back(614, 72);
//	refContour.emplace_back(511, 73);
//	refContour.emplace_back(196, 66);
//	refContour.emplace_back(152, 74);
//	refContour.emplace_back(113, 113);
//	refContour.emplace_back(84, 143);
//	refContour.emplace_back(75, 258);
//	refContour.emplace_back(69, 718);
//	refContour.emplace_back(76, 1110);
//	refContour.emplace_back(73, 1331);
//	refContour.emplace_back(88, 1455);
//	refContour.emplace_back(123, 1512);
//	refContour.emplace_back(211, 1515);
//	refContour.emplace_back(482, 1505);
//	refContour.emplace_back(811, 1502);

	return refContour;
}

void drawResultContour(Mat &img, const Contour &contour, const ErrorType errorType) {
	Scalar contourColor;

	switch (errorType) {
		case ErrorType::VALID:
			contourColor = Scalar(0, 255, 0);
			break;

		case ErrorType::NO_BEAD:
			contourColor = Scalar(0, 0, 255);
			break;

		case ErrorType::OUT_OF_RANGE:
			contourColor = Scalar(0, 255, 255);
			break;

		default:
			contourColor = Scalar(0, 165, 255);
	}

	for (size_t i = 1; i < contour.size(); i++) {
		const Point2i &current = contour[i - 1];
		const Point2i &next = contour[i];

		line(img, reverse(current), reverse(next), contourColor);
	}
}

int main(int argc, char *argv[]) {
	// Read the image
	Mat image;
	Mat gray;

	//image = imread(argv[1]);
	image = imread("D:/Programming/Workspaces/Zebra/H-Rev-C/bead/images/bead1.jpg");
	Mat resultsImg = image.clone();
	cvtColor(image, gray, COLOR_BGR2GRAY);

	/** ---------------------------------- //
	// Contour creation. Preset or custom //
	// ---------------------------------- */
	Contour refContour = generateRefContour();
//	Contour refContour = pickContour(image);

	BeadModel model(refContour, 14, 7, 30);
//	drawBeadModel(image, model);
//	imshow("orig", image);
//	waitKey(0);

	/** ----------------------------------------- //
	// Template creation for 1d-measuring stuffs //
	// ----------------------------------------- */
	MeasureRectTemplate rectTemplate(45, 5);

	std::vector<double> xVals({7, 14, 21});
	std::vector<double> yVals({0.6, 1.0, 0.6});
	FuzzyFunc sizeFunc(xVals, yVals);
	rectTemplate.addFuzzy(sizeFunc, FuzzyType::SIZE);

	FuzzyMeasuringFunctor fuzzyFunctor(1.1, 30, 0.5, TransitionType::NEGATIVE);


	/** ----------------  //
	//  Core method call  //
	//  ----------------  */
	int samples = 1;
	int numIters = 1;
	BeadInspectionResult res;

	for (int t = 0; t < samples; t++) {
		auto start = high_resolution_clock::now();
		for (int i = 0; i < numIters; i++) {
			res = model.applyBeadModel(gray, rectTemplate, fuzzyFunctor);
//			if (i % (numIters / 10) == 0) {
//				std::cout << i << "\n";
//			}
		}

		auto stop = high_resolution_clock::now();
		double cumTime = duration_cast<microseconds>(stop - start).count();
		std::cout << "Full Runtime: " << cumTime / numIters << "\n";
	}
	/** ----------------- //
	// Debugging drawing //
	// ----------------- */

	// Draw the results from applyBeadModel function
	for (size_t i = 0; i < res.leftContours.size(); i++) {
		const Contour &leftContour = res.leftContours[i].first;
		const Contour &rightContour = res.rightContours[i].first;
		const ErrorType errorType = res.leftContours[i].second;

		drawResultContour(resultsImg, leftContour, errorType);
		drawResultContour(resultsImg, rightContour, errorType);
	}


	for (const MeasureRectangle &rect : model.modelMeasures) {
		drawMeasureHandle(image, rect);
	}

	imshow("results", resultsImg);
	imshow("orig", image);

	while (true) {
		auto keyPressed = waitKey(0);

		if (keyPressed == int('q')) {
			destroyAllWindows();
			break;
		}
	}
}