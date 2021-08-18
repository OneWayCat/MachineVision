#include "pickContour.hpp"

using namespace cv;

// Event handler to pick the point and append it to the points vector
void onMouse(const int event, const int col, const int row, const int flags, void* param) {
	if (event == EVENT_LBUTTONDOWN) {
		Contour* ptPtr = (Contour*)param;
		ptPtr->emplace_back(row, col);
	}
}

Contour pickContour(const Mat& img) {
	std::cout << "Press 'q' to confirm and quit out of the selection." << "\n";
	Contour points;

	const char* winName = "Pick Contour";
	namedWindow(winName);

	Mat imCopy = img.clone();
	setMouseCallback(winName, onMouse, (void*)&points);

	constexpr int pointRadius = 3;
	const Scalar pointColor{ 0, 255, 0 };
	const Scalar lineColor{ 0, 0, 255 };

	while (true) {
		imshow(winName, imCopy);
		const int keyCode = waitKey(10);

		if (keyCode == 'q')
			break;

		for (const Point2i& pt : points)
			circle(imCopy, Point2i{ pt.y, pt.x }, pointRadius, pointColor, FILLED);

		for (size_t i = 1; i < points.size(); i++) {
			const Point2i& currentPt = points[i - 1];
			const Point2i& nextPt = points[i];
			line(imCopy, Point2i{ currentPt.y, currentPt.x }, Point2i{ nextPt.y, nextPt.x }, lineColor);
		}
	}

	return points;
}