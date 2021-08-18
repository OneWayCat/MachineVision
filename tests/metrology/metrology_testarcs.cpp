#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

#include "metrologyObject.hpp"
#include "drawMetrology.hpp"
#include "metcout.hpp"

using namespace boost::math;
using namespace boost::random;
using namespace cv;

int main(int argc, char *argv[]) {
    Mat image;
    Mat gray;

    image = imread(argv[1]);
    cvtColor(image, gray, COLOR_BGR2GRAY);

    srand((unsigned int) time(0));
    Size shape = image.size();

    int row = shape.height / 2;
    int col = shape.width / 2;

    int rad = shape.height / 4;

    mt19937 rng;
    rng.seed((unsigned int) time(0));

    auto randomAngle = uniform_real_distribution<double>(-double_constants::two_pi, double_constants::two_pi);

    while (true) {
        Mat copy;
        image.copyTo(copy);

        double angleStart = randomAngle(rng);
        double angleExtent = randomAngle(rng);

        MeasureParams params{};
        params.numMeasures = 10;
        ArcObject arcObject{params, row, col, rad, angleStart, angleExtent};

        drawArcObject(copy, arcObject);
        std::cout << arcObject;

        imshow("image", copy);
        int keyPressed = waitKey(0);
        if (keyPressed == int('q')) {
            destroyAllWindows();
            break;
        }
    }
}
