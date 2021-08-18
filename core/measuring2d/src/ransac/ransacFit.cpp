#include <boost/math/constants/constants.hpp>
#include "ransacFit.hpp"
#include "mathutils.hpp"

constexpr int NUM_SIDES = 4;  // Number of sides in a rectangle

using namespace cv;
using namespace boost::math;

inline double square(double x) {
    return x * x;
}

inline bool sameSign(double a, double b) {
    return signbit(a) == signbit(b);
}

CircleParams fitCircle(const std::vector<Point2d> &points) {
    /* We can take the general equation (x - a)^2 + (y - b)^2 = r^2 and expand it like so:
     * 2ax + 2by + (r^2 - a^2 - b^2) = x^2 + y^2
     * This becomes c1 * 2x + c2 * 2y + c3 = x^2 + y^2, where c1 = a, c2 = b, c3 = r^2 - a^2 - b^2. */
    int n = (int) points.size();
    CircleParams result{};

    if (n < 3)
        return result;

    Mat_<double> A(n, 3);  // Matrix of coefficients
    Mat_<double> b(n, 1);  // Matrix of RHS values
    Mat_<double> C(3, 1);  // Result matrix

    for (int i = 0; i < n; i++) {
        auto *Ai = A.ptr<double>(i);
        auto *bi = b.ptr<double>(i);
        Point2d pt = points[i];

        Ai[0] = 2 * pt.x;
        Ai[1] = 2 * pt.y;
        Ai[2] = 1;

        bi[0] = square(pt.x) + square(pt.y);
    }

    if (n == 3)
        // Solve directly for an exact solution
        solve(A, b, C);
    else
        // Solve the least squares system A^T * A * c = A^T * b
        solve(A, b, C, DECOMP_NORMAL);

    // Find the center of the circle
    double centerX = C(0, 0);
    double centerY = C(1, 0);

    // Find the radius of the circle
    double c3 = C(2, 0);
    double radius = sqrt(c3 + square(centerX) + square(centerY));

    result.center = Point2d(centerX, centerY);
    result.radius = radius;

    return result;
}

/**
 * Given the coefficients for the general form of an ellipse, get its shape.
 */
EllipseParams getEllipseShape(const Mat &a1, const Mat &a2) {
    // General form: ax^2 + bxy + cy^2 + dx + ey + f = 0
    auto a1Ptr = a1.ptr<double>(0);
    double a = a1Ptr[0];
    double b = a1Ptr[1];
    double c = a1Ptr[2];

    auto a2Ptr = a2.ptr<double>(0);
    double d = a2Ptr[0];
    double e = a2Ptr[1];
    double f = a2Ptr[2];

    double discriminant = b * b - 4 * a * c;

    // This shouldn't happen unless our ellipse is a parabola
    assert(discriminant != 0);

    // Calculate the center of the ellipse
    double x = (2 * c * d - b * e) / discriminant;
    double y = (2 * a * e - b * d) / discriminant;

    // Calculate the radii of the ellipse
    double mu = 1 / (a * x * x + b * x * y + c * y * y - f);
    double mA = mu * a;
    double mB = mu * b;
    double mC = mu * c;

    double lhs = mA + mC;
    double rhs = sqrt(square(mA - mC) + square(mB));

    double L1 = (lhs + rhs) / 2;
    double L2 = (lhs - rhs) / 2;

    double shortRadius = 1 / sqrt(L1);
    double longRadius = 1 / sqrt(L2);

    // Calculate the orientation of the major axis
    double phi = atan(1 / b * (c - a - sqrt(square(a - c) + b * b)));

    if (a < c)
        phi -= double_constants::half_pi;

    return EllipseParams{Point2d{x, y}, shortRadius, longRadius, phi};
}

EllipseParams fitEllipse5(const std::vector<Point2d> &points) {
    const int N = 5;  // Number of points
    const int F = 100000;  // Constrain F

    // Solve the system ax^2 + bxy + cx^2 + dx + ey = -F
    Mat_<double> A(N, 5);  // Coefficient matrix

    for (int i = 0; i < N; i++) {
        double x = points[i].x;
        double y = points[i].y;
        auto Aptr = A.ptr<double>(i);

        Aptr[0] = x * x;
        Aptr[1] = x * y;
        Aptr[2] = y * y;
        Aptr[3] = x;
        Aptr[4] = y;
    }

    Mat_<double> b(N, 1, -F);  // Right hand side
    Mat_<double> coeffs(5, 1);  // Result vector to store (a, b, c, d, e)

    solve(A, b, coeffs);

    coeffs = coeffs.t();
    auto cPtr = coeffs.ptr<double>(0);

    double cA = cPtr[0];
    double cB = cPtr[1];
    double cC = cPtr[2];

    // Check if it's a valid ellipse
    if (cB * cB - 4 * cA * cC >= 0)
        return EllipseParams{};

    Mat_<double> a1 = (Mat_<double>(1, 3) << cA, cB, cC);
    Mat_<double> a2 = (Mat_<double>(1, 3) << cPtr[3], cPtr[4], F);
    return getEllipseShape(a1, a2);
}

EllipseParams fitEllipse(const std::vector<Point2d> &points) {
    /* The algorithm used is from the paper "Numerically Stable Direct Least Squares Fitting
     * of Ellipses" by Halir and Flusser. */
    int n = (int) points.size();

    if (n < 5)
        return EllipseParams{};

    Mat_<double> D1(n, 3);  // Quadratic part of the design matrix
    Mat_<double> D2(n, 3);  // Linear part of the design matrix

    for (int i = 0; i < n; i++) {
        auto D1ptr = D1.ptr<double>(i);
        auto D2ptr = D2.ptr<double>(i);

        double x = points[i].x;
        double y = points[i].y;

        D1ptr[0] = x * x;
        D1ptr[1] = x * y;
        D1ptr[2] = y * y;

        D2ptr[0] = x;
        D2ptr[1] = y;
        D2ptr[2] = 1;
    }

    Mat S1 = D1.t() * D1;  // Quadratic part of the scatter matrix
    Mat S2 = D1.t() * D2;  // Combined part of the scatter matrix
    Mat S3 = D2.t() * D2;  // Linear part of the scatter matrix

    Mat T = -S3.inv() * S2.t();  // For getting a2 from a1
    Mat M = S1 + S2 * T;         // Reduced scatter matrix

    Mat invC1 = (Mat_<double>(3, 3) << 0, 0, 0.5,
            0, -1, 0,
            0.5, 0, 0);  // Inverse of constraint matrix
    M = invC1 * M;            // Premultiply by C1.inv()

    Mat_<double> eval(1, 3);  // Eigenvalues
    Mat_<double> evec(3, 3);  // Eigenvectors
    eigenNonSymmetric(M, eval, evec);

    Mat a1 = evec.row(0);  // Contains a, b, c
    Mat a2 = a1 * T.t();       // Contains d, e, f

    return getEllipseShape(a1, a2);
}

LineEqParams fitLine2(const std::vector<Point2d> &points) {
    double x1 = points[0].x;
    double x2 = points[1].x;

    double y1 = points[0].y;
    double y2 = points[1].y;

    // Parameters for the line equation ax + by + c = 0
    double a, b, c;

    if (x1 == x2) {
        // Vertical line x - c = 0
        a = 1;
        b = 0;
        c = -x1;
    } else {
        // -mx + y + c = 0
        double slope = (y2 - y1) / (x2 - x1);
        a = -slope;
        b = 1;
        c = slope * x1 - y1;
    }

    return LineEqParams{a, b, c};
}

LineEqParams fitLine(const std::vector<Point2d> &points) {
    // We'll derive the line equation to ax + by + c = 0 form
    int numPoints = (int) points.size();
    LineEqParams result{};

    if (numPoints < 2)
        return result;

    std::vector<double> x = std::vector<double>(numPoints);
    std::vector<double> y = std::vector<double>(numPoints);

    double sumX = 0;
    double sumY = 0;
    double sumX2 = 0;
    double sumY2 = 0;
    double sumXY = 0;

    for (const auto &pt : points) {
        x.push_back(pt.x);
        y.push_back(pt.y);

        sumX += pt.x;
        sumY += pt.y;
        sumX2 += square(pt.x);
        sumY2 += square(pt.y);
        sumXY += pt.x * pt.y;
    }

    double minX = *std::min_element(x.begin(), x.end());
    double maxX = *std::max_element(x.begin(), x.end());
    double minY = *std::min_element(y.begin(), y.end());
    double maxY = *std::max_element(y.begin(), y.end());

    double a, b, c;

    if ((maxX - minX) > (maxY - minY)) {
        a = (numPoints * sumXY - sumX * sumY) / (numPoints * sumX2 - square(sumX));
        b = -1;
        c = (sumY - a * sumX) / numPoints;
    } else {
        a = -1;
        b = (numPoints * sumXY - sumY * sumX) / (numPoints * sumY2 - square(sumY));
        c = (sumX - b * sumY) / numPoints;
    }

    double discriminant = sqrt(square(a) + square(b));
    result.a = a / discriminant;
    result.b = b / discriminant;
    result.c = c / discriminant;

    return result;
}

// Return the upper triangular matrix (R) part of the QR decomposition of the matrix A.
Mat_<double> qr(Mat_<double> &A) {
    int m = A.rows;
    int n = A.cols;

    Mat_<double> Q = Mat_<double>::zeros(m, m);
    Mat_<double> R = Mat_<double>::zeros(m, n);

    // Gram - Schmidt process
    for (int k = 0; k < n; k++) {
        R(k, k) = norm(A.col(k));
        Q.col(k) = A.col(k) / R(k, k);

        for (int i = k + 1; i < n; i++) {
            double s = 0;

            for (int j = 0; j < m; j++)
                s += A(j, i) * Q(j, k);

            R(k, i) = s;

            for (int j = 0; j < m; j++)
                A(j, i) -= R(k, i) * Q(j, k);
        }
    }

    return R;
}

// Helper function for fitRect to solve the constrained least squares problem
std::pair<Mat_<double>, Mat_<double>> clsq(Mat_<double> &A) {
    Mat_<double> R = qr(A);

    // Enforce unique QR factorization
    if (sameSign(R(5, 5), R(4, 4)))
        R(5, 5) *= -1;

    // Take the part of the R matrix to do SVD on
    Mat_<double> subMat(R, Range(NUM_SIDES, NUM_SIDES + 2), Range(NUM_SIDES, NUM_SIDES + 2));
    Mat_<double> V = SVD(subMat).vt;

    // Enforce unique SVD
    if (sameSign(V(0, 0), V(1, 1)))
        V.row(1) *= -1;

    Mat_<double> n = V.col(1);

    Mat_<double> lhs(R, Range(0, NUM_SIDES), Range(0, NUM_SIDES));
    lhs = -lhs;

    Mat_<double> rhs(R, Range(0, NUM_SIDES), Range(NUM_SIDES, NUM_SIDES + 2));
    Mat_<double> c(NUM_SIDES, 2);
    solve(lhs, rhs, c);
    c *= n;

    return std::make_pair(c, n);
}

// Given the 4 corners of the rectangle, return the parameters of the rectangle (center, width, height, angle)
RectParams getRectParams(const std::vector<Point2d> &rectPoints) {
    const Point2d &r0 = rectPoints[0];
    const Point2d &r1 = rectPoints[1];
    const Point2d &r2 = rectPoints[2];

    // Since points are in counterclockwise order, r0 and r2 are opposite points
    Point2d center = (r0 + r2) / 2;
    double width = euclideanDistance(r0.x, r1.x, r0.y, r1.y);
    double height = euclideanDistance(r1.x, r2.x, r1.y, r2.y);

    // Find the angle of the vector that goes from r0 to r1
    double angle = atan2(r1.y - r0.y, r1.x - r0.x) - double_constants::half_pi;

    return RectParams{center.x, center.y, width / 2, height / 2, angle};
}

RectParams fitRect(const std::vector<Point2d> &points, const std::vector<size_t> &pointsPerSide) {
    int numPoints = (int) points.size();

    for (const size_t& numPts : pointsPerSide)
        if (numPts < 2)
            return RectParams{};

    // Coefficients matrix
    Mat_<double> A = Mat_<double>::zeros(numPoints, NUM_SIDES + 2);

    int row = 0;  // Row counter

    // Fill the first 4 columns
    for (int i = 0; i < NUM_SIDES; i++) {
        for (int j = 0; j < pointsPerSide[i]; j++) {
            A(row, i) = 1;
            row++;
        }
    }

    // Fill the last 2 columns
    for (int i = 0; i < numPoints; i++) {
        const Point2d &pt = points[i];

        if (A(i, 0) == 1 or A(i, 2) == 1) {
            A(i, 4) = pt.x;
            A(i, 5) = pt.y;
        } else {
            A(i, 4) = pt.y;
            A(i, 5) = -pt.x;
        }
    }

    auto retval = clsq(A);
    Mat_<double> &c = retval.first;
    Mat_<double> &n = retval.second;

    double n0 = n(0, 0);
    double n1 = n(1, 0);

    Mat_<double> B = (Mat_<double>(2, 2) << n0, -n1, n1, n0);

    double c0 = c(0, 0);
    double c1 = c(1, 0);
    double c2 = c(2, 0);
    double c3 = c(3, 0);

    Mat_<double> X = (Mat_<double>(2, 4) << c0, c2, c2, c0, c1, c1, c3, c3);
    X = -B * X;

    // Get the corners of the rectangle in counterclockwise order
    std::vector<Point2d> rectPoints;

    for (int i = 0; i < NUM_SIDES; i++)
        rectPoints.emplace_back(X(0, i), X(1, i));

    auto rectParams = getRectParams(rectPoints);
    rectParams.rectPoints = rectPoints;

    return rectParams;
}
