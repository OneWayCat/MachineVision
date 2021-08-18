#ifndef GENERATOR_ANCHORS_HPP
#define GENERATOR_ANCHORS_HPP

/**
 * @brief Parent structure which defines commonly used variables to assist with
 * generating test cases for Metrology testing.
 * 
 * @details Provides original values which GeneratorParams should perform its random jitters around.
 */
struct BaseAnchor {
    int row;
    int column;
    double phi;
};

/**
 * @brief An anchor for circle related parameters. Used when generating circular test cases for Metrology testing.
 */
struct ArcAnchor : BaseAnchor {
    int radius;
};

/**
 * @brief An anchor for ellipse related parameters. Used when generating elliptical test cases for Metrology testing.
 */
struct EllipseAnchor : BaseAnchor {
    int shortRadius;
    int longRadius;
};

/**
 * @brief An anchor for line related parameters. Used when generating linear test cases for Metrology testing.
 * Does NOT use the 'phi' paramter in its parent class during generation.
 */
struct LineAnchor : BaseAnchor {
    int endRow = 0;
    int endColumn = 0;
    int thickness = 5;
};

/**
 * @brief An anchor for ellipse related parameters. Used when generating rectangular test cases for Metrology testing.
 */
struct RectAnchor : BaseAnchor {
    int halfWidth;
    int halfHeight;
};
#endif