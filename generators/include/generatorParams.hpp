
#ifndef GENERATOR_PARAMS_HPP
#define GENERATOR_PARAMS_HPP

#include <boost/math/constants/constants.hpp>

#include "mathutils.hpp"

/**
 * @brief Struct which stores oft-used parameters during random test case generation for Metrology testing.
 * @details 'jitter' defines a range to substract or add to the original 'anchor' value during random generation.
 * Literally jitters the shape.
 */
struct GeneratorParams {
    int jitterRow = 3;
    int jitterColumn = 3;
    double jitterPhi = deg2rad(2);

    // Number of instances to generate. Additional instances head towards the center of the shape except in case of lines where it alternates sides.
    int numInstances = 1;
    int instanceSeparation = 0;
};

/**
 * @brief Struct which stores jitter parameters for circular test generation. 
 * Also stores a range of values from which to sample start angle values and end angle values for partial circles.
 */
struct ArcGeneratorParams : GeneratorParams {
    ArcGeneratorParams(int r, std::vector<double> as, std::vector<double> ae) :
            jitterRadius{r},
            angleStartRange{std::move(as)},
            angleEndRange{std::move(ae)} {}

    int jitterRadius = 10;
    std::vector<double> angleStartRange{0, boost::math::double_constants::two_pi};
    std::vector<double> angleEndRange{0, boost::math::double_constants::two_pi};
};

/**
 * @brief Struct which stores jitter parameters for elliptical test generation.
 * Also stores a range of values from which to sample start angle values and end angle values for partial ellipses.
 */
struct EllipseGeneratorParams : GeneratorParams {
    EllipseGeneratorParams(int sr, int lr, std::vector<double> as, std::vector<double> ae) :
            jitterShortRadius{sr},
            jitterLongRadius{lr},
            angleStartRange{std::move(as)},
            angleEndRange{std::move(ae)} {}

    int jitterShortRadius = 10;
    int jitterLongRadius = 10;
    std::vector<double> angleStartRange{0, boost::math::double_constants::two_pi};
    std::vector<double> angleEndRange{0, boost::math::double_constants::two_pi};
};

/**
 * @brief Struct which stores oft-used parameters for line test generation.
 */
struct LineGeneratorParams : GeneratorParams {
    int jitterEndRow = 5;
    int jitterEndColumn = 5;
    int jitterThickness = 0;
};

/**
 * @brief Struct which stores oft-used parameters for rectangular test generation.
 */
struct RectGeneratorParams : GeneratorParams {
    int jitterHalfWidth = 5;
    int jitterHalfHeight = 5;
};
#endif