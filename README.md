# H-Rev-C
## Building
___
Depends on Boost 1.73.0 or onwards (probably) and atleast OpenCV 4.3.0.

To build:  
mkdir build  
cd build  
cmake ..  
make  

Executables should end up in a bin/ folder in the root directory.

Most subdirectories have their own CMakeLists. They are basically treated as independent libs.
The master CMakeLists in the root directory handles building everything.

## General Documentation
___
### Overview
___
This repository consists of four total projects. 1D Measuring (1DM), 2D Metrology (2DM), Bead Inspection and Meniscus Detection. <br>
There are, additionally, a variety of utility libraries that interdepend and are mostly used for development debugging. <br>
1D Measuring and 2D Metrology are core, with 2DM being dependent on 1DM. <br>
Bead Inspection and Meniscus Detection can be considered side projects with both being dependent solely on 1DM.

For all of the projects, generally speaking you can refer to the test files inside each of their subfolders for a rudimentary idea of how to use the respective libraries.
All of the projects/libraries were designed with the forward facing in mind, so in theory all that a user would need to know to use the library is already in the test files.

For more in depth details about the algorithms, please refer to our presentation (Put link here. Maybe commit the presentation directly to this repo)

### 1D Measuring
___
1D Measuring technically consists of two parts, "normal" measuring and "fuzzy" measuring although the latter is simply an
adaptation onto the former. This library is intended to detect and analyze "notable" edges inside either a rectangular or circular region. <br>

To perform "normal" measuring:
* Create a MeasureHandle object (MeasureRectangle or MeasureArc). Declarations in `measureHandle.hpp`
* Call one of the main processing functions (measurePairs, measurePos, measureThresh) Declarations in `measuring.hpp`

To perform "fuzzy" measuring:
* Create two vectors which represent the piecewise linear function that will be applied to the fuzzy attribute.
* Create a FuzzyFunc object with the vectors
* Create a MeasureHandle object like with 1D Measuring
* Call the MeasureHandle object's setFuzzyFunc method with the FuzzyFunc object and the appropriate fuzzy attribute. Declarations in `measureHandle.hpp`
* Call one of the fuzzy processing functions (fuzzyMeasurePairs, fuzzyMeasurePos, fuzzyMeasurePairings). Declarations in `fuzzyMeasuring.hpp`

Fuzzy measuring provides more precise ways to extract edge points from an area of interest by providing constraints on what can be considered a "good" edge point. 
Due to the large variety of possibilities currently available, please refer to the `measureHandle.hpp` file for more information.

### 2D Metrology
___
2D Metrology is an advancement on the 1D Measuring concept. This library is intended to detect entire shapes. It performs similar duties to manual quick and dirty template matching
but should be quite precise and relatively faster. <br>
The code currently supports: Rectangles, Arcs, Ellipses, Lines

To perform metrology:
* Create a MeasureParams object which should hold all of the hyperparameters that will be used by the main processing method. Declaration in `metrologyObject.hpp`
* Create a MetrologyObject object (ArcObject, EllipseObject, LineObject, RectObject) with the MeasureParams object and any additional parameters. Declarations in `metrologyObject.hpp`
* Call the MetrologyObject's applyMetrologyModel method with more hyper parameters. This call will also be performing the processing and effectively finalizes the MetrologyObject.
* Call the MetrologyObject's getMetrologyResults method to obtain all of the results.

2D Metrology also takes advantage of fuzzy measuring from 1D Measuring. All of the relevant methods are declared in `metrologyObject.hpp` and
consists essentially as getter/setter wrappers on top of the 1DM fuzzy getter/setters.
### Bead Inspection
___
Bead Inspection can be considered a specialty case of 1DM. The purpose of this library is to detect and validate adhesive beads and the likes
during manufacturing. This library also depends on GLM for its NURBs curve implemention.

To perform bead inspection:
* Define a "contour" of points which roughly follows the bead that is to be validated. This is imply a vector of cv::Point objects
* Create a BeadModel object with that contour of points
* Create a MeasureRectTemplate which will be used to create MeasureRectangles along the contour specified. 
  This is simply a wrapper over the MeasureRectangle class from 1D measuring which facilitates dynamic creation of MeasureRectangle objects.
  This is done because bead inspection essentially performs 2D metrology except along a spline. 
  Positions along this spline are interpolated dynamically, so MeasureHandle creation ended up being dynamic as well.
* Create a FuzzyMeasuringFunctor object. Similarly to MeasureRectTemplate, this facilitates the fuzzy measuring calls performed during the bead validation.
  Otherwise it acts essentially identically to a FuzzyFunc from 1DM.
* Call the BeadModel's applyBeadModel method with all of the arguments provided.

Note that Bead Inspection MUST use fuzzy. 
This is because each MeasureHandle along the contour must only give one pair of points (one point per side of the bead) to perform validation with.
The non fuzzy variant of 1DM has no way to disambiguate "good" pairs from bad pairs, hence the forced usage of the fuzzy variants.
### Meniscus Detection
___
Meniscus Detection can be considered a specialty case of 1DM. The purpose of this library is to detect the vertical position of the meniscus of a liquid in a container.

To perform meniscus detection:
* Create a MeniscusParams object. This just holds a variety of the hyper parameters used during the algorithm.
* Define the top left and bottom right points of a rectangle which the meniscus resides in. This is the detection area.
* Call the findMeniscus function with all of the arguments provided.
### Generators
___
Entirely a development library used to test 1DM and 2D metrology. Given its purpose and status, please refer to the source files to determine how to use it.
Generally speaking, however, all that needs to be done is an appropriate configuration object is created and provided to the appropriate generator.
Examples of how this library can be used can be found in the file `metrology_testgenerators.cpp`.

### Drawing
___
Similar to the Generators library, this is mostly intended to be a development library. This provides a variety of ways to visualize the 1DM and 2D Metrology
objects and provides some rudimentary sub pixel drawing (which basically just zooms into a section of an image and scales it up before drawing on it).
Specific parts of this library were designed to visualize specific things, so refer to the keywords to determine how and where to use the different parts of this library.

# TODO
___
* Meniscus Detection should probably expose fuzzy measuring as an option to use during detection.
* Maybe modularize the `measureHandle.hpp` file a bit more by splitting off the fuzzy specific declaration/definitions.