%module dso
%{
#define SWIG_FILE_WITH_INIT
#include <Eigen/Core>
#include "src/util/MinimalImage.h"
#include "src/util/ImageAndExposure.h"
#include "src/util/Undistort.h"
#include "src/IOWrapper/Output3DWrapper.h"
#include "src/FullSystem/FullSystem.h"
//using namespace dso;
%}

%include <exception.i>
%include <pyabc.i>
%include <std_string.i>
%include <std_vector.i>

// Templates for the vectors we use
%template (VecFloat) std::vector<float>;

// handle numpy
%include numpy.i
%init %{
  import_array();
%}

// Typemapping for numpy arrays to char pointer arrays
%apply (unsigned char* IN_ARRAY2, int DIM1, int DIM2) {
  (unsigned char* image, int rows, int cols)
}
%apply (unsigned short* IN_ARRAY2, int DIM1, int DIM2) {
  (unsigned short* image, int rows, int cols)
}

// Always ignore the Eigen macro, it's not actually a method, but SWIG picks it up
%ignore EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
%ignore dso::ImageAndExposure::EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
%ignore dso::Undistort::EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
%ignore dso::FullSystem::EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// Hide the image pointer in ImageAndExposure
%ignore dso::ImageAndExposure::image;

// Hide the PhotometricUndistorter constructors that take images, we'll override them with numpy below
%ignore dso::PhotometricUndistorter::PhotometricUndistorter(int width, int height, std::vector<float> gamma, MinimalImageB* vignette_image);
%ignore dso::PhotometricUndistorter::PhotometricUndistorter(int width, int height, std::vector<float> gamma, MinimalImage<unsigned short>* vignette_image);

// Tweak the Undistort API
%ignore dso::Undistort::getK;
%ignore dso::Undistort::getSize;
%ignore dso::Undistort::getOriginalParameter;
%ignore dso::Undistort::getOriginalSize;
%ignore dso::Undistort::photometricUndist;
%ignore dso::Undistort::undistort;
%ignore dso::Undistort::makePhotometricCalibration;

// Hide a bunch of the Output3DWrapper API for now
%ignore dso::IOWrap::Output3DWrapper::publishGraph;
%ignore dso::IOWrap::Output3DWrapper::publishKeyframes;
%ignore dso::IOWrap::Output3DWrapper::publishCamPose;
%ignore dso::IOWrap::Output3DWrapper::pushLiveFrame;
%ignore dso::IOWrap::Output3DWrapper::pushDepthImage;
%ignore dso::IOWrap::Output3DWrapper::pushDepthImageFloat;

// Hide other things in the FullSystem header
%ignore dso::deleteOut;
%ignore dso::deleteOutPt;
%ignore dso::deleteOutOrder;
%ignore dso::eigenTestNan;

// Tweak the FullSystem API, renaming it to DSOSystem
%ignore dso::FullSystem::marginalizeFrame;
%ignore dso::FullSystem::setOriginalCalib;
%rename dso::FullSystem DSOSystem;

// Handle exceptions in the constructor. API could be better.
// see https://stackoverflow.com/questions/1394484/how-do-i-propagate-c-exceptions-to-python-in-a-swig-wrapper-library
%exception { 
    try {
        $action
    } catch (...) {
        SWIG_exception(SWIG_RuntimeError, "Exception, see std::err for details");
    }
}

// Import the headers
%include "src/util/ImageAndExposure.h"
%include "src/util/Undistort.h"
%include "src/IOWrapper/Output3DWrapper.h"
%include "src/FullSystem/FullSystem.h"

%extend dso::Undistort {
    ImageAndExposure* undistort_greyscale(unsigned char* image, int rows, int cols, float exposure=0, double timestamp=0, float factor=1) const
    {
        dso::MinimalImage<unsigned char> dso_img(cols, rows);
        memcpy(dso_img.data, image, rows * cols * sizeof(unsigned char));
        return $self->undistort<unsigned char>(&dso_img, exposure, timestamp, factor);
    }

	  void make_photometric_calibration(std::vector<float> gamma, unsigned short* image, int rows, int cols)
    {
        dso::MinimalImage<unsigned short> dso_img(cols, rows);
        memcpy(dso_img.data, image, rows * cols * sizeof(unsigned char));
        $self->makePhotometricCalibration(gamma, &dso_img);
    }
}

