%module dsp
%{
#define SWIG_FILE_WITH_INIT
#include "src/util/ImageAndExposure.h"
#include "src/util/Undistort.h"
#include "src/IOWrapper/Output3DWrapper.h"
#include "src/FullSystem/FullSystem.h"
//using namespace dso;
%}

%include <std_string.i>
%include <std_vector.i>

// handle numpy
%include numpy.i
%init %{
  import_array();
%}

// Typemapping for numpy arrays to char pointer arrays
%apply (unsigned char* IN_ARRAY2, int DIM1, int DIM2) {
  (unsigned char* image, int rows, int cols)
}

// Tweak the Undistort API
%ignore dso::Undistort::getK;
%ignore dso::Undistort::getSize;
%ignore dso::Undistort::getOriginalParameter;
%ignore dso::Undistort::getOriginalSize;
%ignore dso::Undistort::photometricUndist;

%rename dso::FullSystem DSOSystem

%include "src/util/ImageAndExposure.h"
%include "src/util/Undistort.h"
%include "src/IOWrapper/Output3DWrapper.h"
%include "src/FullSystem/FullSystem.h"

%extend dso::Undistort {
    ImageAndExposure* undistort_greyscale(unsigned char* image, int rows, int cols, float exposure=0, double timestamp=0, float factor=1) const
    {
        MinimalImage<unsigned char> dso_img(cols, rows);
        memcpy(dso_img->data, image, rows * cols * sizeof(unsigned char));
        return $self->undistort<unsigned char>(dso_img, exposure, timestamp, factor);
    }
}

