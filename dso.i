%module(directors="1") dso
%{
#define SWIG_FILE_WITH_INIT
#include <exception>
#include <Eigen/Core>
#include "src/util/NumType.h"
#include "src/util/MinimalImage.h"
#include "src/util/ImageAndExposure.h"
#include "src/util/Undistort.h"
#include "src/FullSystem/HessianBlocks.h"
#include "src/util/FrameShell.h"
#include "src/IOWrapper/Output3DWrapper.h"
#include "src/FullSystem/FullSystem.h"
#include "src/settings_conf.h"
//using namespace dso;
%}

%include <exception.i>
%include <pyabc.i>
%include <std_shared_ptr.i>
%include <std_string.i>
%include <std_vector.i>

// Templates for the vectors we use
%template (VecFloat) std::vector<float>;
%template (PointVec) std::vector<dso::PointHessian*>;
%template (FrameHessianVec) std::vector<dso::FrameHessian*>;
%template (OutputVec) std::vector<dso::IOWrap::Output3DWrapper*>;

// handle numpy and Eigen
%include numpy.i
%init %{
  import_array();
%}
%include eigen.i

// Fudge some eigen includes for SWIG
#ifndef EIGEN_STRONG_INLINE
#define EIGEN_STRONG_INLINE inline
#endif
#ifndef EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

// Eigen typemaps for the specific matrix shapes used
%eigen_typemaps(dso::Vec2)
%eigen_typemaps(dso::Vec3)
%eigen_typemaps(dso::Vec4)
%eigen_typemaps(dso::Vec6)
%eigen_typemaps(dso::Vec10)
%eigen_typemaps(dso::Mat66)
%eigen_typemaps(dso::Mat42)

// Typemapping for numpy arrays to char pointer arrays
%apply (unsigned char* IN_ARRAY2, int DIM1, int DIM2) {
  (unsigned char* image, int rows, int cols)
}
%apply (unsigned short* IN_ARRAY2, int DIM1, int DIM2) {
  (unsigned short* image, int rows, int cols)
}
%apply (float* INPLACE_ARRAY2, int DIM1, int DIM2) { (float* data, int rows, int cols) } 

// Rename all values to python conventions
//%rename("%(undercase)s", regextarget=1) "dso::Undistort::.*";

// Always ignore the Eigen macro, it's not actually a method, but SWIG picks it up
// %ignore EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
// %ignore dso::ImageAndExposure::EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
// %ignore dso::Undistort::EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
// %ignore dso::FullSystem::EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// Ignore extra types in the HessianBlocks header
%ignore dso::affFromTo;
%ignore SCALE_IDEPTH;
%ignore SCALE_XI_ROT;
%ignore SCALE_XI_TRANS;
%ignore SCALE_F;
%ignore SCALE_C;
%ignore SCALE_W;
%ignore SCALE_A;
%ignore SCALE_B;
%ignore SCALE_IDEPTH_INVERSE;
%ignore SCALE_XI_ROT_INVERSE;
%ignore SCALE_XI_TRANS_INVERSE;
%ignore SCALE_F_INVERSE;
%ignore SCALE_C_INVERSE;
%ignore SCALE_W_INVERSE;
%ignore SCALE_A_INVERSE;
%ignore SCALE_B_INVERSE;
%ignore dso::FrameFramePrecalc;

// Ignore a bunch of the FrameHessian properties
%ignore dso::FrameHessian::efFrame;
%ignore dso::FrameHessian::instanceCounter;
%ignore dso::FrameHessian::dI;
%ignore dso::FrameHessian::dIp;
%ignore dso::FrameHessian::absSquaredGrad;
%ignore dso::FrameHessian::worldToCam_evalPT;
%ignore dso::FrameHessian::state_zero;
%ignore dso::FrameHessian::state_scaled;
%ignore dso::FrameHessian::state;
%ignore dso::FrameHessian::step;
%ignore dso::FrameHessian::step_backup;
%ignore dso::FrameHessian::state_backup;
%ignore dso::FrameHessian::targetPrecalc;
%ignore dso::FrameHessian::debugImage;

// Ignore setters for FrameHessian properties, for now they are immutable externally
%ignore dso::FrameHessian::setStateZero;
%ignore dso::FrameHessian::setState;
%ignore dso::FrameHessian::setStateScaled;
%ignore dso::FrameHessian::setEvalPT;
%ignore dso::FrameHessian::setEvalPT_scaled;
%ignore dso::FrameHessian::release;
%ignore dso::FrameHessian::makeImages;

// Mark other properties read-only
%immutable dso::FrameHessian::shell;
%immutable dso::FrameHessian::frameID;
%immutable dso::FrameHessian::idx;
%immutable dso::FrameHessian::frameEnergyTH;
%immutable dso::FrameHessian::ab_exposure;
%immutable dso::FrameHessian::flaggedForMarginalization;
%immutable dso::FrameHessian::pointHessians;
%immutable dso::FrameHessian::pointHessiansMarginalized;
%immutable dso::FrameHessian::pointHessiansOut;
%immutable dso::FrameHessian::immaturePoints;
%immutable dso::FrameHessian::nullspaces_pose;
%immutable dso::FrameHessian::nullspaces_affine;
%immutable dso::FrameHessian::nullspaces_scale;
%immutable dso::FrameHessian::PRE_worldToCam;
%immutable dso::FrameHessian::PRE_camToWorld;

// Ignore instanceCounter and setters for CalibHessian
%ignore dso::CalibHessian::instanceCounter;
%ignore dso::CalibHessian::setValue;
%ignore dso::CalibHessian::setValueScaled;

// Prevent setters for CalibHessian properties
%immutable dso::CalibHessian::value_zero;
%immutable dso::CalibHessian::value_scaled;
%immutable dso::CalibHessian::value_scaledf;
%immutable dso::CalibHessian::value_scaledi;
%immutable dso::CalibHessian::value;
%immutable dso::CalibHessian::step;
%immutable dso::CalibHessian::step_backup;
%immutable dso::CalibHessian::value_backup;
%immutable dso::CalibHessian::value_minus_value_zero;

// Hide PointHessian properties and setters
%ignore dso::PointHessian::instanceCounter;
%ignore dso::PointHessian::efPoint;
%ignore dso::PointHessian::residuals;
%ignore dso::PointHessian::lastResiduals;
%ignore dso::PointHessian::setPointStatus;
%ignore dso::PointHessian::setIdepth;
%ignore dso::PointHessian::setIdepthScaled;
%ignore dso::PointHessian::setIdepthZero;
%ignore dso::PointHessian::release;

// Make the visible PointHessian properties immutable
%immutable dso::PointHessian::color;
%immutable dso::PointHessian::weights;
%immutable dso::PointHessian::u;
%immutable dso::PointHessian::v;
%immutable dso::PointHessian::idx;
%immutable dso::PointHessian::energyTH;
%immutable dso::PointHessian::host;
%immutable dso::PointHessian::hasDepthPrior;
%immutable dso::PointHessian::my_type;
%immutable dso::PointHessian::idepth_scaled;
%immutable dso::PointHessian::idepth_zero_scaled;
%immutable dso::PointHessian::idepth_zero;
%immutable dso::PointHessian::idepth;
%immutable dso::PointHessian::step;
%immutable dso::PointHessian::step_backup;
%immutable dso::PointHessian::idepth_backup;
%immutable dso::PointHessian::nullspaces_scale;
%immutable dso::PointHessian::idepth_hessian;
%immutable dso::PointHessian::maxRelBaseline;
%immutable dso::PointHessian::numGoodResiduals;
%immutable dso::PointHessian::status;

// Ignore FrameShell SE3 values, we'll add getters for them
%ignore dso::FrameShell::camToTrackingRef;
%ignore dso::FrameShell::camToWorld;
%ignore dso::FrameShell::aff_g2l;

%immutable dso::FrameShell::id;
%immutable dso::FrameShell::incoming_id;
%immutable dso::FrameShell::timestamp;
%immutable dso::FrameShell::trackingRef;
%immutable dso::FrameShell::poseValid;
%immutable dso::FrameShell::statistics_outlierResOnThis;
%immutable dso::FrameShell::statistics_goodResOnThis;
%immutable dso::FrameShell::marginalizedAt;
%immutable dso::FrameShell::movedByOpt;

// Hide most of the MinimalImage
%ignore dso::MinimalImage::data;
%immutable dso::MinimalImage::w;
%immutable dso::MinimalImage::h;

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
%rename dso::Undistort::distortCoordinates distort_coordinates;
%rename dso::Undistort::getUndistorterForFile get_undistorter_for_file;
%rename dso::Undistort::isValid is_valid;
%rename dso::Undistort::loadPhotometricCalibration load_photometric_calibration;

// Hide a bunch of the Output3DWrapper API for now
%feature("director") dso::IOWrap::Output3DWrapper;
//%ignore dso::IOWrap::Output3DWrapper::publishGraph;
//%ignore dso::IOWrap::Output3DWrapper::publishKeyframes;
//%ignore dso::IOWrap::Output3DWrapper::publishCamPose;
//%ignore dso::IOWrap::Output3DWrapper::pushLiveFrame;
%ignore dso::IOWrap::Output3DWrapper::pushDepthImage;
%ignore dso::IOWrap::Output3DWrapper::needPushDepthImage;
//%ignore dso::IOWrap::Output3DWrapper::pushDepthImageFloat;

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

// Include some structs from HessianBlocks and NumType to pass data in and out
//extern struct AffLight;
//extern struct FrameHessian;
//extern struct CalibHessian;
//extern struct PointHessian;

// Import the headers
%include "src/util/NumType.h"
%include "src/util/MinimalImage.h"
%include "src/util/FrameShell.h"
%include "src/FullSystem/HessianBlocks.h"
%include "src/util/ImageAndExposure.h"
%include "src/settings_conf.h"
%include "src/util/Undistort.h"
%include "src/IOWrapper/Output3DWrapper.h"
%include "src/FullSystem/FullSystem.h"

%template(MinimalImageF) dso::MinimalImage<float>;

%extend dso::Undistort {
    ImageAndExposure* undistort_greyscale(unsigned char* image, int rows, int cols, float exposure=0, double timestamp=0, float factor=1) const
    {
        // We can create a very temporary MinimalImage here because the data will be copied int othe output ImageAndExposure
        dso::MinimalImage<unsigned char> dso_img(cols, rows);
        memcpy(dso_img.data, image, rows * cols * sizeof(unsigned char));
        return $self->undistort<unsigned char>(&dso_img, exposure, timestamp, factor);
    }

	void make_photometric_calibration(std::vector<float> gamma, unsigned short* image, int rows, int cols)
    {
        // Again, the data in this MinimalImage will be copied, and can thus be deleted when we're done.
        dso::MinimalImage<unsigned short> dso_img(cols, rows);
        memcpy(dso_img.data, image, rows * cols * sizeof(unsigned char));
        $self->makePhotometricCalibration(gamma, &dso_img);
    }
}

%extend dso::MinimalImage<float> {
    void toNumpy(float* data, int rows, int cols) {
        if (rows != $self->h || cols != $self->w)
        {
            throw std::exception();
        }
        else
        {
            memcpy(data, $self->data, rows * cols * sizeof(float));
        }
    }
}

%extend dso::FrameShell {
    dso::Vec3 get_cam_to_world_translation()
    {
        return $self->camToWorld.translation();
    }

    dso::Vec4 get_cam_to_world_rotation()
    {
        // Convert the quaternion into a 4-element vector, which will in turn be converted to numpy.
        Eigen::Quaternion<double> quat = $self->camToWorld.so3().unit_quaternion();
        return dso::Vec4(quat.x(), quat.y(), quat.z(), quat.w());
    }

    dso::Vec3 get_cam_to_tracking_ref_translation()
    {
        return $self->camToTrackingRef.translation();
    }

    dso::Vec4 get_cam_to_tracking_ref_rotation()
    {
        // Convert the quaternion into a 4-element vector, which will in turn be converted to numpy.
        Eigen::Quaternion<double> quat = $self->camToTrackingRef.so3().unit_quaternion();
        return dso::Vec4(quat.x(), quat.y(), quat.z(), quat.w());
    }
}
