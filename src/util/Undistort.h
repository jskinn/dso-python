/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include <memory>
#include "util/ImageAndExposure.h"
#include "util/MinimalImage.h"
#include "util/NumType.h"
#include "Eigen/Core"





namespace dso
{


class PhotometricUndistorter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	PhotometricUndistorter(std::string file, std::string noiseImage, std::string vignetteImage, int w_, int h_);
	PhotometricUndistorter(int width, int height, std::vector<float> gamma, MinimalImageB* vignette_image);
	PhotometricUndistorter(int width, int height, std::vector<float> gamma, MinimalImage<unsigned short>* vignette_image);
	~PhotometricUndistorter();

	// removes readout noise, and converts to irradiance.
	// affine normalizes values to 0 <= I < 256.
	// raw irradiance = a*I + b.
	// output will be written in [output].
	template<typename T> void processFrame(T* image_in, float exposure_time, float factor=1);
	void unMapFloatImage(float* image);

	ImageAndExposure* output;

	float* getG() {if(!valid) return 0; else return G;};
private:
    float G[256*256];
    int GDepth;
	float* vignetteMap;
	float* vignetteMapInv;
	int w,h;
	bool valid;
};


class Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	virtual ~Undistort();

	virtual void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const {};

	inline const Mat33 getK() const {return K;};
	inline const Eigen::Vector2i getSize() const {return Eigen::Vector2i(w,h);};
	inline const VecX getOriginalParameter() const {return parsOrg;};
	inline const Eigen::Vector2i getOriginalSize() {return Eigen::Vector2i(wOrg,hOrg);};
	inline bool isValid() {return valid;};

	template<typename T>
	ImageAndExposure* undistort(const MinimalImage<T>* image_raw, float exposure=0, double timestamp=0, float factor=1) const;
	static Undistort* getUndistorterForFile(std::string configFilename, std::string gammaFilename, std::string vignetteFilename);

	void loadPhotometricCalibration(std::string file, std::string noiseImage, std::string vignetteImage);
	void makePhotometricCalibration(std::vector<float> gamma, MinimalImageB* vignette_image);
	void makePhotometricCalibration(std::vector<float> gamma, MinimalImage<unsigned short>* vignette_image);

	std::shared_ptr<PhotometricUndistorter> photometricUndist;

	// Rectification modes
	static const int RECT_CROP = 0;
	static const int RECT_FULL = 1;
	static const int RECT_NONE = 2;

protected:
	// Construct from a file
	Undistort(const char* configFileName, int nPars, std::string prefix = "");

	// Construct from existing parameters
	Undistort(int wOrg, int hOrg, VecX parsOrg, int rectificationMode, int outWidth, int outHeight);
	Undistort(int wOrg, int hOrg, VecX parsOrg, int rectFx, int rectFy, int rectCx, int rectCy, int outWidth, int outHeight);

    int w, h, wOrg, hOrg, wUp, hUp;
    int upsampleUndistFactor;
	Mat33 K;
	VecX parsOrg;
	bool valid;
	bool passthrough;

	float* remapX;
	float* remapY;

	void applyBlurNoise(float* img) const;

	void makeOptimalK_crop();
	void makeOptimalK_full();
};

class UndistortFOV : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	// Create from file
    UndistortFOV(const char* configFileName, bool noprefix);

	// Create from existing parameters
	UndistortFOV(double fx, double fy, double cx, double cy, double omega, int in_width, int in_height, int rectification_mode, int out_width, int out_height);
	UndistortFOV(double fx, double fy, double cx, double cy, double omega, int in_width, int in_height, int rect_fx, int rect_fy, int rect_cx, int rect_cy, int out_width, int out_height);

	~UndistortFOV();
	void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;
	//static Undistort* make(double fx, double fy, double cx, double cy, double omega, int in_width, int in_height, bool crop, int out_width, int out_height);

protected:
	static VecX makeParams(double fx, double fy, double cx, double cy, double omega);
};

class UndistortRadTan : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	// Create from file
    UndistortRadTan(const char* configFileName, bool noprefix);

	// Create from existing parameters
	UndistortRadTan(double fx, double fy, double cx, double cy, double k1, double k2, double r1, double r2, int in_width, int in_height, int rectification_mode, int out_width, int out_height);
	UndistortRadTan(double fx, double fy, double cx, double cy, double k1, double k2, double r1, double r2, int in_width, int in_height, int rect_fx, int rect_fy, int rect_cx, int rect_cy, int out_width, int out_height);

    ~UndistortRadTan();
    void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

protected:
	static VecX makeParams(double fx, double fy, double cx, double cy, double k1, double k2, double r1, double r2);
};

class UndistortEquidistant : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	// Create from file
    UndistortEquidistant(const char* configFileName, bool noprefix);

	// Create from existing parameters
	UndistortEquidistant(double fx, double fy, double cx, double cy, double k1, double k2, double r1, double r2, int in_width, int in_height, int rectification_mode, int out_width, int out_height);
	UndistortEquidistant(double fx, double fy, double cx, double cy, double k1, double k2, double r1, double r2, int in_width, int in_height, int rect_fx, int rect_fy, int rect_cx, int rect_cy, int out_width, int out_height);

	~UndistortEquidistant();
    void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

protected:
	static VecX makeParams(double fx, double fy, double cx, double cy, double k1, double k2, double r1, double r2);
};

class UndistortPinhole : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	// Create from file
    UndistortPinhole(const char* configFileName, bool noprefix);

	// Create from existing parameters
	UndistortPinhole(double fx, double fy, double cx, double cy, int in_width, int in_height, int rectification_mode, int out_width, int out_height);
	UndistortPinhole(double fx, double fy, double cx, double cy, int in_width, int in_height, int rect_fx, int rect_fy, int rect_cx, int rect_cy, int out_width, int out_height);
	~UndistortPinhole();
	void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

protected:
	static VecX makeParams(double fx, double fy, double cx, double cy);

private:
	float inputCalibration[8];
};

class UndistortKB : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	// Create from file
    UndistortKB(const char* configFileName, bool noprefix);

	// Create from existing parameters
	UndistortKB(double fx, double fy, double cx, double cy, double k1, double k2, double r1, double r2, int in_width, int in_height, int rectification_mode, int out_width, int out_height);
	UndistortKB(double fx, double fy, double cx, double cy, double k1, double k2, double r1, double r2, int in_width, int in_height, int rect_fx, int rect_fy, int rect_cx, int rect_cy, int out_width, int out_height);
	~UndistortKB();
	void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

protected:
	static VecX makeParams(double fx, double fy, double cx, double cy, double k1, double k2, double r1, double r2);
};

}

