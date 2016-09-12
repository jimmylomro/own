/*
    Copyright (C) 2016 Jaime Lomeli-R. Univesity of Southampton

    This file is part of OWN.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
       * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
       * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.
       * Neither the name of the ASL nor the names of its contributors may be
         used to endorse or promote products derived from this software without
         specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <algorithm>
#include <own.h>
#include <nms.h>
#include <bfm_centres_8.4_64.h>


//---------------------------------------------------------------------------------------------------------------------
//-----------------------------OwnFeatureMaps--------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

// Constructors
own::OwnFeatureMaps::OwnFeatureMaps(float thresh, int M, int N, int K, int kernSize) {

	if (K > own::params::centres::MAX_K) K = own::params::centres::MAX_K;


	this->thresh	= thresh;
	this->M		= M;
	this->N		= N;
	this->K		= K;
	this->kernSize	= kernSize;

	bfm = new own::BFM(5, M, N, kernSize);

	initCentreMat();
}



// Methods
void own::OwnFeatureMaps::createFeatureMaps(const cv::Mat& image) {

	bfm->fillMagList(image, magList);

	featureMaps.clear();
	featureMaps.resize(K);

	for (int k = 0; k < K; k++) {
		featureMaps[k] = createOneFeatureMap(k, image.rows, image.cols);
	}
}


void own::OwnFeatureMaps::detectKeypoints(std::vector<cv::KeyPoint>& keypoints, const cv::Mat& image) {

	if (featureMaps.empty()) {
		if (image.empty())
			throw std::runtime_error("Error: featureMaps is empty, no image loaded");
		else
			createFeatureMaps(image);
	}
	
	keypoints.clear();
	keypoints.reserve(own::MAX_EXPECTED_KEYPOINTS_PER_MAP * K);

	for (int k = 0; k < K; k++) {
		std::vector<cv::KeyPoint> keypointsInMap = detectKeypointsInMap(k);
		
		for (int i = 0; i < keypointsInMap.size(); i++)	
			keypoints.push_back(keypointsInMap[i]);
	}
}


cv::Mat own::OwnFeatureMaps::createOneFeatureMap(int centIdx, int rows, int cols) {

	if (magList.empty())
		throw std::runtime_error("Error: magList is empty");

	cv::Mat map = cv::Mat::zeros(rows, cols, CV_32FC1);

	float* mag_rowPtr;
	float* map_rowPtr;
	float* cen_rowPtr;
	float* centrePtr;
	
	float num;
	float den;
	float numDif;
	float denDif;
	float denAux;

	int ndims = M * N;

	centrePtr = centres.ptr<float>(centIdx);
	for (int row = 0; row < rows; row++) {
		map_rowPtr = map.ptr<float>(row);
		for (int col = 0; col < cols; col++) {
			mag_rowPtr = magList.ptr<float>(row*cols+col);
			
			num = 0;
			den = 0;
			
			// Calculate numerator
			for (int dim = 0; dim < ndims; dim++) {
				numDif  = centrePtr[dim] - mag_rowPtr[dim];
				num    += numDif * numDif;
			}
			num = std::max(num,(float)0.000001);	// Handle division by 0
			num = 1/(num*num);

			// Calculate denominator
			for (int k = 0; k < K; k++) {
				cen_rowPtr = centres.ptr<float>(k);
				denAux = 0;
				for (int dim = 0; dim < ndims; dim++) {
					denDif  = cen_rowPtr[dim] - mag_rowPtr[dim];
					denAux += denDif * denDif;
				}
				denAux = std::max(denAux,(float)0.000001);	// Handle division by 0
				den += 1/(denAux*denAux);
			}

			// Assign calculated membership to its position
			map_rowPtr[col] = num/den;
			
		}
	}
	return map;
}


std::vector<cv::KeyPoint> own::OwnFeatureMaps::detectKeypointsInMap(int centIdx) {

	if (featureMaps.empty())
		throw std::runtime_error("Error: featureMaps is empty, no image loaded");

	std::vector<cv::KeyPoint> toReturn;

	nonMaximaSuppression(featureMaps[centIdx], 2, toReturn, thresh, kernSize, centIdx);

	return toReturn;
}



void own::OwnFeatureMaps::initCentreMat() {


	centres = cv::Mat::zeros(K, M*N, CV_32FC1);

	float* rowPtr;

	for (int row = 0; row < K; row++) {
		rowPtr = centres.ptr<float>(row);
		for (int col = 0; col < M*N; col++) {
			rowPtr[col] = own::params::centres::centres[row][col];
		}
	}
}



void own::OwnFeatureMaps::getFeatureMaps(std::vector<cv::Mat>& toReturn) {

	toReturn = featureMaps;
}



own::OwnFeatureMaps::~OwnFeatureMaps(void) {

	delete bfm;
}



//---------------------------------------------------------------------------------------------------------------------
//-----------------------------OwnFeatureDetector----------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

// Constructor
own::OwnFeatureDetector::OwnFeatureDetector(float thresh, int M, int N, int K, int kernSize) {

	this->thresh	= thresh;
	this->M 	= M;
	this->N		= N;
	this->K		= K;
	this->kernSize	= kernSize;
	
	fm = new own::OwnFeatureMaps(thresh, M, N, K, kernSize);
}


// Methods
void own::OwnFeatureDetector::getFeatureMaps(std::vector<cv::Mat>& featureMaps) {

	fm->getFeatureMaps(featureMaps);
}


void own::OwnFeatureDetector::detectImpl(const cv::Mat& image,
					std::vector<cv::KeyPoint>& keypoints,
					const cv::Mat& mask) const {

	fm->detectKeypoints(keypoints, image);
}

own::OwnFeatureDetector::~OwnFeatureDetector(void) {

	delete fm;
}
