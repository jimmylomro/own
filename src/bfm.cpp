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
#include <bfm.h>
#include <bfm_filters.h>


//---------------------------------------- Constructors
own::BFM::BFM(float thresh, int M, int N, int kSize) {


	// Bound M and N to fit the maximum number of filters available
	if (M > own::params::filters::MAX_M) M = own::params::filters::MAX_M;
	if (N > own::params::filters::MAX_N) N = own::params::filters::MAX_N;

	this->thresh	= thresh;
	this->M		= M;
	this->N		= N;
	this->kSize	= kSize;

	initKernels();
}



//---------------------------------------- Methods

void own::BFM::fillMagList(const cv::Mat &image, cv::Mat &magList) {

	cv::Mat grayImage;

	assert(!image.empty());

    	// Convert the image to gray scale 
    	switch (image.type()) {

	case CV_8UC1:
		image.convertTo(grayImage, CV_32FC1, 1/255.0);
		break;

    	case CV_8UC3:
        	cvtColor(image, grayImage, CV_BGR2GRAY);
		grayImage.convertTo(grayImage, CV_32FC1, 1/255.0);
        	break;

    	case CV_32FC3:
        	cvtColor(image, grayImage, CV_BGR2GRAY);
		grayImage.convertTo(grayImage, CV_32FC1, 1/255.0);
        	break;

    	default:
        	grayImage = image.clone();
        	break;
    	}

    	assert(grayImage.type() == CV_32FC1);
    	assert(grayImage.isContinuous());

	int nDims = M*N;
	
	cv::Mat tempRe;
	cv::Mat tempIm;
	cv::Mat tempMag;
	cv::Mat tempMagList = cv::Mat::zeros(nDims, grayImage.rows*grayImage.cols, CV_32FC1);


	for (int i = 0; i < nDims; i++) {
		
		cv::filter2D(grayImage, tempRe, -1, re_filters[i]);
		cv::filter2D(grayImage, tempIm, -1, im_filters[i]);
		cv::magnitude(tempRe, tempIm, tempMag);

		assert(tempMag.type() == CV_32FC1);
		memcpy(tempMagList.ptr<float>(i), tempMag.data, grayImage.rows * grayImage.cols * sizeof(float));
	}

	cv::transpose(tempMagList, magList);
	
	float* rowPtr;
	float  E;

	for (int row = 0; row < magList.rows; row++) {
		
		E = 0;
		rowPtr = magList.ptr<float>(row);
		for (int i = 0; i < nDims; i++)
			E += rowPtr[i] * rowPtr[i];
		
		for (int i = 0; i < nDims; i++)
			if (E > thresh)
				rowPtr[i] /= sqrt(E);
			else
				rowPtr[i] = 0;
	}
}



void own::BFM::initKernels() {

	int nDims = M*N;
	
	re_filters.resize(nDims);
	im_filters.resize(nDims);

	for (int i = 0; i < nDims; i++) {

		cv::Mat origRe(own::params::filters::MAX_KERN_SIZE, own::params::filters::MAX_KERN_SIZE, CV_32FC1, own::params::filters::filters_re[i]);
		cv::Mat origIm(own::params::filters::MAX_KERN_SIZE, own::params::filters::MAX_KERN_SIZE, CV_32FC1, own::params::filters::filters_im[i]);
				
		cv::Mat newRe(kSize, kSize, CV_32FC1);
		cv::Mat newIm(kSize, kSize, CV_32FC1);

		cv::resize(origRe, newRe, newRe.size(), 0, 0, CV_INTER_CUBIC);
		cv::resize(origIm, newIm, newIm.size(), 0, 0, CV_INTER_CUBIC);

		re_filters[i] = newRe;
		im_filters[i] = newIm;
	}
}

