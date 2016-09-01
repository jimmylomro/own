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

    DESCRIPTION:
	BFM object generates a structurally normalised list of the magnitude of the local Bessel-Fourier Moments
	of an image. Each row of the magList is the vector result of a single pixel, therefore the size of
	magList is (nPixels x nDims). Bessel-Fourier filters are obtained from .xml files that should be placed
	in the directory res/bf-filters with the name format n-m.xml where n is the scaling and m is the
	rotation repetition. The variable (nDims = n*m) is tested against the number filters in the .xml files,
	if the dimensionality required exeeds the number of available filters nDims will be reduced to the
	number of available filters. Note that the filters in the .xml files have a size 128 x 128 and are
	rescaled using bicubic interpolation to the size required by the kSize variable.
	Filter indexes vary: (m = 1,...,M) and (n = 0,...,N-1)
	The structure of a row of magList (m,n) is (1,0),(2,0),...,(M,0),(1,1),(2,1),...,(M-1,N-1),(M,N-1)
*/

#ifndef _BFM_H_
#define _BFM_H_


#include <opencv2/core/core.hpp>




namespace own{

	class CV_EXPORTS BFM {

		public:
			BFM(float thresh = 5.0, int M = 8, int N = 1, int kSize = 32);

			// public methods	
			void fillMagList(const cv::Mat &image, cv::Mat &magList);
		
		private:

			// Parameter initialisation
			float	thresh;
			int	M, N, kSize;
			std::vector<cv::Mat> re_filters;
			std::vector<cv::Mat> im_filters;

			void initKernels();
	};
}

#endif // _BFM_H_

