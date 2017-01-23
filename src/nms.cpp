/*
    This file is part of OWN.

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
    
    Author: Jaime Lomeli-R.

	Notes:
		- src must be of type CV_32FC1

	This is an adaptation of the code nonmaxsuppts.m by Peter Kovesi: http://www.peterkovesi.com/matlabfns/
*/

#include <stdio.h>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <nms.h>

using namespace cv;

void nonMaximaSuppression(Mat& src, const int sz, std::vector<KeyPoint> &keypoints, const float thresh, int kernSize, const int featID) {

	if (src.type() != CV_32FC1)
		return;

	// initialise variables
	const int M = src.rows;
	const int N = src.cols;

	Mat dilIm;

	int sze = 2*sz+1;                							// calculate size of dilation mask.
    	dilate(src, dilIm, getStructuringElement(MORPH_RECT,Size(sze,sze),Point(sz,sz)));	// Grey-scale dilate.

	float* rowPtrD;
	float* rowPtrS;
	float  valD;
	float  valS;

	for (int row = sz; row < M-sz; row++) {
		rowPtrD = dilIm.ptr<float>(row);
		rowPtrS = src.ptr<float>(row);
		for (int col = sz; col < N-sz; col++) {
			valD = rowPtrD[col];
			valS = rowPtrS[col];
			if (valS == valD && valS > thresh) {
				keypoints.push_back(KeyPoint(col,row,(float)kernSize,-1,valS,0,featID));
			}
		}
	}
}
