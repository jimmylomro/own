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
*/

#ifndef _OWN_H_
#define _OWN_H_


#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <bfm.h>


#ifndef M_PI
	#define M_PI 3.141592653589793
#endif


namespace own{

	const int MAX_EXPECTED_KEYPOINTS_PER_MAP = 1000;

	class CV_EXPORTS OwnFeatureMaps {
		public:
			OwnFeatureMaps(float thresh = 0.5, int M = 8, int N = 1, int K = 30, int kernSize = 32);
			~OwnFeatureMaps(void);
			
			// this function creates the feature maps
			void createFeatureMaps(const cv::Mat& image);

			// this function detects the keypoints in all feature maps
			void detectKeypoints(std::vector<cv::KeyPoint>& keypoints, const cv::Mat& image);
			
			// accessor for the feature maps
			void getFeatureMaps(std::vector<cv::Mat>& toReturn);
	
		
		private:

			float	thresh;
			int	M, N, K, kernSize;
			
			BFM *bfm;

			// Parameter initialisation
			cv::Mat centres;
			std::vector<cv::Mat> re_filters;
			std::vector<cv::Mat> im_filters;

			// Mats containing the memberships for each centre
			std::vector<cv::Mat> featureMaps;
			// List of the magnitudes of the complex filters, this matrix has nPixels rows and M columns
			cv::Mat magList;

			void initCentreMat();
			
			cv::Mat createOneFeatureMap(int centIdx, int rows, int cols);
			std::vector<cv::KeyPoint> detectKeypointsInMap(int centIdx);

	};
	
	
	class CV_EXPORTS_W OwnFeatureDetector : public cv::Feature2D {
	public:

		static cv::Ptr<OwnFeatureDetector> create(float thresh = 0.5, int M = 8, int N = 1, int K = 30, int kernSize = 32);
    	virtual void getFeatureMaps(std::vector<cv::Mat>& featureMaps) = 0;

    	virtual void detect(const cv::Mat& image,
				std::vector<cv::KeyPoint>& keypoints,
				const cv::Mat& mask=cv::Mat() ) = 0;

	};

}

#endif // _OWN_H_
