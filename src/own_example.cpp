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

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>

#include <own.h>

using namespace std;


int main(int argc, char ** argv) {

	int K = 32;

	cv::Mat image = cv::imread("../res/im.jpg");
	cv::namedWindow("Orig");
	cv::imshow("Orig", image);

	std::vector<cv::KeyPoint> keypoints;
	
	cv::Ptr<own::OwnFeatureDetector> detector = own::OwnFeatureDetector::create(0.5,8,4,K,16);
	
	clock_t begin = clock();
	detector->detect(image,keypoints);
	clock_t end = clock();
	
	double secs = double(end-begin)/CLOCKS_PER_SEC;
	cout << "Time for " << K << " feature maps = " << secs << endl;
	cout << "Number of keypoints	 = " << keypoints.size() << endl;

	std::vector<cv::Mat> maps;
	detector->getFeatureMaps(maps);

	for (int k = 0; k < K; k++) {
		std::stringstream sstm;
		sstm << k;
		std::string str = sstm.str();
		cv::namedWindow(str);
		cv::imshow(str, maps[k]);

	}

	cv::waitKey(0);

	return 0;
}
