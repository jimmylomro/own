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
#include <bfm.h>


int main (int argc, char **argv) {

	cv::Mat image = cv::imread("../res/dataset/butterfly/image_0001.jpg", CV_LOAD_IMAGE_COLOR);

	if (image.empty()) {
		std::cout << "Image not found!!" << std::endl;
		return -1;
	}



	cv::Mat magList;

	//cv::FileStorage fs("../res/bfm_files/butterfly/image_0001.xml", cv::FileStorage::READ);
	//fs["magList"] >> magList;

    own::BFM bfm(5,8,4,32);
    bfm.fillMagList(image, magList);

	cv::Mat tempMagList;
	cv::transpose(magList, tempMagList);

	cv::Mat toShow0(image.size(), CV_32FC1, tempMagList.ptr<float>(0));
	cv::Mat toShow2(image.size(), CV_32FC1, tempMagList.ptr<float>(1));
	cv::Mat toShow4(image.size(), CV_32FC1, tempMagList.ptr<float>(2));
	cv::Mat toShow7(image.size(), CV_32FC1, tempMagList.ptr<float>(3));
	


	cv::namedWindow("Image0", CV_WINDOW_AUTOSIZE);
	cv::imshow("Image0", toShow0);

	cv::namedWindow("Image2", CV_WINDOW_AUTOSIZE);
	cv::imshow("Image2", toShow2);

	cv::namedWindow("Image4", CV_WINDOW_AUTOSIZE);
	cv::imshow("Image4", toShow4);

	cv::namedWindow("Image7", CV_WINDOW_AUTOSIZE);
	cv::imshow("Image7", toShow7);

	cv::waitKey(0);

	return 0;
}

