
#include <opencv2/opencv.hpp>
#include <iostream>
#include <bfm.h>


int main (int argc, char **argv) {

	cv::Mat image = cv::imread("../res/images/im.jpg", CV_LOAD_IMAGE_COLOR);

	if (image.empty()) {
		std::cout << "Image not found!!" << std::endl;
		return -1;
	}

	cv::Mat magList;
	own::BFM bfm(5,8,4,50);
	bfm.fillMagList(image, magList);


	cv::Mat tempMagList;
	cv::transpose(magList, tempMagList);

	cv::Mat toShow0(image.size(), CV_32FC1, tempMagList.ptr<float>(7));
	cv::Mat toShow2(image.size(), CV_32FC1, tempMagList.ptr<float>(15));
	cv::Mat toShow4(image.size(), CV_32FC1, tempMagList.ptr<float>(23));
	cv::Mat toShow7(image.size(), CV_32FC1, tempMagList.ptr<float>(31));
	


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

