/*
	Author: Jaime Lomeli-R.
	Date:	29th of August, 2016

	This code if for generating the local Bessel Fourier moments of all images pointed by in the file listfile.txt
	
	NOTE: This code does not check for the validity of the file listfile.txt, behaivour is unknown if the file has the wrong format.

*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <bfm.h>


#define DEFAULT_PATH	"../res/images/dataset/listfile.txt"


int main(int argc, char *argv[]) {
	
	std::ifstream infile;

	if (argc > 2) {
		std::cout << "Too many parameters" << std::endl;
	}
	else if (argc == 2) {
		std::cout << "Path to listfile.txt: " << argv[1] << std::endl;
		infile.open(argv[1]);
		if (!infile.good()) {
			std::cout << "listfile.txt not found or is empty..." << std::endl;
			infile.close();

			std::cout << "Using default path to listfile.txt" << std::endl;
			infile.open(DEFAULT_PATH);
			if (!infile.good()) {
				std::cout << "Default listfile.txt not found or is empty..." << std::endl;
				std::cout << "Cannot continue..." << std::endl;
				infile.close();
				return -1;
			}
		}
	}
	else {
		std::cout << "Using default path to listfile.txt" << std::endl;
		infile.open(DEFAULT_PATH);
		if (!infile.good()) {
			std::cout << "Default listfile.txt not found or is empty..." << std::endl;
			std::cout << "Cannot continue..." << std::endl;
			infile.close();
			return -1;
		}
	}


	std::string  fileName;	
	cv::Mat magList;
	own::BFM bfm(5,8,4,32);
	cv::Mat image;

	while (std::getline(infile,fileName)) {
	
		std::cout << fileName << "...    ";
		
		image = cv::imread(fileName, CV_LOAD_IMAGE_COLOR);

		if (image.empty()) {
			std::cout << "Image not found..." << std::endl;
			continue;
		}

		bfm.fillMagList(image, magList);
		
			
		cv::FileStorage fs("../res/bfm/test.xml", cv::FileStorage::WRITE);

		fs << "info" << "[";
		fs << "dummy";
		fs << "]";
	
		fs << "magList" << magList;
	
		fs.release();

		std::cout << "File wrote..." << std::endl;
	}

	infile.close();

	cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
	cv::imshow("Image", image);

	cv::waitKey(0);

	return 0;
}

