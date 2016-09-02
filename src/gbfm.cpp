/*
	Author: Jaime Lomeli-R.
	Date:	29th of August, 2016

	This code if for generating the local Bessel Fourier moments of all images pointed by in the file listfile.txt
	
	NOTE: This code does not check for the validity of the file listfile.txt, behaivour is unknown if the file has the wrong format.

*/

#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <bfm.h>


#define DEFAULT_LIST_PATH	"../res/dataset/listfile.txt"
#define DEFAULT_OUTPUT_PATH	"../output/"


int main(int argc, char *argv[]) {
	
	std::ifstream	infile;
	std::string	infilePath;
	std::string	outputPath(DEFAULT_OUTPUT_PATH);

	if (argc > 2) {
		std::cout << "Too many parameters..." << std::endl;
	}
	else if (argc == 2) {
	
		infilePath = std::string(argv[1]);
		
		std::cout << "Path to listfile.txt: " << infilePath << std::endl;
		infile.open(infilePath.c_str());
		if (!infile.good()) {

			infile.close();
			infilePath = std::string(DEFAULT_LIST_PATH);
			std::cout << "listfile.txt not found or is empty..." << std::endl;
			std::cout << "Using default path to listfile.txt" << std::endl;
			infile.open(infilePath.c_str());
			if (!infile.good()) {
				std::cout << "Default listfile.txt not found or is empty..." << std::endl;
				std::cout << "Cannot continue..." << std::endl;
				infile.close();
				return -1;
			}
		}
	}
	else {
		infilePath = std::string(DEFAULT_LIST_PATH);
		std::cout << "Using default path to listfile.txt" << std::endl;
		infile.open(infilePath.c_str());
		if (!infile.good()) {
			std::cout << "Default listfile.txt not found or is empty..." << std::endl;
			std::cout << "Cannot continue..." << std::endl;
			infile.close();
			return -1;
		}
	}


	struct stat info;
	std::string  fileName;
	std::string  outFileName;
	std::string  dirName;
	std::string  fullPath;
	cv::Mat magList;
	own::BFM bfm(5,8,4,32);
	cv::Mat image;


	if( stat(outputPath.c_str(), &info ) == 0) {
		std::cout << "Output directory exists, deleteing it now" << std::endl;
		if (system((std::string("rm -rf ") + outputPath).c_str())) {
			std::cout << "Something went wrong..." << std::endl;
			return -1;
		}
	}


	std::cout << "Creating output directory" << std::endl;
	if (system((std::string("mkdir ") + outputPath).c_str())) {
		std::cout << "Something went wrong..." << std::endl;
		std::cout << "Cannot continue..." << std::endl;
		return -1;
	}

	while (std::getline(infile,fullPath)) {
	
		std::size_t splitIdxName = fullPath.find_last_of("/\\");
		fileName = fullPath.substr(splitIdxName+1);
		std::size_t splitIdxDir = fullPath.substr(0,splitIdxName).find_last_of("/\\");
		dirName  = fullPath.substr(splitIdxDir+1,splitIdxName-splitIdxDir);
		
		if( stat((outputPath + dirName).c_str(), &info ) != 0) {
			std::cout << "Creating directory " << outputPath + dirName << std::endl;
			if (system((std::string("mkdir ") + outputPath + dirName).c_str())) {
				std::cout << "Something went wrong creating output direcoty..." << std::endl;
				std::cout << "Cannot continue..." << std::endl;
				return -1;
			}
		}


		std::cout << fullPath << "...    ";
		
		image = cv::imread(fullPath, CV_LOAD_IMAGE_COLOR);
		if (image.empty()) {
			std::cout << "Image not found..." << std::endl;
			continue;
		}
		bfm.fillMagList(image, magList);

		std::size_t splitIdxExt = fileName.find_last_of(".");
		outFileName = fileName.substr(0,splitIdxExt) + std::string(".xml");
		cv::FileStorage fs((outputPath + dirName + outFileName).c_str(), cv::FileStorage::WRITE);
		fs << "info" << "[";
		fs << "dummy";
		fs << "]";
		fs << "magList" << magList;
		fs.release();

		std::cout << "File wrote..." << std::endl;
	}

	infile.close();

	std::cout << "Done!!" << std::endl;

	return 0;
}

