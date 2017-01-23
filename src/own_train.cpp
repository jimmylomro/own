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
		- This code creates a training matrix of a fixed size of N_TRAININGSAMPS by 8*4
		This code does not check if the file paths read from the listfile.txt are good, it only
		assumes the .xml files exist. Behaviour if one of the files does not exist or is corrupted
		is unknown.
*/

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>
#include <string>
#include <bfm.h>


#define DEFAULT_LIST_PATH	"../res/dataset/listfile.txt"
#define DEFAULT_OUTPUT_FILE	"../res/output/centres.xml"
#define N_TRAINING_SAMPS	500000
#define M			8
#define N			4

int main(int argc, char **argv) {


//-------------------------------------------------- Init input listfile.txt
	std::ifstream	infile;
	std::string	infilePath;

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



//-------------------------------------------------- Output file
// This section must be merged with the one above when handling output file as a parameter.
	struct stat info;
	std::string outputFile(DEFAULT_OUTPUT_FILE);
	std::size_t pathIdx = outputFile.find_last_of("/");
	std::string outputPath = outputFile.substr(0,pathIdx);
	

	if( stat(outputPath.c_str(), &info ) != 0) {
		std::cout << "Output directory does not exist..." << std::endl;
		std::cout << "Cannot continue..." << std::endl;
		return -1;
	}

	std::cout << "Using default output file" << std::endl;


//-------------------------------------------------- Read all file paths into vector of strings
	std::string			auxFilePath;
	std::vector<std::string> 	filePaths;
	filePaths.reserve(1500);

	while (std::getline(infile,auxFilePath))
		filePaths.push_back(auxFilePath);



//-------------------------------------------------- Init random number of samples from each image
	int auxIdx;
	cv::Mat sampsFromFile = cv::Mat::zeros(filePaths.size(),1,CV_16UC1);
	srand(time(NULL));
	
	for (int sampIdx = 0; sampIdx < N_TRAINING_SAMPS; sampIdx++) {
		auxIdx = rand() % filePaths.size();
		sampsFromFile.at<unsigned short>(auxIdx)++;
	}



//-------------------------------------------------- Generate training matrix	
	cv::Mat image;
	cv::Mat	magList;
	cv::Mat	trainingMat(N_TRAINING_SAMPS,M*N,CV_32FC1);
	own::BFM bfm(5,M,N,32);

	float acum;
	int numTries;
	int numTriesOverflow;
	int numTriesOverflowTotal = 0;
	int sampsTotal = 0;
	int trainingRowIdx = 0;
	float *trainingRowPtr, *magListRowPtr;


	std::cout << "Creating training matrix from " << filePaths.size() << " images..." << std::endl;

	for (int fileIdx = 0; fileIdx < filePaths.size(); fileIdx++) {

		std::cout << filePaths[fileIdx] << "... ";

		image = cv::imread(filePaths[fileIdx], CV_LOAD_IMAGE_COLOR);	// Read image
		if (image.empty()) {
			std::cout << "Image not found..." << std::endl;
			continue;	// Skip image if there is a read error
		}
		bfm.fillMagList(image, magList);
		
		numTriesOverflow = 0;
		for (int sampIdx = 0; sampIdx < sampsFromFile.at<unsigned short>(fileIdx); sampIdx++) {
			numTries = 0;
			trainingRowPtr = trainingMat.ptr<float>(trainingRowIdx++);
			while (numTries < 15) {
				numTries++;
				acum = 0;
				auxIdx = rand() % magList.rows;
				magListRowPtr = magList.ptr<float>(auxIdx);
				for (int colIdx = 0; colIdx < magList.cols; colIdx++) {
					acum += magListRowPtr[colIdx];
					trainingRowPtr[colIdx] = magListRowPtr[colIdx];
				}
				if (acum > 0.5) break;
				if (numTries == 15) numTriesOverflow++;
			}
		}
		
		sampsTotal = sampsTotal + sampsFromFile.at<unsigned short>(fileIdx) - numTriesOverflow;
		numTriesOverflowTotal += numTriesOverflow;
		std::cout << sampsFromFile.at<unsigned short>(fileIdx) << " (" << numTriesOverflow << ")" << std::endl;
	}
	
	std::cout << "Total of successful samps = " << sampsTotal << std::endl;
	std::cout << "Total of tries overflow   = " << numTriesOverflowTotal << std::endl;



//-------------------------------------------------- Run K-Means clustering on the created training matrix
	cv::Mat centres;
	cv::Mat labels;
	
	std::cout << "Running K-Means in training matrix..." << std::endl;
	
	cv::kmeans(trainingMat, 64, labels, cv::TermCriteria(cv::TermCriteria::COUNT, 1000, 0), 4, cv::KMEANS_RANDOM_CENTERS, centres);

	std::cout << "Writting output file" << std::endl;
	cv::FileStorage ofs("../res/output/centres.xml", cv::FileStorage::WRITE);
	ofs << "centres" << centres;
	ofs.release();	


	return 0;
}

