## Unsupervised learning of rotation invariant structures
___

This code uses opencv 3 libraries


### Compilation

CMake and basic compilation tools are needed for this to work.

```
git clone https://github.com/jimmylomro/own.git
cd own
mkdir build
cd build
cmake ..
make
```

If the compilation goes well, directories with binaries and static link libraries should appear.


### Usage

There are several binaries in this project:

- bfm_example: This file shows how to calculate the local Bessel-Fourier moments of an image. This uses the image 'own/res/im.jpg'.
- own_example: This file shows how to calculate the features. This file uses the keypoint models in bfm_centres_M.N_K.h
where M, N and K are the parameters described in the publication. This uses the image 'own/res/im.jpg'.
- own_train: This file trains the keypoint models using K-means clustering. To use this file you will have to create some directories:
	1. own/res/dataset -> This directory must contain a file named 'listfile.txt' containing the path of each of the images to be used for training, one path per line and no other lines are accepted.
	2. own/res/output  -> The output of the training is a file in this directory called 'centres.xml', this is a file containing the cv::Mat of the learned centroids with the opencv tool shown [here](http://docs.opencv.org/2.4/doc/tutorials/core/file_input_output_with_xml_yml/file_input_output_with_xml_yml.html). To use these centres you will have to put them in a header file like the 'bfm_centres_M.N_K.h' and include them in your implementation.


This code has been tested in Ubuntu 16.04 and OSX 10.11


### MATLAB

The Matlab code is only an example and may be tricky to compile, this functionality has only been tested in OSX 10.11.

___

#### Author:
Jaime Lomeli-R. (2016)
University of Southampton
jaime.lomeli.rodriguez@gmail.com
