/*=================================================================
 *
 *  Author: Jaime Lomeli-R.
 *
 * The calling syntax is:
 *
 *	varargout = own_interface(subfunction, morevarargin)
 *
 *      where subfunction is to be used in order:
 *
 *      'init'        Initialize brisk. Optionally pass arguments to 
 *                    set properties (see below). 
 *                    Attention: this will create the pattern look-up table,
 *                    so this may take some fraction of a second. 
 *                    Do not rerun!
 *
 *      'set'         Set properties. The following may be set:
 *                    '-threshold'    FAST/AGAST detection threshold.
 *                                  The default value is 60.
 *                    '-kernSize'      No. kernSize for the detection.
 *                                  The default value is 4.
 *                    '-patternScale' Scale factor for the BRISK pattern.
 *                                  The default value is 1.0.
 *                    '-type'         BRISK special type 'S', 'U', 'SU'.
 *                                  By default, the standard BRISK is used.
 *                                    See [1] for explanations on this.
 *                    Attention: if the patternScale or the type is reset, 
 *                    the pattern will be regenerated, which is time-
 *                    consuming!
 *
 *      'loadImage'   Load an image either from Matlab workspace by passing
 *                    a UINT8 Matrix as a second argument, or by specifying 
 *                    a path to an image:
 *                        brisk('loadImage',imread('path/to/image'));
 *                        brisk('loadImage','path/to/image');
 *
 *      'detect'      Detect the keypoints. Optionally get the points back:
 *                      brisk('detect');
 *                      keyPoints=brisk('detect');
 *
 *      'image'       Returns the currently used gray-scale image
 *                      image=brisk('image');
 *
 *      'terminate'   Free the memory.
 *
 *
 * 
 * This file is part of OWN.
 * 
 * Copyright (C) 2011  The Autonomous Systems Lab (ASL), ETH Zurich,
 * Stefan Leutenegger, Simon Lynen and Margarita Chli.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the ASL nor the names of its contributors may be 
 *       used to endorse or promote products derived from this software without 
 *       specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *=================================================================*/

#include "own_interface.h"
#include <cstring>
#include <string>

// helper function to test input args
bool mxIsScalarNonComplexDouble(const mxArray* arg){
    mwSize mrows,ncols;
    mrows = mxGetM(arg);
    ncols = mxGetN(arg);
    if( !mxIsDouble(arg) || mxIsComplex(arg) ||
      !(mrows==1 && ncols==1) )
        return false;
    return true;
}

// constructor: set pointers to 0
OwnInterface::OwnInterface( int nlhs, mxArray *plhs[], 
            int nrhs, const mxArray *prhs[] ){
    // init default parameters
    threshold   = 0.5;
    kernSize    = 31;
    nMaps       = 15;
    
    // init the pointers
    detectorPtr = 0;
    
    // use the inputs to set
    set(nlhs, plhs, nrhs, prhs);
}

OwnInterface::~OwnInterface(){
    // cleanup pointers
    if(detectorPtr!=0) delete detectorPtr;
}

// reset kernSize / threshold / Pattern
inline void OwnInterface::set(int nlhs, mxArray *plhs[], 
            int nrhs, const mxArray *prhs[]){
    if((nrhs-1)%2!=0) mexErrMsgTxt("Bad input.");
    
    // remember what to re-initialize
    bool initDetector = false;
    bool scaleSet     = false;
    
    for(int i=1; i<nrhs; i+=2){
        // parse option
        char* str2=mxArrayToString(prhs[i]);
        if(!(mxGetClassID(prhs[i])==mxCHAR_CLASS))
            mexErrMsgTxt("Bad input.");
        if(strcmp(str2,"threshold")==0){
            if(!mxIsScalarNonComplexDouble(prhs[i+1]))
                mexErrMsgTxt("Bad input.");
            double* x=mxGetPr(prhs[i+1]);
            // bound
            if(*x<0) threshold =0;
            //else if(*x>1) threshold = 1;
            else threshold=float(*x);
            initDetector=true;
        }
        else if(strcmp(str2,"nMaps")==0){
            if(!mxIsScalarNonComplexDouble(prhs[i+1]))
                mexErrMsgTxt("Bad input.");
            double* x=mxGetPr(prhs[i+1]);
            if(*x<0) nMaps=0;
            else if(*x>64) nMaps = 64;
            else nMaps=int(*x);
            initDetector=true;
        }
        else if(strcmp(str2,"kernSize")==0){
            if(!mxIsScalarNonComplexDouble(prhs[i+1]))
                mexErrMsgTxt("Bad input.");
            double* x=mxGetPr(prhs[i+1]);
            if(*x<0) kernSize=0;
            else if(*x>128) kernSize = 128;
            else kernSize=int(*x);
            initDetector=true;
        }
        else mexErrMsgTxt("Unrecognized input option.");
    }
    // reset if requested
    if(initDetector||detectorPtr==0){
        if(detectorPtr!=0)
            delete detectorPtr;
        
        detectorPtr = new own::OwnFeatureDetector(threshold, 8, 4, nMaps, kernSize);
    }
}



// load an image
inline void OwnInterface::loadImage( int nlhs, mxArray *plhs[], 
            int nrhs, const mxArray *prhs[] ){
    if(nrhs<2) 
        mexErrMsgTxt("No image passed.");
    if((mxGetClassID(prhs[1]) == mxUINT8_CLASS)){
        // image dimensions
        int M = mxGetM(prhs[1]);
        int N = mxGetN(prhs[1]);
        mwSize dim = mxGetNumberOfDimensions(prhs[1]);
        if(dim==3){
            // this means we need to merge the channels.
            uchar *data = (uchar*) mxGetData(prhs[1]);
            std::vector<cv::Mat> BGR;
            BGR.push_back(cv::Mat(N/3, M, CV_8U, data+2*N*M/3));
            BGR.push_back(cv::Mat(N/3, M, CV_8U, data+N*M/3));
            BGR.push_back(cv::Mat(N/3, M, CV_8U, data));

            // merge into one BGR matrix
            cv::Mat imageBGR;
            cv::Mat auxImg;
            cv::merge(BGR,imageBGR);
            // color conversion
            cv::cvtColor(imageBGR, auxImg, CV_BGR2GRAY);
            
            // transpose
            img = auxImg.t();
        }
        else if(dim == 2){// cast image to a cv::Mat
            cv::Mat auxImg;
            uchar* data = (uchar*) mxGetData(prhs[1]); 
            auxImg = cv::Mat(N, M, CV_8U, data);
            
            // transpose
            img = auxImg.t();
        }
        else
            mexErrMsgTxt("Image dimension must be 2 or 3.");
    }
    else if((mxGetClassID(prhs[1])==mxCHAR_CLASS)){
        char* fname = mxArrayToString(prhs[1]);
        img = cv::imread(fname,0); // forcing gray
        if(img.data == 0){
            mexPrintf("%s ",fname);
            mexErrMsgTxt("Image could not be loaded.");
        }
    }
    else
        mexErrMsgTxt("Pass an UINT8_T image matrix or a path.");
}


// detection
inline void OwnInterface::detect( int nlhs, mxArray *plhs[], 
            int nrhs, const mxArray *prhs[] ){
    if(img.empty()) 
            mexErrMsgTxt("Currently no image loaded.");
        
    // actual detection step
    assert(detectorPtr);
    detectorPtr->detect(img, keypoints);

    // send the keypoints to the user, if he wants it
    // allocate plhs
    if(nlhs>=1){
        const int keypoint_size=keypoints.size();
        mxArray* tmp;
        tmp=mxCreateDoubleMatrix(6,keypoint_size,mxREAL);
        double *ptr=mxGetPr(tmp);
        // fill it - attention: in Matlab, memory is transposed...
        for(int k=0; k<keypoint_size; k++){
            const int k6=6*k;
            ptr[k6]=keypoints[k].pt.x;
            ptr[k6+1]=keypoints[k].pt.y;
            ptr[k6+2]=keypoints[k].size;
            ptr[k6+3]=-1;
            ptr[k6+4]=keypoints[k].class_id;
            ptr[k6+5]=keypoints[k].response;
        }
        
        // finally, re-transpose for better readibility:
        mexCallMATLAB(1, plhs, 1, &tmp, "transpose");
    }   
}


// if feature maps are requested
inline void OwnInterface::getFeatureMaps( int nlhs, mxArray *plhs[], 
        int nrhs, const mxArray *prhs[] ){
    
    if(nlhs!=1)
        mexErrMsgTxt("No output variable specified.");
    
    if(img.empty())
        mexErrMsgTxt("No image loaded.");
    
    std::vector<cv::Mat> maps;
    
    detectorPtr->getFeatureMaps(maps);
    
    const int N = img.rows;
    const int M = img.cols;
    int dim[3] = {M,N,maps.size()};

    mxArray* tmp = mxCreateNumericArray(3,dim,mxDOUBLE_CLASS,mxREAL);
    double* data =(double*) mxGetData(tmp);
    
    // copy - kind of dumb, but necessary due to the matlab memory management
    for (int k = 0; k < maps.size(); k++) {
        cv::Mat tempMat;
        maps[k].convertTo(tempMat, CV_64FC1);
        memcpy(data,tempMat.ptr<double>(),dim[0]*dim[1]*sizeof(double));
        data += dim[0]*dim[1];
    }

    plhs[0] = tmp;
    // transpose for better readibility
    //mexCallMATLAB(1, &plhs[0], 1, &tmp, "transpose");
}
    

// grayImage access
inline void OwnInterface::image( int nlhs, mxArray *plhs[], 
        int nrhs, const mxArray *prhs[] ){
    if(nlhs!=1)
        mexErrMsgTxt("No output variable specified.");
    if(nrhs!=1)
        mexErrMsgTxt("bad input.");
    if(img.empty())
        mexErrMsgTxt("No image loaded.");
    int dim[2];

    // depending on whether or not the image was imported from Matlab 
    // workspace, it needs to be transposed or not
    // must be transposed
    dim[0]=img.cols;
    dim[1]=img.rows;
    mxArray* tmp=mxCreateNumericArray(2,dim,mxUINT8_CLASS,mxREAL);
    uchar* dst=(uchar*)mxGetData(tmp);
    memcpy(dst,img.data,img.cols*img.rows);
    mexCallMATLAB(1, plhs, 1, &tmp, "transpose");
}


    
// The interface object
OwnInterface* interfacePtr = 0;

// this is the actual (single) entry point:
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )     
{
    // user must provide at least one right-hand argument:
	if(nrhs < 1) mexErrMsgTxt("No input specified.");
    // and this must be a string
    if(!(mxGetClassID(prhs[0])==mxCHAR_CLASS)) mexErrMsgTxt("Bad input.");
    // parse the first input argument:
    char* str=mxArrayToString(prhs[0]);
    if(strcmp(str,"init")==0) {
        if(!interfacePtr) {
            interfacePtr = new OwnInterface(nlhs, plhs, nrhs, prhs);
        }
        else{
            mexErrMsgTxt("Interface is already initialized.");
        }
    }
    else if(strcmp(str,"set")==0){
        interfacePtr->set(nlhs, plhs, nrhs, prhs);
    }
    else if(strcmp(str,"loadImage")==0){
        // init if necessary
        if(!interfacePtr) {
            interfacePtr = new OwnInterface(nlhs, plhs, 1, prhs);
        }
        interfacePtr->loadImage(nlhs, plhs, nrhs, prhs);
    }
    else if(strcmp(str,"detect")==0){
        // force initialized
        if(!interfacePtr) {
            mexErrMsgTxt("Not initialized, no image loaded.");
        }
        interfacePtr->detect(nlhs, plhs, nrhs, prhs);
    }
    else if(strcmp(str,"image")==0) {
        // error if no image loaded
        if(!interfacePtr) {
           mexErrMsgTxt("Not initialized, no image loaded.");
        }
        interfacePtr->image(nlhs,plhs,nrhs,prhs);
    }
    else if(strcmp(str,"terminate")==0) {
        if(interfacePtr) {
            delete interfacePtr;
            interfacePtr = 0;
        }
        else{
            mexErrMsgTxt("Brisk was not initialized anyways.");
        }
    }
    else if(strcmp(str,"getFeatureMaps")==0) {
        if(!interfacePtr)
            interfacePtr = new OwnInterface(nlhs, plhs, 1, prhs);
        interfacePtr->getFeatureMaps(nlhs,plhs,nrhs,prhs);
    }
    else{
        mexErrMsgTxt("Unrecognized input.");
    }
}
