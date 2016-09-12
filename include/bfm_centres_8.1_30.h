/*
    Copyright (C) 2016 Jaime Lomeli-R. Univesity of Southampton

    This file is part of OWN.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
       * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
       * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.
       * Neither the name of the ASL nor the names of its contributors may be
         used to endorse or promote products derived from this software without
         specific prior written permission.

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

OwnFeatureMaps:
    There are two ways of creating the feature maps, you could either call detectKeypoints and send a valid image
    or you could first call createFeatureMaps. The first method will also detect the keypoints.
    If detectKeypoints is called without an image argument and createFeatureMaps has not been called (i.e. featureMaps is empty)
    the function will throw an error.
*/


#ifndef _BFM_CENTRES_H_
#define _BFM_CENTRES_H_

namespace own{
	namespace params{
		namespace centres{

		const int MAX_K		= 30;

		float centres[30][8] =
		{	{0.4409,0.2610,0.1166,0.0450,0.0473,0.0386,0.0253,0.0252},
			{0.1236,0.4819,0.1056,0.0946,0.0625,0.0536,0.0414,0.0368},
			{0.0064,0.0058,0.0053,0.4757,0.0044,0.0040,0.0036,0.4948},
			{0.4273,0.1924,0.0726,0.1017,0.0668,0.0526,0.0439,0.0426},
			{0.6148,0.0660,0.1302,0.0484,0.0529,0.0321,0.0306,0.0250},
			{0.7420,0.0788,0.0572,0.0390,0.0257,0.0197,0.0160,0.0216},
			{0.3190,0.1354,0.1031,0.0829,0.1211,0.0899,0.0784,0.0701},
			{0.6193,0.1595,0.0598,0.0563,0.0320,0.0284,0.0217,0.0230},
			{0.3876,0.3131,0.0668,0.0700,0.0587,0.0392,0.0342,0.0304},
			{0.4735,0.0867,0.1797,0.0684,0.0677,0.0484,0.0407,0.0349},
			{0.3312,0.2755,0.1621,0.0690,0.0501,0.0449,0.0370,0.0303},
			{0.1462,0.1247,0.1201,0.1158,0.1374,0.1307,0.1127,0.1125},
			{0.1280,0.3295,0.0937,0.1713,0.0796,0.0838,0.0584,0.0558},
			{0.1226,0.0887,0.0682,0.2971,0.0460,0.0395,0.0341,0.3038},
			{0.5204,0.2406,0.0476,0.0681,0.0440,0.0285,0.0286,0.0221},
			{0.2707,0.3830,0.0819,0.0828,0.0618,0.0468,0.0392,0.0340},
			{0.1273,0.1677,0.1612,0.2116,0.1059,0.0892,0.0682,0.0690},
			{0.2031,0.2397,0.1209,0.0958,0.1245,0.0798,0.0755,0.0607},
			{0.2508,0.2139,0.2058,0.1119,0.0777,0.0546,0.0445,0.0408},
			{0.1065,0.2834,0.2331,0.1028,0.0953,0.0701,0.0586,0.0502},
			{0.3221,0.0990,0.2356,0.0917,0.0852,0.0671,0.0515,0.0477},
			{0.5251,0.1582,0.1090,0.0678,0.0426,0.0412,0.0277,0.0284},
			{0.3021,0.2353,0.0881,0.1403,0.0706,0.0647,0.0504,0.0486},
			{0.1584,0.1247,0.2890,0.1089,0.1141,0.0798,0.0673,0.0578},
			{0.2127,0.3321,0.1787,0.0816,0.0651,0.0510,0.0422,0.0366},
			{0.3957,0.0954,0.1143,0.1459,0.0741,0.0681,0.0541,0.0524},
			{0.2547,0.1136,0.1386,0.1873,0.0960,0.0818,0.0639,0.0642},
			{0.3142,0.0900,0.0590,0.2158,0.0377,0.0319,0.0275,0.2239},
			{0.3847,0.1799,0.1708,0.0776,0.0629,0.0494,0.0398,0.0350},
			{0.5223,0.0884,0.0692,0.1047,0.0629,0.0533,0.0428,0.0565}
		};
	}
	}
}

#endif // _BFM_CENTRES_H_
