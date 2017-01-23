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

		const int MAX_K		= 64;

		float centres[64][32] =
{
{0.33028,0.27325,0.26196,0.22813,0.19042,0.15586,0.12873,0.10965,0.50937,0.23513,0.18116,0.1511,0.1287,0.10928,0.095253,0.084053,0.21845,0.15211,0.12619,0.10783,0.096051,0.082312,0.073669,0.067252,0.14807,0.10767,0.094598,0.086409,0.075559,0.066924,0.059905,0.05766},
{0.099634,0.91219,0.061009,0.24327,0.040524,0.081717,0.027959,0.050865,0.051255,0.17734,0.03397,0.11341,0.025655,0.062073,0.019677,0.039603,0.030807,0.097654,0.021822,0.045713,0.01728,0.026925,0.014182,0.024357,0.022723,0.098295,0.017639,0.047634,0.012942,0.023136,0.011404,0.019474},
{0.21977,0.60832,0.16711,0.19013,0.10392,0.092821,0.072195,0.066046,0.18077,0.46719,0.13184,0.12187,0.080137,0.076929,0.057951,0.053106,0.14179,0.28169,0.096136,0.088295,0.063528,0.062269,0.048143,0.045253,0.10419,0.17234,0.070929,0.068059,0.049795,0.048244,0.03934,0.039181},
{0.28662,0.22378,0.50187,0.18126,0.22718,0.12007,0.10665,0.084853,0.55697,0.17827,0.1384,0.11544,0.10126,0.082368,0.069632,0.059964,0.15716,0.11162,0.091655,0.075917,0.070956,0.056076,0.051618,0.046591,0.10837,0.077176,0.068088,0.058585,0.055081,0.044995,0.042274,0.039214},
{0.45903,0.30327,0.23097,0.1531,0.085368,0.053413,0.051098,0.048591,0.4104,0.28901,0.1956,0.11072,0.055322,0.050087,0.051901,0.045114,0.33937,0.2227,0.13217,0.068904,0.048244,0.052236,0.048549,0.038611,0.24283,0.14764,0.079014,0.050561,0.047886,0.047929,0.039619,0.032184},
{0.27206,0.63636,0.19699,0.39231,0.16355,0.19552,0.12484,0.12717,0.18679,0.16325,0.13438,0.11232,0.098578,0.080265,0.07669,0.057566,0.13906,0.14642,0.099995,0.10298,0.078783,0.066251,0.063885,0.055168,0.10308,0.074669,0.074337,0.060962,0.058981,0.045231,0.048151,0.040281},
{0.56899,0.42333,0.20744,0.19938,0.1678,0.14399,0.12633,0.11317,0.1736,0.17198,0.18445,0.1568,0.1254,0.10959,0.099441,0.088517,0.16978,0.1548,0.13045,0.1127,0.094762,0.085554,0.075417,0.070364,0.13165,0.11145,0.094489,0.088493,0.073961,0.06608,0.060188,0.058374},
{0.60468,0.21859,0.23541,0.24572,0.14796,0.1404,0.11867,0.089844,0.20551,0.34368,0.20803,0.12787,0.11359,0.10384,0.078226,0.073087,0.19741,0.15658,0.11023,0.099023,0.082497,0.069312,0.062854,0.058037,0.11867,0.10275,0.081153,0.070811,0.061475,0.054434,0.047524,0.047742},
{0.33969,0.42546,0.31635,0.17585,0.1017,0.088289,0.077892,0.063559,0.3977,0.34766,0.19994,0.098953,0.080346,0.076587,0.063322,0.051149,0.28691,0.19551,0.098609,0.06978,0.07022,0.062251,0.049348,0.04087,0.15292,0.10177,0.061072,0.057558,0.056224,0.045508,0.035652,0.03557},
{0.39114,0.5405,0.3012,0.21733,0.24052,0.1194,0.16411,0.089367,0.28343,0.1117,0.18142,0.11416,0.1141,0.091041,0.078492,0.080894,0.19331,0.10384,0.1238,0.078383,0.085217,0.066539,0.062646,0.054402,0.11892,0.097527,0.074866,0.085437,0.058043,0.061302,0.048243,0.053955},
{0.28295,0.8118,0.14856,0.1113,0.085622,0.077749,0.053797,0.048529,0.16092,0.32097,0.088049,0.094079,0.056902,0.049935,0.040403,0.037006,0.092241,0.13389,0.054171,0.058983,0.037392,0.032398,0.028034,0.028579,0.057909,0.091816,0.036538,0.037246,0.027118,0.023117,0.020736,0.022064},
{0.90786,0.080705,0.048487,0.030211,0.020207,0.015797,0.013168,0.013926,0.33065,0.037081,0.026198,0.020769,0.013178,0.010763,0.0092439,0.012974,0.18575,0.023122,0.017011,0.018456,0.0093425,0.0081283,0.0071991,0.013201,0.12435,0.016778,0.012685,0.018382,0.0076038,0.0066409,0.0059611,0.013685},
{0.28305,0.80647,0.1612,0.31342,0.11049,0.12637,0.073703,0.070902,0.1569,0.09732,0.084437,0.083885,0.057544,0.056419,0.04471,0.04159,0.088699,0.112,0.054192,0.060173,0.039294,0.036435,0.031594,0.030167,0.062125,0.069441,0.042387,0.050053,0.033333,0.030739,0.02744,0.02496},
{0.29654,0.47304,0.51466,0.247,0.18299,0.14266,0.11204,0.092274,0.24001,0.23822,0.17585,0.12193,0.10296,0.08416,0.071614,0.063241,0.14257,0.12392,0.10379,0.081522,0.069267,0.059608,0.052894,0.048113,0.095014,0.085265,0.07541,0.061055,0.053101,0.046061,0.041838,0.039698},
{0.55965,0.23532,0.45517,0.15352,0.19812,0.14472,0.11708,0.11455,0.2858,0.187,0.14962,0.15211,0.10132,0.09658,0.08591,0.071333,0.15451,0.12677,0.099806,0.088354,0.075163,0.065187,0.060734,0.054492,0.10583,0.096137,0.087377,0.070553,0.066779,0.05303,0.051461,0.046514},
{0.51898,0.30919,0.30743,0.37623,0.22803,0.16877,0.13106,0.10674,0.29964,0.18873,0.15645,0.12789,0.10094,0.084052,0.074287,0.066171,0.15578,0.10879,0.091882,0.082305,0.067463,0.059411,0.053564,0.049001,0.099644,0.074432,0.06545,0.062654,0.051906,0.046003,0.041748,0.040779},
{0.25821,0.56409,0.35276,0.16523,0.16082,0.10632,0.085421,0.071432,0.49089,0.21488,0.12183,0.11029,0.092428,0.068242,0.05922,0.05196,0.1575,0.10256,0.077304,0.070625,0.057332,0.046711,0.041811,0.039025,0.095162,0.072913,0.056334,0.049809,0.042945,0.035818,0.032868,0.03161},
{0.73635,0.18863,0.15655,0.14627,0.13746,0.11533,0.10257,0.097225,0.25026,0.14105,0.14476,0.13754,0.10944,0.097364,0.086668,0.080417,0.17961,0.13993,0.12196,0.10776,0.088477,0.079224,0.070185,0.067645,0.14109,0.10824,0.092556,0.091451,0.072109,0.063429,0.05721,0.057636},
{0.8817,0.12985,0.061408,0.048299,0.034633,0.027039,0.022644,0.023923,0.35535,0.064491,0.038226,0.035992,0.022971,0.019495,0.016637,0.021941,0.19413,0.039372,0.025921,0.031343,0.016862,0.014752,0.013162,0.021784,0.12922,0.028373,0.019525,0.030194,0.013523,0.012094,0.010824,0.022182},
{0.53497,0.40317,0.23453,0.10058,0.053619,0.062903,0.05335,0.035974,0.46604,0.28617,0.12346,0.050582,0.063305,0.058572,0.038065,0.02982,0.28839,0.14551,0.051116,0.051823,0.055074,0.038545,0.02647,0.028898,0.14963,0.067347,0.036987,0.041256,0.034438,0.024007,0.02129,0.024687},
{0.22867,0.74303,0.25226,0.16498,0.14493,0.10557,0.094585,0.079779,0.24607,0.21809,0.1388,0.11749,0.092943,0.076161,0.066078,0.05771,0.14878,0.12221,0.089528,0.077574,0.06407,0.054558,0.04842,0.0453,0.094001,0.089686,0.065117,0.058162,0.048129,0.042119,0.038376,0.03666},
{0.72051,0.12973,0.37135,0.19679,0.12458,0.16889,0.084947,0.09126,0.15481,0.25768,0.12797,0.12332,0.1284,0.071261,0.083108,0.068981,0.10024,0.083564,0.086539,0.07053,0.056675,0.056557,0.046847,0.0469,0.11424,0.064841,0.070443,0.053587,0.047215,0.045839,0.038915,0.037502},
{0.9392,0.078171,0.19095,0.048202,0.074043,0.031791,0.039207,0.022873,0.15995,0.039208,0.029562,0.023191,0.017372,0.015488,0.012574,0.012119,0.15614,0.027328,0.04344,0.018951,0.020599,0.013521,0.013394,0.011211,0.075078,0.017644,0.018716,0.013306,0.010404,0.0090035,0.007878,0.008323},
{0.56089,0.40097,0.18777,0.13035,0.13746,0.12487,0.098658,0.08318,0.42384,0.20093,0.11703,0.12718,0.11897,0.092942,0.074187,0.072288,0.18517,0.11031,0.10877,0.10715,0.088082,0.068079,0.062176,0.065027,0.10878,0.094008,0.089529,0.079283,0.062186,0.054169,0.053294,0.054392},
{0.84795,0.14184,0.096662,0.073729,0.055762,0.045341,0.037317,0.036731,0.37925,0.087602,0.060141,0.055378,0.039306,0.032908,0.028096,0.033058,0.20217,0.056455,0.042445,0.046627,0.029544,0.025575,0.022771,0.031293,0.13237,0.041254,0.032563,0.042688,0.023685,0.020936,0.019108,0.030787},
{0.64673,0.17609,0.23653,0.16705,0.12242,0.097918,0.080654,0.067954,0.46912,0.16308,0.13649,0.10413,0.082735,0.06786,0.058816,0.052356,0.23968,0.1096,0.090034,0.075388,0.061775,0.052727,0.046331,0.043637,0.15019,0.078476,0.065997,0.059087,0.048493,0.041925,0.037447,0.037622},
{0.92039,0.12377,0.23845,0.087696,0.086038,0.056547,0.04667,0.038572,0.11134,0.078043,0.057979,0.042621,0.032411,0.027808,0.02311,0.021193,0.12798,0.04928,0.054188,0.033241,0.026928,0.023283,0.018743,0.018144,0.061511,0.033831,0.032297,0.02237,0.019043,0.015679,0.014462,0.013706},
{0.50487,0.57998,0.20822,0.1502,0.12216,0.10276,0.084676,0.07312,0.20942,0.31192,0.14495,0.1094,0.087512,0.073376,0.063723,0.055834,0.1499,0.15679,0.094837,0.078285,0.063803,0.054476,0.048103,0.044865,0.10271,0.10553,0.0681,0.05928,0.048529,0.043179,0.03828,0.037309},
{0.91539,0.20588,0.11963,0.089739,0.040023,0.046392,0.025509,0.027686,0.2243,0.056617,0.048955,0.028861,0.02554,0.018169,0.017444,0.014322,0.1363,0.044563,0.026435,0.024188,0.015121,0.014365,0.011723,0.011634,0.096657,0.024139,0.025082,0.016744,0.013582,0.011041,0.0097059,0.010623},
{0.75019,0.20913,0.16257,0.10211,0.076884,0.061667,0.05001,0.042865,0.44882,0.13849,0.089446,0.067723,0.054602,0.044833,0.037701,0.034803,0.23293,0.083756,0.059743,0.050623,0.040331,0.034282,0.029477,0.030249,0.14114,0.056255,0.043665,0.040981,0.031632,0.027047,0.024183,0.027535},
{0.65718,0.43474,0.18009,0.059539,0.080378,0.064277,0.035993,0.034994,0.45353,0.20709,0.054428,0.07214,0.064872,0.034386,0.031746,0.033068,0.20097,0.069146,0.046304,0.047364,0.030302,0.024232,0.025405,0.021883,0.095428,0.044,0.027894,0.025271,0.018905,0.017234,0.015365,0.015895},
{0.60776,0.66894,0.14484,0.13189,0.084748,0.063344,0.053711,0.042785,0.17715,0.18607,0.089883,0.068153,0.049688,0.039892,0.034271,0.030163,0.095833,0.11026,0.050176,0.043996,0.032761,0.02762,0.023903,0.023469,0.069297,0.078083,0.035473,0.035552,0.02431,0.021862,0.018551,0.019949},
{0.77674,0.3699,0.10438,0.06421,0.061666,0.03691,0.029274,0.028645,0.39592,0.13669,0.050731,0.053797,0.032673,0.025231,0.024504,0.019646,0.17736,0.059656,0.033843,0.028085,0.019838,0.017657,0.014472,0.015583,0.11151,0.047839,0.021379,0.020801,0.015022,0.01242,0.010918,0.013447},
{0.45357,0.68227,0.32458,0.21064,0.14938,0.096111,0.086494,0.060265,0.202,0.12279,0.10668,0.081097,0.067023,0.052749,0.04781,0.039497,0.11071,0.093993,0.068529,0.055839,0.045024,0.038592,0.032566,0.031744,0.069855,0.06602,0.049331,0.042485,0.035201,0.029471,0.02643,0.025719},
{0.84885,0.34068,0.060235,0.11245,0.057325,0.038481,0.040783,0.024003,0.30977,0.056186,0.071563,0.041493,0.028441,0.029228,0.018312,0.020076,0.1163,0.041345,0.023556,0.020513,0.016212,0.013119,0.011987,0.012317,0.1076,0.040484,0.02044,0.023777,0.014225,0.012619,0.011231,0.011188},
{0.52435,0.60311,0.18956,0.12349,0.11367,0.078336,0.061952,0.053775,0.39494,0.20287,0.093761,0.088811,0.067618,0.050369,0.04469,0.039659,0.14573,0.091393,0.062983,0.054041,0.04189,0.035613,0.031548,0.029185,0.088124,0.068596,0.043045,0.037572,0.030809,0.026411,0.023598,0.023796},
{0.90038,0.13369,0.14684,0.099555,0.074478,0.060875,0.048401,0.043207,0.23396,0.1029,0.079107,0.060203,0.046876,0.039701,0.034354,0.031423,0.13537,0.062499,0.051355,0.042905,0.033739,0.029416,0.025785,0.025415,0.092787,0.043723,0.038356,0.034052,0.026606,0.023278,0.021073,0.022144},
{0.55881,0.4784,0.41601,0.19801,0.19829,0.10893,0.11315,0.079421,0.21361,0.1438,0.12383,0.10463,0.078367,0.071894,0.05627,0.054097,0.12757,0.085242,0.083285,0.064828,0.054959,0.048995,0.041882,0.038691,0.075321,0.074038,0.057158,0.054952,0.043163,0.037492,0.034751,0.032207},
{0.72906,0.23787,0.43532,0.13848,0.15911,0.10107,0.075077,0.067686,0.23974,0.1627,0.098999,0.091379,0.067044,0.050631,0.046331,0.037715,0.10982,0.072768,0.067022,0.04903,0.046819,0.035702,0.034043,0.029956,0.070942,0.05369,0.049785,0.037869,0.033688,0.027706,0.025695,0.023997},
{0.77058,0.19824,0.1947,0.26279,0.13785,0.10767,0.10939,0.074561,0.16167,0.20195,0.18769,0.10156,0.088295,0.092604,0.061609,0.054352,0.14183,0.11199,0.074849,0.073031,0.065638,0.050624,0.044751,0.043751,0.081394,0.065663,0.055312,0.049312,0.041231,0.038605,0.033897,0.033125},
{0.85746,0.12438,0.27674,0.1825,0.082532,0.0978,0.063898,0.051051,0.11531,0.18673,0.12946,0.066035,0.070017,0.051633,0.03914,0.037188,0.082869,0.069693,0.058936,0.048302,0.040353,0.036551,0.030757,0.028377,0.071675,0.04972,0.047877,0.034958,0.034079,0.029037,0.024247,0.02481},
{0.89038,0.29775,0.13292,0.12336,0.063148,0.059914,0.041496,0.037578,0.16289,0.084071,0.068156,0.043667,0.037772,0.028889,0.026438,0.02218,0.11199,0.063042,0.037167,0.034122,0.025018,0.020999,0.018386,0.01717,0.080083,0.036749,0.033369,0.024843,0.019596,0.017272,0.014604,0.014923},
{0.7461,0.46783,0.096311,0.12776,0.102,0.055911,0.054363,0.044724,0.33974,0.10104,0.093932,0.079171,0.043524,0.044264,0.036969,0.029641,0.10032,0.056689,0.046518,0.036106,0.028959,0.025908,0.022035,0.021808,0.086975,0.056578,0.031188,0.03062,0.024764,0.020133,0.018041,0.018094},
{0.8513,0.25386,0.31326,0.10014,0.11253,0.066874,0.05965,0.047309,0.11682,0.10888,0.080651,0.060599,0.047051,0.039501,0.032869,0.02963,0.11475,0.071524,0.065161,0.041593,0.037979,0.029415,0.026312,0.024492,0.062692,0.044923,0.040461,0.030882,0.026172,0.022126,0.019723,0.019305},
{0.56944,0.42464,0.35098,0.19199,0.12863,0.092868,0.072895,0.062641,0.39283,0.20997,0.12946,0.088464,0.069827,0.056793,0.049818,0.043706,0.15531,0.091085,0.068555,0.055628,0.045963,0.039135,0.035192,0.032161,0.088752,0.061295,0.049762,0.041567,0.034515,0.030188,0.027099,0.026117},
{0.83254,0.14159,0.14974,0.12225,0.10356,0.083673,0.072046,0.066606,0.28873,0.12633,0.10633,0.091897,0.073682,0.062689,0.055737,0.052906,0.16623,0.092503,0.0784,0.070693,0.055811,0.048947,0.044247,0.04556,0.11357,0.067557,0.059003,0.05917,0.044287,0.039852,0.036548,0.039416},
{0.80421,0.46235,0.17123,0.10595,0.074577,0.061976,0.046119,0.038515,0.16013,0.11509,0.075211,0.051001,0.042292,0.033387,0.028446,0.025187,0.0996,0.084406,0.045662,0.036129,0.028246,0.023789,0.020697,0.019396,0.074371,0.054062,0.034632,0.028107,0.021924,0.018897,0.016382,0.016787},
{0.72709,0.35033,0.11383,0.16575,0.15241,0.094239,0.079806,0.07817,0.35487,0.096935,0.14538,0.13252,0.083646,0.067894,0.070271,0.060147,0.10291,0.10129,0.097908,0.069572,0.054777,0.055931,0.049929,0.04198,0.086551,0.068128,0.055135,0.051406,0.04414,0.039908,0.034651,0.034428},
{0.69142,0.51225,0.32801,0.15064,0.11216,0.076796,0.064783,0.052705,0.16408,0.13282,0.091237,0.068176,0.056723,0.044706,0.03775,0.033986,0.094001,0.083491,0.059185,0.049486,0.03774,0.03205,0.028022,0.025819,0.065747,0.056095,0.043304,0.034288,0.028091,0.023954,0.022026,0.021099},
{0.71794,0.3499,0.14147,0.12215,0.090999,0.079632,0.065367,0.05664,0.31928,0.23309,0.11334,0.087426,0.068965,0.058352,0.049636,0.045918,0.20906,0.12933,0.078254,0.066567,0.052764,0.044297,0.039917,0.039276,0.13349,0.086479,0.057992,0.053195,0.041654,0.036147,0.032063,0.034172},
{0.84009,0.2888,0.12043,0.12094,0.079848,0.064532,0.053549,0.045513,0.29784,0.10439,0.088091,0.064458,0.050565,0.042525,0.036156,0.033888,0.13158,0.068413,0.051398,0.04406,0.034414,0.029964,0.026317,0.026295,0.10151,0.048742,0.03883,0.036691,0.027163,0.024016,0.021651,0.023146},
{0.65275,0.53014,0.15338,0.22103,0.12954,0.1147,0.088908,0.078048,0.19691,0.13904,0.12591,0.098918,0.078653,0.068542,0.058329,0.052191,0.13199,0.10774,0.077457,0.07224,0.056192,0.050674,0.043147,0.04242,0.093212,0.074011,0.05706,0.054127,0.042998,0.039015,0.035404,0.034559},
{0.72605,0.35991,0.28559,0.14748,0.13852,0.10893,0.094646,0.080929,0.15646,0.15716,0.13508,0.11002,0.088392,0.077763,0.067145,0.060725,0.14018,0.1079,0.092424,0.078123,0.065708,0.057351,0.051461,0.048453,0.095887,0.083264,0.070313,0.062826,0.052255,0.046013,0.042372,0.040613},
{0.82967,0.32312,0.15232,0.16472,0.1092,0.089375,0.073159,0.061526,0.1525,0.1313,0.11194,0.084245,0.066784,0.058333,0.049024,0.043616,0.11584,0.088687,0.067926,0.058125,0.047147,0.041671,0.036901,0.034955,0.083769,0.061128,0.051286,0.045544,0.037107,0.033571,0.030408,0.029143},
{0.74324,0.39193,0.2583,0.13105,0.09253,0.075423,0.059766,0.050053,0.31419,0.1455,0.091882,0.06782,0.054785,0.045068,0.038508,0.033971,0.13327,0.074536,0.056991,0.045153,0.036734,0.031399,0.027384,0.025971,0.091516,0.055398,0.04292,0.034984,0.027859,0.024195,0.021719,0.021604},
{0.24037,0.21526,0.19219,0.19391,0.16987,0.16158,0.1437,0.1414,0.22107,0.21387,0.19834,0.20737,0.17614,0.15653,0.14178,0.13465,0.21509,0.21878,0.20196,0.19621,0.16055,0.14352,0.12439,0.11646,0.21496,0.20076,0.18099,0.18083,0.13888,0.11886,0.10262,0.1014},
{0.53721,0.20524,0.19267,0.17741,0.16345,0.14542,0.12922,0.12119,0.28799,0.17699,0.17577,0.17494,0.1495,0.12534,0.11378,0.10727,0.20575,0.17396,0.16534,0.14694,0.12009,0.10715,0.096134,0.092562,0.18262,0.14669,0.12963,0.12518,0.10045,0.088803,0.07887,0.079904},
{0.14834,0.64135,0.071077,0.43838,0.053974,0.28597,0.042894,0.22971,0.078573,0.19772,0.046372,0.13486,0.03717,0.11039,0.031485,0.076494,0.054636,0.22433,0.037947,0.18724,0.030632,0.12291,0.026536,0.11369,0.042056,0.10315,0.028994,0.064518,0.024318,0.065396,0.021085,0.043672},
{0.31463,0.22471,0.19969,0.21534,0.29102,0.16795,0.21825,0.125,0.22562,0.22331,0.35877,0.16371,0.14216,0.11481,0.10574,0.086451,0.37272,0.16405,0.12555,0.10113,0.096408,0.08352,0.076818,0.069977,0.13381,0.099647,0.091418,0.086477,0.073829,0.067116,0.063125,0.057149},
{0.26186,0.2604,0.2093,0.43438,0.17312,0.25509,0.12538,0.13484,0.22755,0.46555,0.15899,0.15537,0.10973,0.11383,0.086942,0.08551,0.15894,0.14633,0.10405,0.11201,0.084254,0.085918,0.067708,0.069033,0.10602,0.11088,0.082449,0.079223,0.065037,0.063243,0.052788,0.053582},
{0.29052,0.26911,0.24328,0.19161,0.13084,0.085028,0.059678,0.051694,0.31307,0.29215,0.24293,0.17853,0.11018,0.069209,0.052199,0.049081,0.31299,0.26833,0.20675,0.1453,0.083928,0.056434,0.048313,0.048017,0.2726,0.22067,0.15941,0.1114,0.063999,0.049433,0.046542,0.046469},
{0.08115,0.78225,0.052029,0.43418,0.035392,0.25421,0.02779,0.16092,0.046456,0.1316,0.029593,0.077207,0.021243,0.056643,0.018078,0.046577,0.029334,0.18971,0.020579,0.11149,0.016604,0.064632,0.014319,0.048651,0.021514,0.051016,0.01627,0.047082,0.014169,0.040287,0.012839,0.031723},
{0.28147,0.2721,0.39534,0.27796,0.23245,0.19237,0.1644,0.1401,0.26789,0.24,0.21718,0.18281,0.15329,0.12589,0.11049,0.096698,0.19219,0.16917,0.14927,0.12244,0.10315,0.08867,0.079459,0.074188,0.13964,0.11717,0.10418,0.092491,0.07788,0.06977,0.062771,0.063054},
{0.25631,0.53527,0.21591,0.19467,0.17872,0.15954,0.13516,0.12393,0.23124,0.25866,0.18795,0.17507,0.1442,0.1231,0.10723,0.095968,0.1905,0.1819,0.14814,0.13088,0.10942,0.094319,0.086307,0.079796,0.15077,0.13508,0.11235,0.10045,0.085896,0.075003,0.070379,0.06684},
};

	}
	}
}

#endif // _BFM_CENTRES_H_
