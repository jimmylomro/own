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
			{0.135106,0.0488409,0.0525984,0.0549019,0.0330596,0.0313708,0.0265155,0.020074,0.0459167,0.0767881,0.0464808,0.0285695,0.0253797,0.023202,0.0174783,0.01633,0.0441077,0.0349851,0.0246299,0.0221249,0.0184325,0.0154865,0.0140435,0.0129673,0.0265149,0.0229582,0.0181322,0.0158215,0.0137354,0.0121624,0.0106184,0.0106671},
			{0.172593,0.0442126,0.0366946,0.0342836,0.0322181,0.0270312,0.0240406,0.0227886,0.0586575,0.03306,0.0339314,0.0322391,0.0256514,0.0228212,0.0203141,0.018849,0.042099,0.032799,0.0285862,0.0252571,0.0207382,0.0185693,0.0164507,0.0158554,0.0330712,0.0253712,0.0216942,0.0214352,0.0169017,0.0148671,0.0134095,0.0135092},
			{0.123862,0.142288,0.0510842,0.0368494,0.0299691,0.0252098,0.0207737,0.0179388,0.0513777,0.0765241,0.0355621,0.0268385,0.0214695,0.0180016,0.0156332,0.0136978,0.0367764,0.0384646,0.0232665,0.0192057,0.0156528,0.0133647,0.0118012,0.0110069,0.0251971,0.0258906,0.0167072,0.0145434,0.0119056,0.0105932,0.00939139,0.00915305},
			{0.207902,0.0678316,0.124138,0.0394911,0.0453716,0.0288217,0.0214095,0.0193018,0.0683658,0.046397,0.028231,0.0260582,0.0191187,0.0144382,0.013212,0.0107549,0.0313155,0.0207509,0.0191125,0.0139817,0.0133511,0.0101809,0.00970778,0.0085423,0.0202301,0.0153105,0.0141971,0.0107988,0.00960675,0.00790074,0.00732742,0.006843},
			{0.0387359,0.354641,0.0237191,0.0945783,0.0157551,0.03177,0.0108698,0.0197755,0.019927,0.0689473,0.013207,0.0440929,0.0099741,0.0241327,0.00765005,0.015397,0.0119772,0.0379659,0.00848389,0.0177724,0.00671803,0.0104681,0.00551388,0.00946945,0.0088341,0.0382152,0.00685777,0.0185193,0.00503166,0.00899504,0.00443383,0.00757118},
			{0.0870409,0.249722,0.0456984,0.0342385,0.0263388,0.0239168,0.016549,0.0149282,0.0495022,0.098735,0.0270855,0.0289404,0.0175041,0.0153608,0.0124285,0.0113838,0.0283748,0.0411862,0.0166639,0.0181442,0.0115025,0.00996627,0.00862377,0.00879146,0.0178137,0.0282443,0.0112396,0.0114576,0.00834191,0.00711103,0.00637869,0.00678731},
			{0.0534819,0.148035,0.0406672,0.0462684,0.0252878,0.022588,0.0175688,0.0160723,0.043991,0.113691,0.0320844,0.0296572,0.0195013,0.0187208,0.0141025,0.0129234,0.0345058,0.0685495,0.0233947,0.0214866,0.0154595,0.0151533,0.0117156,0.0110124,0.0253536,0.0419396,0.0172605,0.0165623,0.0121176,0.0117403,0.00957335,0.0095347},
			{0.24161,0.151497,0.0311886,0.0413738,0.0330318,0.0181057,0.0176045,0.014483,0.11002,0.032721,0.030418,0.0256379,0.0140944,0.0143342,0.0119718,0.00959854,0.0324872,0.0183578,0.0150639,0.0116923,0.00937793,0.00838996,0.00713577,0.00706202,0.0281654,0.0183216,0.0100996,0.00991565,0.00801945,0.00651962,0.00584223,0.00585953},
			{0.0578924,0.188116,0.0638657,0.0417693,0.0366918,0.0267289,0.0239467,0.020198,0.062299,0.0552144,0.035142,0.029746,0.0235308,0.019282,0.0167294,0.0146108,0.0376679,0.0309397,0.0226662,0.0196399,0.016221,0.0138128,0.0122588,0.0114689,0.0237988,0.0227062,0.016486,0.0147251,0.0121851,0.0106635,0.00971585,0.00928154},
			{0.0648066,0.141576,0.0885356,0.0414703,0.0403628,0.0266835,0.021439,0.017928,0.123205,0.0539298,0.0305771,0.027682,0.0231978,0.0171275,0.0148632,0.0130411,0.0395308,0.0257414,0.0194018,0.0177255,0.0143894,0.0117237,0.0104937,0.00979461,0.0238839,0.0182999,0.0141387,0.0125011,0.0107783,0.00898955,0.00824918,0.00793355},
			{0.222151,0.117146,0.0772033,0.0391716,0.0276568,0.0225437,0.0178637,0.0149607,0.0939086,0.0434904,0.0274631,0.0202712,0.0163749,0.0134705,0.0115099,0.0101539,0.0398342,0.0222785,0.0170344,0.013496,0.0109797,0.00938516,0.00818482,0.00776256,0.0273538,0.0165581,0.0128286,0.0104567,0.00832705,0.00723189,0.00649184,0.0064572},
			{0.190857,0.210068,0.0454846,0.0414173,0.0266135,0.019892,0.016867,0.0134357,0.0556321,0.0584315,0.0282262,0.0214023,0.0156035,0.0125273,0.0107623,0.00947228,0.0300945,0.0346252,0.0157567,0.013816,0.0102881,0.0086734,0.00750634,0.00737002,0.0217615,0.0245206,0.0111397,0.0111646,0.00763421,0.00686547,0.00582567,0.00626464},
			{0.363158,0.053483,0.0252929,0.0198938,0.0142649,0.0111368,0.00932664,0.00985346,0.146363,0.026563,0.0157448,0.0148245,0.00946156,0.00802975,0.00685243,0.00903729,0.0799598,0.0162166,0.0106766,0.0129096,0.00694538,0.00607629,0.00542129,0.00897237,0.0532252,0.0116866,0.00804214,0.0124365,0.00556985,0.0049815,0.00445808,0.00913638},
			{0.238407,0.0405447,0.0428782,0.0350082,0.0296556,0.0239607,0.020631,0.0190734,0.0826814,0.0361753,0.0304499,0.0263157,0.0210996,0.0179516,0.0159609,0.0151502,0.0476019,0.0264892,0.0224505,0.0202437,0.0159821,0.0140164,0.0126705,0.0130466,0.0325232,0.0193457,0.0168962,0.0169438,0.0126821,0.0114121,0.0104658,0.0112871},
			{0.0683545,0.109038,0.118632,0.0569349,0.0421801,0.0328831,0.025825,0.0212696,0.0553224,0.0549115,0.0405335,0.0281063,0.0237319,0.0193992,0.0165074,0.0145772,0.032863,0.0285637,0.0239251,0.0187911,0.0159664,0.0137398,0.0121922,0.0110902,0.021901,0.0196538,0.0173824,0.0140733,0.01224,0.0106173,0.00964384,0.00915051},
			{0.0557238,0.0554116,0.044539,0.0924354,0.0368393,0.0542828,0.0266807,0.0286939,0.0484227,0.0990679,0.033833,0.0330625,0.0233495,0.0242229,0.0185009,0.0181962,0.0338213,0.0311388,0.0221415,0.0238344,0.0179289,0.018283,0.014408,0.0146899,0.0225617,0.0235945,0.0175449,0.0168585,0.0138397,0.013458,0.0112331,0.0114021},
			{0.184748,0.0915824,0.0726701,0.0375277,0.0352483,0.0277178,0.0240832,0.0205928,0.0398114,0.0399913,0.0343719,0.0279948,0.022492,0.0197873,0.0170855,0.0154518,0.03567,0.0274555,0.0235178,0.0198789,0.0167197,0.0145932,0.0130946,0.0123291,0.024399,0.0211871,0.0178916,0.0159864,0.0132967,0.0117082,0.0107819,0.0103343},
			{0.208093,0.154169,0.0987191,0.0453361,0.0337565,0.0231129,0.0194975,0.0158625,0.0493833,0.0399752,0.0274593,0.0205187,0.0170718,0.0134551,0.0113615,0.0102285,0.0282909,0.025128,0.0178127,0.0148937,0.0113583,0.00964593,0.00843356,0.00777061,0.0197875,0.0168827,0.0130329,0.0103195,0.00845453,0.00720933,0.00662917,0.00634999},
			{0.192233,0.0936878,0.037878,0.032706,0.0243655,0.0213218,0.0175022,0.0151656,0.0854875,0.0624115,0.0303464,0.0234086,0.0184657,0.015624,0.0132903,0.0122947,0.0559765,0.0346295,0.0209528,0.0178237,0.0141277,0.0118606,0.010688,0.0105163,0.0357433,0.0231552,0.0155277,0.0142432,0.0111531,0.0096786,0.00858508,0.00914984},
			{0.202224,0.0520234,0.0510967,0.0689641,0.0361765,0.0282561,0.0287065,0.0195672,0.0424269,0.0529984,0.0492549,0.0266525,0.0231714,0.0243023,0.0161682,0.0142637,0.0372206,0.0293911,0.0196427,0.0191657,0.0172256,0.0132855,0.011744,0.0114817,0.0213603,0.0172322,0.0145157,0.0129411,0.0108204,0.0101311,0.00889577,0.00869307},
			{0.10694,0.0408553,0.0383546,0.0353165,0.0325367,0.0289472,0.0257231,0.0241257,0.0573287,0.0352317,0.0349906,0.0348251,0.0297598,0.0249502,0.0226503,0.0213537,0.0409584,0.0346301,0.0329144,0.0292514,0.0239067,0.0213295,0.019137,0.018426,0.0363529,0.0292016,0.0258055,0.0249182,0.0199952,0.0176776,0.0157002,0.0159062},
			{0.0684062,0.0534086,0.11978,0.0432598,0.0542204,0.0286563,0.0254546,0.0202516,0.132929,0.0425465,0.0330318,0.0275505,0.0241664,0.0196585,0.0166188,0.0143115,0.0375089,0.0266394,0.0218749,0.0181188,0.0169349,0.0133834,0.0123194,0.0111196,0.0258651,0.0184192,0.0162502,0.0139823,0.013146,0.0107389,0.0100893,0.00935912},
			{0.381994,0.0859132,0.0499203,0.037448,0.0167018,0.0193593,0.010645,0.0115536,0.0935986,0.0236264,0.020429,0.0120438,0.0106579,0.00758176,0.00727929,0.00597666,0.0568767,0.018596,0.0110312,0.0100938,0.00630995,0.00599446,0.00489195,0.00485475,0.0403351,0.0100733,0.0104666,0.00698743,0.00566783,0.00460745,0.00405027,0.00443288},
			{0.0556706,0.0538189,0.0781931,0.0549764,0.0459763,0.0380487,0.0325169,0.0277098,0.0529852,0.0474693,0.0429549,0.0361585,0.0303184,0.0249,0.0218534,0.0191257,0.0380119,0.0334595,0.0295239,0.0242172,0.0204027,0.0175379,0.015716,0.0146734,0.0276187,0.0231743,0.0206048,0.0182935,0.0154037,0.0137997,0.0124154,0.0124713},
			{0.203837,0.134843,0.0558596,0.018467,0.0249308,0.0199368,0.0111639,0.0108541,0.14067,0.0642326,0.0168819,0.0223756,0.0201214,0.0106655,0.00984657,0.0102568,0.0623346,0.0214468,0.014362,0.0146907,0.00939886,0.00751591,0.00787998,0.00678746,0.0295987,0.0136475,0.00865188,0.00783824,0.00586361,0.00534536,0.00476581,0.00493011},
			{0.125836,0.189284,0.0900503,0.0584391,0.0414433,0.0266645,0.0239964,0.0167195,0.0560428,0.0340648,0.0295954,0.0224991,0.0185944,0.0146344,0.0132642,0.0109579,0.0307142,0.0260767,0.0190123,0.0154915,0.0124913,0.0107068,0.00903496,0.00880683,0.01938,0.0183161,0.0136862,0.0117867,0.00976604,0.00817617,0.0073325,0.00713544},
			{0.0872051,0.120507,0.067154,0.0484535,0.0536257,0.0266216,0.0365885,0.0199247,0.0631918,0.0249039,0.040448,0.0254518,0.0254392,0.0202979,0.0175001,0.0180355,0.0431,0.0231517,0.027602,0.0174758,0.0189993,0.0148351,0.0139672,0.0121292,0.0265137,0.0217439,0.0166917,0.0190484,0.0129409,0.0136675,0.0107559,0.0120295},
			{0.359905,0.0483979,0.0932408,0.0342921,0.033644,0.0221119,0.0182497,0.0150831,0.0435363,0.0305173,0.0226717,0.0166661,0.0126739,0.0108738,0.00903684,0.00828706,0.0500452,0.0192702,0.0211895,0.0129982,0.0105298,0.00910443,0.00732926,0.00709511,0.0240529,0.0132291,0.0126294,0.00874737,0.00744666,0.00613105,0.00565516,0.00535942},
			{0.171811,0.139538,0.0403716,0.058178,0.0340973,0.0301901,0.0234016,0.0205431,0.0518288,0.0365966,0.0331411,0.0260362,0.0207023,0.018041,0.0153528,0.0137372,0.0347404,0.0283583,0.0203875,0.0190144,0.0147903,0.013338,0.0113567,0.0111655,0.0245343,0.0194806,0.0150188,0.0142467,0.0113175,0.0102692,0.00931876,0.00909636},
			{0.147071,0.16916,0.0531693,0.0346378,0.0318831,0.0219719,0.0173763,0.015083,0.110773,0.0569011,0.0262982,0.0249097,0.0189655,0.0141275,0.0125347,0.0111235,0.0408747,0.025634,0.0176655,0.0151575,0.0117493,0.00998873,0.00884853,0.00818581,0.024717,0.01924,0.0120732,0.0105381,0.0086414,0.00740788,0.00661886,0.00667444},
			{0.141396,0.12105,0.105265,0.050103,0.0501749,0.0275619,0.0286297,0.020096,0.0540512,0.0363865,0.0313335,0.026476,0.0198295,0.0181916,0.0142382,0.0136882,0.032279,0.0215689,0.0210738,0.0164037,0.0139063,0.0123973,0.0105975,0.00978997,0.0190586,0.0187339,0.0144629,0.0139047,0.0109216,0.00948681,0.0087931,0.00814943},
			{0.0271811,0.262013,0.0174269,0.145427,0.0118546,0.0851481,0.0093083,0.0538995,0.0155604,0.0440789,0.00991211,0.0258605,0.00711539,0.0189725,0.00605503,0.0156009,0.00982549,0.0635438,0.00689282,0.037343,0.00556137,0.0216484,0.00479627,0.0162955,0.00720623,0.0170877,0.00544955,0.0157699,0.00474584,0.013494,0.0043005,0.0106256},
			{0.272718,0.0395596,0.0880171,0.0580439,0.0262498,0.0311059,0.0203231,0.016237,0.036676,0.0593902,0.0411763,0.0210027,0.0222692,0.016422,0.0124487,0.0118278,0.026357,0.0221661,0.0187449,0.0153626,0.0128346,0.0116253,0.00978245,0.00902537,0.0227967,0.0158137,0.0152276,0.0111185,0.0108391,0.00923543,0.00771196,0.00789086},
			{0.225574,0.0628827,0.0488844,0.0307047,0.0231181,0.0185425,0.0150375,0.012889,0.134956,0.0416433,0.0268955,0.0203635,0.0164182,0.0134808,0.0113362,0.0104649,0.0700393,0.0251844,0.0179641,0.0152217,0.012127,0.0103081,0.00886349,0.00909563,0.0424398,0.0169152,0.0131296,0.0123226,0.00951147,0.00813275,0.00727168,0.00827961},
			{0.122661,0.0912603,0.0447186,0.0429815,0.0361738,0.0310411,0.0272329,0.024397,0.0374237,0.0370754,0.0397627,0.0338018,0.0270329,0.023626,0.0214372,0.0190822,0.0366007,0.0333712,0.0281219,0.0242961,0.0204286,0.0184435,0.0162582,0.0151688,0.0283812,0.0240269,0.0203697,0.019077,0.0159442,0.0142454,0.0129752,0.0125842},
			{0.0518753,0.108333,0.0436983,0.0394003,0.0361721,0.0322885,0.0273548,0.0250823,0.0467999,0.052351,0.0380398,0.0354317,0.0291853,0.0249148,0.0217033,0.0194231,0.0385545,0.0368151,0.0299815,0.0264883,0.022145,0.0190893,0.0174677,0.0161499,0.0305141,0.0273384,0.0227379,0.0203292,0.0173846,0.0151799,0.014244,0.0135277},
			{0.126891,0.0533536,0.103202,0.0348085,0.0449202,0.0328133,0.0265453,0.0259728,0.0647991,0.0423991,0.0339228,0.034489,0.0229717,0.0218978,0.0194786,0.0161734,0.0350323,0.0287417,0.0226291,0.0200327,0.0170418,0.01478,0.0137703,0.0123551,0.0239945,0.0217973,0.0198112,0.0159966,0.015141,0.0120236,0.0116679,0.0105463},
			{0.327137,0.131296,0.0232139,0.0433357,0.0220923,0.0148303,0.0157175,0.00925063,0.119381,0.0216536,0.0275794,0.0159911,0.0109609,0.0112642,0.00705722,0.00773708,0.0448201,0.0159339,0.0090781,0.00790547,0.00624782,0.005056,0.0046198,0.00474666,0.0414666,0.015602,0.00787741,0.00916336,0.00548231,0.00486308,0.00432833,0.00431189},
			{0.183615,0.0330606,0.094634,0.0501506,0.0317476,0.0430402,0.0216479,0.0232566,0.0394516,0.0656679,0.0326112,0.0314278,0.0327222,0.0181601,0.0211792,0.017579,0.0255459,0.0212955,0.0220535,0.017974,0.014443,0.0144131,0.0119386,0.0119521,0.0291121,0.016524,0.0179516,0.0136562,0.0120322,0.0116817,0.00991712,0.00955706},
			{0.0388753,0.168075,0.0186265,0.114883,0.0141445,0.0749428,0.011241,0.0601976,0.020591,0.0518159,0.0121524,0.0353428,0.00974095,0.02893,0.0082511,0.0200463,0.0143181,0.0587874,0.00994452,0.0490693,0.00802758,0.032209,0.00695403,0.0297931,0.0110212,0.0270324,0.00759834,0.0169078,0.00637286,0.0171379,0.00552551,0.0114447},
			{0.162973,0.0443728,0.0596044,0.0420958,0.0308491,0.0246748,0.0203243,0.0171241,0.118216,0.0410943,0.0343956,0.0262399,0.0208486,0.0171003,0.0148213,0.0131933,0.0603977,0.0276192,0.0226879,0.0189974,0.015567,0.0132869,0.0116751,0.0109963,0.0378465,0.0197755,0.0166308,0.0148895,0.0122199,0.0105649,0.00943629,0.0094805},
			{0.297385,0.0497463,0.0339004,0.0258577,0.0195562,0.0159016,0.0130876,0.012882,0.133008,0.0307231,0.0210922,0.0194217,0.0137851,0.0115413,0.00985355,0.0115939,0.0709035,0.0197994,0.0148859,0.0163527,0.0103615,0.00896956,0.00798618,0.0109748,0.0464232,0.0144684,0.0114201,0.0149712,0.00830646,0.0073426,0.00670151,0.0107973},
			{0.193276,0.0931254,0.0302585,0.0440591,0.0405142,0.0250508,0.0212141,0.0207792,0.0943319,0.0257673,0.0386443,0.0352264,0.0222349,0.0180478,0.0186795,0.0159883,0.0273566,0.0269243,0.026026,0.0184936,0.014561,0.0148677,0.0132723,0.0111592,0.023007,0.0181098,0.014656,0.0136648,0.0117333,0.0106084,0.00921101,0.00915178},
			{0.0785556,0.0983908,0.0731572,0.0406656,0.0235194,0.0204174,0.0180129,0.0146984,0.09197,0.0803977,0.0462383,0.0228834,0.0185805,0.0177111,0.0146436,0.0118286,0.0663505,0.0452121,0.0228038,0.016137,0.0162387,0.0143958,0.0114119,0.00945134,0.0353645,0.0235338,0.0141232,0.0133107,0.013002,0.0105239,0.0082448,0.00822572},
			{0.118074,0.0703431,0.0699439,0.0855968,0.0518785,0.0383968,0.0298173,0.0242836,0.0681724,0.0429382,0.0355935,0.0290958,0.022964,0.0191227,0.0169012,0.0150547,0.0354418,0.0247516,0.0209042,0.0187253,0.0153485,0.0135167,0.0121863,0.0111483,0.0226701,0.0169342,0.0148905,0.0142544,0.0118092,0.0104662,0.0094981,0.00927779},
			{0.24663,0.0960525,0.0452785,0.0489635,0.0324602,0.0265678,0.0217474,0.0182894,0.0453317,0.0390303,0.0332753,0.0250429,0.0198523,0.0173402,0.0145728,0.0129653,0.0344357,0.0263633,0.0201919,0.0172783,0.014015,0.0123873,0.0109692,0.0103908,0.0249012,0.018171,0.0152454,0.0135385,0.0110305,0.0099794,0.0090391,0.00866323},
			{0.283914,0.0846651,0.104474,0.0333986,0.0375288,0.0223027,0.0198936,0.0157779,0.0389607,0.0363127,0.0268976,0.0202101,0.0156917,0.0131738,0.010962,0.00988176,0.0382704,0.0238538,0.0217315,0.0138714,0.0126663,0.00980996,0.00877508,0.00816836,0.0209082,0.0149821,0.0134939,0.0102992,0.00872867,0.00737904,0.00657782,0.00643828},
			{0.427135,0.035551,0.0868419,0.0219217,0.0336736,0.0144583,0.0178307,0.0104022,0.0727418,0.0178314,0.0134443,0.010547,0.00790066,0.00704371,0.00571839,0.00551136,0.0710096,0.0124283,0.0197561,0.00861877,0.00936835,0.00614899,0.00609157,0.00509881,0.0341444,0.00802406,0.00851188,0.00605119,0.00473181,0.00409469,0.00358283,0.00378519},
			{0.0689087,0.0570112,0.0546548,0.0475971,0.0397299,0.0325178,0.0268582,0.0228762,0.106275,0.0490574,0.0377968,0.0315256,0.0268516,0.0227995,0.0198735,0.0175366,0.0455762,0.0317355,0.0263275,0.0224982,0.02004,0.0171734,0.0153701,0.0140313,0.0308929,0.022465,0.0197367,0.0180282,0.0157645,0.013963,0.0124985,0.0120301},
			{0.14535,0.109539,0.0637225,0.0273272,0.0145681,0.0170907,0.0144951,0.00977417,0.126621,0.0777509,0.0335426,0.0137429,0.0171999,0.015914,0.0103421,0.00810202,0.0783542,0.039534,0.0138882,0.0140803,0.0149634,0.0104725,0.0071917,0.00785149,0.0406538,0.018298,0.0100494,0.0112092,0.00935663,0.00652277,0.00578445,0.0067074},
			{0.0432922,0.0387699,0.0346137,0.0349246,0.0305937,0.0291009,0.0258815,0.0254658,0.0398158,0.0385178,0.0357219,0.0373478,0.0317238,0.0281921,0.0255355,0.0242503,0.0387379,0.0394037,0.0363741,0.0353382,0.0289155,0.0258485,0.0224026,0.0209743,0.0387148,0.0361583,0.0325965,0.0325689,0.0250126,0.0214062,0.0184825,0.0182623},
			{0.0609086,0.0564211,0.0510048,0.0401731,0.0274313,0.0178268,0.0125119,0.010838,0.0656363,0.0612512,0.050933,0.0374305,0.0231002,0.0145101,0.0109439,0.0102902,0.0656204,0.0562582,0.0433457,0.030463,0.0175962,0.0118318,0.0101292,0.0100671,0.057153,0.0462645,0.0334206,0.0233568,0.0134178,0.010364,0.00975779,0.00974258},
			{0.10848,0.0716714,0.0545833,0.0361816,0.0201748,0.012623,0.0120757,0.0114834,0.0969888,0.0682997,0.0462258,0.0261666,0.0130741,0.0118369,0.0122656,0.0106615,0.0802023,0.0526308,0.0312348,0.0162838,0.0114013,0.0123447,0.0114733,0.00912481,0.0573863,0.0348922,0.0186731,0.0119488,0.0113167,0.0113269,0.00936291,0.00760586},
			{0.0636904,0.0454876,0.0404239,0.0435924,0.0589126,0.033999,0.0441809,0.0253038,0.0456725,0.0452062,0.0726276,0.0331401,0.0287769,0.0232405,0.0214059,0.0175005,0.0754514,0.0332096,0.0254164,0.0204713,0.0195162,0.0169071,0.0155505,0.0141657,0.0270883,0.0201718,0.0185059,0.0175058,0.0149454,0.0135864,0.0127785,0.0115688},
			{0.27553,0.158406,0.0586669,0.0362989,0.0255509,0.0212336,0.015801,0.0131956,0.0548638,0.0394299,0.025768,0.0174735,0.0144898,0.0114389,0.00974581,0.00862935,0.0341241,0.0289185,0.0156444,0.0123782,0.00967755,0.00815048,0.00709109,0.00664531,0.0254802,0.0185224,0.0118653,0.00962967,0.00751147,0.00647436,0.00561255,0.00575128},
			{0.274019,0.0942011,0.0392811,0.0394475,0.0260447,0.021049,0.0174666,0.0148455,0.0971474,0.0340511,0.0287335,0.0210247,0.0164931,0.0138709,0.0117934,0.0110536,0.0429191,0.022315,0.0167648,0.0143713,0.0112251,0.00977377,0.00858406,0.00857672,0.0331099,0.0158985,0.0126656,0.0119677,0.00886006,0.00783352,0.0070622,0.00754981},
			{0.0842038,0.239917,0.0479566,0.0932409,0.0328691,0.0375938,0.0219261,0.0210927,0.0466774,0.0289518,0.0251193,0.0249551,0.0171189,0.0167841,0.0133008,0.0123726,0.0263872,0.0333182,0.0161217,0.017901,0.0116896,0.0108392,0.00939895,0.00897454,0.0184818,0.0206581,0.0126097,0.0148903,0.00991641,0.00914449,0.00816331,0.0074254},
			{0.0636123,0.14879,0.046059,0.0917282,0.038241,0.0457163,0.0291896,0.0297342,0.0436743,0.0381706,0.0314209,0.026261,0.023049,0.0187672,0.0179313,0.0134599,0.0325152,0.0342359,0.0233804,0.0240783,0.0184206,0.0154904,0.0149373,0.0128992,0.0241007,0.0174587,0.0173812,0.0142539,0.0137907,0.0105756,0.0112585,0.00941835},
			{0.316539,0.0469998,0.0516241,0.0349999,0.0261836,0.0214013,0.017016,0.0151899,0.0822524,0.0361748,0.027811,0.0211652,0.0164798,0.0139574,0.0120777,0.0110472,0.0475901,0.0219722,0.0180546,0.0150837,0.0118613,0.0103416,0.00906492,0.00893497,0.0326204,0.0153712,0.0134846,0.0119713,0.0093536,0.00818366,0.00740839,0.00778488},
			{0.274394,0.130673,0.0368723,0.0226831,0.0217843,0.0130391,0.0103416,0.0101192,0.139864,0.0482888,0.0179215,0.0190045,0.0115422,0.00891334,0.0086564,0.00694021,0.0626542,0.0210744,0.0119556,0.00992134,0.00700792,0.00623744,0.00511256,0.00550476,0.0393911,0.0168998,0.00755261,0.00734839,0.00530673,0.00438747,0.0038571,0.00475049},
			{0.149175,0.111243,0.0919443,0.0502939,0.0336959,0.0243285,0.0190962,0.0164098,0.102908,0.0550063,0.0339139,0.0231747,0.0182925,0.0148778,0.0130506,0.0114496,0.0406849,0.0238614,0.0179593,0.0145728,0.0120409,0.0102522,0.00921912,0.0084251,0.0232502,0.0160573,0.013036,0.0108893,0.00904191,0.00790838,0.00709898,0.00684191},
			{0.436574,0.0388097,0.0233165,0.0145279,0.0097172,0.00759661,0.00633236,0.00669658,0.159004,0.0178317,0.0125981,0.00998759,0.00633685,0.00517562,0.00444523,0.00623885,0.0893232,0.0111187,0.00818032,0.00887508,0.00449264,0.00390876,0.00346193,0.00634794,0.0597969,0.00806838,0.00610008,0.00883963,0.00365652,0.00319348,0.00286659,0.0065807},
			{0.331003,0.110689,0.049415,0.0458601,0.0234758,0.0222734,0.0154264,0.0139699,0.0605536,0.0312539,0.0253375,0.0162334,0.0140419,0.0107398,0.00982855,0.00824548,0.0416319,0.0234363,0.0138169,0.012685,0.00930053,0.00780646,0.00683518,0.00638305,0.0297713,0.0136617,0.0124052,0.00923551,0.00728491,0.00642106,0.00542914,0.00554759},
			{0.129181,0.0923492,0.0432448,0.0300217,0.0316589,0.0287592,0.0227223,0.0191575,0.0976154,0.046277,0.0269539,0.0292918,0.0274,0.0214057,0.0170861,0.0166488,0.0426468,0.025405,0.0250514,0.024678,0.0202863,0.0156795,0.0143199,0.0149766,0.0250524,0.0216513,0.0206196,0.01826,0.0143223,0.0124758,0.0122742,0.0125272}
		};
	}
	}
}

#endif // _BFM_CENTRES_H_