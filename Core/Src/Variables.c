/*
 * VariablesInit.c
 *
 *  Created on: Jul 30, 2022
 *      Author: freeman
 */


#include "Variables.h"

int16_t dutyLED;
int16_t dutyCmd;
int16_t stepNumber;
int16_t roundNumber;

float CAL_stepSize = 5.12f;
int16_t dirCmd;	// 1:Clockwise 0:Counterclockwise
uint16_t testAngle;
int32_t filteredAngle;
int32_t fullAngle;
int32_t fullAngle_k_1;
int32_t fullAngle_k_2;
int32_t angularSpeed;
int32_t angularSpeed_k_1;
int32_t angularAcceleration;
int32_t commandAngle = 0;
int32_t commandAngle_k_1 = 0;
int32_t filteredAngle_k_1;
int32_t commandSpeed = 300;
int32_t targetSpeed = 0;
int32_t deltaSpeed = 0;
int32_t commandOut;
int32_t commandOut_k_1;
int32_t errorAngleOut;
int32_t errorSpeedOut;
uint16_t flashAngle;
uint16_t amplitude;
uint16_t counterTIM1_SLOW;
uint16_t counterTIM1_5k;
uint16_t counterTIM1_1k;
uint16_t counterTIM2_10k;
uint16_t counterTIM2_1k;
uint16_t counterTIM2_5k;
uint16_t counterTIM2_speed;
uint16_t testARR;
int32_t freeCounter;

// Motor Speed Control Vars
int16_t CAL_rotationSpeed;
int16_t rotationSpeed_k;
int16_t rotationSpeed_k_1;
int32_t rotationSpeedEncoder;
int32_t rotationSpeedRPM;
uint32_t speedCmd;

// PID parameters
int32_t controlEffort;


// REAL-TIME Parameters Tuning
uint8_t parameterTuningEnable = 0;
int32_t recKp;
int32_t recKi;
int32_t recKd;
int32_t recCommandAngle;
int32_t recCommandSpeed;
int32_t recFilterCoef;
uint8_t recFilterEnableFlag;
uint8_t recStepSizeIdx;
uint32_t recSpeedSetpoint;	// in RPM unit


// USART Communication
uint8_t inData[nBytes];
uint8_t idx = 0;


// For speed regulation
uint32_t setpointARR = 2000;
int32_t deltaARR = -1;
int32_t stepARR = 0;
uint32_t outputARR = 2000;
uint32_t readARR = 0;
int32_t deltaPositiveMax = 20;
int32_t deltaNegativeMax = -20;
uint16_t lowpassFilterCoef = 100;	// 0~1000
int32_t deltaARRmin = -100;
int32_t deltaARRmax = 100;
uint32_t ARRmin;
uint32_t ARRmax;
int32_t speedMax;
int32_t speedMin;
int32_t effort2freqCoef;

int32_t accelerationLimit;		// given in RPM increment

uint8_t CAL_closedLoop = 1;
uint8_t CAL_releaseMotor = 0;
uint8_t CAL_dirCmd = 1;
uint8_t CAL_calibrateEncoder = 0;


PID_TypeDef pidPosition, pidEffort, pidSpeed;


/****************  CONSTANTS  *******************/

const int16_t ppsConst;
const uint32_t frequencyMIN = 4000;	// unit: ARR value 4000 for 1kHz
const uint32_t frequencyMAX = 4000;
const uint32_t speedSetpoint = 3000;	// RPM
const uint32_t speedMAX = 5000;			// RPM, must larger than RPM corresponding to ARRmin, here for 0.98RPM

const int16_t sinLookupTable[] =
{
    0,      1,      3,      4,      6,      7,      9,     10,     12,     14,     15,     17,     18,     20,     21,     23,
   25,     26,     28,     29,     31,     32,     34,     36,     37,     39,     40,     42,     43,     45,     47,     48,
   50,     51,     53,     54,     56,     58,     59,     61,     62,     64,     65,     67,     69,     70,     72,     73,
   75,     76,     78,     80,     81,     83,     84,     86,     87,     89,     90,     92,     94,     95,     97,     98,
  100,    101,    103,    105,    106,    108,    109,    111,    112,    114,    115,    117,    119,    120,    122,    123,
  125,    126,    128,    130,    131,    133,    134,    136,    137,    139,    140,    142,    144,    145,    147,    148,
  150,    151,    153,    154,    156,    158,    159,    161,    162,    164,    165,    167,    168,    170,    171,    173,
  175,    176,    178,    179,    181,    182,    184,    185,    187,    188,    190,    192,    193,    195,    196,    198,
  199,    201,    202,    204,    205,    207,    209,    210,    212,    213,    215,    216,    218,    219,    221,    222,
  224,    225,    227,    228,    230,    232,    233,    235,    236,    238,    239,    241,    242,    244,    245,    247,
  248,    250,    251,    253,    254,    256,    257,    259,    260,    262,    264,    265,    267,    268,    270,    271,
  273,    274,    276,    277,    279,    280,    282,    283,    285,    286,    288,    289,    291,    292,    294,    295,
  297,    298,    300,    301,    303,    304,    306,    307,    309,    310,    312,    313,    315,    316,    318,    319,
  321,    322,    324,    325,    327,    328,    330,    331,    333,    334,    336,    337,    339,    340,    342,    343,
  344,    346,    347,    349,    350,    352,    353,    355,    356,    358,    359,    361,    362,    364,    365,    367,
  368,    369,    371,    372,    374,    375,    377,    378,    380,    381,    383,    384,    386,    387,    388,    390,
  391,    393,    394,    396,    397,    399,    400,    402,    403,    404,    406,    407,    409,    410,    412,    413,
  414,    416,    417,    419,    420,    422,    423,    424,    426,    427,    429,    430,    432,    433,    434,    436,
  437,    439,    440,    442,    443,    444,    446,    447,    449,    450,    451,    453,    454,    456,    457,    458,
  460,    461,    463,    464,    466,    467,    468,    470,    471,    472,    474,    475,    477,    478,    479,    481,
  482,    484,    485,    486,    488,    489,    491,    492,    493,    495,    496,    497,    499,    500,    501,    503,
  504,    506,    507,    508,    510,    511,    512,    514,    515,    516,    518,    519,    521,    522,    523,    525,
  526,    527,    529,    530,    531,    533,    534,    535,    537,    538,    539,    541,    542,    543,    545,    546,
  547,    549,    550,    551,    553,    554,    555,    557,    558,    559,    561,    562,    563,    564,    566,    567,
  568,    570,    571,    572,    574,    575,    576,    578,    579,    580,    581,    583,    584,    585,    587,    588,
  589,    590,    592,    593,    594,    596,    597,    598,    599,    601,    602,    603,    604,    606,    607,    608,
  609,    611,    612,    613,    615,    616,    617,    618,    620,    621,    622,    623,    625,    626,    627,    628,
  629,    631,    632,    633,    634,    636,    637,    638,    639,    641,    642,    643,    644,    645,    647,    648,
  649,    650,    652,    653,    654,    655,    656,    658,    659,    660,    661,    662,    664,    665,    666,    667,
  668,    670,    671,    672,    673,    674,    675,    677,    678,    679,    680,    681,    683,    684,    685,    686,
  687,    688,    690,    691,    692,    693,    694,    695,    696,    698,    699,    700,    701,    702,    703,    704,
  706,    707,    708,    709,    710,    711,    712,    714,    715,    716,    717,    718,    719,    720,    721,    722,
  724,    725,    726,    727,    728,    729,    730,    731,    732,    734,    735,    736,    737,    738,    739,    740,
  741,    742,    743,    744,    745,    747,    748,    749,    750,    751,    752,    753,    754,    755,    756,    757,
  758,    759,    760,    761,    762,    763,    765,    766,    767,    768,    769,    770,    771,    772,    773,    774,
  775,    776,    777,    778,    779,    780,    781,    782,    783,    784,    785,    786,    787,    788,    789,    790,
  791,    792,    793,    794,    795,    796,    797,    798,    799,    800,    801,    802,    803,    804,    805,    806,
  807,    808,    809,    810,    811,    812,    813,    813,    814,    815,    816,    817,    818,    819,    820,    821,
  822,    823,    824,    825,    826,    827,    828,    828,    829,    830,    831,    832,    833,    834,    835,    836,
  837,    838,    839,    839,    840,    841,    842,    843,    844,    845,    846,    847,    847,    848,    849,    850,
  851,    852,    853,    854,    854,    855,    856,    857,    858,    859,    860,    860,    861,    862,    863,    864,
  865,    865,    866,    867,    868,    869,    870,    870,    871,    872,    873,    874,    875,    875,    876,    877,
  878,    879,    879,    880,    881,    882,    883,    883,    884,    885,    886,    887,    887,    888,    889,    890,
  890,    891,    892,    893,    894,    894,    895,    896,    897,    897,    898,    899,    900,    900,    901,    902,
  903,    903,    904,    905,    906,    906,    907,    908,    908,    909,    910,    911,    911,    912,    913,    913,
  914,    915,    916,    916,    917,    918,    918,    919,    920,    920,    921,    922,    922,    923,    924,    925,
  925,    926,    927,    927,    928,    929,    929,    930,    930,    931,    932,    932,    933,    934,    934,    935,
  936,    936,    937,    938,    938,    939,    939,    940,    941,    941,    942,    943,    943,    944,    944,    945,
  946,    946,    947,    947,    948,    949,    949,    950,    950,    951,    951,    952,    953,    953,    954,    954,
  955,    955,    956,    957,    957,    958,    958,    959,    959,    960,    960,    961,    962,    962,    963,    963,
  964,    964,    965,    965,    966,    966,    967,    967,    968,    968,    969,    969,    970,    970,    971,    971,
  972,    972,    973,    973,    974,    974,    975,    975,    976,    976,    977,    977,    978,    978,    978,    979,
  979,    980,    980,    981,    981,    982,    982,    983,    983,    983,    984,    984,    985,    985,    986,    986,
  986,    987,    987,    988,    988,    988,    989,    989,    990,    990,    990,    991,    991,    992,    992,    992,
  993,    993,    994,    994,    994,    995,    995,    995,    996,    996,    997,    997,    997,    998,    998,    998,
  999,    999,    999,   1000,   1000,   1000,   1001,   1001,   1001,   1002,   1002,   1002,   1003,   1003,   1003,   1004,
 1004,   1004,   1004,   1005,   1005,   1005,   1006,   1006,   1006,   1006,   1007,   1007,   1007,   1008,   1008,   1008,
 1008,   1009,   1009,   1009,   1009,   1010,   1010,   1010,   1010,   1011,   1011,   1011,   1011,   1012,   1012,   1012,
 1012,   1013,   1013,   1013,   1013,   1014,   1014,   1014,   1014,   1014,   1015,   1015,   1015,   1015,   1015,   1016,
 1016,   1016,   1016,   1016,   1017,   1017,   1017,   1017,   1017,   1017,   1018,   1018,   1018,   1018,   1018,   1018,
 1019,   1019,   1019,   1019,   1019,   1019,   1019,   1020,   1020,   1020,   1020,   1020,   1020,   1020,   1020,   1021,
 1021,   1021,   1021,   1021,   1021,   1021,   1021,   1021,   1022,   1022,   1022,   1022,   1022,   1022,   1022,   1022,
 1022,   1022,   1022,   1022,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,
 1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,
 1024,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,
 1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1023,   1022,   1022,   1022,
 1022,   1022,   1022,   1022,   1022,   1022,   1022,   1022,   1022,   1021,   1021,   1021,   1021,   1021,   1021,   1021,
 1021,   1021,   1020,   1020,   1020,   1020,   1020,   1020,   1020,   1020,   1019,   1019,   1019,   1019,   1019,   1019,
 1019,   1018,   1018,   1018,   1018,   1018,   1018,   1017,   1017,   1017,   1017,   1017,   1017,   1016,   1016,   1016,
 1016,   1016,   1015,   1015,   1015,   1015,   1015,   1014,   1014,   1014,   1014,   1014,   1013,   1013,   1013,   1013,
 1012,   1012,   1012,   1012,   1011,   1011,   1011,   1011,   1010,   1010,   1010,   1010,   1009,   1009,   1009,   1009,
 1008,   1008,   1008,   1008,   1007,   1007,   1007,   1006,   1006,   1006,   1006,   1005,   1005,   1005,   1004,   1004,
 1004,   1004,   1003,   1003,   1003,   1002,   1002,   1002,   1001,   1001,   1001,   1000,   1000,   1000,    999,    999,
  999,    998,    998,    998,    997,    997,    997,    996,    996,    995,    995,    995,    994,    994,    994,    993,
  993,    992,    992,    992,    991,    991,    990,    990,    990,    989,    989,    988,    988,    988,    987,    987,
  986,    986,    986,    985,    985,    984,    984,    983,    983,    983,    982,    982,    981,    981,    980,    980,
  979,    979,    978,    978,    978,    977,    977,    976,    976,    975,    975,    974,    974,    973,    973,    972,
  972,    971,    971,    970,    970,    969,    969,    968,    968,    967,    967,    966,    966,    965,    965,    964,
  964,    963,    963,    962,    962,    961,    960,    960,    959,    959,    958,    958,    957,    957,    956,    955,
  955,    954,    954,    953,    953,    952,    951,    951,    950,    950,    949,    949,    948,    947,    947,    946,
  946,    945,    944,    944,    943,    943,    942,    941,    941,    940,    939,    939,    938,    938,    937,    936,
  936,    935,    934,    934,    933,    932,    932,    931,    930,    930,    929,    929,    928,    927,    927,    926,
  925,    925,    924,    923,    922,    922,    921,    920,    920,    919,    918,    918,    917,    916,    916,    915,
  914,    913,    913,    912,    911,    911,    910,    909,    908,    908,    907,    906,    906,    905,    904,    903,
  903,    902,    901,    900,    900,    899,    898,    897,    897,    896,    895,    894,    894,    893,    892,    891,
  890,    890,    889,    888,    887,    887,    886,    885,    884,    883,    883,    882,    881,    880,    879,    879,
  878,    877,    876,    875,    875,    874,    873,    872,    871,    870,    870,    869,    868,    867,    866,    865,
  865,    864,    863,    862,    861,    860,    860,    859,    858,    857,    856,    855,    854,    854,    853,    852,
  851,    850,    849,    848,    847,    847,    846,    845,    844,    843,    842,    841,    840,    839,    839,    838,
  837,    836,    835,    834,    833,    832,    831,    830,    829,    828,    828,    827,    826,    825,    824,    823,
  822,    821,    820,    819,    818,    817,    816,    815,    814,    813,    813,    812,    811,    810,    809,    808,
  807,    806,    805,    804,    803,    802,    801,    800,    799,    798,    797,    796,    795,    794,    793,    792,
  791,    790,    789,    788,    787,    786,    785,    784,    783,    782,    781,    780,    779,    778,    777,    776,
  775,    774,    773,    772,    771,    770,    769,    768,    767,    766,    765,    763,    762,    761,    760,    759,
  758,    757,    756,    755,    754,    753,    752,    751,    750,    749,    748,    747,    745,    744,    743,    742,
  741,    740,    739,    738,    737,    736,    735,    734,    732,    731,    730,    729,    728,    727,    726,    725,
  724,    722,    721,    720,    719,    718,    717,    716,    715,    714,    712,    711,    710,    709,    708,    707,
  706,    704,    703,    702,    701,    700,    699,    698,    696,    695,    694,    693,    692,    691,    690,    688,
  687,    686,    685,    684,    683,    681,    680,    679,    678,    677,    675,    674,    673,    672,    671,    670,
  668,    667,    666,    665,    664,    662,    661,    660,    659,    658,    656,    655,    654,    653,    652,    650,
  649,    648,    647,    645,    644,    643,    642,    641,    639,    638,    637,    636,    634,    633,    632,    631,
  629,    628,    627,    626,    625,    623,    622,    621,    620,    618,    617,    616,    615,    613,    612,    611,
  609,    608,    607,    606,    604,    603,    602,    601,    599,    598,    597,    596,    594,    593,    592,    590,
  589,    588,    587,    585,    584,    583,    581,    580,    579,    578,    576,    575,    574,    572,    571,    570,
  568,    567,    566,    564,    563,    562,    561,    559,    558,    557,    555,    554,    553,    551,    550,    549,
  547,    546,    545,    543,    542,    541,    539,    538,    537,    535,    534,    533,    531,    530,    529,    527,
  526,    525,    523,    522,    521,    519,    518,    516,    515,    514,    512,    511,    510,    508,    507,    506,
  504,    503,    501,    500,    499,    497,    496,    495,    493,    492,    491,    489,    488,    486,    485,    484,
  482,    481,    479,    478,    477,    475,    474,    472,    471,    470,    468,    467,    466,    464,    463,    461,
  460,    458,    457,    456,    454,    453,    451,    450,    449,    447,    446,    444,    443,    442,    440,    439,
  437,    436,    434,    433,    432,    430,    429,    427,    426,    424,    423,    422,    420,    419,    417,    416,
  414,    413,    412,    410,    409,    407,    406,    404,    403,    402,    400,    399,    397,    396,    394,    393,
  391,    390,    388,    387,    386,    384,    383,    381,    380,    378,    377,    375,    374,    372,    371,    369,
  368,    367,    365,    364,    362,    361,    359,    358,    356,    355,    353,    352,    350,    349,    347,    346,
  344,    343,    342,    340,    339,    337,    336,    334,    333,    331,    330,    328,    327,    325,    324,    322,
  321,    319,    318,    316,    315,    313,    312,    310,    309,    307,    306,    304,    303,    301,    300,    298,
  297,    295,    294,    292,    291,    289,    288,    286,    285,    283,    282,    280,    279,    277,    276,    274,
  273,    271,    270,    268,    267,    265,    264,    262,    260,    259,    257,    256,    254,    253,    251,    250,
  248,    247,    245,    244,    242,    241,    239,    238,    236,    235,    233,    232,    230,    228,    227,    225,
  224,    222,    221,    219,    218,    216,    215,    213,    212,    210,    209,    207,    205,    204,    202,    201,
  199,    198,    196,    195,    193,    192,    190,    188,    187,    185,    184,    182,    181,    179,    178,    176,
  175,    173,    171,    170,    168,    167,    165,    164,    162,    161,    159,    158,    156,    154,    153,    151,
  150,    148,    147,    145,    144,    142,    140,    139,    137,    136,    134,    133,    131,    130,    128,    126,
  125,    123,    122,    120,    119,    117,    115,    114,    112,    111,    109,    108,    106,    105,    103,    101,
  100,     98,     97,     95,     94,     92,     90,     89,     87,     86,     84,     83,     81,     80,     78,     76,
   75,     73,     72,     70,     69,     67,     65,     64,     62,     61,     59,     58,     56,     54,     53,     51,
   50,     48,     47,     45,     43,     42,     40,     39,     37,     36,     34,     32,     31,     29,     28,     26,
   25,     23,     21,     20,     18,     17,     15,     14,     12,     10,      9,      7,      6,      4,      3,      1,
    0,     -1,     -3,     -4,     -6,     -7,     -9,    -10,    -12,    -14,    -15,    -17,    -18,    -20,    -21,    -23,
  -25,    -26,    -28,    -29,    -31,    -32,    -34,    -36,    -37,    -39,    -40,    -42,    -43,    -45,    -47,    -48,
  -50,    -51,    -53,    -54,    -56,    -58,    -59,    -61,    -62,    -64,    -65,    -67,    -69,    -70,    -72,    -73,
  -75,    -76,    -78,    -80,    -81,    -83,    -84,    -86,    -87,    -89,    -90,    -92,    -94,    -95,    -97,    -98,
 -100,   -101,   -103,   -105,   -106,   -108,   -109,   -111,   -112,   -114,   -115,   -117,   -119,   -120,   -122,   -123,
 -125,   -126,   -128,   -130,   -131,   -133,   -134,   -136,   -137,   -139,   -140,   -142,   -144,   -145,   -147,   -148,
 -150,   -151,   -153,   -154,   -156,   -158,   -159,   -161,   -162,   -164,   -165,   -167,   -168,   -170,   -171,   -173,
 -175,   -176,   -178,   -179,   -181,   -182,   -184,   -185,   -187,   -188,   -190,   -192,   -193,   -195,   -196,   -198,
 -199,   -201,   -202,   -204,   -205,   -207,   -209,   -210,   -212,   -213,   -215,   -216,   -218,   -219,   -221,   -222,
 -224,   -225,   -227,   -228,   -230,   -232,   -233,   -235,   -236,   -238,   -239,   -241,   -242,   -244,   -245,   -247,
 -248,   -250,   -251,   -253,   -254,   -256,   -257,   -259,   -260,   -262,   -264,   -265,   -267,   -268,   -270,   -271,
 -273,   -274,   -276,   -277,   -279,   -280,   -282,   -283,   -285,   -286,   -288,   -289,   -291,   -292,   -294,   -295,
 -297,   -298,   -300,   -301,   -303,   -304,   -306,   -307,   -309,   -310,   -312,   -313,   -315,   -316,   -318,   -319,
 -321,   -322,   -324,   -325,   -327,   -328,   -330,   -331,   -333,   -334,   -336,   -337,   -339,   -340,   -342,   -343,
 -344,   -346,   -347,   -349,   -350,   -352,   -353,   -355,   -356,   -358,   -359,   -361,   -362,   -364,   -365,   -367,
 -368,   -369,   -371,   -372,   -374,   -375,   -377,   -378,   -380,   -381,   -383,   -384,   -386,   -387,   -388,   -390,
 -391,   -393,   -394,   -396,   -397,   -399,   -400,   -402,   -403,   -404,   -406,   -407,   -409,   -410,   -412,   -413,
 -414,   -416,   -417,   -419,   -420,   -422,   -423,   -424,   -426,   -427,   -429,   -430,   -432,   -433,   -434,   -436,
 -437,   -439,   -440,   -442,   -443,   -444,   -446,   -447,   -449,   -450,   -451,   -453,   -454,   -456,   -457,   -458,
 -460,   -461,   -463,   -464,   -466,   -467,   -468,   -470,   -471,   -472,   -474,   -475,   -477,   -478,   -479,   -481,
 -482,   -484,   -485,   -486,   -488,   -489,   -491,   -492,   -493,   -495,   -496,   -497,   -499,   -500,   -501,   -503,
 -504,   -506,   -507,   -508,   -510,   -511,   -512,   -514,   -515,   -516,   -518,   -519,   -521,   -522,   -523,   -525,
 -526,   -527,   -529,   -530,   -531,   -533,   -534,   -535,   -537,   -538,   -539,   -541,   -542,   -543,   -545,   -546,
 -547,   -549,   -550,   -551,   -553,   -554,   -555,   -557,   -558,   -559,   -561,   -562,   -563,   -564,   -566,   -567,
 -568,   -570,   -571,   -572,   -574,   -575,   -576,   -578,   -579,   -580,   -581,   -583,   -584,   -585,   -587,   -588,
 -589,   -590,   -592,   -593,   -594,   -596,   -597,   -598,   -599,   -601,   -602,   -603,   -604,   -606,   -607,   -608,
 -609,   -611,   -612,   -613,   -615,   -616,   -617,   -618,   -620,   -621,   -622,   -623,   -625,   -626,   -627,   -628,
 -629,   -631,   -632,   -633,   -634,   -636,   -637,   -638,   -639,   -641,   -642,   -643,   -644,   -645,   -647,   -648,
 -649,   -650,   -652,   -653,   -654,   -655,   -656,   -658,   -659,   -660,   -661,   -662,   -664,   -665,   -666,   -667,
 -668,   -670,   -671,   -672,   -673,   -674,   -675,   -677,   -678,   -679,   -680,   -681,   -683,   -684,   -685,   -686,
 -687,   -688,   -690,   -691,   -692,   -693,   -694,   -695,   -696,   -698,   -699,   -700,   -701,   -702,   -703,   -704,
 -706,   -707,   -708,   -709,   -710,   -711,   -712,   -714,   -715,   -716,   -717,   -718,   -719,   -720,   -721,   -722,
 -724,   -725,   -726,   -727,   -728,   -729,   -730,   -731,   -732,   -734,   -735,   -736,   -737,   -738,   -739,   -740,
 -741,   -742,   -743,   -744,   -745,   -747,   -748,   -749,   -750,   -751,   -752,   -753,   -754,   -755,   -756,   -757,
 -758,   -759,   -760,   -761,   -762,   -763,   -765,   -766,   -767,   -768,   -769,   -770,   -771,   -772,   -773,   -774,
 -775,   -776,   -777,   -778,   -779,   -780,   -781,   -782,   -783,   -784,   -785,   -786,   -787,   -788,   -789,   -790,
 -791,   -792,   -793,   -794,   -795,   -796,   -797,   -798,   -799,   -800,   -801,   -802,   -803,   -804,   -805,   -806,
 -807,   -808,   -809,   -810,   -811,   -812,   -813,   -813,   -814,   -815,   -816,   -817,   -818,   -819,   -820,   -821,
 -822,   -823,   -824,   -825,   -826,   -827,   -828,   -828,   -829,   -830,   -831,   -832,   -833,   -834,   -835,   -836,
 -837,   -838,   -839,   -839,   -840,   -841,   -842,   -843,   -844,   -845,   -846,   -847,   -847,   -848,   -849,   -850,
 -851,   -852,   -853,   -854,   -854,   -855,   -856,   -857,   -858,   -859,   -860,   -860,   -861,   -862,   -863,   -864,
 -865,   -865,   -866,   -867,   -868,   -869,   -870,   -870,   -871,   -872,   -873,   -874,   -875,   -875,   -876,   -877,
 -878,   -879,   -879,   -880,   -881,   -882,   -883,   -883,   -884,   -885,   -886,   -887,   -887,   -888,   -889,   -890,
 -890,   -891,   -892,   -893,   -894,   -894,   -895,   -896,   -897,   -897,   -898,   -899,   -900,   -900,   -901,   -902,
 -903,   -903,   -904,   -905,   -906,   -906,   -907,   -908,   -908,   -909,   -910,   -911,   -911,   -912,   -913,   -913,
 -914,   -915,   -916,   -916,   -917,   -918,   -918,   -919,   -920,   -920,   -921,   -922,   -922,   -923,   -924,   -925,
 -925,   -926,   -927,   -927,   -928,   -929,   -929,   -930,   -930,   -931,   -932,   -932,   -933,   -934,   -934,   -935,
 -936,   -936,   -937,   -938,   -938,   -939,   -939,   -940,   -941,   -941,   -942,   -943,   -943,   -944,   -944,   -945,
 -946,   -946,   -947,   -947,   -948,   -949,   -949,   -950,   -950,   -951,   -951,   -952,   -953,   -953,   -954,   -954,
 -955,   -955,   -956,   -957,   -957,   -958,   -958,   -959,   -959,   -960,   -960,   -961,   -962,   -962,   -963,   -963,
 -964,   -964,   -965,   -965,   -966,   -966,   -967,   -967,   -968,   -968,   -969,   -969,   -970,   -970,   -971,   -971,
 -972,   -972,   -973,   -973,   -974,   -974,   -975,   -975,   -976,   -976,   -977,   -977,   -978,   -978,   -978,   -979,
 -979,   -980,   -980,   -981,   -981,   -982,   -982,   -983,   -983,   -983,   -984,   -984,   -985,   -985,   -986,   -986,
 -986,   -987,   -987,   -988,   -988,   -988,   -989,   -989,   -990,   -990,   -990,   -991,   -991,   -992,   -992,   -992,
 -993,   -993,   -994,   -994,   -994,   -995,   -995,   -995,   -996,   -996,   -997,   -997,   -997,   -998,   -998,   -998,
 -999,   -999,   -999,  -1000,  -1000,  -1000,  -1001,  -1001,  -1001,  -1002,  -1002,  -1002,  -1003,  -1003,  -1003,  -1004,
-1004,  -1004,  -1004,  -1005,  -1005,  -1005,  -1006,  -1006,  -1006,  -1006,  -1007,  -1007,  -1007,  -1008,  -1008,  -1008,
-1008,  -1009,  -1009,  -1009,  -1009,  -1010,  -1010,  -1010,  -1010,  -1011,  -1011,  -1011,  -1011,  -1012,  -1012,  -1012,
-1012,  -1013,  -1013,  -1013,  -1013,  -1014,  -1014,  -1014,  -1014,  -1014,  -1015,  -1015,  -1015,  -1015,  -1015,  -1016,
-1016,  -1016,  -1016,  -1016,  -1017,  -1017,  -1017,  -1017,  -1017,  -1017,  -1018,  -1018,  -1018,  -1018,  -1018,  -1018,
-1019,  -1019,  -1019,  -1019,  -1019,  -1019,  -1019,  -1020,  -1020,  -1020,  -1020,  -1020,  -1020,  -1020,  -1020,  -1021,
-1021,  -1021,  -1021,  -1021,  -1021,  -1021,  -1021,  -1021,  -1022,  -1022,  -1022,  -1022,  -1022,  -1022,  -1022,  -1022,
-1022,  -1022,  -1022,  -1022,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,
-1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,
-1024,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,
-1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1023,  -1022,  -1022,  -1022,
-1022,  -1022,  -1022,  -1022,  -1022,  -1022,  -1022,  -1022,  -1022,  -1021,  -1021,  -1021,  -1021,  -1021,  -1021,  -1021,
-1021,  -1021,  -1020,  -1020,  -1020,  -1020,  -1020,  -1020,  -1020,  -1020,  -1019,  -1019,  -1019,  -1019,  -1019,  -1019,
-1019,  -1018,  -1018,  -1018,  -1018,  -1018,  -1018,  -1017,  -1017,  -1017,  -1017,  -1017,  -1017,  -1016,  -1016,  -1016,
-1016,  -1016,  -1015,  -1015,  -1015,  -1015,  -1015,  -1014,  -1014,  -1014,  -1014,  -1014,  -1013,  -1013,  -1013,  -1013,
-1012,  -1012,  -1012,  -1012,  -1011,  -1011,  -1011,  -1011,  -1010,  -1010,  -1010,  -1010,  -1009,  -1009,  -1009,  -1009,
-1008,  -1008,  -1008,  -1008,  -1007,  -1007,  -1007,  -1006,  -1006,  -1006,  -1006,  -1005,  -1005,  -1005,  -1004,  -1004,
-1004,  -1004,  -1003,  -1003,  -1003,  -1002,  -1002,  -1002,  -1001,  -1001,  -1001,  -1000,  -1000,  -1000,   -999,   -999,
 -999,   -998,   -998,   -998,   -997,   -997,   -997,   -996,   -996,   -995,   -995,   -995,   -994,   -994,   -994,   -993,
 -993,   -992,   -992,   -992,   -991,   -991,   -990,   -990,   -990,   -989,   -989,   -988,   -988,   -988,   -987,   -987,
 -986,   -986,   -986,   -985,   -985,   -984,   -984,   -983,   -983,   -983,   -982,   -982,   -981,   -981,   -980,   -980,
 -979,   -979,   -978,   -978,   -978,   -977,   -977,   -976,   -976,   -975,   -975,   -974,   -974,   -973,   -973,   -972,
 -972,   -971,   -971,   -970,   -970,   -969,   -969,   -968,   -968,   -967,   -967,   -966,   -966,   -965,   -965,   -964,
 -964,   -963,   -963,   -962,   -962,   -961,   -960,   -960,   -959,   -959,   -958,   -958,   -957,   -957,   -956,   -955,
 -955,   -954,   -954,   -953,   -953,   -952,   -951,   -951,   -950,   -950,   -949,   -949,   -948,   -947,   -947,   -946,
 -946,   -945,   -944,   -944,   -943,   -943,   -942,   -941,   -941,   -940,   -939,   -939,   -938,   -938,   -937,   -936,
 -936,   -935,   -934,   -934,   -933,   -932,   -932,   -931,   -930,   -930,   -929,   -929,   -928,   -927,   -927,   -926,
 -925,   -925,   -924,   -923,   -922,   -922,   -921,   -920,   -920,   -919,   -918,   -918,   -917,   -916,   -916,   -915,
 -914,   -913,   -913,   -912,   -911,   -911,   -910,   -909,   -908,   -908,   -907,   -906,   -906,   -905,   -904,   -903,
 -903,   -902,   -901,   -900,   -900,   -899,   -898,   -897,   -897,   -896,   -895,   -894,   -894,   -893,   -892,   -891,
 -890,   -890,   -889,   -888,   -887,   -887,   -886,   -885,   -884,   -883,   -883,   -882,   -881,   -880,   -879,   -879,
 -878,   -877,   -876,   -875,   -875,   -874,   -873,   -872,   -871,   -870,   -870,   -869,   -868,   -867,   -866,   -865,
 -865,   -864,   -863,   -862,   -861,   -860,   -860,   -859,   -858,   -857,   -856,   -855,   -854,   -854,   -853,   -852,
 -851,   -850,   -849,   -848,   -847,   -847,   -846,   -845,   -844,   -843,   -842,   -841,   -840,   -839,   -839,   -838,
 -837,   -836,   -835,   -834,   -833,   -832,   -831,   -830,   -829,   -828,   -828,   -827,   -826,   -825,   -824,   -823,
 -822,   -821,   -820,   -819,   -818,   -817,   -816,   -815,   -814,   -813,   -813,   -812,   -811,   -810,   -809,   -808,
 -807,   -806,   -805,   -804,   -803,   -802,   -801,   -800,   -799,   -798,   -797,   -796,   -795,   -794,   -793,   -792,
 -791,   -790,   -789,   -788,   -787,   -786,   -785,   -784,   -783,   -782,   -781,   -780,   -779,   -778,   -777,   -776,
 -775,   -774,   -773,   -772,   -771,   -770,   -769,   -768,   -767,   -766,   -765,   -763,   -762,   -761,   -760,   -759,
 -758,   -757,   -756,   -755,   -754,   -753,   -752,   -751,   -750,   -749,   -748,   -747,   -745,   -744,   -743,   -742,
 -741,   -740,   -739,   -738,   -737,   -736,   -735,   -734,   -732,   -731,   -730,   -729,   -728,   -727,   -726,   -725,
 -724,   -722,   -721,   -720,   -719,   -718,   -717,   -716,   -715,   -714,   -712,   -711,   -710,   -709,   -708,   -707,
 -706,   -704,   -703,   -702,   -701,   -700,   -699,   -698,   -696,   -695,   -694,   -693,   -692,   -691,   -690,   -688,
 -687,   -686,   -685,   -684,   -683,   -681,   -680,   -679,   -678,   -677,   -675,   -674,   -673,   -672,   -671,   -670,
 -668,   -667,   -666,   -665,   -664,   -662,   -661,   -660,   -659,   -658,   -656,   -655,   -654,   -653,   -652,   -650,
 -649,   -648,   -647,   -645,   -644,   -643,   -642,   -641,   -639,   -638,   -637,   -636,   -634,   -633,   -632,   -631,
 -629,   -628,   -627,   -626,   -625,   -623,   -622,   -621,   -620,   -618,   -617,   -616,   -615,   -613,   -612,   -611,
 -609,   -608,   -607,   -606,   -604,   -603,   -602,   -601,   -599,   -598,   -597,   -596,   -594,   -593,   -592,   -590,
 -589,   -588,   -587,   -585,   -584,   -583,   -581,   -580,   -579,   -578,   -576,   -575,   -574,   -572,   -571,   -570,
 -568,   -567,   -566,   -564,   -563,   -562,   -561,   -559,   -558,   -557,   -555,   -554,   -553,   -551,   -550,   -549,
 -547,   -546,   -545,   -543,   -542,   -541,   -539,   -538,   -537,   -535,   -534,   -533,   -531,   -530,   -529,   -527,
 -526,   -525,   -523,   -522,   -521,   -519,   -518,   -516,   -515,   -514,   -512,   -511,   -510,   -508,   -507,   -506,
 -504,   -503,   -501,   -500,   -499,   -497,   -496,   -495,   -493,   -492,   -491,   -489,   -488,   -486,   -485,   -484,
 -482,   -481,   -479,   -478,   -477,   -475,   -474,   -472,   -471,   -470,   -468,   -467,   -466,   -464,   -463,   -461,
 -460,   -458,   -457,   -456,   -454,   -453,   -451,   -450,   -449,   -447,   -446,   -444,   -443,   -442,   -440,   -439,
 -437,   -436,   -434,   -433,   -432,   -430,   -429,   -427,   -426,   -424,   -423,   -422,   -420,   -419,   -417,   -416,
 -414,   -413,   -412,   -410,   -409,   -407,   -406,   -404,   -403,   -402,   -400,   -399,   -397,   -396,   -394,   -393,
 -391,   -390,   -388,   -387,   -386,   -384,   -383,   -381,   -380,   -378,   -377,   -375,   -374,   -372,   -371,   -369,
 -368,   -367,   -365,   -364,   -362,   -361,   -359,   -358,   -356,   -355,   -353,   -352,   -350,   -349,   -347,   -346,
 -344,   -343,   -342,   -340,   -339,   -337,   -336,   -334,   -333,   -331,   -330,   -328,   -327,   -325,   -324,   -322,
 -321,   -319,   -318,   -316,   -315,   -313,   -312,   -310,   -309,   -307,   -306,   -304,   -303,   -301,   -300,   -298,
 -297,   -295,   -294,   -292,   -291,   -289,   -288,   -286,   -285,   -283,   -282,   -280,   -279,   -277,   -276,   -274,
 -273,   -271,   -270,   -268,   -267,   -265,   -264,   -262,   -260,   -259,   -257,   -256,   -254,   -253,   -251,   -250,
 -248,   -247,   -245,   -244,   -242,   -241,   -239,   -238,   -236,   -235,   -233,   -232,   -230,   -228,   -227,   -225,
 -224,   -222,   -221,   -219,   -218,   -216,   -215,   -213,   -212,   -210,   -209,   -207,   -205,   -204,   -202,   -201,
 -199,   -198,   -196,   -195,   -193,   -192,   -190,   -188,   -187,   -185,   -184,   -182,   -181,   -179,   -178,   -176,
 -175,   -173,   -171,   -170,   -168,   -167,   -165,   -164,   -162,   -161,   -159,   -158,   -156,   -154,   -153,   -151,
 -150,   -148,   -147,   -145,   -144,   -142,   -140,   -139,   -137,   -136,   -134,   -133,   -131,   -130,   -128,   -126,
 -125,   -123,   -122,   -120,   -119,   -117,   -115,   -114,   -112,   -111,   -109,   -108,   -106,   -105,   -103,   -101,
 -100,    -98,    -97,    -95,    -94,    -92,    -90,    -89,    -87,    -86,    -84,    -83,    -81,    -80,    -78,    -76,
  -75,    -73,    -72,    -70,    -69,    -67,    -65,    -64,    -62,    -61,    -59,    -58,    -56,    -54,    -53,    -51,
  -50,    -48,    -47,    -45,    -43,    -42,    -40,    -39,    -37,    -36,    -34,    -32,    -31,    -29,    -28,    -26,
  -25,    -23,    -21,    -20,    -18,    -17,    -15,    -14,    -12,    -10,     -9,     -7,     -6,     -4,     -3,     -1,
    0,
};



/**************************/
