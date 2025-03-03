/*
 * temperature.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "measurements/temperature.h"

/**
* The NTC table has 1025 interpolation points.
* Unit:0.1 °C
*
*/
int NTC_table[1025] = {
  4080, 3474, 2868, 2565, 2369, 2226, 2115,
  2026, 1950, 1886, 1829, 1780, 1735, 1695,
  1658, 1624, 1593, 1565, 1538, 1513, 1489,
  1467, 1446, 1426, 1407, 1389, 1372, 1356,
  1340, 1325, 1311, 1297, 1284, 1271, 1258,
  1246, 1235, 1224, 1213, 1202, 1192, 1182,
  1172, 1163, 1154, 1145, 1136, 1128, 1120,
  1112, 1104, 1096, 1089, 1081, 1074, 1067,
  1060, 1053, 1047, 1040, 1034, 1028, 1021,
  1015, 1009, 1004, 998, 992, 987, 981, 976,
  971, 966, 961, 956, 951, 946, 941, 936, 932,
  927, 922, 918, 914, 909, 905, 901, 896, 892,
  888, 884, 880, 876, 872, 869, 865, 861, 857,
  854, 850, 846, 843, 839, 836, 832, 829, 826,
  822, 819, 816, 813, 809, 806, 803, 800, 797,
  794, 791, 788, 785, 782, 779, 776, 773, 770,
  768, 765, 762, 759, 757, 754, 751, 748, 746,
  743, 741, 738, 735, 733, 730, 728, 725, 723,
  721, 718, 716, 713, 711, 709, 706, 704, 702,
  699, 697, 695, 693, 690, 688, 686, 684, 681,
  679, 677, 675, 673, 671, 669, 667, 665, 663,
  660, 658, 656, 654, 652, 650, 648, 646, 645,
  643, 641, 639, 637, 635, 633, 631, 629, 627,
  626, 624, 622, 620, 618, 616, 615, 613, 611,
  609, 608, 606, 604, 602, 601, 599, 597, 595,
  594, 592, 590, 589, 587, 585, 584, 582, 580,
  579, 577, 576, 574, 572, 571, 569, 568, 566,
  565, 563, 561, 560, 558, 557, 555, 554, 552,
  551, 549, 548, 546, 545, 543, 542, 540, 539,
  537, 536, 535, 533, 532, 530, 529, 527, 526,
  525, 523, 522, 520, 519, 518, 516, 515, 513,
  512, 511, 509, 508, 507, 505, 504, 503, 501,
  500, 499, 497, 496, 495, 493, 492, 491, 490,
  488, 487, 486, 484, 483, 482, 481, 479, 478,
  477, 476, 474, 473, 472, 471, 469, 468, 467,
  466, 464, 463, 462, 461, 460, 458, 457, 456,
  455, 454, 452, 451, 450, 449, 448, 447, 445,
  444, 443, 442, 441, 440, 438, 437, 436, 435,
  434, 433, 432, 430, 429, 428, 427, 426, 425,
  424, 423, 422, 420, 419, 418, 417, 416, 415,
  414, 413, 412, 411, 409, 408, 407, 406, 405,
  404, 403, 402, 401, 400, 399, 398, 397, 396,
  394, 393, 392, 391, 390, 389, 388, 387, 386,
  385, 384, 383, 382, 381, 380, 379, 378, 377,
  376, 375, 374, 373, 372, 371, 370, 369, 368,
  367, 366, 365, 364, 363, 362, 361, 360, 359,
  358, 357, 356, 355, 354, 353, 352, 351, 350,
  349, 348, 347, 346, 345, 344, 343, 342, 341,
  340, 339, 338, 337, 336, 335, 334, 333, 333,
  332, 331, 330, 329, 328, 327, 326, 325, 324,
  323, 322, 321, 320, 319, 318, 317, 317, 316,
  315, 314, 313, 312, 311, 310, 309, 308, 307,
  306, 305, 304, 304, 303, 302, 301, 300, 299,
  298, 297, 296, 295, 294, 294, 293, 292, 291,
  290, 289, 288, 287, 286, 285, 285, 284, 283,
  282, 281, 280, 279, 278, 277, 276, 276, 275,
  274, 273, 272, 271, 270, 269, 268, 268, 267,
  266, 265, 264, 263, 262, 261, 261, 260, 259,
  258, 257, 256, 255, 254, 253, 253, 252, 251,
  250, 249, 248, 247, 247, 246, 245, 244, 243,
  242, 241, 240, 240, 239, 238, 237, 236, 235,
  234, 233, 233, 232, 231, 230, 229, 228, 227,
  227, 226, 225, 224, 223, 222, 221, 221, 220,
  219, 218, 217, 216, 215, 215, 214, 213, 212,
  211, 210, 209, 209, 208, 207, 206, 205, 204,
  203, 203, 202, 201, 200, 199, 198, 197, 197,
  196, 195, 194, 193, 192, 191, 191, 190, 189,
  188, 187, 186, 185, 185, 184, 183, 182, 181,
  180, 179, 179, 178, 177, 176, 175, 174, 174,
  173, 172, 171, 170, 169, 168, 168, 167, 166,
  165, 164, 163, 162, 162, 161, 160, 159, 158,
  157, 156, 156, 155, 154, 153, 152, 151, 150,
  150, 149, 148, 147, 146, 145, 144, 143, 143,
  142, 141, 140, 139, 138, 137, 137, 136, 135,
  134, 133, 132, 131, 131, 130, 129, 128, 127,
  126, 125, 124, 124, 123, 122, 121, 120, 119,
  118, 117, 117, 116, 115, 114, 113, 112, 111,
  110, 110, 109, 108, 107, 106, 105, 104, 103,
  102, 102, 101, 100, 99, 98, 97, 96, 95, 94,
  94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 85,
  84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74,
  74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64,
  63, 62, 62, 61, 60, 59, 58, 57, 56, 55, 54,
  53, 52, 51, 50, 49, 48, 47, 46, 46, 45, 44,
  43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33,
  32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22,
  21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11,
  10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2,
  -3, -4, -6, -7, -8, -9, -10, -11, -12, -13,
  -14, -15, -16, -17, -18, -19, -21, -22, -23,
  -24, -25, -26, -27, -28, -29, -30, -32, -33,
  -34, -35, -36, -37, -38, -40, -41, -42, -43,
  -44, -45, -46, -48, -49, -50, -51, -52, -54,
  -55, -56, -57, -58, -60, -61, -62, -63, -64,
  -66, -67, -68, -69, -71, -72, -73, -74, -76,
  -77, -78, -79, -81, -82, -83, -85, -86, -87,
  -89, -90, -91, -93, -94, -95, -97, -98, -99,
  -101, -102, -103, -105, -106, -108, -109,
  -110, -112, -113, -115, -116, -118, -119,
  -121, -122, -124, -125, -127, -128, -130,
  -131, -133, -134, -136, -137, -139, -140,
  -142, -144, -145, -147, -148, -150, -152,
  -153, -155, -157, -158, -160, -162, -164,
  -165, -167, -169, -171, -172, -174, -176,
  -178, -180, -182, -183, -185, -187, -189,
  -191, -193, -195, -197, -199, -201, -203,
  -205, -207, -209, -211, -213, -216, -218,
  -220, -222, -224, -227, -229, -231, -234,
  -236, -238, -241, -243, -246, -248, -251,
  -253, -256, -258, -261, -264, -266, -269,
  -272, -275, -278, -281, -284, -287, -290,
  -293, -296, -299, -303, -306, -309, -313,
  -316, -320, -324, -327, -331, -335, -339,
  -343, -347, -352, -356, -360, -365, -370,
  -375, -380, -385, -390, -396, -401, -407,
  -414, -420, -427, -434, -441, -449, -457,
  -465, -474, -484, -494, -505, -517, -530,
  -544, -560, -579, -600, -625, -657, -700,
  -769, -838
};
 
 
 
/**
* \brief    Converts the ADC result into a temperature value.
*
*           P1 and p2 are the interpolating point just before and after the
*           ADC value. The function interpolates between these two points
*           The resulting code is very small and fast.
*           Only one integer multiplication is used.
*           The compiler can replace the division by a shift operation.
*
*           In the temperature range from 0°C to 120°C the error
*           caused by the usage of a table is 0.125°C
*
* \param    adc_value  The converted ADC result
* \return              The temperature in 0.1 °C
*
*/
float NTC_ADC2Temperature(uint16_t adc_value){
 
  float p1,p2;
  /* Estimate the interpolating point before and after the ADC value. */
  p1 = (float)NTC_table[ (adc_value >> 2)  ];
  p2 = (float)NTC_table[ (adc_value >> 2)+1];
 
  /* Interpolate between both points. */
  return (p1 - ( (p1-p2) * (float)(adc_value & 0x0003) ) / 4.0f) / 10.0f;
};
