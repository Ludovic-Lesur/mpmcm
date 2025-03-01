/*
 * simulation.c
 *
 *  Created on: 20 apr. 2024
 *      Author: Ludo
 */

#include "simulation.h"

#include "adc.h"
#include "measure.h"
#include "mpmcm_flags.h"
#include "types.h"

#ifdef MPMCM_ANALOG_SIMULATION

/*** SIMULATION global variables ***/

const uint8_t SIMULATION_GPIO_ACI_DETECT[MEASURE_NUMBER_OF_ACI_CHANNELS] = { 1, 0, 0, 0 };

const int16_t SIMULATION_ACV_BUFFER[SIMULATION_BUFFER_SIZE] = {
    0, 23, 46, 70, 93, 116, 140, 163, 186, 209,
    233, 256, 279, 302, 325, 347, 370, 393, 415, 438,
    460, 482, 504, 526, 548, 570, 591, 613, 634, 655,
    676, 697, 717, 738, 758, 778, 798, 818, 837, 856,
    875, 894, 913, 931, 949, 967, 985, 1002, 1019, 1036,
    1053, 1070, 1086, 1102, 1117, 1133, 1148, 1162, 1177, 1191,
    1205, 1219, 1232, 1245, 1258, 1270, 1282, 1294, 1305, 1316,
    1327, 1338, 1348, 1357, 1367, 1376, 1385, 1393, 1401, 1409,
    1417, 1424, 1430, 1437, 1443, 1448, 1454, 1459, 1463, 1467,
    1471, 1475, 1478, 1481, 1483, 1485, 1487, 1488, 1489, 1489,
    1490, 1489, 1489, 1488, 1487, 1485, 1483, 1481, 1478, 1475,
    1471, 1467, 1463, 1459, 1454, 1448, 1443, 1437, 1430, 1424,
    1417, 1409, 1401, 1393, 1385, 1376, 1367, 1357, 1348, 1338,
    1327, 1316, 1305, 1294, 1282, 1270, 1258, 1245, 1232, 1219,
    1205, 1191, 1177, 1162, 1148, 1133, 1117, 1102, 1086, 1070,
    1053, 1036, 1019, 1002, 985, 967, 949, 931, 913, 894,
    875, 856, 837, 818, 798, 778, 758, 738, 717, 697,
    676, 655, 634, 613, 591, 570, 548, 526, 504, 482,
    460, 438, 415, 393, 370, 347, 325, 302, 279, 256,
    233, 209, 186, 163, 140, 116, 93, 70, 46, 23,
    0, -23, -46, -70, -93, -116, -140, -163, -186, -209,
    -233, -256, -279, -302, -325, -347, -370, -393, -415, -438,
    -460, -482, -504, -526, -548, -570, -591, -613, -634, -655,
    -676, -697, -717, -738, -758, -778, -798, -818, -837, -856,
    -875, -894, -913, -931, -949, -967, -985, -1002, -1019, -1036,
    -1053, -1070, -1086, -1102, -1117, -1133, -1148, -1162, -1177, -1191,
    -1205, -1219, -1232, -1245, -1258, -1270, -1282, -1294, -1305, -1316,
    -1327, -1338, -1348, -1357, -1367, -1376, -1385, -1393, -1401, -1409,
    -1417, -1424, -1430, -1437, -1443, -1448, -1454, -1459, -1463, -1467,
    -1471, -1475, -1478, -1481, -1483, -1485, -1487, -1488, -1489, -1489,
    -1490, -1489, -1489, -1488, -1487, -1485, -1483, -1481, -1478, -1475,
    -1471, -1467, -1463, -1459, -1454, -1448, -1443, -1437, -1430, -1424,
    -1417, -1409, -1401, -1393, -1385, -1376, -1367, -1357, -1348, -1338,
    -1327, -1316, -1305, -1294, -1282, -1270, -1258, -1245, -1232, -1219,
    -1205, -1191, -1177, -1162, -1148, -1133, -1117, -1102, -1086, -1070,
    -1053, -1036, -1019, -1002, -985, -967, -949, -931, -913, -894,
    -875, -856, -837, -818, -798, -778, -758, -738, -717, -697,
    -676, -655, -634, -613, -591, -570, -548, -526, -504, -482,
    -460, -438, -415, -393, -370, -347, -325, -302, -279, -256,
    -233, -209, -186, -163, -140, -116, -93, -70, -46, -23
};

const int16_t SIMULATION_ACI_BUFFER[SIMULATION_BUFFER_SIZE] = {
    467, 481, 495, 509, 522, 535, 549, 562, 575, 587,
    600, 612, 625, 637, 649, 661, 673, 684, 695, 707,
    718, 728, 739, 750, 760, 770, 780, 790, 799, 809,
    818, 827, 835, 844, 852, 860, 868, 876, 883, 891,
    898, 904, 911, 917, 923, 929, 935, 940, 946, 951,
    955, 960, 964, 968, 972, 975, 979, 982, 985, 987,
    990, 992, 993, 995, 996, 998, 998, 999, 999, 1000,
    999, 999, 998, 998, 996, 995, 993, 992, 990, 987,
    985, 982, 979, 975, 972, 968, 964, 960, 955, 951,
    946, 940, 935, 929, 923, 917, 911, 904, 898, 891,
    883, 876, 868, 860, 852, 844, 835, 827, 818, 809,
    799, 790, 780, 770, 760, 750, 739, 728, 718, 707,
    695, 684, 673, 661, 649, 637, 625, 612, 600, 587,
    575, 562, 549, 535, 522, 509, 495, 481, 467, 453,
    439, 425, 411, 397, 382, 368, 353, 338, 323, 309,
    294, 278, 263, 248, 233, 218, 202, 187, 171, 156,
    140, 125, 109, 94, 78, 62, 47, 31, 15, 0,
    -15, -31, -47, -62, -78, -94, -109, -125, -140, -156,
    -171, -187, -202, -218, -233, -248, -263, -278, -294, -309,
    -323, -338, -353, -368, -382, -397, -411, -425, -439, -453,
    -467, -481, -495, -509, -522, -535, -549, -562, -575, -587,
    -600, -612, -625, -637, -649, -661, -673, -684, -695, -707,
    -718, -728, -739, -750, -760, -770, -780, -790, -799, -809,
    -818, -827, -835, -844, -852, -860, -868, -876, -883, -891,
    -898, -904, -911, -917, -923, -929, -935, -940, -946, -951,
    -955, -960, -964, -968, -972, -975, -979, -982, -985, -987,
    -990, -992, -993, -995, -996, -998, -998, -999, -999, -1000,
    -999, -999, -998, -998, -996, -995, -993, -992, -990, -987,
    -985, -982, -979, -975, -972, -968, -964, -960, -955, -951,
    -946, -940, -935, -929, -923, -917, -911, -904, -898, -891,
    -883, -876, -868, -860, -852, -844, -835, -827, -818, -809,
    -799, -790, -780, -770, -760, -750, -739, -728, -718, -707,
    -695, -684, -673, -661, -649, -637, -625, -612, -600, -587,
    -575, -562, -549, -535, -522, -509, -495, -481, -467, -453,
    -439, -425, -411, -397, -382, -368, -353, -338, -323, -309,
    -294, -278, -263, -248, -233, -218, -202, -187, -171, -156,
    -140, -125, -109, -94, -78, -62, -47, -31, -15, 0,
    15, 31, 47, 62, 78, 94, 109, 125, 140, 156,
    171, 187, 202, 218, 233, 248, 263, 278, 294, 309,
    323, 338, 353, 368, 382, 397, 411, 425, 439, 453
};

#endif /* MPMCM_ANALOG_SIMULATION */
