EESchema Schematic File Version 5
LIBS:chibi_2024-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
Comment5 ""
Comment6 ""
Comment7 ""
Comment8 ""
Comment9 ""
$EndDescr
$Comp
L components:7seg_4digit_cc D14
U 1 1 5900990E
P 7550 2450
AR Path="/5901390A/5900990E" Ref="D14"  Part="1" 
AR Path="/59008B1D/5900990E" Ref="D10"  Part="1" 
F 0 "D10" H 8100 2750 60  0000 C CNN
F 1 "7seg_4digit_cc" H 7550 2450 60  0000 C CNN
F 2 "footprints:7seg_4digit_cc_modified" H 8550 2350 60  0001 C CNN
F 3 "" H 8550 2350 60  0001 C CNN
	1    7550 2450
	1    0    0    -1  
$EndComp
Text HLabel 3700 1100 0    60   Input ~ 0
SEG_A
Text HLabel 3700 1200 0    60   Input ~ 0
SEG_B
Text HLabel 3700 1300 0    60   Input ~ 0
SEG_C
Text HLabel 3700 1400 0    60   Input ~ 0
SEG_D
Text HLabel 3700 1500 0    60   Input ~ 0
SEG_E
Text HLabel 3700 1600 0    60   Input ~ 0
SEG_F
Text HLabel 3700 1700 0    60   Input ~ 0
SEG_G
Text HLabel 3700 1800 0    60   Input ~ 0
SEG_DP
Text HLabel 8100 5200 2    60   Input ~ 0
GND
Text HLabel 8100 5300 2    60   Input ~ 0
VDD
Text HLabel 8100 5400 2    60   Input ~ 0
CLK
Text HLabel 8100 5500 2    60   Input ~ 0
LE
Text HLabel 8100 5600 2    60   Input ~ 0
~OE
Text HLabel 8100 4550 2    60   Input ~ 0
SDI
Text HLabel 3700 4550 0    60   Input ~ 0
SDO
Text HLabel 3700 5200 0    60   Input ~ 0
GND
Text HLabel 3700 5300 0    60   Input ~ 0
VDD
Text HLabel 3700 5400 0    60   Input ~ 0
CLK
Text HLabel 3700 5500 0    60   Input ~ 0
LE
Text HLabel 3700 5600 0    60   Input ~ 0
~OE
Text HLabel 8100 1100 2    60   Input ~ 0
SEG_A
Text HLabel 8100 1200 2    60   Input ~ 0
SEG_B
Text HLabel 8100 1300 2    60   Input ~ 0
SEG_C
Text HLabel 8100 1400 2    60   Input ~ 0
SEG_D
Text HLabel 8100 1500 2    60   Input ~ 0
SEG_E
Text HLabel 8100 1600 2    60   Input ~ 0
SEG_F
Text HLabel 8100 1700 2    60   Input ~ 0
SEG_G
Text HLabel 8100 1800 2    60   Input ~ 0
SEG_DP
Text Notes 7300 6900 0    197  ~ 39
Single multiplexing unit
Text Notes 8150 7100 0    118  ~ 0
4 displays, 1 driver
$Comp
L chibi_2024-rescue:R R41
U 1 1 590099AE
P 6250 5950
AR Path="/5901390A/590099AE" Ref="R41"  Part="1" 
AR Path="/59008B1D/590099AE" Ref="R39"  Part="1" 
F 0 "R39" V 6330 5950 50  0000 C CNN
F 1 "R" V 6250 5950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6180 5950 50  0001 C CNN
F 3 "" H 6250 5950 50  0000 C CNN
	1    6250 5950
	1    0    0    1   
$EndComp
$Comp
L chibi_2024-rescue:R R38
U 1 1 590DC56C
P 6050 5950
AR Path="/59008B1D/590DC56C" Ref="R38"  Part="1" 
AR Path="/5901390A/590DC56C" Ref="R40"  Part="1" 
F 0 "R38" V 6130 5950 50  0000 C CNN
F 1 "R" V 6050 5950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5980 5950 50  0001 C CNN
F 3 "" H 6050 5950 50  0000 C CNN
	1    6050 5950
	1    0    0    1   
$EndComp
Text HLabel 3700 6200 0    63   Input ~ 0
R_SET_0
Text HLabel 3700 6300 0    63   Input ~ 0
R_SET_1
Text HLabel 8100 6300 2    63   Input ~ 0
R_SET_1
Text HLabel 8100 6200 2    63   Input ~ 0
R_SET_0
Wire Wire Line
	8100 4550 6950 4550
Wire Wire Line
	3700 4550 4850 4550
Wire Wire Line
	6050 5800 6050 5700
Wire Wire Line
	6050 5700 6150 5700
Wire Wire Line
	6250 5700 6250 5800
Wire Wire Line
	6150 5100 6150 5700
Wire Wire Line
	6050 6200 6050 6100
Wire Wire Line
	6250 6100 6250 6300
Wire Wire Line
	6150 5700 6250 5700
$Comp
L components:7seg_4digit_cc D17
U 1 1 59009923
P 4250 2450
AR Path="/5901390A/59009923" Ref="D17"  Part="1" 
AR Path="/59008B1D/59009923" Ref="D13"  Part="1" 
F 0 "D13" H 4800 2750 60  0000 C CNN
F 1 "7seg_4digit_cc" H 4250 2450 60  0000 C CNN
F 2 "footprints:7seg_4digit_cc_modified" H 5250 2350 60  0001 C CNN
F 3 "" H 5250 2350 60  0001 C CNN
	1    4250 2450
	1    0    0    -1  
$EndComp
$Comp
L components:7seg_4digit_cc D16
U 1 1 5900991C
P 5350 2450
AR Path="/5901390A/5900991C" Ref="D16"  Part="1" 
AR Path="/59008B1D/5900991C" Ref="D12"  Part="1" 
F 0 "D12" H 5900 2750 60  0000 C CNN
F 1 "7seg_4digit_cc" H 5350 2450 60  0000 C CNN
F 2 "footprints:7seg_4digit_cc_modified" H 6350 2350 60  0001 C CNN
F 3 "" H 6350 2350 60  0001 C CNN
	1    5350 2450
	1    0    0    -1  
$EndComp
$Comp
L components:7seg_4digit_cc D15
U 1 1 59009915
P 6450 2450
AR Path="/5901390A/59009915" Ref="D15"  Part="1" 
AR Path="/59008B1D/59009915" Ref="D11"  Part="1" 
F 0 "D11" H 7000 2750 60  0000 C CNN
F 1 "7seg_4digit_cc" H 6450 2450 60  0000 C CNN
F 2 "footprints:7seg_4digit_cc_modified" H 7450 2350 60  0001 C CNN
F 3 "" H 7450 2350 60  0001 C CNN
	1    6450 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5200 6550 5200
Wire Wire Line
	3700 5300 6650 5300
Wire Wire Line
	3700 5400 6000 5400
Wire Wire Line
	3700 5500 5900 5500
Wire Wire Line
	3700 5600 5800 5600
Wire Wire Line
	5800 5100 5800 5600
Connection ~ 5800 5600
Wire Wire Line
	5800 5600 8100 5600
Wire Wire Line
	5900 5500 5900 5100
Connection ~ 5900 5500
Wire Wire Line
	5900 5500 8100 5500
Wire Wire Line
	6000 5100 6000 5400
Connection ~ 6000 5400
Wire Wire Line
	6000 5400 8100 5400
Wire Wire Line
	6550 5100 6550 5200
Connection ~ 6550 5200
Wire Wire Line
	6550 5200 8100 5200
Wire Wire Line
	6650 5100 6650 5300
Connection ~ 6650 5300
Wire Wire Line
	6650 5300 8100 5300
Connection ~ 6050 6200
Wire Wire Line
	6050 6200 8100 6200
Connection ~ 6150 5700
Connection ~ 6250 6300
Wire Wire Line
	6250 6300 8100 6300
Wire Wire Line
	3700 6200 6050 6200
Wire Wire Line
	3700 6300 6250 6300
$Comp
L components:MBI5026 U9
U 1 1 5900992A
P 5900 4550
AR Path="/5901390A/5900992A" Ref="U9"  Part="1" 
AR Path="/59008B1D/5900992A" Ref="U8"  Part="1" 
F 0 "U8" H 6600 4950 60  0000 C CNN
F 1 "MBI5026" H 5900 4550 60  0000 C CNN
F 2 "footprints:SSOP24-1.0-nosilk" H 5900 4550 60  0001 C CNN
F 3 "" H 5900 4550 60  0001 C CNN
	1    5900 4550
	-1   0    0    1   
$EndComp
Wire Wire Line
	5150 3950 5150 3550
Wire Wire Line
	5150 3550 4400 3550
Wire Wire Line
	4400 3550 4400 3000
Wire Wire Line
	4300 3000 4300 3650
Wire Wire Line
	4300 3650 5250 3650
Wire Wire Line
	5250 3650 5250 3950
Wire Wire Line
	5350 3950 5350 3750
Wire Wire Line
	5350 3750 4200 3750
Wire Wire Line
	4200 3750 4200 3000
Wire Wire Line
	5850 3950 5850 3250
Wire Wire Line
	5850 3250 5500 3250
Wire Wire Line
	5500 3250 5500 3000
Wire Wire Line
	5400 3000 5400 3350
Wire Wire Line
	5400 3350 5750 3350
Wire Wire Line
	5750 3350 5750 3950
Wire Wire Line
	5650 3450 5300 3450
Wire Wire Line
	5300 3450 5300 3000
Wire Wire Line
	5650 3450 5650 3950
Wire Wire Line
	4100 3000 4100 3500
Wire Wire Line
	4100 3500 5550 3500
Wire Wire Line
	5550 3500 5550 3950
Wire Wire Line
	5450 3950 5450 3550
Wire Wire Line
	5450 3550 5200 3550
Wire Wire Line
	5200 3550 5200 3000
Wire Wire Line
	5950 3950 5950 3150
Wire Wire Line
	5950 3150 6600 3150
Wire Wire Line
	6600 3150 6600 3000
Wire Wire Line
	6500 3000 6500 3250
Wire Wire Line
	6500 3250 6050 3250
Wire Wire Line
	6050 3250 6050 3950
Wire Wire Line
	6150 3950 6150 3350
Wire Wire Line
	6150 3350 6400 3350
Wire Wire Line
	6400 3350 6400 3000
Wire Wire Line
	6300 3000 6300 3450
Wire Wire Line
	6300 3450 6250 3450
Wire Wire Line
	6250 3450 6250 3950
Wire Wire Line
	6350 3500 7700 3500
Wire Wire Line
	7700 3500 7700 3000
Wire Wire Line
	6350 3500 6350 3950
Wire Wire Line
	7600 3000 7600 3550
Wire Wire Line
	7600 3550 6450 3550
Wire Wire Line
	6450 3550 6450 3950
Wire Wire Line
	6550 3950 6550 3700
Wire Wire Line
	6550 3700 7450 3700
Wire Wire Line
	7450 3700 7450 3000
Wire Wire Line
	7450 3000 7500 3000
Wire Wire Line
	7400 3000 7400 3850
Wire Wire Line
	7400 3850 6650 3850
Wire Wire Line
	6650 3850 6650 3950
Wire Wire Line
	3700 1400 4100 1400
Wire Wire Line
	3700 1100 4000 1100
Wire Wire Line
	3700 1300 4200 1300
Wire Wire Line
	3700 1500 4500 1500
Wire Wire Line
	3700 1600 4400 1600
Wire Wire Line
	3700 1200 3900 1200
Wire Wire Line
	3700 1700 4300 1700
Wire Wire Line
	3700 1800 4600 1800
Wire Wire Line
	4300 1900 4300 1700
Connection ~ 4300 1700
Wire Wire Line
	4200 1900 4200 1300
Connection ~ 4200 1300
Wire Wire Line
	4600 1900 4600 1800
Connection ~ 4600 1800
Wire Wire Line
	4100 1900 4100 1400
Connection ~ 4100 1400
Wire Wire Line
	4500 1900 4500 1500
Connection ~ 4500 1500
Wire Wire Line
	4000 1900 4000 1100
Connection ~ 4000 1100
Wire Wire Line
	3900 1200 3900 1900
Connection ~ 3900 1200
Wire Wire Line
	4400 1900 4400 1600
Connection ~ 4400 1600
Wire Wire Line
	4000 1100 5100 1100
Wire Wire Line
	4200 1300 5300 1300
Wire Wire Line
	4400 1600 5500 1600
Wire Wire Line
	4300 1700 5400 1700
Wire Wire Line
	3900 1200 5000 1200
Wire Wire Line
	4500 1500 5600 1500
Wire Wire Line
	4100 1400 5200 1400
Wire Wire Line
	4600 1800 5700 1800
Wire Wire Line
	5000 1900 5000 1200
Connection ~ 5000 1200
Wire Wire Line
	5000 1200 6100 1200
Wire Wire Line
	5100 1100 5100 1900
Connection ~ 5100 1100
Wire Wire Line
	5100 1100 6200 1100
Wire Wire Line
	5200 1900 5200 1400
Connection ~ 5200 1400
Wire Wire Line
	5200 1400 6300 1400
Wire Wire Line
	5300 1300 5300 1900
Connection ~ 5300 1300
Wire Wire Line
	5300 1300 6400 1300
Wire Wire Line
	5400 1900 5400 1700
Connection ~ 5400 1700
Wire Wire Line
	5400 1700 6500 1700
Wire Wire Line
	5500 1600 5500 1900
Connection ~ 5500 1600
Wire Wire Line
	5500 1600 6600 1600
Wire Wire Line
	5600 1900 5600 1500
Connection ~ 5600 1500
Wire Wire Line
	5600 1500 6700 1500
Wire Wire Line
	5700 1800 5700 1900
Connection ~ 5700 1800
Wire Wire Line
	5700 1800 6800 1800
Wire Wire Line
	6100 1900 6100 1200
Connection ~ 6100 1200
Wire Wire Line
	6100 1200 7200 1200
Wire Wire Line
	6200 1100 6200 1900
Connection ~ 6200 1100
Wire Wire Line
	6200 1100 7300 1100
Wire Wire Line
	6300 1900 6300 1400
Connection ~ 6300 1400
Wire Wire Line
	6300 1400 7400 1400
Wire Wire Line
	6400 1300 6400 1900
Connection ~ 6400 1300
Wire Wire Line
	6400 1300 7500 1300
Wire Wire Line
	6500 1900 6500 1700
Connection ~ 6500 1700
Wire Wire Line
	6500 1700 7600 1700
Wire Wire Line
	6600 1900 6600 1600
Connection ~ 6600 1600
Wire Wire Line
	6600 1600 7700 1600
Wire Wire Line
	6700 1500 6700 1900
Connection ~ 6700 1500
Wire Wire Line
	6700 1500 7800 1500
Wire Wire Line
	6800 1900 6800 1800
Connection ~ 6800 1800
Wire Wire Line
	6800 1800 7900 1800
Wire Wire Line
	7200 1900 7200 1200
Connection ~ 7200 1200
Wire Wire Line
	7200 1200 8100 1200
Wire Wire Line
	7300 1100 7300 1900
Connection ~ 7300 1100
Wire Wire Line
	7300 1100 8100 1100
Wire Wire Line
	7400 1900 7400 1400
Connection ~ 7400 1400
Wire Wire Line
	7400 1400 8100 1400
Wire Wire Line
	7500 1300 7500 1900
Connection ~ 7500 1300
Wire Wire Line
	7500 1300 8100 1300
Wire Wire Line
	7600 1900 7600 1700
Connection ~ 7600 1700
Wire Wire Line
	7600 1700 8100 1700
Wire Wire Line
	7700 1600 7700 1900
Connection ~ 7700 1600
Wire Wire Line
	7700 1600 8100 1600
Wire Wire Line
	7800 1900 7800 1500
Connection ~ 7800 1500
Wire Wire Line
	7800 1500 8100 1500
Wire Wire Line
	7900 1800 7900 1900
Connection ~ 7900 1800
Wire Wire Line
	7900 1800 8100 1800
$EndSCHEMATC
