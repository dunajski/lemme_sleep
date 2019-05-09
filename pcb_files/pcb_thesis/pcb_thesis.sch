EESchema Schematic File Version 4
LIBS:pcb_thesis-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R R?
U 1 1 5C4616F0
P 3200 2350
F 0 "R?" V 3407 2350 50  0000 C CNN
F 1 "10k" V 3200 2350 50  0000 C CNN
F 2 "" V 3130 2350 50  0001 C CNN
F 3 "~" H 3200 2350 50  0001 C CNN
	1    3200 2350
	0    -1   -1   0   
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5C461B67
P 1800 2250
F 0 "#FLG?" H 1800 2325 50  0001 C CNN
F 1 "PWR_FLAG" H 1800 2423 50  0000 C CNN
F 2 "" H 1800 2250 50  0001 C CNN
F 3 "~" H 1800 2250 50  0001 C CNN
	1    1800 2250
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5C461CB9
P 1800 1950
F 0 "#PWR?" H 1800 1800 50  0001 C CNN
F 1 "+5V" H 1815 2123 50  0000 C CNN
F 2 "" H 1800 1950 50  0001 C CNN
F 3 "" H 1800 1950 50  0001 C CNN
	1    1800 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 2250 1800 1950
$Comp
L power:+5V #PWR?
U 1 1 5C461DE5
P 2750 2200
F 0 "#PWR?" H 2750 2050 50  0001 C CNN
F 1 "+5V" H 2765 2373 50  0000 C CNN
F 2 "" H 2750 2200 50  0001 C CNN
F 3 "" H 2750 2200 50  0001 C CNN
	1    2750 2200
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_DPST_x2 SW?
U 1 1 5C461EFF
P 3450 2750
F 0 "SW?" V 3500 3100 50  0000 R CNN
F 1 "RESET SW" V 3350 3200 50  0000 R CNN
F 2 "" H 3450 2750 50  0001 C CNN
F 3 "" H 3450 2750 50  0001 C CNN
	1    3450 2750
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C461FC9
P 6150 1900
F 0 "#PWR?" H 6150 1650 50  0001 C CNN
F 1 "GND" H 6155 1727 50  0000 C CNN
F 2 "" H 6150 1900 50  0001 C CNN
F 3 "" H 6150 1900 50  0001 C CNN
	1    6150 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y?
U 1 1 5C462107
P 4150 2850
F 0 "Y?" V 3900 2850 50  0000 L CNN
F 1 "8MHz" V 4400 2800 50  0000 L CNN
F 2 "" H 4150 2850 50  0001 C CNN
F 3 "~" H 4150 2850 50  0001 C CNN
	1    4150 2850
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 5C46250B
P 3850 2700
F 0 "C?" V 4000 2550 50  0000 C CNN
F 1 "22p" V 4000 2800 50  0000 C CNN
F 2 "" H 3888 2550 50  0001 C CNN
F 3 "~" H 3850 2700 50  0001 C CNN
	1    3850 2700
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C?
U 1 1 5C4625EF
P 3850 3000
F 0 "C?" V 3700 2900 50  0000 C CNN
F 1 "22p" V 3700 3050 50  0000 C CNN
F 2 "" H 3888 2850 50  0001 C CNN
F 3 "~" H 3850 3000 50  0001 C CNN
	1    3850 3000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4000 3000 4150 3000
Wire Wire Line
	4000 2700 4150 2700
$Comp
L power:GND #PWR?
U 1 1 5C462701
P 3650 3150
F 0 "#PWR?" H 3650 2900 50  0001 C CNN
F 1 "GND" H 3655 2977 50  0000 C CNN
F 2 "" H 3650 3150 50  0001 C CNN
F 3 "" H 3650 3150 50  0001 C CNN
	1    3650 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 3150 3650 3000
Wire Wire Line
	3650 3000 3700 3000
Wire Wire Line
	3700 2700 3650 2700
Wire Wire Line
	3650 2700 3650 3000
Connection ~ 3650 3000
Wire Wire Line
	4550 2550 4400 2550
Wire Wire Line
	4400 2550 4400 2700
Wire Wire Line
	4400 2700 4150 2700
Connection ~ 4150 2700
Wire Wire Line
	4150 3000 4450 3000
Wire Wire Line
	4450 3000 4450 2750
Wire Wire Line
	4450 2750 4550 2750
Connection ~ 4150 3000
Wire Wire Line
	3350 2350 3450 2350
Wire Wire Line
	3450 2350 3450 2550
Wire Wire Line
	3450 2350 4550 2350
Connection ~ 3450 2350
Wire Wire Line
	3050 2350 2750 2350
Wire Wire Line
	2750 2350 2750 2200
$Comp
L Device:C C?
U 1 1 5C4636FC
P 5350 1750
F 0 "C?" H 5250 1850 50  0000 C CNN
F 1 "100n" H 5250 1650 50  0000 C CNN
F 2 "" H 5388 1600 50  0001 C CNN
F 3 "~" H 5350 1750 50  0001 C CNN
	1    5350 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C463759
P 5650 1750
F 0 "C?" H 5550 1850 50  0000 C CNN
F 1 "100n" H 5550 1650 50  0000 C CNN
F 2 "" H 5688 1600 50  0001 C CNN
F 3 "~" H 5650 1750 50  0001 C CNN
	1    5650 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5C463787
P 4400 1750
F 0 "#PWR?" H 4400 1600 50  0001 C CNN
F 1 "+5V" H 4415 1923 50  0000 C CNN
F 2 "" H 4400 1750 50  0001 C CNN
F 3 "" H 4400 1750 50  0001 C CNN
	1    4400 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 1750 4400 1950
Wire Wire Line
	4900 1600 4900 1550
Wire Wire Line
	4900 1550 5350 1550
Wire Wire Line
	5350 1550 5350 1600
Wire Wire Line
	5650 1600 5650 1550
Wire Wire Line
	5650 1550 5350 1550
Connection ~ 5350 1550
Wire Wire Line
	5650 1550 6150 1550
Wire Wire Line
	6150 1550 6150 1900
Connection ~ 5650 1550
$Comp
L Device:CP C?
U 1 1 5C4649B7
P 4900 1750
F 0 "C?" H 5050 1650 50  0000 R CNN
F 1 "22u" H 5050 1850 50  0000 R CNN
F 2 "" H 4938 1600 50  0001 C CNN
F 3 "~" H 4900 1750 50  0001 C CNN
	1    4900 1750
	-1   0    0    1   
$EndComp
Wire Wire Line
	4400 1950 4900 1950
Wire Wire Line
	4900 1950 4900 1900
Wire Wire Line
	5650 1900 5650 1950
Wire Wire Line
	5650 1950 5350 1950
Wire Wire Line
	5350 1950 5350 1900
Wire Wire Line
	5350 1950 5250 1950
Connection ~ 5350 1950
Connection ~ 4900 1950
Wire Wire Line
	5150 2050 5150 1950
Connection ~ 5150 1950
Wire Wire Line
	5150 1950 4900 1950
Wire Wire Line
	5250 2050 5250 1950
Connection ~ 5250 1950
Wire Wire Line
	5250 1950 5150 1950
$Comp
L Connector:Conn_01x04_Male J?
U 1 1 5C46621E
P 6250 3850
F 0 "J?" H 6223 3730 50  0000 R CNN
F 1 "goldpin 1x4" H 6223 3821 50  0000 R CNN
F 2 "" H 6250 3850 50  0001 C CNN
F 3 "~" H 6250 3850 50  0001 C CNN
	1    6250 3850
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4672B4
P 5150 6350
F 0 "#PWR?" H 5150 6100 50  0001 C CNN
F 1 "GND" H 5155 6177 50  0000 C CNN
F 2 "" H 5150 6350 50  0001 C CNN
F 3 "" H 5150 6350 50  0001 C CNN
	1    5150 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 6350 5150 6050
Wire Wire Line
	5750 3950 6050 3950
Wire Wire Line
	5750 3850 6050 3850
Wire Wire Line
	5750 3750 6050 3750
Text Label 3250 1950 2    50   ~ 0
~RESET
Wire Wire Line
	3250 1950 3450 1950
Wire Wire Line
	3450 1950 3450 2350
Text Label 6200 3400 0    50   ~ 0
~RESET
Wire Wire Line
	6200 3400 6000 3400
Wire Wire Line
	6000 3400 6000 3650
Wire Wire Line
	6000 3650 6050 3650
$Comp
L Device:LED D?
U 1 1 5C46AD33
P 6250 2850
F 0 "D?" H 6250 2750 50  0000 C CNN
F 1 "LED" H 6450 2750 50  0000 C CNN
F 2 "" H 6250 2850 50  0001 C CNN
F 3 "~" H 6250 2850 50  0001 C CNN
	1    6250 2850
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D?
U 1 1 5C46AE3D
P 6250 3050
F 0 "D?" H 6150 3200 50  0000 C CNN
F 1 "LED" H 6400 3200 50  0000 C CNN
F 2 "" H 6250 3050 50  0001 C CNN
F 3 "~" H 6250 3050 50  0001 C CNN
	1    6250 3050
	-1   0    0    1   
$EndComp
Wire Wire Line
	6100 3050 5750 3050
Wire Wire Line
	6100 2850 5950 2850
Wire Wire Line
	5950 2850 5950 2950
Wire Wire Line
	5950 2950 5750 2950
$Comp
L Device:R R?
U 1 1 5C46D13B
P 6650 2850
F 0 "R?" V 6750 2950 50  0000 C CNN
F 1 "370" V 6650 2850 50  0000 C CNN
F 2 "" V 6580 2850 50  0001 C CNN
F 3 "~" H 6650 2850 50  0001 C CNN
	1    6650 2850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6400 2850 6500 2850
$Comp
L Device:R R?
U 1 1 5C46DB71
P 6650 3050
F 0 "R?" V 6550 3150 50  0000 C CNN
F 1 "370" V 6650 3050 50  0000 C CNN
F 2 "" V 6580 3050 50  0001 C CNN
F 3 "~" H 6650 3050 50  0001 C CNN
	1    6650 3050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6400 3050 6500 3050
$Comp
L power:GND #PWR?
U 1 1 5C46E858
P 7100 3150
F 0 "#PWR?" H 7100 2900 50  0001 C CNN
F 1 "GND" H 7105 2977 50  0000 C CNN
F 2 "" H 7100 3150 50  0001 C CNN
F 3 "" H 7100 3150 50  0001 C CNN
	1    7100 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 3050 7100 3050
Wire Wire Line
	7100 3050 7100 3150
Wire Wire Line
	6800 2850 7100 2850
Wire Wire Line
	7100 2850 7100 3050
Connection ~ 7100 3050
$Comp
L Device:Q_PNP_BCE Q?
U 1 1 5C46FF67
P 7600 2650
F 0 "Q?" H 7791 2604 50  0000 L CNN
F 1 "Q_PNP_BCE" H 7791 2695 50  0000 L CNN
F 2 "" H 7800 2750 50  0001 C CNN
F 3 "~" H 7600 2650 50  0001 C CNN
	1    7600 2650
	1    0    0    1   
$EndComp
Wire Wire Line
	5750 2850 5850 2850
Wire Wire Line
	5850 2850 5850 2650
$Comp
L power:+5V #PWR?
U 1 1 5C4718CB
P 7700 2200
F 0 "#PWR?" H 7700 2050 50  0001 C CNN
F 1 "+5V" H 7715 2373 50  0000 C CNN
F 2 "" H 7700 2200 50  0001 C CNN
F 3 "" H 7700 2200 50  0001 C CNN
	1    7700 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 2200 7700 2450
$Comp
L Device:D_Schottky D?
U 1 1 5C472630
P 7700 3250
F 0 "D?" V 7746 3171 50  0000 R CNN
F 1 "D_Schottky" V 7655 3171 50  0000 R CNN
F 2 "" H 7700 3250 50  0001 C CNN
F 3 "~" H 7700 3250 50  0001 C CNN
	1    7700 3250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7700 3100 7700 2850
$Comp
L Device:D_Schottky D?
U 1 1 5C473502
P 7700 3700
F 0 "D?" V 7746 3621 50  0000 R CNN
F 1 "D_Schottky" V 7655 3621 50  0000 R CNN
F 2 "" H 7700 3700 50  0001 C CNN
F 3 "~" H 7700 3700 50  0001 C CNN
	1    7700 3700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7700 3550 7700 3400
$Comp
L Device:D D?
U 1 1 5C474305
P 7700 4150
F 0 "D?" V 7654 4229 50  0000 L CNN
F 1 "D" V 7745 4229 50  0000 L CNN
F 2 "" H 7700 4150 50  0001 C CNN
F 3 "~" H 7700 4150 50  0001 C CNN
	1    7700 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 3850 7700 3950
Wire Wire Line
	7700 3950 8000 3950
$Comp
L power:GND #PWR?
U 1 1 5C477E95
P 7700 4400
F 0 "#PWR?" H 7700 4150 50  0001 C CNN
F 1 "GND" H 7705 4227 50  0000 C CNN
F 2 "" H 7700 4400 50  0001 C CNN
F 3 "" H 7700 4400 50  0001 C CNN
	1    7700 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 4400 7700 4350
Wire Wire Line
	7700 4350 8000 4350
$Comp
L Connector:Conn_01x02_Male J?
U 1 1 5C478F49
P 8300 4200
F 0 "J?" H 8406 4378 50  0000 C CNN
F 1 "goldpin 1x2" H 8300 3950 50  0000 C CNN
F 2 "" H 8300 4200 50  0001 C CNN
F 3 "~" H 8300 4200 50  0001 C CNN
	1    8300 4200
	-1   0    0    1   
$EndComp
NoConn ~ 5750 5750
NoConn ~ 5750 5650
NoConn ~ 5750 5550
NoConn ~ 5750 5450
NoConn ~ 5750 5350
NoConn ~ 5750 5250
NoConn ~ 5750 4850
NoConn ~ 5750 4750
NoConn ~ 5750 4550
NoConn ~ 5750 4450
NoConn ~ 5750 4350
NoConn ~ 5750 4250
NoConn ~ 5750 4150
NoConn ~ 5750 3650
NoConn ~ 5750 3550
NoConn ~ 5750 3450
NoConn ~ 5750 3350
NoConn ~ 5750 3250
NoConn ~ 5750 2750
NoConn ~ 5750 2650
NoConn ~ 5750 2550
NoConn ~ 5750 2350
$Comp
L Connector:Conn_01x02_Male J?
U 1 1 5C49A27B
P 6250 5150
F 0 "J?" H 6310 5190 50  0000 L CNN
F 1 "goldpin 1x2" H 6150 5300 50  0000 L CNN
F 2 "" H 6250 5150 50  0001 C CNN
F 3 "~" H 6250 5150 50  0001 C CNN
	1    6250 5150
	-1   0    0    1   
$EndComp
$Comp
L MCU_Microchip_ATmega:ATmega32A-AU U?
U 1 1 5C46225A
P 5150 4050
F 0 "U?" H 4700 2100 50  0000 C CNN
F 1 "ATmega32A-AU" H 5500 2050 50  0000 C CNN
F 2 "Package_QFP:TQFP-44_10x10mm_P0.8mm" H 5150 4050 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-8155-8-bit-microcontroller-avr-atmega32a_datasheet.pdf" H 5150 4050 50  0001 C CNN
	1    5150 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 5050 6050 5050
Wire Wire Line
	5750 5150 6050 5150
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5C4A18D2
P 2200 1950
F 0 "#FLG?" H 2200 2025 50  0001 C CNN
F 1 "PWR_FLAG" H 2200 2124 50  0000 C CNN
F 2 "" H 2200 1950 50  0001 C CNN
F 3 "~" H 2200 1950 50  0001 C CNN
	1    2200 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4A1931
P 2200 2250
F 0 "#PWR?" H 2200 2000 50  0001 C CNN
F 1 "GND" H 2205 2077 50  0000 C CNN
F 2 "" H 2200 2250 50  0001 C CNN
F 3 "" H 2200 2250 50  0001 C CNN
	1    2200 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 2250 2200 1950
Text Notes 6050 4950 0    50   ~ 0
USART
Wire Notes Line
	5850 4800 6500 4800
Wire Notes Line
	6500 4800 6500 5450
Wire Notes Line
	6500 5450 5850 5450
Wire Notes Line
	5850 5450 5850 4800
$Comp
L Switch:SW_DPST_x2 SW?
U 1 1 5C49ECC9
P 6450 2350
F 0 "SW?" H 6500 2500 50  0000 R CNN
F 1 "ACTION SW" H 6600 2250 50  0000 R CNN
F 2 "" H 6450 2350 50  0001 C CNN
F 3 "" H 6450 2350 50  0001 C CNN
	1    6450 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 2650 7400 2650
Text Notes 8300 3600 0    50   ~ 0
Silnik\nwibracyjny
Wire Wire Line
	7700 4000 7700 3950
Connection ~ 7700 3950
Wire Wire Line
	7700 4300 7700 4350
Connection ~ 7700 4350
Wire Wire Line
	8000 4200 8000 4350
Wire Wire Line
	8000 3950 8000 4100
Wire Wire Line
	8000 4100 8100 4100
Wire Wire Line
	8000 4200 8100 4200
$Comp
L Motor:Motor_DC M?
U 1 1 5CD5C955
P 9100 4050
F 0 "M?" H 9258 4046 50  0000 L CNN
F 1 "Motor_DC" H 9258 3955 50  0000 L CNN
F 2 "" H 9100 3960 50  0001 C CNN
F 3 "~" H 9100 3960 50  0001 C CNN
	1    9100 4050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 5CD5CD44
P 8500 4200
F 0 "J?" H 8394 3875 50  0000 C CNN
F 1 "goldpin 1x2" H 8400 4300 50  0000 C CNN
F 2 "" H 8500 4200 50  0001 C CNN
F 3 "~" H 8500 4200 50  0001 C CNN
	1    8500 4200
	-1   0    0    1   
$EndComp
Wire Wire Line
	9100 4350 9100 4400
Wire Wire Line
	9100 4400 8850 4400
Wire Wire Line
	8850 4400 8850 4200
Wire Wire Line
	8850 4200 8700 4200
Wire Wire Line
	8700 4100 8850 4100
Wire Wire Line
	8850 4100 8850 3800
Wire Wire Line
	8850 3800 9100 3800
Wire Wire Line
	9100 3800 9100 3850
Wire Notes Line
	7350 1950 7350 4800
Wire Notes Line
	7350 4800 9750 4800
Wire Notes Line
	9750 4800 9750 1950
Wire Notes Line
	9750 1950 7350 1950
Wire Notes Line
	5850 3250 6950 3250
Wire Notes Line
	6950 3250 6950 4150
Wire Notes Line
	6950 4150 5850 4150
Wire Notes Line
	5850 4150 5850 3250
Text Notes 6550 4000 0    50   ~ 0
USBasp
$Comp
L power:GND #PWR?
U 1 1 5CD6656A
P 3450 3150
F 0 "#PWR?" H 3450 2900 50  0001 C CNN
F 1 "GND" H 3455 2977 50  0000 C CNN
F 2 "" H 3450 3150 50  0001 C CNN
F 3 "" H 3450 3150 50  0001 C CNN
	1    3450 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 2950 3450 3150
$Comp
L power:GND #PWR?
U 1 1 5CD6833F
P 6800 2400
F 0 "#PWR?" H 6800 2150 50  0001 C CNN
F 1 "GND" H 6805 2227 50  0000 C CNN
F 2 "" H 6800 2400 50  0001 C CNN
F 3 "" H 6800 2400 50  0001 C CNN
	1    6800 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 2350 6800 2350
Wire Wire Line
	6800 2350 6800 2400
Wire Wire Line
	6250 2350 5900 2350
Wire Wire Line
	5900 2350 5900 2450
Wire Wire Line
	5900 2450 5750 2450
NoConn ~ 5750 4650
$Comp
L Device:Battery_Cell BT?
U 1 1 5CD7005F
P 1750 4300
F 0 "BT?" H 1868 4396 50  0000 L CNN
F 1 "9V" H 1868 4305 50  0000 L CNN
F 2 "" V 1750 4360 50  0001 C CNN
F 3 "~" V 1750 4360 50  0001 C CNN
	1    1750 4300
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LD1117S50TR_SOT223 U?
U 1 1 5CD70646
P 2700 3950
F 0 "U?" H 2700 4192 50  0000 C CNN
F 1 "LD1117S50TR_SOT223" H 2700 4101 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 2700 4150 50  0001 C CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00000544.pdf" H 2800 3700 50  0001 C CNN
	1    2700 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 4100 1750 3950
Wire Wire Line
	1750 3950 2400 3950
$Comp
L power:GND #PWR?
U 1 1 5CD7263D
P 1750 4600
F 0 "#PWR?" H 1750 4350 50  0001 C CNN
F 1 "GND" H 1755 4427 50  0000 C CNN
F 2 "" H 1750 4600 50  0001 C CNN
F 3 "" H 1750 4600 50  0001 C CNN
	1    1750 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 4600 1750 4450
Wire Wire Line
	2700 4250 2700 4450
Wire Wire Line
	2700 4450 1750 4450
Connection ~ 1750 4450
Wire Wire Line
	1750 4450 1750 4400
$Comp
L power:+5VP #PWR?
U 1 1 5CD76483
P 3250 3850
F 0 "#PWR?" H 3250 3700 50  0001 C CNN
F 1 "+5VP" H 3265 4023 50  0000 C CNN
F 2 "" H 3250 3850 50  0001 C CNN
F 3 "" H 3250 3850 50  0001 C CNN
	1    3250 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 3950 3250 3950
Wire Wire Line
	3250 3950 3250 3850
$EndSCHEMATC
