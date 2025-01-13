EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr B 17000 11000
encoding utf-8
Sheet 2 2
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
L Connector:RJ45_Amphenol_RJMG1BD3B8K1ANR J?
U 1 1 60F5BCCD
P 11650 4925
F 0 "J?" H 11650 5647 50  0000 C CNN
F 1 "RJ45_Amphenol_RJMG1BD3B8K1ANR" H 11650 5557 50  0000 C CNN
F 2 "Connector_RJ:RJ45_Amphenol_RJMG1BD3B8K1ANR" H 11650 5625 50  0001 C CNN
F 3 "https://www.amphenolcanada.com/ProductSearch/Drawings/AC/RJMG1BD3B8K1ANR.PDF" H 11650 5725 50  0001 C CNN
	1    11650 4925
	1    0    0    -1  
$EndComp
$Comp
L davenport-kicad:LAN8742A U?
U 1 1 60F5BCD3
P 6400 4975
F 0 "U?" H 5625 5900 60  0000 C CNN
F 1 "LAN8742A" H 5775 4050 60  0000 C CNN
F 2 "" H 6400 4975 60  0000 C CNN
F 3 "" H 6400 4975 60  0000 C CNN
	1    6400 4975
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 60F5BCD9
P 6150 3300
F 0 "#PWR?" H 6150 3150 50  0001 C CNN
F 1 "+3.3V" H 6164 3473 50  0000 C CNN
F 2 "" H 6150 3300 50  0001 C CNN
F 3 "" H 6150 3300 50  0001 C CNN
	1    6150 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 3300 6150 3525
$Comp
L Device:Ferrite_Bead_Small FB?
U 1 1 60F5BCE0
P 6350 3525
F 0 "FB?" V 6115 3525 50  0000 C CNN
F 1 "FB" V 6205 3525 50  0000 C CNN
F 2 "" V 6280 3525 50  0001 C CNN
F 3 "~" H 6350 3525 50  0001 C CNN
	1    6350 3525
	0    1    1    0   
$EndComp
Wire Wire Line
	6150 3525 6250 3525
Connection ~ 6150 3525
Wire Wire Line
	6600 3525 6600 3825
Wire Wire Line
	6450 3525 6600 3525
Wire Wire Line
	6700 3825 6700 3525
Wire Wire Line
	6700 3525 6600 3525
Connection ~ 6600 3525
Wire Wire Line
	9150 4525 9150 4625
Wire Wire Line
	8900 4625 8900 4725
Wire Wire Line
	8900 4725 8975 4725
Wire Wire Line
	8600 4725 8600 4925
Wire Wire Line
	8600 4925 9250 4925
$Comp
L Device:R R?
U 1 1 60F5BCF2
P 8425 3900
F 0 "R?" H 8495 3945 50  0000 L CNN
F 1 "49.9" V 8425 3900 50  0000 C CNN
F 2 "" V 8355 3900 50  0001 C CNN
F 3 "~" H 8425 3900 50  0001 C CNN
	1    8425 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8425 3525 8425 3750
Wire Wire Line
	8425 4050 8425 4425
$Comp
L Device:R R?
U 1 1 60F5BCFA
P 8975 3900
F 0 "R?" H 9045 3945 50  0000 L CNN
F 1 "49.9" V 8975 3900 50  0000 C CNN
F 2 "" V 8905 3900 50  0001 C CNN
F 3 "~" H 8975 3900 50  0001 C CNN
	1    8975 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8975 3525 8975 3750
$Comp
L Device:R R?
U 1 1 60F5BD01
P 8725 3900
F 0 "R?" H 8795 3945 50  0000 L CNN
F 1 "49.9" V 8725 3900 50  0000 C CNN
F 2 "" V 8655 3900 50  0001 C CNN
F 3 "~" H 8725 3900 50  0001 C CNN
	1    8725 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8725 3525 8725 3750
$Comp
L Device:R R?
U 1 1 60F5BD08
P 9250 3900
F 0 "R?" H 9320 3945 50  0000 L CNN
F 1 "49.9" V 9250 3900 50  0000 C CNN
F 2 "" V 9180 3900 50  0001 C CNN
F 3 "~" H 9250 3900 50  0001 C CNN
	1    9250 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 3525 9250 3750
Connection ~ 8425 3525
Wire Wire Line
	8725 3525 8975 3525
Connection ~ 8975 3525
Wire Wire Line
	8975 3525 9250 3525
Wire Wire Line
	8725 4050 8725 4525
Connection ~ 8725 4525
Wire Wire Line
	8725 4525 9150 4525
Wire Wire Line
	8975 4050 8975 4725
Connection ~ 8975 4725
Wire Wire Line
	9250 4050 9250 4925
Connection ~ 9250 4925
Wire Wire Line
	9250 3525 9500 3525
Connection ~ 9250 3525
Wire Wire Line
	9500 4525 9500 4825
$Comp
L Device:C C?
U 1 1 60F5BD1D
P 9500 5375
F 0 "C?" H 9550 5475 50  0000 L CNN
F 1 "0.022uF" H 9550 5275 50  0000 L CNN
F 2 "" H 9538 5225 50  0001 C CNN
F 3 "~" H 9500 5375 50  0001 C CNN
	1    9500 5375
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 4525 10750 4525
Wire Wire Line
	9150 4625 10750 4625
Wire Wire Line
	8975 4725 10750 4725
Wire Wire Line
	9500 4825 10750 4825
Wire Wire Line
	9250 4925 10750 4925
Wire Wire Line
	8425 4425 8425 5225
Wire Wire Line
	8975 4725 8975 5225
$Comp
L Device:C C?
U 1 1 60F5BD2A
P 8425 5375
F 0 "C?" H 8475 5475 50  0000 L CNN
F 1 "10pF" H 8475 5275 50  0000 L CNN
F 2 "" H 8463 5225 50  0001 C CNN
F 3 "~" H 8425 5375 50  0001 C CNN
	1    8425 5375
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60F5BD30
P 8975 5375
F 0 "C?" H 9000 5475 50  0000 L CNN
F 1 "10pF" H 9025 5275 50  0000 L CNN
F 2 "" H 9013 5225 50  0001 C CNN
F 3 "~" H 8975 5375 50  0001 C CNN
	1    8975 5375
	1    0    0    -1  
$EndComp
Wire Wire Line
	8725 4525 8725 5550
Wire Wire Line
	9250 4925 9250 5575
$Comp
L Device:C C?
U 1 1 60F5BD38
P 8725 5700
F 0 "C?" H 8750 5800 50  0000 L CNN
F 1 "10pF" H 8775 5600 50  0000 L CNN
F 2 "" H 8763 5550 50  0001 C CNN
F 3 "~" H 8725 5700 50  0001 C CNN
	1    8725 5700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60F5BD3E
P 9250 5725
F 0 "C?" H 9275 5825 50  0000 L CNN
F 1 "10pF" H 9300 5625 50  0000 L CNN
F 2 "" H 9288 5575 50  0001 C CNN
F 3 "~" H 9250 5725 50  0001 C CNN
	1    9250 5725
	1    0    0    -1  
$EndComp
Connection ~ 8425 4425
Wire Wire Line
	8425 4425 10750 4425
Wire Wire Line
	7650 4425 8425 4425
Wire Wire Line
	7650 4525 8725 4525
Wire Wire Line
	7650 4625 8900 4625
Wire Wire Line
	7650 4725 8600 4725
Text Label 7675 4425 0    50   ~ 0
TX_P
Text Label 7675 4525 0    50   ~ 0
TX_N
Text Label 7675 4625 0    50   ~ 0
RX_P
Text Label 7675 4725 0    50   ~ 0
RN_N
Wire Wire Line
	6700 3525 6900 3525
Connection ~ 6700 3525
Wire Wire Line
	6150 3525 6150 3825
Wire Wire Line
	6150 3525 5800 3525
$Comp
L Device:C C?
U 1 1 60F5BD52
P 6900 3675
F 0 "C?" H 6950 3775 50  0000 L CNN
F 1 "100nF" H 6950 3575 50  0000 L CNN
F 2 "" H 6938 3525 50  0001 C CNN
F 3 "~" H 6900 3675 50  0001 C CNN
	1    6900 3675
	1    0    0    -1  
$EndComp
Connection ~ 6900 3525
Wire Wire Line
	6900 3525 7225 3525
$Comp
L Device:C C?
U 1 1 60F5BD5A
P 7225 3675
F 0 "C?" H 7275 3775 50  0000 L CNN
F 1 "100nF" H 7275 3575 50  0000 L CNN
F 2 "" H 7263 3525 50  0001 C CNN
F 3 "~" H 7225 3675 50  0001 C CNN
	1    7225 3675
	1    0    0    -1  
$EndComp
Connection ~ 7225 3525
Wire Wire Line
	7225 3525 7550 3525
$Comp
L Device:C C?
U 1 1 60F5BD62
P 7550 3675
F 0 "C?" H 7600 3775 50  0000 L CNN
F 1 "4.7uF" H 7600 3575 50  0000 L CNN
F 2 "" H 7588 3525 50  0001 C CNN
F 3 "~" H 7550 3675 50  0001 C CNN
	1    7550 3675
	1    0    0    -1  
$EndComp
Connection ~ 7550 3525
Wire Wire Line
	7550 3525 8425 3525
$Comp
L Device:C C?
U 1 1 60F5BD6A
P 5800 3675
F 0 "C?" H 5850 3775 50  0000 L CNN
F 1 "100nF" H 5850 3575 50  0000 L CNN
F 2 "" H 5838 3525 50  0001 C CNN
F 3 "~" H 5800 3675 50  0001 C CNN
	1    5800 3675
	1    0    0    -1  
$EndComp
Wire Wire Line
	8425 3525 8725 3525
Connection ~ 8725 3525
Wire Wire Line
	8425 5525 8425 5975
Wire Wire Line
	8425 5975 8725 5975
Wire Wire Line
	9250 5875 9250 5975
Connection ~ 9250 5975
Wire Wire Line
	9250 5975 9500 5975
Wire Wire Line
	8975 5525 8975 5975
Connection ~ 8975 5975
Wire Wire Line
	8975 5975 9250 5975
Wire Wire Line
	8725 5850 8725 5975
Connection ~ 8725 5975
Wire Wire Line
	8725 5975 8975 5975
Wire Wire Line
	8975 5975 8975 6050
$Comp
L power:GND #PWR?
U 1 1 60F5BD7E
P 8975 6050
F 0 "#PWR?" H 8975 5800 50  0001 C CNN
F 1 "GND" H 8975 5900 50  0000 C CNN
F 2 "" H 8975 6050 50  0001 C CNN
F 3 "" H 8975 6050 50  0001 C CNN
	1    8975 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 3900 7550 3975
$Comp
L power:GND #PWR?
U 1 1 60F5BD85
P 7550 3975
F 0 "#PWR?" H 7550 3725 50  0001 C CNN
F 1 "GND" H 7550 3825 50  0000 C CNN
F 2 "" H 7550 3975 50  0001 C CNN
F 3 "" H 7550 3975 50  0001 C CNN
	1    7550 3975
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 3900 7225 3900
Wire Wire Line
	6900 3900 6900 3825
Wire Wire Line
	7225 3825 7225 3900
Connection ~ 7225 3900
Wire Wire Line
	7225 3900 6900 3900
Wire Wire Line
	7550 3825 7550 3900
Connection ~ 7550 3900
Wire Wire Line
	5800 3825 5800 3900
$Comp
L power:GND #PWR?
U 1 1 60F5BD93
P 5800 3900
F 0 "#PWR?" H 5800 3650 50  0001 C CNN
F 1 "GND" H 5800 3750 50  0000 C CNN
F 2 "" H 5800 3900 50  0001 C CNN
F 3 "" H 5800 3900 50  0001 C CNN
	1    5800 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 4825 9500 5225
Connection ~ 9500 4825
Wire Wire Line
	9500 5525 9500 5975
$Comp
L Device:R R?
U 1 1 60F5BD9C
P 7750 5900
F 0 "R?" H 7820 5945 50  0000 L CNN
F 1 "12k1" V 7750 5800 50  0000 L CNN
F 2 "" V 7680 5900 50  0001 C CNN
F 3 "~" H 7750 5900 50  0001 C CNN
	1    7750 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 5675 7750 5675
Wire Wire Line
	7750 5675 7750 5750
Wire Wire Line
	7750 6050 7750 6125
$Comp
L power:GND #PWR?
U 1 1 60F5BDA5
P 7750 6125
F 0 "#PWR?" H 7750 5875 50  0001 C CNN
F 1 "GND" H 7750 5975 50  0000 C CNN
F 2 "" H 7750 6125 50  0001 C CNN
F 3 "" H 7750 6125 50  0001 C CNN
	1    7750 6125
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 4425 5125 4425
Wire Wire Line
	4800 4325 5250 4325
Wire Wire Line
	5125 4425 5125 4100
Connection ~ 5125 4425
Wire Wire Line
	5125 4425 4800 4425
$Comp
L Device:R R?
U 1 1 60F5BDB0
P 5125 3950
F 0 "R?" H 5195 3995 50  0000 L CNN
F 1 "1k5" V 5125 3875 50  0000 L CNN
F 2 "" V 5055 3950 50  0001 C CNN
F 3 "~" H 5125 3950 50  0001 C CNN
	1    5125 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5125 3800 5125 3525
Wire Wire Line
	5125 3525 5800 3525
Connection ~ 5800 3525
Text GLabel 4800 4425 0    50   Input ~ 0
MDIO
Text GLabel 4800 4325 0    50   Input ~ 0
MDIC
Wire Wire Line
	9500 3525 9500 4525
Connection ~ 9500 4525
Wire Wire Line
	4800 4625 5250 4625
Text GLabel 4350 4625 0    50   Input ~ 0
TXD0
Wire Wire Line
	4800 4725 5250 4725
Text GLabel 4350 4725 0    50   Input ~ 0
TXD1
Wire Wire Line
	4800 4825 5250 4825
Text GLabel 4350 4825 0    50   Input ~ 0
TXEN
Text GLabel 4350 4925 0    50   Input ~ 0
CRS_DV
Wire Wire Line
	4800 5025 5250 5025
Text GLabel 4350 5025 0    50   Input ~ 0
RXD0
Wire Wire Line
	4800 5125 5250 5125
Text GLabel 4350 5125 0    50   Input ~ 0
RXD1
Wire Wire Line
	4800 5325 5250 5325
Text GLabel 4350 5325 0    50   Input ~ 0
REFCLK
$Comp
L Device:Crystal Y?
U 1 1 60F5BDCB
P 5000 6050
F 0 "Y?" H 5000 6316 50  0000 C CNN
F 1 "25 MHz" H 5000 6226 50  0000 C CNN
F 2 "" H 5000 6050 50  0001 C CNN
F 3 "~" H 5000 6050 50  0001 C CNN
	1    5000 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 5675 5200 5675
Wire Wire Line
	5200 5675 5200 6050
Wire Wire Line
	5200 6050 5150 6050
Wire Wire Line
	4850 6050 4775 6050
Wire Wire Line
	4775 6050 4775 5575
Wire Wire Line
	4775 5575 5250 5575
Wire Wire Line
	4775 6050 4775 6275
Connection ~ 4775 6050
Wire Wire Line
	5200 6050 5200 6275
Connection ~ 5200 6050
$Comp
L Device:C C?
U 1 1 60F5BDDB
P 4775 6425
F 0 "C?" H 4825 6525 50  0000 L CNN
F 1 "C" H 4825 6325 50  0000 L CNN
F 2 "" H 4813 6275 50  0001 C CNN
F 3 "~" H 4775 6425 50  0001 C CNN
	1    4775 6425
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60F5BDE1
P 5200 6425
F 0 "C?" H 5250 6525 50  0000 L CNN
F 1 "C" H 5250 6325 50  0000 L CNN
F 2 "" H 5238 6275 50  0001 C CNN
F 3 "~" H 5200 6425 50  0001 C CNN
	1    5200 6425
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60F7B1A9
P 4775 6650
F 0 "#PWR?" H 4775 6400 50  0001 C CNN
F 1 "GND" H 4779 6478 50  0000 C CNN
F 2 "" H 4775 6650 50  0001 C CNN
F 3 "" H 4775 6650 50  0001 C CNN
	1    4775 6650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60F7B5DA
P 5200 6650
F 0 "#PWR?" H 5200 6400 50  0001 C CNN
F 1 "GND" H 5204 6478 50  0000 C CNN
F 2 "" H 5200 6650 50  0001 C CNN
F 3 "" H 5200 6650 50  0001 C CNN
	1    5200 6650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60F7C388
P 6650 6425
F 0 "C?" H 6700 6525 50  0000 L CNN
F 1 "1uF" H 6700 6325 50  0000 L CNN
F 2 "" H 6688 6275 50  0001 C CNN
F 3 "~" H 6650 6425 50  0001 C CNN
	1    6650 6425
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60F7C97B
P 6925 6425
F 0 "C?" H 6975 6525 50  0000 L CNN
F 1 "470pF" H 6975 6325 50  0000 L CNN
F 2 "" H 6963 6275 50  0001 C CNN
F 3 "~" H 6925 6425 50  0001 C CNN
	1    6925 6425
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 6575 5200 6650
Wire Wire Line
	4775 6575 4775 6650
Wire Wire Line
	6650 6275 6650 6175
Wire Wire Line
	6650 6175 6925 6175
Wire Wire Line
	6925 6175 6925 6275
Wire Wire Line
	6650 6575 6650 6650
Wire Wire Line
	6650 6650 6800 6650
Wire Wire Line
	6925 6650 6925 6575
Wire Wire Line
	6650 6125 6650 6175
Connection ~ 6650 6175
Wire Wire Line
	6800 6650 6800 6775
Connection ~ 6800 6650
Wire Wire Line
	6800 6650 6925 6650
$Comp
L power:GND #PWR?
U 1 1 60F91821
P 6800 6775
F 0 "#PWR?" H 6800 6525 50  0001 C CNN
F 1 "GND" H 6800 6625 50  0000 C CNN
F 2 "" H 6800 6775 50  0001 C CNN
F 3 "" H 6800 6775 50  0001 C CNN
	1    6800 6775
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 6125 6450 6650
Wire Wire Line
	6450 6650 6650 6650
Connection ~ 6650 6650
$Comp
L Device:R R?
U 1 1 60F9543B
P 6250 6375
F 0 "R?" H 6320 6420 50  0000 L CNN
F 1 "10k" V 6250 6300 50  0000 L CNN
F 2 "" V 6180 6375 50  0001 C CNN
F 3 "~" H 6250 6375 50  0001 C CNN
	1    6250 6375
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 6225 6250 6125
Wire Wire Line
	6250 6525 6250 6600
Wire Wire Line
	6250 6600 6025 6600
Wire Wire Line
	6025 6600 6025 6425
$Comp
L power:+3.3V #PWR?
U 1 1 60F9CBDF
P 6025 6425
F 0 "#PWR?" H 6025 6275 50  0001 C CNN
F 1 "+3.3V" H 6025 6575 50  0000 C CNN
F 2 "" H 6025 6425 50  0001 C CNN
F 3 "" H 6025 6425 50  0001 C CNN
	1    6025 6425
	1    0    0    -1  
$EndComp
NoConn ~ 5250 5225
Wire Wire Line
	7650 5125 8025 5125
Wire Wire Line
	8025 5125 8025 5400
Wire Wire Line
	7650 5225 7950 5225
Text Label 7950 5225 2    50   ~ 0
LED1
Text Label 7950 5125 2    50   ~ 0
LED2
$Comp
L Device:R R?
U 1 1 60FA8DA7
P 8025 5550
F 0 "R?" H 8095 5595 50  0000 L CNN
F 1 "10k" V 8025 5475 50  0000 L CNN
F 2 "" V 7955 5550 50  0001 C CNN
F 3 "~" H 8025 5550 50  0001 C CNN
	1    8025 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8025 5700 8025 5825
$Comp
L power:GND #PWR?
U 1 1 60FA99B6
P 8025 5825
F 0 "#PWR?" H 8025 5575 50  0001 C CNN
F 1 "GND" H 8025 5675 50  0000 C CNN
F 2 "" H 8025 5825 50  0001 C CNN
F 3 "" H 8025 5825 50  0001 C CNN
	1    8025 5825
	1    0    0    -1  
$EndComp
Wire Wire Line
	10750 5125 10450 5125
Text Label 10450 5125 0    50   ~ 0
LED1
Wire Wire Line
	10750 5325 10450 5325
Text Label 10450 5325 0    50   ~ 0
LED2
Wire Wire Line
	10750 5225 10300 5225
Wire Wire Line
	10300 5225 10300 5475
Wire Wire Line
	10750 5425 10650 5425
Wire Wire Line
	10650 5425 10650 5475
$Comp
L Device:R R?
U 1 1 60FBD06B
P 10300 5625
F 0 "R?" H 10370 5670 50  0000 L CNN
F 1 "270" V 10300 5550 50  0000 L CNN
F 2 "" V 10230 5625 50  0001 C CNN
F 3 "~" H 10300 5625 50  0001 C CNN
	1    10300 5625
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 5775 10300 5900
$Comp
L power:GND #PWR?
U 1 1 60FBD072
P 10300 5900
F 0 "#PWR?" H 10300 5650 50  0001 C CNN
F 1 "GND" H 10300 5750 50  0000 C CNN
F 2 "" H 10300 5900 50  0001 C CNN
F 3 "" H 10300 5900 50  0001 C CNN
	1    10300 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60FC12B7
P 10650 5625
F 0 "R?" H 10720 5670 50  0000 L CNN
F 1 "270" V 10650 5550 50  0000 L CNN
F 2 "" V 10580 5625 50  0001 C CNN
F 3 "~" H 10650 5625 50  0001 C CNN
	1    10650 5625
	1    0    0    -1  
$EndComp
Wire Wire Line
	10650 5775 10650 5900
$Comp
L power:GND #PWR?
U 1 1 60FC12BE
P 10650 5900
F 0 "#PWR?" H 10650 5650 50  0001 C CNN
F 1 "GND" H 10650 5750 50  0000 C CNN
F 2 "" H 10650 5900 50  0001 C CNN
F 3 "" H 10650 5900 50  0001 C CNN
	1    10650 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	11650 5625 11650 5700
$Comp
L power:GND #PWR?
U 1 1 60FC947E
P 11650 5800
F 0 "#PWR?" H 11650 5550 50  0001 C CNN
F 1 "GND" H 11650 5650 50  0000 C CNN
F 2 "" H 11650 5800 50  0001 C CNN
F 3 "" H 11650 5800 50  0001 C CNN
	1    11650 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	12550 5325 12650 5325
Wire Wire Line
	12650 5325 12650 5700
Wire Wire Line
	12650 5700 11650 5700
Connection ~ 11650 5700
Wire Wire Line
	11650 5700 11650 5800
$Comp
L Device:R R?
U 1 1 60FD30B0
P 4650 4625
F 0 "R?" V 4600 4375 50  0000 L CNN
F 1 "33" V 4650 4550 50  0000 L CNN
F 2 "" V 4580 4625 50  0001 C CNN
F 3 "~" H 4650 4625 50  0001 C CNN
	1    4650 4625
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 60FD4359
P 4650 4725
F 0 "R?" V 4600 4475 50  0000 L CNN
F 1 "33" V 4650 4650 50  0000 L CNN
F 2 "" V 4580 4725 50  0001 C CNN
F 3 "~" H 4650 4725 50  0001 C CNN
	1    4650 4725
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 60FD45F5
P 4650 4825
F 0 "R?" V 4600 4575 50  0000 L CNN
F 1 "33" V 4650 4750 50  0000 L CNN
F 2 "" V 4580 4825 50  0001 C CNN
F 3 "~" H 4650 4825 50  0001 C CNN
	1    4650 4825
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 60FD4F06
P 4650 5325
F 0 "R?" V 4600 5075 50  0000 L CNN
F 1 "33" V 4650 5250 50  0000 L CNN
F 2 "" V 4580 5325 50  0001 C CNN
F 3 "~" H 4650 5325 50  0001 C CNN
	1    4650 5325
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 60FD5347
P 4650 5125
F 0 "R?" V 4600 4875 50  0000 L CNN
F 1 "33" V 4650 5050 50  0000 L CNN
F 2 "" V 4580 5125 50  0001 C CNN
F 3 "~" H 4650 5125 50  0001 C CNN
	1    4650 5125
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 60FD57CE
P 4650 5025
F 0 "R?" V 4600 4775 50  0000 L CNN
F 1 "33" V 4650 4950 50  0000 L CNN
F 2 "" V 4580 5025 50  0001 C CNN
F 3 "~" H 4650 5025 50  0001 C CNN
	1    4650 5025
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 4925 5250 4925
Wire Wire Line
	4500 4825 4350 4825
Wire Wire Line
	4500 4725 4350 4725
Wire Wire Line
	4500 4625 4350 4625
Wire Wire Line
	4500 5025 4350 5025
Wire Wire Line
	4500 5125 4350 5125
Wire Wire Line
	4500 5325 4350 5325
$Comp
L Device:C C?
U 1 1 61004C6D
P 3675 2900
F 0 "C?" H 3725 3000 50  0000 L CNN
F 1 "100nf" H 3725 2800 50  0000 L CNN
F 2 "" H 3713 2750 50  0001 C CNN
F 3 "~" H 3675 2900 50  0001 C CNN
	1    3675 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3675 2750 4000 2750
$Comp
L Device:C C?
U 1 1 61004C75
P 4000 2900
F 0 "C?" H 4050 3000 50  0000 L CNN
F 1 "10uF" H 4050 2800 50  0000 L CNN
F 2 "" H 4038 2750 50  0001 C CNN
F 3 "~" H 4000 2900 50  0001 C CNN
	1    4000 2900
	1    0    0    -1  
$EndComp
Connection ~ 4000 2750
Wire Wire Line
	4000 2750 4325 2750
$Comp
L Device:C C?
U 1 1 61004C7D
P 4325 2900
F 0 "C?" H 4375 3000 50  0000 L CNN
F 1 "10uF" H 4375 2800 50  0000 L CNN
F 2 "" H 4363 2750 50  0001 C CNN
F 3 "~" H 4325 2900 50  0001 C CNN
	1    4325 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 3125 4000 3200
$Comp
L power:GND #PWR?
U 1 1 61004C86
P 4000 3200
F 0 "#PWR?" H 4000 2950 50  0001 C CNN
F 1 "GND" H 4000 3050 50  0000 C CNN
F 2 "" H 4000 3200 50  0001 C CNN
F 3 "" H 4000 3200 50  0001 C CNN
	1    4000 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4325 3125 4000 3125
Wire Wire Line
	3675 3125 3675 3050
Wire Wire Line
	4000 3050 4000 3125
Connection ~ 4000 3125
Wire Wire Line
	4000 3125 3675 3125
Wire Wire Line
	4325 3050 4325 3125
Wire Wire Line
	4000 2750 4000 2575
$Comp
L power:+3.3V #PWR?
U 1 1 6101A067
P 4000 2575
F 0 "#PWR?" H 4000 2425 50  0001 C CNN
F 1 "+3.3V" H 4000 2725 50  0000 C CNN
F 2 "" H 4000 2575 50  0001 C CNN
F 3 "" H 4000 2575 50  0001 C CNN
	1    4000 2575
	1    0    0    -1  
$EndComp
$EndSCHEMATC
