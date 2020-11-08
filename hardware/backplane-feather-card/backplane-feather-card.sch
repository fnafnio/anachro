EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 7900 850  850  900 
U 5F8938DC
F0 "go-jumpers" 50
F1 "go-jumpers.sch" 50
F2 "CARD1-GO" I R 8750 950 50 
F3 "CARD2-GO" I R 8750 1050 50 
F4 "CARD3-GO" I R 8750 1150 50 
F5 "CARD4-GO" I R 8750 1250 50 
F6 "CARD5-GO" I R 8750 1350 50 
F7 "CARD6-GO" I R 8750 1450 50 
F8 "CARD7-GO" I R 8750 1550 50 
F9 "CARD8-GO" I R 8750 1650 50 
F10 "+3v3" I L 7900 950 50 
F11 "GO" O L 7900 1650 50 
$EndSheet
Wire Wire Line
	8750 950  9750 950 
Wire Wire Line
	8750 1050 9750 1050
Wire Wire Line
	8750 1150 9750 1150
Wire Wire Line
	8750 1250 9750 1250
Wire Wire Line
	8750 1350 9750 1350
Wire Wire Line
	8750 1450 9750 1450
Wire Wire Line
	8750 1550 9750 1550
Wire Wire Line
	8750 1650 9750 1650
$Comp
L power:+3V3 #PWR0101
U 1 1 5F8A60B7
P 7750 950
F 0 "#PWR0101" H 7750 800 50  0001 C CNN
F 1 "+3V3" H 7765 1123 50  0000 C CNN
F 2 "" H 7750 950 50  0001 C CNN
F 3 "" H 7750 950 50  0001 C CNN
	1    7750 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 950  7900 950 
Text Label 7750 1650 2    50   ~ 0
GO
Wire Wire Line
	7750 1650 7900 1650
Text Label 9750 3100 2    50   ~ 0
+3v3-backplane
Text Label 6150 1750 0    50   ~ 0
+3v3-backplane
$Comp
L power:+3V3 #PWR0102
U 1 1 5F8A78B6
P 6250 1650
F 0 "#PWR0102" H 6250 1500 50  0001 C CNN
F 1 "+3V3" H 6265 1823 50  0000 C CNN
F 2 "" H 6250 1650 50  0001 C CNN
F 3 "" H 6250 1650 50  0001 C CNN
	1    6250 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 1650 6250 1650
$Comp
L power:GND #PWR0103
U 1 1 5F8A8270
P 9000 2900
F 0 "#PWR0103" H 9000 2650 50  0001 C CNN
F 1 "GND" H 9005 2727 50  0000 C CNN
F 2 "" H 9000 2900 50  0001 C CNN
F 3 "" H 9000 2900 50  0001 C CNN
	1    9000 2900
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0104
U 1 1 5F8A883A
P 9200 2750
F 0 "#PWR0104" H 9200 2600 50  0001 C CNN
F 1 "+5V" H 9215 2923 50  0000 C CNN
F 2 "" H 9200 2750 50  0001 C CNN
F 3 "" H 9200 2750 50  0001 C CNN
	1    9200 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 2900 9000 2900
Wire Wire Line
	9750 3000 9200 3000
Wire Wire Line
	9200 3000 9200 2750
Text Label 9750 3200 2    50   ~ 0
Reset
Wire Wire Line
	8300 3400 9750 3400
Wire Wire Line
	8300 3500 9750 3500
Wire Wire Line
	8300 3650 9750 3650
Wire Wire Line
	8300 3750 9750 3750
Wire Wire Line
	8300 3900 9750 3900
Wire Wire Line
	8300 4000 9750 4000
Wire Wire Line
	8300 4150 9750 4150
Wire Wire Line
	8300 4250 9750 4250
Wire Wire Line
	8300 4400 9750 4400
Wire Wire Line
	8300 4500 9750 4500
Wire Wire Line
	8300 4650 9750 4650
Wire Wire Line
	8300 4750 9750 4750
NoConn ~ 9750 4950
NoConn ~ 9750 5050
NoConn ~ 9750 5200
NoConn ~ 9750 5300
NoConn ~ 9750 5750
NoConn ~ 9750 5850
NoConn ~ 9750 2350
NoConn ~ 9750 2450
NoConn ~ 9750 2550
NoConn ~ 9750 2650
Text Label 9750 1850 2    50   ~ 0
COPI
Text Label 9750 1950 2    50   ~ 0
SCK
Text Label 9750 2050 2    50   ~ 0
CSn
Text Label 9750 2150 2    50   ~ 0
CIPO
$Comp
L stargazer-backplane:SN74LVC125AD U5
U 1 1 5F88ACF8
P 2300 5550
F 0 "U5" H 2300 5650 50  0000 C CNN
F 1 "SN74LVC125AD" H 2300 4750 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 2300 5673 50  0001 C CNN
F 3 "" H 2300 5650 50  0001 C CNN
	1    2300 5550
	1    0    0    -1  
$EndComp
Text Label 2700 6200 0    50   ~ 0
COPI-BUFFER
Text Label 1900 6100 2    50   ~ 0
SCK-BUFFER
Text Label 2700 5900 0    50   ~ 0
CSn-BUFFER
Text Label 1900 5700 2    50   ~ 0
CIPO
Text Label 2700 6100 0    50   ~ 0
COPI-FEATHER
Text Label 1900 6000 2    50   ~ 0
SCK-FEATHER
Text Label 2700 5800 0    50   ~ 0
CSn-FEATHER
Text Label 1900 5800 2    50   ~ 0
CIPO-BUFFER
Text Label 2700 5700 0    50   ~ 0
GO
Text Label 2700 6000 0    50   ~ 0
GO
Text Label 1900 5600 2    50   ~ 0
GO
Text Label 1900 5900 2    50   ~ 0
GO
$Comp
L Device:C C1
U 1 1 5F88DB54
P 2300 5250
F 0 "C1" V 2048 5250 50  0000 C CNN
F 1 "0.1uF" V 2139 5250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2338 5100 50  0001 C CNN
F 3 "~" H 2300 5250 50  0001 C CNN
	1    2300 5250
	0    1    1    0   
$EndComp
Wire Wire Line
	2700 5600 2950 5600
Wire Wire Line
	2950 5600 2950 5250
Wire Wire Line
	2950 5250 2450 5250
Wire Wire Line
	1300 6200 1300 5250
$Comp
L power:+3V3 #PWR0105
U 1 1 5F88FD93
P 2950 5250
F 0 "#PWR0105" H 2950 5100 50  0001 C CNN
F 1 "+3V3" H 2965 5423 50  0000 C CNN
F 2 "" H 2950 5250 50  0001 C CNN
F 3 "" H 2950 5250 50  0001 C CNN
	1    2950 5250
	1    0    0    -1  
$EndComp
Connection ~ 2950 5250
$Comp
L power:GND #PWR0106
U 1 1 5F890268
P 1300 6200
F 0 "#PWR0106" H 1300 5950 50  0001 C CNN
F 1 "GND" H 1305 6027 50  0000 C CNN
F 2 "" H 1300 6200 50  0001 C CNN
F 3 "" H 1300 6200 50  0001 C CNN
	1    1300 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 5250 2150 5250
Wire Wire Line
	1300 6200 1900 6200
Connection ~ 1300 6200
Text Label 2500 2500 2    50   ~ 0
SCK-FEATHER
Text Label 2500 2600 2    50   ~ 0
COPI-FEATHER
Text Label 2500 2700 2    50   ~ 0
CIPO-FEATHER
Text Label 2500 2400 2    50   ~ 0
CSn-FEATHER
Text Label 2500 2300 2    50   ~ 0
GO
$Comp
L power:+3V3 #PWR0107
U 1 1 5F89710D
P 1650 1600
F 0 "#PWR0107" H 1650 1450 50  0001 C CNN
F 1 "+3V3" H 1665 1773 50  0000 C CNN
F 2 "" H 1650 1600 50  0001 C CNN
F 3 "" H 1650 1600 50  0001 C CNN
	1    1650 1600
	1    0    0    -1  
$EndComp
Text Label 2500 1500 2    50   ~ 0
Reset
$Comp
L power:GND #PWR0108
U 1 1 5F898B77
P 1650 1800
F 0 "#PWR0108" H 1650 1550 50  0001 C CNN
F 1 "GND" H 1655 1627 50  0000 C CNN
F 2 "" H 1650 1800 50  0001 C CNN
F 3 "" H 1650 1800 50  0001 C CNN
	1    1650 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1600 2500 1600
Wire Wire Line
	2500 1800 1650 1800
$Comp
L power:+5V #PWR0109
U 1 1 5F89CDF8
P 5050 2100
F 0 "#PWR0109" H 5050 1950 50  0001 C CNN
F 1 "+5V" H 5065 2273 50  0000 C CNN
F 2 "" H 5050 2100 50  0001 C CNN
F 3 "" H 5050 2100 50  0001 C CNN
	1    5050 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2100 5050 2100
$Comp
L power:GND #PWR0110
U 1 1 5F89FB8C
P 5500 1700
F 0 "#PWR0110" H 5500 1450 50  0001 C CNN
F 1 "GND" H 5505 1527 50  0000 C CNN
F 2 "" H 5500 1700 50  0001 C CNN
F 3 "" H 5500 1700 50  0001 C CNN
	1    5500 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 1700 5500 1700
Wire Wire Line
	4750 1700 4750 2000
Wire Wire Line
	4750 2000 4000 2000
Text Label 4300 5450 2    50   ~ 0
CIPO-BUFFER
Text Label 4600 5450 0    50   ~ 0
CIPO-FEATHER
Text Label 4300 5750 2    50   ~ 0
COPI-BUFFER
Text Label 4600 5750 0    50   ~ 0
COPI
Text Label 4600 6050 0    50   ~ 0
SCK
Text Label 4600 6350 0    50   ~ 0
CSn
Text Label 4300 6050 2    50   ~ 0
SCK-BUFFER
Text Label 4300 6350 2    50   ~ 0
CSn-BUFFER
Text Notes 3800 5150 0    50   ~ 0
Series Termination Resistors for SPI port\nValues TBD - 50-100 Ohms?
Wire Notes Line
	3600 4850 3600 6500
Wire Notes Line
	3600 6500 5500 6500
Wire Notes Line
	5500 6500 5500 4850
Wire Notes Line
	5500 4850 3600 4850
Wire Notes Line
	1400 1250 1400 3450
Wire Notes Line
	1400 3450 4500 3450
Wire Notes Line
	4500 3450 4500 1250
Wire Notes Line
	4500 1250 1400 1250
Text Notes 3900 1350 0    50   ~ 0
Feather Socket
Wire Notes Line
	1050 4850 3350 4850
Wire Notes Line
	3350 4850 3350 6550
Wire Notes Line
	3350 6550 1050 6550
Wire Notes Line
	1050 6550 1050 4850
Text Notes 1100 4950 0    50   ~ 0
Bus Tranceiver Gate
Wire Notes Line
	4600 1050 4600 2050
Wire Notes Line
	4600 2050 4950 2050
Wire Notes Line
	4950 2050 4950 1800
Wire Notes Line
	4950 1800 5300 1800
Wire Notes Line
	5300 1800 5300 2000
Wire Notes Line
	5750 2000 5750 1050
$Comp
L stargazer-backplane:ADAFRUIT_FEATHER U1
U 1 1 5F882C3E
P 3750 2450
F 0 "U1" H 3650 3300 60  0000 L CNN
F 1 "ADAFRUIT_FEATHER" H 3350 1750 60  0000 L CNN
F 2 "Modules:ADAFRUIT_FEATHER" H 3900 2250 60  0001 C CNN
F 3 "" H 3900 2250 60  0000 C CNN
	1    3750 2450
	-1   0    0    -1  
$EndComp
Wire Notes Line
	7000 1050 7000 2000
Wire Notes Line
	5300 2000 7000 2000
Wire Notes Line
	4600 1050 7000 1050
Text Notes 5800 1250 0    50   ~ 0
Connect Jumper to use\nBus +3v3 voltage
Text Notes 4650 1350 0    50   ~ 0
Connect Jumper to Disable\n5v0->3v3 on-board\nregulator
Text Notes 4850 1000 0    50   ~ 0
CONNECT BOTH OR NEITHER OF THESE JUMPERS!
$Comp
L Device:R R2
U 1 1 5F88667C
P 4450 5450
F 0 "R2" V 4243 5450 50  0000 C CNN
F 1 "50 Ohm" V 4334 5450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4380 5450 50  0001 C CNN
F 3 "~" H 4450 5450 50  0001 C CNN
	1    4450 5450
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5F8875B7
P 4450 5750
F 0 "R3" V 4243 5750 50  0000 C CNN
F 1 "50 Ohm" V 4334 5750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4380 5750 50  0001 C CNN
F 3 "~" H 4450 5750 50  0001 C CNN
	1    4450 5750
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5F88863F
P 4450 6050
F 0 "R4" V 4243 6050 50  0000 C CNN
F 1 "50 Ohm" V 4334 6050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4380 6050 50  0001 C CNN
F 3 "~" H 4450 6050 50  0001 C CNN
	1    4450 6050
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5F8896A8
P 4450 6350
F 0 "R5" V 4243 6350 50  0000 C CNN
F 1 "50 Ohm" V 4334 6350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4380 6350 50  0001 C CNN
F 3 "~" H 4450 6350 50  0001 C CNN
	1    4450 6350
	0    1    1    0   
$EndComp
$Comp
L stargazer-backplane:jumper J2
U 1 1 5F89F4D0
P 4650 1700
F 0 "J2" V 4838 1372 50  0000 R CNN
F 1 "jumper" V 4747 1372 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 4850 1750 50  0001 C CNN
F 3 "" H 4850 1750 50  0001 C CNN
	1    4650 1700
	0    -1   -1   0   
$EndComp
$Comp
L stargazer-backplane:jumper J3
U 1 1 5F8A67A0
P 6150 1850
F 0 "J3" H 6292 1385 50  0000 C CNN
F 1 "jumper" H 6292 1476 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 6350 1900 50  0001 C CNN
F 3 "" H 6350 1900 50  0001 C CNN
	1    6150 1850
	-1   0    0    1   
$EndComp
$Comp
L stargazer-backplane:card-header J1
U 1 1 5F8821E3
P 9750 800
F 0 "J1" H 10100 750 50  0000 L CNN
F 1 "card-header" H 9950 -4350 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x40_P2.54mm_Horizontal" H 10200 800 50  0001 C CNN
F 3 "" H 10200 800 50  0001 C CNN
	1    9750 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 5500 9750 5500
Wire Wire Line
	8300 5600 9750 5600
$Comp
L stargazer-backplane:card-programming-header-a J13
U 1 1 5F905E84
P 8300 3300
F 0 "J13" H 8742 3465 50  0000 C CNN
F 1 "card-programming-header-a" H 8742 3374 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x12_P2.54mm_Vertical" H 8650 3350 50  0001 C CNN
F 3 "" H 8650 3350 50  0001 C CNN
	1    8300 3300
	-1   0    0    -1  
$EndComp
$Comp
L stargazer-backplane:card-programming-header-b J14
U 1 1 5F90B4FF
P 8200 5400
F 0 "J14" H 8467 5565 50  0000 C CNN
F 1 "card-programming-header-b" H 8467 5474 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8550 5450 50  0001 C CNN
F 3 "" H 8550 5450 50  0001 C CNN
	1    8200 5400
	-1   0    0    -1  
$EndComp
$Sheet
S 3000 1450 550  1650
U 5F9273C1
F0 "debug-headers" 50
F1 "Debug Headers.sch" 50
F2 "RST" I L 3000 1500 50 
F3 "+3v3" I L 3000 1600 50 
F4 "AREF" I L 3000 1700 50 
F5 "GND" I L 3000 1800 50 
F6 "A0" I L 3000 1900 50 
F7 "A1" I L 3000 2000 50 
F8 "A2" I L 3000 2100 50 
F9 "A3" I L 3000 2200 50 
F10 "A4" I L 3000 2300 50 
F11 "A5" I L 3000 2400 50 
F12 "SCK" I L 3000 2500 50 
F13 "MOSI" I L 3000 2600 50 
F14 "MISO" I L 3000 2700 50 
F15 "D0" I L 3000 2800 50 
F16 "D1" I L 3000 2900 50 
F17 "DIO1" I L 3000 3000 50 
F18 "VBAT" I R 3550 1900 50 
F19 "EN" I R 3550 2000 50 
F20 "VBUS" I R 3550 2100 50 
F21 "D13" I R 3550 2200 50 
F22 "D12" I R 3550 2300 50 
F23 "D11" I R 3550 2400 50 
F24 "D10" I R 3550 2500 50 
F25 "D9" I R 3550 2600 50 
F26 "D6" I R 3550 2700 50 
F27 "D5" I R 3550 2800 50 
F28 "D3" I R 3550 2900 50 
F29 "D2" I R 3550 3000 50 
$EndSheet
Wire Wire Line
	3000 3000 2500 3000
Wire Wire Line
	3000 2900 2500 2900
Wire Wire Line
	3000 2800 2500 2800
Wire Wire Line
	3000 2200 2500 2200
Wire Wire Line
	3000 2100 2500 2100
Wire Wire Line
	3000 2000 2500 2000
Wire Wire Line
	3000 1900 2500 1900
Wire Wire Line
	3000 1800 2500 1800
Wire Wire Line
	3000 1700 2500 1700
Wire Wire Line
	3000 1600 2500 1600
Wire Wire Line
	3000 1500 2500 1500
Wire Wire Line
	3550 3000 4000 3000
Wire Wire Line
	3550 1900 4000 1900
Wire Wire Line
	3550 2000 4000 2000
Connection ~ 4000 2000
Wire Wire Line
	4000 2100 3550 2100
Connection ~ 4000 2100
Wire Wire Line
	4000 2200 3550 2200
Wire Wire Line
	4000 2300 3550 2300
Wire Wire Line
	4000 2400 3550 2400
Wire Wire Line
	4000 2500 3550 2500
Wire Wire Line
	4000 2600 3550 2600
Wire Wire Line
	4000 2700 3550 2700
Wire Wire Line
	4000 2800 3550 2800
Wire Wire Line
	4000 2900 3550 2900
$Comp
L Device:C C2
U 1 1 5F891B09
P 5500 3650
F 0 "C2" H 5615 3696 50  0000 L CNN
F 1 "1uF" H 5615 3605 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 5538 3500 50  0001 C CNN
F 3 "~" H 5500 3650 50  0001 C CNN
	1    5500 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5F892155
P 5850 3650
F 0 "C3" H 5965 3696 50  0000 L CNN
F 1 "1uF" H 5965 3605 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 5888 3500 50  0001 C CNN
F 3 "~" H 5850 3650 50  0001 C CNN
	1    5850 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5F89319C
P 5850 3950
F 0 "#PWR0111" H 5850 3700 50  0001 C CNN
F 1 "GND" H 5855 3777 50  0000 C CNN
F 2 "" H 5850 3950 50  0001 C CNN
F 3 "" H 5850 3950 50  0001 C CNN
	1    5850 3950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0112
U 1 1 5F893450
P 5500 3300
F 0 "#PWR0112" H 5500 3150 50  0001 C CNN
F 1 "+5V" H 5515 3473 50  0000 C CNN
F 2 "" H 5500 3300 50  0001 C CNN
F 3 "" H 5500 3300 50  0001 C CNN
	1    5500 3300
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0113
U 1 1 5F893699
P 5850 3300
F 0 "#PWR0113" H 5850 3150 50  0001 C CNN
F 1 "+3V3" H 5865 3473 50  0000 C CNN
F 2 "" H 5850 3300 50  0001 C CNN
F 3 "" H 5850 3300 50  0001 C CNN
	1    5850 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 3300 5500 3500
Wire Wire Line
	5500 3800 5850 3800
Wire Wire Line
	5850 3800 5850 3950
Connection ~ 5850 3800
Wire Wire Line
	5850 3500 5850 3300
$Comp
L stargazer-backplane:ADAFRUIT_FEATHER U1
U 2 1 5F882406
P 2750 2250
F 0 "U1" H 2800 2950 60  0000 C CNN
F 1 "ADAFRUIT_FEATHER" H 2700 1400 60  0000 C CNN
F 2 "Modules:ADAFRUIT_FEATHER" H 2900 2050 60  0001 C CNN
F 3 "" H 2900 2050 60  0000 C CNN
	2    2750 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 2700 2950 2700
Wire Wire Line
	3000 2600 2900 2600
Wire Wire Line
	3000 2500 2850 2500
Wire Wire Line
	3000 2400 2800 2400
Wire Wire Line
	3000 2300 2750 2300
Wire Wire Line
	2500 2700 2250 2700
Wire Wire Line
	2500 2600 2150 2600
Wire Wire Line
	2500 2500 2050 2500
Wire Wire Line
	2500 2400 1950 2400
Wire Wire Line
	2500 2300 1850 2300
Wire Wire Line
	2950 3750 2600 3750
Wire Wire Line
	2950 2700 2950 3750
$Comp
L Jumper:SolderJumper_2_Open CIPO1
U 1 1 5F8D38A7
P 2450 3750
F 0 "CIPO1" H 2450 3863 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 2450 3864 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 2450 3750 50  0001 C CNN
F 3 "~" H 2450 3750 50  0001 C CNN
	1    2450 3750
	1    0    0    -1  
$EndComp
$Comp
L Jumper:SolderJumper_2_Open COPI1
U 1 1 5F8D9CA7
P 2450 3950
F 0 "COPI1" H 2450 4063 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 2450 4064 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 2450 3950 50  0001 C CNN
F 3 "~" H 2450 3950 50  0001 C CNN
	1    2450 3950
	1    0    0    -1  
$EndComp
$Comp
L Jumper:SolderJumper_2_Open SCK1
U 1 1 5F8DD529
P 2450 4150
F 0 "SCK1" H 2450 4263 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 2450 4264 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 2450 4150 50  0001 C CNN
F 3 "~" H 2450 4150 50  0001 C CNN
	1    2450 4150
	1    0    0    -1  
$EndComp
$Comp
L Jumper:SolderJumper_2_Open A5/CSn1
U 1 1 5F8E0F1F
P 2450 4350
F 0 "A5/CSn1" H 2450 4463 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 2450 4464 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 2450 4350 50  0001 C CNN
F 3 "~" H 2450 4350 50  0001 C CNN
	1    2450 4350
	1    0    0    -1  
$EndComp
$Comp
L Jumper:SolderJumper_2_Open A4/GO1
U 1 1 5F8F4C89
P 2450 4550
F 0 "A4/GO1" H 2450 4663 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 2450 4664 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 2450 4550 50  0001 C CNN
F 3 "~" H 2450 4550 50  0001 C CNN
	1    2450 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3750 2250 3750
Wire Wire Line
	2250 2700 2250 3750
Wire Wire Line
	2900 3950 2600 3950
Wire Wire Line
	2900 2600 2900 3950
Wire Wire Line
	2150 3950 2300 3950
Wire Wire Line
	2150 2600 2150 3950
Wire Wire Line
	2050 4150 2300 4150
Wire Wire Line
	2050 2500 2050 4150
Wire Wire Line
	2850 4150 2600 4150
Wire Wire Line
	2850 2500 2850 4150
Wire Wire Line
	2800 4350 2600 4350
Wire Wire Line
	2800 2400 2800 4350
Wire Wire Line
	2750 4550 2600 4550
Wire Wire Line
	2750 2300 2750 4550
Wire Wire Line
	1950 4350 2300 4350
Wire Wire Line
	1950 2400 1950 4350
Wire Wire Line
	1850 4550 2300 4550
Wire Wire Line
	1850 2300 1850 4550
$EndSCHEMATC
