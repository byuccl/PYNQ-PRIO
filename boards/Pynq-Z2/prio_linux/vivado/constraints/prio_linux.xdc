## Switches
#set_property -dict {PACKAGE_PIN M20 IOSTANDARD LVCMOS33} [get_ports {sws_2bits_tri_i[0]}]
#set_property -dict {PACKAGE_PIN M19 IOSTANDARD LVCMOS33} [get_ports {sws_2bits_tri_i[1]}]

## Buttons
#set_property -dict {PACKAGE_PIN D19 IOSTANDARD LVCMOS33} [get_ports {btns_4bits_tri_i[0]}]
#set_property -dict {PACKAGE_PIN D20 IOSTANDARD LVCMOS33} [get_ports {btns_4bits_tri_i[1]}]
#set_property -dict {PACKAGE_PIN L20 IOSTANDARD LVCMOS33} [get_ports {btns_4bits_tri_i[2]}]
#set_property -dict {PACKAGE_PIN L19 IOSTANDARD LVCMOS33} [get_ports {btns_4bits_tri_i[3]}]

## LEDs
#set_property -dict {PACKAGE_PIN R14 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[0]}]
#set_property -dict {PACKAGE_PIN P14 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[1]}]
#set_property -dict {PACKAGE_PIN N16 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[2]}]
#set_property -dict {PACKAGE_PIN M14 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[3]}]

## RGBLEDs
#set_property -dict { PACKAGE_PIN L15   IOSTANDARD LVCMOS33 } [get_ports { rgbleds_6bits_tri_o[0] }];
#set_property -dict { PACKAGE_PIN G17   IOSTANDARD LVCMOS33 } [get_ports { rgbleds_6bits_tri_o[1] }];
#set_property -dict { PACKAGE_PIN N15   IOSTANDARD LVCMOS33 } [get_ports { rgbleds_6bits_tri_o[2] }];
#set_property -dict { PACKAGE_PIN G14   IOSTANDARD LVCMOS33 } [get_ports { rgbleds_6bits_tri_o[3] }];
#set_property -dict { PACKAGE_PIN L14   IOSTANDARD LVCMOS33 } [get_ports { rgbleds_6bits_tri_o[4] }];
#set_property -dict { PACKAGE_PIN M15   IOSTANDARD LVCMOS33 } [get_ports { rgbleds_6bits_tri_o[5] }];

### pmod Header JA, pr_4
set_property -dict {PACKAGE_PIN Y18 IOSTANDARD LVCMOS33} [get_ports {pr_4_tri_io[0]}]
set_property -dict {PACKAGE_PIN Y19 IOSTANDARD LVCMOS33} [get_ports {pr_4_tri_io[1]}]
set_property -dict {PACKAGE_PIN Y17 IOSTANDARD LVCMOS33} [get_ports {pr_4_tri_io[3]}]
set_property -dict {PACKAGE_PIN Y16 IOSTANDARD LVCMOS33} [get_ports {pr_4_tri_io[2]}]
set_property -dict {PACKAGE_PIN U19 IOSTANDARD LVCMOS33} [get_ports {pr_4_tri_io[5]}]
set_property -dict {PACKAGE_PIN U18 IOSTANDARD LVCMOS33} [get_ports {pr_4_tri_io[4]}]
set_property -dict {PACKAGE_PIN W19 IOSTANDARD LVCMOS33} [get_ports {pr_4_tri_io[7]}]
set_property -dict {PACKAGE_PIN W18 IOSTANDARD LVCMOS33} [get_ports {pr_4_tri_io[6]}]

### pmod Header JB, pr_5
set_property -dict {PACKAGE_PIN W14 IOSTANDARD LVCMOS33} [get_ports {pr_5_tri_io[0]}]
set_property -dict {PACKAGE_PIN Y14 IOSTANDARD LVCMOS33} [get_ports {pr_5_tri_io[1]}]
set_property -dict {PACKAGE_PIN T10 IOSTANDARD LVCMOS33} [get_ports {pr_5_tri_io[3]}]
set_property -dict {PACKAGE_PIN T11 IOSTANDARD LVCMOS33} [get_ports {pr_5_tri_io[2]}]
set_property -dict {PACKAGE_PIN W16 IOSTANDARD LVCMOS33} [get_ports {pr_5_tri_io[5]}]
set_property -dict {PACKAGE_PIN V16 IOSTANDARD LVCMOS33} [get_ports {pr_5_tri_io[4]}]
set_property -dict {PACKAGE_PIN W13 IOSTANDARD LVCMOS33} [get_ports {pr_5_tri_io[7]}]
set_property -dict {PACKAGE_PIN V12 IOSTANDARD LVCMOS33} [get_ports {pr_5_tri_io[6]}]

## Arduino shield digital io pins 0-7, pr__0
set_property -dict {PACKAGE_PIN T14 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[0]}]
set_property -dict {PACKAGE_PIN U12 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[1]}]
set_property -dict {PACKAGE_PIN U13 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[2]}]
set_property -dict {PACKAGE_PIN V13 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[3]}]
set_property -dict {PACKAGE_PIN V15 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[4]}]
set_property -dict {PACKAGE_PIN T15 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[5]}]
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[6]}]
set_property -dict {PACKAGE_PIN U17 IOSTANDARD LVCMOS33} [get_ports {pr_0_tri_io[7]}]

### Arduino shield digital io pins 8-13, pr__1
set_property -dict {PACKAGE_PIN V17 IOSTANDARD LVCMOS33} [get_ports {pr_1_tri_io[0]}]
set_property -dict {PACKAGE_PIN V18 IOSTANDARD LVCMOS33} [get_ports {pr_1_tri_io[1]}]
set_property -dict {PACKAGE_PIN T16 IOSTANDARD LVCMOS33} [get_ports {pr_1_tri_io[2]}]
set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVCMOS33} [get_ports {pr_1_tri_io[3]}]
set_property -dict {PACKAGE_PIN P18 IOSTANDARD LVCMOS33} [get_ports {pr_1_tri_io[4]}]
set_property -dict {PACKAGE_PIN N17 IOSTANDARD LVCMOS33} [get_ports {pr_1_tri_io[5]}]
set_property -dict {PACKAGE_PIN Y11 IOSTANDARD LVCMOS33} [get_ports {pr_1_tri_io[6]}]
set_property -dict {PACKAGE_PIN Y12 IOSTANDARD LVCMOS33} [get_ports {pr_1_tri_io[7]}]

### RPi GPIO 15-8, pr_2
set_property -dict { PACKAGE_PIN F19    IOSTANDARD LVCMOS33 } [get_ports { pr_2_tri_io[0] }];
set_property -dict { PACKAGE_PIN V10    IOSTANDARD LVCMOS33 } [get_ports { pr_2_tri_io[1] }];
set_property -dict { PACKAGE_PIN V8     IOSTANDARD LVCMOS33 } [get_ports { pr_2_tri_io[2] }];
set_property -dict { PACKAGE_PIN W10    IOSTANDARD LVCMOS33 } [get_ports { pr_2_tri_io[3] }];
set_property -dict { PACKAGE_PIN B20    IOSTANDARD LVCMOS33 } [get_ports { pr_2_tri_io[4] }];
set_property -dict { PACKAGE_PIN W8     IOSTANDARD LVCMOS33 } [get_ports { pr_2_tri_io[5] }];
set_property -dict { PACKAGE_PIN V6     IOSTANDARD LVCMOS33 } [get_ports { pr_2_tri_io[6] }];
set_property -dict { PACKAGE_PIN Y6     IOSTANDARD LVCMOS33 } [get_ports { pr_2_tri_io[7] }];

### RPi GPIO 23-16, pr_3
set_property -dict { PACKAGE_PIN B19    IOSTANDARD LVCMOS33 } [get_ports { pr_3_tri_io[0] }];
set_property -dict { PACKAGE_PIN U7     IOSTANDARD LVCMOS33 } [get_ports { pr_3_tri_io[1] }];
set_property -dict { PACKAGE_PIN C20    IOSTANDARD LVCMOS33 } [get_ports { pr_3_tri_io[2] }];
set_property -dict { PACKAGE_PIN Y8     IOSTANDARD LVCMOS33 } [get_ports { pr_3_tri_io[3] }];
set_property -dict { PACKAGE_PIN A20    IOSTANDARD LVCMOS33 } [get_ports { pr_3_tri_io[4] }];
set_property -dict { PACKAGE_PIN Y9     IOSTANDARD LVCMOS33 } [get_ports { pr_3_tri_io[5] }];
set_property -dict { PACKAGE_PIN U8     IOSTANDARD LVCMOS33 } [get_ports { pr_3_tri_io[6] }];
set_property -dict { PACKAGE_PIN W6     IOSTANDARD LVCMOS33 } [get_ports { pr_3_tri_io[7] }];
