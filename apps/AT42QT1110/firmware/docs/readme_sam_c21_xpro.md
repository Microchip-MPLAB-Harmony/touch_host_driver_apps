---
grand_parent: Touch Applications
parent: AT42QT1110 Examples
title: SAMC21 Xplained Pro
has_toc: false
---
![Microchip logo](../../../../images/microchip_logo.png)
![Harmony logo small](../../../../images/microchip_mplab_harmony_logo_small.png)

#  SAMC21 Xplained PRO and AT42QT1110 EVK Example 

Path of the application within the repository is **apps/AT42QT1110/firmware**

To build the application, refer to the following table and open the project using its IDE.

## Host Driver Application

| Project Name      | Description                                    |
| ----------------- | ---------------------------------------------- |
| AT42QT1110/firmware/sam_c21_xpro.X    | MPLABX Project for [SAM C21 Xplained Pro Evaluation Kit](https://www.microchip.com/developmenttools/ProductDetails/atsamc21-xpro)|
|||

## Setting up the hardware
1. Connect the Host microcontroller and the turnkey device
2. Connect SPI lines, VCC, GND from AT42QT1110 to SAMC21 Xplained Pro through external wires
2. Connect the Debug USB port on the board to the computer using a micro USB cable
## Running The Application

1. Open the application project */firmware/sam_c1_xpro.X* in the IDE
2. Build the project and program the target board

## Monitoring Touch Data in MPLAB DV Touch plugin
**Prerequisite**
MPLAB Data Visualizer and Touch Plugin. Procedure for Installation is available in [Introduction to Touch Plugin](https://microchipdeveloper.com/touch:introduction-to-touch-plugin).

1. Configure COM Port
    -    Select the correct COM port in **Connection** tab
    -    Update the **Baud Rate** to 115200 in **COM Settings**
    -    Start **Streaming** for COM data
2. COM Port on Touch
    - Select Touch in **Connection** tab
    - For data source, from drop-down menu, select configured COM port as source
    - Start **Streaming** on Touch

Detailed procedure can be found in [Introduction to Touch Plugin](https://microchipdeveloper.com/touch:introduction-to-touch-plugin) page.