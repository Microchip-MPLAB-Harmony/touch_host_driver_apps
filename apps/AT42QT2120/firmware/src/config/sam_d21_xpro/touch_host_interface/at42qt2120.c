/*******************************************************************************
  MPLAB Harmony Touch Host Interface v1.1.0 Release

  Company:
    Microchip Technology Inc.

  File Name:
    at42qt2120.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
 * Copyright (C) 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * Subject to your compliance with these terms, you may use Microchip software
 * and any derivatives exclusively with Microchip products. It is your
 * responsibility to comply with third party license terms applicable to your
 * use of third party software (including open source software) that may
 * accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
 * ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "at42qt2120.h"
#include "touchI2C.h"

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

static volatile uint8_t touchDeviceDataReceived;
static volatile uint8_t touchDeviceTransmitInProgress;
volatile uint8_t timeToRead;
#define NUM_OF_TRY 500u
CommunicationStatus_t communicationStatus;
configurationDataT configData;
debugDataT debugData;
statusDataT statusData;
static touchI2CState_t touchI2CState = STATE_INVALID;

static uint8_t touchDeviceVerifyChipID(void);
void touchDeviceReadConfigurationData(void);
void touchDeviceReadDebugData(void);
void touchDeviceReadStatusData(void);
void touchDeviceProcessData(void);

static uint8_t touchDeviceVerifyChipID(void)
{
    uint8_t retvar = 0u;
    touchI2cSetMemoryAddrss(getChipIDAddress());
    if (touchI2cReadByte() == getDeviceExpectedChipID())
    {
        retvar = 1u;
        communicationStatus.initError = 0u;
    }
    else
    {
        communicationStatus.initError = 1u;
    }

    return retvar;
}

void touchDeviceReadConfigurationData(void)
{
    uint8_t address = getConfigStartAddress();
    uint16_t try = 0u;
    touchDeviceDataReceived = 0;
    do
    {
        try++;
        touchI2cReceiveDataFromAddress(SLAVE_DEVICE_ADDRESS, address, (uint8_t *)&configData.lowPowerMode, (transferSize_t)sizeof(configData));
    } while ((touchDeviceDataReceived == 0u) && (try < NUM_OF_TRY));
    if (try >= NUM_OF_TRY)
    {
        communicationStatus.ConfigDataError = 1u;
    }
}

void touchDeviceReadDebugData(void)
{
    uint8_t address = getDebugDataStartAddress();
    uint16_t try = 0u;
    touchDeviceDataReceived = 0u;
    do
    {
        try++;
        touchI2cReceiveDataFromAddress(SLAVE_DEVICE_ADDRESS, address, (uint8_t *)&debugData.key_signal[0], (transferSize_t)sizeof(debugData));
    } while ((touchDeviceDataReceived == 0u) && (try < NUM_OF_TRY));
    if (try >= NUM_OF_TRY)
    {
        communicationStatus.DebugDataError = 1u;
    }
}

void touchDeviceReadStatusData(void)
{
    uint8_t address = getChipIDAddress();
    uint16_t try = 0;
    touchDeviceDataReceived = 0u;
    do
    {
        try++;
        touchI2cReceiveDataFromAddress(SLAVE_DEVICE_ADDRESS, address, (uint8_t *)&statusData.chip_id, (transferSize_t)sizeof(statusData));
    } while ((touchDeviceDataReceived == 0u) && (try < NUM_OF_TRY));
    if (try >= NUM_OF_TRY)
    {
        communicationStatus.StatusDataError = 1u;
    }
}

void touchDeviceProcessData(void)
{
    touchTuneNewDataAvailable();
}

void touchDeviceProcess(void)
{
    static uint8_t init = 0u;
    if (init == 0u)
    {
        init = 1u;
        touchDeviceInit();
        touchTuneInit();
    }

    touchI2CState = READ_DEBUG_DATA_FROM_TOUCH;
    touchDeviceReadDebugData();

    if (touchDeviceDataReceived == 1u)
    {
        touchDeviceDataReceived = 0u;
        switch (touchI2CState)
        {

        case READ_DEBUG_DATA_FROM_TOUCH:
            touchDeviceReadDebugData();
            touchI2CState = READ_STATUS_DATA_FROM_TOUCH;
            break;

        case READ_STATUS_DATA_FROM_TOUCH:
            touchDeviceReadStatusData();
            touchI2CState = SEND_DATA_TO_PC;
            break;

        case SEND_DATA_TO_PC:
            touchDeviceProcessData();
            touchI2CState = STATE_INVALID;
            break;
        default:
            ;   // do nothing...
            break;
        }
    }
}

void touchDeviceRxCompleteCallback(uint8_t data)
{
    touchDeviceDataReceived = 1u;
}

void touchDeviceTxCompleteCallback(void)
{
    touchDeviceTransmitInProgress = 0u;
}

void touchDeviceInit(void)
{
    touchI2cInit(touchDeviceTxCompleteCallback, touchDeviceRxCompleteCallback);
    touchI2cSetSlaveAddress(SLAVE_DEVICE_ADDRESS);
    if (touchDeviceVerifyChipID() == 1u)
    {
        touchDeviceReadConfigurationData();
        touchDeviceReadDebugData();
    }
}

/*******************************************************************************
 End of File
 */
