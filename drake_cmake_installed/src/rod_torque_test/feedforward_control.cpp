//============================================================================
// Name        : HelloEposCmd.cpp
// Author      : Dawid Sienkiewicz
// Version     :
// Copyright   : maxon motor ag 2014-2017
// Description : Hello Epos in C++
//============================================================================

#include <iostream>
#include "Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>
#include <stdint.h>
#include <math.h>
#include <chrono>
#include <cmath>

#include "torque_calculator.h"

typedef void* HANDLE;
typedef int BOOL;

enum EAppMode
{
    AM_UNKNOWN,
    AM_DEMO,
    AM_INTERFACE_LIST,
    AM_PROTOCOL_LIST,
    AM_VERSION_INFO
};

using namespace std;
using namespace std::chrono;

void* g_pKeyHandle = 0;
unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;
EAppMode g_eAppMode = AM_DEMO;

const string g_programName = "HelloEposCmd";

const int EC52_GP52_HALL_COUNT_PER_ROUND = 2544;
const int EC52_GP52_REDUCTION_RATIO = 53;

#ifndef MMC_SUCCESS
    #define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
    #define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
    #define MMC_MAX_LOG_MSG_SIZE 512
#endif

void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintUsage();
void  PrintHeader();
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int   ParseArguments(int argc, char** argv);
int   Demo(unsigned int* p_pErrorCode);
int   PrepareDemo(unsigned int* p_pErrorCode);
int   PrintAvailableInterfaces();
int   PrintAvailablePorts(char* p_pInterfaceNameSel);
int   PrintAvailableProtocols();
int   PrintDeviceVersion();

void PrintUsage()
{
    cout << "Usage: HelloEposCmd" << endl;
    cout << "\t-h : this help" << endl;
    cout << "\t-n : node id (default 1)" << endl;
    cout << "\t-d   : device name (EPOS2, EPOS4, default - EPOS4)"  << endl;
    cout << "\t-s   : protocol stack name (MAXON_RS232, CANopen, MAXON SERIAL V2, default - MAXON SERIAL V2)"  << endl;
    cout << "\t-i   : interface name (RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default - USB)"  << endl;
    cout << "\t-p   : port name (COM1, USB0, CAN0,... default - USB0)" << endl;
    cout << "\t-b   : baudrate (115200, 1000000,... default - 1000000)" << endl;
    cout << "\t-l   : list available interfaces (valid device name and protocol stack required)" << endl;
    cout << "\t-r   : list supported protocols (valid device name required)" << endl;
    cout << "\t-v   : display device version" << endl;
}

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
    cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

void LogInfo(string message)
{
    cout << message << endl;
}

void SeparatorLine()
{
    const int lineLength = 65;
    for(int i=0; i<lineLength; i++)
    {
        cout << "-";
    }
    cout << endl;
}

void PrintSettings()
{
    stringstream msg;

    msg << "default settings:" << endl;
    msg << "node id             = " << g_usNodeId << endl;
    msg << "device name         = '" << g_deviceName << "'" << endl;
    msg << "protocal stack name = '" << g_protocolStackName << "'" << endl;
    msg << "interface name      = '" << g_interfaceName << "'" << endl;
    msg << "port name           = '" << g_portName << "'"<< endl;
    msg << "baudrate            = " << g_baudrate;

    LogInfo(msg.str());

    SeparatorLine();
}

void SetDefaultParameters()
{
    //USB
    g_usNodeId = 1;
    g_deviceName = "EPOS4"; 
    g_protocolStackName = "MAXON SERIAL V2"; 
    g_interfaceName = "USB"; 
    g_portName = "USB0"; 
    g_baudrate = 1000000; 
}

int OpenDevice(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;

    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];

    strcpy(pDeviceName, g_deviceName.c_str());
    strcpy(pProtocolStackName, g_protocolStackName.c_str());
    strcpy(pInterfaceName, g_interfaceName.c_str());
    strcpy(pPortName, g_portName.c_str());

    LogInfo("Open device...");

    g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

    if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
    {
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
        {
            if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
            {
                if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
                {
                    if(g_baudrate==(int)lBaudrate)
                    {
                        lResult = MMC_SUCCESS;
                    }
                }
            }
        }
    }
    else
    {
        g_pKeyHandle = 0;
    }

    delete []pDeviceName;
    delete []pProtocolStackName;
    delete []pInterfaceName;
    delete []pPortName;

    return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;

    *p_pErrorCode = 0;

    LogInfo("Close device");

    if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
    {
        lResult = MMC_SUCCESS;
    }

    return lResult;
}

int ParseArguments(int argc, char** argv)
{
    int lOption;
    int lResult = MMC_SUCCESS;

    opterr = 0;

    while((lOption = getopt(argc, argv, "hlrvd:s:i:p:b:n:")) != -1)
    {
        switch (lOption)
        {
            case 'h':
                PrintUsage();
                lResult = 1;
                break;
            case 'd':
                g_deviceName = optarg;
                break;
            case 's':
                g_protocolStackName = optarg;
                break;
            case 'i':
                g_interfaceName = optarg;
                break;
            case 'p':
                g_portName = optarg;
                break;
            case 'b':
                g_baudrate = atoi(optarg);
                break;
            case 'n':
                g_usNodeId = (unsigned short)atoi(optarg);
                break;
            case 'l':
                g_eAppMode = AM_INTERFACE_LIST;
                break;
            case 'r':
                g_eAppMode = AM_PROTOCOL_LIST;
                break;
            case 'v':
                g_eAppMode = AM_VERSION_INFO;
                break;
            case '?':  // unknown option...
                stringstream msg;
                msg << "Unknown option: '" << char(optopt) << "'!";
                LogInfo(msg.str());
                PrintUsage();
                lResult = MMC_FAILED;
                break;
        }
    }

    return lResult;
}

double ConvertDegreeToRadian(double angleInDegree) {
    return angleInDegree / 180 * M_PI;
}

int ConvertRelativeAngleToRelativeHallSensorCount(double angleInRadian, int countPerRound) {
    double angleInRadianInRound = std::fmod(angleInRadian, 2 * M_PI);
    return round(angleInRadianInRound / (2 * M_PI) * countPerRound);
}

double ConvertRelativeHallSensorCountToRelativeAngle(int32_t count, int countPerRound) {
    int32_t countInRound = count % countPerRound;
    return countInRound * 1.0 / countPerRound * 2 * M_PI;
}

int TestProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    stringstream msg;
    int retcode;

    msg << "set profile position mode, node = " << p_usNodeId;
    LogInfo(msg.str());

    if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
    {
        LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }
    else
    {
        double targetAngleDegree = 5;
        double targetAngleRadian = ConvertDegreeToRadian(targetAngleDegree);
        long targetPosition = ConvertRelativeAngleToRelativeHallSensorCount(targetAngleRadian, EC52_GP52_HALL_COUNT_PER_ROUND);

        stringstream msg;
        msg << "move to position = " << targetPosition << ", node = " << p_usNodeId;
        LogInfo(msg.str());

        // VCS_MoveToPosition(void* KeyHandle, unsigned short NodeId, long TargetPosition, int Absolute, int Immediately, unsigned int* pErrorCode);
        if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, targetPosition, false, true, &p_rlErrorCode) == 0)
        {
            LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }

        unsigned int NbOfBytesRead;        

        // When the motor is still moving, we get real-time updates on various paramters
        while (VCS_WaitForTargetReached(p_DeviceHandle, p_usNodeId, 1, &p_rlErrorCode) == 0) {
            int cur_position = 0;
            // int cur_velocity = 0;
            // int cur_velocity_avg = 0;

            // Get real-time position value
            retcode = VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &cur_position, &p_rlErrorCode);
            if (retcode != 0) {
                cout << "current position = " << cur_position << "\n";
            } else {
                LogError("VCS_GetPositionIs", lResult, p_rlErrorCode);
                lResult = MMC_FAILED;
            }

            // // NOTE: this seems to always return 0
            // retcode = VCS_GetVelocityIs(p_DeviceHandle, p_usNodeId, &cur_velocity, &p_rlErrorCode);
            // if (retcode != 0) {
            //     cout << "current velocity = " << cur_velocity << "\n";
            // } else {
            //     LogError("VCS_GetVelocityIs", lResult, p_rlErrorCode);
            //     lResult = MMC_FAILED;
            // }

            // // Get real-time velocity avg value
            // retcode = VCS_GetVelocityIsAveraged(p_DeviceHandle, p_usNodeId, &cur_velocity_avg, &p_rlErrorCode);
            // if (retcode != 0) {
            //     cout << "current velocity avg = " << cur_velocity_avg << "\n";
            // } else {
            //     LogError("VCS_GetVelocityIsAveraged", lResult, p_rlErrorCode);
            //     lResult = MMC_FAILED;
            // }

            /*
            VCS_GetObject(void* KeyHandle, unsigned short NodeId, unsigned short ObjectIndex, unsigned char ObjectSubIndex, void* pData, unsigned int NbOfBytesToRead, unsigned int* pNbOfBytesRead, unsigned int* pErrorCode);
            */
            
            // int16_t torqueActualValue;
            // bool gottorqueActualValue = false;
            // if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6077, 0x00, &torqueActualValue, 2, &NbOfBytesRead, &p_rlErrorCode) == 0) {
            //     LogError("VCS_GetObject", lResult, p_rlErrorCode);
            //     lResult = MMC_FAILED;
            // } else {
            //     // cout << "torqueActualValue (0x6077): " << torqueActualValue << "\n";
            //     gottorqueActualValue = true;
            // }

            // int32_t currentActualValue;
            // bool gotcurrentActualValue = false;
            // if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x30D1, 0x02, &currentActualValue, 4, &NbOfBytesRead, &p_rlErrorCode) == 0) {
            //     LogError("VCS_GetObject", lResult, p_rlErrorCode);
            //     lResult = MMC_FAILED;
            // } else {
            //     gotcurrentActualValue = true;
            // }

            // int32_t currentActualValueAvg;
            // bool gotcurrentActualValueAvg = false;
            // if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x30D1, 0x01, &currentActualValueAvg, 4, &NbOfBytesRead, &p_rlErrorCode) == 0) {
            //     LogError("VCS_GetObject", lResult, p_rlErrorCode);
            //     lResult = MMC_FAILED;
            // } else {
            //     gotcurrentActualValueAvg = true;
            // }

            // if (gottorqueActualValue && gotcurrentActualValue && gotcurrentActualValueAvg) {
            //     cout << torqueActualValue << "," << currentActualValue << "," << currentActualValueAvg << "\n";
            // }
        }

        // if(VCS_WaitForTargetReached(p_DeviceHandle, p_usNodeId, 10 * 1000, &p_rlErrorCode) == 0)
        // {
        //     LogError("VCS_WaitForTargetReached", lResult, p_rlErrorCode);
        //     lResult = MMC_FAILED;
        // }

        if(lResult == MMC_SUCCESS)
        {
            LogInfo("halt position movement");

            if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
            {
                LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
                lResult = MMC_FAILED;
            }
        }
    }

    return lResult;
}

int ActivateCyclicSynchronousPositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode) {
    // Set Modes of operation to 0x08 (CSP)
    unsigned int NbOfBytesWritten;
    int8_t desiredModesOfOperation = 0x08;
    int retcode = VCS_SetObject(p_DeviceHandle, p_usNodeId, 0x6060, 0x00, &desiredModesOfOperation, sizeof(desiredModesOfOperation), &NbOfBytesWritten, &p_rlErrorCode);
    if (retcode == 0) {
        return retcode;
    } else {
        cout << "Successfully write " << (int)desiredModesOfOperation << " to Modes of operation\n";
        char operationMode;
        if(VCS_GetOperationMode(p_DeviceHandle, p_usNodeId, &operationMode, &p_rlErrorCode) == 0) {
            return 0;
        } else {
            cout << "New operationMode: " << (int)operationMode << "\n";
        }
        return retcode;
    }
}

int SetTorqueOffsetCSP(HANDLE p_DeviceHandle, unsigned short p_usNodeId, int16_t torqueOffsetPerThousandOfRatedTorque, unsigned int & p_rlErrorCode) {
    unsigned int NbOfBytesWritten;
    return VCS_SetObject(p_DeviceHandle, p_usNodeId, 0x60B2, 0x00, &torqueOffsetPerThousandOfRatedTorque, sizeof(torqueOffsetPerThousandOfRatedTorque), &NbOfBytesWritten, &p_rlErrorCode);
}

int MoveToPositionCSP(HANDLE p_DeviceHandle, unsigned short p_usNodeId, int32_t targetAbsolutePosition, unsigned int & p_rlErrorCode) {
    unsigned int NbOfBytesWritten;
    return VCS_SetObject(p_DeviceHandle, p_usNodeId, 0x607A, 0x00, &targetAbsolutePosition, sizeof(targetAbsolutePosition), &NbOfBytesWritten, &p_rlErrorCode);
}

int WaitForTargetReachedCSP(HANDLE p_DeviceHandle, unsigned short p_usNodeId, int32_t targetAbsolutePosition, unsigned int timeout, unsigned int & p_rlErrorCode) {
    unsigned int NbOfBytesRead;
    int32_t positionActualValue;

    high_resolution_clock::time_point start_time = high_resolution_clock::now();

    while (std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - start_time).count() < timeout) {
        if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6064, 0x00, &positionActualValue, sizeof(positionActualValue), &NbOfBytesRead, &p_rlErrorCode) == 0) {
            return 0;
        } else {
            if (positionActualValue == targetAbsolutePosition) {
                return 1;
            }
        }
    }
    return 0;
}

int TestGravityCompensationForRodAndPayload(HANDLE p_DeviceHandle, unsigned short p_usNodeId, bool hasGravityCompensation, double payloadWeight, double angleDeltaInDegree, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    stringstream msg;
    int retcode;
    TorqueCalculator torqueCalculator;
    unsigned int NbOfBytesRead;
    double neededTorque;
    double neededTorqueOnMotor;
    int16_t torqueOffsetPerThousandOfRatedTorque;

    torqueCalculator.LoadModel(payloadWeight);

    msg << "set CyclicSynchronousPositionMode, node = " << p_usNodeId << "\n";
    LogInfo(msg.str());

    if (ActivateCyclicSynchronousPositionMode(p_DeviceHandle, p_usNodeId, p_rlErrorCode) == 0) {
        LogError("ActivateCyclicSynchronousPositionMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
        return lResult;
    }

    // Get start position
    int startPosition = 0;
    if (VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &startPosition, &p_rlErrorCode) == 0) {
        LogError("VCS_GetPositionIs", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
        return lResult;
    }
    // IMPORTANT IMPORTANT!: We assume the start angle to always be 90-degree. Change this if it isn't true anymore.
    double startAngleFromPerpendicular = M_PI / 2;

    // Get motor rated torque
    uint32_t motorRatedTorque; // Unit: uN * m
    if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6076, 0x00, &motorRatedTorque, 4, &NbOfBytesRead, &p_rlErrorCode) == 0) {
        LogError("VCS_GetObject", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
        return lResult;
    }

    // Set torque offset
    neededTorque = torqueCalculator.CalcGravityCompensatingTorque(startAngleFromPerpendicular); // Unit: N * m
    // cout << "neededTorque: " << neededTorque << "\n";
    neededTorqueOnMotor = neededTorque / EC52_GP52_REDUCTION_RATIO;

    if (hasGravityCompensation) {
        torqueOffsetPerThousandOfRatedTorque = (int16_t)(neededTorqueOnMotor * 1000000 * 1000 / motorRatedTorque);
    } else {
        torqueOffsetPerThousandOfRatedTorque = 0;
    }
    cout << "torqueOffsetPerRatedTorque: " << torqueOffsetPerThousandOfRatedTorque * 1.0 / 1000 << "\n";
    if (SetTorqueOffsetCSP(p_DeviceHandle, p_usNodeId, torqueOffsetPerThousandOfRatedTorque, p_rlErrorCode) == 0) {
        LogError("SetTorqueOffsetCSP", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
        return lResult;
    }
    
    // Set target position
    double targetRelativeAngleDegree = angleDeltaInDegree;
    double targetRelativeAngleRadian = ConvertDegreeToRadian(targetRelativeAngleDegree);
    long targetRelativePosition = ConvertRelativeAngleToRelativeHallSensorCount(targetRelativeAngleRadian, EC52_GP52_HALL_COUNT_PER_ROUND);
    long targetAbsolutePosition = startPosition + targetRelativePosition;

    msg << "move to position = " << targetAbsolutePosition << ", node = " << p_usNodeId;
    LogInfo(msg.str());

    if (MoveToPositionCSP(p_DeviceHandle, p_usNodeId, targetAbsolutePosition, p_rlErrorCode) == 0) {
        LogError("MoveToPositionCSP", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
        return lResult;
    }

    while (WaitForTargetReachedCSP(p_DeviceHandle, p_usNodeId, targetAbsolutePosition, 1, p_rlErrorCode) == 0) {
        // Get real-time position value
        int currentPosition;
        retcode = VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &currentPosition, &p_rlErrorCode);
        if (retcode != 0) {
            cout << "current position = " << currentPosition << "\n";
        } else {
            LogError("VCS_GetPositionIs", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
            return lResult;
        }

        double angleDelta = ConvertRelativeHallSensorCountToRelativeAngle((currentPosition - startPosition), EC52_GP52_HALL_COUNT_PER_ROUND);
        double currentAngleFromPerpendicular = startAngleFromPerpendicular + angleDelta;
        neededTorque = torqueCalculator.CalcGravityCompensatingTorque(currentAngleFromPerpendicular); // Unit: N * m
        // cout << "neededTorque: " << neededTorque << "\n";
        neededTorqueOnMotor = neededTorque / EC52_GP52_REDUCTION_RATIO;

        if (hasGravityCompensation) {
            torqueOffsetPerThousandOfRatedTorque = (int16_t)(neededTorqueOnMotor * 1000000 * 1000 / motorRatedTorque);
        } else {
            torqueOffsetPerThousandOfRatedTorque = 0;
        }
        cout << "torqueOffsetPerRatedTorque: " << torqueOffsetPerThousandOfRatedTorque * 1.0 / 1000 << "\n";
        
        if (SetTorqueOffsetCSP(p_DeviceHandle, p_usNodeId, torqueOffsetPerThousandOfRatedTorque, p_rlErrorCode) == 0) {
            LogError("SetTorqueOffsetCSP", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
            return lResult;
        }
    }

    if(lResult == MMC_SUCCESS)
    {
        LogInfo("halt position movement");

        if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
        {
            LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }
    }

    return lResult;
}

int PrepareDemo(unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    BOOL oIsFault = 0;

    if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
    {
        LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
        lResult = MMC_FAILED;
    }

    if(lResult==0)
    {
        if(oIsFault)
        {
            stringstream msg;
            msg << "clear fault, node = '" << g_usNodeId << "'";
            LogInfo(msg.str());

            if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
            {
                LogError("VCS_ClearFault", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
        }

        if(lResult==0)
        {
            BOOL oIsEnabled = 0;

            if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
            {
                LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }

            if(lResult==0)
            {
                if(!oIsEnabled)
                {
                    if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
                    {
                        LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
                        lResult = MMC_FAILED;
                    }
                }
            }
        }
    }

    return lResult;
}

int Demo(unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    unsigned int lErrorCode = 0;
    unsigned int NbOfBytesRead;
    unsigned int NbOfBytesWritten;
    int8_t desiredModesOfOperation;

    // int  VCS_GetOperationMode(void* KeyHandle, unsigned short NodeId, char* pOperationMode, unsigned int* pErrorCode);

    char operationMode;
    if(VCS_GetOperationMode(g_pKeyHandle, g_usNodeId, &operationMode, p_pErrorCode) == 0) {
        LogError("VCS_GetOperationMode", lResult, *p_pErrorCode);
        lResult = MMC_FAILED;
    } else {
        cout << "operationMode: " << (int)operationMode << "\n";
    }

    // Read Modes of operation
    // int8_t modesOfOperation;
    // if (VCS_GetObject(g_pKeyHandle, g_usNodeId, 0x6060, 0x00, &modesOfOperation, sizeof(modesOfOperation), &NbOfBytesRead, p_pErrorCode) == 0) {
    //     LogError("VCS_GetObject", lResult, *p_pErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "modesOfOperation: " << (int)modesOfOperation << "\n";
    // }

    // // Modes of operation display
    // int8_t modesOfOperationDisplay;
    // if (VCS_GetObject(g_pKeyHandle, g_usNodeId, 0x6061, 0x00, &modesOfOperationDisplay, sizeof(modesOfOperationDisplay), &NbOfBytesRead, p_pErrorCode) == 0) {
    //     LogError("VCS_GetObject", lResult, *p_pErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "modesOfOperationDisplay: " << (int)modesOfOperationDisplay << "\n";
    // }

    // // Set Modes of operation to 10 (CST)
    // // int  VCS_SetObject(void* KeyHandle, unsigned short NodeId, unsigned short ObjectIndex, unsigned char ObjectSubIndex, void* pData, unsigned int NbOfBytesToWrite, unsigned int* pNbOfBytesWritten, unsigned int* pErrorCode);
    // desiredModesOfOperation = 10;
    // if (VCS_SetObject(g_pKeyHandle, g_usNodeId, 0x6060, 0x00, &desiredModesOfOperation, sizeof(desiredModesOfOperation), &NbOfBytesWritten, p_pErrorCode) == 0) {
    //     LogError("VCS_SetObject", lResult, *p_pErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "Successfully write " << (int)desiredModesOfOperation << " to Modes of operation\n";
    //     if (VCS_GetObject(g_pKeyHandle, g_usNodeId, 0x6060, 0x00, &modesOfOperation, sizeof(modesOfOperation), &NbOfBytesRead, p_pErrorCode) == 0) {
    //         LogError("VCS_GetObject", lResult, *p_pErrorCode);
    //         lResult = MMC_FAILED;
    //     } else {
    //         cout << "modesOfOperation: " << (int)modesOfOperation << "\n";
    //     }

    //     if (VCS_GetObject(g_pKeyHandle, g_usNodeId, 0x6061, 0x00, &modesOfOperationDisplay, sizeof(modesOfOperationDisplay), &NbOfBytesRead, p_pErrorCode) == 0) {
    //         LogError("VCS_GetObject", lResult, *p_pErrorCode);
    //         lResult = MMC_FAILED;
    //     } else {
    //         cout << "modesOfOperationDisplay: " << (int)modesOfOperationDisplay << "\n";
    //     }
    // }

    // Set Modes of operation to 8 (CSP)
    // desiredModesOfOperation = 8;
    // if (VCS_SetObject(g_pKeyHandle, g_usNodeId, 0x6060, 0x00, &desiredModesOfOperation, sizeof(desiredModesOfOperation), &NbOfBytesWritten, p_pErrorCode) == 0) {
    //     LogError("VCS_SetObject", lResult, *p_pErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "Successfully write " << (int)desiredModesOfOperation << " to Modes of operation\n";
    //     char operationMode;
    //     if(VCS_GetOperationMode(g_pKeyHandle, g_usNodeId, &operationMode, p_pErrorCode) == 0) {
    //         LogError("VCS_GetOperationMode", lResult, *p_pErrorCode);
    //         lResult = MMC_FAILED;
    //     } else {
    //         cout << "operationMode: " << (int)operationMode << "\n";
    //     }
    // }


    // Read motor rated torque
    // uint32_t motorRatedTorque;
    // if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6076, 0x00, &motorRatedTorque, 4, &NbOfBytesRead, &p_rlErrorCode) == 0) {
    //     LogError("VCS_GetObject", lResult, p_rlErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "Motor Rated Torque (0x6076): " << motorRatedTorque << "\n";
    // }

    // // Read current controller P gain
    // uint32_t currentControllerPGain;
    // if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x30A0, 0x01, &currentControllerPGain, sizeof(currentControllerPGain), &NbOfBytesRead, &p_rlErrorCode) == 0) {
    //     LogError("VCS_GetObject", lResult, p_rlErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "currentControllerPGain: " << currentControllerPGain << "\n";
    // }

    // // Read current controller I gain
    // uint32_t currentControllerIGain;
    // if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x30A0, 0x02, &currentControllerIGain, sizeof(currentControllerIGain), &NbOfBytesRead, &p_rlErrorCode) == 0) {
    //     LogError("VCS_GetObject", lResult, p_rlErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "currentControllerIGain: " << currentControllerIGain << "\n";
    // }

    // // Read position controller P gain
    // uint32_t positionControllerPGain;
    // if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x30A1, 0x01, &positionControllerPGain, sizeof(positionControllerPGain), &NbOfBytesRead, &p_rlErrorCode) == 0) {
    //     LogError("VCS_GetObject", lResult, p_rlErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "positionControllerPGain: " << positionControllerPGain << "\n";
    // }

    // // Read position controller I gain
    // uint32_t positionControllerIGain;
    // if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x30A1, 0x02, &positionControllerIGain, sizeof(positionControllerIGain), &NbOfBytesRead, &p_rlErrorCode) == 0) {
    //     LogError("VCS_GetObject", lResult, p_rlErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "positionControllerIGain: " << positionControllerIGain << "\n";
    // }

    // // Read position controller D gain
    // uint32_t positionControllerDGain;
    // if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x30A1, 0x03, &positionControllerDGain, sizeof(positionControllerDGain), &NbOfBytesRead, &p_rlErrorCode) == 0) {
    //     LogError("VCS_GetObject", lResult, p_rlErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "positionControllerDGain: " << positionControllerDGain << "\n";
    // }

    // // Read position controller FF velocity gain
    // uint32_t positionControllerFFVelocityGain;
    // if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x30A1, 0x04, &positionControllerFFVelocityGain, sizeof(positionControllerFFVelocityGain), &NbOfBytesRead, &p_rlErrorCode) == 0) {
    //     LogError("VCS_GetObject", lResult, p_rlErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "positionControllerFFVelocityGain: " << positionControllerFFVelocityGain << "\n";
    // }

    // // Read position controller FF acceleration gain
    // uint32_t positionControllerFFAccelerationGain;
    // if (VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x30A1, 0x05, &positionControllerFFAccelerationGain, sizeof(positionControllerFFAccelerationGain), &NbOfBytesRead, &p_rlErrorCode) == 0) {
    //     LogError("VCS_GetObject", lResult, p_rlErrorCode);
    //     lResult = MMC_FAILED;
    // } else {
    //     cout << "positionControllerFFAccelerationGain: " << positionControllerFFAccelerationGain << "\n";
    // }

    int num_steps = 10;
    double angleDeltaInDegree = 20;
    double payloadWeight = 1.03;
    bool hasGravityCompensation = false;

    // Move to new angle
    for (int i = 0; i < num_steps; i++) {
        double angleIncrementalInDegree = angleDeltaInDegree / num_steps;

        if(TestGravityCompensationForRodAndPayload(g_pKeyHandle, g_usNodeId, hasGravityCompensation, payloadWeight, angleIncrementalInDegree, lErrorCode) != MMC_SUCCESS) {
            LogError("TestGravityCompensationForRodAndPayload", lResult, lErrorCode);
        }

        // Wait a bit so we can see what's going on
        usleep(0.5 * 1000 * 1000);
    }

    // Move back to original angle
    for (int i = 0; i < num_steps; i++) {
        double angleIncrementalInDegree = -angleDeltaInDegree / num_steps;

        if(TestGravityCompensationForRodAndPayload(g_pKeyHandle, g_usNodeId, hasGravityCompensation, payloadWeight, angleIncrementalInDegree, lErrorCode) != MMC_SUCCESS) {
            LogError("TestGravityCompensationForRodAndPayload", lResult, lErrorCode);
        }

        // Wait a bit so we can see what's going on
        usleep(0.5 * 1000 * 1000);
    }

    if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0) {
        LogError("VCS_SetDisableState", lResult, lErrorCode);
        lResult = MMC_FAILED;
    } 

    // lResult = TestOverwriteTorqueOffsetWhenInMotion(g_pKeyHandle, g_usNodeId, lErrorCode);

    // if(lResult != MMC_SUCCESS)
    // {
    //     LogError("TestOverwriteTorqueOffsetWhenInMotion", lResult, lErrorCode);
    // }
    // else
    // {
    //     if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
    //     {
    //         LogError("VCS_SetDisableState", lResult, lErrorCode);
    //         lResult = MMC_FAILED;
    //     }
    // }

    return lResult;
}

void PrintHeader()
{
    SeparatorLine();

    LogInfo("Epos Command Library Example Program, (c) maxonmotor ag 2014-2017");

    SeparatorLine();
}

int PrintAvailablePorts(char* p_pInterfaceNameSel)
{
    int lResult = MMC_FAILED;
    int lStartOfSelection = 1;
    int lMaxStrSize = 255;
    char* pPortNameSel = new char[lMaxStrSize];
    int lEndOfSelection = 0;
    unsigned int ulErrorCode = 0;

    do
    {
        if(!VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), p_pInterfaceNameSel, lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
        {
            lResult = MMC_FAILED;
            LogError("GetPortNameSelection", lResult, ulErrorCode);
            break;
        }
        else
        {
            lResult = MMC_SUCCESS;
            printf("            port = %s\n", pPortNameSel);
        }

        lStartOfSelection = 0;
    }
    while(lEndOfSelection == 0);

    return lResult;
}

int PrintAvailableInterfaces()
{
    int lResult = MMC_FAILED;
    int lStartOfSelection = 1;
    int lMaxStrSize = 255;
    char* pInterfaceNameSel = new char[lMaxStrSize];
    int lEndOfSelection = 0;
    unsigned int ulErrorCode = 0;

    do
    {
        if(!VCS_GetInterfaceNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), lStartOfSelection, pInterfaceNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
        {
            lResult = MMC_FAILED;
            LogError("GetInterfaceNameSelection", lResult, ulErrorCode);
            break;
        }
        else
        {
            lResult = MMC_SUCCESS;

            printf("interface = %s\n", pInterfaceNameSel);

            PrintAvailablePorts(pInterfaceNameSel);
        }

        lStartOfSelection = 0;
    }
    while(lEndOfSelection == 0);

    SeparatorLine();

    delete[] pInterfaceNameSel;

    return lResult;
}

int PrintDeviceVersion()
{
    int lResult = MMC_FAILED;
    unsigned short usHardwareVersion = 0;
    unsigned short usSoftwareVersion = 0;
    unsigned short usApplicationNumber = 0;
    unsigned short usApplicationVersion = 0;
    unsigned int ulErrorCode = 0;

    if(VCS_GetVersion(g_pKeyHandle, g_usNodeId, &usHardwareVersion, &usSoftwareVersion, &usApplicationNumber, &usApplicationVersion, &ulErrorCode))
    {
        printf("%s Hardware Version    = 0x%04x\n      Software Version    = 0x%04x\n      Application Number  = 0x%04x\n      Application Version = 0x%04x\n",
                g_deviceName.c_str(), usHardwareVersion, usSoftwareVersion, usApplicationNumber, usApplicationVersion);
        lResult = MMC_SUCCESS;
    }

    return lResult;
}

int PrintAvailableProtocols()
{
    int lResult = MMC_FAILED;
    int lStartOfSelection = 1;
    int lMaxStrSize = 255;
    char* pProtocolNameSel = new char[lMaxStrSize];
    int lEndOfSelection = 0;
    unsigned int ulErrorCode = 0;

    do
    {
        if(!VCS_GetProtocolStackNameSelection((char*)g_deviceName.c_str(), lStartOfSelection, pProtocolNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
        {
            lResult = MMC_FAILED;
            LogError("GetProtocolStackNameSelection", lResult, ulErrorCode);
            break;
        }
        else
        {
            lResult = MMC_SUCCESS;

            printf("protocol stack name = %s\n", pProtocolNameSel);
        }

        lStartOfSelection = 0;
    }
    while(lEndOfSelection == 0);

    SeparatorLine();

    delete[] pProtocolNameSel;

    return lResult;
}

int main(int argc, char** argv)
{
    int lResult = MMC_FAILED;
    unsigned int ulErrorCode = 0;

    PrintHeader();

    SetDefaultParameters();

    if((lResult = ParseArguments(argc, argv))!=MMC_SUCCESS)
    {
        return lResult;
    }

    PrintSettings();

    switch(g_eAppMode)
    {
        case AM_DEMO:
        {
            if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("OpenDevice", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("PrepareDemo", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = Demo(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("Demo", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("CloseDevice", lResult, ulErrorCode);
                return lResult;
            }
        } break;
        case AM_INTERFACE_LIST:
            PrintAvailableInterfaces();
            break;
        case AM_PROTOCOL_LIST:
            PrintAvailableProtocols();
            break;
        case AM_VERSION_INFO:
        {
            if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("OpenDevice", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = PrintDeviceVersion()) != MMC_SUCCESS)
            {
                LogError("PrintDeviceVersion", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("CloseDevice", lResult, ulErrorCode);
                return lResult;
            }
        } break;
        case AM_UNKNOWN:
            printf("unknown option\n");
            break;
    }

    return lResult;
}
