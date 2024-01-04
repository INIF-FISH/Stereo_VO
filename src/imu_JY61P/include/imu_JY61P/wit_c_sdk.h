#ifndef __WIT_C_SDK_H
#define __WIT_C_SDK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "REG.h"

#define WIT_HAL_OK (0)
#define WIT_HAL_BUSY (-1)
#define WIT_HAL_TIMEOUT (-2)
#define WIT_HAL_ERROR (-3)
#define WIT_HAL_NOMEM (-4)
#define WIT_HAL_EMPTY (-5)
#define WIT_HAL_INVAL (-6)

#define WIT_DATA_BUFF_SIZE 256

#define WIT_PROTOCOL_NORMAL 0
#define WIT_PROTOCOL_MODBUS 1
#define WIT_PROTOCOL_CAN 2
#define WIT_PROTOCOL_I2C 3
#define WIT_PROTOCOL_JY61 4
#define WIT_PROTOCOL_905x_MODBUS 5
#define WIT_PROTOCOL_905x_CAN 6

    typedef void (*SerialWrite)(uint8_t *p_ucData, uint32_t uiLen);
    int32_t WitSerialWriteRegister(SerialWrite write_func);
    void WitSerialDataIn(uint8_t ucData);

    typedef int32_t (*WitI2cWrite)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

    typedef int32_t (*WitI2cRead)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
    int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func);

    typedef void (*CanWrite)(uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen);
    int32_t WitCanWriteRegister(CanWrite write_func);

    typedef void (*DelaymsCb)(uint16_t ucMs);
    int32_t WitDelayMsRegister(DelaymsCb delayms_func);

    void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen);

    typedef void (*RegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);
    int32_t WitRegisterCallBack(RegUpdateCb update_func);
    int32_t WitWriteReg(uint32_t uiReg, uint16_t usData);
    int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum);
    int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr);
    void WitDeInit(void);

    int32_t WitStartAccCali(void);
    int32_t WitStopAccCali(void);
    int32_t WitStartMagCali(void);
    int32_t WitStopMagCali(void);
    int32_t WitSetUartBaud(int32_t uiBaudIndex);
    int32_t WitSetBandwidth(int32_t uiBaudWidth);
    int32_t WitSetOutputRate(int32_t uiRate);
    int32_t WitSetContent(int32_t uiRsw);
    int32_t WitSetCanBaud(int32_t uiBaudIndex);
    int32_t WitSaveParameter(void);
    int32_t WitSetForReset(void);
    int32_t WitCaliRefAngle(void);

    char CheckRange(short sTemp, short sMin, short sMax);

    extern int16_t sReg[REGSIZE];

#ifdef __cplusplus
}
#endif

#endif /* __WIT_C_SDK_H */
