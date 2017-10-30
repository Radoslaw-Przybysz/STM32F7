
#include "crc.h"
#include "modbus.h"

/*!
* \file
*/

/*!===============================================================
 *  public functions  */

void ModbusThread( modbusProtocol *modbus );
void ModbusDecodeMessage( modbusProtocol *modbus );

/*!===============================================================
 *  public data  */

/*!===============================================================
 *  private functions  */

#ifdef COILREAD
static int8_t ModbusCoilsRead( modbusProtocol *modbus, uint8_t *buff );
#endif
#ifdef COILWRITE
static int8_t ModbusCoilWrite( modbusProtocol *modbus, uint8_t *buff );
#endif
#ifdef COILMULWRITE
static int8_t ModbusForceMultipleCoils( modbusProtocol *modbus, uint8_t *buff );
#endif
#ifdef HOLDREAD
static int8_t ModbusHoldingRegistersRead( modbusProtocol *modbus, uint8_t *buff );
#endif
#ifdef INPUTSREAD
static int8_t ModbusInputsRead( modbusProtocol *modbus, uint8_t *buff  );
#endif
#ifdef REGREAD
static int8_t ModbusRegistersRead( modbusProtocol *modbus, uint8_t *buff  );
#endif
#ifdef REGWRITE
static int8_t ModbusRegistersWrite( modbusProtocol *modbus, uint8_t *buff );
#endif
#ifdef REGMULWRITE
static int8_t ModbusRegistersMultipleWrite( modbusProtocol *modbus, uint8_t *buff );
#endif

static int8_t  ModbusException( modbusProtocol *modbus, uint8_t exception );
static int16_t  ModbusReadFrame( modbusProtocol *modbus, uint8_t *dat, uint16_t *length );
static int16_t  ModbusSendFrame( modbusProtocol *modbus, uint8_t *dat, uint16_t length );

/*!===============================================================
 *  private data  */

/*!===============================================================
 *  file code */

/*!
 * @brief      Function for detect and respond.
 * @param[in]  modbus 	 	 :
 */
void ModbusThread( modbusProtocol *modbus )
{

    if( modbus->IsFrame() )
        ModbusDecodeMessage(modbus);

}

/*!
 * @brief      Function decode modbus frame and send respond.
 * @param[in]  modbus 	 	 :
*/
void ModbusDecodeMessage( modbusProtocol *modbus )
{

    uint8_t buff[256];
    uint16_t length = 0;

    if( ModbusReadFrame( modbus, buff, &length ) < 0 )
        return;

    if( buff[0] == modbus->addr )
    {

        if( modbus->frameTotalCount < 0xFFFFFFFF)
            modbus->frameTotalCount++;

        switch( buff[1] )
        {
#ifdef COILREAD
        case 0x01:
            ModbusCoilsRead(modbus,buff);
            break;
#endif
#ifdef INPUTSREAD
        case 0x02:
            ModbusInputsRead(modbus,buff);
            break;
#endif
#ifdef HOLDREAD
        case 0x03:
            ModbusHoldingRegistersRead(modbus,buff);
            break;
#endif
#ifdef REGREAD
        case 0x04:
            ModbusRegistersRead(modbus,buff);
            break;
#endif
#ifdef COILWRITE
        case 0x05:
            ModbusCoilWrite(modbus,buff);
            break;
#endif
#ifdef REGWRITE
        case 0x06:
            ModbusRegistersWrite(modbus,buff);
            break;
#endif
#ifdef COILMULWRITE
        case 0x0F:
            ModbusForceMultipleCoils(modbus,buff);
            break;
#endif
#ifdef REGMULWRITE
        case 0x10:
            ModbusRegistersMultipleWrite(modbus,buff);
            break;
#endif
        default:
            ModbusException( modbus, 0x01 );
            break;
        }

    }

}

#ifdef COILREAD
/*!
 * @brief      Function decode command "Coils read".
 * @param[in]  modbus 	 :
 * @param[in]  buff 	 : frame data
*/
static int8_t ModbusCoilsRead(  modbusProtocol *modbus, uint8_t *buff )
{

    uint16_t count = ((uint16_t)buff[4] << 8) | buff[5];
    uint16_t begin = ((uint16_t)buff[2] << 8) | buff[3];
    uint16_t bytes = 1;
    uint8_t *tmp1,*tmp2;
    uint16_t tmp0, tmp3;

    if( ( begin + count ) > modbus->coilsCount || count == 0 )
        return ModbusException( modbus, 0x02 );

    tmp2 = &buff[3];
    tmp3 = 0;

    while( count-- )
    {

        tmp0 = begin;
        tmp1 = modbus->coils;

        while( tmp0 > 7 )
        {
            tmp0 -= 8;
            tmp1 += 1;
        }

        if( ( *tmp1 & (1<<tmp0)) != 0 )
            *tmp2 |= 1<<tmp0;
        else
            *tmp2 &= ~(1<<tmp0);

        if( ++tmp3 == 8 )
        {
            *(++tmp2)=0;
            bytes += count != 0 ? 1:0;
            tmp3=0;
        }

        begin++;

    }

    buff[2] = bytes;
    return ModbusSendFrame( modbus, buff, bytes + 3  );

}
#endif

#ifdef COILWRITE
/*!
 * @brief      Function decode command "Force coil".
 * @param[in]  modbus 	 :
 * @param[in]  buff 	 : frame data
*/
static int8_t ModbusCoilWrite( modbusProtocol *modbus, uint8_t *buff )
{

    uint16_t state = ((uint16_t)buff[4] << 8) | buff[5];
    uint16_t begin = ((uint16_t)buff[2] << 8) | buff[3];
    uint8_t *tmp1;

    tmp1 = modbus->coils;

    if( begin >= modbus->coilsCount )
        return ModbusException( modbus, 0x02 );

    while( begin > 7 )
    {
        begin -= 8;
        tmp1++;
    }

    if( state == 0xFF00 )
        *tmp1 |= 1<<begin;
    else if( state == 0x0000 )
        *tmp1 &= ~(1<<begin);
    else
        return ModbusException( modbus, 0x02 );

    return  ModbusSendFrame( modbus, buff, 6 );

}
#endif

/*!
 * @brief      Function send modbus exception frame.
 * @param[in]  modbus 	 :
 * @param[in]  exception : value from 0 -:- 255
*/
static int8_t ModbusException( modbusProtocol *modbus, uint8_t exception )
{

    ModbusSendFrame( modbus, (uint8_t[])
    {
        modbus->addr, 0x84, exception,0,0,0,0,0
    }, 3 );

    if( modbus->frameErrCount < 0xFFFFFFFF)
        modbus->frameErrCount++;

    return -1;

}

#ifdef COILMULWRITE
/*!
 * @brief      Function decode command "Force multiple coils".
 * @param[in]  modbus 	 :
 * @param[in]  buff 	 : frame data
*/
static int8_t ModbusForceMultipleCoils( modbusProtocol *modbus, uint8_t *buff  )
{

    uint16_t count = ((uint16_t)buff[4] << 8) | buff[5];
    uint16_t begin = ((uint16_t)buff[2] << 8) | buff[3];
    uint8_t *tmp1,*tmp2;
    uint16_t tmp0, tmp3;

    tmp2 = &buff[7];
    tmp3 = 0;

    if( (begin + count) > modbus->coilsCount )
        return ModbusException( modbus, 0x02 );

    if( ( (count>>3) + 1 ) != buff[6] )
        return ModbusException( modbus, 0x03 );

    while( count-- )
    {

        tmp0 = begin;
        tmp1 = modbus->coils;

        while( tmp0 > 7 )
        {
            tmp0 -= 8;
            tmp1++;
        }

        if( (*tmp2 & (1 << tmp3)) == 0 )
            *tmp1 &= ~(1<<tmp0);
        else
            *tmp1 |= 1<<tmp0;

        if( ++tmp3 == 8)
        {
            tmp3 = 0;
            tmp2++;
        }

        begin++;

    }

    return ModbusSendFrame( modbus, buff, 6 );

}
#endif

#ifdef HOLDREAD
/*!
 * @brief      Function decode command "Read holding registers".
 * @param[in]  modbus 	 :
 * @param[in]  buff 	 : frame data
*/
static int8_t ModbusHoldingRegistersRead( modbusProtocol *modbus, uint8_t *buff )
{

    uint16_t count = ((uint16_t)buff[4] << 8) | buff[5];
    uint16_t begin = ((uint16_t)buff[2] << 8) | buff[3];
    uint16_t tmp0, tmp1, tmp2;
    uint8_t *tmp3;

    modbusRegisters *modbusRegs;
    modbusRegister *modbusReg;

    modbusRegs = modbus->holdingRegisters;
    tmp0 = modbus->holdingStructCount;
    tmp3 = &buff[3];

    while( tmp0-- )
    {

        tmp1 = modbusRegs->registersBase;
        tmp2 = modbusRegs->registersCount;

        if( (begin >= tmp1) && ( ( begin + count - 1 ) < ( tmp1 + tmp2 ) ) )
            goto _holdRead;

        modbusRegs++;

    }

    return ModbusException( modbus, 0x02 );

_holdRead:

    modbusReg = modbusRegs->registersAddress + (begin - tmp1);
    tmp2 = 0;

    while( count-- )
    {

        *tmp3++ = ( modbusReg->dat >> 8 ) & 0xFF;
        *tmp3++ = ( modbusReg->dat ) & 0xFF;

        modbusReg++;
        tmp2  += 2;

    }

    buff[2] =  tmp2;

    return ModbusSendFrame( modbus, buff, tmp2 + 3 );

}
#endif

#ifdef INPUTSREAD
/*!
 * @brief      Function decode command "Read Discrete Inputs".
 * @param[in]  modbus 	 :
 * @param[in]  buff 	 : frame data
*/
static int8_t ModbusInputsRead( modbusProtocol *modbus, uint8_t *buff  )
{

    uint16_t count = ((uint16_t)buff[4] << 8) | buff[5];
    uint16_t begin = ((uint16_t)buff[2] << 8) | buff[3];
    uint16_t bytes = 1;
    uint8_t *tmp1,*tmp2;
    uint16_t tmp0, tmp3;

    if( ( begin + count ) > modbus->inputsCount || count == 0 )
        return ModbusException( modbus, 0x02 );

    tmp2 = &buff[3];
    tmp3 = 0;

    while( count-- )
    {

        tmp0 = begin;
        tmp1 = modbus->coils;

        while( tmp0 > 7 )
        {
            tmp0 -= 8;
            tmp1 += 1;
        }

        if( ( *tmp1 & (1<<tmp0)) != 0 )
            *tmp2 |= 1<<tmp0;
        else
            *tmp2 &= ~(1<<tmp0);

        if( ++tmp3 == 8 )
        {
            *(++tmp2)=0;
            bytes += count != 0 ? 1:0;
            tmp3=0;
        }

        begin++;

    }

    buff[2] = bytes;
    return ModbusSendFrame( modbus, buff, bytes + 3  );

}
#endif

#ifdef REGREAD
/*!
 * @brief      Function decode command "Registers read".
 * @param[in]  modbus 	 :
 * @param[in]  buff 	 : frame data
*/
static int8_t ModbusRegistersRead( modbusProtocol *modbus, uint8_t *buff )
{

    uint16_t count = ((uint16_t)buff[4] << 8) | buff[5];
    uint16_t begin = ((uint16_t)buff[2] << 8) | buff[3];
    uint16_t tmp0, tmp1, tmp2;
    uint8_t *tmp3;

    modbusRegisters *modbusRegs;
    modbusRegister *modbusReg;

    modbusRegs = modbus->registers;
    tmp0 = modbus->registersStructCount;
    tmp3 = &buff[3];

    while( tmp0-- )
    {

        tmp1 = modbusRegs->registersBase;
        tmp2 = modbusRegs->registersCount;

        if( (begin >= tmp1) && ( ( begin + count - 1 ) < ( tmp1 + tmp2 ) ) )
            goto _regRead;

        modbusRegs++;

    }

    return ModbusException( modbus, 0x02 );

_regRead:

    modbusReg = modbusRegs->registersAddress + (begin - tmp1);
    tmp2 = 0;

    while( count-- )
    {

        *tmp3++ = ( modbusReg->dat >> 8 ) & 0xFF;
        *tmp3++ = ( modbusReg->dat ) & 0xFF;

        modbusReg++;
        tmp2  += 2;

    }

    buff[2] =  tmp2;

    return ModbusSendFrame( modbus, buff, tmp2 + 3 );

}
#endif

#ifdef REGWRITE
/*!
 * @brief      Function decode command "Registers write".
 * @param[in]  modbus 	 :
 * @param[in]  buff 	 : frame data
*/
static int8_t ModbusRegistersWrite( modbusProtocol *modbus, uint8_t *buff )
{

    uint16_t dat = ((uint16_t)buff[4] << 8) | buff[5];
    uint16_t begin = ((uint16_t)buff[2] << 8) | buff[3];
    uint16_t tmp0, tmp1, tmp2;
    uint8_t *tmp3;

    modbusRegisters *modbusRegs;
    modbusRegister *modbusReg;

    modbusRegs = modbus->registers;
    tmp0 = modbus->registersStructCount;
    tmp3 = &buff[3];

    while( tmp0-- )
    {

        tmp1 = modbusRegs->registersBase;
        tmp2 = modbusRegs->registersCount;

        if( (begin >= tmp1) && ( ( begin ) < ( tmp1 + tmp2 ) ) )
            goto _regWrite;

        modbusRegs++;

    }

    return ModbusException( modbus, 0x02 );

_regWrite:

    modbusReg = modbusRegs->registersAddress + ( begin - tmp1 );
    modbusReg->dat = dat;
    modbusReg->updateFlag = _modbusNewValue;

    return ModbusSendFrame( modbus, buff, 6 );

}
#endif

#ifdef REGMULWRITE
/*!
 * @brief      Function decode command "Write Multiple registers".
 * @param[in]  modbus 	 :
 * @param[in]  buff 	 : frame data
*/
static int8_t ModbusRegistersMultipleWrite( modbusProtocol *modbus, uint8_t *buff  )
{

    uint16_t count = ((uint16_t)buff[4] << 8) | buff[5];
    uint16_t begin = ((uint16_t)buff[2] << 8) | buff[3];
    uint16_t tmp0, tmp1, tmp2;
    uint8_t *tmp3;

    modbusRegisters *modbusRegs;
    modbusRegister *modbusReg;

    modbusRegs = modbus->registers;
    tmp0 = modbus->registersStructCount;
    tmp3 = &buff[7];

    while( tmp0-- )
    {

        tmp1 = modbusRegs->registersBase;
        tmp2 = modbusRegs->registersCount;

        if( (begin >= tmp1) && ( ( begin + count - 1 ) < ( tmp1 + tmp2 ) ) )
            goto _regMulWrite;

        modbusRegs++;

    }

    return ModbusException( modbus, 0x02 );

_regMulWrite:

    modbusReg = modbusRegs->registersAddress + (begin - tmp1);

    if( ( count<<1 ) != buff[6] )
        return ModbusException( modbus, 0x03 );

    while( count-- )
    {

        tmp0  = *tmp3++ << 8;
        tmp0 |= *tmp3++;

        modbusReg->dat  = tmp0;
        modbusReg->updateFlag = _modbusNewValue;
        modbusReg++;

    }

    return ModbusSendFrame( modbus, buff, 6 );

}
#endif

/*!
 * @brief      Function read modbus protocol frame using function IN/OUT.
 * @param[in]  modbus 	:
 * @param[out] dat 		: protocol frame data without CRC
 * @param[out] length	: protocol frame data length
 * @return     0 ok, -1 init error, -2 frame size error, -3 crc error
*/
static int16_t  ModbusReadFrame( modbusProtocol *modbus, uint8_t *dat, uint16_t *length )
{

    uint16_t i = 0;
    uint16_t crc;

    if( ( modbus->config & _modbusInited ) != _modbusInited )
        return -1;

    if( modbus->ReadFrame( dat, &i ) == 0 )
    {

        /*! frame length always >= 5 */
        if( i < 4 )
            return -2;

        /*! frame length without CRC */
        *length = i-2;

        /*! CRC from frame */
        crc = ((int16_t )dat[i-2] << 8 ) | dat[i-1];

        if( crc != CRCcalculate( dat, i-2 ) )
            return -3;

    }

    return 0;

}

/*!
 * @brief     Function send modbus protocol frame using function IN/OUT.
 * @param[in] modbus 	:
 * @param[in] dat 		: protocol frame data without CRC
 * @param[in] length	: protocol frame data length
 * @return    0 ok
*/
static int16_t  ModbusSendFrame( modbusProtocol *modbus, uint8_t *dat, uint16_t length )
{

    int16_t  i;

    i = CRCcalculate( dat, length );
    dat[ length ] = ( i >> 8 ) & 0xFF;
    dat[ length+1 ] = i & 0xFF;

    modbus->SendFrame( dat, length+2 );
    return 0;

}

//end
