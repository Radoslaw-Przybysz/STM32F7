
 #ifndef _modbus_h_
 #define _modbus_h_

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include <stdint.h>

/*!===============================================================
 *  global definitions */

 #define _modbusNewValue    	1

/*!===============================================================
 *  configuration flags */

 #define _modbusInited       	1

 //#define COILREAD			//! enable coils read  
 //#define COILWRITE		//! enable coil write
 //#define COILMULWRITE	//! enable multiple coils write
 #define HOLDREAD			//! enable holding registers read
 #define REGREAD			//! enable registers read
 #define REGWRITE			//! enable register write
 #define REGMULWRITE	//! enable multiple registers write
 //#define INPUTSREAD		//! enable inputs read

/*!===============================================================
 *  public data  */

 typedef struct
 {
  uint16_t dat;                                                          
  uint16_t updateFlag;
 } modbusRegister;

 typedef struct
 {

  modbusRegister *registersAddress;   //! registers pointer
  uint16_t  registersCount;            //! registers count
  uint16_t  registersBase;             //! registers address using modbus protocol addressing

 } modbusRegisters;

 typedef struct
 {
 
 #if defined(COILREAD) || defined(COILWRITE) || defined(COILMULWRITE)
  uint8_t  *coils;
  uint16_t  coilsCount;
 #endif
 
 #ifdef HOLDREAD
  modbusRegisters *holdingRegisters;
  uint16_t holdingStructCount;
 #endif
 
 #ifdef INPUTSREAD
  uint8_t  *inputs;
  uint16_t  inputsCount;
 #endif
 
 #if defined(REGREAD) || defined(REGWRITE) || defined(REGMULWRITE)
  modbusRegisters *registers;
  uint16_t registersStructCount;
 #endif

  uint8_t  addr;
  uint8_t  config;

  uint32_t frameTotalCount;
  uint32_t frameErrCount;

  int8_t (*ReadFrame)( uint8_t *dat, uint16_t *length );
  int8_t (*SendFrame)( uint8_t *dat, int16_t length );
  int8_t (*IsFrame)(void);
  
 } modbusProtocol;

 /*!===============================================================
 *  public functions  */

 extern void ModbusThread( modbusProtocol *modbus );
 extern void ModbusDecodeMessage( modbusProtocol *modbus );

#ifdef __cplusplus
}
#endif

#endif
 
//! end

