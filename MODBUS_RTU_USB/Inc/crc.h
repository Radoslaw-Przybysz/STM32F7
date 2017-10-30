 #ifndef _crc_h_
 #define _crc_h_

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include <stdint.h>
	 
/*!===============================================================
 *  global definitions */

/*!===============================================================
 *  configuration flags */

/*!===============================================================
 *  public data  */

 /*!===============================================================
 *  public functions  */

 extern uint16_t CRCcalculate( uint8_t *puchMsg, uint16_t usDataLen );

#ifdef __cplusplus
}
#endif

#endif

//end

