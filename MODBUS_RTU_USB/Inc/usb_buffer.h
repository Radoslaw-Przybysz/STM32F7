
 #ifndef _usb_buffer_h_
 #define _usb_buffer_h_

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include <stdint.h>

/*!===============================================================
 *  global definitions */

	 #define _MaxRxBuffersCount    	3
	 #define _RxBufferSize    			64
	 
/*!===============================================================
 *  configuration flags */

/*!===============================================================
 *  public data  */

 typedef struct
{
    int8_t pos_receive; //!< pos_receive is the current position in buffer to save incoming data.
	  int8_t pos_process; //!< pos_process is the index of data in buffer which has been processed.
												//   if pos_receive=pos_process, it means all data were processed, waiting for new data coming

    uint8_t RxBuffers[_MaxRxBuffersCount][_RxBufferSize];		//!< it could save <MaxCommandsInBuffer> number of commands
    uint8_t RxBuffersLength[_MaxRxBuffersCount]; 						//!< save the length of each package
	
} s_USBBuffer;

extern s_USBBuffer _usbBuffers;

 /*!===============================================================
 *  public functions  */

#ifdef __cplusplus
}
#endif

#endif
 
//! end

