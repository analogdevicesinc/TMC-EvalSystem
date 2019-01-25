#ifndef __WLAN_H_
	#define __WLAN_H_

	#include "RXTX.h"
	RXTXTypeDef WLAN;

	typedef enum
	{
		BUFFER_CLEAR,
		BUFFER_WRITE,
		BUFFER_EXECUTE
	} BufferCommandTypedef;

	typedef enum
	{
		WLAN_INIT_CMD_MODE,  // wait time inbetween sending $$$ and entering command mode - writing is disabled completely. rx doesn't read out data - we enter this mode only after a clearBuffer() anyways
		WLAN_CMD_MODE,       // Command mode - writing is disabled for the HAL tx function, rawTx still writes. rx doesn't read out data, rawRx does
		WLAN_DATA_MODE       // Data mode - HAL tx/rx functions works normally
	} WLANStateTypedef;

	uint32 checkReadyToSend();
	void enableWLANCommandMode();
	uint32 checkCmdModeEnabled();
	uint32 handleWLANCommand(BufferCommandTypedef cmd, uint32 value);
	uint32 getCMDReply();

#endif /* __WLAN_H_ */
