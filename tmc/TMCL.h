/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#ifndef TMCL_H
#define TMCL_H

#include "tmc/helpers/API_Header.h"

#define	BL_MAGIC_VALUE_BL_NEW		0x43215678
#define BL_MAGIC_VALUE_APP_NEW		0x12348765
#define BL_MAGIC_VALUE_OLD			0x12345678

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall) || defined(LandungsbrueckeV3)
    struct BootloaderConfig {
        uint32_t BLMagic;
        uint32_t drvEnableResetValue;
    };
#endif
void tmcl_init();
void tmcl_process();
void tmcl_boot();

#endif /* TMCL_H */
