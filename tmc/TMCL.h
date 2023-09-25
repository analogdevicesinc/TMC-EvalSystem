/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#ifndef TMCL_H
#define TMCL_H

#include "tmc/helpers/API_Header.h"

void tmcl_init();
void tmcl_process();
void tmcl_boot();

#endif /* TMCL_H */
