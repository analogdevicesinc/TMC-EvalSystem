/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/
/*
 * This is an empty placeholder file. It is copied by make if git version
 * information is disabled.
 *
 * To enable git version information in the build, you must:
 * - Have git installed and available - Either:
 *     - Have "git" in your PATH for the Makefile to call
 *     - Set the $GIT environment variable for the Makefile to use instead of "git"
 * - Have python 3 installed and available - either:
 *     - Have "python" in your PATH for the Makefile to call
 *     - Set the $PYTHON environment variable for the Makefile to use instead of "python"
 * - Pass ENABLE_GIT_VERSION_INFO=1 to make
 */

#ifndef GIT_INFO_H_
#define GIT_INFO_H_

// This value is invalid - it is used by C code to identify that
// no git information is available.
#define GIT_VERSION_INFO 0xFFFFFFFF

#endif /* GIT_INFO_H_ */
