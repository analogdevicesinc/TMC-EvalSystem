/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/
#ifndef PROJECT_CONFIG_H_
#define PROJECT_CONFIG_H_

#ifndef GETINFO_RELEASE_TYPE
//#define GETINFO_RELEASE_TYPE   GETINFO_FW_RELEASE_TYPE_PUBLIC
#define GETINFO_RELEASE_TYPE   GETINFO_FW_RELEASE_TYPE_LOCAL
#endif

// Version information
// Note: These values are passed during the build process via the
// Makefile. The version information is located in the version.txt file.
#ifndef VERSION_MAJOR
#define VERSION_MAJOR 0 // <- Not used, overridden by Makefile definition!
#endif
#ifndef VERSION_MINOR
#define VERSION_MINOR 0 // <- Not used, overridden by Makefile definition!
#endif
#ifndef VERSION_PATCH
#define VERSION_PATCH 0 // <- Not used, overridden by Makefile definition!
#endif
#ifndef BUILD_VERSION
#define BUILD_VERSION 0 // <- Not used, overridden by Makefile definition!
#endif

#endif /* PROJECT_CONFIG_H_ */
