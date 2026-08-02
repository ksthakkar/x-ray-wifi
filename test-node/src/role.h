// Build-role selection.
//
// PlatformIO's generated src/CMakeLists.txt globs every file in src/, so both
// role files are always compiled. Only the one matching CSI_ROLE defines
// app_main(); the other compiles to nothing. This keeps a single project and a
// single credentials.h for both firmwares.
//
// Select the role per build environment in platformio.ini:
//     build_flags = -DCSI_ROLE=CSI_ROLE_TX
//
#pragma once

#define CSI_ROLE_RX 0
#define CSI_ROLE_TX 1

#ifndef CSI_ROLE
#define CSI_ROLE CSI_ROLE_RX
#endif
