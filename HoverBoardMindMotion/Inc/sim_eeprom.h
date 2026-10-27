////////////////////////////////////////////////////////////////////////////////
/// @file    sim_eeprom.h
/// @author  AE TEAM
/// @brief   THIS FILE PROVIDES ALL THE SYSTEM FIRMWARE FUNCTIONS.
////////////////////////////////////////////////////////////////////////////////

// Define to prevent recursive inclusion
#ifndef __SIM_EEPROM_H
#define __SIM_EEPROM_H

// Files includes
#include <string.h>

#include "mcu.h"                // device header of the selected MCU
#include "hal_conf.h"





////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Exported_Variables
/// @{
#ifdef _SIM_EEPROM_C_
#define GLOBAL
#else
#define GLOBAL extern
#endif
#undef GLOBAL

/// @}


////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Exported_Functions
/// @{
s32 EEPROM_Write(u8* ptr, u16 len);
uint8_t ConfigLoad(void);     //newest record with a valid magic -> pinstorage (+ extension record)
uint8_t restorecfg(void);     //ConfigLoad() and the hall + uart pins are set
void ConfigMigrate(void);     //older layouts (CFG_VERSION) to the current one
void SaveConfig(void);    //extension record, then the main record
/// @}


/// @}

/// @}


////////////////////////////////////////////////////////////////////////////////
#endif
////////////////////////////////////////////////////////////////////////////////
