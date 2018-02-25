#ifndef FIMC_IS_VENDOR_CONFIG_JACKPOT_H
#define FIMC_IS_VENDOR_CONFIG_JACKPOT_H

#include "fimc-is-eeprom-rear-4h5yc_v001.h"
#include "fimc-is-otprom-front-5e3_v001.h"


/* This count is defined by count of fimc_is_sensor in the dtsi file */
#define FIMC_IS_HW_SENSOR_COUNT 2

#define VENDER_PATH

#define CAMERA_MODULE_ES_VERSION_REAR 'A'
#define CAMERA_MODULE_ES_VERSION_FRONT 'A'
#define CAL_MAP_ES_VERSION_REAR '1'
#define CAL_MAP_ES_VERSION_FRONT '1'  /* VF01 version */

#define CAMERA_SYSFS_V2
#define USE_COMMON_CAM_IO_PWR
//#define USE_COMMON_FRONT_CAL_MAP

#define EEPROM_DEBUG

#define USE_CAMERA_HW_BIG_DATA
#ifdef USE_CAMERA_HW_BIG_DATA
/* #define USE_CAMERA_HW_BIG_DATA_FOR_PANIC */
#define CSI_SCENARIO_SEN_REAR	(0) /* This value follows dtsi */
#define CSI_SCENARIO_SEN_FRONT	(1)
#endif

#endif /* FIMC_IS_VENDOR_CONFIG_JACKPOT_H */
