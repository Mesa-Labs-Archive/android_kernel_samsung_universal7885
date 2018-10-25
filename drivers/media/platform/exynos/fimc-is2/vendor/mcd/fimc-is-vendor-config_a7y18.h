#ifndef FIMC_IS_VENDOR_CONFIG_A7Y18_H
#define FIMC_IS_VENDOR_CONFIG_A7Y18_H

/***** HISTORY *****
 * < HW_REV 00 >
 *  rear eeprom version v001 : fimc-is-eeprom-rear-imx576_v001.h
 *  CAL_MAP_ES_VERSION_REAR  : 1
 *
 * <HW_REV 01 >
 *  rear eeprom version v002 : fimc-is-eeprom-rear-imx576_v002.h
 *  CAL_MAP_ES_VERSION_REAR  : 2
 */

#include "fimc-is-eeprom-rear-imx576_v001.h"
//#include "fimc-is-eeprom-rear-4ha_v001.h"
#include "fimc-is-eeprom-front-imx576_v001.h"

#define VENDER_PATH

#define CAMERA_MODULE_ES_VERSION_REAR 'A'
#define CAMERA_MODULE_ES_VERSION_FRONT 'A'
#define CAL_MAP_ES_VERSION_REAR '1'
#define CAL_MAP_ES_VERSION_FRONT '1'

#define FIMC_IS_HW_SENSOR_COUNT 4

#define CAMERA_SYSFS_V2

//#define CAMERA_REAR2				/* Support Rear2 for Dual Camera */
#define CAMERA_REAR3				/* Support Rear3 */

#define SAMSUNG_LIVE_OUTFOCUS		/* Allocate memory For Dual Camera */
#define ENABLE_REMOSAIC_CAPTURE

#define USE_COMMON_CAM_IO_PWR

#define USE_SSRM_CAMERA_INFO		/* Match with SAMSUNG_SSRM define of Camera Hal side */

#define EEPROM_DEBUG
#define SKIP_CHECK_CRC				/* Skip the EEPROM CAL DATA CRC CHECK */

#define USE_CAMERA_HW_BIG_DATA
#ifdef USE_CAMERA_HW_BIG_DATA
/* #define USE_CAMERA_HW_BIG_DATA_FOR_PANIC */
#define CSI_SCENARIO_SEN_REAR	(0) /* This value follows dtsi */
#define CSI_SCENARIO_SEN_FRONT	(1)
#endif

/* VRA 1.4 improvement - adding VRA 1.4 interface : move from fimc-is-config.h */
/* Be enable this feature for New Model since A7 2018 */
#define ENABLE_VRA_LIBRARY_IMPROVE

#endif /* FIMC_IS_VENDOR_CONFIG_A7Y18_H */
