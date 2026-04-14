/*
 * measurement_config.h
 *
 *  Created on: Mar 7, 2026
 *      Author: aleks
 */

#ifndef INC_MEASUREMENT_CONFIG_H_
#define INC_MEASUREMENT_CONFIG_H_

/*
 * measurement_config.c
 *
 *  Created on: Mar 7, 2026
 *      Author: aleks
 */

#include <invn/icu_interface/shasta_external_regs.h>
#include <invn/icu_interface/ch-rangefinder/structs.h>  // algorithm specific definitions
#include <invn/soniclib/soniclib.h>

extern measurement_queue_t measurement_config_queue;

extern InvnAlgoRangeFinderConfig measurement_config_cfg;



#endif /* INC_MEASUREMENT_CONFIG_H_ */
