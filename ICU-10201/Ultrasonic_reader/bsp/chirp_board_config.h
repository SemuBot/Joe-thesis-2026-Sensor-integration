#ifndef CHIRP_BOARD_CONFIG_H_
#define CHIRP_BOARD_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define INCLUDE_SHASTA_SUPPORT

#define CHIRP_MAX_NUM_SENSORS  2
#define CHIRP_NUM_BUSES        1

#define CHIRP_SENSOR_INT_PIN   1
#define CHIRP_SENSOR_TRIG_PIN  1

#define CHIRP_RTC_CAL_PULSE_MS 100

#ifdef __cplusplus
}
#endif

#endif /* CHIRP_BOARD_CONFIG_H_ */
