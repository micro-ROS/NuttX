#ifndef __INCLUDE_NUTTX_INPUT_JOYSTICK_H
#define __INCLUDE_NUTTX_INPUT_JOYSTICK_H

#ifdef __cplusplus
extern "C"
{
#endif
/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This type defines the button data report from the controller. */

struct joystick_buttonstate_s
{
	int8_t joy_x;
	int8_t joy_y;
	uint8_t dir_button;
	uint8_t joy_z;
	uint8_t unknown_0;
	uint8_t throttle;
	uint8_t unknown_1;
};

/* The supported IOCTL commands. */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_JOYSTICK_H */

