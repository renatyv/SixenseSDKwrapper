
#ifndef STRUCTDEF_H
#define STRUCTDEF_H

typedef struct _sixenseControllerData {
	float pos[3];
	float rot_mat[3][3];
	float joystick_x;
	float joystick_y;
	float trigger;
	unsigned int buttons;
	unsigned char sequence_number;
	float rot_quat[4];
	unsigned short firmware_revision;
	unsigned short hardware_revision;
	unsigned short packet_type;
	unsigned short magnetic_frequency;
	int enabled;
	int controller_index;
	unsigned char is_docked;
	unsigned char which_hand;
	unsigned char hemi_tracking_enabled;
} sixenseControllerData;

#endif