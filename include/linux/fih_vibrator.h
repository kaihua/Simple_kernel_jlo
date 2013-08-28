#ifndef	FIH_VIBRATOR_DRIVER_H
#define	FIH_VIBRATOR_DRIVER_H

#include <linux/vibrator_class.h>

struct	vib_device_data
{
	char		*name;
	unsigned int	level;
};

struct	vib_driver_data
{
	struct vibrator_classdev	classdev;
	unsigned int	level;
};

#endif