#ifndef	FIH_VIBRATOR_CLSAA_H
#define	FIH_VIBRATOR_CLSAA_H

#include <linux/hrtimer.h>
#include <linux/workqueue.h>

#define	CLASS_NOT_EXIST	0
#define	CLASS_EXIST	1

#define	VIBRATE_OFF	0
#define	VIBRATE_60_SEC	( 60 * 1000 )
#define	VIBRATE_ALWAYS	-1

#define	TIME_OUT_30_MS	30

#define	TURN_OFF		0
#define	TURN_ON		1

#define	VIBRATE_SEC( time )	( time / 1000 )
#define	VIBRATE_NS( time )	( ( time % 1000 ) * 1000000 )

// Internal use
struct	vibrator_class
{
	struct class		*vibrator;
	struct workqueue_struct	*work_queue;
	atomic_t			count;
	unsigned int		exist : 1;
};

struct	vibrator_classdev
{
	const char		*name;
	struct device		*dev;
	struct hrtimer		vibrate_time;
	struct work_struct	work_on;
	struct work_struct	work_off;
	atomic_t			turn_off;
	unsigned int		turn_off_time;
	int			index;

	void	( *control )( struct vibrator_classdev*, int );
	void	( *level )( struct vibrator_classdev*, int );
};

// Internal use
struct	allocate_device_attribute
{
	char	*name;
	struct device_attribute	*attribute;
	void ( **function )( struct vibrator_classdev*, int );
};

int vibrator_classdev_register( struct vibrator_classdev* );
void vibrator_classdev_unregister( struct vibrator_classdev* );

#endif