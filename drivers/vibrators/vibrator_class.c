
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/vibrator_class.h>

static struct vibrator_class	vib_class;

static void empty_func( struct vibrator_classdev *classdev, int state )
{
	printk( KERN_ERR "VIB : Vibrator(%s) does not set function\n", classdev->name );
}

static enum hrtimer_restart vibrator_timeout( struct hrtimer *timer )
{
	struct  vibrator_classdev	*classdev	= container_of( timer, struct  vibrator_classdev, vibrate_time );

	atomic_set( &classdev->turn_off, TURN_ON );
	queue_work( vib_class.work_queue, &classdev->work_off );
	printk( "VIB : Vibrator(%s) timeout, turn off vibrator\n", classdev->name );
	return	HRTIMER_NORESTART;
}

static void vibrator_sched_on( struct work_struct *work )
{
	struct  vibrator_classdev	*classdev	= container_of( work, struct  vibrator_classdev, work_on );

	hrtimer_cancel( &classdev->vibrate_time );

	classdev->control( classdev, TURN_ON );

	if( classdev->turn_off_time )
		hrtimer_start( &classdev->vibrate_time, ktime_set( VIBRATE_SEC( classdev->turn_off_time ), VIBRATE_NS( classdev->turn_off_time ) ), HRTIMER_MODE_REL );

}

static void vibrator_sched_off( struct work_struct *work )
{
	struct  vibrator_classdev	*classdev	= container_of( work, struct  vibrator_classdev, work_off );

	int	turn_off_vibrator	= 0;

	if( !classdev->turn_off_time || classdev->turn_off_time >= TIME_OUT_30_MS )
		turn_off_vibrator	= 1;

	if( !turn_off_vibrator && atomic_read( &classdev->turn_off ) )
	{
		turn_off_vibrator	= 1;
		atomic_set( &classdev->turn_off, TURN_OFF );
	}

	if( turn_off_vibrator )
		classdev->control( classdev, TURN_OFF );
}

static ssize_t vibrator_control( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	struct vibrator_classdev	*classdev	= dev_get_drvdata( dev );
	unsigned int	vibrate_time;

	sscanf( buf, "%d", &vibrate_time );

	switch( vibrate_time )
	{
		case	VIBRATE_OFF :
			printk( "VIB : Turn off vibrator(%s)\n", classdev->name );
			queue_work( vib_class.work_queue, &classdev->work_off );
			break;

		case	VIBRATE_ALWAYS :
			printk( "VIB : Always turn on vibrator(%s)\n", classdev->name );
			classdev->turn_off_time	= 0;
			queue_work( vib_class.work_queue, &classdev->work_on );
			break;

		case	VIBRATE_60_SEC :
		default :
			classdev->turn_off_time	= vibrate_time >= VIBRATE_60_SEC ? VIBRATE_60_SEC : vibrate_time;
			printk( "VIB : Turn off vibrator(%s) after %dms\n", classdev->name, classdev->turn_off_time );
			queue_work( vib_class.work_queue, &classdev->work_on );
			break;
	}

	return size;
}
static DEVICE_ATTR( control, S_IRUGO | S_IWUSR, NULL, vibrator_control );

static ssize_t vibrator_level( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	struct vibrator_classdev	*classdev	= dev_get_drvdata( dev );
	int	level;

	sscanf( buf, "%d", &level );
	classdev->level( classdev, level );
	return size;
}
static DEVICE_ATTR( level, S_IRUGO | S_IWUSR, NULL, vibrator_level );

int vibrator_classdev_register( struct vibrator_classdev *classdev )
{
	struct allocate_device_attribute	vibrator_device_attribute[] =
	{
		{ "control", &dev_attr_control, &classdev->control },
		{ "level", &dev_attr_level, &classdev->level },
	};

	int	loop;

	if( vib_class.exist == CLASS_NOT_EXIST )
		return	-EINVAL;

	classdev->index = atomic_inc_return( &vib_class.count );
	classdev->dev	= device_create( vib_class.vibrator, NULL, MKDEV( 0, classdev->index ), NULL, classdev->name );

	if( IS_ERR( classdev->dev ) )
	{
		printk( KERN_ERR "VIB : Vibrator(%s), device_create() failed\n", classdev->name );
		return	PTR_ERR( classdev->dev );
	}

	INIT_WORK( &classdev->work_on, vibrator_sched_on );
	INIT_WORK( &classdev->work_off, vibrator_sched_off );
	hrtimer_init( &classdev->vibrate_time, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	classdev->vibrate_time.function	= vibrator_timeout;
	dev_set_drvdata( classdev->dev, classdev );

	for( loop = 0 ; loop < sizeof( vibrator_device_attribute ) / sizeof( *vibrator_device_attribute ) ; ++loop )
	{
		struct allocate_device_attribute	*device_attribute	= vibrator_device_attribute + loop;

		if( !*device_attribute->function )
		{
			*device_attribute->function	= empty_func;
			printk( KERN_ERR "VIB : Vibrator(%s) does not set %s()\n", classdev->name, device_attribute->name );
			continue;
		}

		if( device_create_file( classdev->dev, device_attribute->attribute ) )
			printk( KERN_ERR "VIB : Vibrator(%s), device_create_file() failed\n", classdev->name );
	}

	return	0;
}
EXPORT_SYMBOL_GPL(vibrator_classdev_register);

void vibrator_classdev_unregister( struct vibrator_classdev *classdev )
{
	struct device_attribute	*vibrator_attribute[] =
	{
		&dev_attr_control, &dev_attr_level
	};

	int	loop;

	for( loop = 0 ; loop < sizeof( vibrator_attribute ) / sizeof( *vibrator_attribute ) ; ++loop )
		device_remove_file( classdev->dev, *( vibrator_attribute + loop ) );

	device_destroy( vib_class.vibrator, MKDEV( 0, classdev->index ) );
	dev_set_drvdata( classdev->dev, NULL );
}
EXPORT_SYMBOL_GPL(vibrator_classdev_unregister);

static int __init class_vibrator_init(void)
{
	vib_class.vibrator	= class_create( THIS_MODULE, "vibrator" );
	vib_class.exist		= CLASS_NOT_EXIST;

	if( IS_ERR( vib_class.vibrator ) )
	{
		printk( KERN_ERR "VIB : calss_create() failed\n" );
		return	PTR_ERR( vib_class.vibrator );
	}

	atomic_set( &vib_class.count, 0 );
	vib_class.work_queue	= create_workqueue( "vibrator_wq" );
	vib_class.exist		= CLASS_EXIST;
	return 0;
}
module_init(class_vibrator_init);

static void __exit class_vibrator_exit(void)
{
	if( vib_class.exist == CLASS_EXIST )
		class_destroy( vib_class.vibrator );
}
module_exit(class_vibrator_exit);

MODULE_AUTHOR("Y.S Chang <yschang@fih-foxconn.com>");

MODULE_DESCRIPTION("vibrator class driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:vibrator");
