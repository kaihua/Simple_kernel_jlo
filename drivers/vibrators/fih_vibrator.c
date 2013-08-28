
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/fih_vibrator.h>
#include <mach/pmic.h>

static void vibrator_control( struct vibrator_classdev *classdev, int state )
{
	struct vib_driver_data	*driver_data	= container_of( classdev, struct vib_driver_data, classdev );

	if( pmic_vib_mot_set_volt( state ? driver_data->level : 0 ) < 0 )
		printk( KERN_ERR "VIB : Vibrator(%s) set voltage level failed\n", classdev->name );
	else
		printk( "VIB : Vibrator(%s) %s\n", classdev->name, state ? "on" : "off" );
}

static void vibrator_level( struct vibrator_classdev *classdev, int level )
{
	struct vib_driver_data	*driver_data	= container_of( classdev, struct vib_driver_data, classdev );

	driver_data->level	= level;
	printk( "VIB : Vibrator(%s), vibrator level(%d)\n", classdev->name, driver_data->level );
}

static int __devinit vibrator_probe(struct platform_device *pdev)
{
	struct vib_driver_data		*driver_data	= kzalloc( sizeof( struct vib_driver_data ), GFP_KERNEL );
	struct vib_device_data		*device_data	= pdev->dev.platform_data;
	struct vibrator_classdev	*classdev	= &driver_data->classdev;

	if( !driver_data )
	{
		printk( KERN_ERR "VIB : kzalloc() failed\n" );
		return	-ENOMEM;
	}

	driver_data->level	= device_data->level;
	classdev->name		= device_data->name;
	classdev->control	= vibrator_control;
	classdev->level		= vibrator_level;
	vibrator_classdev_register( &driver_data->classdev );
	return	0;
}

static int __devexit vibrator_remove(struct platform_device *pdev)
{
	struct vib_driver_data	*driver_data	= dev_get_drvdata( &pdev->dev );

	vibrator_classdev_unregister( &driver_data->classdev );
	kfree(driver_data);
	return 0;
}

static struct platform_driver vibrator_device_driver =
{
	.probe		= vibrator_probe,
	.remove		= __devexit_p(vibrator_remove),
	.driver		=
	{
		.name	= "fih_vibrator",
		.owner	= THIS_MODULE,
	}
};

static int __init fih_vibrator_init(void)
{
	return platform_driver_register(&vibrator_device_driver);
}
module_init(fih_vibrator_init);

static void __exit fih_vibrator_exit(void)
{
	platform_driver_unregister(&vibrator_device_driver);
}
module_exit(fih_vibrator_exit);

MODULE_AUTHOR("Y.S Chang <yschang@fih-foxconn.com>");

MODULE_DESCRIPTION("FIH vibrator driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:vibrator");
