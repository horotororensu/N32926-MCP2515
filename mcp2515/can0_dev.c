#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>

#include <linux/platform_device.h>

#include "mcp2515.h"

void can0_release(struct device *dev)
{

}

struct resource can0_res[] = {

};

struct platform_device can0_dev = {
	.name = "can0",
	.id = -1,
	.dev = {
		.init_name = "can0",
		.release = can0_release,
	},
	.num_resources = ARRAY_SIZE(can0_res),
	.resource = can0_res,	
};

static __init int can0_init(void)
{
	return platform_device_register(&can0_dev);
}

static __exit void can0_exit(void)
{
	platform_device_unregister(&can0_dev);
}

module_init(can0_init);
module_exit(can0_exit);
MODULE_LICENSE("GPL");
