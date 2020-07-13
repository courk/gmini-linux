#ifndef __LINUX_PCA9956_H
#define __LINUX_PCA9956_H

#define MAX_NUM_LED 8
struct pca9956_platform_data {
	int num_leds;
	int reset_gpio;
	int start_num[MAX_NUM_LED];
};

#endif /* __LINUX_PCA9956_H */
