#ifndef __LINUX_ATSAM_D21X_H
#define __LINUX_ATSAM_D21X_H

struct atsam_d21x_platform_data {
	unsigned int max_x;
	unsigned int max_y;
	int irq;
	int reset_gpio;
};

#endif /* __LINUX_ATSAM_D21X_H */
