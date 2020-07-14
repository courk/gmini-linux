#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/host_ops.h>

void __ndelay(unsigned long nsecs)
{
	unsigned long long start = lkl_ops->time();

	while (lkl_ops->time() < start + nsecs)
		;
}

void __udelay(unsigned long usecs)
{
	__ndelay(usecs * NSEC_PER_USEC);
}

void __const_udelay(unsigned long xloops)
{
	__udelay(xloops / 5);
}

void calibrate_delay(void)
{
}

static cycle_t clock_read(struct clocksource *cs)
{
	return lkl_ops->time();
}

static struct clocksource clocksource = {
	.name	= "lkl",
	.rating = 499,
	.read	= clock_read,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
	.mask	= CLOCKSOURCE_MASK(64),
};

static void *timer;

static int timer_irq;

static void timer_fn(void *arg)
{
	lkl_trigger_irq(timer_irq);
}

static int clockevent_set_state_shutdown(struct clock_event_device *evt)
{
	if (timer) {
		lkl_ops->timer_free(timer);
		timer = NULL;
	}

	return 0;
}

static int clockevent_set_state_oneshot(struct clock_event_device *evt)
{
	timer = lkl_ops->timer_alloc(timer_fn, NULL);
	if (!timer)
		return -ENOMEM;

	return 0;
}

static void clockevent_set_mode(enum clock_event_mode mode, struct clock_event_device *evt)
{
	
	int ret;
	
	if (mode == CLOCK_EVT_MODE_SHUTDOWN)
		clockevent_set_state_shutdown(evt);
	else if (mode == CLOCK_EVT_MODE_ONESHOT) {
		ret = clockevent_set_state_oneshot(evt);
	}
}

static irqreturn_t timer_irq_handler(int irq, void *dev_id)
{
	struct clock_event_device *dev = (struct clock_event_device *)dev_id;

	dev->event_handler(dev);

	return IRQ_HANDLED;
}

static int clockevent_next_event(unsigned long ns,
				 struct clock_event_device *evt)
{
	return lkl_ops->timer_set_oneshot(timer, ns);
}

static struct clock_event_device clockevent = {
	.name			= "lkl",
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	//.set_state_oneshot	= clockevent_set_state_oneshot,
	.set_next_event		= clockevent_next_event,
	//.set_state_shutdown	= clockevent_set_state_shutdown,
	.set_mode               = clockevent_set_mode,
};

static struct irqaction irq0  = {
	.handler = timer_irq_handler,
	.flags = IRQF_NOBALANCING | IRQF_TIMER,
	.dev_id = &clockevent,
	.name = "timer"
};

void __init time_init(void)
{
	int ret;
	struct timespec ts;
	unsigned long long time;

	if (!lkl_ops->timer_alloc || !lkl_ops->timer_free ||
	    !lkl_ops->timer_set_oneshot || !lkl_ops->time) {
		pr_err("lkl: no time or timer support provided by host\n");
		return;
	}

	timer_irq = lkl_get_free_irq("timer");
	setup_irq(timer_irq, &irq0);

	ret = clocksource_register_khz(&clocksource, 1000000);
	if (ret)
		pr_err("lkl: unable to register clocksource\n");

	clockevents_config_and_register(&clockevent, NSEC_PER_SEC, 1, ULONG_MAX);

	time = lkl_ops->time();
	ts.tv_sec = time / NSEC_PER_SEC;
	ts.tv_nsec = time % NSEC_PER_SEC;
	do_settimeofday(&ts);

	pr_info("lkl: time and timers initialized (irq%d)\n", timer_irq);
}
