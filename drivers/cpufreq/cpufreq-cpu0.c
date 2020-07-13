/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * The OPP code in function cpu0_set_target() is reused from
 * drivers/cpufreq/omap-cpufreq.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/opp.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

static unsigned int transition_latency;
static unsigned int voltage_tolerance; /* in percentage */

static struct device *cpu_dev;
static struct clk *cpu_clk;
static struct regulator *cpu_reg;
static struct cpufreq_frequency_table *freq_table;

static int cpu0_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, freq_table);
}

static unsigned int cpu0_get_speed(unsigned int cpu)
{
	return clk_get_rate(cpu_clk) / 1000;
}

int __cpu_get_mv(struct cpufreq_policy *policy,
		unsigned int *mv)
{
	unsigned long volt;
	*mv = 0;

	if (cpu_reg) {
		volt = regulator_get_voltage(cpu_reg);
		*mv = volt/1000;
	}
	return 0;
}

static int __cpu_get_cur_voltages(unsigned int *cur_volt,
				  unsigned int *norm_volt)
{
	unsigned int cur_freq;
	struct opp *opp;
	unsigned int tol;

	cur_freq = clk_get_rate(cpu_clk);

	if (cpu_reg) {
		rcu_read_lock();
		opp = opp_find_freq_ceil(cpu_dev, &cur_freq);
		if (IS_ERR(opp)) {
			rcu_read_unlock();
			pr_err("failed to find OPP for %ld\n", cur_freq);
			return PTR_ERR(opp);
		}
		*norm_volt = opp_get_voltage(opp);
		rcu_read_unlock();
		*cur_volt = regulator_get_voltage(cpu_reg);
	}
	else
	{
		return 1;
	}
	return 0;
}

int __cpu_get_boost(struct cpufreq_policy *policy,
			unsigned int *boost)
{
	unsigned int cur_volt, norm_volt;

	if (__cpu_get_cur_voltages(&cur_volt,&norm_volt))
	{
		*boost = 1;
		return 1;
	}

	if (cur_volt < norm_volt){
		*boost = 0;
		return 0;
	}
	else if (cur_volt == norm_volt){
		*boost = 1;
		return 1;
	}
	else{
		*boost = 2;
		return 2;
	}

}

int __cpu_set_boost(struct cpufreq_policy *policy,
		unsigned int boost)
{
	int ret;
	unsigned int cur_volt, norm_volt, new_volt,tol;

	if (__cpu_get_cur_voltages(&cur_volt,&norm_volt))
	{
		pr_err("unable to determine current voltage\n");
		return 1;
	}

	switch (boost){
		case 0:
			new_volt = norm_volt - 25000;
			break;
		case 1:
			new_volt = norm_volt;
			break;
		case 2:
			new_volt = norm_volt + 25000;
			break;
		default:
			new_volt = norm_volt;
	}

	if (new_volt > 1180000){
		pr_err("voltage out of bounds\n");
		return -EINVAL;
	}

	if (cpu_reg) {
		tol = new_volt * voltage_tolerance / 100;

		ret = regulator_set_voltage_tol(cpu_reg, new_volt, tol);
		if (ret) {
			pr_err("failed to scale voltage: %d\n", ret);
			return ret;
		}
	}
	return 0;
}

static int cpu0_set_target(struct cpufreq_policy *policy,
			   unsigned int target_freq, unsigned int relation)
{
	struct cpufreq_freqs freqs;
	struct opp *opp;
	unsigned long freq_Hz, volt = 0, volt_old = 0, tol = 0;
	unsigned int index, cpu;
	int ret;


	ret = cpufreq_frequency_table_target(policy, freq_table, target_freq,
					     relation, &index);
	if (ret) {
		pr_err("failed to match target freqency %d: %d\n",
		       target_freq, ret);
		return ret;
	}

	freq_Hz = clk_round_rate(cpu_clk, freq_table[index].frequency * 1000);
	if (freq_Hz < 0)
		freq_Hz = freq_table[index].frequency * 1000;
	freqs.new = freq_Hz / 1000;
	freqs.old = clk_get_rate(cpu_clk) / 1000;


	if (freqs.old == freqs.new)
		return 0;

	for_each_online_cpu(cpu) {
		freqs.cpu = cpu;
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	}

	if (cpu_reg) {
		rcu_read_lock();
		opp = opp_find_freq_ceil(cpu_dev, &freq_Hz);
		if (IS_ERR(opp)) {
			rcu_read_unlock();
			pr_err("failed to find OPP for %ld\n", freq_Hz);
			return PTR_ERR(opp);
		}
		volt = opp_get_voltage(opp);
		rcu_read_unlock();
		tol = volt * voltage_tolerance / 100;
		volt_old = regulator_get_voltage(cpu_reg);
	}

	pr_debug("%u MHz, %ld mV --> %u MHz, %ld mV\n",
		 freqs.old / 1000, volt_old ? volt_old / 1000 : -1,
		 freqs.new / 1000, volt ? volt / 1000 : -1);

	/* scaling up?  scale voltage before frequency */
	if (cpu_reg && freqs.new > freqs.old) {
		ret = regulator_set_voltage_tol(cpu_reg, volt, tol);
		if (ret) {
			pr_err("failed to scale voltage up: %d\n", ret);
			freqs.new = freqs.old;
			return ret;
		}
	}

	ret = clk_set_rate(cpu_clk, freqs.new * 1000);
	if (ret) {
		pr_err("failed to set clock rate: %d\n", ret);
		if (cpu_reg)
			regulator_set_voltage_tol(cpu_reg, volt_old, tol);
		return ret;
	}

	/* scaling down?  scale voltage after frequency */
	if (cpu_reg && freqs.new < freqs.old) {
		ret = regulator_set_voltage_tol(cpu_reg, volt, tol);
		if (ret) {
			pr_err("failed to scale voltage down: %d\n", ret);
			clk_set_rate(cpu_clk, freqs.old * 1000);
			freqs.new = freqs.old;
			return ret;
		}
	}

	for_each_online_cpu(cpu) {
		freqs.cpu = cpu;
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}

	return 0;
}

static int cpu0_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret;

	if (policy->cpu != 0)
		return -EINVAL;

	ret = cpufreq_frequency_table_cpuinfo(policy, freq_table);
	if (ret) {
		pr_err("invalid frequency table: %d\n", ret);
		return ret;
	}

	policy->cpuinfo.transition_latency = transition_latency;
	policy->cur = clk_get_rate(cpu_clk) / 1000;

	/*
	 * The driver only supports the SMP configuartion where all processors
	 * share the clock and voltage and clock.  Use cpufreq affected_cpus
	 * interface to have all CPUs scaled together.
	 */
	policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
	cpumask_setall(policy->cpus);

	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);

	return 0;
}

static int cpu0_cpufreq_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);

	return 0;
}

static struct freq_attr *cpu0_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver cpu0_cpufreq_driver = {
	.flags = CPUFREQ_STICKY,
	.verify = cpu0_verify_speed,
	.target = cpu0_set_target,
	.get = cpu0_get_speed,
	.init = cpu0_cpufreq_init,
	.exit = cpu0_cpufreq_exit,
	.name = "generic_cpu0",
	.attr = cpu0_cpufreq_attr,
};

static int cpu0_cpufreq_driver_init(void)
{
	struct device_node *np;
	int ret;

	np = of_find_node_by_path("/cpus/cpu@0");
	if (!np) {
		pr_err("failed to find cpu0 node\n");
		return -ENOENT;
	}

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev) {
		pr_err("failed to get cpu0 device\n");
		ret = -ENODEV;
		goto out_put_node;
	}

	cpu_dev->of_node = np;

	cpu_clk = clk_get(cpu_dev, NULL);
	if (IS_ERR(cpu_clk)) {
		ret = PTR_ERR(cpu_clk);
		pr_err("failed to get cpu0 clock: %d\n", ret);
		goto out_put_node;
	}
	cpu_reg = regulator_get(cpu_dev, "cpu0");
	if (IS_ERR(cpu_reg)) {
		pr_warn("failed to get cpu0 regulator\n");
		cpu_reg = NULL;
	}

	ret = opp_get_opp_count(cpu_dev);
	if (ret < 0) {
		ret = of_init_opp_table(cpu_dev);
		if (ret) {
			pr_err("failed to init OPP table: %d\n", ret);
			goto out_put_node;
		}
	}

	ret = opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		pr_err("failed to init cpufreq table: %d\n", ret);
		goto out_put_node;
	}

	of_property_read_u32(np, "voltage-tolerance", &voltage_tolerance);

	if (of_property_read_u32(np, "clock-latency", &transition_latency))
		transition_latency = CPUFREQ_ETERNAL;

	if (cpu_reg) {
		struct opp *opp;
		unsigned long min_uV, max_uV;
		int i;

		/*
		 * OPP is maintained in order of increasing frequency, and
		 * freq_table initialised from OPP is therefore sorted in the
		 * same order.
		 */
		for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++)
			;
		rcu_read_lock();
		opp = opp_find_freq_exact(cpu_dev,
				freq_table[0].frequency * 1000, true);
		min_uV = opp_get_voltage(opp);
		opp = opp_find_freq_exact(cpu_dev,
				freq_table[i-1].frequency * 1000, true);
		max_uV = opp_get_voltage(opp);
		rcu_read_unlock();
		ret = regulator_set_voltage_time(cpu_reg, min_uV, max_uV);
		if (ret > 0)
			transition_latency += ret * 1000;
	}

	ret = cpufreq_register_driver(&cpu0_cpufreq_driver);
	if (ret) {
		pr_err("failed register driver: %d\n", ret);
		goto out_free_table;
	}

	of_node_put(np);
	return 0;

out_free_table:
	opp_free_cpufreq_table(cpu_dev, &freq_table);
out_put_node:
	of_node_put(np);
	return ret;
}
late_initcall(cpu0_cpufreq_driver_init);

MODULE_AUTHOR("Shawn Guo <shawn.guo@linaro.org>");
MODULE_DESCRIPTION("Generic CPU0 cpufreq driver");
MODULE_LICENSE("GPL");