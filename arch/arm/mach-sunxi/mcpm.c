/*
 * Copyright (c) 2015 Chen-Yu Tsai
 *
 * Chen-Yu Tsai <wens@csie.org>
 *
 * arch/arm/mach-sunxi/mcpm.c
 *
 * Based on arch/arm/mach-exynos/mcpm-exynos.c and Allwinner code
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/arm-cci.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_address.h>

#include <asm/cputype.h>
#include <asm/cp15.h>
#include <asm/mcpm.h>

#define SUNXI_CPUS_PER_CLUSTER		4
#define SUNXI_NR_CLUSTERS		2

#define SUN9I_A80_A15_CLUSTER		1

#define CPUCFG_CX_CTRL_REG0(c)		(0x10 * (c))
#define CPUCFG_CX_CTRL_REG0_L1_RST_DISABLE(n)	BIT(n)
#define CPUCFG_CX_CTRL_REG0_L1_RST_DISABLE_ALL	0xf
#define CPUCFG_CX_CTRL_REG0_L2_RST_DISABLE_A7	BIT(4)
#define CPUCFG_CX_CTRL_REG0_L2_RST_DISABLE_A15	BIT(0)
#define CPUCFG_CX_CTRL_REG1(c)		(0x10 * (c) + 0x4)
#define CPUCFG_CX_CTRL_REG1_ACINACTM	BIT(0)
#define CPUCFG_CX_RST_CTRL(c)		(0x80 + 0x4 * (c))
#define CPUCFG_CX_RST_CTRL_DBG_SOC_RST	BIT(24)
#define CPUCFG_CX_RST_CTRL_ETM_RST(n)	BIT(20 + (n))
#define CPUCFG_CX_RST_CTRL_ETM_RST_ALL	(0xf << 20)
#define CPUCFG_CX_RST_CTRL_DBG_RST(n)	BIT(16 + (n))
#define CPUCFG_CX_RST_CTRL_DBG_RST_ALL	(0xf << 16)
#define CPUCFG_CX_RST_CTRL_H_RST	BIT(12)
#define CPUCFG_CX_RST_CTRL_L2_RST	BIT(8)
#define CPUCFG_CX_RST_CTRL_CX_RST(n)	BIT(4 + (n))
#define CPUCFG_CX_RST_CTRL_CORE_RST(n)	BIT(n)
#define CPUCFG_CX_RST_CTRL_CORE_RST_ALL	(0xf << 0)

#define PRCM_CPU_PO_RST_CTRL(c)		(0x4 + 0x4 * (c))
#define PRCM_CPU_PO_RST_CTRL_CORE(n)	BIT(n)
#define PRCM_CPU_PO_RST_CTRL_CORE_ALL	0xf

#define R_CPUCFG_CLUSTER_PO_RST_CTRL(c)	(0x30 + (c) * 0x4)
#define R_CPUCFG_CLUSTER_PO_RST_CTRL_CORE(n)	BIT(n)
#define R_CPUCFG_CPU_SOFT_ENTRY_REG	0x01a4

#define PRCM_PWROFF_GATING_REG(c)	(0x100 + 0x4 * (c))
#define PRCM_PWROFF_GATING_REG_CLUSTER	BIT(0)
#define PRCM_PWROFF_GATING_REG_CORE(n)	BIT(n)
#define PRCM_PWR_SWITCH_REG(c, cpu)	(0x140 + 0x10 * (c) + 0x4 * (cpu))

static void __iomem *cpucfg_base;
static void __iomem *r_cpucfg_base;
static void __iomem *prcm_base;

static int sunxi_cpu_power_switch_set(unsigned int cpu, unsigned int cluster,
				      bool enable)
{
	u32 reg;

	/* control sequence from Allwinner A80 user manual v1.2 PRCM section */
	reg = readl(prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
	if (enable) {
		if (reg == 0x00) {
			pr_debug("power clamp for cluster %u cpu %u already open\n",
				 cluster, cpu);
			return 0;
		}

		writel(0xff, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
		writel(0xfe, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
		writel(0xf8, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
		writel(0xf0, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
		writel(0x00, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
	} else {
		writel(0xff, prcm_base + PRCM_PWR_SWITCH_REG(cluster, cpu));
		udelay(10);
	}

	return 0;
}

static int sunxi_cpu_powerup(unsigned int cpu, unsigned int cluster)
{
	u32 reg;

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	if (cpu >= SUNXI_CPUS_PER_CLUSTER || cluster >= SUNXI_NR_CLUSTERS)
		return -EINVAL;

	/* assert processor power-on reset */
	reg = readl(prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));
	reg &= ~PRCM_CPU_PO_RST_CTRL_CORE(cpu);
	writel(reg, prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));
	/* assert cpu power-on reset FROM ALLWINNER */
	reg  = readl(r_cpucfg_base + R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
	reg &= ~(R_CPUCFG_CLUSTER_PO_RST_CTRL_CORE(cpu));
	writel(reg, r_cpucfg_base + R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
	udelay(10);

	/* Cortex-A7: hold L1 reset disable signal low */
	if (!(of_machine_is_compatible("allwinner,sun9i-a80") &&
			cluster == SUN9I_A80_A15_CLUSTER)) {
		reg = readl(cpucfg_base + CPUCFG_CX_CTRL_REG0(cluster));
		reg &= ~CPUCFG_CX_CTRL_REG0_L1_RST_DISABLE(cpu);
		writel(reg, cpucfg_base + CPUCFG_CX_CTRL_REG0(cluster));
	}

	/* assert processor related resets */
	reg = readl(cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
	reg &= ~CPUCFG_CX_RST_CTRL_DBG_RST(cpu);

	/*
	 * Allwinner code also asserts resets for NEON on A15. According
	 * to ARM manuals, asserting power-on reset is sufficient.
	 */
	if (!(of_machine_is_compatible("allwinner,sun9i-a80") &&
			cluster == SUN9I_A80_A15_CLUSTER)) {
		reg &= ~CPUCFG_CX_RST_CTRL_ETM_RST(cpu);
	}
	writel(reg, cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));

	/* open power switch */
	sunxi_cpu_power_switch_set(cpu, cluster, true);

	/* Added by ALLWINNER to handle A83T bit swap */
	if (cpu == 0)
		cpu = 4;

	/* clear processor power gate */
	reg = readl(prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	reg &= ~PRCM_PWROFF_GATING_REG_CORE(cpu);
	writel(reg, prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	udelay(20);

	if (cpu == 4)
		cpu = 0;

	/* de-assert processor power-on reset */
	reg = readl(prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));
	reg |= PRCM_CPU_PO_RST_CTRL_CORE(cpu);
	writel(reg, prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));

	reg  = readl(r_cpucfg_base + R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
	reg |= R_CPUCFG_CLUSTER_PO_RST_CTRL_CORE(cpu);
	writel(reg, r_cpucfg_base + R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
	udelay(10);

	/* de-assert all processor resets */
	reg = readl(cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
	reg |= CPUCFG_CX_RST_CTRL_DBG_RST(cpu);
	reg |= CPUCFG_CX_RST_CTRL_CORE_RST(cpu);
	if (!(of_machine_is_compatible("allwinner,sun9i-a80") &&
			cluster == SUN9I_A80_A15_CLUSTER)) {
		reg |= CPUCFG_CX_RST_CTRL_ETM_RST(cpu);
	} else {
		reg |= CPUCFG_CX_RST_CTRL_CX_RST(cpu); /* NEON */
	}
	writel(reg, cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));

	return 0;
}

static int sunxi_cluster_powerup(unsigned int cluster)
{
	u32 reg;

	pr_debug("%s: cluster %u\n", __func__, cluster);
	if (cluster >= SUNXI_NR_CLUSTERS)
		return -EINVAL;

	/* assert cluster cores resets */
	reg = readl(cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
	reg &= ~CPUCFG_CX_RST_CTRL_CORE_RST_ALL;   /* Core Reset    */
	writel(reg, cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
	udelay(10);

	/* assert ACINACTM */
	reg = readl(cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));
	reg |= CPUCFG_CX_CTRL_REG1_ACINACTM;
	writel(reg, cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));

	/* assert cluster processor power-on resets */
	reg = readl(prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));
	reg &= ~PRCM_CPU_PO_RST_CTRL_CORE_ALL;
	writel(reg, prcm_base + PRCM_CPU_PO_RST_CTRL(cluster));

	/* assert cluster cores resets */
	reg  = readl(r_cpucfg_base + R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
	reg &= ~CPUCFG_CX_RST_CTRL_CORE_RST_ALL;
	writel(reg, r_cpucfg_base + R_CPUCFG_CLUSTER_PO_RST_CTRL(cluster));
	udelay(10);

	/* assert cluster resets */
	reg = readl(cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
	reg &= ~CPUCFG_CX_RST_CTRL_DBG_SOC_RST;
	reg &= ~CPUCFG_CX_RST_CTRL_DBG_RST_ALL;
	reg &= ~CPUCFG_CX_RST_CTRL_H_RST;
	reg &= ~CPUCFG_CX_RST_CTRL_L2_RST;

	/*
	 * Allwinner code also asserts resets for NEON on A15. According
	 * to ARM manuals, asserting power-on reset is sufficient.
	 */
	if (!(of_machine_is_compatible("allwinner,sun9i-a80") &&
			cluster == SUN9I_A80_A15_CLUSTER)) {
		reg &= ~CPUCFG_CX_RST_CTRL_ETM_RST_ALL;
	}
	writel(reg, cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));

	/* hold L1/L2 reset disable signals low */
	reg = readl(cpucfg_base + CPUCFG_CX_CTRL_REG0(cluster));
	if (of_machine_is_compatible("allwinner,sun9i-a80") &&
			cluster == SUN9I_A80_A15_CLUSTER) {
		/* Cortex-A15: hold L2RSTDISABLE low */
		reg &= ~CPUCFG_CX_CTRL_REG0_L2_RST_DISABLE_A15;
	} else {
		/* Cortex-A7: hold L1RSTDISABLE and L2RSTDISABLE low */
		reg &= ~CPUCFG_CX_CTRL_REG0_L1_RST_DISABLE_ALL;
		reg &= ~CPUCFG_CX_CTRL_REG0_L2_RST_DISABLE_A7;
	}
	writel(reg, cpucfg_base + CPUCFG_CX_CTRL_REG0(cluster));

	/* clear cluster power gate */
	reg = readl(prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	reg &= ~PRCM_PWROFF_GATING_REG_CLUSTER;
	writel(reg, prcm_base + PRCM_PWROFF_GATING_REG(cluster));
	udelay(20);

	/* de-assert cluster resets */
	reg = readl(cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));
	reg |= CPUCFG_CX_RST_CTRL_DBG_SOC_RST;
	reg |= CPUCFG_CX_RST_CTRL_H_RST;
	reg |= CPUCFG_CX_RST_CTRL_L2_RST;
	writel(reg, cpucfg_base + CPUCFG_CX_RST_CTRL(cluster));

	/* de-assert ACINACTM */
	reg = readl(cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));
	reg &= ~CPUCFG_CX_CTRL_REG1_ACINACTM;
	writel(reg, cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));

	return 0;
}

static void sunxi_cpu_cache_disable(void)
{
	/* Disable and flush the local CPU cache. */
	v7_exit_coherency_flush(louis);
}

/*
 * This bit is shared between the initial mcpm_sync_init call to enable
 * CCI-400 and proper cluster cache disable before power down.
 */
static void sunxi_cluster_cache_disable_without_axi(void)
{
	if (read_cpuid_part() == ARM_CPU_PART_CORTEX_A15) {
		/*
		 * On the Cortex-A15 we need to disable
		 * L2 prefetching before flushing the cache.
		 */
		asm volatile(
		"mcr	p15, 1, %0, c15, c0, 3\n"
		"isb\n"
		"dsb"
		: : "r" (0x400));
	}

	/* Flush all cache levels for this cluster. */
	v7_exit_coherency_flush(all);

	/*
	 * Disable cluster-level coherency by masking
	 * incoming snoops and DVM messages:
	 */
	cci_disable_port_by_cpu(read_cpuid_mpidr());
}

static void sunxi_cluster_cache_disable(void)
{
	unsigned int cluster = MPIDR_AFFINITY_LEVEL(read_cpuid_mpidr(), 1);
	u32 reg;

	pr_info("%s: cluster %u\n", __func__, cluster);

	sunxi_cluster_cache_disable_without_axi();

	/* last man standing, assert ACINACTM */
	reg = readl(cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));
	reg |= CPUCFG_CX_CTRL_REG1_ACINACTM;
	writel(reg, cpucfg_base + CPUCFG_CX_CTRL_REG1(cluster));
}

static const struct mcpm_platform_ops sunxi_power_ops = {
	.cpu_powerup		= sunxi_cpu_powerup,
	.cluster_powerup	= sunxi_cluster_powerup,
	.cpu_cache_disable	= sunxi_cpu_cache_disable,
	.cluster_cache_disable	= sunxi_cluster_cache_disable,
};

/*
 * Enable cluster-level coherency, in preparation for turning on the MMU.
 *
 * Also enable regional clock gating and L2 data latency settings for
 * Cortex-A15.
 */
static void __naked sunxi_power_up_setup(unsigned int affinity_level)
{
	asm volatile (
		"mrc	p15, 0, r1, c0, c0, 0\n"
		"movw	r2, #" __stringify(ARM_CPU_PART_MASK & 0xffff) "\n"
		"movt	r2, #" __stringify(ARM_CPU_PART_MASK >> 16) "\n"
		"and	r1, r1, r2\n"
		"movw	r2, #" __stringify(ARM_CPU_PART_CORTEX_A15 & 0xffff) "\n"
		"movt	r2, #" __stringify(ARM_CPU_PART_CORTEX_A15 >> 16) "\n"
		"cmp	r1, r2\n"
		"bne	not_a15\n"

		/* The following is Cortex-A15 specific */

		/* L2CTRL: Enable CPU regional clock gates */
		"mrc p15, 1, r1, c15, c0, 4\n"
		"orr r1, r1, #(0x1<<31)\n"
		"mcr p15, 1, r1, c15, c0, 4\n"

		/* L2ACTLR */
		"mrc p15, 1, r1, c15, c0, 0\n"
		/* Enable L2, GIC, and Timer regional clock gates */
		"orr r1, r1, #(0x1<<26)\n"
		/* Disable clean/evict from being pushed to external */
		"orr r1, r1, #(0x1<<3)\n"
		"mcr p15, 1, r1, c15, c0, 0\n"

		/* L2 data RAM latency */
		"mrc p15, 1, r1, c9, c0, 2\n"
		"bic r1, r1, #(0x7<<0)\n"
		"orr r1, r1, #(0x3<<0)\n"
		"mcr p15, 1, r1, c9, c0, 2\n"

		/* End of Cortex-A15 specific setup */
		"not_a15:\n"

		"cmp	r0, #1\n"
		"bxne	lr\n"
		"b	cci_enable_port_for_self"
	);
}

static void sunxi_mcpm_setup_entry_point(void)
{
	__raw_writel(virt_to_phys(mcpm_entry_point), r_cpucfg_base +
		     R_CPUCFG_CPU_SOFT_ENTRY_REG);
}

static int __init sunxi_mcpm_init(void)
{
	struct device_node *node;
	int ret;

	if (!of_machine_is_compatible("allwinner,sun8i-a83t"))
		return -ENODEV;

	if (!cci_probed())
		return -ENODEV;

	node = of_find_compatible_node(NULL, NULL,
			"allwinner,sun8i-a83t-cpucfg");
	if (!node)
		return -ENODEV;

	cpucfg_base = of_iomap(node, 0);
	of_node_put(node);
	if (!cpucfg_base) {
		pr_err("%s: failed to map CPUCFG registers\n", __func__);
		return -ENOMEM;
	}

	node = of_find_compatible_node(NULL, NULL,
			"allwinner,sun8i-a83t-r-cpucfg");
	if (!node)
		return -ENODEV;
	r_cpucfg_base = of_iomap(node, 0);

	of_node_put(node);
	if (!r_cpucfg_base) {
		pr_err("%s: failed to map R-CPUCFG registers\n", __func__);
		return -ENOMEM;
	}

	node = of_find_compatible_node(NULL, NULL, "allwinner,sun8i-a83t-prcm");
	if (!node)
		return -ENODEV;

	prcm_base = of_iomap(node, 0);
	of_node_put(node);
	if (!prcm_base) {
		pr_err("%s: failed to map PRCM registers\n", __func__);
		iounmap(prcm_base);
		return -ENOMEM;
	}

	ret = mcpm_platform_register(&sunxi_power_ops);
	if (!ret)
		ret = mcpm_sync_init(sunxi_power_up_setup);
	if (!ret)
		/* do not disable AXI master as no one will re-enable it */
		ret = mcpm_loopback(sunxi_cluster_cache_disable_without_axi);
	if (ret) {
		iounmap(cpucfg_base);
		iounmap(prcm_base);
		return ret;
	}

	mcpm_smp_set_ops();

	pr_info("sunxi MCPM support installed\n");

	sunxi_mcpm_setup_entry_point();

	return ret;
}

early_initcall(sunxi_mcpm_init);
