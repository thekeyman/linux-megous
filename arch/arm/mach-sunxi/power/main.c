/*
 * SUNXI suspend
 *
 * Copyright (C) 2014 AllWinnertech Ltd.
 * Author: xiafeng <xiafeng@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpu_pm.h>
#include <linux/io.h>
#include <linux/export.h>
#include <linux/time.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <linux/major.h>
#include <linux/device.h>
#include <linux/console.h>
#include <linux/syscore_ops.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/tlbflush.h>
#include <asm/mach/map.h>
#include <mach/sys_config.h>

#include <asm/smp_plat.h>
#include <asm/delay.h>
#include <asm/cp15.h>
#include <asm/suspend.h>
#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <mach/cci.h>
#include <asm/mcpm.h>

#include <linux/arisc/arisc.h>
#include <linux/power/aw_pm.h>
#include <linux/power/scenelock.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/cpuidle-sunxi.h>

#define SUNXI_PM_DBG   1
#ifdef PM_DBG
#undef PM_DBG
#endif
#if SUNXI_PM_DBG
    #define PM_DBG(format,args...)   printk("[pm]"format,##args)
#else
    #define PM_DBG(format,args...)   do{}while(0)
#endif

#define BEFORE_EARLY_SUSPEND    (0x00)
#define SUSPEND_BEGIN           (0x20)
#define SUSPEND_ENTER           (0x40)
#define BEFORE_LATE_RESUME      (0x60)
#define LATE_RESUME_START       (0x80)
#define CLK_RESUME_START        (0xA0)
#define AFTER_LATE_RESUME       (0xC0)
#define RESUME_COMPLETE_FLAG    (0xE0)
#define SUSPEND_FAIL_FLAG       (0xFF)
#define FIRST_BOOT_FLAG         (0x00)

standby_type_e standby_type = NON_STANDBY;
EXPORT_SYMBOL(standby_type);
standby_level_e standby_level = STANDBY_INITIAL;
EXPORT_SYMBOL(standby_level);

static unsigned long debug_mask = 0;
module_param_named(debug_mask, debug_mask, ulong, S_IRUGO | S_IWUSR);

static unsigned long time_to_wakeup = 0;
module_param_named(time_to_wakeup, time_to_wakeup, ulong, S_IRUGO | S_IWUSR);

extern char sunxi_bromjump_start;
extern char sunxi_bromjump_end;
static const extended_standby_manager_t *extended_standby_manager_id = NULL;

extern unsigned long cpu_brom_addr[2];

static inline int sunxi_mem_get_status(void)
{
	return readl(IO_ADDRESS(SUNXI_RTC_PBASE + 0x104));
}

static inline void sunxi_mem_set_status(int val)
{
	writel(val, IO_ADDRESS(SUNXI_RTC_PBASE + 0x104));
	asm volatile ("dsb");
	asm volatile ("isb");
}

/**
 * sunxi_pm_valid() - determine if given system sleep state is supported
 *
 * @state: suspend state
 * @return: if the state is valid, return 1, else return 0
 */
static int sunxi_pm_valid(suspend_state_t state)
{
	if (!((state > PM_SUSPEND_ON) && (state < PM_SUSPEND_MAX))) {
		PM_DBG("state:%d invalid!\n", state);
		return 0;
	}
	PM_DBG("state:%d valid\n", state);

	if (PM_SUSPEND_STANDBY == state)
		standby_type = SUPER_STANDBY;
	else if (PM_SUSPEND_MEM == state || PM_SUSPEND_BOOTFAST == state)
		standby_type = NORMAL_STANDBY;

	return 1;
}

/**
 * sunxi_pm_begin() - Initialise a transition to given system sleep state
 *
 * @state: suspend state
 * @return: return 0 for process successed
 * @note: this function will be called before devices suspened
 */
static int sunxi_pm_begin(suspend_state_t state)
{
	int last_suspend_status;
	static bool backup_console_suspend_enabled = 0;
	static bool backup_initcall_debug = 0;
	static int backup_console_loglevel = 0;
	static int backup_debug_mask = 0;

	last_suspend_status = sunxi_mem_get_status();
	if (RESUME_COMPLETE_FLAG != last_suspend_status) {
		printk(KERN_WARNING "last suspend err, rtc:%x\n", last_suspend_status);
		/* adjust: loglevel, console_suspend, initcall_debug.
		 * set debug_mask, disable console suspend if last suspend fail
		 */
		backup_console_suspend_enabled = console_suspend_enabled;
		console_suspend_enabled = 0;
		backup_initcall_debug = initcall_debug;
		initcall_debug = 1;
		backup_console_loglevel = console_loglevel;
		console_loglevel = 8;
		backup_debug_mask = debug_mask;
		debug_mask |= 0x0f;
	} else if ((suspend_stats.success + suspend_stats.fail) > 0) {
		/* restore console suspend, initcall_debug, debug_mask, if
		 * suspend_stats.success + .fail is 0 means the first time enter
		 * suspend, all the back data is 0.
		 */
		console_suspend_enabled = backup_console_suspend_enabled;
		initcall_debug = backup_initcall_debug;
		console_loglevel = backup_console_loglevel;
		debug_mask = backup_debug_mask;
	}

	sunxi_mem_set_status(SUSPEND_BEGIN | 0x01);

	return 0;
}

/**
 * sunxi_pm_prepare() - Prepare for entering the system suspend state
 *
 * @return: return 0 for process successed, and negative code for error
 * @note: called after devices suspended, and before device late suspend
 *             call-back functions
 */
static int sunxi_pm_prepare(void)
{
	sunxi_mem_set_status(SUSPEND_BEGIN | 0x03);

	return 0;
}

/**
 * sunxi_pm_prepare_late() - Finish preparing for entering the system suspend
 *
 * @return: return 0 for process successed, and negative code for error
 * @note: called before disabling nonboot CPUs and after device
 *        drivers' late suspend callbacks have been executed
 */
static int sunxi_pm_prepare_late(void)
{
	sunxi_mem_set_status(SUSPEND_BEGIN | 0x05);

	return 0;
}

/**
 * sunxi_suspend_enter() - enter suspend state
 *
 * @val: useless
 * @return: no return if success
 */
static int sunxi_suspend_enter(unsigned long val)
{
	super_standby_para_t st_para;

	sunxi_mem_set_status(SUSPEND_ENTER | 0x03);

	standby_level = STANDBY_WITH_POWER_OFF;

	memset((void *)&st_para, 0, sizeof(super_standby_para_t));

	st_para.event = CPUS_MEM_WAKEUP;
	st_para.event |= CPUS_WAKEUP_IR;
	st_para.timeout = time_to_wakeup;
	if (st_para.timeout > 0)
		st_para.event |= CPUS_WAKEUP_TIMEOUT;
	st_para.gpio_enable_bitmap = 0;
	st_para.cpux_gpiog_bitmap = 0;
	st_para.pextended_standby = NULL;
	st_para.resume_code_length = 0;
	/* the wakeup src is independent of the scene_lock.
	 * the developer only need to care about: the scene support the wakeup src
	 */
	if (NULL != extended_standby_manager_id) {
		st_para.event |= extended_standby_manager_id->event;
		st_para.gpio_enable_bitmap = extended_standby_manager_id->wakeup_gpio_map;
		st_para.cpux_gpiog_bitmap = extended_standby_manager_id->wakeup_gpio_group;
	}
	/* set cpu0 entry address */
#if (defined CONFIG_ARCH_SUN8IW6P1) || (defined CONFIG_ARCH_SUN9IW1P1)
	mcpm_set_entry_vector(0, 0, cpu_resume);
	st_para.resume_entry = virt_to_phys(&sunxi_bromjump_start);
	st_para.resume_code_src = virt_to_phys(mcpm_entry_point);
	PM_DBG("cpu resume:%x, mcpm enter:%x\n",
	       cpu_resume, virt_to_phys(mcpm_entry_point));
#else
	//cpu_brom_addr[0] = cpu_resume;
	st_para.resume_entry = virt_to_phys(&sunxi_bromjump_start);
	st_para.resume_code_src = virt_to_phys(cpu_resume);
	PM_DBG("cpu resume:%x\n", virt_to_phys(cpu_resume));
#endif
	if (unlikely(debug_mask)) {
		printk(KERN_INFO "standby paras:\n" \
		       "  event:%x\n" \
		       "  resume_code_src:%x\n" \
		       "  resume_entry:%x\n" \
		       "  timeout:%u\n" \
		       "  gpio_enable_bitmap:%x\n" \
		       "  cpux_gpiog_bitmap:%x\n" \
		       "  pextended_standby:%p\n", \
		       (unsigned int)st_para.event,
		       (unsigned int)st_para.resume_code_src,
		       (unsigned int)st_para.resume_entry,
		       (unsigned int)st_para.timeout,
		       (unsigned int)st_para.gpio_enable_bitmap,
		       (unsigned int)st_para.cpux_gpiog_bitmap,
		       st_para.pextended_standby);
	}

	if (unlikely(debug_mask)) {
		printk(KERN_INFO "system environment\n");
	}

#ifdef CONFIG_SUNXI_ARISC
	arisc_standby_super(&st_para, NULL, NULL);
	sunxi_idle_cluster_die(A7_CLUSTER);
#else
	asm("wfe" : : : "memory", "cc");
#endif

	return 0;
}

/**
 * sunxi_pm_enter() - Enter the system sleep state
 *
 * @state: suspend state
 * @return: return 0 is process successed
 * @note: the core function for platform sleep
 */
static int sunxi_pm_enter(suspend_state_t state)
{
	sunxi_mem_set_status(SUSPEND_ENTER | 0x01);

	return cpu_suspend(0, sunxi_suspend_enter);
}

/**
 * sunxi_pm_wake() - platform wakeup
 *
 * @return: called when the system has just left a sleep state,
 *          after the nonboot CPUs have been enabled and before
 *          device drivers' early resume callbacks are executed.
 */
static void sunxi_pm_wake(void)
{
	sunxi_mem_set_status(AFTER_LATE_RESUME);
}

/**
 * sunxi_pm_finish() - Finish wake-up of the platform
 *
 * @return: called prior to calling device drivers' regular suspend callbacks
 */
static void sunxi_pm_finish(void)
{
	sunxi_mem_set_status(RESUME_COMPLETE_FLAG);
}

/**
 * sunxi_pm_end() - Notify the platform that system is in work mode now
 *
 * @note: called after resuming devices, to indicate the
 *        platform that the system has returned to the working state or the
 *        transition to the sleep state has been aborted.
 */
static void sunxi_pm_end(void)
{
	sunxi_mem_set_status(RESUME_COMPLETE_FLAG);
}

/**
 * sunxi_pm_recover() - Recover platform from a suspend failure
 *
 * @note: called by the PM core if the suspending of devices fails.
 */
static void sunxi_pm_recover(void)
{
	printk(KERN_WARNING "suspend failure!\n");

	sunxi_mem_set_status(SUSPEND_FAIL_FLAG);
}


/* define platform_suspend_ops call-back functions, registered into PM core */
static struct platform_suspend_ops sunxi_pm_ops = {
    .valid = sunxi_pm_valid,
    .begin = sunxi_pm_begin,
    .prepare = sunxi_pm_prepare,
    .prepare_late = sunxi_pm_prepare_late,
    .enter = sunxi_pm_enter,
    .wake = sunxi_pm_wake,
    .finish = sunxi_pm_finish,
    .end = sunxi_pm_end,
    .recover = sunxi_pm_recover,
};

/* for back ahb configuration */
static unsigned int ahb_config;

static int sunxi_pm_syscore_suspend(void)
{
	/* back AHB1/APB1 Configuration Register */
	ahb_config = readl(IO_ADDRESS(SUNXI_CCM_PBASE + 0x54));
	if (unlikely(debug_mask)) {
		printk(KERN_INFO "ahb config:%x\n", ahb_config);
	}

	return 0;
}

static void sunxi_pm_syscore_resume(void)
{
	unsigned long value;

	sunxi_mem_set_status(CLK_RESUME_START);
	/* restore AHB1/APB1 Configuration Register form low to high frequency */
	value = readl(IO_ADDRESS(SUNXI_CCM_PBASE + 0x54));
	/* set AHB1_PRE_DIV, bit6~7 */
	value &= ~(0x03 << 6);
	value |= (ahb_config & (0x03 << 6));
	writel(value, IO_ADDRESS(SUNXI_CCM_PBASE + 0x54));
	mdelay(2);
	sunxi_mem_set_status(CLK_RESUME_START | 3);
	/* set AHB1_CLK_DIV_RATIO, bit4~5 */
	value &= ~(0x03 << 4);
	value |= (ahb_config & (0x03 << 4));
	writel(value, IO_ADDRESS(SUNXI_CCM_PBASE + 0x54));
	mdelay(2);
	sunxi_mem_set_status(CLK_RESUME_START | 5);
	/* set APB1_CLK_RATIO, bit8~9 */
	value &= ~(0x03 << 8);
	value |= (ahb_config & (0x03 << 8));
	writel(value, IO_ADDRESS(SUNXI_CCM_PBASE + 0x54));
	mdelay(2);
	sunxi_mem_set_status(CLK_RESUME_START | 7);
	/* set APB1_CLK_SRC_SEL, bit12~13 */
	writel(ahb_config, IO_ADDRESS(SUNXI_CCM_PBASE + 0x54));
	mdelay(2);
	sunxi_mem_set_status(CLK_RESUME_START | 9);
}

static struct syscore_ops sunxi_pm_syscore_ops = {
	.suspend = sunxi_pm_syscore_suspend,
	.resume = sunxi_pm_syscore_resume,
};

/**
 * sunxi_pm_init() - initial pm sub-system
 */
static int __init sunxi_pm_init(void)
{
	script_item_u *list = NULL;
	int wakeup_src_cnt = 0;
	unsigned int i = 0;
	unsigned int gpio = 0;

	pr_info("sunxi pm init\n");

	/* config wakeup sources */
	wakeup_src_cnt = script_get_pio_list("wakeup_src_para",&list);
	pr_info("wakeup src cnt is : %d. \n", wakeup_src_cnt);

	if(wakeup_src_cnt){
		while(wakeup_src_cnt--){
			gpio = (list + (i++) )->gpio.gpio;
			extended_standby_enable_wakeup_src(CPUS_GPIO_SRC, gpio);
		}
	}

	register_syscore_ops(&sunxi_pm_syscore_ops);
	suspend_set_ops(&sunxi_pm_ops);

	return 0;
}

/**
 * sunxi_pm_exit() - exit pm sub-system
 */
static void __exit sunxi_pm_exit(void)
{
	pr_info("sunxi pm exit\n");

	unregister_syscore_ops(&sunxi_pm_syscore_ops);
	suspend_set_ops(NULL);
}

core_initcall(sunxi_pm_init);
module_exit(sunxi_pm_exit);
