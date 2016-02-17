/*
 * Crystalfontz CFA-10049 FIQ handler
 *
 * Copyright (C) 2012 Free Electrons
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <asm/fiq.h>
#include <asm/pgtable.h>

#include "cfa10049_fiq.h"

#define TIMROT_TIMCTRL_REG(n)		(0x20 + (n) * 0x40)
#define TIMROT_TIMCTRL_SELECT_32K		(0xb)
#define TIMROT_TIMCTRL_ALWAYS_TICK		(0xf)
#define TIMROT_TIMCTRL_RELOAD			(1 << 6)
#define TIMROT_TIMCTRL_UPDATE			(1 << 7)
#define TIMROT_TIMCTRL_IRQ_EN			(1 << 14)
#define TIMROT_TIMCTRL_IRQ			(1 << 15)
#define TIMROT_FIXED_COUNT_REG(n)	(0x40 + (n) * 0x40)

#define GPIO	(3 * 32 + 4)

#define BIT_SET 0x4
#define BIT_CLR	0x8
#define BIT_TOG	0xc

#define HW_PINCTRL_DOE3			0xb30
#define HW_PINCTRL_DOUT3		0x730
#define HW_TIMROT_TIMCTRL2		0xa0

#define TIMER_DEFAULT			1000

#define FIQ

struct cfafiq_data {
	struct cdev	chrdev;
	dma_addr_t	dma;
	void __iomem	*fiq_base;
	unsigned int	irq;
	void __iomem	*pinctrl_base;
	struct clk	*timer_clk;
	void __iomem	*timrot_base;
	int		num_gpios;
	int		*gpios;
};

static struct cfafiq_data *cfa10049_fiq_data;
extern unsigned char cfa10049_fiq_handler, cfa10049_fiq_handler_end;
extern void mxs_icoll_set_irq_fiq(unsigned int irq);

#ifndef FIQ
static irqreturn_t cfafiq_handler(int irq, void *private)
{
	asm volatile (
		"ldr	r8, =0xf5068000	\n"
		"ldr	r9, =0xf5018000	\n"
		"mov	r10, %[fiqbase]	\n"
		/*
		 * Detect the underuns, and stop the execution if the
		 * application is too slow. Unprogram the timer and
		 * return directly from the FIQ
		 */
		"ldr	r12, [r10, #4]	\n"
		"cmp	r11, r12	\n"
		"moveq	r11, #0		\n"
		"streq	r11, [r8, #0xc0]\n"
		"moveq	r11, #2		\n"
		"streq	r11, [r10, #12]	\n"
		"beq	out		\n"
		/* Acknowledge the interrupt */
		"mov	r11, #1		\n"
		"lsl	r11, r11, #15	\n"
		"str	r11, [r8, #0xa8]\n"
		/* Get the read index */
		"ldr	r11, [r10]	\n"
		/* Increment it */
		"add	r11, r11, #1	\n"
		/* If we are over the edge of the buffer, return to
		 * the beginning */
		"ldr	r12, [r10, #8]	\n"
		"cmp	r11, r12	\n"
		"movcs	r11, #0		\n"
		/* Update the read index in the buffer */
		"str	r11, [r10]	\n"
		/* Get the start address of the array */
		"add	r12, r10, #16	\n"
		/* cells have 3 * 4 bytes */
		"add	r11, r11, r11, lsl #1	\n"
		"lsl	r11, r11, #2		\n"
		/* Compute the ptr to the cell */
		"add	r12, r12, r11		\n"
		/* Store the 1st cell in the timer control register */
		"ldr	r11, [r12, #0]		\n"
		"add	r11, r11, r11, lsl #1	\n"
		"lsl	r11, r11, #3		\n"
		"str	r11, [r8, #0xc0]	\n"
		/* Store the 2nd cell in the PIO clear register */
		"ldr	r11, [r12, #4]		\n"
		"str	r11, [r9, #0x738]	\n"
		/* Store the 3rd cell in the PIO set register */
		"ldr	r11, [r12, #8]		\n"
		"str	r11, [r9, #0x734]	\n"
		"out:"
		:: [fiqbase] "r" (cfa10049_fiq_data->fiq_base)
		: "memory", "cc", "r8", "r9", "r10", "r11", "r12");

	return IRQ_HANDLED;
}
#endif

#ifdef FIQ
static struct fiq_handler cfa10049_fh = {
	.name	= "cfa10049_fiq_handler"
};
#endif

static int cfa10049_fiq_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct fiq_buffer *fiq_buf = (struct fiq_buffer *)cfa10049_fiq_data->fiq_base;
	size_t size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset + size > FIQ_BUFFER_SIZE)
		return -EINVAL;

	offset += __pa(cfa10049_fiq_data->fiq_base);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Remap-pfn-range will mark the range VM_IO */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    offset >> PAGE_SHIFT,
			    size,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	}

	fiq_buf->rd_idx = 0;
	fiq_buf->wr_idx = 0;
	fiq_buf->status = 0;

	return 0;
}

static long cfa10049_fiq_ioctl(struct file *file,
			       unsigned int cmd, unsigned long arg)
{
	struct fiq_buffer *fiq_buf = (struct fiq_buffer *)cfa10049_fiq_data->fiq_base;

	switch (cmd) {
	case FIQ_START:
		fiq_buf->status = FIQ_STATUS_RUNNING;
		writel(TIMER_DEFAULT,
		       cfa10049_fiq_data->timrot_base + TIMROT_FIXED_COUNT_REG(2));
		break;
	case FIQ_STOP:
		fiq_buf->status = FIQ_STATUS_STOPPED;
		writel(0,
		       cfa10049_fiq_data->timrot_base + TIMROT_FIXED_COUNT_REG(2));
		break;
	case FIQ_RESET:
		fiq_buf->rd_idx = 0;
		fiq_buf->wr_idx = 0;
		fiq_buf->status = FIQ_STATUS_STOPPED;
		writel(TIMER_DEFAULT,
		       cfa10049_fiq_data->timrot_base + TIMROT_FIXED_COUNT_REG(2));
		break;
	default:
		return -ENOTTY;
	};

	return 0;
}

static const struct file_operations cfa10049_fiq_fops = {
	.mmap		= &cfa10049_fiq_mmap,
	.unlocked_ioctl	= &cfa10049_fiq_ioctl,
};

static struct miscdevice cfa10049_fiq_dev = {
	.name	= "cfa10049_fiq",
	.fops	= &cfa10049_fiq_fops,
	.minor	= MISC_DYNAMIC_MINOR,
};

static int cfafiq_probe(struct platform_device *pdev)
{
	struct fiq_buffer *fiq_buf;
	struct device_node *np;
	int ret, i;

#ifdef FIQ
	struct pt_regs regs;
#endif

	np = pdev->dev.of_node;
	if (!np) {
		dev_err(&pdev->dev, "No device tree data available\n");
		return -EINVAL;
	}

	cfa10049_fiq_data = devm_kzalloc(&pdev->dev,
					 sizeof(*cfa10049_fiq_data),
					 GFP_KERNEL);
	if (!cfa10049_fiq_data)
		return -ENOMEM;

	cfa10049_fiq_data->irq = irq_of_parse_and_map(np, 0);
	if (cfa10049_fiq_data->irq < 0) {
		dev_err(&pdev->dev, "Couldn't register given IRQ\n");
		return -EINVAL;
	}

	cfa10049_fiq_data->num_gpios = of_gpio_named_count(np,
							   "crystalfontz,fiq-gpios");
	if (cfa10049_fiq_data->num_gpios <= 0) {
		dev_err(&pdev->dev, "Invalid or no gpios given, error %d\n",
			cfa10049_fiq_data->num_gpios);
		return -EINVAL;
	}

	cfa10049_fiq_data->gpios = devm_kzalloc(&pdev->dev,
						sizeof(unsigned int) * cfa10049_fiq_data->num_gpios,
						GFP_KERNEL);
	if (!cfa10049_fiq_data->gpios)
		return -ENOMEM;

	for (i = 0; i < cfa10049_fiq_data->num_gpios; i++) {
		cfa10049_fiq_data->gpios[i] = of_get_named_gpio(np,
								"crystalfontz,fiq-gpios",
								i);

		if (cfa10049_fiq_data->gpios[i] == -EPROBE_DEFER) {
			dev_info(&pdev->dev,
				 "GPIO requested is not here yet, deferring the probe\n");
			return -EPROBE_DEFER;
		}

		if (!gpio_is_valid(cfa10049_fiq_data->gpios[i])) {
			dev_err(&pdev->dev, "Invalid GPIO given\n");
			return -EINVAL;
		}

		ret = devm_gpio_request_one(&pdev->dev,
					    cfa10049_fiq_data->gpios[i],
					    GPIOF_OUT_INIT_LOW, "fiq-pins");
		if (ret) {
			dev_err(&pdev->dev,
				"failed to request gpio %d: %d\n",
				cfa10049_fiq_data->gpios[i],
				ret);
			return -EINVAL;
		}
	}

	cfa10049_fiq_data->timer_clk = of_clk_get(np, 0);
	if (IS_ERR(cfa10049_fiq_data->timer_clk)) {
		dev_err(&pdev->dev, "Couldn't get the parent clock\n");
		return -EINVAL;
	}

	clk_prepare_enable(cfa10049_fiq_data->timer_clk);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx28-timrot");
	cfa10049_fiq_data->timrot_base = of_iomap(np, 0);
	if (!cfa10049_fiq_data->timrot_base)
		return -ENOMEM;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx28-pinctrl");
	cfa10049_fiq_data->pinctrl_base = of_iomap(np, 0);
	if (!cfa10049_fiq_data->pinctrl_base)
		return -ENOMEM;

	cfa10049_fiq_data->fiq_base = dma_zalloc_coherent(&pdev->dev,
							  FIQ_BUFFER_SIZE,
							  &cfa10049_fiq_data->dma,
							  GFP_KERNEL);
	if (!cfa10049_fiq_data->fiq_base) {
		dev_err(&pdev->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	fiq_buf = (struct fiq_buffer *)cfa10049_fiq_data->fiq_base;
	fiq_buf->size = ((FIQ_BUFFER_SIZE - 4 * sizeof(unsigned long)) /
			 sizeof(struct fiq_cell));
	dev_info(&pdev->dev,
		 "Allocated pages at address 0x%p, with size %dMB (%lu cells)\n",
		 cfa10049_fiq_data->fiq_base, FIQ_BUFFER_SIZE >> 20,
		 fiq_buf->size);

	/*
	 * Setup timer 2 for our FIQ (the two first are already used
	 * for the clocksource events). Since we are targeting an
	 * imx28, we only use the timrotv2.
	 */
	writel(TIMROT_TIMCTRL_RELOAD | TIMROT_TIMCTRL_UPDATE |
	       TIMROT_TIMCTRL_ALWAYS_TICK | TIMROT_TIMCTRL_IRQ_EN,
	       cfa10049_fiq_data->timrot_base + TIMROT_TIMCTRL_REG(2));

#ifdef FIQ
	ret = claim_fiq(&cfa10049_fh);
	if (ret)
		return ret;

	set_fiq_handler(&cfa10049_fiq_handler,
			&cfa10049_fiq_handler_end - &cfa10049_fiq_handler);

	regs.ARM_r8 = (long)cfa10049_fiq_data->timrot_base;
	regs.ARM_r9 = (long)cfa10049_fiq_data->pinctrl_base;
	regs.ARM_r10 = (long)cfa10049_fiq_data->fiq_base;
	set_fiq_regs(&regs);

	/* Enable the FIQ */
	mxs_icoll_set_irq_fiq(cfa10049_fiq_data->irq);
	enable_fiq(cfa10049_fiq_data->irq);
#else
	ret = request_irq(cfa10049_fiq_data->irq,
			  cfafiq_handler,
			  0,
			  pdev->dev.driver->name,
			  cfa10049_fiq_data);
#endif

	ret = misc_register(&cfa10049_fiq_dev);
	if (ret)
		return ret;

	return 0;
}

static int cfafiq_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id cfafiq_of_match[] = {
	{ .compatible = "crystalfontz,cfa10049-fiq" },
	{}
};

static struct platform_driver cfafiq_driver = {
	.probe	= cfafiq_probe,
	.remove	= cfafiq_remove,
	.driver = {
		.name = "cfafiq",
		.of_match_table = of_match_ptr(cfafiq_of_match),
		.owner = THIS_MODULE,
	},
};

module_platform_driver(cfafiq_driver);

MODULE_DESCRIPTION("FIQ handler to driver the CFA-10049 stepper drivers");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_LICENSE("GPL");
