// SPDX-License-Identifier: GPL-2.0+
/*
 * PCIe host controller driver for Xilinx AXI PCIe Bridge
 *
 * Copyright (c) 2012 - 2014 Xilinx, Inc.
 *
 * Based on the Tegra PCIe driver
 *
 * Bits taken from Synopsys DesignWare Host controller driver and
 * ARM PCI Host generic driver.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/platform_device.h>
#include <linux/dma-direct.h>

#include "../pci.h"

/* Register definitions */
#define XILINX_PCIE_REG_BIR		0x00000130
#define XILINX_PCIE_REG_IDR		0x00000138
#define XILINX_PCIE_REG_IMR		0x0000013c
#define XILINX_PCIE_REG_PSCR		0x00000144
#define XILINX_PCIE_REG_RPSC		0x00000148
#define XILINX_PCIE_REG_MSIBASE1	0x0000014c
#define XILINX_PCIE_REG_MSIBASE2	0x00000150
#define XILINX_PCIE_REG_RPEFR		0x00000154
#define XILINX_PCIE_REG_RPIFR1		0x00000158
#define XILINX_PCIE_REG_RPIFR2		0x0000015c

/* Interrupt registers definitions */
#define XILINX_PCIE_INTR_LINK_DOWN	BIT(0)
#define XILINX_PCIE_INTR_ECRC_ERR	BIT(1)
#define XILINX_PCIE_INTR_STR_ERR	BIT(2)
#define XILINX_PCIE_INTR_HOT_RESET	BIT(3)
#define XILINX_PCIE_INTR_CFG_TIMEOUT	BIT(8)
#define XILINX_PCIE_INTR_CORRECTABLE	BIT(9)
#define XILINX_PCIE_INTR_NONFATAL	BIT(10)
#define XILINX_PCIE_INTR_FATAL		BIT(11)
#define XILINX_PCIE_INTR_INTX		BIT(16)
#define XILINX_PCIE_INTR_MSI		BIT(17)
#define XILINX_PCIE_INTR_SLV_UNSUPP	BIT(20)
#define XILINX_PCIE_INTR_SLV_UNEXP	BIT(21)
#define XILINX_PCIE_INTR_SLV_COMPL	BIT(22)
#define XILINX_PCIE_INTR_SLV_ERRP	BIT(23)
#define XILINX_PCIE_INTR_SLV_CMPABT	BIT(24)
#define XILINX_PCIE_INTR_SLV_ILLBUR	BIT(25)
#define XILINX_PCIE_INTR_MST_DECERR	BIT(26)
#define XILINX_PCIE_INTR_MST_SLVERR	BIT(27)
#define XILINX_PCIE_INTR_MST_ERRP	BIT(28)
#define XILINX_PCIE_IMR_ALL_MASK	0x1FF30FED
#define XILINX_PCIE_IMR_ENABLE_MASK	0x1FF30F0D
#define XILINX_PCIE_IDR_ALL_MASK	0xFFFFFFFF

/* Root Port Error FIFO Read Register definitions */
#define XILINX_PCIE_RPEFR_ERR_VALID	BIT(18)
#define XILINX_PCIE_RPEFR_REQ_ID	GENMASK(15, 0)
#define XILINX_PCIE_RPEFR_ALL_MASK	0xFFFFFFFF

/* Root Port Interrupt FIFO Read Register 1 definitions */
#define XILINX_PCIE_RPIFR1_INTR_VALID	BIT(31)
#define XILINX_PCIE_RPIFR1_MSI_INTR	BIT(30)
#define XILINX_PCIE_RPIFR1_INTR_MASK	GENMASK(28, 27)
#define XILINX_PCIE_RPIFR1_ALL_MASK	0xFFFFFFFF
#define XILINX_PCIE_RPIFR1_INTR_SHIFT	27

/* Bridge Info Register definitions */
#define XILINX_PCIE_BIR_ECAM_SZ_MASK	GENMASK(18, 16)
#define XILINX_PCIE_BIR_ECAM_SZ_SHIFT	16

/* Root Port Interrupt FIFO Read Register 2 definitions */
#define XILINX_PCIE_RPIFR2_MSG_DATA	GENMASK(15, 0)

/* Root Port Status/control Register definitions */
#define XILINX_PCIE_REG_RPSC_BEN	BIT(0)

/* Phy Status/Control Register definitions */
#define XILINX_PCIE_REG_PSCR_LNKUP	BIT(11)

/* Number of MSI IRQs */
#define XILINX_NUM_MSI_IRQS		128

/**
 * struct xilinx_pcie - PCIe port information
 * @dev: Device pointer
 * @reg_base: IO Mapped Register Base
 * @msi_map: Bitmap of allocated MSIs
 * @map_lock: Mutex protecting the MSI allocation
 * @msi_domain: MSI IRQ domain pointer
 * @leg_domain: Legacy IRQ domain pointer
 * @resources: Bus Resources
 */
struct xilinx_pcie {
	struct device *dev;
	void __iomem *reg_base;
	unsigned long msi_map[BITS_TO_LONGS(XILINX_NUM_MSI_IRQS)];
	struct mutex map_lock;
	struct irq_domain *msi_domain;
	struct irq_domain *leg_domain;
	struct list_head resources;
};

static inline u32 pcie_read(struct xilinx_pcie *pcie, u32 reg)
{
	return readl(pcie->reg_base + reg);
}

static inline void pcie_write(struct xilinx_pcie *pcie, u32 val, u32 reg)
{
	writel(val, pcie->reg_base + reg);
}

static inline bool xilinx_pcie_link_up(struct xilinx_pcie *pcie)
{
	return (pcie_read(pcie, XILINX_PCIE_REG_PSCR) &
		XILINX_PCIE_REG_PSCR_LNKUP) ? 1 : 0;
}

/**
 * xilinx_pcie_clear_err_interrupts - Clear Error Interrupts
 * @pcie: PCIe port information
 */
static void xilinx_pcie_clear_err_interrupts(struct xilinx_pcie *pcie)
{
	struct device *dev = pcie->dev;
	unsigned long val = pcie_read(pcie, XILINX_PCIE_REG_RPEFR);

	if (val & XILINX_PCIE_RPEFR_ERR_VALID) {
		dev_dbg(dev, "Requester ID %lu\n",
			val & XILINX_PCIE_RPEFR_REQ_ID);
		pcie_write(pcie, XILINX_PCIE_RPEFR_ALL_MASK,
			   XILINX_PCIE_REG_RPEFR);
	}
}

/**
 * xilinx_pcie_valid_device - Check if a valid device is present on bus
 * @bus: PCI Bus structure
 * @devfn: device/function
 *
 * Return: 'true' on success and 'false' if invalid device is found
 */
static bool xilinx_pcie_valid_device(struct pci_bus *bus, unsigned int devfn)
{
	struct xilinx_pcie *pcie = bus->sysdata;

	/* Check if link is up when trying to access downstream pcie ports */
	if (!pci_is_root_bus(bus)) {
		if (!xilinx_pcie_link_up(pcie))
			return false;
	} else if (devfn > 0) {
		/* Only one device down on each root port */
		return false;
	}
	return true;
}

/**
 * xilinx_pcie_map_bus - Get configuration base
 * @bus: PCI Bus structure
 * @devfn: Device/function
 * @where: Offset from base
 *
 * Return: Base address of the configuration space needed to be
 *	   accessed.
 */
static void __iomem *xilinx_pcie_map_bus(struct pci_bus *bus,
					 unsigned int devfn, int where)
{
	struct xilinx_pcie *pcie = bus->sysdata;

	if (!xilinx_pcie_valid_device(bus, devfn))
		return NULL;

	return pcie->reg_base + PCIE_ECAM_OFFSET(bus->number, devfn, where);
}

/* PCIe operations */
static struct pci_ops xilinx_pcie_ops = {
	.map_bus = xilinx_pcie_map_bus,
	.read	= pci_generic_config_read,
	.write	= pci_generic_config_write,
};

/* MSI functions */

static void xilinx_msi_top_irq_ack(struct irq_data *d)
{
	/*
	 * xilinx_pcie_intr_handler() will have performed the Ack.
	 * Eventually, this should be fixed and the Ack be moved in
	 * the respective callbacks for INTx and MSI.
	 */
}

static struct irq_chip xilinx_msi_top_chip = {
	.name		= "PCIe MSI",
	.irq_ack	= xilinx_msi_top_irq_ack,
};

static int xilinx_msi_set_affinity(struct irq_data *d, const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static void xilinx_compose_msi_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct xilinx_pcie *pcie = irq_data_get_irq_chip_data(data);
	phys_addr_t pa = ALIGN_DOWN(virt_to_phys(pcie), SZ_4K);

	msg->address_lo = lower_32_bits(pa);
	msg->address_hi = upper_32_bits(pa);
	msg->data = data->hwirq;
}

static struct irq_chip xilinx_msi_bottom_chip = {
	.name			= "Xilinx MSI",
	.irq_set_affinity 	= xilinx_msi_set_affinity,
	.irq_compose_msi_msg	= xilinx_compose_msi_msg,
};

static int xilinx_msi_domain_alloc(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs, void *args)
{
	struct xilinx_pcie *pcie = domain->host_data;
	int hwirq, i;

	mutex_lock(&pcie->map_lock);

	hwirq = bitmap_find_free_region(pcie->msi_map, XILINX_NUM_MSI_IRQS, order_base_2(nr_irqs));

	mutex_unlock(&pcie->map_lock);

	if (hwirq < 0)
		return -ENOSPC;

	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_info(domain, virq + i, hwirq + i,
				    &xilinx_msi_bottom_chip, domain->host_data,
				    handle_edge_irq, NULL, NULL);

	return 0;
}

static void xilinx_msi_domain_free(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct xilinx_pcie *pcie = domain->host_data;

	mutex_lock(&pcie->map_lock);

	bitmap_release_region(pcie->msi_map, d->hwirq, order_base_2(nr_irqs));

	mutex_unlock(&pcie->map_lock);
}

static const struct irq_domain_ops xilinx_msi_domain_ops = {
	.alloc	= xilinx_msi_domain_alloc,
	.free	= xilinx_msi_domain_free,
};

static struct msi_domain_info xilinx_msi_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS),
	.chip	= &xilinx_msi_top_chip,
};

static int xilinx_allocate_msi_domains(struct xilinx_pcie *pcie)
{
	struct fwnode_handle *fwnode = dev_fwnode(pcie->dev);
	struct irq_domain *parent;

	parent = irq_domain_create_linear(fwnode, XILINX_NUM_MSI_IRQS,
					  &xilinx_msi_domain_ops, pcie);
	if (!parent) {
		dev_err(pcie->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}
	irq_domain_update_bus_token(parent, DOMAIN_BUS_NEXUS);

	pcie->msi_domain = pci_msi_create_irq_domain(fwnode, &xilinx_msi_info, parent);
	if (!pcie->msi_domain) {
		dev_err(pcie->dev, "failed to create MSI domain\n");
		irq_domain_remove(parent);
		return -ENOMEM;
	}

	return 0;
}

static void xilinx_free_msi_domains(struct xilinx_pcie *pcie)
{
	struct irq_domain *parent = pcie->msi_domain->parent;

	irq_domain_remove(pcie->msi_domain);
	irq_domain_remove(parent);
}

/* INTx Functions */

/**
 * xilinx_pcie_intx_map - Set the handler for the INTx and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int xilinx_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
				irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

/* INTx IRQ Domain operations */
static const struct irq_domain_ops intx_domain_ops = {
	.map = xilinx_pcie_intx_map,
	.xlate = pci_irqd_intx_xlate,
};

/* PCIe HW Functions */

/**
 * xilinx_pcie_intr_handler - Interrupt Service Handler
 * @irq: IRQ number
 * @data: PCIe port information
 *
 * Return: IRQ_HANDLED on success and IRQ_NONE on failure
 */
static irqreturn_t xilinx_pcie_intr_handler(int irq, void *data)
{
	struct xilinx_pcie *pcie = (struct xilinx_pcie *)data;
	struct device *dev = pcie->dev;
	u32 val, mask, status;

	/* Read interrupt decode and mask registers */
	val = pcie_read(pcie, XILINX_PCIE_REG_IDR);
	mask = pcie_read(pcie, XILINX_PCIE_REG_IMR);

	status = val & mask;
	if (!status)
		return IRQ_NONE;

	if (status & XILINX_PCIE_INTR_LINK_DOWN)
		dev_warn(dev, "Link Down\n");

	if (status & XILINX_PCIE_INTR_ECRC_ERR)
		dev_warn(dev, "ECRC failed\n");

	if (status & XILINX_PCIE_INTR_STR_ERR)
		dev_warn(dev, "Streaming error\n");

	if (status & XILINX_PCIE_INTR_HOT_RESET)
		dev_info(dev, "Hot reset\n");

	if (status & XILINX_PCIE_INTR_CFG_TIMEOUT)
		dev_warn(dev, "ECAM access timeout\n");

	if (status & XILINX_PCIE_INTR_CORRECTABLE) {
		dev_warn(dev, "Correctable error message\n");
		xilinx_pcie_clear_err_interrupts(pcie);
	}

	if (status & XILINX_PCIE_INTR_NONFATAL) {
		dev_warn(dev, "Non fatal error message\n");
		xilinx_pcie_clear_err_interrupts(pcie);
	}

	if (status & XILINX_PCIE_INTR_FATAL) {
		dev_warn(dev, "Fatal error message\n");
		xilinx_pcie_clear_err_interrupts(pcie);
	}

	if (status & (XILINX_PCIE_INTR_INTX | XILINX_PCIE_INTR_MSI)) {
		struct irq_domain *domain;

		val = pcie_read(pcie, XILINX_PCIE_REG_RPIFR1);

		/* Check whether interrupt valid */
		if (!(val & XILINX_PCIE_RPIFR1_INTR_VALID)) {
			dev_warn(dev, "RP Intr FIFO1 read error\n");
			goto error;
		}

		/* Decode the IRQ number */
		if (val & XILINX_PCIE_RPIFR1_MSI_INTR) {
			val = pcie_read(pcie, XILINX_PCIE_REG_RPIFR2) &
				XILINX_PCIE_RPIFR2_MSG_DATA;
			domain = pcie->msi_domain->parent;
		} else {
			val = (val & XILINX_PCIE_RPIFR1_INTR_MASK) >>
				XILINX_PCIE_RPIFR1_INTR_SHIFT;
			domain = pcie->leg_domain;
		}

		/* Clear interrupt FIFO register 1 */
		pcie_write(pcie, XILINX_PCIE_RPIFR1_ALL_MASK,
			   XILINX_PCIE_REG_RPIFR1);

		generic_handle_domain_irq(domain, val);
	}

	if (status & XILINX_PCIE_INTR_SLV_UNSUPP)
		dev_warn(dev, "Slave unsupported request\n");

	if (status & XILINX_PCIE_INTR_SLV_UNEXP)
		dev_warn(dev, "Slave unexpected completion\n");

	if (status & XILINX_PCIE_INTR_SLV_COMPL)
		dev_warn(dev, "Slave completion timeout\n");

	if (status & XILINX_PCIE_INTR_SLV_ERRP)
		dev_warn(dev, "Slave Error Poison\n");

	if (status & XILINX_PCIE_INTR_SLV_CMPABT)
		dev_warn(dev, "Slave Completer Abort\n");

	if (status & XILINX_PCIE_INTR_SLV_ILLBUR)
		dev_warn(dev, "Slave Illegal Burst\n");

	if (status & XILINX_PCIE_INTR_MST_DECERR)
		dev_warn(dev, "Master decode error\n");

	if (status & XILINX_PCIE_INTR_MST_SLVERR)
		dev_warn(dev, "Master slave error\n");

	if (status & XILINX_PCIE_INTR_MST_ERRP)
		dev_warn(dev, "Master error poison\n");

error:
	/* Clear the Interrupt Decode register */
	pcie_write(pcie, status, XILINX_PCIE_REG_IDR);

	return IRQ_HANDLED;
}

/**
 * xilinx_pcie_init_irq_domain - Initialize IRQ domain
 * @pcie: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_init_irq_domain(struct xilinx_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *pcie_intc_node;
	int ret;

	/* Setup INTx */
	pcie_intc_node = of_get_next_child(dev->of_node, NULL);
	if (!pcie_intc_node) {
		dev_err(dev, "No PCIe Intc node found\n");
		return -ENODEV;
	}

	pcie->leg_domain = irq_domain_add_linear(pcie_intc_node, PCI_NUM_INTX,
						 &intx_domain_ops,
						 pcie);
	of_node_put(pcie_intc_node);
	if (!pcie->leg_domain) {
		dev_err(dev, "Failed to get a INTx IRQ domain\n");
		return -ENODEV;
	}

	/* Setup MSI */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		phys_addr_t pa = ALIGN_DOWN(virt_to_phys(pcie), SZ_4K);

		ret = xilinx_allocate_msi_domains(pcie);
		if (ret)
			return ret;

		pcie_write(pcie, upper_32_bits(pa), XILINX_PCIE_REG_MSIBASE1);
		pcie_write(pcie, lower_32_bits(pa), XILINX_PCIE_REG_MSIBASE2);
	}

	return 0;
}

/**
 * xilinx_pcie_init_port - Initialize hardware
 * @pcie: PCIe port information
 */
static void xilinx_pcie_init_port(struct xilinx_pcie *pcie)
{
	struct device *dev = pcie->dev;

	if (xilinx_pcie_link_up(pcie))
		dev_info(dev, "PCIe Link is UP\n");
	else
		dev_info(dev, "PCIe Link is DOWN\n");

	/* Disable all interrupts */
	pcie_write(pcie, ~XILINX_PCIE_IDR_ALL_MASK,
		   XILINX_PCIE_REG_IMR);

	/* Clear pending interrupts */
	pcie_write(pcie, pcie_read(pcie, XILINX_PCIE_REG_IDR) &
			 XILINX_PCIE_IMR_ALL_MASK,
		   XILINX_PCIE_REG_IDR);

	/* Enable all interrupts we handle */
	pcie_write(pcie, XILINX_PCIE_IMR_ENABLE_MASK, XILINX_PCIE_REG_IMR);

	/* Enable the Bridge enable bit */
	pcie_write(pcie, pcie_read(pcie, XILINX_PCIE_REG_RPSC) |
			 XILINX_PCIE_REG_RPSC_BEN,
		   XILINX_PCIE_REG_RPSC);
	printk("xilinx_pcie_init_port done\n");
}

/**
 * xilinx_pcie_parse_dt - Parse Device tree
 * @pcie: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_parse_dt(struct xilinx_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	struct resource regs;
	unsigned int irq;
	int err;

	err = of_address_to_resource(node, 0, &regs);
	if (err) {
		dev_err(dev, "missing \"reg\" property\n");
		return err;
	}

	pcie->reg_base = devm_pci_remap_cfg_resource(dev, &regs);
	if (IS_ERR(pcie->reg_base))
		return PTR_ERR(pcie->reg_base);

	irq = irq_of_parse_and_map(node, 0);
	printk("get irq: %x\n", irq);
	err = devm_request_irq(dev, irq, xilinx_pcie_intr_handler,
			       IRQF_SHARED | IRQF_NO_THREAD,
			       "xilinx-pcie", pcie);
	if (err) {
		dev_err(dev, "unable to request irq %d\n", irq);
		return err;
	}

	return 0;
}

struct metadata {
	uint64_t lo;
	uint64_t hi;
};

// static void dma_guard_check(void) {

	// for (int i = 0; i < 32768; i += 1000) {
	// 	pr_info("[bittervan] reading metadata %08x: %016llx%016llx\n", i, metadata[i].hi, metadata[i].lo);
	// }
// }

extern volatile struct dma_guard_metadata *dma_guard_metadata;

static inline void write_dma_guard_keyl(uint64_t keyl) {
	__asm__ volatile (
		"csrw dma_guard_keyl, %0"			\
		: : "r" (keyl)						\
		: "memory"							\
	);
}

static inline uint64_t read_dma_guard_keyl(void) {
	uint64_t ret;
	__asm__ volatile (
		"csrr %[asm_ret], dma_guard_keyl"	\
		: [asm_ret] "=r" (ret)				\
		:									\
		: "memory"							\
	);
	return ret;
}

static inline void write_dma_guard_keyh(uint64_t keyh) {
	__asm__ volatile (
		"csrw dma_guard_keyh, %0"			\
		: : "r" (keyh)						\
		: "memory"							\
	);
}

static inline uint64_t read_dma_guard_keyh(void) {
	uint64_t ret;
	__asm__ volatile (
		"csrr %[asm_ret], dma_guard_keyh"	\
		: [asm_ret] "=r" (ret)				\
		:									\
		: "memory"							\
	);
	return ret;
}

static inline void dma_guard_initialize(struct platform_device *pdev) {
	uint64_t dma_guard_keyh;
	uint64_t dma_guard_keyl;
	void* buf_for_vc709 = kmalloc(0x100000, GFP_KERNEL);
	dma_addr_t handle_for_vc709;

	if (!dma_guard_metadata) { // Not initialized, initialize key, the metadata reset is conducted by the hardware itself
		uint64_t* dma_guard_key_slot = ioremap(DMA_GUARD_KEY_HARDWARE_ADDR, 0x200000);
		// uint64_t* dma_guard_key_slot = kmalloc(0x200000, GFP_KERNEL);
		
		// lower 64 bit
		// uint64_t key = get_random_u64();

		// for vc709 test
		uint64_t key = 0;
		*dma_guard_key_slot = key;
		dma_guard_keyl = key;
		write_dma_guard_keyl(key);
		// upper 64 bit	
		// key = get_random_u64();
		*(dma_guard_key_slot + 1) = key;
		dma_guard_keyh = key;
		write_dma_guard_keyh(key);

		// write_dma_guard_keyl(0xdead0086beef1234UL);
		// pr_info("write keyl done\n");
		// read_keyl = read_dma_guard_keyl();
		// pr_info("get keyl: %016llx\n", read_keyl);

		pr_info("************ All The Keys Are Initialized! **************\n");

		pr_info("In kernel: low: %016llx, high: %016llx\n", dma_guard_keyl, dma_guard_keyh);
		pr_info("In csrs  : low: %016llx, high: %016llx\n", read_dma_guard_keyl(), read_dma_guard_keyh());
		pr_info("In guard : low: %016llx, high: %016llx\n", *(dma_guard_key_slot), *(dma_guard_key_slot + 1));

		pr_info("************ All The Keys Are Initialized! **************\n");

		// pr_info("test sign: 0x87654321, with 0xdeadbeefcafebabe: %016llx\n", dma_guard_sign(0x87654321, 0xdeadbeefcafebabe));
		// pr_info("test sign: 0x87654321, with 0xdeadbeefcafebabe: %016llx\n", dma_guard_sign(0x87654321, 0xdeadbeefcafebabe));
		// pr_info("test sign: 0x87654321, with 0xcafebabedeadbeef: %016llx\n", dma_guard_sign(0x87654321, 0xcafebabedeadbeef));

		dma_guard_metadata = (void*)dma_guard_key_slot + 0x100000;

		for (int i = 0; i < 0x10000; i++) {
			dma_guard_metadata[i].attributes = 0;
		}

		// pr_info("dev_name(&pdev->dev): %s\n", dev_name(&pdev->dev));
		dma_addr_t dma_guard_map(struct device* dev, dma_addr_t dma_handle, size_t size, enum dma_data_direction direction);
		((char*)dev_name(&pdev->dev))[4] = '1';
		handle_for_vc709 = dma_guard_map(&(pdev->dev), 0xc0000000, 0x10000000, DMA_BIDIRECTIONAL);
		((char*)dev_name(&pdev->dev))[4] = '0';
		pr_info("*********** handle for vc709: %016llx ************\n", handle_for_vc709);
	}
}

/**
 * xilinx_pcie_probe - Probe function
 * @pdev: Platform device pointer
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct xilinx_pcie *pcie;
	struct pci_host_bridge *bridge;
	int err;
	// dma_guard_check();

	dma_guard_initialize(pdev);	

	if (!dev->of_node)
		return -ENODEV;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENODEV;

	pcie = pci_host_bridge_priv(bridge);
	mutex_init(&pcie->map_lock);
	pcie->dev = dev;

	err = xilinx_pcie_parse_dt(pcie);
	if (err) {
		dev_err(dev, "Parsing DT failed\n");
		return err;
	}

	xilinx_pcie_init_port(pcie);

	err = xilinx_pcie_init_irq_domain(pcie);
	if (err) {
		dev_err(dev, "Failed creating IRQ Domain\n");
		return err;
	}

	bridge->sysdata = pcie;
	bridge->ops = &xilinx_pcie_ops;

	err = pci_host_probe(bridge);
	if (err)
		xilinx_free_msi_domains(pcie);

	return err;
}

static const struct of_device_id xilinx_pcie_of_match[] = {
	{ .compatible = "xlnx,axi-pcie-host-1.00.a", },
	{}
};

static struct platform_driver xilinx_pcie_driver = {
	.driver = {
		.name = "xilinx-pcie",
		.of_match_table = xilinx_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = xilinx_pcie_probe,
};
builtin_platform_driver(xilinx_pcie_driver);
