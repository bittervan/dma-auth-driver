// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018-2020 Christoph Hellwig.
 *
 * DMA operations that map physical memory directly without using an IOMMU.
 */
#include <linux/memblock.h> /* for max_pfn */
#include <linux/export.h>
#include <linux/mm.h>
#include <linux/dma-map-ops.h>
#include <linux/scatterlist.h>
#include <linux/pfn.h>
#include <linux/vmalloc.h>
#include <linux/set_memory.h>
#include <linux/slab.h>
#include "direct.h"

/*
 * Most architectures use ZONE_DMA for the first 16 Megabytes, but some use
 * it for entirely different regions. In that case the arch code needs to
 * override the variable below for dma-direct to work properly.
 */
unsigned int zone_dma_bits __ro_after_init = 24;

static inline dma_addr_t phys_to_dma_direct(struct device *dev,
		phys_addr_t phys)
{
	if (force_dma_unencrypted(dev))
		return phys_to_dma_unencrypted(dev, phys);
	return phys_to_dma(dev, phys);
}

static inline struct page *dma_direct_to_page(struct device *dev,
		dma_addr_t dma_addr)
{
	return pfn_to_page(PHYS_PFN(dma_to_phys(dev, dma_addr)));
}

u64 dma_direct_get_required_mask(struct device *dev)
{
	phys_addr_t phys = (phys_addr_t)(max_pfn - 1) << PAGE_SHIFT;
	u64 max_dma = phys_to_dma_direct(dev, phys);

	return (1ULL << (fls64(max_dma) - 1)) * 2 - 1;
}

static gfp_t dma_direct_optimal_gfp_mask(struct device *dev, u64 dma_mask,
				  u64 *phys_limit)
{
	u64 dma_limit = min_not_zero(dma_mask, dev->bus_dma_limit);

	/*
	 * Optimistically try the zone that the physical address mask falls
	 * into first.  If that returns memory that isn't actually addressable
	 * we will fallback to the next lower zone and try again.
	 *
	 * Note that GFP_DMA32 and GFP_DMA are no ops without the corresponding
	 * zones.
	 */
	*phys_limit = dma_to_phys(dev, dma_limit);
	if (*phys_limit <= DMA_BIT_MASK(zone_dma_bits))
		return GFP_DMA;
	if (*phys_limit <= DMA_BIT_MASK(32))
		return GFP_DMA32;
	return 0;
}

static bool dma_coherent_ok(struct device *dev, phys_addr_t phys, size_t size)
{
	dma_addr_t dma_addr = phys_to_dma_direct(dev, phys);

	if (dma_addr == DMA_MAPPING_ERROR)
		return false;
	return dma_addr + size - 1 <=
		min_not_zero(dev->coherent_dma_mask, dev->bus_dma_limit);
}

static int dma_set_decrypted(struct device *dev, void *vaddr, size_t size)
{
	if (!force_dma_unencrypted(dev))
		return 0;
	return set_memory_decrypted((unsigned long)vaddr, PFN_UP(size));
}

static int dma_set_encrypted(struct device *dev, void *vaddr, size_t size)
{
	int ret;

	if (!force_dma_unencrypted(dev))
		return 0;
	ret = set_memory_encrypted((unsigned long)vaddr, PFN_UP(size));
	if (ret)
		pr_warn_ratelimited("leaking DMA memory that can't be re-encrypted\n");
	return ret;
}

static void __dma_direct_free_pages(struct device *dev, struct page *page,
				    size_t size)
{
	if (swiotlb_free(dev, page, size))
		return;
	dma_free_contiguous(dev, page, size);
}

static struct page *dma_direct_alloc_swiotlb(struct device *dev, size_t size)
{
	struct page *page = swiotlb_alloc(dev, size);

	if (page && !dma_coherent_ok(dev, page_to_phys(page), size)) {
		swiotlb_free(dev, page, size);
		return NULL;
	}

	return page;
}

static struct page *__dma_direct_alloc_pages(struct device *dev, size_t size,
		gfp_t gfp, bool allow_highmem)
{
	int node = dev_to_node(dev);
	struct page *page = NULL;
	u64 phys_limit;

	WARN_ON_ONCE(!PAGE_ALIGNED(size));

	if (is_swiotlb_for_alloc(dev))
		return dma_direct_alloc_swiotlb(dev, size);

	gfp |= dma_direct_optimal_gfp_mask(dev, dev->coherent_dma_mask,
					   &phys_limit);
	page = dma_alloc_contiguous(dev, size, gfp);
	if (page) {
		if (!dma_coherent_ok(dev, page_to_phys(page), size) ||
		    (!allow_highmem && PageHighMem(page))) {
			dma_free_contiguous(dev, page, size);
			page = NULL;
		}
	}
again:
	if (!page)
		page = alloc_pages_node(node, gfp, get_order(size));
	if (page && !dma_coherent_ok(dev, page_to_phys(page), size)) {
		dma_free_contiguous(dev, page, size);
		page = NULL;

		if (IS_ENABLED(CONFIG_ZONE_DMA32) &&
		    phys_limit < DMA_BIT_MASK(64) &&
		    !(gfp & (GFP_DMA32 | GFP_DMA))) {
			gfp |= GFP_DMA32;
			goto again;
		}

		if (IS_ENABLED(CONFIG_ZONE_DMA) && !(gfp & GFP_DMA)) {
			gfp = (gfp & ~GFP_DMA32) | GFP_DMA;
			goto again;
		}
	}

	return page;
}

/*
 * Check if a potentially blocking operations needs to dip into the atomic
 * pools for the given device/gfp.
 */
static bool dma_direct_use_pool(struct device *dev, gfp_t gfp)
{
	return !gfpflags_allow_blocking(gfp) && !is_swiotlb_for_alloc(dev);
}

static void *dma_direct_alloc_from_pool(struct device *dev, size_t size,
		dma_addr_t *dma_handle, gfp_t gfp)
{
	struct page *page;
	u64 phys_mask;
	void *ret;

	if (WARN_ON_ONCE(!IS_ENABLED(CONFIG_DMA_COHERENT_POOL)))
		return NULL;

	gfp |= dma_direct_optimal_gfp_mask(dev, dev->coherent_dma_mask,
					   &phys_mask);
	page = dma_alloc_from_pool(dev, size, &ret, gfp, dma_coherent_ok);
	if (!page)
		return NULL;
	*dma_handle = phys_to_dma_direct(dev, page_to_phys(page));
	return ret;
}

static void *dma_direct_alloc_no_mapping(struct device *dev, size_t size,
		dma_addr_t *dma_handle, gfp_t gfp)
{
	struct page *page;

	page = __dma_direct_alloc_pages(dev, size, gfp & ~__GFP_ZERO, true);
	if (!page)
		return NULL;

	/* remove any dirty cache lines on the kernel alias */
	if (!PageHighMem(page))
		arch_dma_prep_coherent(page, size);

	/* return the page pointer as the opaque cookie */
	*dma_handle = phys_to_dma_direct(dev, page_to_phys(page));
	return page;
}

void *dma_direct_alloc(struct device *dev, size_t size,
		dma_addr_t *dma_handle, gfp_t gfp, unsigned long attrs)
{
	bool remap = false, set_uncached = false;
	struct page *page;
	void *ret;

	size = PAGE_ALIGN(size);
	if (attrs & DMA_ATTR_NO_WARN)
		gfp |= __GFP_NOWARN;

	if ((attrs & DMA_ATTR_NO_KERNEL_MAPPING) &&
	    !force_dma_unencrypted(dev) && !is_swiotlb_for_alloc(dev))
		return dma_direct_alloc_no_mapping(dev, size, dma_handle, gfp);

	if (!dev_is_dma_coherent(dev)) {
		/*
		 * Fallback to the arch handler if it exists.  This should
		 * eventually go away.
		 */
		if (!IS_ENABLED(CONFIG_ARCH_HAS_DMA_SET_UNCACHED) &&
		    !IS_ENABLED(CONFIG_DMA_DIRECT_REMAP) &&
		    !IS_ENABLED(CONFIG_DMA_GLOBAL_POOL) &&
		    !is_swiotlb_for_alloc(dev))
			return arch_dma_alloc(dev, size, dma_handle, gfp,
					      attrs);

		/*
		 * If there is a global pool, always allocate from it for
		 * non-coherent devices.
		 */
		if (IS_ENABLED(CONFIG_DMA_GLOBAL_POOL))
			return dma_alloc_from_global_coherent(dev, size,
					dma_handle);

		/*
		 * Otherwise remap if the architecture is asking for it.  But
		 * given that remapping memory is a blocking operation we'll
		 * instead have to dip into the atomic pools.
		 */
		remap = IS_ENABLED(CONFIG_DMA_DIRECT_REMAP);
		if (remap) {
			if (dma_direct_use_pool(dev, gfp))
				return dma_direct_alloc_from_pool(dev, size,
						dma_handle, gfp);
		} else {
			if (!IS_ENABLED(CONFIG_ARCH_HAS_DMA_SET_UNCACHED))
				return NULL;
			set_uncached = true;
		}
	}

	/*
	 * Decrypting memory may block, so allocate the memory from the atomic
	 * pools if we can't block.
	 */
	if (force_dma_unencrypted(dev) && dma_direct_use_pool(dev, gfp))
		return dma_direct_alloc_from_pool(dev, size, dma_handle, gfp);

	/* we always manually zero the memory once we are done */
	page = __dma_direct_alloc_pages(dev, size, gfp & ~__GFP_ZERO, true);
	if (!page)
		return NULL;

	/*
	 * dma_alloc_contiguous can return highmem pages depending on a
	 * combination the cma= arguments and per-arch setup.  These need to be
	 * remapped to return a kernel virtual address.
	 */
	if (PageHighMem(page)) {
		remap = true;
		set_uncached = false;
	}

	if (remap) {
		pgprot_t prot = dma_pgprot(dev, PAGE_KERNEL, attrs);

		if (force_dma_unencrypted(dev))
			prot = pgprot_decrypted(prot);

		/* remove any dirty cache lines on the kernel alias */
		arch_dma_prep_coherent(page, size);

		/* create a coherent mapping */
		ret = dma_common_contiguous_remap(page, size, prot,
				__builtin_return_address(0));
		if (!ret)
			goto out_free_pages;
	} else {
		ret = page_address(page);
		if (dma_set_decrypted(dev, ret, size))
			goto out_free_pages;
	}

	memset(ret, 0, size);

	if (set_uncached) {
		arch_dma_prep_coherent(page, size);
		ret = arch_dma_set_uncached(ret, size);
		if (IS_ERR(ret))
			goto out_encrypt_pages;
	}

	*dma_handle = phys_to_dma_direct(dev, page_to_phys(page));
	return ret;

out_encrypt_pages:
	if (dma_set_encrypted(dev, page_address(page), size))
		return NULL;
out_free_pages:
	__dma_direct_free_pages(dev, page, size);
	return NULL;
}

void dma_direct_free(struct device *dev, size_t size,
		void *cpu_addr, dma_addr_t dma_addr, unsigned long attrs)
{
	unsigned int page_order = get_order(size);

	if ((attrs & DMA_ATTR_NO_KERNEL_MAPPING) &&
	    !force_dma_unencrypted(dev) && !is_swiotlb_for_alloc(dev)) {
		/* cpu_addr is a struct page cookie, not a kernel address */
		dma_free_contiguous(dev, cpu_addr, size);
		return;
	}

	if (!IS_ENABLED(CONFIG_ARCH_HAS_DMA_SET_UNCACHED) &&
	    !IS_ENABLED(CONFIG_DMA_DIRECT_REMAP) &&
	    !IS_ENABLED(CONFIG_DMA_GLOBAL_POOL) &&
	    !dev_is_dma_coherent(dev) &&
	    !is_swiotlb_for_alloc(dev)) {
		arch_dma_free(dev, size, cpu_addr, dma_addr, attrs);
		return;
	}

	if (IS_ENABLED(CONFIG_DMA_GLOBAL_POOL) &&
	    !dev_is_dma_coherent(dev)) {
		if (!dma_release_from_global_coherent(page_order, cpu_addr))
			WARN_ON_ONCE(1);
		return;
	}

	/* If cpu_addr is not from an atomic pool, dma_free_from_pool() fails */
	if (IS_ENABLED(CONFIG_DMA_COHERENT_POOL) &&
	    dma_free_from_pool(dev, cpu_addr, PAGE_ALIGN(size)))
		return;

	if (is_vmalloc_addr(cpu_addr)) {
		vunmap(cpu_addr);
	} else {
		if (IS_ENABLED(CONFIG_ARCH_HAS_DMA_CLEAR_UNCACHED))
			arch_dma_clear_uncached(cpu_addr, size);
		if (dma_set_encrypted(dev, cpu_addr, size))
			return;
	}

	__dma_direct_free_pages(dev, dma_direct_to_page(dev, dma_addr), size);
}

struct page *dma_direct_alloc_pages(struct device *dev, size_t size,
		dma_addr_t *dma_handle, enum dma_data_direction dir, gfp_t gfp)
{
	struct page *page;
	void *ret;

	if (force_dma_unencrypted(dev) && dma_direct_use_pool(dev, gfp))
		return dma_direct_alloc_from_pool(dev, size, dma_handle, gfp);

	page = __dma_direct_alloc_pages(dev, size, gfp, false);
	if (!page)
		return NULL;

	ret = page_address(page);
	if (dma_set_decrypted(dev, ret, size))
		goto out_free_pages;
	memset(ret, 0, size);
	*dma_handle = phys_to_dma_direct(dev, page_to_phys(page));
	return page;
out_free_pages:
	__dma_direct_free_pages(dev, page, size);
	return NULL;
}

void dma_direct_free_pages(struct device *dev, size_t size,
		struct page *page, dma_addr_t dma_addr,
		enum dma_data_direction dir)
{
	void *vaddr = page_address(page);

	/* If cpu_addr is not from an atomic pool, dma_free_from_pool() fails */
	if (IS_ENABLED(CONFIG_DMA_COHERENT_POOL) &&
	    dma_free_from_pool(dev, vaddr, size))
		return;

	if (dma_set_encrypted(dev, vaddr, size))
		return;
	__dma_direct_free_pages(dev, page, size);
}

#if defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_DEVICE) || \
    defined(CONFIG_SWIOTLB)
void dma_direct_sync_sg_for_device(struct device *dev,
		struct scatterlist *sgl, int nents, enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

	for_each_sg(sgl, sg, nents, i) {
		phys_addr_t paddr = dma_to_phys(dev, sg_dma_address(sg));

		if (unlikely(is_swiotlb_buffer(dev, paddr)))
			swiotlb_sync_single_for_device(dev, paddr, sg->length,
						       dir);

		if (!dev_is_dma_coherent(dev))
			arch_sync_dma_for_device(paddr, sg->length,
					dir);
	}
}
#endif

#if defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU) || \
    defined(CONFIG_ARCH_HAS_SYNC_DMA_FOR_CPU_ALL) || \
    defined(CONFIG_SWIOTLB)
void dma_direct_sync_sg_for_cpu(struct device *dev,
		struct scatterlist *sgl, int nents, enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

	for_each_sg(sgl, sg, nents, i) {
		phys_addr_t paddr = dma_to_phys(dev, sg_dma_address(sg));

		if (!dev_is_dma_coherent(dev))
			arch_sync_dma_for_cpu(paddr, sg->length, dir);

		if (unlikely(is_swiotlb_buffer(dev, paddr)))
			swiotlb_sync_single_for_cpu(dev, paddr, sg->length,
						    dir);

		if (dir == DMA_FROM_DEVICE)
			arch_dma_mark_clean(paddr, sg->length);
	}

	if (!dev_is_dma_coherent(dev))
		arch_sync_dma_for_cpu_all();
}

extern int dma_guard_mapping_count;

/*
 * Unmaps segments, except for ones marked as pci_p2pdma which do not
 * require any further action as they contain a bus address.
 */
void dma_direct_unmap_sg(struct device *dev, struct scatterlist *sgl,
		int nents, enum dma_data_direction dir, unsigned long attrs)
{
	struct scatterlist *sg;
	int i;

	for_each_sg(sgl,  sg, nents, i) {
		if (sg_is_dma_bus_address(sg))
			sg_dma_unmark_bus_address(sg);
		else {
			sg->dma_address = dma_guard_unmap(dev, sg->dma_address, sg_dma_len(sg));
			dma_direct_unmap_page(dev, sg->dma_address,
					      sg_dma_len(sg), dir, attrs);
		}
	}
}
#endif

int dma_direct_map_sg(struct device *dev, struct scatterlist *sgl, int nents,
		enum dma_data_direction dir, unsigned long attrs)
{
	struct pci_p2pdma_map_state p2pdma_state = {};
	enum pci_p2pdma_map_type map;
	struct scatterlist *sg;
	int i, ret;

	for_each_sg(sgl, sg, nents, i) {
		if (is_pci_p2pdma_page(sg_page(sg))) {
			map = pci_p2pdma_map_segment(&p2pdma_state, dev, sg);
			switch (map) {
			case PCI_P2PDMA_MAP_BUS_ADDR:
				continue;
			case PCI_P2PDMA_MAP_THRU_HOST_BRIDGE:
				/*
				 * Any P2P mapping that traverses the PCI
				 * host bridge must be mapped with CPU physical
				 * address and not PCI bus addresses. This is
				 * done with dma_direct_map_page() below.
				 */
				break;
			default:
				ret = -EREMOTEIO;
				goto out_unmap;
			}
		}

		sg->dma_address = dma_direct_map_page(dev, sg_page(sg),
				sg->offset, sg->length, dir, attrs);
		if (sg->dma_address == DMA_MAPPING_ERROR) {
			ret = -EIO;
			goto out_unmap;
		}

		sg->dma_address = dma_guard_map(dev, sg->dma_address, sg->length, dir);
		sg_dma_len(sg) = sg->length;
	}

	return nents;

out_unmap:
	dma_direct_unmap_sg(dev, sgl, i, dir, attrs | DMA_ATTR_SKIP_CPU_SYNC);
	return ret;
}

dma_addr_t dma_direct_map_resource(struct device *dev, phys_addr_t paddr,
		size_t size, enum dma_data_direction dir, unsigned long attrs)
{
	dma_addr_t dma_addr = paddr;

	if (unlikely(!dma_capable(dev, dma_addr, size, false))) {
		dev_err_once(dev,
			     "DMA addr %pad+%zu overflow (mask %llx, bus limit %llx).\n",
			     &dma_addr, size, *dev->dma_mask, dev->bus_dma_limit);
		WARN_ON_ONCE(1);
		return DMA_MAPPING_ERROR;
	}

	return dma_addr;
}

int dma_direct_get_sgtable(struct device *dev, struct sg_table *sgt,
		void *cpu_addr, dma_addr_t dma_addr, size_t size,
		unsigned long attrs)
{
	struct page *page = dma_direct_to_page(dev, dma_addr);
	int ret;

	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (!ret)
		sg_set_page(sgt->sgl, page, PAGE_ALIGN(size), 0);
	return ret;
}

bool dma_direct_can_mmap(struct device *dev)
{
	return dev_is_dma_coherent(dev) ||
		IS_ENABLED(CONFIG_DMA_NONCOHERENT_MMAP);
}

int dma_direct_mmap(struct device *dev, struct vm_area_struct *vma,
		void *cpu_addr, dma_addr_t dma_addr, size_t size,
		unsigned long attrs)
{
	unsigned long user_count = vma_pages(vma);
	unsigned long count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	unsigned long pfn = PHYS_PFN(dma_to_phys(dev, dma_addr));
	int ret = -ENXIO;

	vma->vm_page_prot = dma_pgprot(dev, vma->vm_page_prot, attrs);
	if (force_dma_unencrypted(dev))
		vma->vm_page_prot = pgprot_decrypted(vma->vm_page_prot);

	if (dma_mmap_from_dev_coherent(dev, vma, cpu_addr, size, &ret))
		return ret;
	if (dma_mmap_from_global_coherent(vma, cpu_addr, size, &ret))
		return ret;

	if (vma->vm_pgoff >= count || user_count > count - vma->vm_pgoff)
		return -ENXIO;
	return remap_pfn_range(vma, vma->vm_start, pfn + vma->vm_pgoff,
			user_count << PAGE_SHIFT, vma->vm_page_prot);
}

int dma_direct_supported(struct device *dev, u64 mask)
{
	u64 min_mask = (max_pfn - 1) << PAGE_SHIFT;

	/*
	 * Because 32-bit DMA masks are so common we expect every architecture
	 * to be able to satisfy them - either by not supporting more physical
	 * memory, or by providing a ZONE_DMA32.  If neither is the case, the
	 * architecture needs to use an IOMMU instead of the direct mapping.
	 */
	if (mask >= DMA_BIT_MASK(32))
		return 1;

	/*
	 * This check needs to be against the actual bit mask value, so use
	 * phys_to_dma_unencrypted() here so that the SME encryption mask isn't
	 * part of the check.
	 */
	if (IS_ENABLED(CONFIG_ZONE_DMA))
		min_mask = min_t(u64, min_mask, DMA_BIT_MASK(zone_dma_bits));
	return mask >= phys_to_dma_unencrypted(dev, min_mask);
}

size_t dma_direct_max_mapping_size(struct device *dev)
{
	/* If SWIOTLB is active, use its maximum mapping size */
	if (is_swiotlb_active(dev) &&
	    (dma_addressing_limited(dev) || is_swiotlb_force_bounce(dev)))
		return swiotlb_max_mapping_size(dev);
	return SIZE_MAX;
}

bool dma_direct_need_sync(struct device *dev, dma_addr_t dma_addr)
{
	return !dev_is_dma_coherent(dev) ||
	       is_swiotlb_buffer(dev, dma_to_phys(dev, dma_addr));
}

/**
 * dma_direct_set_offset - Assign scalar offset for a single DMA range.
 * @dev:	device pointer; needed to "own" the alloced memory.
 * @cpu_start:  beginning of memory region covered by this offset.
 * @dma_start:  beginning of DMA/PCI region covered by this offset.
 * @size:	size of the region.
 *
 * This is for the simple case of a uniform offset which cannot
 * be discovered by "dma-ranges".
 *
 * It returns -ENOMEM if out of memory, -EINVAL if a map
 * already exists, 0 otherwise.
 *
 * Note: any call to this from a driver is a bug.  The mapping needs
 * to be described by the device tree or other firmware interfaces.
 */
int dma_direct_set_offset(struct device *dev, phys_addr_t cpu_start,
			 dma_addr_t dma_start, u64 size)
{
	struct bus_dma_region *map;
	u64 offset = (u64)cpu_start - (u64)dma_start;

	if (dev->dma_range_map) {
		dev_err(dev, "attempt to add DMA range to existing map\n");
		return -EINVAL;
	}

	if (!offset)
		return 0;

	map = kcalloc(2, sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;
	map[0].cpu_start = cpu_start;
	map[0].dma_start = dma_start;
	map[0].offset = offset;
	map[0].size = size;
	dev->dma_range_map = map;
	return 0;
}

volatile struct dma_guard_metadata *dma_guard_metadata = NULL;
volatile uint64_t dma_guard_keyh;
volatile uint64_t dma_guard_keyl;
volatile int _mapping_count = 0;

volatile uint64_t total_map_operations = 0;

// uint16_t dma_guard_hash(dma_addr_t dma_handle, struct dma_guard_metadata metadata) {
// 	uint16_t ret;
// 	uint32_t xor;
// 	xor = metadata.attr ^ metadata.identifier ^ metadata.lower_bound ^ metadata.upper_bound;
// 	xor = xor ^ dma_guard_keyl;
// 	ret = (xor >> 16) ^ (dma_handle >> ((metadata.attr << 2) >> 27));
// 	return ret;
// };

// void dma_guard_dump(void) {
// 	pr_info("------------------------------------------\n");
// 	for (int i = 0; i < 0x10000; i++) {
// 		if (dma_guard_metadata[i].attr) {
// 			pr_info("[%04x] %08x %08x %08x %08x\n", i, dma_guard_metadata[i].attr, dma_guard_metadata[i].identifier, dma_guard_metadata[i].lower_bound, dma_guard_metadata[i].upper_bound);
// 		}
// 	}
// }

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

uint64_t dma_guard_sign(uint64_t dma_pointer, uint64_t modifier) {
	uint64_t ret;
	__asm__ volatile (
		".byte 0x6b, 0x05, 0xb5, 0x00"
		: "=r" (ret)
		: "r" (dma_pointer), "r" (modifier)
		: "a1"
	);
	return ret;
}

// Take the bare DMA pointer as input, create the mapping, and return the DMAGuard pointer
__attribute__((optimize("O0"))) dma_addr_t dma_guard_map(struct device *dev, dma_addr_t dma_handle, size_t size, enum dma_data_direction direction) {
	dma_addr_t ret;
	uint32_t offset_length = 0;
	struct dma_guard_metadata metadata;
	uint16_t hash;
	uint64_t signed_pointer;
	uint64_t upper_bound;
	uint64_t identifier;
	// uint64_t read_keyl;

	if (dev_name(dev)[4] == '0') {
		return dma_handle;
	}

	// if (!dma_guard_metadata) { // Not initialized, initialize key, the metadata reset is conducted by the hardware itself
	// 	uint64_t* dma_guard_key_slot = ioremap(DMA_GUARD_KEY_HARDWARE_ADDR, 0x200000);
	// 	// uint64_t* dma_guard_key_slot = kmalloc(0x200000, GFP_KERNEL);
		
	// 	// lower 64 bit
	// 	uint64_t key = get_random_u64();
	// 	*dma_guard_key_slot = key;
	// 	dma_guard_keyl = key;
	// 	write_dma_guard_keyl(key);
	// 	// upper 64 bit	
	// 	key = get_random_u64();
	// 	*(dma_guard_key_slot + 1) = key;
	// 	dma_guard_keyh = key;
	// 	write_dma_guard_keyh(key);

	// 	// write_dma_guard_keyl(0xdead0086beef1234UL);
	// 	// pr_info("write keyl done\n");
	// 	// read_keyl = read_dma_guard_keyl();
	// 	// pr_info("get keyl: %016llx\n", read_keyl);

	// 	pr_info("************ All The Keys Are Initialized! **************\n");

	// 	pr_info("In kernel: low: %016llx, high: %016llx\n", dma_guard_keyl, dma_guard_keyh);
	// 	pr_info("In csrs  : low: %016llx, high: %016llx\n", read_dma_guard_keyl(), read_dma_guard_keyh());
	// 	pr_info("In guard : low: %016llx, high: %016llx\n", *(dma_guard_key_slot), *(dma_guard_key_slot + 1));

	// 	pr_info("************ All The Keys Are Initialized! **************\n");

	// 	pr_info("test sign: 0x87654321, with 0xdeadbeefcafebabe: %016llx\n", dma_guard_sign(0x87654321, 0xdeadbeefcafebabe));
	// 	pr_info("test sign: 0x87654321, with 0xdeadbeefcafebabe: %016llx\n", dma_guard_sign(0x87654321, 0xdeadbeefcafebabe));
	// 	pr_info("test sign: 0x87654321, with 0xcafebabedeadbeef: %016llx\n", dma_guard_sign(0x87654321, 0xcafebabedeadbeef));

	// 	dma_guard_metadata = (void*)dma_guard_key_slot + 0x100000;

	// 	for (int i = 0; i < 0x10000; i++) {
	// 		dma_guard_metadata[i].attributes = 0;
	// 	}
	// }

	upper_bound = dma_handle + size - 1;
	metadata.lower_boundl = lower_32_bits(dma_handle);
	metadata.upper_boundl = lower_32_bits(upper_bound);
	metadata.lower_boundh = (uint16_t)(dma_handle >> 32);
	metadata.upper_boundh = (uint16_t)(upper_bound >> 32);

	// get common prefix length
	for (int i = 16; i < 48; i++) {
		if (dma_handle >> i == upper_bound >> i) {
			offset_length = i;
			break;			
		}
	}
	
	// Here, 00 means no access
	// 11 mean r/w, 01 mean wo, 10 means ro 
	// metadata.attr = ((~direction) << 30) | (offset_length << 25) | 0;
	metadata.attributes = (uint8_t)((~direction) << 6 | offset_length);

	_mapping_count++;

	while (1) {
		// metadata.identifier = get_random_u32();
		identifier = get_random_u32();
		metadata.identifierh = (uint8_t)(identifier >> 24);
		metadata.identifierl = lower_16_bits(identifier);
		// hash = dma_guard_hash(dma_handle, metadata);
		// pr_info("***hash: %x\n", hash);
		signed_pointer = dma_guard_sign((dma_handle >> offset_length) << offset_length, (*(uint64_t*)&metadata) ^ (*(((uint64_t*)&metadata) + 1)));
		// pr_info("signed_pointer: %016llx, sign using: %016llx, %016llx, %016llx\n", signed_pointer, (*(uint64_t*)&metadata), (*(((uint64_t*)&metadata) + 1)), (*(uint64_t*)&metadata) ^ (*(((uint64_t*)&metadata) + 1)));
		// while (1);
		// pr_info("metadata generated: hi: %016llx lo: %016llx\n", (*(((uint64_t*)&metadata) + 1)), (*(uint64_t*)&metadata));
		hash = signed_pointer >> 48;
		if (dma_guard_metadata[hash].attributes == 0) {
			dma_guard_metadata[hash] = metadata;
			// BUG_ON(dma_guard_metadata[hash].attr == 0);
			// if (
			// 	(dma_guard_metadata[hash].attr != metadata.attr) ||
			// 	(dma_guard_metadata[hash].identifier != metadata.identifier) ||
			// 	(dma_guard_metadata[hash].lower_bound != metadata.lower_bound) ||
			// 	(dma_guard_metadata[hash].upper_bound != metadata.upper_bound)
			// ) {
			// 	pr_info("Not written correctly, in hardware: %08x %08x %08x %08x, pointer: %016llx, total mapping: %d\n",  dma_guard_metadata[hash].attr, dma_guard_metadata[hash].identifier, dma_guard_metadata[hash].lower_bound, dma_guard_metadata[hash].upper_bound, dma_handle, _mapping_count);
			// 	pr_info("Not written correctly, in metadata: %08x %08x %08x %08x, pointer: %016llx, total mapping: %d\n",  metadata.attr, metadata.identifier, metadata.lower_bound, metadata.upper_bound, dma_handle, _mapping_count);
			// }
			break;
		} else {
			// pr_info("Slot taken, metadata: %08x %08x %08x %08x, pointer: %016llx, total mapping: %d\n",  dma_guard_metadata[hash].attr, dma_guard_metadata[hash].identifier, dma_guard_metadata[hash].lower_bound, dma_guard_metadata[hash].upper_bound, dma_handle, _mapping_count);
		}
	}

	ret = dma_handle | ((uint64_t)hash << 48);
	// pr_info("ret: %016llx\n", ret);

	// dma_guard_dump();
	// pr_info("*** DMAGuard *** device: %s, %016llx *** Mapping ***, count: %x\n", dev_name(dev), ret, ++_mapping_count);
	return ret;
}

// Take the DMAGuard pointer as input, remove the mapping, return the bare DMA pointer
__attribute__((optimize("O0"))) dma_addr_t dma_guard_unmap(struct device *dev, dma_addr_t dma_handle, size_t size) {
	dma_addr_t ret;
	uint16_t hash = dma_handle >> 48;

	if (dev_name(dev)[4] == '0') {
		return dma_handle;
	} else if (dev_name(dev)[4] != ':') {
		pr_info("unknown device: %s\n", dev_name(dev));
	}

	// if (((dma_handle & 0xffffffffffff) != dma_guard_metadata[hash].lower_bound) || (dma_handle & 0xffffffffffff) + size != dma_guard_metadata[hash].upper_bound + 1) {
	// 	pr_info("Not a legal unmap: %016llx, size: %08lx metadata: %08x %08x %08x %08x, pointer: %016llx\n", dma_handle, size, dma_guard_metadata[hash].attr, dma_guard_metadata[hash].identifier, dma_guard_metadata[hash].lower_bound, dma_guard_metadata[hash].upper_bound, dma_handle);
	// 	dma_guard_dump();
	// 	dump_stack();
	// 	while (1);
	// }
	// // BUG_ON(dma_guard_metadata[hash].attr == 0);
	// if (dma_guard_metadata[hash].attr == 0) {
	// 	pr_info("Buffer already unmapped: %016llx, metadata: %08x %08x %08x %08x, pointer: %016llx\n", dma_handle, dma_guard_metadata[hash].attr, dma_guard_metadata[hash].identifier, dma_guard_metadata[hash].lower_bound, dma_guard_metadata[hash].upper_bound, dma_handle);
	// 	dma_guard_dump();
	// 	dump_stack();
	// 	while (1);
	// }
	dma_guard_metadata[hash].attributes = 0;

	// BUG_ON((dma_handle & DMA_GUARD_PROTOTYPE_MASK) != DMA_GUARD_PROTOTYPE_MASK);
	ret = dma_handle & 0xffffffffffff;
	// dma_guard_dump();
	// pr_info("*** DMAGuard *** device: %s, %016llx ** Unmapping **, count: %x\n", dev_name(dev), dma_handle, --_mapping_count);
	_mapping_count--;
	return ret;
}
