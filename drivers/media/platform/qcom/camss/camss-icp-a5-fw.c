// SPDX-License-Identifier: GPL-2.0-only
#include <linux/elf.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/minmax.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "camss-icp-a5.h"

static u32 ph_word(const struct firmware *fw, u32 table, unsigned int index,
		   unsigned int offset)
{
	return get_unaligned_le32(fw->data + table + index * sizeof(Elf32_Phdr) + offset);
}

int camss_a5_fw_inspect(const struct firmware *fw, u32 window,
			struct camss_a5_fw_layout *layout)
{
	struct camss_a5_fw_layout found = {};
	u32 table, start, bytes, offset, file_bytes, flags, align;
	unsigned int count, i;
	bool executable_zero = false;

	if (!fw || !fw->data || !layout || !window)
		return -EINVAL;
	if (fw->size < sizeof(Elf32_Ehdr) ||
	    memcmp(fw->data, ELFMAG, SELFMAG) ||
	    fw->data[EI_CLASS] != ELFCLASS32 || fw->data[EI_DATA] != ELFDATA2LSB ||
	    fw->data[EI_VERSION] != EV_CURRENT || get_unaligned_le16(fw->data + 16) != ET_EXEC ||
	    get_unaligned_le16(fw->data + 18) != EM_ARM ||
	    get_unaligned_le32(fw->data + 20) != EV_CURRENT ||
	    get_unaligned_le32(fw->data + 24) ||
	    get_unaligned_le16(fw->data + 40) != sizeof(Elf32_Ehdr) ||
	    get_unaligned_le16(fw->data + 42) != sizeof(Elf32_Phdr))
		return -ENOEXEC;
	table = get_unaligned_le32(fw->data + 28);
	count = get_unaligned_le16(fw->data + 44);
	if (table > fw->size || !count || count > (fw->size - table) / sizeof(Elf32_Phdr))
		return -ENOEXEC;

	for (i = 0; i < count; i++) {
		if (ph_word(fw, table, i, 0) != PT_LOAD)
			continue;
		offset = ph_word(fw, table, i, 4);
		start = ph_word(fw, table, i, 8);
		file_bytes = ph_word(fw, table, i, 16);
		bytes = ph_word(fw, table, i, 20);
		flags = ph_word(fw, table, i, 24);
		align = ph_word(fw, table, i, 28);
		if (file_bytes > bytes || offset > fw->size || file_bytes > fw->size - offset ||
		    start > window || bytes > window - start ||
		    (align > 1 && ((align & (align - 1)) || (start % align != offset % align))))
			return -ENOEXEC;
		if (!bytes)
			continue;
		if (!start && (flags & PF_X) && file_bytes)
			executable_zero = true;
		found.span = max(found.span, start + bytes);
		found.segments++;
	}
	if (!executable_zero || !found.segments)
		return -ENOEXEC;
	*layout = found;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_fw_inspect);

int camss_a5_fw_load(const struct firmware *fw, void *destination, u32 window,
		    struct camss_a5_fw_layout *layout)
{
	struct camss_a5_fw_layout found;
	u32 table;
	unsigned int i, count;
	int ret;

	if (!destination)
		return -EINVAL;
	ret = camss_a5_fw_inspect(fw, window, &found);
	if (ret)
		return ret;
	table = get_unaligned_le32(fw->data + 28);
	count = get_unaligned_le16(fw->data + 44);
	/* Validate the complete image before modifying owned coherent RAM.
	 * Clearing the span also initializes BSS and holes between segments.
	 */
	memset(destination, 0, found.span);
	for (i = 0; i < count; i++) {
		u32 offset, start, bytes;

		if (ph_word(fw, table, i, 0) != PT_LOAD)
			continue;
		offset = ph_word(fw, table, i, 4);
		start = ph_word(fw, table, i, 8);
		bytes = ph_word(fw, table, i, 16);
		if (bytes)
			memcpy((u8 *)destination + start, fw->data + offset, bytes);
	}
	if (layout)
		*layout = found;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_fw_load);
