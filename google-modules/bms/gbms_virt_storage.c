/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/fs.h> /* register_chrdev, unregister_chrdev */
#include <linux/genalloc.h>
#include <linux/hashtable.h>
#include <linux/log2.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/seq_file.h> /* seq_read, seq_lseek, single_release */
#include <linux/slab.h>

#include <linux/hash.h>
#include <linux/hashtable.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysfs.h>

#include "google_bms.h"

#define ERASE_VAL 0xFF
#define DATA_NBYTES_LIMIT (PAGE_SIZE * 4096)

#define BATT_EEPROM_TAG_MINF_LEN GBMS_MINF_LEN
#define BATT_EEPROM_TAG_BGPN_LEN GBMS_BGPN_LEN
#define BATT_EEPROM_TAG_BRID_LEN 1
#define BATT_EEPROM_TAG_MYMD_LEN BATT_EEPROM_TAG_XYMD_LEN
#define BATT_EEPROM_TAG_STRD_LEN 12
#define BATT_EEPROM_TAG_RSOC_LEN 2
#define BATT_EEPROM_TAG_ACIM_LEN 2
#define BATT_EEPROM_TAG_BCNT_LEN (GBMS_CCBIN_BUCKET_COUNT * 2)
#define BATT_EEPROM_TAG_GMSR_LEN GBMS_GMSR_LEN
#define BATT_EEPROM_TAG_LOTR_LEN 1
#define BATT_EEPROM_TAG_CNHS_LEN 2
#define BATT_EEPROM_TAG_SELC_LEN 1
#define BATT_EEPROM_TAG_CELC_LEN 1

#define BATT_EEPROM_TAG_HIST_LEN BATT_ONE_HIST_LEN
#define BATT_TOTAL_HIST_LEN (BATT_ONE_HIST_LEN * BATT_MAX_HIST_CNT)

/* 0x3E2 is the first free with 75 history entries */
#define BATT_EEPROM_TAG_AYMD_LEN BATT_EEPROM_TAG_XYMD_LEN
#define BATT_EEPROM_TAG_GCFE_LEN 2
#define BATT_EEPROM_TAG_RAVG_LEN 2
#define BATT_EEPROM_TAG_RFCN_LEN 2
#define BATT_EEPROM_TAG_DINF_LEN GBMS_DINF_LEN
#define BATT_EEPROM_TAG_THAS_LEN 2

#define HASH_BIT_SIZE 8

struct virt_storage {
	struct mutex lock;
	struct kobject kobj;
	bool is_initialized;
	bool needs_flush;
	ktime_t last_update_time;
	ktime_t updates_since;
	DECLARE_HASHTABLE(hash_table, HASH_BIT_SIZE);
};

struct entry {
	u32 tag;
	size_t nbytes;
	ktime_t updated_on;
	void *data;
	struct hlist_node node;
};

static int virt_storage_iter(int index, gbms_tag_t *tag, void *ptr);
static int virt_storage_info(gbms_tag_t tag, size_t *addr, size_t *size, void *ptr);
static int virt_storage_read(gbms_tag_t tag, void *data, size_t nbytes, void *ptr);
static int virt_storage_write(gbms_tag_t tag, const void *data, size_t nbytes, void *ptr);
static int virt_storage_flush(bool force, void *ptr);
static int virt_storage_read_data(gbms_tag_t tag, void *data, size_t nbytes, int idx, void *ptr);
static int virt_storage_write_data(gbms_tag_t tag, const void *data, size_t nbytes, int idx,
				   void *ptr);

struct gbms_storage_desc gbms_virt_storage = {
	.info = virt_storage_info,
	.iter = virt_storage_iter,
	.read = virt_storage_read,
	.write = virt_storage_write,
	.flush = virt_storage_flush,
	.read_data = virt_storage_read_data,
	.write_data = virt_storage_write_data,
};

static struct entry *find_entry(struct virt_storage *storage, gbms_tag_t tag)
{
	struct entry *entry;
	u32 key = hash_32(tag, HASH_BIT_SIZE);

	hash_for_each_possible (storage->hash_table, entry, node, key) {
		if (entry->tag == tag) {
			pr_debug("Found entry with tag: %08x!\n", tag);
			return entry;
		}
	}

	return NULL;
}

static void add_entry(struct virt_storage *storage, struct entry *entry)
{
	u32 key = hash_32(entry->tag, HASH_BIT_SIZE);

	hash_add(storage->hash_table, &entry->node, key);
}

static int tag_info(gbms_tag_t tag, size_t *len)
{
	int ret = 0;

	switch (tag) {
	case GBMS_TAG_MINF:
		*len = BATT_EEPROM_TAG_MINF_LEN;
		break;
	case GBMS_TAG_DINF:
		*len = BATT_EEPROM_TAG_DINF_LEN;
		break;
	case GBMS_TAG_HIST:
		*len = BATT_EEPROM_TAG_HIST_LEN;
		break;
	case GBMS_TAG_SNUM:
		/* FALLTHROUGH */
	case GBMS_TAG_BGPN:
		*len = BATT_EEPROM_TAG_BGPN_LEN;
		break;
	case GBMS_TAG_GMSR:
		*len = BATT_EEPROM_TAG_GMSR_LEN;
		break;
	case GBMS_TAG_BCNT:
		*len = BATT_EEPROM_TAG_BCNT_LEN;
		break;
	case GBMS_TAG_CNHS:
		*len = BATT_EEPROM_TAG_CNHS_LEN;
		break;
	case GBMS_TAG_LOTR:
		*len = BATT_EEPROM_TAG_LOTR_LEN;
		break;
	case GBMS_TAG_SELC:
		*len = BATT_EEPROM_TAG_SELC_LEN;
		break;
	case GBMS_TAG_CELC:
		*len = BATT_EEPROM_TAG_CELC_LEN;
		break;
	case GBMS_TAG_STRD:
		*len = BATT_EEPROM_TAG_STRD_LEN;
		break;
	case GBMS_TAG_RSOC:
		*len = BATT_EEPROM_TAG_RSOC_LEN;
		break;
	case GBMS_TAG_ACIM:
		*len = BATT_EEPROM_TAG_ACIM_LEN;
		break;
	case GBMS_TAG_GCFE:
		*len = BATT_EEPROM_TAG_GCFE_LEN;
		break;
	case GBMS_TAG_RAVG:
		*len = BATT_EEPROM_TAG_RAVG_LEN;
		break;
	case GBMS_TAG_RFCN:
		*len = BATT_EEPROM_TAG_RFCN_LEN;
		break;
	case GBMS_TAG_THAS:
		*len = BATT_EEPROM_TAG_THAS_LEN;
		break;
	case GBMS_TAG_AYMD:
		*len = BATT_EEPROM_TAG_AYMD_LEN;
		break;
	case GBMS_TAG_MYMD:
		*len = BATT_EEPROM_TAG_MYMD_LEN;
		break;
	default:
		ret = -ENOENT;
		break;
	}

	return ret;
}

static int virt_storage_iter(int index, gbms_tag_t *tag, void *ptr)
{
	static const gbms_tag_t keys[] = { GBMS_TAG_BGPN, GBMS_TAG_MINF, GBMS_TAG_DINF,
					   GBMS_TAG_HIST, GBMS_TAG_BRID, GBMS_TAG_SNUM,
					   GBMS_TAG_GMSR, GBMS_TAG_BCNT, GBMS_TAG_CNHS,
					   GBMS_TAG_SELC, GBMS_TAG_CELC, GBMS_TAG_LOTR,
					   GBMS_TAG_STRD, GBMS_TAG_RSOC, GBMS_TAG_ACIM,
					   GBMS_TAG_GCFE, GBMS_TAG_RAVG, GBMS_TAG_RFCN,
					   GBMS_TAG_THAS, GBMS_TAG_AYMD, GBMS_TAG_MYMD };
	const int nelems = ARRAY_SIZE(keys);
	struct virt_storage *storage = (struct virt_storage *)ptr;
	bool initialized = false;

	if (index < 0 || index >= nelems)
		return -ENOENT;

	pr_debug("iter\n");
	mutex_lock(&storage->lock);
	initialized = storage->is_initialized;
	mutex_unlock(&storage->lock);

	if (!initialized) {
		pr_debug("attempted iter when storage is not initialized\n");
		return -EPROBE_DEFER;
	}
	*tag = keys[index];
	return 0;
}

static int virt_storage_info(gbms_tag_t tag, size_t *addr, size_t *nbytes, void *ptr)
{
	struct virt_storage *storage = (struct virt_storage *)ptr;
	bool initialized = false;

	pr_debug("info\n");
	mutex_lock(&storage->lock);
	initialized = storage->is_initialized;
	mutex_unlock(&storage->lock);

	if (!initialized) {
		pr_debug("attempted info when storage is not initialized\n");
		return -EPROBE_DEFER;
	}

	*addr = 0;
	return tag_info(tag, nbytes);
}

static int virt_storage_read(gbms_tag_t tag, void *data, size_t nbytes, void *ptr)
{
	pr_debug("read tag: %08x\n", tag);
	return virt_storage_read_data(tag, data, nbytes, 0, ptr);
}

static int virt_storage_write(gbms_tag_t tag, const void *data, size_t nbytes, void *ptr)
{
	pr_debug("write tag: %08x\n", tag);
	return virt_storage_write_data(tag, data, nbytes, 0, ptr);
}

static int virt_storage_flush(bool force, void *ptr)
{
	int ret = 0;
	struct virt_storage *storage = (struct virt_storage *)ptr;

	pr_debug("flush\n");
	mutex_lock(&storage->lock);
	if (!storage->is_initialized) {
		pr_debug("attempted flush when storage is not initialized\n");
		ret = -EPROBE_DEFER;
		goto out;
	}
	storage->needs_flush = 1;

out:
	mutex_unlock(&storage->lock);
	return ret;
}

static int virt_storage_read_data(gbms_tag_t tag, void *data, size_t data_nbytes, int idx,
				  void *ptr)
{
	int ret;
	struct virt_storage *storage = (struct virt_storage *)ptr;
	struct entry *entry;
	size_t offset = 0;
	size_t tag_nbytes = 0;

	pr_debug("read_data tag: %08x data_nbytes: %zd idx: %d\n", tag, data_nbytes, idx);

	if (data == NULL || data_nbytes == 0) {
		/*
		 * TODO(b/324440554) in google_eeprom proper this is a special case that
		 * needs to return the maximum number of history elements stored in the
		 * battery history (75) when the provided idx is not invalid.
		 */
		return 0;
	}

	if (idx < 0)
		return -EINVAL;

	ret = tag_info(tag, &tag_nbytes);
	if (ret != 0)
		return ret;

	if (tag_nbytes > data_nbytes) {
		pr_err("Data buffer sized: %zd not large enough for tag's data: %zd\n", data_nbytes,
		       tag_nbytes);
		return -EINVAL;
	}
	if ((data_nbytes % tag_nbytes) != 0) {
		pr_err("Data buffer sized: %zd is not multiple of tag nbytes: %zd", data_nbytes,
		       tag_nbytes);
		return -EINVAL;
	}

	mutex_lock(&storage->lock);

	if (!storage->is_initialized) {
		pr_debug("attempted read_data when storage is not initialized\n");
		ret = -EPROBE_DEFER;
		goto out;
	}

	entry = find_entry(storage, tag);
	if (entry) {
		pr_debug("Found entry in read_data!\n");

		offset = idx * tag_nbytes;
		if (offset >= entry->nbytes) {
			memset(data, ERASE_VAL, data_nbytes);
			ret = data_nbytes;
			goto out;
		}

		pr_debug(
			"Reading data for tag: %08x data_nbytes: %zd entry->nbytes: %zd offset: %zd tag_nbytes: %zd\n",
			tag, data_nbytes, entry->nbytes, offset, tag_nbytes);
		data_nbytes = min(data_nbytes, entry->nbytes - offset);

		memcpy(data, &entry->data[offset], data_nbytes);
		ret = data_nbytes;
		goto out;
	}

	memset(data, ERASE_VAL, data_nbytes);
	ret = data_nbytes;

out:
	mutex_unlock(&storage->lock);
	return ret;
}

static int virt_storage_write_data(gbms_tag_t tag, const void *data, size_t data_nbytes, int idx,
				   void *ptr)
{
	int ret;
	struct virt_storage *storage = (struct virt_storage *)ptr;
	struct entry *entry = NULL;
	size_t offset = 0;
	size_t tag_nbytes = 0;

	pr_debug("write_data tag: %08x data_nbytes: %zd idx: %d\n", tag, data_nbytes, idx);

	if (idx < 0 || data == NULL || data_nbytes == 0)
		return -EINVAL;

	ret = tag_info(tag, &tag_nbytes);
	if (ret != 0)
		return ret;

	if (data_nbytes > tag_nbytes) {
		pr_err("Data buffer sized: %zd has additional bytes over tag nbytes: %zd\n",
		       data_nbytes, tag_nbytes);
		return -EINVAL;
	}

	mutex_lock(&storage->lock);

	if (!storage->is_initialized) {
		pr_debug("attempted write_data when storage is not initialized\n");
		ret = -EPROBE_DEFER;
		goto out;
	}

	storage->last_update_time = ktime_get_boottime();

	offset = tag_nbytes * idx;

	entry = find_entry(storage, tag);
	if (entry) {
		pr_debug("Found entry for tag: %08x\n", tag);
		entry->updated_on = ktime_get_boottime();

		if (offset + data_nbytes > entry->nbytes) {
			/* Need to extend data for additional information. */
			u8 *new_data = NULL;
			pr_debug("Append tag: %08x offset: %zd ndata: %zd ntag: %zd\n", tag, offset,
				 data_nbytes, tag_nbytes);

			new_data = kzalloc(offset + data_nbytes, GFP_KERNEL);
			if (new_data == NULL) {
				ret = -ENOMEM;
				goto out;
			}
			memcpy(new_data, entry->data, entry->nbytes);
			if (offset > entry->nbytes)
				memset(&new_data[entry->nbytes], ERASE_VAL, offset - entry->nbytes);
			memcpy(&new_data[offset], data, data_nbytes);
			entry->nbytes = offset + data_nbytes;
			kfree(entry->data);
			entry->data = new_data;
			ret = data_nbytes;
		} else {
			/* Replacing bytes in the data. */
			pr_debug("Replace tag: %08x offset: %zd ndata: %zd ntag: %zd\n", tag,
				 offset, data_nbytes, tag_nbytes);
			memcpy(&entry->data[offset], data, data_nbytes);
			ret = data_nbytes;
		}
	} else {
		/* Creating a new entry. */
		entry = kzalloc(sizeof(*entry), GFP_KERNEL);
		entry->data = kzalloc(offset + data_nbytes, GFP_KERNEL);
		if (entry == NULL || entry->data == NULL) {
			if (entry->data == NULL)
				kfree(entry);
			ret = -ENOMEM;
			goto out;
		}
		entry->tag = tag;
		entry->nbytes = offset + data_nbytes;
		entry->updated_on = ktime_get_boottime();
		pr_debug(
			"Creating new entry for tag: %08x offset: %zd data_nbytes: %zd tag_nbytes: %zd\n",
			tag, offset, data_nbytes, tag_nbytes);
		memset(entry->data, ERASE_VAL, entry->nbytes);
		memcpy(&entry->data[offset], data, data_nbytes);
		add_entry(storage, entry);
		ret = data_nbytes;
	}

out:
	mutex_unlock(&storage->lock);
	return ret;
}

/*****************************************************************************
 * SysFS Handles
 */

static ssize_t all_data_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	size_t i;
	struct entry *entry;
	u32 bkt;
	ssize_t n = 0;
	struct virt_storage *storage = container_of(kobj, struct virt_storage, kobj);

	pr_debug("all data show\n");
	mutex_lock(&storage->lock);
	/* List all elements in the table. */
	hash_for_each (storage->hash_table, bkt, entry, node) {
		pr_debug("tag: %08x nbytes: %zd data: %p\n", entry->tag, entry->nbytes,
			 entry->data);
		n += scnprintf(&buf[n], PAGE_SIZE - n, "%08x %zd", entry->tag, entry->nbytes);
		for (i = 0; i < entry->nbytes; ++i) {
			uint8_t *d = (uint8_t *)entry->data;
			n += scnprintf(&buf[n], PAGE_SIZE - n, " %02x", d[i]);
		}
		n += scnprintf(&buf[n], PAGE_SIZE - n, "\n");
	}
	mutex_unlock(&storage->lock);
	return n;
}

static struct kobj_attribute dev_attr_all_data = __ATTR_RO(all_data);

static ssize_t data_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
			  size_t buf_nbytes)
{
	int rc;
	u32 tag;
	size_t i;
	uint8_t *bytes = NULL;
	struct entry *entry;
	size_t actual_data_nbytes = 0;
	size_t data_nbytes = 0;
	char const *pos = NULL;
	struct virt_storage *storage = container_of(kobj, struct virt_storage, kobj);

	pr_debug("data store\n");

	/* Read the tag address and data nbytes. */
	rc = sscanf(buf, "%x %zd", &tag, &data_nbytes);
	if (rc != 2) {
		pr_err("Failure to read tag and data nbytes rc: %d\n", rc);
		return rc;
	}
	if (data_nbytes > DATA_NBYTES_LIMIT) {
		pr_err("Specified data nbytes: %zd is larger than: %lu\n", data_nbytes,
		       DATA_NBYTES_LIMIT);
		return -EINVAL;
	}
	bytes = kzalloc(data_nbytes, GFP_KERNEL);
	if (bytes == NULL)
		return -ENOMEM;

	/* Skip past the spacing between in tag and nbytes to the data. */
	pos = buf;
	for (i = 0; i < 2; ++i) {
		pos = memchr(pos, ' ', buf_nbytes);
		if (pos == NULL) {
			pr_err("Improperly formatted input buffer should be <tag> <nbytes> <data ..>\n");
			return -EINVAL;
		}
		while (*pos == ' ')
			++pos;
	}

	/* Begin reading individual byte from the stream.*/
	for (i = 0; i < data_nbytes; ++i) {
		while (*pos == ' ')
			++pos;
		rc = sscanf(pos, "%2hhx", &bytes[i]);
		if (rc == 1) {
			actual_data_nbytes += 1;
			pos += 2;
		} else {
			pr_err("Failure to parse data byte: %zd rc: %d\n", i, rc);
			break;
		}
	}

	if (data_nbytes != actual_data_nbytes) {
		pr_err("Specified %zd data_nbytes but actual was: %zd\n", data_nbytes,
		       actual_data_nbytes);
	}

	pr_debug("Scanned for tag: %08x in data with data_nbytes: %zd first: %02x last: %02x\n",
		 tag, data_nbytes, bytes[0], bytes[data_nbytes - 1]);

	mutex_lock(&storage->lock);
	entry = find_entry(storage, tag);
	if (!entry) {
		/* Creating a new entry. */
		entry = kzalloc(sizeof(*entry), GFP_KERNEL);
		if (entry == NULL) {
			pr_err("Failure to alloc mem for entry in data store\n");
			kfree(bytes);
			rc = -ENOMEM;
			goto out;
		}
		entry->tag = tag;
		add_entry(storage, entry);
	}
	entry->data = bytes;
	entry->nbytes = data_nbytes;
	entry->updated_on = ktime_get_boottime();
	rc = buf_nbytes;

out:
	mutex_unlock(&storage->lock);
	return rc;
}

static struct kobj_attribute dev_attr_data = __ATTR_WO(data);

static ssize_t updates_since_timestamp_show(struct kobject *kobj, struct kobj_attribute *attr,
					    char *buf)
{
	int n;
	struct virt_storage *storage = container_of(kobj, struct virt_storage, kobj);

	pr_debug("is init show\n");
	mutex_lock(&storage->lock);
	n = scnprintf(buf, PAGE_SIZE, "%lld\n", storage->updates_since);
	mutex_unlock(&storage->lock);
	return n;
}

static ssize_t updates_since_timestamp_store(struct kobject *kobj, struct kobj_attribute *attr,
					     const char *buf, size_t nbytes)
{
	int rc;
	u64 updates_since;
	struct virt_storage *storage = container_of(kobj, struct virt_storage, kobj);

	rc = kstrtou64(buf, 10, &updates_since);
	if (rc != 0) {
		pr_err("failure to convert updates since to value rc: %d\n", rc);
		return rc;
	}

	pr_debug("eeprom stored updates since timestamp: %llu\n", updates_since);
	mutex_lock(&storage->lock);
	storage->updates_since = updates_since;
	mutex_unlock(&storage->lock);
	return (ssize_t)nbytes;
}

static struct kobj_attribute dev_attr_updates_since_timestamp = __ATTR_RW(updates_since_timestamp);

static ssize_t data_updated_since_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct virt_storage *storage = container_of(kobj, struct virt_storage, kobj);
	size_t i;
	ssize_t n = 0;
	struct entry *entry;
	u32 bkt;

	mutex_lock(&storage->lock);
	pr_debug("all keys updated since: %lld\n", storage->updates_since);

	hash_for_each (storage->hash_table, bkt, entry, node) {
		if (entry->updated_on >= storage->updates_since) {
			pr_debug("tag: %08x nbytes: %zd data: %p\n", entry->tag, entry->nbytes,
				 entry->data);
			n += scnprintf(&buf[n], PAGE_SIZE - n, "%08x %zd", entry->tag,
				       entry->nbytes);
			for (i = 0; i < entry->nbytes; ++i) {
				uint8_t *d = (uint8_t *)entry->data;
				n += scnprintf(&buf[n], PAGE_SIZE - n, " %02x", d[i]);
			}
			n += scnprintf(&buf[n], PAGE_SIZE - n, "\n");
		}
	}
	mutex_unlock(&storage->lock);
	return n;
}

static struct kobj_attribute dev_attr_data_updated_since = __ATTR_RO(data_updated_since);

static ssize_t is_initialized_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int n;
	struct virt_storage *storage = container_of(kobj, struct virt_storage, kobj);

	pr_debug("is init show\n");
	mutex_lock(&storage->lock);
	n = scnprintf(buf, PAGE_SIZE, "%d\n", storage->is_initialized);
	mutex_unlock(&storage->lock);
	return n;
}

static ssize_t is_initialized_store(struct kobject *kobj, struct kobj_attribute *attr,
				    const char *buf, size_t nbytes)
{
	int rc;
	bool is_initialized;
	struct virt_storage *storage = container_of(kobj, struct virt_storage, kobj);

	rc = kstrtobool(buf, &is_initialized);
	if (rc != 0) {
		pr_err("Failure to store is_initialized rc: %d\n", rc);
		return rc;
	}

	pr_debug("is initialized store: %d\n", is_initialized);
	mutex_lock(&storage->lock);
	storage->is_initialized = is_initialized;
	mutex_unlock(&storage->lock);
	return (ssize_t)nbytes;
}

static struct kobj_attribute dev_attr_is_init_done = __ATTR_RW(is_initialized);

static ssize_t needs_flush_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int n;
	struct virt_storage *storage = container_of(kobj, struct virt_storage, kobj);

	pr_debug("needs_flush show\n");
	mutex_lock(&storage->lock);
	n = scnprintf(buf, PAGE_SIZE, "%d\n", storage->needs_flush);
	mutex_unlock(&storage->lock);
	return n;
}

static ssize_t needs_flush_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
				 size_t nbytes)
{
	int rc;
	bool needs_flush;
	struct virt_storage *storage = container_of(kobj, struct virt_storage, kobj);

	rc = kstrtobool(buf, &needs_flush);
	if (rc != 0) {
		pr_err("Failure to convert needs store bool rc: %d\n", rc);
		return rc;
	}

	pr_debug("needs flush store: %d\n", needs_flush);
	mutex_lock(&storage->lock);
	storage->needs_flush = needs_flush;
	mutex_unlock(&storage->lock);
	return (ssize_t)nbytes;
}

static struct kobj_attribute dev_attr_needs_flush = __ATTR_RW(needs_flush);

static ssize_t last_update_time_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int n;
	struct virt_storage *storage = container_of(kobj, struct virt_storage, kobj);

	pr_debug("last_update_time show\n");
	mutex_lock(&storage->lock);
	n = scnprintf(buf, PAGE_SIZE, "%lld\n", storage->last_update_time);
	mutex_unlock(&storage->lock);
	return n;
}

static struct kobj_attribute dev_attr_last_update_time = __ATTR_RO(last_update_time);

static struct attribute *default_file_attrs[] = { &dev_attr_is_init_done.attr,
						  &dev_attr_needs_flush.attr,
						  &dev_attr_all_data.attr,
						  &dev_attr_data.attr,
						  &dev_attr_updates_since_timestamp.attr,
						  &dev_attr_data_updated_since.attr,
						  &dev_attr_last_update_time.attr,
						  NULL };

static void virt_storage_release(struct kobject *kobj)
{
	u32 bkt;
	struct entry *entry;
	struct hlist_node *tmp;
	struct virt_storage *storage = container_of(kobj, struct virt_storage, kobj);

	mutex_lock(&storage->lock);
	pr_debug("Cleaning up hash table memory.\n");
	hash_for_each_safe (storage->hash_table, bkt, tmp, entry, node) {
		hash_del(&entry->node);
		kfree(entry->data); /* Free dynamically allocated data */
		kfree(entry);
	}

	mutex_unlock(&storage->lock);
	kfree(storage);
}

static struct kobj_type gbms_virt_storage_ktype = {
	.release = virt_storage_release,
	.default_attrs = default_file_attrs,
	.sysfs_ops = &kobj_sysfs_ops,
};

static int __init gbms_virt_storage_init(void)
{
	int rc = 0;

	struct virt_storage *storage = kzalloc(sizeof(struct virt_storage), GFP_KERNEL);
	if (storage == NULL) {
		rc = -ENOMEM;
		goto out;
	}
	mutex_init(&storage->lock);

	rc = kobject_init_and_add(&storage->kobj, &gbms_virt_storage_ktype, kernel_kobj,
				  "gbms_virt_storage");
	if (0 != rc) {
		pr_err("failed to create gbms storage userland kobject\n");
		return 0;
	}

	gbms_storage_register(&gbms_virt_storage, "virt_storage", storage);

out:
	return rc;
}

static void __exit gbms_virt_storage_exit(void)
{
}

module_init(gbms_virt_storage_init);
module_exit(gbms_virt_storage_exit);
MODULE_AUTHOR("Brandon Edens <brandonedens@google.com>");
MODULE_DESCRIPTION("Google Virtual BMS Storage");
MODULE_LICENSE("GPL");
