// SPDX-License-Identifier: GPL-2.0+
/*
 * storage_common.c -- Common definitions for mass storage functionality
 *
 * Copyright (C) 2003-2008 Alan Stern
 * Copyeight (C) 2009 Samsung Electronics
 * Author: Michal Nazarewicz (mina86@mina86.com)
 */

/*
 * This file requires the following identifiers used in USB strings to
 * be defined (each of type pointer to char):
 *  - fsg_string_interface    -- name of the interface
 */

/*
 * When USB_GADGET_DEBUG_FILES is defined the module param num_buffers
 * sets the number of pipeline buffers (length of the fsg_buffhd array).
 * The valid range of num_buffers is: num >= 2 && num <= 4.
 */

#include <linux/module.h>
#include <linux/blkdev.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kstrtox.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/usb/composite.h>

#include "storage_common.h"

/* There is only one interface. */

struct usb_interface_descriptor fsg_intf_desc = {
	.bLength =		sizeof fsg_intf_desc,
	.bDescriptorType =	USB_DT_INTERFACE,

	.bNumEndpoints =	2,		/* Adjusted during fsg_bind() */
	.bInterfaceClass =	USB_CLASS_MASS_STORAGE,
	.bInterfaceSubClass =	USB_SC_SCSI,	/* Adjusted during fsg_bind() */
	.bInterfaceProtocol =	USB_PR_BULK,	/* Adjusted during fsg_bind() */
	.iInterface =		FSG_STRING_INTERFACE,
};
EXPORT_SYMBOL_GPL(fsg_intf_desc);

/*
 * Three full-speed endpoint descriptors: bulk-in, bulk-out, and
 * interrupt-in.
 */

struct usb_endpoint_descriptor fsg_fs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* wMaxPacketSize set by autoconfiguration */
};
EXPORT_SYMBOL_GPL(fsg_fs_bulk_in_desc);

struct usb_endpoint_descriptor fsg_fs_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* wMaxPacketSize set by autoconfiguration */
};
EXPORT_SYMBOL_GPL(fsg_fs_bulk_out_desc);

struct usb_descriptor_header *fsg_fs_function[] = {
	(struct usb_descriptor_header *) &fsg_intf_desc,
	(struct usb_descriptor_header *) &fsg_fs_bulk_in_desc,
	(struct usb_descriptor_header *) &fsg_fs_bulk_out_desc,
	NULL,
};
EXPORT_SYMBOL_GPL(fsg_fs_function);


/*
 * USB 2.0 devices need to expose both high speed and full speed
 * descriptors, unless they only run at full speed.
 *
 * That means alternate endpoint descriptors (bigger packets).
 */
struct usb_endpoint_descriptor fsg_hs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	/* bEndpointAddress copied from fs_bulk_in_desc during fsg_bind() */
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};
EXPORT_SYMBOL_GPL(fsg_hs_bulk_in_desc);

struct usb_endpoint_descriptor fsg_hs_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	/* bEndpointAddress copied from fs_bulk_out_desc during fsg_bind() */
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
	.bInterval =		1,	/* NAK every 1 uframe */
};
EXPORT_SYMBOL_GPL(fsg_hs_bulk_out_desc);


struct usb_descriptor_header *fsg_hs_function[] = {
	(struct usb_descriptor_header *) &fsg_intf_desc,
	(struct usb_descriptor_header *) &fsg_hs_bulk_in_desc,
	(struct usb_descriptor_header *) &fsg_hs_bulk_out_desc,
	NULL,
};
EXPORT_SYMBOL_GPL(fsg_hs_function);

struct usb_endpoint_descriptor fsg_ss_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	/* bEndpointAddress copied from fs_bulk_in_desc during fsg_bind() */
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};
EXPORT_SYMBOL_GPL(fsg_ss_bulk_in_desc);

struct usb_ss_ep_comp_descriptor fsg_ss_bulk_in_comp_desc = {
	.bLength =		sizeof(fsg_ss_bulk_in_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/*.bMaxBurst =		DYNAMIC, */
};
EXPORT_SYMBOL_GPL(fsg_ss_bulk_in_comp_desc);

struct usb_endpoint_descriptor fsg_ss_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	/* bEndpointAddress copied from fs_bulk_out_desc during fsg_bind() */
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};
EXPORT_SYMBOL_GPL(fsg_ss_bulk_out_desc);

struct usb_ss_ep_comp_descriptor fsg_ss_bulk_out_comp_desc = {
	.bLength =		sizeof(fsg_ss_bulk_in_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/*.bMaxBurst =		DYNAMIC, */
};
EXPORT_SYMBOL_GPL(fsg_ss_bulk_out_comp_desc);

struct usb_descriptor_header *fsg_ss_function[] = {
	(struct usb_descriptor_header *) &fsg_intf_desc,
	(struct usb_descriptor_header *) &fsg_ss_bulk_in_desc,
	(struct usb_descriptor_header *) &fsg_ss_bulk_in_comp_desc,
	(struct usb_descriptor_header *) &fsg_ss_bulk_out_desc,
	(struct usb_descriptor_header *) &fsg_ss_bulk_out_comp_desc,
	NULL,
};
EXPORT_SYMBOL_GPL(fsg_ss_function);


 /*-------------------------------------------------------------------------*/

/*
 * If the next two routines are called while the gadget is registered,
 * the caller must own fsg->filesem for writing.
 */

void fsg_lun_close(struct fsg_lun *curlun)
{
	if (curlun->filp) {
		LDBG(curlun, "close backing file\n");
		fput(curlun->filp);
		curlun->filp = NULL;
	}
	/* The tracks are the medium's */
	kfree(curlun->tracks);
	curlun->tracks = NULL;
	curlun->num_tracks = 0;
}
EXPORT_SYMBOL_GPL(fsg_lun_close);

/* The sectors a CD addresses, up to 99:59:74 after the 2 s before LBA 0 */
#define FSG_CD_SECTORS	(100 * 60 * 75 - 150)

/*
 * Lays the tracks out on the disc from the file's size: each runs to the
 * next one's offset, the last to the end of the file. Gives the disc's
 * sectors.
 */
static int fsg_cd_layout(struct fsg_cd_track *tracks, unsigned int count,
			 loff_t size, u32 *sectors)
{
	u32 lba = 0;
	unsigned int i;

	for (i = 0; i < count; i++) {
		struct fsg_cd_track *t = &tracks[i];
		loff_t end = i + 1 < count ? tracks[i + 1].offset : size;
		u32 rem;
		u64 n;

		if (end > size || end <= t->offset)
			return -EINVAL;
		n = div_u64_rem(end - t->offset, t->size, &rem);
		if ((i + 1 < count && rem) || n <= t->pregap ||
		    n > FSG_CD_SECTORS - lba)
			return -EINVAL;
		t->first = lba;
		t->start = lba + t->pregap;
		lba += n;
		t->end = lba;
	}
	if (lba < 300)		/* Smallest track is 300 frames */
		return -EINVAL;

	*sectors = lba;
	return 0;
}

int fsg_lun_open(struct fsg_lun *curlun, const char *filename)
{
	int				ro;
	struct file			*filp = NULL;
	int				rc = -EINVAL;
	struct inode			*inode = NULL;
	loff_t				size;
	loff_t				num_sectors;
	loff_t				min_sectors;
	unsigned int			blkbits;
	unsigned int			blksize;

	/* R/W if we can, R/O if we must */
	ro = curlun->initially_ro;
	if (!ro) {
		filp = filp_open(filename, O_RDWR | O_LARGEFILE, 0);
		if (PTR_ERR(filp) == -EROFS || PTR_ERR(filp) == -EACCES)
			ro = 1;
	}
	if (ro)
		filp = filp_open(filename, O_RDONLY | O_LARGEFILE, 0);
	if (IS_ERR(filp)) {
		LINFO(curlun, "unable to open backing file: %s\n", filename);
		return PTR_ERR(filp);
	}

	if (!(filp->f_mode & FMODE_WRITE))
		ro = 1;

	inode = filp->f_mapping->host;
	if ((!S_ISREG(inode->i_mode) && !S_ISBLK(inode->i_mode))) {
		LINFO(curlun, "invalid file type: %s\n", filename);
		goto out;
	}

	/*
	 * If we can't read the file, it's no good.
	 * If we can't write the file, use it read-only.
	 */
	if (!(filp->f_mode & FMODE_CAN_READ)) {
		LINFO(curlun, "file not readable: %s\n", filename);
		goto out;
	}
	if (!(filp->f_mode & FMODE_CAN_WRITE))
		ro = 1;

	size = i_size_read(inode);
	if (size < 0) {
		LINFO(curlun, "unable to find file size: %s\n", filename);
		rc = (int) size;
		goto out;
	}

	if (curlun->cdrom) {
		blksize = 2048;
		blkbits = 11;
	} else if (S_ISBLK(inode->i_mode)) {
		blksize = bdev_logical_block_size(I_BDEV(inode));
		blkbits = blksize_bits(blksize);
	} else {
		blksize = 512;
		blkbits = 9;
	}

	num_sectors = size >> blkbits; /* File size in logic-block-size blocks */
	min_sectors = 1;
	if (curlun->cdrom) {
		min_sectors = 300;	/* Smallest track is 300 frames */
		if (curlun->profile == FSG_PROFILE_CD &&
		    num_sectors >= 256*60*75) {
			num_sectors = 256*60*75 - 1;
			LINFO(curlun, "file too big: %s\n", filename);
			LINFO(curlun, "using only first %d blocks\n",
					(int) num_sectors);
		}
	}
	if (num_sectors < min_sectors) {
		LINFO(curlun, "file too small: %s\n", filename);
		rc = -ETOOSMALL;
		goto out;
	}

	/* Tracks given for a new medium: a CD with sectors as the file has them */
	if (curlun->tracks && !fsg_lun_is_open(curlun)) {
		u32 sectors;

		if (!curlun->cdrom || curlun->profile != FSG_PROFILE_CD ||
		    fsg_cd_layout(curlun->tracks, curlun->num_tracks, size,
				  &sectors)) {
			LINFO(curlun, "tracks do not fit: %s\n", filename);
			goto out;
		}
		num_sectors = sectors;
	}

	if (fsg_lun_is_open(curlun))
		fsg_lun_close(curlun);

	curlun->blksize = blksize;
	curlun->blkbits = blkbits;
	curlun->ro = ro;
	curlun->filp = filp;
	curlun->file_length = size;
	curlun->num_sectors = num_sectors;
	LDBG(curlun, "open backing file: %s\n", filename);
	return 0;

out:
	fput(filp);
	return rc;
}
EXPORT_SYMBOL_GPL(fsg_lun_open);


/*-------------------------------------------------------------------------*/

/*
 * Sync the file data, don't bother with the metadata.
 * This code was copied from fs/buffer.c:sys_fdatasync().
 */
int fsg_lun_fsync_sub(struct fsg_lun *curlun)
{
	struct file	*filp = curlun->filp;

	if (curlun->ro || !filp)
		return 0;
	return vfs_fsync(filp, 1);
}
EXPORT_SYMBOL_GPL(fsg_lun_fsync_sub);

void store_cdrom_address(u8 *dest, int msf, u32 addr)
{
	if (msf) {
		/*
		 * Convert to Minutes-Seconds-Frames.
		 * Sector size is already set to 2048 bytes.
		 */
		addr += 2*75;		/* Lead-in occupies 2 seconds */
		/* DVDs and BDs go past 255:59:74, the last address there is */
		addr = min(addr, 256U*60*75 - 1);
		dest[3] = addr % 75;	/* Frames */
		addr /= 75;
		dest[2] = addr % 60;	/* Seconds */
		addr /= 60;
		dest[1] = addr;		/* Minutes */
		dest[0] = 0;		/* Reserved */
	} else {
		/* Absolute sector */
		put_unaligned_be32(addr, dest);
	}
}
EXPORT_SYMBOL_GPL(store_cdrom_address);

/*-------------------------------------------------------------------------*/


ssize_t fsg_show_ro(struct fsg_lun *curlun, char *buf)
{
	return sprintf(buf, "%d\n", fsg_lun_is_open(curlun)
				  ? curlun->ro
				  : curlun->initially_ro);
}
EXPORT_SYMBOL_GPL(fsg_show_ro);

ssize_t fsg_show_nofua(struct fsg_lun *curlun, char *buf)
{
	return sprintf(buf, "%u\n", curlun->nofua);
}
EXPORT_SYMBOL_GPL(fsg_show_nofua);

ssize_t fsg_show_file(struct fsg_lun *curlun, struct rw_semaphore *filesem,
		      char *buf)
{
	char		*p;
	ssize_t		rc;

	down_read(filesem);
	if (fsg_lun_is_open(curlun)) {	/* Get the complete pathname */
		p = file_path(curlun->filp, buf, PAGE_SIZE - 1);
		if (IS_ERR(p))
			rc = PTR_ERR(p);
		else {
			rc = strlen(p);
			memmove(buf, p, rc);
			buf[rc] = '\n';		/* Add a newline */
			buf[++rc] = 0;
		}
	} else {				/* No file, return 0 bytes */
		*buf = 0;
		rc = 0;
	}
	up_read(filesem);
	return rc;
}
EXPORT_SYMBOL_GPL(fsg_show_file);

ssize_t fsg_show_cdrom(struct fsg_lun *curlun, char *buf)
{
	return sprintf(buf, "%u\n", curlun->cdrom);
}
EXPORT_SYMBOL_GPL(fsg_show_cdrom);

static const char * const fsg_profile_names[] = {
	[FSG_PROFILE_CD]	= "cd",
	[FSG_PROFILE_DVD]	= "dvd",
	[FSG_PROFILE_BD]	= "bd",
};

ssize_t fsg_show_profile(struct fsg_lun *curlun, char *buf)
{
	return sprintf(buf, "%s\n", fsg_profile_names[curlun->profile]);
}
EXPORT_SYMBOL_GPL(fsg_show_profile);

ssize_t fsg_show_tracks(struct fsg_lun *curlun, struct rw_semaphore *filesem,
			char *buf)
{
	ssize_t len = 0;
	unsigned int i;

	down_read(filesem);
	for (i = 0; i < curlun->num_tracks; i++) {
		const struct fsg_cd_track *t = &curlun->tracks[i];

		len += sysfs_emit_at(buf, len, "%u %u %u %u %u %llu\n",
				     t->number, t->mode, t->control, t->size,
				     t->pregap, (unsigned long long)t->offset);
	}
	up_read(filesem);
	return len;
}
EXPORT_SYMBOL_GPL(fsg_show_tracks);

ssize_t fsg_show_removable(struct fsg_lun *curlun, char *buf)
{
	return sprintf(buf, "%u\n", curlun->removable);
}
EXPORT_SYMBOL_GPL(fsg_show_removable);

ssize_t fsg_show_inquiry_string(struct fsg_lun *curlun, char *buf)
{
	return sprintf(buf, "%s\n", curlun->inquiry_string);
}
EXPORT_SYMBOL_GPL(fsg_show_inquiry_string);

/*
 * The caller must hold fsg->filesem for reading when calling this function.
 */
static ssize_t _fsg_store_ro(struct fsg_lun *curlun, bool ro)
{
	if (fsg_lun_is_open(curlun)) {
		LDBG(curlun, "read-only status change prevented\n");
		return -EBUSY;
	}

	curlun->ro = ro;
	curlun->initially_ro = ro;
	LDBG(curlun, "read-only status set to %d\n", curlun->ro);

	return 0;
}

ssize_t fsg_store_ro(struct fsg_lun *curlun, struct rw_semaphore *filesem,
		     const char *buf, size_t count)
{
	ssize_t		rc;
	bool		ro;

	rc = kstrtobool(buf, &ro);
	if (rc)
		return rc;

	/*
	 * Allow the write-enable status to change only while the
	 * backing file is closed.
	 */
	down_read(filesem);
	rc = _fsg_store_ro(curlun, ro);
	if (!rc)
		rc = count;
	up_read(filesem);

	return rc;
}
EXPORT_SYMBOL_GPL(fsg_store_ro);

ssize_t fsg_store_nofua(struct fsg_lun *curlun, const char *buf, size_t count)
{
	bool		nofua;
	int		ret;

	ret = kstrtobool(buf, &nofua);
	if (ret)
		return ret;

	/* Sync data when switching from async mode to sync */
	if (!nofua && curlun->nofua)
		fsg_lun_fsync_sub(curlun);

	curlun->nofua = nofua;

	return count;
}
EXPORT_SYMBOL_GPL(fsg_store_nofua);

ssize_t fsg_store_file(struct fsg_lun *curlun, struct rw_semaphore *filesem,
		       const char *buf, size_t count)
{
	int		rc = 0;

	if (curlun->prevent_medium_removal && fsg_lun_is_open(curlun)) {
		LDBG(curlun, "eject attempt prevented\n");
		return -EBUSY;				/* "Door is locked" */
	}

	/* Remove a trailing newline */
	if (count > 0 && buf[count-1] == '\n')
		((char *) buf)[count-1] = 0;		/* Ugh! */

	/* Load new medium */
	down_write(filesem);
	if (count > 0 && buf[0]) {
		/* fsg_lun_open() will close existing file if any. */
		rc = fsg_lun_open(curlun, buf);
		if (rc == 0) {
			curlun->unit_attention_data =
					SS_NOT_READY_TO_READY_TRANSITION;
			curlun->media_event = FSG_MEDIA_NEW;
		}
	} else if (fsg_lun_is_open(curlun)) {
		fsg_lun_close(curlun);
		curlun->unit_attention_data = SS_MEDIUM_NOT_PRESENT;
		curlun->media_event = FSG_MEDIA_REMOVAL;
	}
	up_write(filesem);
	return (rc < 0 ? rc : count);
}
EXPORT_SYMBOL_GPL(fsg_store_file);

ssize_t fsg_store_cdrom(struct fsg_lun *curlun, struct rw_semaphore *filesem,
			const char *buf, size_t count)
{
	bool		cdrom;
	int		ret;

	ret = kstrtobool(buf, &cdrom);
	if (ret)
		return ret;

	down_read(filesem);
	ret = cdrom ? _fsg_store_ro(curlun, true) : 0;

	if (!ret) {
		curlun->cdrom = cdrom;
		ret = count;
	}
	up_read(filesem);

	return ret;
}
EXPORT_SYMBOL_GPL(fsg_store_cdrom);

/* The profile goes with the medium: it is set before a file is opened. */
ssize_t fsg_store_profile(struct fsg_lun *curlun, struct rw_semaphore *filesem,
			  const char *buf, size_t count)
{
	int profile = sysfs_match_string(fsg_profile_names, buf);
	ssize_t ret = count;

	if (profile < 0)
		return profile;

	down_write(filesem);
	if (fsg_lun_is_open(curlun))
		ret = -EBUSY;
	else
		curlun->profile = profile;
	up_write(filesem);

	return ret;
}
EXPORT_SYMBOL_GPL(fsg_store_profile);

/* Audio in 2352-byte sectors; mode 1 in 2048 or 2352, mode 2 in 2336 or 2352 */
static bool fsg_cd_track_valid(unsigned int mode, unsigned int control,
			       unsigned int size)
{
	if (control > 0xf || !(control & 0x4) != !mode)
		return false;

	switch (mode) {
	case 0:
		return size == 2352;
	case 1:
		return size == 2048 || size == 2352;
	case 2:
		return size == 2336 || size == 2352;
	}
	return false;
}

/*
 * The tracks of the next medium, a line each: number, mode (0 for audio, 1
 * or 2), the control bits of its Q subchannel, the bytes of a sector in the
 * file, the sectors of its pregap and the offset in the file of its first
 * one. Nothing for one data track of 2048-byte sectors.
 */
ssize_t fsg_store_tracks(struct fsg_lun *curlun, struct rw_semaphore *filesem,
			 const char *buf, size_t count)
{
	struct fsg_cd_track *tracks;
	unsigned int n = 0;
	char *text, *rest, *line;
	ssize_t ret = count;

	tracks = kcalloc(FSG_CD_MAX_TRACKS, sizeof(*tracks), GFP_KERNEL);
	text = kstrndup(buf, count, GFP_KERNEL);
	if (!tracks || !text) {
		ret = -ENOMEM;
		goto out;
	}

	rest = text;
	while ((line = strsep(&rest, "\n"))) {
		unsigned int number, mode, control, size, pregap;
		unsigned long long offset;
		struct fsg_cd_track *t = &tracks[n];

		line = strim(line);
		if (!*line)
			continue;
		if (n == FSG_CD_MAX_TRACKS ||
		    sscanf(line, "%u %u %u %u %u %llu", &number, &mode,
			   &control, &size, &pregap, &offset) != 6 ||
		    number < 1 || number > FSG_CD_MAX_TRACKS ||
		    !fsg_cd_track_valid(mode, control, size) ||
		    offset > LLONG_MAX ||
		    (n && (number != t[-1].number + 1 ||
			   offset <= t[-1].offset))) {
			ret = -EINVAL;
			goto out;
		}
		t->number = number;
		t->mode = mode;
		t->control = control;
		t->size = size;
		t->pregap = pregap;
		t->offset = offset;
		n++;
	}

	down_write(filesem);
	if (fsg_lun_is_open(curlun)) {
		ret = -EBUSY;
	} else {
		kfree(curlun->tracks);
		curlun->tracks = n ? tracks : NULL;
		curlun->num_tracks = n;
		if (n)
			tracks = NULL;
	}
	up_write(filesem);

out:
	kfree(tracks);
	kfree(text);
	return ret;
}
EXPORT_SYMBOL_GPL(fsg_store_tracks);

ssize_t fsg_store_removable(struct fsg_lun *curlun, const char *buf,
			    size_t count)
{
	bool		removable;
	int		ret;

	ret = kstrtobool(buf, &removable);
	if (ret)
		return ret;

	curlun->removable = removable;

	return count;
}
EXPORT_SYMBOL_GPL(fsg_store_removable);

ssize_t fsg_store_inquiry_string(struct fsg_lun *curlun, const char *buf,
				 size_t count)
{
	const size_t len = min(count, sizeof(curlun->inquiry_string));

	if (len == 0 || buf[0] == '\n') {
		curlun->inquiry_string[0] = 0;
	} else {
		snprintf(curlun->inquiry_string,
			 sizeof(curlun->inquiry_string), "%-28s", buf);
		if (curlun->inquiry_string[len-1] == '\n')
			curlun->inquiry_string[len-1] = ' ';
	}

	return count;
}
EXPORT_SYMBOL_GPL(fsg_store_inquiry_string);

ssize_t fsg_store_forced_eject(struct fsg_lun *curlun, struct rw_semaphore *filesem,
			       const char *buf, size_t count)
{
	int ret;

	/*
	 * Forcibly detach the backing file from the LUN
	 * regardless of whether the host has allowed it.
	 */
	curlun->prevent_medium_removal = 0;
	ret = fsg_store_file(curlun, filesem, "", 0);
	return ret < 0 ? ret : count;
}
EXPORT_SYMBOL_GPL(fsg_store_forced_eject);

/*
 * As the eject button of an optical drive: the host is told, by GET EVENT
 * STATUS NOTIFICATION, that the medium is to come out, and ejects it once
 * it is done with it.
 */
ssize_t fsg_store_eject_request(struct fsg_lun *curlun,
				struct rw_semaphore *filesem,
				const char *buf, size_t count)
{
	bool		request;
	ssize_t		ret;

	ret = kstrtobool(buf, &request);
	if (ret)
		return ret;

	down_write(filesem);
	if (!curlun->cdrom)
		ret = -EINVAL;
	else if (!fsg_lun_is_open(curlun))
		ret = -ENOMEDIUM;
	else if (request)
		curlun->media_event = FSG_MEDIA_EJECT_REQUEST;
	up_write(filesem);

	return ret < 0 ? ret : count;
}
EXPORT_SYMBOL_GPL(fsg_store_eject_request);

MODULE_DESCRIPTION("Common definitions for mass storage functionality");
MODULE_LICENSE("GPL");
