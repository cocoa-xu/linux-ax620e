// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * f_mass_storage.c -- Mass Storage USB Composite Function
 *
 * Copyright (C) 2003-2008 Alan Stern
 * Copyright (C) 2009 Samsung Electronics
 *                    Author: Michal Nazarewicz <mina86@mina86.com>
 * All rights reserved.
 */

/*
 * The Mass Storage Function acts as a USB Mass Storage device,
 * appearing to the host as a disk drive or as a CD-ROM drive.  In
 * addition to providing an example of a genuinely useful composite
 * function for a USB device, it also illustrates a technique of
 * double-buffering for increased throughput.
 *
 * For more information about MSF and in particular its module
 * parameters and sysfs interface read the
 * <Documentation/usb/mass-storage.rst> file.
 */

/*
 * MSF is configured by specifying a fsg_config structure.  It has the
 * following fields:
 *
 *	nluns		Number of LUNs function have (anywhere from 1
 *				to FSG_MAX_LUNS).
 *	luns		An array of LUN configuration values.  This
 *				should be filled for each LUN that
 *				function will include (ie. for "nluns"
 *				LUNs).  Each element of the array has
 *				the following fields:
 *	->filename	The path to the backing file for the LUN.
 *				Required if LUN is not marked as
 *				removable.
 *	->ro		Flag specifying access to the LUN shall be
 *				read-only.  This is implied if CD-ROM
 *				emulation is enabled as well as when
 *				it was impossible to open "filename"
 *				in R/W mode.
 *	->removable	Flag specifying that LUN shall be indicated as
 *				being removable.
 *	->cdrom		Flag specifying that LUN shall be reported as
 *				being a CD-ROM.
 *	->profile	For a CD-ROM LUN, the medium it reports: "cd"
 *				(the default), "dvd" or "bd".  Set before
 *				the file; a DVD or BD is not limited to the
 *				CD's 255 minutes.
 *	->tracks	For a CD whose file has its sectors as the disc
 *				has them, such as a CUE sheet's BIN: a line
 *				for each track, "number mode control size
 *				pregap offset" (mode 0 for audio, 1 or 2 for
 *				data; the Q subchannel's control bits; 2048,
 *				2336 or 2352 bytes a sector; the sectors before
 *				index 1; where its first sector is in the
 *				file).  Set without a medium, before the file;
 *				ejecting the medium clears it.
 *	->eject_request	For a CD-ROM LUN, as its eject button: the host
 *				hears of it through GET EVENT STATUS
 *				NOTIFICATION and ejects the medium.
 *	->nofua		Flag specifying that FUA flag in SCSI WRITE(10,12)
 *				commands for this LUN shall be ignored.
 *
 *	vendor_name
 *	product_name
 *	release		Information used as a reply to INQUIRY
 *				request.  To use default set to NULL,
 *				NULL, 0xffff respectively.  The first
 *				field should be 8 and the second 16
 *				characters or less.
 *
 *	can_stall	Set to permit function to halt bulk endpoints.
 *				Disabled on some USB devices known not
 *				to work correctly.  You should set it
 *				to true.
 *
 * If "removable" is not set for a LUN then a backing file must be
 * specified.  If it is set, then NULL filename means the LUN's medium
 * is not loaded (an empty string as "filename" in the fsg_config
 * structure causes error).  The CD-ROM emulation includes a single
 * data track and no audio tracks; hence there need be only one
 * backing file per LUN.
 *
 * This function is heavily based on "File-backed Storage Gadget" by
 * Alan Stern which in turn is heavily based on "Gadget Zero" by David
 * Brownell.  The driver's SCSI command interface was based on the
 * "Information technology - Small Computer System Interface - 2"
 * document from X3T9.2 Project 375D, Revision 10L, 7-SEP-93,
 * available at <http://www.t10.org/ftp/t10/drafts/s2/s2-r10l.pdf>.
 * The single exception is opcode 0x23 (READ FORMAT CAPACITIES), which
 * was based on the "Universal Serial Bus Mass Storage Class UFI
 * Command Specification" document, Revision 1.0, December 14, 1998,
 * available at
 * <http://www.usb.org/developers/devclass_docs/usbmass-ufi10.pdf>.
 */

/*
 *				Driver Design
 *
 * The MSF is fairly straightforward.  There is a main kernel
 * thread that handles most of the work.  Interrupt routines field
 * callbacks from the controller driver: bulk- and interrupt-request
 * completion notifications, endpoint-0 events, and disconnect events.
 * Completion events are passed to the main thread by wakeup calls.  Many
 * ep0 requests are handled at interrupt time, but SetInterface,
 * SetConfiguration, and device reset requests are forwarded to the
 * thread in the form of "exceptions" using SIGUSR1 signals (since they
 * should interrupt any ongoing file I/O operations).
 *
 * The thread's main routine implements the standard command/data/status
 * parts of a SCSI interaction.  It and its subroutines are full of tests
 * for pending signals/exceptions -- all this polling is necessary since
 * the kernel has no setjmp/longjmp equivalents.  (Maybe this is an
 * indication that the driver really wants to be running in userspace.)
 * An important point is that so long as the thread is alive it keeps an
 * open reference to the backing file.  This will prevent unmounting
 * the backing file's underlying filesystem and could cause problems
 * during system shutdown, for example.  To prevent such problems, the
 * thread catches INT, TERM, and KILL signals and converts them into
 * an EXIT exception.
 *
 * In normal operation the main thread is started during the gadget's
 * fsg_bind() callback and stopped during fsg_unbind().  But it can
 * also exit when it receives a signal, and there's no point leaving
 * the gadget running when the thread is dead.  As of this moment, MSF
 * provides no way to deregister the gadget when thread dies -- maybe
 * a callback functions is needed.
 *
 * To provide maximum throughput, the driver uses a circular pipeline of
 * buffer heads (struct fsg_buffhd).  In principle the pipeline can be
 * arbitrarily long; in practice the benefits don't justify having more
 * than 2 stages (i.e., double buffering).  But it helps to think of the
 * pipeline as being a long one.  Each buffer head contains a bulk-in and
 * a bulk-out request pointer (since the buffer can be used for both
 * output and input -- directions always are given from the host's
 * point of view) as well as a pointer to the buffer and various state
 * variables.
 *
 * Use of the pipeline follows a simple protocol.  There is a variable
 * (fsg->next_buffhd_to_fill) that points to the next buffer head to use.
 * At any time that buffer head may still be in use from an earlier
 * request, so each buffer head has a state variable indicating whether
 * it is EMPTY, FULL, or BUSY.  Typical use involves waiting for the
 * buffer head to be EMPTY, filling the buffer either by file I/O or by
 * USB I/O (during which the buffer head is BUSY), and marking the buffer
 * head FULL when the I/O is complete.  Then the buffer will be emptied
 * (again possibly by USB I/O, during which it is marked BUSY) and
 * finally marked EMPTY again (possibly by a completion routine).
 *
 * A module parameter tells the driver to avoid stalling the bulk
 * endpoints wherever the transport specification allows.  This is
 * necessary for some UDCs like the SuperH, which cannot reliably clear a
 * halt on a bulk endpoint.  However, under certain circumstances the
 * Bulk-only specification requires a stall.  In such cases the driver
 * will halt the endpoint and set a flag indicating that it should clear
 * the halt in software during the next device reset.  Hopefully this
 * will permit everything to work correctly.  Furthermore, although the
 * specification allows the bulk-out endpoint to halt when the host sends
 * too much data, implementing this would cause an unavoidable race.
 * The driver will always use the "no-stall" approach for OUT transfers.
 *
 * One subtle point concerns sending status-stage responses for ep0
 * requests.  Some of these requests, such as device reset, can involve
 * interrupting an ongoing file I/O operation, which might take an
 * arbitrarily long time.  During that delay the host might give up on
 * the original ep0 request and issue a new one.  When that happens the
 * driver should not notify the host about completion of the original
 * request, as the host will no longer be waiting for it.  So the driver
 * assigns to each ep0 request a unique tag, and it keeps track of the
 * tag value of the request associated with a long-running exception
 * (device-reset, interface-change, or configuration-change).  When the
 * exception handler is finished, the status-stage response is submitted
 * only if the current ep0 request tag is equal to the exception request
 * tag.  Thus only the most recently received ep0 request will get a
 * status-stage response.
 *
 * Warning: This driver source file is too long.  It ought to be split up
 * into a header file plus about 3 separate .c files, to handle the details
 * of the Gadget, USB Mass Storage, and SCSI protocols.
 */


/* #define VERBOSE_DEBUG */
/* #define DUMP_MSGS */

#include <linux/bcd.h>
#include <linux/blkdev.h>
#include <linux/cdrom.h>
#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kstrtox.h>
#include <linux/kthread.h>
#include <linux/sched/signal.h>
#include <linux/limits.h>
#include <linux/overflow.h>
#include <linux/pagemap.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/freezer.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>

#include <linux/nospec.h>
#include <linux/once.h>

#include "configfs.h"


/*------------------------------------------------------------------------*/

#define FSG_DRIVER_DESC		"Mass Storage Function"

static const char fsg_string_interface[] = "Mass Storage";

#include "storage_common.h"
#include "f_mass_storage.h"

/* Static strings, in UTF-8 (for simplicity we use only ASCII characters) */
static struct usb_string		fsg_strings[] = {
	{FSG_STRING_INTERFACE,		fsg_string_interface},
	{}
};

static struct usb_gadget_strings	fsg_stringtab = {
	.language	= 0x0409,		/* en-us */
	.strings	= fsg_strings,
};

static struct usb_gadget_strings *fsg_strings_array[] = {
	&fsg_stringtab,
	NULL,
};

/*-------------------------------------------------------------------------*/

struct fsg_dev;
struct fsg_common;

/* Data shared by all the FSG instances. */
struct fsg_common {
	struct usb_gadget	*gadget;
	struct usb_composite_dev *cdev;
	struct fsg_dev		*fsg;
	wait_queue_head_t	io_wait;
	wait_queue_head_t	fsg_wait;

	/* filesem protects: backing files in use */
	struct rw_semaphore	filesem;

	/* lock protects: state and thread_task */
	spinlock_t		lock;

	struct usb_ep		*ep0;		/* Copy of gadget->ep0 */
	struct usb_request	*ep0req;	/* Copy of cdev->req */
	unsigned int		ep0_req_tag;

	struct fsg_buffhd	*next_buffhd_to_fill;
	struct fsg_buffhd	*next_buffhd_to_drain;
	struct fsg_buffhd	*buffhds;
	unsigned int		fsg_num_buffers;

	int			cmnd_size;
	u8			cmnd[MAX_COMMAND_SIZE];

	unsigned int		lun;
	struct fsg_lun		*luns[FSG_MAX_LUNS];
	struct fsg_lun		*curlun;

	unsigned int		bulk_out_maxpacket;
	enum fsg_state		state;		/* For exception handling */
	unsigned int		exception_req_tag;
	void			*exception_arg;

	enum data_direction	data_dir;
	u32			data_size;
	u32			data_size_from_cmnd;
	u32			tag;
	u32			residue;
	u32			usb_amount_left;

	unsigned int		can_stall:1;
	unsigned int		free_storage_on_release:1;
	unsigned int		phase_error:1;
	unsigned int		short_packet_received:1;
	unsigned int		bad_lun_okay:1;
	unsigned int		running:1;
	unsigned int		sysfs:1;

	struct completion	thread_notifier;
	struct task_struct	*thread_task;

	/* Gadget's private data. */
	void			*private_data;

	char inquiry_string[INQUIRY_STRING_LEN];
};

struct fsg_dev {
	struct usb_function	function;
	struct usb_gadget	*gadget;	/* Copy of cdev->gadget */
	struct fsg_common	*common;

	u16			interface_number;

	unsigned int		bulk_in_enabled:1;
	unsigned int		bulk_out_enabled:1;

	unsigned long		atomic_bitflags;
#define IGNORE_BULK_OUT		0

	struct usb_ep		*bulk_in;
	struct usb_ep		*bulk_out;
};

static inline int __fsg_is_set(struct fsg_common *common,
			       const char *func, unsigned line)
{
	if (common->fsg)
		return 1;
	ERROR(common, "common->fsg is NULL in %s at %u\n", func, line);
	WARN_ON(1);
	return 0;
}

#define fsg_is_set(common) likely(__fsg_is_set(common, __func__, __LINE__))

static inline struct fsg_dev *fsg_from_func(struct usb_function *f)
{
	return container_of(f, struct fsg_dev, function);
}

static int exception_in_progress(struct fsg_common *common)
{
	return common->state > FSG_STATE_NORMAL;
}

/* Make bulk-out requests be divisible by the maxpacket size */
static void set_bulk_out_req_length(struct fsg_common *common,
				    struct fsg_buffhd *bh, unsigned int length)
{
	unsigned int	rem;

	bh->bulk_out_intended_length = length;
	rem = length % common->bulk_out_maxpacket;
	if (rem > 0)
		length += common->bulk_out_maxpacket - rem;
	bh->outreq->length = length;
}


/*-------------------------------------------------------------------------*/

static int fsg_set_halt(struct fsg_dev *fsg, struct usb_ep *ep)
{
	const char	*name;

	if (ep == fsg->bulk_in)
		name = "bulk-in";
	else if (ep == fsg->bulk_out)
		name = "bulk-out";
	else
		name = ep->name;
	DBG(fsg, "%s set halt\n", name);
	return usb_ep_set_halt(ep);
}


/*-------------------------------------------------------------------------*/

/* These routines may be called in process context or in_irq */

static void __raise_exception(struct fsg_common *common, enum fsg_state new_state,
			      void *arg)
{
	unsigned long		flags;

	/*
	 * Do nothing if a higher-priority exception is already in progress.
	 * If a lower-or-equal priority exception is in progress, preempt it
	 * and notify the main thread by sending it a signal.
	 */
	spin_lock_irqsave(&common->lock, flags);
	if (common->state <= new_state) {
		common->exception_req_tag = common->ep0_req_tag;
		common->state = new_state;
		common->exception_arg = arg;
		if (common->thread_task)
			send_sig_info(SIGUSR1, SEND_SIG_PRIV,
				      common->thread_task);
	}
	spin_unlock_irqrestore(&common->lock, flags);
}

static void raise_exception(struct fsg_common *common, enum fsg_state new_state)
{
	__raise_exception(common, new_state, NULL);
}

/*-------------------------------------------------------------------------*/

static int ep0_queue(struct fsg_common *common)
{
	int	rc;

	rc = usb_ep_queue(common->ep0, common->ep0req, GFP_ATOMIC);
	common->ep0->driver_data = common;
	if (rc != 0 && rc != -ESHUTDOWN) {
		/* We can't do much more than wait for a reset */
		WARNING(common, "error in submission: %s --> %d\n",
			common->ep0->name, rc);
	}
	return rc;
}


/*-------------------------------------------------------------------------*/

/* Completion handlers. These always run in_irq. */

static void bulk_in_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct fsg_common	*common = ep->driver_data;
	struct fsg_buffhd	*bh = req->context;

	if (req->status || req->actual != req->length)
		DBG(common, "%s --> %d, %u/%u\n", __func__,
		    req->status, req->actual, req->length);
	if (req->status == -ECONNRESET)		/* Request was cancelled */
		usb_ep_fifo_flush(ep);

	/* Synchronize with the smp_load_acquire() in sleep_thread() */
	smp_store_release(&bh->state, BUF_STATE_EMPTY);
	wake_up(&common->io_wait);
}

static void bulk_out_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct fsg_common	*common = ep->driver_data;
	struct fsg_buffhd	*bh = req->context;

	dump_msg(common, "bulk-out", req->buf, req->actual);
	if (req->status || req->actual != bh->bulk_out_intended_length)
		DBG(common, "%s --> %d, %u/%u\n", __func__,
		    req->status, req->actual, bh->bulk_out_intended_length);
	if (req->status == -ECONNRESET)		/* Request was cancelled */
		usb_ep_fifo_flush(ep);

	/* Synchronize with the smp_load_acquire() in sleep_thread() */
	smp_store_release(&bh->state, BUF_STATE_FULL);
	wake_up(&common->io_wait);
}

static int _fsg_common_get_max_lun(struct fsg_common *common)
{
	int i = ARRAY_SIZE(common->luns) - 1;

	while (i >= 0 && !common->luns[i])
		--i;

	return i;
}

static int fsg_setup(struct usb_function *f,
		     const struct usb_ctrlrequest *ctrl)
{
	struct fsg_dev		*fsg = fsg_from_func(f);
	struct usb_request	*req = fsg->common->ep0req;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	if (!fsg_is_set(fsg->common))
		return -EOPNOTSUPP;

	++fsg->common->ep0_req_tag;	/* Record arrival of a new request */
	req->context = NULL;
	req->length = 0;
	dump_msg(fsg, "ep0-setup", (u8 *) ctrl, sizeof(*ctrl));

	switch (ctrl->bRequest) {

	case US_BULK_RESET_REQUEST:
		if (ctrl->bRequestType !=
		    (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE))
			break;
		if (w_index != fsg->interface_number || w_value != 0 ||
				w_length != 0)
			return -EDOM;

		/*
		 * Raise an exception to stop the current operation
		 * and reinitialize our state.
		 */
		DBG(fsg, "bulk reset request\n");
		raise_exception(fsg->common, FSG_STATE_PROTOCOL_RESET);
		return USB_GADGET_DELAYED_STATUS;

	case US_BULK_GET_MAX_LUN:
		if (ctrl->bRequestType !=
		    (USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE))
			break;
		if (w_index != fsg->interface_number || w_value != 0 ||
				w_length != 1)
			return -EDOM;
		VDBG(fsg, "get max LUN\n");
		*(u8 *)req->buf = _fsg_common_get_max_lun(fsg->common);

		/* Respond with data/status */
		req->length = min_t(u16, 1, w_length);
		return ep0_queue(fsg->common);
	}

	VDBG(fsg,
	     "unknown class-specific control req %02x.%02x v%04x i%04x l%u\n",
	     ctrl->bRequestType, ctrl->bRequest,
	     le16_to_cpu(ctrl->wValue), w_index, w_length);
	return -EOPNOTSUPP;
}


/*-------------------------------------------------------------------------*/

/* All the following routines run in process context */

/* Use this for bulk or interrupt transfers, not ep0 */
static int start_transfer(struct fsg_dev *fsg, struct usb_ep *ep,
			   struct usb_request *req)
{
	int	rc;

	if (ep == fsg->bulk_in)
		dump_msg(fsg, "bulk-in", req->buf, req->length);

	rc = usb_ep_queue(ep, req, GFP_KERNEL);
	if (rc) {

		/* We can't do much more than wait for a reset */
		req->status = rc;

		/*
		 * Note: currently the net2280 driver fails zero-length
		 * submissions if DMA is enabled.
		 */
		if (rc != -ESHUTDOWN &&
				!(rc == -EOPNOTSUPP && req->length == 0))
			WARNING(fsg, "error in submission: %s --> %d\n",
					ep->name, rc);
	}
	return rc;
}

static bool start_in_transfer(struct fsg_common *common, struct fsg_buffhd *bh)
{
	int rc;

	if (!fsg_is_set(common))
		return false;
	bh->state = BUF_STATE_SENDING;
	rc = start_transfer(common->fsg, common->fsg->bulk_in, bh->inreq);
	if (rc) {
		bh->state = BUF_STATE_EMPTY;
		if (rc == -ESHUTDOWN) {
			common->running = 0;
			return false;
		}
	}
	return true;
}

static bool start_out_transfer(struct fsg_common *common, struct fsg_buffhd *bh)
{
	int rc;

	if (!fsg_is_set(common))
		return false;
	bh->state = BUF_STATE_RECEIVING;
	rc = start_transfer(common->fsg, common->fsg->bulk_out, bh->outreq);
	if (rc) {
		bh->state = BUF_STATE_FULL;
		if (rc == -ESHUTDOWN) {
			common->running = 0;
			return false;
		}
	}
	return true;
}

static int sleep_thread(struct fsg_common *common, bool can_freeze,
		struct fsg_buffhd *bh)
{
	int	rc;

	/* Wait until a signal arrives or bh is no longer busy */
	if (can_freeze)
		/*
		 * synchronize with the smp_store_release(&bh->state) in
		 * bulk_in_complete() or bulk_out_complete()
		 */
		rc = wait_event_freezable(common->io_wait,
				bh && smp_load_acquire(&bh->state) >=
					BUF_STATE_EMPTY);
	else
		rc = wait_event_interruptible(common->io_wait,
				bh && smp_load_acquire(&bh->state) >=
					BUF_STATE_EMPTY);
	return rc ? -EINTR : 0;
}


/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/

/*
 * CDs whose image keeps the sectors as the disc has them, as told by the
 * LUN's tracks (struct fsg_cd_track): audio, and data of mode 1 or mode 2 in
 * sectors of 2352 bytes, or of what follows the header.
 */

/* What READ CD selects of a sector's main channel (its byte 9) */
#define FSG_CD_SYNC		0x80
#define FSG_CD_SUBHEADER	0x40
#define FSG_CD_HEADER		0x20
#define FSG_CD_USER		0x10
#define FSG_CD_EDC_ECC		0x08

/* What a sector is read as */
enum fsg_cd_kind {
	FSG_CD_AUDIO,
	FSG_CD_MODE1,
	FSG_CD_MODE2,		/* Formless: 2336 bytes of user data */
	FSG_CD_FORM1,
	FSG_CD_FORM2,
};

/* READ CD's expected sector type of each kind; 0 takes any */
static const u8 fsg_cd_types[] = {
	[FSG_CD_AUDIO] = 1, [FSG_CD_MODE1] = 2, [FSG_CD_MODE2] = 3,
	[FSG_CD_FORM1] = 4, [FSG_CD_FORM2] = 5,
};

/* Where the parts of a sector are in its 2352 bytes, in the order sent */
static const u8 fsg_cd_part_bits[] = {
	FSG_CD_SYNC, FSG_CD_HEADER, FSG_CD_SUBHEADER, FSG_CD_USER,
	FSG_CD_EDC_ECC,
};

static const struct {
	u16 at, len;
} fsg_cd_parts[][ARRAY_SIZE(fsg_cd_part_bits)] = {
	[FSG_CD_AUDIO] = { {0, 0}, {0, 0}, {0, 0}, {0, 2352}, {0, 0} },
	[FSG_CD_MODE1] = { {0, 12}, {12, 4}, {0, 0}, {16, 2048}, {2064, 288} },
	[FSG_CD_MODE2] = { {0, 12}, {12, 4}, {0, 0}, {16, 2336}, {0, 0} },
	[FSG_CD_FORM1] = { {0, 12}, {12, 4}, {16, 8}, {24, 2048}, {2072, 280} },
	[FSG_CD_FORM2] = { {0, 12}, {12, 4}, {16, 8}, {24, 2324}, {2348, 4} },
};

static const u8 fsg_cd_sync[12] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00
};

/* GF(2^8) for the ECC and the EDC's CRC, as in ECMA-130 annexes A and B */
static u8 fsg_ecc_f[256], fsg_ecc_b[256];
static u32 fsg_edc_table[256];

static void fsg_cd_init_tables(void)
{
	u32 i, j, edc;

	for (i = 0; i < 256; i++) {
		j = (i << 1) ^ (i & 0x80 ? 0x11d : 0);
		fsg_ecc_f[i] = j;
		fsg_ecc_b[i ^ j] = i;
		edc = i;
		for (j = 0; j < 8; j++)
			edc = (edc >> 1) ^ (edc & 1 ? 0xd8018001 : 0);
		fsg_edc_table[i] = edc;
	}
}

static u32 fsg_cd_edc(const u8 *data, size_t len)
{
	u32 edc = 0;

	while (len--)
		edc = (edc >> 8) ^ fsg_edc_table[(edc ^ *data++) & 0xff];
	return edc;
}

/* One of the two Reed-Solomon product code parities, P or Q */
static void fsg_cd_ecc(const u8 *src, u32 majors, u32 minors, u32 major_mult,
		       u32 minor_inc, u8 *dest)
{
	u32 size = majors * minors, major, minor;

	for (major = 0; major < majors; major++) {
		u32 index = (major >> 1) * major_mult + (major & 1);
		u8 a = 0, b = 0;

		for (minor = 0; minor < minors; minor++) {
			u8 v = src[index];

			index += minor_inc;
			if (index >= size)
				index -= size;
			a ^= v;
			b ^= v;
			a = fsg_ecc_f[a];
		}
		a = fsg_ecc_b[fsg_ecc_f[a] ^ b];
		dest[major] = a;
		dest[major + majors] = a ^ b;
	}
}

/* The EDC and ECC of a mode 1 sector with its sync, header and data */
static void fsg_cd_mode1_edc_ecc(u8 *raw)
{
	DO_ONCE(fsg_cd_init_tables);
	put_unaligned_le32(fsg_cd_edc(raw, 2064), raw + 2064);
	memset(raw + 2068, 0, 8);
	fsg_cd_ecc(raw + 12, 86, 24, 2, 86, raw + 2076);
	fsg_cd_ecc(raw + 12, 52, 43, 86, 88, raw + 2248);
}

/* Minutes, seconds and frames in BCD, as the disc has them */
static void fsg_cd_msf_bcd(u8 *dest, u32 frames)
{
	dest[0] = bin2bcd(frames / (60 * 75));
	dest[1] = bin2bcd(frames / 75 % 60);
	dest[2] = bin2bcd(frames % 75);
}

static bool fsg_cd_has_tracks(struct fsg_lun *curlun)
{
	return curlun->tracks && fsg_lun_is_open(curlun);
}

/* The track a sector is in, NULL past the last */
static const struct fsg_cd_track *fsg_cd_track_of(struct fsg_lun *curlun,
						   u32 lba)
{
	unsigned int i;

	for (i = 0; i < curlun->num_tracks; i++)
		if (lba < curlun->tracks[i].end)
			return &curlun->tracks[i];
	return NULL;
}

/* Part of a sector as the file keeps it */
static int fsg_cd_pread(struct fsg_lun *curlun, const struct fsg_cd_track *t,
			u32 lba, unsigned int skip, void *buf, size_t len)
{
	loff_t pos = t->offset + (loff_t)(lba - t->first) * t->size + skip;

	return kernel_read(curlun->filp, buf, len, &pos) == len ? 0 : -EIO;
}

/*
 * A sector as the disc has it: as the file keeps it, with the sync and the
 * header it leaves out made anew, and for a mode 1 sector its EDC and ECC
 * when asked for.
 */
static int fsg_cd_raw(struct fsg_lun *curlun, const struct fsg_cd_track *t,
		      u32 lba, u8 *raw, bool edc_ecc)
{
	if (t->size == 2352)
		return fsg_cd_pread(curlun, t, lba, 0, raw, 2352);

	if (fsg_cd_pread(curlun, t, lba, 0, raw + 16, t->size))
		return -EIO;
	memcpy(raw, fsg_cd_sync, sizeof(fsg_cd_sync));
	fsg_cd_msf_bcd(raw + 12, lba + 150);
	raw[15] = t->mode;
	if (t->mode == 1 && edc_ecc)
		fsg_cd_mode1_edc_ecc(raw);
	return 0;
}

/*
 * What a sector is read as: audio or mode 1 as its track, and mode 2 either
 * formless, when READ CD asks for that, or in the form its subheader gives.
 */
static int fsg_cd_kind(struct fsg_lun *curlun, const struct fsg_cd_track *t,
		       u32 lba, u8 type, enum fsg_cd_kind *kind)
{
	u8 subheader[8];

	if (t->mode != 2 || type == fsg_cd_types[FSG_CD_MODE2]) {
		*kind = t->mode == 0 ? FSG_CD_AUDIO :
			t->mode == 1 ? FSG_CD_MODE1 : FSG_CD_MODE2;
		return 0;
	}
	if (fsg_cd_pread(curlun, t, lba, t->size == 2352 ? 16 : 0, subheader,
			 sizeof(subheader)))
		return -EIO;
	*kind = subheader[2] & 0x20 ? FSG_CD_FORM2 : FSG_CD_FORM1;
	return 0;
}

/* The Q subchannel in mode 1, where a sector is, with its CRC */
static void fsg_cd_q(const struct fsg_cd_track *t, u32 lba, u8 *q)
{
	bool pregap = lba < t->start;
	u16 crc = 0;
	int i, j;

	q[0] = t->control << 4 | 0x01;
	q[1] = bin2bcd(t->number);
	q[2] = pregap ? 0 : 1;
	fsg_cd_msf_bcd(q + 3, pregap ? t->start - lba : lba - t->start);
	q[6] = 0;
	fsg_cd_msf_bcd(q + 7, lba + 150);
	for (i = 0; i < 10; i++) {
		crc ^= q[i] << 8;
		for (j = 0; j < 8; j++)
			crc = crc & 0x8000 ? crc << 1 ^ 0x1021 : crc << 1;
	}
	put_unaligned_be16(~crc, q + 10);
}

/* READ CD's subchannel of a sector: 1 raw P-W, 2 Q, 4 R-W */
static void fsg_cd_subchannel(const struct fsg_cd_track *t, u32 lba, u8 sub,
			      u8 *out)
{
	u8 p = lba < t->start ? 0x80 : 0, q[12];
	int i;

	memset(out, 0, 96);
	if (sub == 4)
		return;

	fsg_cd_q(t, lba, q);
	if (sub == 2) {
		memcpy(out, q, sizeof(q));
		out[15] = p;
		return;
	}
	for (i = 0; i < 96; i++)
		out[i] = p | ((q[i / 8] >> (7 - i % 8)) & 1) << 6;
}

static unsigned int fsg_cd_sub_size(u8 sub)
{
	return sub == 2 ? 16 : sub ? 96 : 0;
}

/* The C2 error pointers, all clear, and with the block error byte */
static unsigned int fsg_cd_c2_size(u8 select)
{
	static const u16 sizes[] = { 0, 294, 296 };

	return sizes[(select >> 1) & 3];
}

static unsigned int fsg_cd_main_size(enum fsg_cd_kind kind, u8 select)
{
	unsigned int i, size = 0;

	for (i = 0; i < ARRAY_SIZE(fsg_cd_part_bits); i++)
		if (select & fsg_cd_part_bits[i])
			size += fsg_cd_parts[kind][i].len;
	return size;
}

/* What goes to the host, a buffer at a time */
struct fsg_cd_out {
	struct fsg_buffhd	*bh;
	u32			used;
	u32			left;
};

/* Puts data, or zeros for NULL, into the buffers, sending each full one */
static int fsg_cd_put(struct fsg_common *common, struct fsg_cd_out *out,
		      const u8 *data, u32 len)
{
	int rc;

	while (len && out->left) {
		u32 n;

		if (!out->bh || out->used == FSG_BUFLEN) {
			if (out->bh) {
				out->bh->inreq->length = out->used;
				out->bh->inreq->zero = 0;
				out->bh->state = BUF_STATE_FULL;
				if (!start_in_transfer(common, out->bh))
					return -EIO;
				common->next_buffhd_to_fill = out->bh->next;
			}
			out->bh = common->next_buffhd_to_fill;
			rc = sleep_thread(common, false, out->bh);
			if (rc) {
				out->bh = NULL;
				return rc;
			}
			out->used = 0;
		}

		n = min3(len, FSG_BUFLEN - out->used, out->left);
		if (data) {
			memcpy(out->bh->buf + out->used, data, n);
			data += n;
		} else {
			memset(out->bh->buf + out->used, 0, n);
		}
		out->used += n;
		out->left -= n;
		len -= n;
		common->residue -= n;
	}
	return 0;
}

/* The last buffer is for finish_reply to send */
static int fsg_cd_put_end(struct fsg_cd_out *out)
{
	if (out->bh) {
		out->bh->inreq->length = out->used;
		out->bh->state = BUF_STATE_FULL;
	}
	return -EIO;		/* No default reply */
}

static void fsg_cd_sense(struct fsg_lun *curlun, u32 sense, u32 lba)
{
	curlun->sense_data = sense;
	curlun->sense_data_info = lba;
	curlun->info_valid = 1;
}

/* READ(10) and the like: the 2048 bytes of a data sector, of mode 1 or form 1 */
static int fsg_cd_read_data(struct fsg_common *common, u32 lba)
{
	struct fsg_lun *curlun = common->curlun;
	u32 count = common->data_size_from_cmnd >> 11;
	struct fsg_cd_out out = { .left = common->data_size_from_cmnd };
	enum fsg_cd_kind kind;
	u32 i;
	u8 *buf;
	int rc = 0;

	if (count > curlun->num_sectors - lba) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}
	for (i = 0; i < count; i++) {
		const struct fsg_cd_track *t = fsg_cd_track_of(curlun, lba + i);

		if (fsg_cd_kind(curlun, t, lba + i, 0, &kind)) {
			fsg_cd_sense(curlun, SS_UNRECOVERED_READ_ERROR, lba + i);
			return -EINVAL;
		}
		if (kind != FSG_CD_MODE1 && kind != FSG_CD_FORM1) {
			fsg_cd_sense(curlun, SS_ILLEGAL_MODE_FOR_THIS_TRACK,
				     lba + i);
			return -EINVAL;
		}
	}

	buf = kmalloc(2048, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	for (i = 0; i < count && !rc; i++) {
		const struct fsg_cd_track *t = fsg_cd_track_of(curlun, lba + i);
		unsigned int skip = t->size == 2352 ? (t->mode == 1 ? 16 : 24) :
				    t->size == 2336 ? 8 : 0;

		if (fsg_cd_pread(curlun, t, lba + i, skip, buf, 2048)) {
			fsg_cd_sense(curlun, SS_UNRECOVERED_READ_ERROR, lba + i);
			break;
		}
		rc = fsg_cd_put(common, &out, buf, 2048);
	}
	kfree(buf);
	return rc ? rc : fsg_cd_put_end(&out);
}

/*
 * The bytes READ CD sends for what it asks: checks that each sector is of
 * the expected type, gives the sense of what is wrong, else 0.
 */
static u32 fsg_cd_read_cd_size(struct fsg_common *common, u32 *bad)
{
	struct fsg_lun *curlun = common->curlun;
	u8 type = (common->cmnd[1] >> 2) & 0x07;
	u32 lba = get_unaligned_be32(&common->cmnd[2]);
	u32 count = get_unaligned_be24(&common->cmnd[6]);
	u8 select = common->cmnd[9], sub = common->cmnd[10] & 0x07;
	u64 size = 0;
	u32 i;

	common->data_size_from_cmnd = 0;
	if (type > 5 || ((select >> 1) & 3) == 3 || sub == 3 || sub > 4)
		return SS_INVALID_FIELD_IN_CDB;
	if (lba >= curlun->num_sectors || count > curlun->num_sectors - lba)
		return SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;

	for (i = 0; i < count; i++) {
		const struct fsg_cd_track *t = fsg_cd_track_of(curlun, lba + i);
		enum fsg_cd_kind kind;

		*bad = lba + i;
		if (fsg_cd_kind(curlun, t, lba + i, type, &kind))
			return SS_UNRECOVERED_READ_ERROR;
		if (type && type != fsg_cd_types[kind])
			return SS_ILLEGAL_MODE_FOR_THIS_TRACK;
		size += fsg_cd_main_size(kind, select) + fsg_cd_c2_size(select) +
			fsg_cd_sub_size(sub);
	}
	if (size > U32_MAX)
		return SS_INVALID_FIELD_IN_CDB;

	common->data_size_from_cmnd = size;
	return 0;
}

/* READ CD: the parts of each sector asked for, its C2 pointers, its subchannel */
static int fsg_cd_read_cd(struct fsg_common *common)
{
	struct fsg_lun *curlun = common->curlun;
	u8 type = (common->cmnd[1] >> 2) & 0x07;
	u32 lba = get_unaligned_be32(&common->cmnd[2]);
	u32 count = get_unaligned_be24(&common->cmnd[6]);
	u8 select = common->cmnd[9], sub = common->cmnd[10] & 0x07;
	struct fsg_cd_out out = { .left = common->data_size_from_cmnd };
	u8 *raw, *subchannel;
	u32 i, j;
	int rc = 0;

	if (!out.left)
		return 0;

	raw = kmalloc(2352 + 96, GFP_KERNEL);
	if (!raw)
		return -ENOMEM;
	subchannel = raw + 2352;

	for (i = 0; i < count && !rc; i++) {
		u32 at = lba + i;
		const struct fsg_cd_track *t = fsg_cd_track_of(curlun, at);
		enum fsg_cd_kind kind;

		if (fsg_cd_kind(curlun, t, at, type, &kind) ||
		    (fsg_cd_main_size(kind, select) &&
		     fsg_cd_raw(curlun, t, at, raw, select & FSG_CD_EDC_ECC))) {
			fsg_cd_sense(curlun, SS_UNRECOVERED_READ_ERROR, at);
			break;
		}
		for (j = 0; j < ARRAY_SIZE(fsg_cd_part_bits) && !rc; j++)
			if (select & fsg_cd_part_bits[j])
				rc = fsg_cd_put(common, &out,
						raw + fsg_cd_parts[kind][j].at,
						fsg_cd_parts[kind][j].len);
		if (!rc)
			rc = fsg_cd_put(common, &out, NULL, fsg_cd_c2_size(select));
		if (!rc && sub) {
			fsg_cd_subchannel(t, at, sub, subchannel);
			rc = fsg_cd_put(common, &out, subchannel,
					fsg_cd_sub_size(sub));
		}
		curlun->cd_position = at;
	}
	kfree(raw);
	return rc ? rc : fsg_cd_put_end(&out);
}

/* One entry of the raw TOC, as the lead-in's Q subchannel has it */
static u8 *fsg_cd_raw_toc_entry(u8 *p, u8 control, u8 point, u8 pmin, u8 psec,
				u8 pframe)
{
	memset(p, 0, 11);
	p[0] = 1;			/* Session */
	p[1] = 0x10 | control;		/* ADR 1 */
	p[3] = point;
	p[8] = pmin;
	p[9] = psec;
	p[10] = pframe;
	return p + 11;
}

static u8 *fsg_cd_raw_toc_address(u8 *p, u8 control, u8 point, u32 lba)
{
	u32 frames = lba + 150;

	return fsg_cd_raw_toc_entry(p, control, point, frames / (60 * 75),
				    frames / 75 % 60, frames % 75);
}

static u8 *fsg_cd_toc_entry(u8 *p, int msf, u8 control, u8 number, u32 lba)
{
	memset(p, 0, 8);
	p[1] = 0x10 | control;		/* ADR 1 */
	p[2] = number;
	store_cdrom_address(&p[4], msf, lba);
	return p + 8;
}

/* READ TOC of the tracks: formatted, the session, or raw */
static int fsg_cd_read_toc(struct fsg_common *common, struct fsg_buffhd *bh,
			   u8 format)
{
	struct fsg_lun *curlun = common->curlun;
	const struct fsg_cd_track *tracks = curlun->tracks;
	const struct fsg_cd_track *last = &tracks[curlun->num_tracks - 1];
	int msf = common->cmnd[1] & 0x02;
	u8 start_track = common->cmnd[6];
	u8 *buf = (u8 *)bh->buf, *p = buf + 4;
	bool xa = false;
	unsigned int i;

	for (i = 0; i < curlun->num_tracks; i++)
		xa |= tracks[i].mode == 2;

	switch (format) {
	case 0:			/* Formatted TOC */
		if (start_track > last->number && start_track != 0xaa) {
			curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
			return -EINVAL;
		}
		buf[2] = tracks[0].number;
		buf[3] = last->number;
		for (i = 0; i < curlun->num_tracks; i++)
			if (tracks[i].number >= start_track)
				p = fsg_cd_toc_entry(p, msf, tracks[i].control,
						     tracks[i].number,
						     tracks[i].start);
		p = fsg_cd_toc_entry(p, msf, last->control, 0xaa,
				     curlun->num_sectors);
		break;

	case 1:			/* The first track of the last session */
		buf[2] = 1;
		buf[3] = 1;
		p = fsg_cd_toc_entry(p, msf, tracks[0].control,
				     tracks[0].number, tracks[0].start);
		break;

	case 2:			/* Raw TOC: A0, A1, A2, then the tracks */
		buf[2] = 1;
		buf[3] = 1;
		p = fsg_cd_raw_toc_entry(p, tracks[0].control, 0xa0,
					 tracks[0].number, xa ? 0x20 : 0, 0);
		p = fsg_cd_raw_toc_entry(p, last->control, 0xa1, last->number,
					 0, 0);
		p = fsg_cd_raw_toc_address(p, last->control, 0xa2,
					   curlun->num_sectors);
		for (i = 0; i < curlun->num_tracks; i++)
			p = fsg_cd_raw_toc_address(p, tracks[i].control,
						   tracks[i].number,
						   tracks[i].start);
		break;

	default:
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	put_unaligned_be16(p - buf - 2, buf);
	return p - buf;
}

/* GET EVENT STATUS NOTIFICATION, polled: the media events */
static int do_get_event_status_notification(struct fsg_common *common,
					    struct fsg_buffhd *bh)
{
	struct fsg_lun *curlun = common->curlun;
	u8 *buf = (u8 *)bh->buf;

	if (!(common->cmnd[1] & 0x01)) {	/* Polled only */
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	memset(buf, 0, 8);
	buf[3] = 0x10;				/* Media events */
	if (!(common->cmnd[4] & 0x10)) {
		put_unaligned_be16(2, buf);
		buf[2] = 0x80;			/* No event available */
		return 4;
	}

	put_unaligned_be16(6, buf);
	buf[2] = 0x04;				/* Media class */
	buf[4] = curlun->media_event;
	buf[5] = fsg_lun_is_open(curlun) ? 0x02 : 0;	/* Media present */
	curlun->media_event = FSG_MEDIA_NO_CHANGE;
	return 8;
}

/*
 * READ SUB-CHANNEL: where READ CD read last, with no audio playing; no
 * media catalog number nor ISRC. A medium without tracks is one data track.
 */
static int do_read_subchannel(struct fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun *curlun = common->curlun;
	struct fsg_cd_track one = {
		.end = curlun->num_sectors, .number = 1, .control = 0x4,
		.mode = 1,
	};
	const struct fsg_cd_track *t = &one;
	int msf = common->cmnd[1] & 0x02;
	u8 format = common->cmnd[3];
	u8 *buf = (u8 *)bh->buf;
	u32 at = curlun->cd_position, rel;

	if (format < 1 || format > 3) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}
	if (fsg_cd_has_tracks(curlun))
		t = fsg_cd_track_of(curlun, at) ?: &curlun->tracks[0];
	if (at >= t->end)
		at = t->start;

	memset(buf, 0, 24);
	buf[1] = 0x15;			/* No current audio status */
	if (!(common->cmnd[2] & 0x40))	/* Only the header */
		return 4;

	buf[4] = format;
	switch (format) {
	case 1:				/* Current position */
		put_unaligned_be16(12, &buf[2]);
		buf[5] = 0x10 | t->control;
		buf[6] = t->number;
		buf[7] = at < t->start ? 0 : 1;
		store_cdrom_address(&buf[8], msf, at);
		rel = at < t->start ? t->start - at : at - t->start;
		if (msf) {
			buf[13] = rel / (60 * 75);
			buf[14] = rel / 75 % 60;
			buf[15] = rel % 75;
		} else {
			put_unaligned_be32(at < t->start ? -rel : rel,
					   &buf[12]);
		}
		return 16;
	case 3:				/* ISRC of a track */
		buf[5] = 0x10 | t->control;
		buf[6] = common->cmnd[6];
		fallthrough;
	default:			/* Media catalog number */
		put_unaligned_be16(20, &buf[2]);
		return 24;
	}
}

/*-------------------------------------------------------------------------*/

static int do_read(struct fsg_common *common)
{
	struct fsg_lun		*curlun = common->curlun;
	u64			lba;
	struct fsg_buffhd	*bh;
	int			rc;
	u32			amount_left;
	loff_t			file_offset, file_offset_tmp;
	unsigned int		amount;
	ssize_t			nread;

	/*
	 * Get the starting Logical Block Address and check that it's
	 * not too big.
	 */
	if (common->cmnd[0] == READ_6)
		lba = get_unaligned_be24(&common->cmnd[1]);
	else {
		if (common->cmnd[0] == READ_16)
			lba = get_unaligned_be64(&common->cmnd[2]);
		else		/* READ_10 or READ_12 */
			lba = get_unaligned_be32(&common->cmnd[2]);

		/*
		 * We allow DPO (Disable Page Out = don't save data in the
		 * cache) and FUA (Force Unit Access = don't read from the
		 * cache), but we don't implement them.
		 */
		if ((common->cmnd[1] & ~0x18) != 0) {
			curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
			return -EINVAL;
		}
	}
	if (lba >= curlun->num_sectors) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}
	file_offset = ((loff_t) lba) << curlun->blkbits;

	/* Carry out the file reads */
	amount_left = common->data_size_from_cmnd;
	if (unlikely(amount_left == 0))
		return -EIO;		/* No default reply */
	if (fsg_cd_has_tracks(curlun))
		return fsg_cd_read_data(common, lba);

	for (;;) {
		/*
		 * Figure out how much we need to read:
		 * Try to read the remaining amount.
		 * But don't read more than the buffer size.
		 * And don't try to read past the end of the file.
		 */
		amount = min(amount_left, FSG_BUFLEN);
		amount = min_t(loff_t, amount,
			     curlun->file_length - file_offset);

		/* Wait for the next buffer to become available */
		bh = common->next_buffhd_to_fill;
		rc = sleep_thread(common, false, bh);
		if (rc)
			return rc;

		/*
		 * If we were asked to read past the end of file,
		 * end with an empty buffer.
		 */
		if (amount == 0) {
			curlun->sense_data =
					SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
			curlun->sense_data_info =
					file_offset >> curlun->blkbits;
			curlun->info_valid = 1;
			bh->inreq->length = 0;
			bh->state = BUF_STATE_FULL;
			break;
		}

		/* Perform the read */
		file_offset_tmp = file_offset;
		nread = kernel_read(curlun->filp, bh->buf, amount,
				&file_offset_tmp);
		VLDBG(curlun, "file read %u @ %llu -> %d\n", amount,
		      (unsigned long long)file_offset, (int)nread);
		if (signal_pending(current))
			return -EINTR;

		if (nread < 0) {
			LDBG(curlun, "error in file read: %d\n", (int)nread);
			nread = 0;
		} else if (nread < amount) {
			LDBG(curlun, "partial file read: %d/%u\n",
			     (int)nread, amount);
			nread = round_down(nread, curlun->blksize);
		}
		file_offset  += nread;
		amount_left  -= nread;
		common->residue -= nread;

		/*
		 * Except at the end of the transfer, nread will be
		 * equal to the buffer size, which is divisible by the
		 * bulk-in maxpacket size.
		 */
		bh->inreq->length = nread;
		bh->state = BUF_STATE_FULL;

		/* If an error occurred, report it and its position */
		if (nread < amount) {
			curlun->sense_data = SS_UNRECOVERED_READ_ERROR;
			curlun->sense_data_info =
					file_offset >> curlun->blkbits;
			curlun->info_valid = 1;
			break;
		}

		if (amount_left == 0)
			break;		/* No more left to read */

		/* Send this buffer and go read some more */
		bh->inreq->zero = 0;
		if (!start_in_transfer(common, bh))
			/* Don't know what to do if common->fsg is NULL */
			return -EIO;
		common->next_buffhd_to_fill = bh->next;
	}

	return -EIO;		/* No default reply */
}


/*-------------------------------------------------------------------------*/

static int do_write(struct fsg_common *common)
{
	struct fsg_lun		*curlun = common->curlun;
	u64			lba;
	struct fsg_buffhd	*bh;
	int			get_some_more;
	u32			amount_left_to_req, amount_left_to_write;
	loff_t			usb_offset, file_offset, file_offset_tmp;
	unsigned int		amount;
	ssize_t			nwritten;
	int			rc;

	if (curlun->ro) {
		curlun->sense_data = SS_WRITE_PROTECTED;
		return -EINVAL;
	}
	spin_lock(&curlun->filp->f_lock);
	curlun->filp->f_flags &= ~O_SYNC;	/* Default is not to wait */
	spin_unlock(&curlun->filp->f_lock);

	/*
	 * Get the starting Logical Block Address and check that it's
	 * not too big
	 */
	if (common->cmnd[0] == WRITE_6)
		lba = get_unaligned_be24(&common->cmnd[1]);
	else {
		if (common->cmnd[0] == WRITE_16)
			lba = get_unaligned_be64(&common->cmnd[2]);
		else		/* WRITE_10 or WRITE_12 */
			lba = get_unaligned_be32(&common->cmnd[2]);

		/*
		 * We allow DPO (Disable Page Out = don't save data in the
		 * cache) and FUA (Force Unit Access = write directly to the
		 * medium).  We don't implement DPO; we implement FUA by
		 * performing synchronous output.
		 */
		if (common->cmnd[1] & ~0x18) {
			curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
			return -EINVAL;
		}
		if (!curlun->nofua && (common->cmnd[1] & 0x08)) { /* FUA */
			spin_lock(&curlun->filp->f_lock);
			curlun->filp->f_flags |= O_SYNC;
			spin_unlock(&curlun->filp->f_lock);
		}
	}
	if (lba >= curlun->num_sectors) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}

	/* Carry out the file writes */
	get_some_more = 1;
	file_offset = usb_offset = ((loff_t) lba) << curlun->blkbits;
	amount_left_to_req = common->data_size_from_cmnd;
	amount_left_to_write = common->data_size_from_cmnd;

	while (amount_left_to_write > 0) {

		/* Queue a request for more data from the host */
		bh = common->next_buffhd_to_fill;
		if (bh->state == BUF_STATE_EMPTY && get_some_more) {

			/*
			 * Figure out how much we want to get:
			 * Try to get the remaining amount,
			 * but not more than the buffer size.
			 */
			amount = min(amount_left_to_req, FSG_BUFLEN);

			/* Beyond the end of the backing file? */
			if (usb_offset >= curlun->file_length) {
				get_some_more = 0;
				curlun->sense_data =
					SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
				curlun->sense_data_info =
					usb_offset >> curlun->blkbits;
				curlun->info_valid = 1;
				continue;
			}

			/* Get the next buffer */
			usb_offset += amount;
			common->usb_amount_left -= amount;
			amount_left_to_req -= amount;
			if (amount_left_to_req == 0)
				get_some_more = 0;

			/*
			 * Except at the end of the transfer, amount will be
			 * equal to the buffer size, which is divisible by
			 * the bulk-out maxpacket size.
			 */
			set_bulk_out_req_length(common, bh, amount);
			if (!start_out_transfer(common, bh))
				/* Dunno what to do if common->fsg is NULL */
				return -EIO;
			common->next_buffhd_to_fill = bh->next;
			continue;
		}

		/* Write the received data to the backing file */
		bh = common->next_buffhd_to_drain;
		if (bh->state == BUF_STATE_EMPTY && !get_some_more)
			break;			/* We stopped early */

		/* Wait for the data to be received */
		rc = sleep_thread(common, false, bh);
		if (rc)
			return rc;

		common->next_buffhd_to_drain = bh->next;
		bh->state = BUF_STATE_EMPTY;

		/* Did something go wrong with the transfer? */
		if (bh->outreq->status != 0) {
			curlun->sense_data = SS_COMMUNICATION_FAILURE;
			curlun->sense_data_info =
					file_offset >> curlun->blkbits;
			curlun->info_valid = 1;
			break;
		}

		amount = bh->outreq->actual;
		if (curlun->file_length - file_offset < amount) {
			LERROR(curlun, "write %u @ %llu beyond end %llu\n",
				       amount, (unsigned long long)file_offset,
				       (unsigned long long)curlun->file_length);
			amount = curlun->file_length - file_offset;
		}

		/*
		 * Don't accept excess data.  The spec doesn't say
		 * what to do in this case.  We'll ignore the error.
		 */
		amount = min(amount, bh->bulk_out_intended_length);

		/* Don't write a partial block */
		amount = round_down(amount, curlun->blksize);
		if (amount == 0)
			goto empty_write;

		/* Perform the write */
		file_offset_tmp = file_offset;
		nwritten = kernel_write(curlun->filp, bh->buf, amount,
				&file_offset_tmp);
		VLDBG(curlun, "file write %u @ %llu -> %d\n", amount,
				(unsigned long long)file_offset, (int)nwritten);
		if (signal_pending(current))
			return -EINTR;		/* Interrupted! */

		if (nwritten < 0) {
			LDBG(curlun, "error in file write: %d\n",
					(int) nwritten);
			nwritten = 0;
		} else if (nwritten < amount) {
			LDBG(curlun, "partial file write: %d/%u\n",
					(int) nwritten, amount);
			nwritten = round_down(nwritten, curlun->blksize);
		}
		file_offset += nwritten;
		amount_left_to_write -= nwritten;
		common->residue -= nwritten;

		/* If an error occurred, report it and its position */
		if (nwritten < amount) {
			curlun->sense_data = SS_WRITE_ERROR;
			curlun->sense_data_info =
					file_offset >> curlun->blkbits;
			curlun->info_valid = 1;
			break;
		}

 empty_write:
		/* Did the host decide to stop early? */
		if (bh->outreq->actual < bh->bulk_out_intended_length) {
			common->short_packet_received = 1;
			break;
		}
	}

	return -EIO;		/* No default reply */
}


/*-------------------------------------------------------------------------*/

static int do_synchronize_cache(struct fsg_common *common)
{
	struct fsg_lun	*curlun = common->curlun;
	int		rc;

	/* We ignore the requested LBA and write out all file's
	 * dirty data buffers. */
	rc = fsg_lun_fsync_sub(curlun);
	if (rc)
		curlun->sense_data = SS_WRITE_ERROR;
	return 0;
}


/*-------------------------------------------------------------------------*/

static void invalidate_sub(struct fsg_lun *curlun)
{
	struct file	*filp = curlun->filp;
	struct inode	*inode = file_inode(filp);
	unsigned long __maybe_unused	rc;

	rc = invalidate_mapping_pages(inode->i_mapping, 0, -1);
	VLDBG(curlun, "invalidate_mapping_pages -> %ld\n", rc);
}

static int do_verify(struct fsg_common *common)
{
	struct fsg_lun		*curlun = common->curlun;
	u32			lba;
	u32			verification_length;
	struct fsg_buffhd	*bh = common->next_buffhd_to_fill;
	loff_t			file_offset, file_offset_tmp;
	u32			amount_left;
	unsigned int		amount;
	ssize_t			nread;

	/*
	 * Get the starting Logical Block Address and check that it's
	 * not too big.
	 */
	lba = get_unaligned_be32(&common->cmnd[2]);
	if (lba >= curlun->num_sectors) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}

	/*
	 * We allow DPO (Disable Page Out = don't save data in the
	 * cache) but we don't implement it.
	 */
	if (common->cmnd[1] & ~0x10) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	verification_length = get_unaligned_be16(&common->cmnd[7]);
	if (unlikely(verification_length == 0))
		return -EIO;		/* No default reply */

	/* Prepare to carry out the file verify */
	amount_left = verification_length << curlun->blkbits;
	file_offset = ((loff_t) lba) << curlun->blkbits;

	/* Write out all the dirty buffers before invalidating them */
	fsg_lun_fsync_sub(curlun);
	if (signal_pending(current))
		return -EINTR;

	invalidate_sub(curlun);
	if (signal_pending(current))
		return -EINTR;

	/* Just try to read the requested blocks */
	while (amount_left > 0) {
		/*
		 * Figure out how much we need to read:
		 * Try to read the remaining amount, but not more than
		 * the buffer size.
		 * And don't try to read past the end of the file.
		 */
		amount = min(amount_left, FSG_BUFLEN);
		amount = min_t(loff_t, amount,
			     curlun->file_length - file_offset);
		if (amount == 0) {
			curlun->sense_data =
					SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
			curlun->sense_data_info =
				file_offset >> curlun->blkbits;
			curlun->info_valid = 1;
			break;
		}

		/* Perform the read */
		file_offset_tmp = file_offset;
		nread = kernel_read(curlun->filp, bh->buf, amount,
				&file_offset_tmp);
		VLDBG(curlun, "file read %u @ %llu -> %d\n", amount,
				(unsigned long long) file_offset,
				(int) nread);
		if (signal_pending(current))
			return -EINTR;

		if (nread < 0) {
			LDBG(curlun, "error in file verify: %d\n", (int)nread);
			nread = 0;
		} else if (nread < amount) {
			LDBG(curlun, "partial file verify: %d/%u\n",
			     (int)nread, amount);
			nread = round_down(nread, curlun->blksize);
		}
		if (nread == 0) {
			curlun->sense_data = SS_UNRECOVERED_READ_ERROR;
			curlun->sense_data_info =
				file_offset >> curlun->blkbits;
			curlun->info_valid = 1;
			break;
		}
		file_offset += nread;
		amount_left -= nread;
	}
	return 0;
}


/*-------------------------------------------------------------------------*/

static int do_inquiry(struct fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun *curlun = common->curlun;
	u8	*buf = (u8 *) bh->buf;

	if (!curlun) {		/* Unsupported LUNs are okay */
		common->bad_lun_okay = 1;
		memset(buf, 0, 36);
		buf[0] = TYPE_NO_LUN;	/* Unsupported, no device-type */
		buf[4] = 31;		/* Additional length */
		return 36;
	}

	buf[0] = curlun->cdrom ? TYPE_ROM : TYPE_DISK;
	buf[1] = curlun->removable ? 0x80 : 0;
	buf[2] = 2;		/* ANSI SCSI level 2 */
	buf[3] = 2;		/* SCSI-2 INQUIRY data format */
	buf[4] = 31;		/* Additional length */
	buf[5] = 0;		/* No special options */
	buf[6] = 0;
	buf[7] = 0;
	if (curlun->inquiry_string[0])
		memcpy(buf + 8, curlun->inquiry_string,
		       sizeof(curlun->inquiry_string));
	else
		memcpy(buf + 8, common->inquiry_string,
		       sizeof(common->inquiry_string));
	return 36;
}

static int do_request_sense(struct fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	u8		*buf = (u8 *) bh->buf;
	u32		sd, sdinfo;
	int		valid;

	/*
	 * From the SCSI-2 spec., section 7.9 (Unit attention condition):
	 *
	 * If a REQUEST SENSE command is received from an initiator
	 * with a pending unit attention condition (before the target
	 * generates the contingent allegiance condition), then the
	 * target shall either:
	 *   a) report any pending sense data and preserve the unit
	 *	attention condition on the logical unit, or,
	 *   b) report the unit attention condition, may discard any
	 *	pending sense data, and clear the unit attention
	 *	condition on the logical unit for that initiator.
	 *
	 * FSG normally uses option a); enable this code to use option b).
	 */
#if 0
	if (curlun && curlun->unit_attention_data != SS_NO_SENSE) {
		curlun->sense_data = curlun->unit_attention_data;
		curlun->unit_attention_data = SS_NO_SENSE;
	}
#endif

	if (!curlun) {		/* Unsupported LUNs are okay */
		common->bad_lun_okay = 1;
		sd = SS_LOGICAL_UNIT_NOT_SUPPORTED;
		sdinfo = 0;
		valid = 0;
	} else {
		sd = curlun->sense_data;
		sdinfo = curlun->sense_data_info;
		valid = curlun->info_valid << 7;
		curlun->sense_data = SS_NO_SENSE;
		curlun->sense_data_info = 0;
		curlun->info_valid = 0;
	}

	memset(buf, 0, 18);
	buf[0] = valid | 0x70;			/* Valid, current error */
	buf[2] = SK(sd);
	put_unaligned_be32(sdinfo, &buf[3]);	/* Sense information */
	buf[7] = 18 - 8;			/* Additional sense length */
	buf[12] = ASC(sd);
	buf[13] = ASCQ(sd);
	return 18;
}

static int do_read_capacity(struct fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	u32		lba = get_unaligned_be32(&common->cmnd[2]);
	int		pmi = common->cmnd[8];
	u8		*buf = (u8 *)bh->buf;
	u32		max_lba;

	/* Check the PMI and LBA fields */
	if (pmi > 1 || (pmi == 0 && lba != 0)) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	if (curlun->num_sectors < 0x100000000ULL)
		max_lba = curlun->num_sectors - 1;
	else
		max_lba = 0xffffffff;
	put_unaligned_be32(max_lba, &buf[0]);		/* Max logical block */
	put_unaligned_be32(curlun->blksize, &buf[4]);	/* Block length */
	return 8;
}

static int do_read_capacity_16(struct fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun  *curlun = common->curlun;
	u64		lba = get_unaligned_be64(&common->cmnd[2]);
	int		pmi = common->cmnd[14];
	u8		*buf = (u8 *)bh->buf;

	/* Check the PMI and LBA fields */
	if (pmi > 1 || (pmi == 0 && lba != 0)) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	put_unaligned_be64(curlun->num_sectors - 1, &buf[0]);
							/* Max logical block */
	put_unaligned_be32(curlun->blksize, &buf[8]);	/* Block length */

	/* It is safe to keep other fields zeroed */
	memset(&buf[12], 0, 32 - 12);
	return 32;
}

static int do_read_header(struct fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	int		msf = common->cmnd[1] & 0x02;
	u32		lba = get_unaligned_be32(&common->cmnd[2]);
	u8		*buf = (u8 *)bh->buf;

	if (common->cmnd[1] & ~0x02) {		/* Mask away MSF */
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}
	if (lba >= curlun->num_sectors) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}

	memset(buf, 0, 8);
	buf[0] = 0x01;		/* 2048 bytes of user data, rest is EC */
	if (fsg_cd_has_tracks(curlun)) {
		const struct fsg_cd_track *t = fsg_cd_track_of(curlun, lba);

		if (!t->mode) {
			curlun->sense_data = SS_ILLEGAL_MODE_FOR_THIS_TRACK;
			return -EINVAL;
		}
		buf[0] = t->mode;
	}
	store_cdrom_address(&buf[4], msf, lba);
	return 8;
}

static int do_read_toc(struct fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	int		msf = common->cmnd[1] & 0x02;
	int		start_track = common->cmnd[6];
	u8		*buf = (u8 *)bh->buf;
	u8		format;
	int		i, len;

	format = common->cmnd[2] & 0xf;

	if ((common->cmnd[1] & ~0x02) != 0 ||	/* Mask away MSF */
			(start_track > 1 && format != 0x1 &&
			 !fsg_cd_has_tracks(curlun))) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	/*
	 * Check if CDB is old style SFF-8020i
	 * i.e. format is in 2 MSBs of byte 9
	 * Mac OS-X host sends us this.
	 */
	if (format == 0)
		format = (common->cmnd[9] >> 6) & 0x3;

	if (fsg_cd_has_tracks(curlun))
		return fsg_cd_read_toc(common, bh, format);

	switch (format) {
	case 0:	/* Formatted TOC */
	case 1:	/* Multi-session info */
		len = 4 + 2*8;		/* 4 byte header + 2 descriptors */
		memset(buf, 0, len);
		buf[1] = len - 2;	/* TOC Length excludes length field */
		buf[2] = 1;		/* First track number */
		buf[3] = 1;		/* Last track number */
		buf[5] = 0x16;		/* Data track, copying allowed */
		buf[6] = 0x01;		/* Only track is number 1 */
		store_cdrom_address(&buf[8], msf, 0);

		buf[13] = 0x16;		/* Lead-out track is data */
		buf[14] = 0xAA;		/* Lead-out track number */
		store_cdrom_address(&buf[16], msf, curlun->num_sectors);
		return len;

	case 2:
		/* Raw TOC */
		len = 4 + 3*11;		/* 4 byte header + 3 descriptors */
		memset(buf, 0, len);	/* Header + A0, A1 & A2 descriptors */
		buf[1] = len - 2;	/* TOC Length excludes length field */
		buf[2] = 1;		/* First complete session */
		buf[3] = 1;		/* Last complete session */

		buf += 4;
		/* fill in A0, A1 and A2 points */
		for (i = 0; i < 3; i++) {
			buf[0] = 1;	/* Session number */
			buf[1] = 0x16;	/* Data track, copying allowed */
			/* 2 - Track number 0 ->  TOC */
			buf[3] = 0xA0 + i; /* A0, A1, A2 point */
			/* 4, 5, 6 - Min, sec, frame is zero */
			buf[8] = 1;	/* Pmin: last track number */
			buf += 11;	/* go to next track descriptor */
		}
		buf -= 11;		/* go back to A2 descriptor */

		/* For A2, 7, 8, 9, 10 - zero, Pmin, Psec, Pframe of Lead out */
		store_cdrom_address(&buf[7], msf, curlun->num_sectors);
		return len;

	default:
		/* PMA, ATIP, CD-TEXT not supported/required */
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}
}

/* MMC profiles, and the sectors of a single-layer DVD */
#define FSG_MMC_PROFILE_CD_ROM		0x0008
#define FSG_MMC_PROFILE_DVD_ROM		0x0010
#define FSG_MMC_PROFILE_BD_ROM		0x0040
#define FSG_DVD_SL_SECTORS		2295104

/* The current profile, or none without a medium */
static u16 fsg_current_profile(struct fsg_lun *curlun)
{
	if (!fsg_lun_is_open(curlun))
		return 0;

	switch (curlun->profile) {
	case FSG_PROFILE_DVD:
		return FSG_MMC_PROFILE_DVD_ROM;
	case FSG_PROFILE_BD:
		return FSG_MMC_PROFILE_BD_ROM;
	default:
		return FSG_MMC_PROFILE_CD_ROM;
	}
}

/* A feature descriptor's header; its data, zeroed, follows */
static u8 *fsg_feature(u8 *p, u16 code, u8 version, bool persistent,
		       bool is_current, u8 length)
{
	put_unaligned_be16(code, p);
	p[2] = (version << 2) | (persistent << 1) | is_current;
	p[3] = length;
	memset(p + 4, 0, length);
	return p + 4;
}

static int do_get_configuration(struct fsg_common *common,
				struct fsg_buffhd *bh)
{
	static const u16 profiles[] = {
		FSG_MMC_PROFILE_BD_ROM,
		FSG_MMC_PROFILE_DVD_ROM,
		FSG_MMC_PROFILE_CD_ROM,
	};
	struct fsg_lun	*curlun = common->curlun;
	u8		*buf = (u8 *)bh->buf;
	u8		*p = buf + 8;
	int		rt = common->cmnd[1] & 0x03;
	u16		start = get_unaligned_be16(&common->cmnd[2]);
	u16		profile = fsg_current_profile(curlun);
	int		i;

	if (rt == 3) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	/* RT 0: all from start on, 1: the current ones, 2: start only */
#define FSG_WANT(code, cur) \
	(rt == 2 ? (code) == start : (code) >= start && (rt == 0 || (cur)))

	memset(buf, 0, 8);

	if (FSG_WANT(0x0000, true)) {		/* Profile List */
		p = fsg_feature(p, 0x0000, 0, true, true,
				4 * ARRAY_SIZE(profiles));
		for (i = 0; i < ARRAY_SIZE(profiles); i++, p += 4) {
			put_unaligned_be16(profiles[i], p);
			p[2] = profiles[i] == profile;
		}
	}
	if (FSG_WANT(0x0001, true)) {		/* Core */
		p = fsg_feature(p, 0x0001, 0, true, true, 4);
		put_unaligned_be32(8, p);	/* Physical interface: USB */
		p += 4;
	}
	if (FSG_WANT(0x0002, true)) {		/* Morphing: polled GESN */
		p = fsg_feature(p, 0x0002, 1, true, true, 4);
		p += 4;
	}
	if (FSG_WANT(0x0003, true)) {		/* Removable Medium */
		p = fsg_feature(p, 0x0003, 0, true, true, 4);
		p[0] = (1 << 5) | (1 << 3) | (1 << 0);	/* Tray, eject, lock */
		p += 4;
	}
	if (FSG_WANT(0x0010, profile != 0)) {	/* Random Readable */
		p = fsg_feature(p, 0x0010, 0, false, profile != 0, 8);
		put_unaligned_be32(2048, p);
		put_unaligned_be16(profile == FSG_MMC_PROFILE_BD_ROM ? 32 :
				   profile == FSG_MMC_PROFILE_DVD_ROM ? 16 : 1,
				   p + 4);
		p += 8;
	}
	if (FSG_WANT(0x001e, profile == FSG_MMC_PROFILE_CD_ROM)) { /* CD Read */
		p = fsg_feature(p, 0x001e, 2, false,
				profile == FSG_MMC_PROFILE_CD_ROM, 4);
		p[0] = 0x02;		/* C2 error pointers */
		p += 4;
	}
	if (FSG_WANT(0x001f, profile == FSG_MMC_PROFILE_DVD_ROM)) { /* DVD Read */
		p = fsg_feature(p, 0x001f, 0, false,
				profile == FSG_MMC_PROFILE_DVD_ROM, 0);
	}
	if (FSG_WANT(0x0040, profile == FSG_MMC_PROFILE_BD_ROM)) { /* BD Read */
		p = fsg_feature(p, 0x0040, 0, false,
				profile == FSG_MMC_PROFILE_BD_ROM, 28);
		put_unaligned_be16(0xffff, p + 20);	/* BD-ROM, class 0 */
		p += 28;
	}
#undef FSG_WANT

	put_unaligned_be32(p - buf - 4, buf);
	put_unaligned_be16(profile, &buf[6]);
	return p - buf;
}

/* Physical format information of a DVD-ROM, one or two layers (OTP) */
static int fsg_dvd_physical_format(struct fsg_lun *curlun, u8 *buf)
{
	u32	sectors = curlun->num_sectors;
	u32	start = 0x30000, end, end0 = 0;
	u8	*pfi = buf + 4;
	bool	dual = sectors > FSG_DVD_SL_SECTORS;

	memset(buf, 0, 4 + 2048);
	put_unaligned_be16(2048 + 2, buf);
	pfi[0] = 0x01;			/* DVD-ROM, part version 1 */
	pfi[1] = 0x0f;			/* 120 mm, rate not specified */
	pfi[2] = dual ? 0x31 : 0x01;	/* Layers, OTP, embossed */
	if (dual) {
		u32 layer0 = sectors - sectors / 2;

		end0 = start + layer0 - 1;
		end = (~end0 & 0xffffff) + sectors / 2 - 1;
	} else {
		end = start + sectors - 1;
	}
	put_unaligned_be32(start, &pfi[4]);
	put_unaligned_be32(end, &pfi[8]);
	put_unaligned_be32(end0, &pfi[12]);
	return 4 + 2048;
}

static int do_read_disc_structure(struct fsg_common *common,
				  struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	u8		*buf = (u8 *)bh->buf;
	u8		media = common->cmnd[1] & 0x0f;
	u8		format = common->cmnd[7];

	if (media == 0 && curlun->profile == FSG_PROFILE_DVD) {
		switch (format) {
		case 0x00:		/* Physical format information */
			return fsg_dvd_physical_format(curlun, buf);
		case 0x01:		/* Copyright: none, all regions */
			memset(buf, 0, 8);
			put_unaligned_be16(6, buf);
			return 8;
		case 0xff:		/* The structures there are */
			memset(buf, 0, 12);
			put_unaligned_be16(10, buf);
			buf[4] = 0x00;
			buf[5] = 0x40;	/* Readable */
			put_unaligned_be16(2048 + 4, &buf[6]);
			buf[8] = 0x01;
			buf[9] = 0x40;
			put_unaligned_be16(4 + 4, &buf[10]);
			return 12;
		}
	}

	curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
	return -EINVAL;
}

/* One closed session: its tracks, or the medium's only data track */
static int do_read_disc_information(struct fsg_common *common,
				    struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	u8		*buf = (u8 *)bh->buf;
	u8		first = 1, last = 1, type = 0;
	unsigned int	i;

	if ((common->cmnd[1] & 0x07) != 0) {	/* Standard information only */
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}
	if (fsg_cd_has_tracks(curlun)) {
		first = curlun->tracks[0].number;
		last = curlun->tracks[curlun->num_tracks - 1].number;
		for (i = 0; i < curlun->num_tracks; i++)
			if (curlun->tracks[i].mode == 2)
				type = 0x20;	/* CD-ROM XA */
	}

	memset(buf, 0, 34);
	put_unaligned_be16(32, buf);
	buf[2] = 0x0e;		/* Last session complete, disc complete */
	buf[3] = first;		/* First track */
	buf[4] = 1;		/* Sessions */
	buf[5] = first;		/* First track in the last session */
	buf[6] = last;		/* Last track in the last session */
	buf[7] = 0x20;		/* Unrestricted use */
	buf[8] = type;		/* CD-DA or CD-ROM, or CD-ROM XA */
	put_unaligned_be32(0xffffffff, &buf[16]);	/* No next session */
	put_unaligned_be32(0xffffffff, &buf[20]);	/* No next lead-out */
	return 34;
}

/* A track by a sector in it, by its number, or the first of session 1 */
static int fsg_cd_read_track_information(struct fsg_common *common,
					 struct fsg_buffhd *bh, u32 address)
{
	struct fsg_lun *curlun = common->curlun;
	const struct fsg_cd_track *t = NULL;
	u8 *buf = (u8 *)bh->buf;
	unsigned int i;

	switch (common->cmnd[1] & 0x03) {
	case 0:			/* By block */
		if (address < curlun->num_sectors)
			t = fsg_cd_track_of(curlun, address);
		break;
	case 1:			/* By track */
		for (i = 0; i < curlun->num_tracks; i++)
			if (curlun->tracks[i].number == address)
				t = &curlun->tracks[i];
		break;
	case 2:			/* By session */
		if (address == 1)
			t = &curlun->tracks[0];
		break;
	}
	if (!t) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	memset(buf, 0, 36);
	put_unaligned_be16(34, buf);
	buf[2] = t->number;
	buf[3] = 1;		/* Session */
	buf[5] = t->control;	/* Track mode */
	buf[6] = t->mode ?: 0x0f;	/* Data mode, unknown for audio */
	put_unaligned_be32(t->start, &buf[8]);
	put_unaligned_be32(t->end - t->start, &buf[24]);
	put_unaligned_be32(t->end - 1, &buf[28]);
	return 36;
}

static int do_read_track_information(struct fsg_common *common,
				     struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	u8		*buf = (u8 *)bh->buf;
	u32		address = get_unaligned_be32(&common->cmnd[2]);
	bool		track1;

	if (fsg_cd_has_tracks(curlun))
		return fsg_cd_read_track_information(common, bh, address);

	switch (common->cmnd[1] & 0x03) {
	case 0:			/* By block */
		track1 = address < curlun->num_sectors;
		break;
	case 1:			/* By track */
	case 2:			/* By session */
		track1 = address == 1;
		break;
	default:
		track1 = false;
	}
	if (!track1) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	memset(buf, 0, 36);
	put_unaligned_be16(34, buf);
	buf[2] = 1;		/* Track */
	buf[3] = 1;		/* Session */
	buf[5] = 0x04;		/* Data, recorded uninterrupted */
	buf[6] = 0x01;		/* Mode 1 */
	put_unaligned_be32(curlun->num_sectors, &buf[24]);
	put_unaligned_be32(curlun->num_sectors - 1, &buf[28]);
	return 36;
}

/* READ CD for 2048-byte user data, as READ(10) gives it */
static int do_read_cd(struct fsg_common *common)
{
	struct fsg_lun	*curlun = common->curlun;
	u8		sector_type = (common->cmnd[1] >> 2) & 0x07;

	if ((sector_type != 0 && sector_type != 2) ||	/* Any, or Mode 1 */
	    (common->cmnd[1] & 0x03) != 0 ||
	    common->cmnd[9] != 0x10 ||			/* User data only */
	    common->cmnd[10] != 0) {			/* No subchannel */
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	common->cmnd[1] = 0;
	return do_read(common);
}

static int do_mode_sense(struct fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	int		mscmnd = common->cmnd[0];
	u8		*buf = (u8 *) bh->buf;
	u8		*buf0 = buf;
	int		pc, page_code;
	int		changeable_values, all_pages;
	int		valid_page = 0;
	int		len, limit;

	if ((common->cmnd[1] & ~0x08) != 0) {	/* Mask away DBD */
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}
	pc = common->cmnd[2] >> 6;
	page_code = common->cmnd[2] & 0x3f;
	if (pc == 3) {
		curlun->sense_data = SS_SAVING_PARAMETERS_NOT_SUPPORTED;
		return -EINVAL;
	}
	changeable_values = (pc == 1);
	all_pages = (page_code == 0x3f);

	/*
	 * Write the mode parameter header.  Fixed values are: default
	 * medium type, no cache control (DPOFUA), and no block descriptors.
	 * The only variable value is the WriteProtect bit.  We will fill in
	 * the mode data length later.
	 */
	memset(buf, 0, 8);
	if (mscmnd == MODE_SENSE) {
		buf[2] = (curlun->ro ? 0x80 : 0x00);		/* WP, DPOFUA */
		buf += 4;
		limit = 255;
	} else {			/* MODE_SENSE_10 */
		buf[3] = (curlun->ro ? 0x80 : 0x00);		/* WP, DPOFUA */
		buf += 8;
		limit = 65535;		/* Should really be FSG_BUFLEN */
	}

	/* No block descriptors */

	/*
	 * The mode pages, in numerical order.  The only page we support
	 * is the Caching page.
	 */
	if (page_code == 0x08 || all_pages) {
		valid_page = 1;
		buf[0] = 0x08;		/* Page code */
		buf[1] = 10;		/* Page length */
		memset(buf+2, 0, 10);	/* None of the fields are changeable */

		if (!changeable_values) {
			buf[2] = 0x04;	/* Write cache enable, */
					/* Read cache not disabled */
					/* No cache retention priorities */
			put_unaligned_be16(0xffff, &buf[4]);
					/* Don't disable prefetch */
					/* Minimum prefetch = 0 */
			put_unaligned_be16(0xffff, &buf[8]);
					/* Maximum prefetch */
			put_unaligned_be16(0xffff, &buf[10]);
					/* Maximum prefetch ceiling */
		}
		buf += 12;
	}

	/* MM Capabilities and Mechanical Status, for optical LUNs */
	if (curlun->cdrom && (page_code == 0x2a || all_pages)) {
		valid_page = 1;
		buf[0] = 0x2a;		/* Page code */
		buf[1] = 20;		/* Page length */
		memset(buf+2, 0, 20);	/* None of the fields are changeable */

		if (!changeable_values) {
			buf[2] = 0x08;	/* Reads DVD-ROM, as well as CD */
			buf[4] = 0x30;	/* Mode 2 form 1 and form 2 */
			buf[5] = 0x13;	/* CD-DA by READ CD, accurately, C2 */
			buf[6] = 0x29 | (curlun->prevent_medium_removal << 1);
					/* Tray, eject, lock and its state */
			put_unaligned_be16(2816, &buf[8]);	/* 16x, KB/s */
			put_unaligned_be16(2816, &buf[14]);
		}
		buf += 22;
	}

	/*
	 * Check that a valid page was requested and the mode data length
	 * isn't too long.
	 */
	len = buf - buf0;
	if (!valid_page || len > limit) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	/*  Store the mode data length */
	if (mscmnd == MODE_SENSE)
		buf0[0] = len - 1;
	else
		put_unaligned_be16(len - 2, buf0);
	return len;
}

static int do_start_stop(struct fsg_common *common)
{
	struct fsg_lun	*curlun = common->curlun;
	int		loej, start;

	if (!curlun) {
		return -EINVAL;
	} else if (!curlun->removable) {
		curlun->sense_data = SS_INVALID_COMMAND;
		return -EINVAL;
	} else if ((common->cmnd[1] & ~0x01) != 0 || /* Mask away Immed */
		   (common->cmnd[4] & ~0x03) != 0) { /* Mask LoEj, Start */
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	loej  = common->cmnd[4] & 0x02;
	start = common->cmnd[4] & 0x01;

	/*
	 * Our emulation doesn't support mounting; the medium is
	 * available for use as soon as it is loaded.
	 */
	if (start) {
		if (!fsg_lun_is_open(curlun)) {
			curlun->sense_data = SS_MEDIUM_NOT_PRESENT;
			return -EINVAL;
		}
		return 0;
	}

	/* Are we allowed to unload the media? */
	if (curlun->prevent_medium_removal) {
		LDBG(curlun, "unload attempt prevented\n");
		curlun->sense_data = SS_MEDIUM_REMOVAL_PREVENTED;
		return -EINVAL;
	}

	if (!loej)
		return 0;

	up_read(&common->filesem);
	down_write(&common->filesem);
	fsg_lun_close(curlun);
	up_write(&common->filesem);
	down_read(&common->filesem);

	return 0;
}

static int do_prevent_allow(struct fsg_common *common)
{
	struct fsg_lun	*curlun = common->curlun;
	int		prevent;

	if (!common->curlun) {
		return -EINVAL;
	} else if (!common->curlun->removable) {
		common->curlun->sense_data = SS_INVALID_COMMAND;
		return -EINVAL;
	}

	prevent = common->cmnd[4] & 0x01;
	if ((common->cmnd[4] & ~0x01) != 0) {	/* Mask away Prevent */
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	if (curlun->prevent_medium_removal && !prevent)
		fsg_lun_fsync_sub(curlun);
	curlun->prevent_medium_removal = prevent;
	return 0;
}

static int do_read_format_capacities(struct fsg_common *common,
			struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;
	u8		*buf = (u8 *) bh->buf;

	buf[0] = buf[1] = buf[2] = 0;
	buf[3] = 8;	/* Only the Current/Maximum Capacity Descriptor */
	buf += 4;

	put_unaligned_be32(curlun->num_sectors, &buf[0]);
						/* Number of blocks */
	put_unaligned_be32(curlun->blksize, &buf[4]);/* Block length */
	buf[4] = 0x02;				/* Current capacity */
	return 12;
}

static int do_mode_select(struct fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun	*curlun = common->curlun;

	/* We don't support MODE SELECT */
	if (curlun)
		curlun->sense_data = SS_INVALID_COMMAND;
	return -EINVAL;
}


/*-------------------------------------------------------------------------*/

static int halt_bulk_in_endpoint(struct fsg_dev *fsg)
{
	int	rc;

	rc = fsg_set_halt(fsg, fsg->bulk_in);
	if (rc == -EAGAIN)
		VDBG(fsg, "delayed bulk-in endpoint halt\n");
	while (rc != 0) {
		if (rc != -EAGAIN) {
			WARNING(fsg, "usb_ep_set_halt -> %d\n", rc);
			rc = 0;
			break;
		}

		/* Wait for a short time and then try again */
		if (msleep_interruptible(100) != 0)
			return -EINTR;
		rc = usb_ep_set_halt(fsg->bulk_in);
	}
	return rc;
}

static int wedge_bulk_in_endpoint(struct fsg_dev *fsg)
{
	int	rc;

	DBG(fsg, "bulk-in set wedge\n");
	rc = usb_ep_set_wedge(fsg->bulk_in);
	if (rc == -EAGAIN)
		VDBG(fsg, "delayed bulk-in endpoint wedge\n");
	while (rc != 0) {
		if (rc != -EAGAIN) {
			WARNING(fsg, "usb_ep_set_wedge -> %d\n", rc);
			rc = 0;
			break;
		}

		/* Wait for a short time and then try again */
		if (msleep_interruptible(100) != 0)
			return -EINTR;
		rc = usb_ep_set_wedge(fsg->bulk_in);
	}
	return rc;
}

static int throw_away_data(struct fsg_common *common)
{
	struct fsg_buffhd	*bh, *bh2;
	u32			amount;
	int			rc;

	for (bh = common->next_buffhd_to_drain;
	     bh->state != BUF_STATE_EMPTY || common->usb_amount_left > 0;
	     bh = common->next_buffhd_to_drain) {

		/* Try to submit another request if we need one */
		bh2 = common->next_buffhd_to_fill;
		if (bh2->state == BUF_STATE_EMPTY &&
				common->usb_amount_left > 0) {
			amount = min(common->usb_amount_left, FSG_BUFLEN);

			/*
			 * Except at the end of the transfer, amount will be
			 * equal to the buffer size, which is divisible by
			 * the bulk-out maxpacket size.
			 */
			set_bulk_out_req_length(common, bh2, amount);
			if (!start_out_transfer(common, bh2))
				/* Dunno what to do if common->fsg is NULL */
				return -EIO;
			common->next_buffhd_to_fill = bh2->next;
			common->usb_amount_left -= amount;
			continue;
		}

		/* Wait for the data to be received */
		rc = sleep_thread(common, false, bh);
		if (rc)
			return rc;

		/* Throw away the data in a filled buffer */
		bh->state = BUF_STATE_EMPTY;
		common->next_buffhd_to_drain = bh->next;

		/* A short packet or an error ends everything */
		if (bh->outreq->actual < bh->bulk_out_intended_length ||
				bh->outreq->status != 0) {
			raise_exception(common, FSG_STATE_ABORT_BULK_OUT);
			return -EINTR;
		}
	}
	return 0;
}

static int finish_reply(struct fsg_common *common)
{
	struct fsg_buffhd	*bh = common->next_buffhd_to_fill;
	int			rc = 0;

	switch (common->data_dir) {
	case DATA_DIR_NONE:
		break;			/* Nothing to send */

	/*
	 * If we don't know whether the host wants to read or write,
	 * this must be CB or CBI with an unknown command.  We mustn't
	 * try to send or receive any data.  So stall both bulk pipes
	 * if we can and wait for a reset.
	 */
	case DATA_DIR_UNKNOWN:
		if (!common->can_stall) {
			/* Nothing */
		} else if (fsg_is_set(common)) {
			fsg_set_halt(common->fsg, common->fsg->bulk_out);
			rc = halt_bulk_in_endpoint(common->fsg);
		} else {
			/* Don't know what to do if common->fsg is NULL */
			rc = -EIO;
		}
		break;

	/* All but the last buffer of data must have already been sent */
	case DATA_DIR_TO_HOST:
		if (common->data_size == 0) {
			/* Nothing to send */

		/* Don't know what to do if common->fsg is NULL */
		} else if (!fsg_is_set(common)) {
			rc = -EIO;

		/* If there's no residue, simply send the last buffer */
		} else if (common->residue == 0) {
			bh->inreq->zero = 0;
			if (!start_in_transfer(common, bh))
				return -EIO;
			common->next_buffhd_to_fill = bh->next;

		/*
		 * For Bulk-only, mark the end of the data with a short
		 * packet.  If we are allowed to stall, halt the bulk-in
		 * endpoint.  (Note: This violates the Bulk-Only Transport
		 * specification, which requires us to pad the data if we
		 * don't halt the endpoint.  Presumably nobody will mind.)
		 */
		} else {
			bh->inreq->zero = 1;
			if (!start_in_transfer(common, bh))
				rc = -EIO;
			common->next_buffhd_to_fill = bh->next;
			if (common->can_stall)
				rc = halt_bulk_in_endpoint(common->fsg);
		}
		break;

	/*
	 * We have processed all we want from the data the host has sent.
	 * There may still be outstanding bulk-out requests.
	 */
	case DATA_DIR_FROM_HOST:
		if (common->residue == 0) {
			/* Nothing to receive */

		/* Did the host stop sending unexpectedly early? */
		} else if (common->short_packet_received) {
			raise_exception(common, FSG_STATE_ABORT_BULK_OUT);
			rc = -EINTR;

		/*
		 * We haven't processed all the incoming data.  Even though
		 * we may be allowed to stall, doing so would cause a race.
		 * The controller may already have ACK'ed all the remaining
		 * bulk-out packets, in which case the host wouldn't see a
		 * STALL.  Not realizing the endpoint was halted, it wouldn't
		 * clear the halt -- leading to problems later on.
		 */
#if 0
		} else if (common->can_stall) {
			if (fsg_is_set(common))
				fsg_set_halt(common->fsg,
					     common->fsg->bulk_out);
			raise_exception(common, FSG_STATE_ABORT_BULK_OUT);
			rc = -EINTR;
#endif

		/*
		 * We can't stall.  Read in the excess data and throw it
		 * all away.
		 */
		} else {
			rc = throw_away_data(common);
		}
		break;
	}
	return rc;
}

static void send_status(struct fsg_common *common)
{
	struct fsg_lun		*curlun = common->curlun;
	struct fsg_buffhd	*bh;
	struct bulk_cs_wrap	*csw;
	int			rc;
	u8			status = US_BULK_STAT_OK;
	u32			sd, sdinfo = 0;

	/* Wait for the next buffer to become available */
	bh = common->next_buffhd_to_fill;
	rc = sleep_thread(common, false, bh);
	if (rc)
		return;

	if (curlun) {
		sd = curlun->sense_data;
		sdinfo = curlun->sense_data_info;
	} else if (common->bad_lun_okay)
		sd = SS_NO_SENSE;
	else
		sd = SS_LOGICAL_UNIT_NOT_SUPPORTED;

	if (common->phase_error) {
		DBG(common, "sending phase-error status\n");
		status = US_BULK_STAT_PHASE;
		sd = SS_INVALID_COMMAND;
	} else if (sd != SS_NO_SENSE) {
		DBG(common, "sending command-failure status\n");
		status = US_BULK_STAT_FAIL;
		VDBG(common, "  sense data: SK x%02x, ASC x%02x, ASCQ x%02x;"
				"  info x%x\n",
				SK(sd), ASC(sd), ASCQ(sd), sdinfo);
	}

	/* Store and send the Bulk-only CSW */
	csw = (void *)bh->buf;

	csw->Signature = cpu_to_le32(US_BULK_CS_SIGN);
	csw->Tag = common->tag;
	csw->Residue = cpu_to_le32(common->residue);
	csw->Status = status;

	bh->inreq->length = US_BULK_CS_WRAP_LEN;
	bh->inreq->zero = 0;
	if (!start_in_transfer(common, bh))
		/* Don't know what to do if common->fsg is NULL */
		return;

	common->next_buffhd_to_fill = bh->next;
	return;
}


/*-------------------------------------------------------------------------*/

/*
 * Check whether the command is properly formed and whether its data size
 * and direction agree with the values we already have.
 */
static int check_command(struct fsg_common *common, int cmnd_size,
			 enum data_direction data_dir, unsigned int mask,
			 int needs_medium, const char *name)
{
	int			i;
	unsigned int		lun = common->cmnd[1] >> 5;
	static const char	dirletter[4] = {'u', 'o', 'i', 'n'};
	char			hdlen[20];
	struct fsg_lun		*curlun;

	hdlen[0] = 0;
	if (common->data_dir != DATA_DIR_UNKNOWN)
		sprintf(hdlen, ", H%c=%u", dirletter[(int) common->data_dir],
			common->data_size);
	VDBG(common, "SCSI command: %s;  Dc=%d, D%c=%u;  Hc=%d%s\n",
	     name, cmnd_size, dirletter[(int) data_dir],
	     common->data_size_from_cmnd, common->cmnd_size, hdlen);

	/*
	 * We can't reply at all until we know the correct data direction
	 * and size.
	 */
	if (common->data_size_from_cmnd == 0)
		data_dir = DATA_DIR_NONE;
	if (common->data_size < common->data_size_from_cmnd) {
		/*
		 * Host data size < Device data size is a phase error.
		 * Carry out the command, but only transfer as much as
		 * we are allowed.
		 */
		common->data_size_from_cmnd = common->data_size;
		common->phase_error = 1;
	}
	common->residue = common->data_size;
	common->usb_amount_left = common->data_size;

	/* Conflicting data directions is a phase error */
	if (common->data_dir != data_dir && common->data_size_from_cmnd > 0) {
		common->phase_error = 1;
		return -EINVAL;
	}

	/* Verify the length of the command itself */
	if (cmnd_size != common->cmnd_size) {

		/*
		 * Special case workaround: There are plenty of buggy SCSI
		 * implementations. Many have issues with cbw->Length
		 * field passing a wrong command size. For those cases we
		 * always try to work around the problem by using the length
		 * sent by the host side provided it is at least as large
		 * as the correct command length.
		 * Examples of such cases would be MS-Windows, which issues
		 * REQUEST SENSE with cbw->Length == 12 where it should
		 * be 6, and xbox360 issuing INQUIRY, TEST UNIT READY and
		 * REQUEST SENSE with cbw->Length == 10 where it should
		 * be 6 as well.
		 */
		if (cmnd_size <= common->cmnd_size) {
			DBG(common, "%s is buggy! Expected length %d "
			    "but we got %d\n", name,
			    cmnd_size, common->cmnd_size);
			cmnd_size = common->cmnd_size;
		} else {
			common->phase_error = 1;
			return -EINVAL;
		}
	}

	/* Check that the LUN values are consistent */
	if (common->lun != lun)
		DBG(common, "using LUN %u from CBW, not LUN %u from CDB\n",
		    common->lun, lun);

	/* Check the LUN */
	curlun = common->curlun;
	if (curlun) {
		if (common->cmnd[0] != REQUEST_SENSE) {
			curlun->sense_data = SS_NO_SENSE;
			curlun->sense_data_info = 0;
			curlun->info_valid = 0;
		}
	} else {
		common->bad_lun_okay = 0;

		/*
		 * INQUIRY and REQUEST SENSE commands are explicitly allowed
		 * to use unsupported LUNs; all others may not.
		 */
		if (common->cmnd[0] != INQUIRY &&
		    common->cmnd[0] != REQUEST_SENSE) {
			DBG(common, "unsupported LUN %u\n", common->lun);
			return -EINVAL;
		}
	}

	/*
	 * If a unit attention condition exists, only INQUIRY and
	 * REQUEST SENSE commands are allowed; anything else must fail.
	 */
	if (curlun && curlun->unit_attention_data != SS_NO_SENSE &&
	    common->cmnd[0] != INQUIRY &&
	    common->cmnd[0] != REQUEST_SENSE) {
		curlun->sense_data = curlun->unit_attention_data;
		curlun->unit_attention_data = SS_NO_SENSE;
		return -EINVAL;
	}

	/* Check that only command bytes listed in the mask are non-zero */
	common->cmnd[1] &= 0x1f;			/* Mask away the LUN */
	for (i = 1; i < cmnd_size; ++i) {
		if (common->cmnd[i] && !(mask & (1 << i))) {
			if (curlun)
				curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
			return -EINVAL;
		}
	}

	/* If the medium isn't mounted and the command needs to access
	 * it, return an error. */
	if (curlun && !fsg_lun_is_open(curlun) && needs_medium) {
		curlun->sense_data = SS_MEDIUM_NOT_PRESENT;
		return -EINVAL;
	}

	return 0;
}

/* wrapper of check_command for data size in blocks handling */
static int check_command_size_in_blocks(struct fsg_common *common,
		int cmnd_size, enum data_direction data_dir,
		unsigned int mask, int needs_medium, const char *name)
{
	if (common->curlun) {
		if (check_shl_overflow(common->data_size_from_cmnd,
				       common->curlun->blkbits,
				       &common->data_size_from_cmnd)) {
			common->phase_error = 1;
			return -EINVAL;
		}
	}

	return check_command(common, cmnd_size, data_dir,
			mask, needs_medium, name);
}

static int do_scsi_command(struct fsg_common *common)
{
	struct fsg_buffhd	*bh;
	int			rc;
	int			reply = -EINVAL;
	int			i;
	static char		unknown[16];

	dump_cdb(common);

	/* Wait for the next buffer to become available for data or status */
	bh = common->next_buffhd_to_fill;
	common->next_buffhd_to_drain = bh;
	rc = sleep_thread(common, false, bh);
	if (rc)
		return rc;

	common->phase_error = 0;
	common->short_packet_received = 0;

	down_read(&common->filesem);	/* We're using the backing file */
	switch (common->cmnd[0]) {

	case INQUIRY:
		common->data_size_from_cmnd = common->cmnd[4];
		reply = check_command(common, 6, DATA_DIR_TO_HOST,
				      (1<<4), 0,
				      "INQUIRY");
		if (reply == 0)
			reply = do_inquiry(common, bh);
		break;

	case MODE_SELECT:
		common->data_size_from_cmnd = common->cmnd[4];
		reply = check_command(common, 6, DATA_DIR_FROM_HOST,
				      (1<<1) | (1<<4), 0,
				      "MODE SELECT(6)");
		if (reply == 0)
			reply = do_mode_select(common, bh);
		break;

	case MODE_SELECT_10:
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = check_command(common, 10, DATA_DIR_FROM_HOST,
				      (1<<1) | (3<<7), 0,
				      "MODE SELECT(10)");
		if (reply == 0)
			reply = do_mode_select(common, bh);
		break;

	case MODE_SENSE:
		common->data_size_from_cmnd = common->cmnd[4];
		reply = check_command(common, 6, DATA_DIR_TO_HOST,
				      (1<<1) | (1<<2) | (1<<4), 0,
				      "MODE SENSE(6)");
		if (reply == 0)
			reply = do_mode_sense(common, bh);
		break;

	case MODE_SENSE_10:
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = check_command(common, 10, DATA_DIR_TO_HOST,
				      (1<<1) | (1<<2) | (3<<7), 0,
				      "MODE SENSE(10)");
		if (reply == 0)
			reply = do_mode_sense(common, bh);
		break;

	case ALLOW_MEDIUM_REMOVAL:
		common->data_size_from_cmnd = 0;
		reply = check_command(common, 6, DATA_DIR_NONE,
				      (1<<4), 0,
				      "PREVENT-ALLOW MEDIUM REMOVAL");
		if (reply == 0)
			reply = do_prevent_allow(common);
		break;

	case READ_6:
		i = common->cmnd[4];
		common->data_size_from_cmnd = (i == 0) ? 256 : i;
		reply = check_command_size_in_blocks(common, 6,
				      DATA_DIR_TO_HOST,
				      (7<<1) | (1<<4), 1,
				      "READ(6)");
		if (reply == 0)
			reply = do_read(common);
		break;

	case READ_10:
		common->data_size_from_cmnd =
				get_unaligned_be16(&common->cmnd[7]);
		reply = check_command_size_in_blocks(common, 10,
				      DATA_DIR_TO_HOST,
				      (1<<1) | (0xf<<2) | (3<<7), 1,
				      "READ(10)");
		if (reply == 0)
			reply = do_read(common);
		break;

	case READ_12:
		common->data_size_from_cmnd =
				get_unaligned_be32(&common->cmnd[6]);
		reply = check_command_size_in_blocks(common, 12,
				      DATA_DIR_TO_HOST,
				      (1<<1) | (0xf<<2) | (0xf<<6), 1,
				      "READ(12)");
		if (reply == 0)
			reply = do_read(common);
		break;

	case READ_16:
		common->data_size_from_cmnd =
				get_unaligned_be32(&common->cmnd[10]);
		reply = check_command_size_in_blocks(common, 16,
				      DATA_DIR_TO_HOST,
				      (1<<1) | (0xff<<2) | (0xf<<10), 1,
				      "READ(16)");
		if (reply == 0)
			reply = do_read(common);
		break;

	case READ_CAPACITY:
		common->data_size_from_cmnd = 8;
		reply = check_command(common, 10, DATA_DIR_TO_HOST,
				      (0xf<<2) | (1<<8), 1,
				      "READ CAPACITY");
		if (reply == 0)
			reply = do_read_capacity(common, bh);
		break;

	case GPCMD_GET_CONFIGURATION:
		if (!common->curlun || !common->curlun->cdrom)
			goto unknown_cmnd;
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = check_command(common, 10, DATA_DIR_TO_HOST,
				      (1<<1) | (3<<2) | (3<<7), 0,
				      "GET CONFIGURATION");
		if (reply == 0)
			reply = do_get_configuration(common, bh);
		break;

	case GPCMD_READ_DVD_STRUCTURE:
		if (!common->curlun || !common->curlun->cdrom)
			goto unknown_cmnd;
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[8]);
		reply = check_command(common, 12, DATA_DIR_TO_HOST,
				      (1<<1) | (0xf<<2) | (1<<6) | (1<<7) |
				      (3<<8) | (1<<10), 1,
				      "READ DISC STRUCTURE");
		if (reply == 0)
			reply = do_read_disc_structure(common, bh);
		break;

	case GPCMD_READ_DISC_INFO:
		if (!common->curlun || !common->curlun->cdrom)
			goto unknown_cmnd;
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = check_command(common, 10, DATA_DIR_TO_HOST,
				      (1<<1) | (3<<7), 1,
				      "READ DISC INFORMATION");
		if (reply == 0)
			reply = do_read_disc_information(common, bh);
		break;

	case GPCMD_READ_TRACK_RZONE_INFO:
		if (!common->curlun || !common->curlun->cdrom)
			goto unknown_cmnd;
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = check_command(common, 10, DATA_DIR_TO_HOST,
				      (1<<1) | (0xf<<2) | (3<<7), 1,
				      "READ TRACK INFORMATION");
		if (reply == 0)
			reply = do_read_track_information(common, bh);
		break;

	case GPCMD_READ_CD:
		if (!common->curlun || !common->curlun->cdrom)
			goto unknown_cmnd;
		if (fsg_cd_has_tracks(common->curlun)) {
			u32 bad = 0;
			u32 sense = fsg_cd_read_cd_size(common, &bad);

			reply = check_command(common, 12, DATA_DIR_TO_HOST,
					      (1<<1) | (0xf<<2) | (7<<6) |
					      (1<<9) | (1<<10), 1, "READ CD");
			if (reply == 0 && sense) {
				fsg_cd_sense(common->curlun, sense, bad);
				reply = -EINVAL;
			} else if (reply == 0) {
				reply = fsg_cd_read_cd(common);
			}
			break;
		}
		common->data_size_from_cmnd =
			get_unaligned_be24(&common->cmnd[6]);
		reply = check_command_size_in_blocks(common, 12,
				      DATA_DIR_TO_HOST,
				      (1<<1) | (0xf<<2) | (7<<6) | (1<<9) |
				      (1<<10), 1,
				      "READ CD");
		if (reply == 0)
			reply = do_read_cd(common);
		break;

	case GPCMD_GET_EVENT_STATUS_NOTIFICATION:
		if (!common->curlun || !common->curlun->cdrom)
			goto unknown_cmnd;
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = check_command(common, 10, DATA_DIR_TO_HOST,
				      (1<<1) | (1<<4) | (3<<7), 0,
				      "GET EVENT STATUS NOTIFICATION");
		if (reply == 0)
			reply = do_get_event_status_notification(common, bh);
		break;

	case GPCMD_READ_SUBCHANNEL:
		if (!common->curlun || !common->curlun->cdrom)
			goto unknown_cmnd;
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = check_command(common, 10, DATA_DIR_TO_HOST,
				      (1<<1) | (1<<2) | (1<<3) | (1<<6) |
				      (3<<7), 1, "READ SUB-CHANNEL");
		if (reply == 0)
			reply = do_read_subchannel(common, bh);
		break;

	case READ_HEADER:
		if (!common->curlun || !common->curlun->cdrom)
			goto unknown_cmnd;
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = check_command(common, 10, DATA_DIR_TO_HOST,
				      (3<<7) | (0x1f<<1), 1,
				      "READ HEADER");
		if (reply == 0)
			reply = do_read_header(common, bh);
		break;

	case READ_TOC:
		if (!common->curlun || !common->curlun->cdrom)
			goto unknown_cmnd;
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = check_command(common, 10, DATA_DIR_TO_HOST,
				      (0xf<<6) | (3<<1), 1,
				      "READ TOC");
		if (reply == 0)
			reply = do_read_toc(common, bh);
		break;

	case READ_FORMAT_CAPACITIES:
		common->data_size_from_cmnd =
			get_unaligned_be16(&common->cmnd[7]);
		reply = check_command(common, 10, DATA_DIR_TO_HOST,
				      (3<<7), 1,
				      "READ FORMAT CAPACITIES");
		if (reply == 0)
			reply = do_read_format_capacities(common, bh);
		break;

	case REQUEST_SENSE:
		common->data_size_from_cmnd = common->cmnd[4];
		reply = check_command(common, 6, DATA_DIR_TO_HOST,
				      (1<<4), 0,
				      "REQUEST SENSE");
		if (reply == 0)
			reply = do_request_sense(common, bh);
		break;

	case SERVICE_ACTION_IN_16:
		switch (common->cmnd[1] & 0x1f) {

		case SAI_READ_CAPACITY_16:
			common->data_size_from_cmnd =
				get_unaligned_be32(&common->cmnd[10]);
			reply = check_command(common, 16, DATA_DIR_TO_HOST,
					      (1<<1) | (0xff<<2) | (0xf<<10) |
					      (1<<14), 1,
					      "READ CAPACITY(16)");
			if (reply == 0)
				reply = do_read_capacity_16(common, bh);
			break;

		default:
			goto unknown_cmnd;
		}
		break;

	case START_STOP:
		common->data_size_from_cmnd = 0;
		reply = check_command(common, 6, DATA_DIR_NONE,
				      (1<<1) | (1<<4), 0,
				      "START-STOP UNIT");
		if (reply == 0)
			reply = do_start_stop(common);
		break;

	case SYNCHRONIZE_CACHE:
		common->data_size_from_cmnd = 0;
		reply = check_command(common, 10, DATA_DIR_NONE,
				      (0xf<<2) | (3<<7), 1,
				      "SYNCHRONIZE CACHE");
		if (reply == 0)
			reply = do_synchronize_cache(common);
		break;

	case TEST_UNIT_READY:
		common->data_size_from_cmnd = 0;
		reply = check_command(common, 6, DATA_DIR_NONE,
				0, 1,
				"TEST UNIT READY");
		break;

	/*
	 * Although optional, this command is used by MS-Windows.  We
	 * support a minimal version: BytChk must be 0.
	 */
	case VERIFY:
		common->data_size_from_cmnd = 0;
		reply = check_command(common, 10, DATA_DIR_NONE,
				      (1<<1) | (0xf<<2) | (3<<7), 1,
				      "VERIFY");
		if (reply == 0)
			reply = do_verify(common);
		break;

	case WRITE_6:
		i = common->cmnd[4];
		common->data_size_from_cmnd = (i == 0) ? 256 : i;
		reply = check_command_size_in_blocks(common, 6,
				      DATA_DIR_FROM_HOST,
				      (7<<1) | (1<<4), 1,
				      "WRITE(6)");
		if (reply == 0)
			reply = do_write(common);
		break;

	case WRITE_10:
		common->data_size_from_cmnd =
				get_unaligned_be16(&common->cmnd[7]);
		reply = check_command_size_in_blocks(common, 10,
				      DATA_DIR_FROM_HOST,
				      (1<<1) | (0xf<<2) | (3<<7), 1,
				      "WRITE(10)");
		if (reply == 0)
			reply = do_write(common);
		break;

	case WRITE_12:
		common->data_size_from_cmnd =
				get_unaligned_be32(&common->cmnd[6]);
		reply = check_command_size_in_blocks(common, 12,
				      DATA_DIR_FROM_HOST,
				      (1<<1) | (0xf<<2) | (0xf<<6), 1,
				      "WRITE(12)");
		if (reply == 0)
			reply = do_write(common);
		break;

	case WRITE_16:
		common->data_size_from_cmnd =
				get_unaligned_be32(&common->cmnd[10]);
		reply = check_command_size_in_blocks(common, 16,
				      DATA_DIR_FROM_HOST,
				      (1<<1) | (0xff<<2) | (0xf<<10), 1,
				      "WRITE(16)");
		if (reply == 0)
			reply = do_write(common);
		break;

	/*
	 * Some mandatory commands that we recognize but don't implement.
	 * They don't mean much in this setting.  It's left as an exercise
	 * for anyone interested to implement RESERVE and RELEASE in terms
	 * of Posix locks.
	 */
	case FORMAT_UNIT:
	case RELEASE_6:
	case RESERVE_6:
	case SEND_DIAGNOSTIC:

	default:
unknown_cmnd:
		common->data_size_from_cmnd = 0;
		sprintf(unknown, "Unknown x%02x", common->cmnd[0]);
		reply = check_command(common, common->cmnd_size,
				      DATA_DIR_UNKNOWN, ~0, 0, unknown);
		if (reply == 0) {
			common->curlun->sense_data = SS_INVALID_COMMAND;
			reply = -EINVAL;
		}
		break;
	}
	up_read(&common->filesem);

	if (reply == -EINTR || signal_pending(current))
		return -EINTR;

	/* Set up the single reply buffer for finish_reply() */
	if (reply == -EINVAL)
		reply = 0;		/* Error reply length */
	if (reply >= 0 && common->data_dir == DATA_DIR_TO_HOST) {
		reply = min_t(u32, reply, common->data_size_from_cmnd);
		bh->inreq->length = reply;
		bh->state = BUF_STATE_FULL;
		common->residue -= reply;
	}				/* Otherwise it's already set */

	return 0;
}


/*-------------------------------------------------------------------------*/

static int received_cbw(struct fsg_dev *fsg, struct fsg_buffhd *bh)
{
	struct usb_request	*req = bh->outreq;
	struct bulk_cb_wrap	*cbw = req->buf;
	struct fsg_common	*common = fsg->common;

	/* Was this a real packet?  Should it be ignored? */
	if (req->status || test_bit(IGNORE_BULK_OUT, &fsg->atomic_bitflags))
		return -EINVAL;

	/* Is the CBW valid? */
	if (req->actual != US_BULK_CB_WRAP_LEN ||
			cbw->Signature != cpu_to_le32(
				US_BULK_CB_SIGN)) {
		DBG(fsg, "invalid CBW: len %u sig 0x%x\n",
				req->actual,
				le32_to_cpu(cbw->Signature));

		/*
		 * The Bulk-only spec says we MUST stall the IN endpoint
		 * (6.6.1), so it's unavoidable.  It also says we must
		 * retain this state until the next reset, but there's
		 * no way to tell the controller driver it should ignore
		 * Clear-Feature(HALT) requests.
		 *
		 * We aren't required to halt the OUT endpoint; instead
		 * we can simply accept and discard any data received
		 * until the next reset.
		 */
		wedge_bulk_in_endpoint(fsg);
		set_bit(IGNORE_BULK_OUT, &fsg->atomic_bitflags);
		return -EINVAL;
	}

	/* Is the CBW meaningful? */
	if (cbw->Lun >= ARRAY_SIZE(common->luns) ||
	    cbw->Flags & ~US_BULK_FLAG_IN || cbw->Length <= 0 ||
	    cbw->Length > MAX_COMMAND_SIZE) {
		DBG(fsg, "non-meaningful CBW: lun = %u, flags = 0x%x, "
				"cmdlen %u\n",
				cbw->Lun, cbw->Flags, cbw->Length);

		/*
		 * We can do anything we want here, so let's stall the
		 * bulk pipes if we are allowed to.
		 */
		if (common->can_stall) {
			fsg_set_halt(fsg, fsg->bulk_out);
			halt_bulk_in_endpoint(fsg);
		}
		return -EINVAL;
	}

	/* Save the command for later */
	common->cmnd_size = cbw->Length;
	memcpy(common->cmnd, cbw->CDB, common->cmnd_size);
	if (cbw->Flags & US_BULK_FLAG_IN)
		common->data_dir = DATA_DIR_TO_HOST;
	else
		common->data_dir = DATA_DIR_FROM_HOST;
	common->data_size = le32_to_cpu(cbw->DataTransferLength);
	if (common->data_size == 0)
		common->data_dir = DATA_DIR_NONE;
	common->lun = cbw->Lun;
	if (common->lun < ARRAY_SIZE(common->luns))
		common->curlun = common->luns[common->lun];
	else
		common->curlun = NULL;
	common->tag = cbw->Tag;
	return 0;
}

static int get_next_command(struct fsg_common *common)
{
	struct fsg_buffhd	*bh;
	int			rc = 0;

	/* Wait for the next buffer to become available */
	bh = common->next_buffhd_to_fill;
	rc = sleep_thread(common, true, bh);
	if (rc)
		return rc;

	/* Queue a request to read a Bulk-only CBW */
	set_bulk_out_req_length(common, bh, US_BULK_CB_WRAP_LEN);
	if (!start_out_transfer(common, bh))
		/* Don't know what to do if common->fsg is NULL */
		return -EIO;

	/*
	 * We will drain the buffer in software, which means we
	 * can reuse it for the next filling.  No need to advance
	 * next_buffhd_to_fill.
	 */

	/* Wait for the CBW to arrive */
	rc = sleep_thread(common, true, bh);
	if (rc)
		return rc;

	rc = fsg_is_set(common) ? received_cbw(common->fsg, bh) : -EIO;
	bh->state = BUF_STATE_EMPTY;

	return rc;
}


/*-------------------------------------------------------------------------*/

static int alloc_request(struct fsg_common *common, struct usb_ep *ep,
		struct usb_request **preq)
{
	*preq = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (*preq)
		return 0;
	ERROR(common, "can't allocate request for %s\n", ep->name);
	return -ENOMEM;
}

/* Reset interface setting and re-init endpoint state (toggle etc). */
static int do_set_interface(struct fsg_common *common, struct fsg_dev *new_fsg)
{
	struct fsg_dev *fsg;
	int i, rc = 0;

	if (common->running)
		DBG(common, "reset interface\n");

reset:
	/* Deallocate the requests */
	if (common->fsg) {
		fsg = common->fsg;

		for (i = 0; i < common->fsg_num_buffers; ++i) {
			struct fsg_buffhd *bh = &common->buffhds[i];

			if (bh->inreq) {
				usb_ep_free_request(fsg->bulk_in, bh->inreq);
				bh->inreq = NULL;
			}
			if (bh->outreq) {
				usb_ep_free_request(fsg->bulk_out, bh->outreq);
				bh->outreq = NULL;
			}
		}

		/* Disable the endpoints */
		if (fsg->bulk_in_enabled) {
			usb_ep_disable(fsg->bulk_in);
			fsg->bulk_in_enabled = 0;
		}
		if (fsg->bulk_out_enabled) {
			usb_ep_disable(fsg->bulk_out);
			fsg->bulk_out_enabled = 0;
		}

		common->fsg = NULL;
		wake_up(&common->fsg_wait);
	}

	common->running = 0;
	if (!new_fsg || rc)
		return rc;

	common->fsg = new_fsg;
	fsg = common->fsg;

	/* Enable the endpoints */
	rc = config_ep_by_speed(common->gadget, &(fsg->function), fsg->bulk_in);
	if (rc)
		goto reset;
	rc = usb_ep_enable(fsg->bulk_in);
	if (rc)
		goto reset;
	fsg->bulk_in->driver_data = common;
	fsg->bulk_in_enabled = 1;

	rc = config_ep_by_speed(common->gadget, &(fsg->function),
				fsg->bulk_out);
	if (rc)
		goto reset;
	rc = usb_ep_enable(fsg->bulk_out);
	if (rc)
		goto reset;
	fsg->bulk_out->driver_data = common;
	fsg->bulk_out_enabled = 1;
	common->bulk_out_maxpacket = usb_endpoint_maxp(fsg->bulk_out->desc);
	clear_bit(IGNORE_BULK_OUT, &fsg->atomic_bitflags);

	/* Allocate the requests */
	for (i = 0; i < common->fsg_num_buffers; ++i) {
		struct fsg_buffhd	*bh = &common->buffhds[i];

		rc = alloc_request(common, fsg->bulk_in, &bh->inreq);
		if (rc)
			goto reset;
		rc = alloc_request(common, fsg->bulk_out, &bh->outreq);
		if (rc)
			goto reset;
		bh->inreq->buf = bh->outreq->buf = bh->buf;
		bh->inreq->context = bh->outreq->context = bh;
		bh->inreq->complete = bulk_in_complete;
		bh->outreq->complete = bulk_out_complete;
	}

	common->running = 1;
	for (i = 0; i < ARRAY_SIZE(common->luns); ++i)
		if (common->luns[i])
			common->luns[i]->unit_attention_data =
				SS_RESET_OCCURRED;
	return rc;
}


/****************************** ALT CONFIGS ******************************/

static int fsg_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct fsg_dev *fsg = fsg_from_func(f);

	__raise_exception(fsg->common, FSG_STATE_CONFIG_CHANGE, fsg);
	return USB_GADGET_DELAYED_STATUS;
}

static void fsg_disable(struct usb_function *f)
{
	struct fsg_dev *fsg = fsg_from_func(f);

	/* Disable the endpoints */
	if (fsg->bulk_in_enabled) {
		usb_ep_disable(fsg->bulk_in);
		fsg->bulk_in_enabled = 0;
	}
	if (fsg->bulk_out_enabled) {
		usb_ep_disable(fsg->bulk_out);
		fsg->bulk_out_enabled = 0;
	}

	__raise_exception(fsg->common, FSG_STATE_CONFIG_CHANGE, NULL);
}


/*-------------------------------------------------------------------------*/

static void handle_exception(struct fsg_common *common)
{
	int			i;
	struct fsg_buffhd	*bh;
	enum fsg_state		old_state;
	struct fsg_lun		*curlun;
	unsigned int		exception_req_tag;
	struct fsg_dev		*new_fsg;

	/*
	 * Clear the existing signals.  Anything but SIGUSR1 is converted
	 * into a high-priority EXIT exception.
	 */
	for (;;) {
		int sig = kernel_dequeue_signal();
		if (!sig)
			break;
		if (sig != SIGUSR1) {
			spin_lock_irq(&common->lock);
			if (common->state < FSG_STATE_EXIT)
				DBG(common, "Main thread exiting on signal\n");
			common->state = FSG_STATE_EXIT;
			spin_unlock_irq(&common->lock);
		}
	}

	/* Cancel all the pending transfers */
	if (likely(common->fsg)) {
		for (i = 0; i < common->fsg_num_buffers; ++i) {
			bh = &common->buffhds[i];
			if (bh->state == BUF_STATE_SENDING)
				usb_ep_dequeue(common->fsg->bulk_in, bh->inreq);
			if (bh->state == BUF_STATE_RECEIVING)
				usb_ep_dequeue(common->fsg->bulk_out,
					       bh->outreq);

			/* Wait for a transfer to become idle */
			if (sleep_thread(common, false, bh))
				return;
		}

		/* Clear out the controller's fifos */
		if (common->fsg->bulk_in_enabled)
			usb_ep_fifo_flush(common->fsg->bulk_in);
		if (common->fsg->bulk_out_enabled)
			usb_ep_fifo_flush(common->fsg->bulk_out);
	}

	/*
	 * Reset the I/O buffer states and pointers, the SCSI
	 * state, and the exception.  Then invoke the handler.
	 */
	spin_lock_irq(&common->lock);

	for (i = 0; i < common->fsg_num_buffers; ++i) {
		bh = &common->buffhds[i];
		bh->state = BUF_STATE_EMPTY;
	}
	common->next_buffhd_to_fill = &common->buffhds[0];
	common->next_buffhd_to_drain = &common->buffhds[0];
	exception_req_tag = common->exception_req_tag;
	new_fsg = common->exception_arg;
	old_state = common->state;
	common->state = FSG_STATE_NORMAL;

	if (old_state != FSG_STATE_ABORT_BULK_OUT) {
		for (i = 0; i < ARRAY_SIZE(common->luns); ++i) {
			curlun = common->luns[i];
			if (!curlun)
				continue;
			curlun->prevent_medium_removal = 0;
			curlun->sense_data = SS_NO_SENSE;
			curlun->unit_attention_data = SS_NO_SENSE;
			curlun->sense_data_info = 0;
			curlun->info_valid = 0;
		}
	}
	spin_unlock_irq(&common->lock);

	/* Carry out any extra actions required for the exception */
	switch (old_state) {
	case FSG_STATE_NORMAL:
		break;

	case FSG_STATE_ABORT_BULK_OUT:
		send_status(common);
		break;

	case FSG_STATE_PROTOCOL_RESET:
		/*
		 * In case we were forced against our will to halt a
		 * bulk endpoint, clear the halt now.  (The SuperH UDC
		 * requires this.)
		 */
		if (!fsg_is_set(common))
			break;
		if (test_and_clear_bit(IGNORE_BULK_OUT,
				       &common->fsg->atomic_bitflags))
			usb_ep_clear_halt(common->fsg->bulk_in);

		if (common->ep0_req_tag == exception_req_tag)
			ep0_queue(common);	/* Complete the status stage */

		/*
		 * Technically this should go here, but it would only be
		 * a waste of time.  Ditto for the INTERFACE_CHANGE and
		 * CONFIG_CHANGE cases.
		 */
		/* for (i = 0; i < common->ARRAY_SIZE(common->luns); ++i) */
		/*	if (common->luns[i]) */
		/*		common->luns[i]->unit_attention_data = */
		/*			SS_RESET_OCCURRED;  */
		break;

	case FSG_STATE_CONFIG_CHANGE:
		do_set_interface(common, new_fsg);
		if (new_fsg)
			usb_composite_setup_continue(common->cdev);
		break;

	case FSG_STATE_EXIT:
		do_set_interface(common, NULL);		/* Free resources */
		spin_lock_irq(&common->lock);
		common->state = FSG_STATE_TERMINATED;	/* Stop the thread */
		spin_unlock_irq(&common->lock);
		break;

	case FSG_STATE_TERMINATED:
		break;
	}
}


/*-------------------------------------------------------------------------*/

static int fsg_main_thread(void *common_)
{
	struct fsg_common	*common = common_;
	int			i;

	/*
	 * Allow the thread to be killed by a signal, but set the signal mask
	 * to block everything but INT, TERM, KILL, and USR1.
	 */
	allow_signal(SIGINT);
	allow_signal(SIGTERM);
	allow_signal(SIGKILL);
	allow_signal(SIGUSR1);

	/* Allow the thread to be frozen */
	set_freezable();

	/* The main loop */
	while (common->state != FSG_STATE_TERMINATED) {
		if (exception_in_progress(common) || signal_pending(current)) {
			handle_exception(common);
			continue;
		}

		if (!common->running) {
			sleep_thread(common, true, NULL);
			continue;
		}

		if (get_next_command(common) || exception_in_progress(common))
			continue;
		if (do_scsi_command(common) || exception_in_progress(common))
			continue;
		if (finish_reply(common) || exception_in_progress(common))
			continue;
		send_status(common);
	}

	spin_lock_irq(&common->lock);
	common->thread_task = NULL;
	spin_unlock_irq(&common->lock);

	/* Eject media from all LUNs */

	down_write(&common->filesem);
	for (i = 0; i < ARRAY_SIZE(common->luns); i++) {
		struct fsg_lun *curlun = common->luns[i];

		if (curlun && fsg_lun_is_open(curlun))
			fsg_lun_close(curlun);
	}
	up_write(&common->filesem);

	/* Let fsg_unbind() know the thread has exited */
	kthread_complete_and_exit(&common->thread_notifier, 0);
}


/*************************** DEVICE ATTRIBUTES ***************************/

static ssize_t ro_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsg_lun		*curlun = fsg_lun_from_dev(dev);

	return fsg_show_ro(curlun, buf);
}

static ssize_t nofua_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct fsg_lun		*curlun = fsg_lun_from_dev(dev);

	return fsg_show_nofua(curlun, buf);
}

static ssize_t file_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct fsg_lun		*curlun = fsg_lun_from_dev(dev);
	struct rw_semaphore	*filesem = dev_get_drvdata(dev);

	return fsg_show_file(curlun, filesem, buf);
}

static ssize_t ro_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct fsg_lun		*curlun = fsg_lun_from_dev(dev);
	struct rw_semaphore	*filesem = dev_get_drvdata(dev);

	return fsg_store_ro(curlun, filesem, buf, count);
}

static ssize_t nofua_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fsg_lun		*curlun = fsg_lun_from_dev(dev);

	return fsg_store_nofua(curlun, buf, count);
}

static ssize_t file_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct fsg_lun		*curlun = fsg_lun_from_dev(dev);
	struct rw_semaphore	*filesem = dev_get_drvdata(dev);

	return fsg_store_file(curlun, filesem, buf, count);
}

static ssize_t forced_eject_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct fsg_lun		*curlun = fsg_lun_from_dev(dev);
	struct rw_semaphore	*filesem = dev_get_drvdata(dev);

	return fsg_store_forced_eject(curlun, filesem, buf, count);
}

static DEVICE_ATTR_RW(nofua);
static DEVICE_ATTR_WO(forced_eject);

/*
 * Mode of the ro and file attribute files will be overridden in
 * fsg_lun_dev_is_visible() depending on if this is a cdrom, or if it is a
 * removable device.
 */
static DEVICE_ATTR_RW(ro);
static DEVICE_ATTR_RW(file);

/****************************** FSG COMMON ******************************/

static void fsg_lun_release(struct device *dev)
{
	/* Nothing needs to be done */
}

static struct fsg_common *fsg_common_setup(struct fsg_common *common)
{
	if (!common) {
		common = kzalloc_obj(*common);
		if (!common)
			return ERR_PTR(-ENOMEM);
		common->free_storage_on_release = 1;
	} else {
		common->free_storage_on_release = 0;
	}
	init_rwsem(&common->filesem);
	spin_lock_init(&common->lock);
	init_completion(&common->thread_notifier);
	init_waitqueue_head(&common->io_wait);
	init_waitqueue_head(&common->fsg_wait);
	common->state = FSG_STATE_TERMINATED;
	memset(common->luns, 0, sizeof(common->luns));

	return common;
}

void fsg_common_set_sysfs(struct fsg_common *common, bool sysfs)
{
	common->sysfs = sysfs;
}
EXPORT_SYMBOL_GPL(fsg_common_set_sysfs);

static void _fsg_common_free_buffers(struct fsg_buffhd *buffhds, unsigned n)
{
	if (buffhds) {
		struct fsg_buffhd *bh = buffhds;
		while (n--) {
			kfree(bh->buf);
			++bh;
		}
		kfree(buffhds);
	}
}

int fsg_common_set_num_buffers(struct fsg_common *common, unsigned int n)
{
	struct fsg_buffhd *bh, *buffhds;
	int i;

	if (n < 2)
		return -EINVAL;

	buffhds = kzalloc_objs(*buffhds, n);
	if (!buffhds)
		return -ENOMEM;

	/* Data buffers cyclic list */
	bh = buffhds;
	i = n;
	goto buffhds_first_it;
	do {
		bh->next = bh + 1;
		++bh;
buffhds_first_it:
		bh->buf = kmalloc(FSG_BUFLEN, GFP_KERNEL);
		if (unlikely(!bh->buf))
			goto error_release;
	} while (--i);
	bh->next = buffhds;

	_fsg_common_free_buffers(common->buffhds, common->fsg_num_buffers);
	common->fsg_num_buffers = n;
	common->buffhds = buffhds;

	return 0;

error_release:
	/*
	 * "buf"s pointed to by heads after n - i are NULL
	 * so releasing them won't hurt
	 */
	_fsg_common_free_buffers(buffhds, n);

	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(fsg_common_set_num_buffers);

void fsg_common_remove_lun(struct fsg_lun *lun)
{
	if (device_is_registered(&lun->dev))
		device_unregister(&lun->dev);
	fsg_lun_close(lun);
	kfree(lun);
}
EXPORT_SYMBOL_GPL(fsg_common_remove_lun);

static void _fsg_common_remove_luns(struct fsg_common *common, int n)
{
	int i;

	for (i = 0; i < n; ++i)
		if (common->luns[i]) {
			fsg_common_remove_lun(common->luns[i]);
			common->luns[i] = NULL;
		}
}

void fsg_common_remove_luns(struct fsg_common *common)
{
	_fsg_common_remove_luns(common, ARRAY_SIZE(common->luns));
}
EXPORT_SYMBOL_GPL(fsg_common_remove_luns);

void fsg_common_free_buffers(struct fsg_common *common)
{
	_fsg_common_free_buffers(common->buffhds, common->fsg_num_buffers);
	common->buffhds = NULL;
}
EXPORT_SYMBOL_GPL(fsg_common_free_buffers);

int fsg_common_set_cdev(struct fsg_common *common,
			 struct usb_composite_dev *cdev, bool can_stall)
{
	struct usb_string *us;

	common->gadget = cdev->gadget;
	common->ep0 = cdev->gadget->ep0;
	common->ep0req = cdev->req;
	common->cdev = cdev;

	us = usb_gstrings_attach(cdev, fsg_strings_array,
				 ARRAY_SIZE(fsg_strings));
	if (IS_ERR(us))
		return PTR_ERR(us);

	fsg_intf_desc.iInterface = us[FSG_STRING_INTERFACE].id;

	/*
	 * Some peripheral controllers are known not to be able to
	 * halt bulk endpoints correctly.  If one of them is present,
	 * disable stalls.
	 */
	common->can_stall = can_stall &&
			gadget_is_stall_supported(common->gadget);

	return 0;
}
EXPORT_SYMBOL_GPL(fsg_common_set_cdev);

static struct attribute *fsg_lun_dev_attrs[] = {
	&dev_attr_ro.attr,
	&dev_attr_file.attr,
	&dev_attr_nofua.attr,
	&dev_attr_forced_eject.attr,
	NULL
};

static umode_t fsg_lun_dev_is_visible(struct kobject *kobj,
				      struct attribute *attr, int idx)
{
	struct device *dev = kobj_to_dev(kobj);
	struct fsg_lun *lun = fsg_lun_from_dev(dev);

	if (attr == &dev_attr_ro.attr)
		return lun->cdrom ? S_IRUGO : (S_IWUSR | S_IRUGO);
	if (attr == &dev_attr_file.attr)
		return lun->removable ? (S_IWUSR | S_IRUGO) : S_IRUGO;
	return attr->mode;
}

static const struct attribute_group fsg_lun_dev_group = {
	.attrs = fsg_lun_dev_attrs,
	.is_visible = fsg_lun_dev_is_visible,
};

static const struct attribute_group *fsg_lun_dev_groups[] = {
	&fsg_lun_dev_group,
	NULL
};

int fsg_common_create_lun(struct fsg_common *common, struct fsg_lun_config *cfg,
			  unsigned int id, const char *name,
			  const char **name_pfx)
{
	struct fsg_lun *lun;
	char *pathbuf = NULL, *p = "(no medium)";
	int rc = -ENOMEM;

	if (id >= ARRAY_SIZE(common->luns))
		return -ENODEV;

	if (common->luns[id])
		return -EBUSY;

	if (!cfg->filename && !cfg->removable) {
		pr_err("no file given for LUN%d\n", id);
		return -EINVAL;
	}

	lun = kzalloc_obj(*lun);
	if (!lun)
		return -ENOMEM;

	lun->name_pfx = name_pfx;

	lun->cdrom = !!cfg->cdrom;
	lun->ro = cfg->cdrom || cfg->ro;
	lun->initially_ro = lun->ro;
	lun->removable = !!cfg->removable;

	if (!common->sysfs) {
		/* we DON'T own the name!*/
		lun->name = name;
	} else {
		lun->dev.release = fsg_lun_release;
		lun->dev.parent = &common->gadget->dev;
		lun->dev.groups = fsg_lun_dev_groups;
		dev_set_drvdata(&lun->dev, &common->filesem);
		dev_set_name(&lun->dev, "%s", name);
		lun->name = dev_name(&lun->dev);

		rc = device_register(&lun->dev);
		if (rc) {
			pr_info("failed to register LUN%d: %d\n", id, rc);
			put_device(&lun->dev);
			goto error_sysfs;
		}
	}

	common->luns[id] = lun;

	if (cfg->filename) {
		rc = fsg_lun_open(lun, cfg->filename);
		if (rc)
			goto error_lun;

		p = "(error)";
		pathbuf = kmalloc(PATH_MAX, GFP_KERNEL);
		if (pathbuf) {
			p = file_path(lun->filp, pathbuf, PATH_MAX);
			if (IS_ERR(p))
				p = "(error)";
		}
	}
	pr_info("LUN: %s%s%sfile: %s\n",
	      lun->removable ? "removable " : "",
	      lun->ro ? "read only " : "",
	      lun->cdrom ? "CD-ROM " : "",
	      p);
	kfree(pathbuf);

	return 0;

error_lun:
	if (device_is_registered(&lun->dev))
		device_unregister(&lun->dev);
	common->luns[id] = NULL;
error_sysfs:
	kfree(lun);
	return rc;
}
EXPORT_SYMBOL_GPL(fsg_common_create_lun);

int fsg_common_create_luns(struct fsg_common *common, struct fsg_config *cfg)
{
	char buf[14];
	int i, rc;

	fsg_common_remove_luns(common);

	for (i = 0; i < cfg->nluns; ++i) {
		snprintf(buf, sizeof(buf), "lun%d", i);
		rc = fsg_common_create_lun(common, &cfg->luns[i], i, buf, NULL);
		if (rc)
			goto fail;
	}

	pr_info("Number of LUNs=%d\n", cfg->nluns);

	return 0;

fail:
	_fsg_common_remove_luns(common, i);
	return rc;
}
EXPORT_SYMBOL_GPL(fsg_common_create_luns);

void fsg_common_set_inquiry_string(struct fsg_common *common, const char *vn,
				   const char *pn)
{
	int i;

	/* Prepare inquiryString */
	i = get_default_bcdDevice();
	snprintf(common->inquiry_string, sizeof(common->inquiry_string),
		 "%-8s%-16s%04x", vn ?: "Linux",
		 /* Assume product name dependent on the first LUN */
		 pn ?: ((*common->luns)->cdrom
		     ? "File-CD Gadget"
		     : "File-Stor Gadget"),
		 i);
}
EXPORT_SYMBOL_GPL(fsg_common_set_inquiry_string);

static void fsg_common_release(struct fsg_common *common)
{
	int i;

	/* If the thread isn't already dead, tell it to exit now */
	if (common->state != FSG_STATE_TERMINATED) {
		raise_exception(common, FSG_STATE_EXIT);
		wait_for_completion(&common->thread_notifier);
	}

	for (i = 0; i < ARRAY_SIZE(common->luns); ++i) {
		struct fsg_lun *lun = common->luns[i];
		if (!lun)
			continue;
		fsg_lun_close(lun);
		if (device_is_registered(&lun->dev))
			device_unregister(&lun->dev);
		kfree(lun);
	}

	_fsg_common_free_buffers(common->buffhds, common->fsg_num_buffers);
	if (common->free_storage_on_release)
		kfree(common);
}


/*-------------------------------------------------------------------------*/

static int fsg_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct fsg_dev		*fsg = fsg_from_func(f);
	struct fsg_common	*common = fsg->common;
	struct usb_gadget	*gadget = c->cdev->gadget;
	int			i;
	struct usb_ep		*ep;
	unsigned		max_burst;
	int			ret;
	struct fsg_opts		*opts;

	/* Don't allow to bind if we don't have at least one LUN */
	ret = _fsg_common_get_max_lun(common);
	if (ret < 0) {
		pr_err("There should be at least one LUN.\n");
		return -EINVAL;
	}

	opts = fsg_opts_from_func_inst(f->fi);
	if (!opts->no_configfs) {
		ret = fsg_common_set_cdev(fsg->common, c->cdev,
					  fsg->common->can_stall);
		if (ret)
			return ret;
		fsg_common_set_inquiry_string(fsg->common, NULL, NULL);
	}

	if (!common->thread_task) {
		common->state = FSG_STATE_NORMAL;
		common->thread_task =
			kthread_run(fsg_main_thread, common, "file-storage");
		if (IS_ERR(common->thread_task)) {
			ret = PTR_ERR(common->thread_task);
			common->thread_task = NULL;
			common->state = FSG_STATE_TERMINATED;
			return ret;
		}
		DBG(common, "I/O thread pid: %d\n",
		    task_pid_nr(common->thread_task));
	}

	fsg->gadget = gadget;

	/* New interface */
	i = usb_interface_id(c, f);
	if (i < 0)
		goto fail;
	fsg_intf_desc.bInterfaceNumber = i;
	fsg->interface_number = i;

	/* Find all the endpoints we will use */
	ep = usb_ep_autoconfig(gadget, &fsg_fs_bulk_in_desc);
	if (!ep)
		goto autoconf_fail;
	fsg->bulk_in = ep;

	ep = usb_ep_autoconfig(gadget, &fsg_fs_bulk_out_desc);
	if (!ep)
		goto autoconf_fail;
	fsg->bulk_out = ep;

	/* Assume endpoint addresses are the same for both speeds */
	fsg_hs_bulk_in_desc.bEndpointAddress =
		fsg_fs_bulk_in_desc.bEndpointAddress;
	fsg_hs_bulk_out_desc.bEndpointAddress =
		fsg_fs_bulk_out_desc.bEndpointAddress;

	/* Calculate bMaxBurst, we know packet size is 1024 */
	max_burst = min_t(unsigned, FSG_BUFLEN / 1024, 15);

	fsg_ss_bulk_in_desc.bEndpointAddress =
		fsg_fs_bulk_in_desc.bEndpointAddress;
	fsg_ss_bulk_in_comp_desc.bMaxBurst = max_burst;

	fsg_ss_bulk_out_desc.bEndpointAddress =
		fsg_fs_bulk_out_desc.bEndpointAddress;
	fsg_ss_bulk_out_comp_desc.bMaxBurst = max_burst;

	ret = usb_assign_descriptors(f, fsg_fs_function, fsg_hs_function,
			fsg_ss_function, fsg_ss_function);
	if (ret)
		goto autoconf_fail;

	return 0;

autoconf_fail:
	ERROR(fsg, "unable to autoconfigure all endpoints\n");
	i = -ENOTSUPP;
fail:
	/* terminate the thread */
	if (fsg->common->state != FSG_STATE_TERMINATED) {
		raise_exception(fsg->common, FSG_STATE_EXIT);
		wait_for_completion(&fsg->common->thread_notifier);
	}
	return i;
}

/****************************** ALLOCATE FUNCTION *************************/

static void fsg_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct fsg_dev		*fsg = fsg_from_func(f);
	struct fsg_common	*common = fsg->common;

	DBG(fsg, "unbind\n");
	if (fsg->common->fsg == fsg) {
		__raise_exception(fsg->common, FSG_STATE_CONFIG_CHANGE, NULL);
		/* FIXME: make interruptible or killable somehow? */
		wait_event(common->fsg_wait, common->fsg != fsg);
	}

	usb_free_all_descriptors(&fsg->function);
}

static inline struct fsg_lun_opts *to_fsg_lun_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct fsg_lun_opts, group);
}

static inline struct fsg_opts *to_fsg_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct fsg_opts,
			    func_inst.group);
}

static void fsg_lun_attr_release(struct config_item *item)
{
	struct fsg_lun_opts *lun_opts;

	lun_opts = to_fsg_lun_opts(item);
	kfree(lun_opts);
}

static const struct configfs_item_operations fsg_lun_item_ops = {
	.release		= fsg_lun_attr_release,
};

static ssize_t fsg_lun_opts_file_show(struct config_item *item, char *page)
{
	struct fsg_lun_opts *opts = to_fsg_lun_opts(item);
	struct fsg_opts *fsg_opts = to_fsg_opts(opts->group.cg_item.ci_parent);

	return fsg_show_file(opts->lun, &fsg_opts->common->filesem, page);
}

static ssize_t fsg_lun_opts_file_store(struct config_item *item,
				       const char *page, size_t len)
{
	struct fsg_lun_opts *opts = to_fsg_lun_opts(item);
	struct fsg_opts *fsg_opts = to_fsg_opts(opts->group.cg_item.ci_parent);

	return fsg_store_file(opts->lun, &fsg_opts->common->filesem, page, len);
}

CONFIGFS_ATTR(fsg_lun_opts_, file);

static ssize_t fsg_lun_opts_ro_show(struct config_item *item, char *page)
{
	return fsg_show_ro(to_fsg_lun_opts(item)->lun, page);
}

static ssize_t fsg_lun_opts_ro_store(struct config_item *item,
				       const char *page, size_t len)
{
	struct fsg_lun_opts *opts = to_fsg_lun_opts(item);
	struct fsg_opts *fsg_opts = to_fsg_opts(opts->group.cg_item.ci_parent);

	return fsg_store_ro(opts->lun, &fsg_opts->common->filesem, page, len);
}

CONFIGFS_ATTR(fsg_lun_opts_, ro);

static ssize_t fsg_lun_opts_removable_show(struct config_item *item,
					   char *page)
{
	return fsg_show_removable(to_fsg_lun_opts(item)->lun, page);
}

static ssize_t fsg_lun_opts_removable_store(struct config_item *item,
				       const char *page, size_t len)
{
	return fsg_store_removable(to_fsg_lun_opts(item)->lun, page, len);
}

CONFIGFS_ATTR(fsg_lun_opts_, removable);

static ssize_t fsg_lun_opts_cdrom_show(struct config_item *item, char *page)
{
	return fsg_show_cdrom(to_fsg_lun_opts(item)->lun, page);
}

static ssize_t fsg_lun_opts_cdrom_store(struct config_item *item,
				       const char *page, size_t len)
{
	struct fsg_lun_opts *opts = to_fsg_lun_opts(item);
	struct fsg_opts *fsg_opts = to_fsg_opts(opts->group.cg_item.ci_parent);

	return fsg_store_cdrom(opts->lun, &fsg_opts->common->filesem, page,
			       len);
}

CONFIGFS_ATTR(fsg_lun_opts_, cdrom);

static ssize_t fsg_lun_opts_profile_show(struct config_item *item, char *page)
{
	return fsg_show_profile(to_fsg_lun_opts(item)->lun, page);
}

static ssize_t fsg_lun_opts_profile_store(struct config_item *item,
					  const char *page, size_t len)
{
	struct fsg_lun_opts *opts = to_fsg_lun_opts(item);
	struct fsg_opts *fsg_opts = to_fsg_opts(opts->group.cg_item.ci_parent);

	return fsg_store_profile(opts->lun, &fsg_opts->common->filesem, page,
				 len);
}

CONFIGFS_ATTR(fsg_lun_opts_, profile);

static ssize_t fsg_lun_opts_tracks_show(struct config_item *item, char *page)
{
	struct fsg_lun_opts *opts = to_fsg_lun_opts(item);
	struct fsg_opts *fsg_opts = to_fsg_opts(opts->group.cg_item.ci_parent);

	return fsg_show_tracks(opts->lun, &fsg_opts->common->filesem, page);
}

static ssize_t fsg_lun_opts_tracks_store(struct config_item *item,
					 const char *page, size_t len)
{
	struct fsg_lun_opts *opts = to_fsg_lun_opts(item);
	struct fsg_opts *fsg_opts = to_fsg_opts(opts->group.cg_item.ci_parent);

	return fsg_store_tracks(opts->lun, &fsg_opts->common->filesem, page,
				len);
}

CONFIGFS_ATTR(fsg_lun_opts_, tracks);

static ssize_t fsg_lun_opts_eject_request_store(struct config_item *item,
						const char *page, size_t len)
{
	struct fsg_lun_opts *opts = to_fsg_lun_opts(item);
	struct fsg_opts *fsg_opts = to_fsg_opts(opts->group.cg_item.ci_parent);

	return fsg_store_eject_request(opts->lun, &fsg_opts->common->filesem,
				       page, len);
}

CONFIGFS_ATTR_WO(fsg_lun_opts_, eject_request);

static ssize_t fsg_lun_opts_nofua_show(struct config_item *item, char *page)
{
	return fsg_show_nofua(to_fsg_lun_opts(item)->lun, page);
}

static ssize_t fsg_lun_opts_nofua_store(struct config_item *item,
				       const char *page, size_t len)
{
	return fsg_store_nofua(to_fsg_lun_opts(item)->lun, page, len);
}

CONFIGFS_ATTR(fsg_lun_opts_, nofua);

static ssize_t fsg_lun_opts_inquiry_string_show(struct config_item *item,
						char *page)
{
	return fsg_show_inquiry_string(to_fsg_lun_opts(item)->lun, page);
}

static ssize_t fsg_lun_opts_inquiry_string_store(struct config_item *item,
						 const char *page, size_t len)
{
	return fsg_store_inquiry_string(to_fsg_lun_opts(item)->lun, page, len);
}

CONFIGFS_ATTR(fsg_lun_opts_, inquiry_string);

static ssize_t fsg_lun_opts_forced_eject_store(struct config_item *item,
					       const char *page, size_t len)
{
	struct fsg_lun_opts *opts = to_fsg_lun_opts(item);
	struct fsg_opts *fsg_opts = to_fsg_opts(opts->group.cg_item.ci_parent);

	return fsg_store_forced_eject(opts->lun, &fsg_opts->common->filesem,
				      page, len);
}

CONFIGFS_ATTR_WO(fsg_lun_opts_, forced_eject);

static struct configfs_attribute *fsg_lun_attrs[] = {
	&fsg_lun_opts_attr_file,
	&fsg_lun_opts_attr_ro,
	&fsg_lun_opts_attr_removable,
	&fsg_lun_opts_attr_cdrom,
	&fsg_lun_opts_attr_profile,
	&fsg_lun_opts_attr_tracks,
	&fsg_lun_opts_attr_eject_request,
	&fsg_lun_opts_attr_nofua,
	&fsg_lun_opts_attr_inquiry_string,
	&fsg_lun_opts_attr_forced_eject,
	NULL,
};

static const struct config_item_type fsg_lun_type = {
	.ct_item_ops	= &fsg_lun_item_ops,
	.ct_attrs	= fsg_lun_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group *fsg_lun_make(struct config_group *group,
					 const char *name)
{
	struct fsg_lun_opts *opts;
	struct fsg_opts *fsg_opts;
	struct fsg_lun_config config;
	char *num_str;
	u8 num;
	int ret;

	num_str = strchr(name, '.');
	if (!num_str) {
		pr_err("Unable to locate . in LUN.NUMBER\n");
		return ERR_PTR(-EINVAL);
	}
	num_str++;

	ret = kstrtou8(num_str, 0, &num);
	if (ret)
		return ERR_PTR(ret);

	fsg_opts = to_fsg_opts(&group->cg_item);
	if (num >= FSG_MAX_LUNS)
		return ERR_PTR(-ERANGE);
	num = array_index_nospec(num, FSG_MAX_LUNS);

	mutex_lock(&fsg_opts->lock);
	if (fsg_opts->refcnt || fsg_opts->common->luns[num]) {
		ret = -EBUSY;
		goto out;
	}

	opts = kzalloc_obj(*opts);
	if (!opts) {
		ret = -ENOMEM;
		goto out;
	}

	memset(&config, 0, sizeof(config));
	config.removable = true;

	ret = fsg_common_create_lun(fsg_opts->common, &config, num, name,
				    (const char **)&group->cg_item.ci_name);
	if (ret) {
		kfree(opts);
		goto out;
	}
	opts->lun = fsg_opts->common->luns[num];
	opts->lun_id = num;
	mutex_unlock(&fsg_opts->lock);

	config_group_init_type_name(&opts->group, name, &fsg_lun_type);

	return &opts->group;
out:
	mutex_unlock(&fsg_opts->lock);
	return ERR_PTR(ret);
}

static void fsg_lun_drop(struct config_group *group, struct config_item *item)
{
	struct fsg_lun_opts *lun_opts;
	struct fsg_opts *fsg_opts;

	lun_opts = to_fsg_lun_opts(item);
	fsg_opts = to_fsg_opts(&group->cg_item);

	mutex_lock(&fsg_opts->lock);
	if (fsg_opts->refcnt) {
		struct config_item *gadget;

		gadget = group->cg_item.ci_parent->ci_parent;
		unregister_gadget_item(gadget);
	}

	fsg_common_remove_lun(lun_opts->lun);
	fsg_opts->common->luns[lun_opts->lun_id] = NULL;
	lun_opts->lun_id = 0;
	mutex_unlock(&fsg_opts->lock);

	config_item_put(item);
}

static void fsg_attr_release(struct config_item *item)
{
	struct fsg_opts *opts = to_fsg_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static const struct configfs_item_operations fsg_item_ops = {
	.release		= fsg_attr_release,
};

static ssize_t fsg_opts_stall_show(struct config_item *item, char *page)
{
	struct fsg_opts *opts = to_fsg_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%d", opts->common->can_stall);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t fsg_opts_stall_store(struct config_item *item, const char *page,
				    size_t len)
{
	struct fsg_opts *opts = to_fsg_opts(item);
	int ret;
	bool stall;

	mutex_lock(&opts->lock);

	if (opts->refcnt) {
		mutex_unlock(&opts->lock);
		return -EBUSY;
	}

	ret = kstrtobool(page, &stall);
	if (!ret) {
		opts->common->can_stall = stall;
		ret = len;
	}

	mutex_unlock(&opts->lock);

	return ret;
}

CONFIGFS_ATTR(fsg_opts_, stall);

#ifdef CONFIG_USB_GADGET_DEBUG_FILES
static ssize_t fsg_opts_num_buffers_show(struct config_item *item, char *page)
{
	struct fsg_opts *opts = to_fsg_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%d", opts->common->fsg_num_buffers);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t fsg_opts_num_buffers_store(struct config_item *item,
					  const char *page, size_t len)
{
	struct fsg_opts *opts = to_fsg_opts(item);
	int ret;
	u8 num;

	mutex_lock(&opts->lock);
	if (opts->refcnt) {
		ret = -EBUSY;
		goto end;
	}
	ret = kstrtou8(page, 0, &num);
	if (ret)
		goto end;

	ret = fsg_common_set_num_buffers(opts->common, num);
	if (ret)
		goto end;
	ret = len;

end:
	mutex_unlock(&opts->lock);
	return ret;
}

CONFIGFS_ATTR(fsg_opts_, num_buffers);
#endif

static struct configfs_attribute *fsg_attrs[] = {
	&fsg_opts_attr_stall,
#ifdef CONFIG_USB_GADGET_DEBUG_FILES
	&fsg_opts_attr_num_buffers,
#endif
	NULL,
};

static const struct configfs_group_operations fsg_group_ops = {
	.make_group	= fsg_lun_make,
	.drop_item	= fsg_lun_drop,
};

static const struct config_item_type fsg_func_type = {
	.ct_item_ops	= &fsg_item_ops,
	.ct_group_ops	= &fsg_group_ops,
	.ct_attrs	= fsg_attrs,
	.ct_owner	= THIS_MODULE,
};

static void fsg_free_inst(struct usb_function_instance *fi)
{
	struct fsg_opts *opts;

	opts = fsg_opts_from_func_inst(fi);
	fsg_common_release(opts->common);
	kfree(opts);
}

static struct usb_function_instance *fsg_alloc_inst(void)
{
	struct fsg_opts *opts;
	struct fsg_lun_config config;
	int rc;

	opts = kzalloc_obj(*opts);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	mutex_init(&opts->lock);
	opts->func_inst.free_func_inst = fsg_free_inst;
	opts->common = fsg_common_setup(opts->common);
	if (IS_ERR(opts->common)) {
		rc = PTR_ERR(opts->common);
		goto release_opts;
	}

	rc = fsg_common_set_num_buffers(opts->common,
					CONFIG_USB_GADGET_STORAGE_NUM_BUFFERS);
	if (rc)
		goto release_common;

	memset(&config, 0, sizeof(config));
	config.removable = true;
	rc = fsg_common_create_lun(opts->common, &config, 0, "lun.0",
			(const char **)&opts->func_inst.group.cg_item.ci_name);
	if (rc)
		goto release_buffers;

	opts->lun0.lun = opts->common->luns[0];
	opts->lun0.lun_id = 0;

	config_group_init_type_name(&opts->func_inst.group, "", &fsg_func_type);

	config_group_init_type_name(&opts->lun0.group, "lun.0", &fsg_lun_type);
	configfs_add_default_group(&opts->lun0.group, &opts->func_inst.group);

	return &opts->func_inst;

release_buffers:
	fsg_common_free_buffers(opts->common);
release_common:
	kfree(opts->common);
release_opts:
	kfree(opts);
	return ERR_PTR(rc);
}

static void fsg_free(struct usb_function *f)
{
	struct fsg_dev *fsg;
	struct fsg_opts *opts;

	fsg = container_of(f, struct fsg_dev, function);
	opts = container_of(f->fi, struct fsg_opts, func_inst);

	mutex_lock(&opts->lock);
	opts->refcnt--;
	mutex_unlock(&opts->lock);

	kfree(fsg);
}

static struct usb_function *fsg_alloc(struct usb_function_instance *fi)
{
	struct fsg_opts *opts = fsg_opts_from_func_inst(fi);
	struct fsg_common *common = opts->common;
	struct fsg_dev *fsg;

	fsg = kzalloc_obj(*fsg);
	if (unlikely(!fsg))
		return ERR_PTR(-ENOMEM);

	mutex_lock(&opts->lock);
	opts->refcnt++;
	mutex_unlock(&opts->lock);

	fsg->function.name	= FSG_DRIVER_DESC;
	fsg->function.bind	= fsg_bind;
	fsg->function.unbind	= fsg_unbind;
	fsg->function.setup	= fsg_setup;
	fsg->function.set_alt	= fsg_set_alt;
	fsg->function.disable	= fsg_disable;
	fsg->function.free_func	= fsg_free;

	fsg->common               = common;

	return &fsg->function;
}

DECLARE_USB_FUNCTION_INIT(mass_storage, fsg_alloc_inst, fsg_alloc);
MODULE_DESCRIPTION("Mass Storage USB Composite Function");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michal Nazarewicz");

/************************* Module parameters *************************/


void fsg_config_from_params(struct fsg_config *cfg,
		       const struct fsg_module_parameters *params,
		       unsigned int fsg_num_buffers)
{
	struct fsg_lun_config *lun;
	unsigned i;

	/* Configure LUNs */
	cfg->nluns =
		min(params->luns ?: (params->file_count ?: 1u),
		    (unsigned)FSG_MAX_LUNS);
	for (i = 0, lun = cfg->luns; i < cfg->nluns; ++i, ++lun) {
		lun->ro = !!params->ro[i];
		lun->cdrom = !!params->cdrom[i];
		lun->removable = !!params->removable[i];
		lun->filename =
			params->file_count > i && params->file[i][0]
			? params->file[i]
			: NULL;
	}

	/* Let MSF use defaults */
	cfg->vendor_name = NULL;
	cfg->product_name = NULL;

	cfg->ops = NULL;
	cfg->private_data = NULL;

	/* Finalise */
	cfg->can_stall = params->stall;
	cfg->fsg_num_buffers = fsg_num_buffers;
}
EXPORT_SYMBOL_GPL(fsg_config_from_params);
