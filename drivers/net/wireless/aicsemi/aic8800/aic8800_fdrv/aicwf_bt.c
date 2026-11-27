// SPDX-License-Identifier: GPL-2.0
/*
 * The AIC8800's Bluetooth controller as an HCI device.
 *
 * With the Wi-Fi firmware built for it (fmacfwbt) and the BT port set to the
 * mailbox, the controller's H4 packets travel as driver messages over SDIO:
 * TDLS_SDIO_BT_SEND_REQ to the controller, TDLS_SDIO_BT_RECV_IND from it,
 * one packet each. The HCI device lives as long as the Wi-Fi driver.
 */

#include <linux/rcupdate.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "rwnx_defs.h"
#include "rwnx_msg_tx.h"
#include "aicwf_bt.h"

#define AICWF_BT_MAX_SEND	sizeof_field(struct mm_bt_send_req, bt_data)
#define AICWF_BT_MAX_RECV	sizeof_field(struct mm_bt_recv_ind, bt_data)
#define AICWF_BT_MAX_ACL	(AICWF_BT_MAX_SEND - 1 - HCI_ACL_HDR_SIZE)

struct aicwf_bt {
	struct rwnx_hw *rwnx_hw;
	struct hci_dev *hdev;
	struct work_struct tx_work;
	struct sk_buff_head txq;
};

static void aicwf_bt_count_tx(struct hci_dev *hdev, struct sk_buff *skb)
{
	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	}
	hdev->stat.byte_tx += skb->len;
}

/* Sending waits for the firmware's confirmation, so it sleeps. */
static void aicwf_bt_tx_work(struct work_struct *work)
{
	struct aicwf_bt *bt = container_of(work, struct aicwf_bt, tx_work);
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&bt->txq))) {
		int err = rwnx_sdio_bt_send_req(bt->rwnx_hw, hci_skb_pkt_type(skb),
						skb->data, skb->len);

		if (err) {
			bt->hdev->stat.err_tx++;
			bt_dev_err(bt->hdev, "packet type %u, %u bytes not sent: %d",
				   hci_skb_pkt_type(skb), skb->len, err);
		} else {
			aicwf_bt_count_tx(bt->hdev, skb);
		}
		kfree_skb(skb);
	}
}

static int aicwf_bt_open(struct hci_dev *hdev)
{
	return 0;
}

static int aicwf_bt_flush(struct hci_dev *hdev)
{
	struct aicwf_bt *bt = hci_get_drvdata(hdev);

	skb_queue_purge(&bt->txq);
	return 0;
}

static int aicwf_bt_close(struct hci_dev *hdev)
{
	struct aicwf_bt *bt = hci_get_drvdata(hdev);

	cancel_work_sync(&bt->tx_work);
	skb_queue_purge(&bt->txq);
	return 0;
}

static int aicwf_bt_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct aicwf_bt *bt = hci_get_drvdata(hdev);

	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
	case HCI_ACLDATA_PKT:
	case HCI_SCODATA_PKT:
		break;
	default:
		return -EILSEQ;
	}

	if (skb->len + 1 > AICWF_BT_MAX_SEND)
		return -EMSGSIZE;

	skb_queue_tail(&bt->txq, skb);
	schedule_work(&bt->tx_work);
	return 0;
}

/*
 * The controller takes ACL packets of 1021 bytes, a message carries 1019 of
 * them: the stack learns the length that fits and fragments to it.
 */
static void aicwf_bt_fixup(struct sk_buff *skb)
{
	struct hci_rp_read_buffer_size *rp;
	struct hci_ev_cmd_complete *cc;

	if (hci_skb_pkt_type(skb) != HCI_EVENT_PKT ||
	    skb->len < HCI_EVENT_HDR_SIZE + sizeof(*cc) + sizeof(*rp) ||
	    hci_event_hdr(skb)->evt != HCI_EV_CMD_COMPLETE)
		return;

	cc = (void *)(skb->data + HCI_EVENT_HDR_SIZE);
	if (__le16_to_cpu(cc->opcode) != HCI_OP_READ_BUFFER_SIZE)
		return;

	rp = (void *)(cc + 1);
	if (!rp->status && __le16_to_cpu(rp->acl_mtu) > AICWF_BT_MAX_ACL)
		rp->acl_mtu = cpu_to_le16(AICWF_BT_MAX_ACL);
}

/* Called from the message path, which may not sleep. */
void aicwf_bt_recv(struct rwnx_hw *rwnx_hw, const u8 *data, u32 len)
{
	struct aicwf_bt *bt;
	struct sk_buff *skb;

	rcu_read_lock();
	bt = rcu_dereference(rwnx_hw->bt);
	if (!bt)
		goto out;

	if (len < 2 || len > AICWF_BT_MAX_RECV) {
		bt->hdev->stat.err_rx++;
		goto out;
	}

	skb = bt_skb_alloc(len - 1, GFP_ATOMIC);
	if (!skb) {
		bt->hdev->stat.err_rx++;
		goto out;
	}

	hci_skb_pkt_type(skb) = data[0];
	skb_put_data(skb, data + 1, len - 1);
	aicwf_bt_fixup(skb);
	bt->hdev->stat.byte_rx += len - 1;
	hci_recv_frame(bt->hdev, skb);
out:
	rcu_read_unlock();
}

int aicwf_bt_init(struct rwnx_hw *rwnx_hw)
{
	struct aicwf_bt *bt;
	struct hci_dev *hdev;
	int err;

	bt = kzalloc(sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;

	hdev = hci_alloc_dev();
	if (!hdev) {
		kfree(bt);
		return -ENOMEM;
	}

	bt->rwnx_hw = rwnx_hw;
	bt->hdev = hdev;
	INIT_WORK(&bt->tx_work, aicwf_bt_tx_work);
	skb_queue_head_init(&bt->txq);

	hdev->bus = HCI_SDIO;
	hci_set_drvdata(hdev, bt);
	SET_HCIDEV_DEV(hdev, rwnx_hw->dev);
	hdev->open = aicwf_bt_open;
	hdev->close = aicwf_bt_close;
	hdev->flush = aicwf_bt_flush;
	hdev->send = aicwf_bt_send;

	/* The controller answers the stack's first commands in registration. */
	rcu_assign_pointer(rwnx_hw->bt, bt);

	err = hci_register_dev(hdev);
	if (err < 0) {
		RCU_INIT_POINTER(rwnx_hw->bt, NULL);
		synchronize_rcu();
		hci_free_dev(hdev);
		kfree(bt);
		return err;
	}

	return 0;
}

void aicwf_bt_deinit(struct rwnx_hw *rwnx_hw)
{
	struct aicwf_bt *bt = rcu_dereference_protected(rwnx_hw->bt, true);

	if (!bt)
		return;

	hci_unregister_dev(bt->hdev);
	RCU_INIT_POINTER(rwnx_hw->bt, NULL);
	synchronize_rcu();
	cancel_work_sync(&bt->tx_work);
	skb_queue_purge(&bt->txq);
	hci_free_dev(bt->hdev);
	kfree(bt);
}
