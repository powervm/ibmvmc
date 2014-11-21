/* TODO - update this header
 * IBM PowerPC Virtual Communications Channel Support.
 *
 *    Copyright (c) 2004 IBM Corp.
 *     Dave Engebretsen engebret@us.ibm.com
 *     Steven Royer seroyer@us.ibm.com
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/percpu.h>
#include <linux/delay.h>

#include <asm/byteorder.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/vio.h>

#include "ibmvmc.h"

#define IBMVMC_DRIVER_VERSION "1.0"

MODULE_DESCRIPTION("IBM VMC");
MODULE_AUTHOR("Steven Royer <seroyer@us.ibm.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(IBMVMC_DRIVER_VERSION);


/*
 * Static global variables
 */
static int ibmvmc_log_level = DEFAULT_LOG_LEVEL;

#define err(format, arg...) printk(KERN_ERR "ibmvmc: " format , ## arg)
#define warn(format, arg...) printk(KERN_WARNING "ibmvmc: " format , ## arg)
#define info(level, format, arg...) \
	if (level <= ibmvmc_log_level) { \
		printk(KERN_INFO "ibmvmc: " format  , ## arg); \
	}

static DECLARE_WAIT_QUEUE_HEAD(ibmvmc_read_wait);

static char ibmvmc_driver_name[] = "ibmvmc";
static char ibmvmc_workq_name[] = "ibmvmc";

static struct ibmvmc_struct ibmvmc;
static struct ibmvmc_hmc hmcs[MAX_HMCS];
static struct crq_server_adapter ibmvmc_adapter;
static dev_t ibmvmc_chrdev;

static int ibmvmc_max_buf_pool_size = DEFAULT_BUF_POOL_SIZE;
static int ibmvmc_max_hmcs = DEFAULT_HMCS;
static int ibmvmc_max_mtu = DEFAULT_MTU;

/* Module parameters */
module_param_named(log_level, ibmvmc_log_level, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(log_level, "Log level");
module_param_named(buf_pool_size, ibmvmc_max_buf_pool_size, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(buf_pool_size, "Buffer pool size");
module_param_named(max_hmcs, ibmvmc_max_hmcs, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_hmcs, "Max HMCs");
module_param_named(max_mtu, ibmvmc_max_mtu, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_mtu, "Max MTU");


static inline long h_copy_rdma(s64 length,
			u64 sliobn, u64 slioba,
			u64 dliobn, u64 dlioba)
{
	long rc = 0;
	mb();
	info(LOG_LEVEL_TRACE, "h_copy_rdma(0x%llx, 0x%llx, 0x%llx, 0x%llx, "
		"0x%llx\n", length, sliobn, slioba, dliobn, dlioba);
	rc = plpar_hcall_norets(H_COPY_RDMA, length, sliobn, slioba,
				dliobn, dlioba);
	info(LOG_LEVEL_TRACE, "h_copy_rdma rc = 0x%lx\n", rc);
	return rc;
}

static inline void h_free_crq(uint32_t unit_address)
{
	long rc = 0;
	do {
		if (H_IS_LONG_BUSY(rc)) {
			msleep(get_longbusy_msecs(rc));
		}
		rc = plpar_hcall_norets(H_FREE_CRQ, unit_address);
	} while ((rc == H_BUSY) || (H_IS_LONG_BUSY(rc)));
}

/* routines for managing a command/response queue */
/**
 * ibmvmc_handle_event: - Interrupt handler for crq events
 * @irq:        number of irq to handle, not used
 * @dev_instance: crq_server_adapter that received interrupt
 *
 * Disables interrupts and schedules ibmvmc_task
 * Always returns IRQ_HANDLED
 */
static irqreturn_t ibmvmc_handle_event(int irq, void *dev_instance)
{
	struct crq_server_adapter *adapter =
		(struct crq_server_adapter *)dev_instance;
	vio_disable_interrupts(to_vio_dev(adapter->dev));
	queue_work(adapter->work_queue, &adapter->work);
	return IRQ_HANDLED;
}

static void ibmvmc_release_crq_queue(struct crq_server_adapter *adapter)
{
	struct vio_dev *vdev = to_vio_dev(adapter->dev);
	struct crq_queue *queue = &adapter->queue;

	free_irq(vdev->irq, (void *)adapter);
	flush_workqueue(adapter->work_queue);
	destroy_workqueue(adapter->work_queue);

	h_free_crq(vdev->unit_address);
	dma_unmap_single(adapter->dev,
			queue->msg_token,
			queue->size * sizeof(*queue->msgs), DMA_BIDIRECTIONAL);
	free_page((unsigned long)queue->msgs);
}

static int ibmvmc_reset_crq_queue(struct crq_server_adapter *adapter)
{
        int rc = 0;
	struct vio_dev *vdev = to_vio_dev(adapter->dev);
	struct crq_queue *queue = &adapter->queue;

	/* Close the CRQ */
	h_free_crq(vdev->unit_address);

	/* Clean out the queue */
	memset(queue->msgs, 0x00, PAGE_SIZE);
	queue->cur = 0;

	/* And re-open it again */
	rc = plpar_hcall_norets(H_REG_CRQ,
				vdev->unit_address,
				queue->msg_token, PAGE_SIZE);
	if (rc == 2) {
		/* Adapter is good, but other end is not ready */
		warn("Partner adapter not ready\n");
	} else if (rc != 0) {
		err("couldn't register crq--rc 0x%x\n", rc);
	}
	return rc;
}

/**
 * crq_queue_next_crq: - Returns the next entry in message queue
 * @queue:      crq_queue to use
 *
 * Returns pointer to next entry in queue, or NULL if there are no new
 * entried in the CRQ.
 */
static struct crq_msg_ibmvmc *crq_queue_next_crq(struct crq_queue *queue)
{
	struct crq_msg_ibmvmc *crq;
	unsigned long flags;

	spin_lock_irqsave(&queue->lock, flags);
	crq = &queue->msgs[queue->cur];
	if (crq->valid & 0x80) {
		if (++queue->cur == queue->size) {
			queue->cur = 0;
		}

		/* Ensure the read of the valid bit occurs before reading any
		 * other bits of the CRQ entry
		 */
		rmb();
	} else {
		crq = NULL;
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	return crq;
}

static long ibmvmc_send_crq(struct crq_server_adapter *adapter,
			u64 word1, u64 word2)
{
	long rc;
	struct vio_dev *vdev = to_vio_dev(adapter->dev);

	info(LOG_LEVEL_TRACE, "ibmvmc_send_crq(0x%x, 0x%016llx, 0x%016llx)\n",
			vdev->unit_address, word1, word2);

	/*
	 * Ensure the command buffer is flushed to memory before handing it
	 * over to the other side to prevent it from fetching any stale data.
	 */
	mb();
	rc = plpar_hcall_norets(H_SEND_CRQ, vdev->unit_address, word1, word2);
	info(LOG_LEVEL_TRACE, "ibmvmc_send_crq rc = 0x%lx\n", rc);

	return rc;
}

static void *alloc_dma_buffer(struct vio_dev *vdev, size_t size,
				dma_addr_t *dma_handle)
{
	// allocate memory
	void *buffer = kzalloc(size, GFP_KERNEL);

	if (!buffer) {
		*dma_handle = 0;
		return NULL;
	}

	// DMA map
	*dma_handle = dma_map_single(&vdev->dev, buffer, size,
				    DMA_BIDIRECTIONAL);

	if (dma_mapping_error(&vdev->dev, *dma_handle)) {
		*dma_handle = 0;
		kzfree(buffer);
		return NULL;
	}

	return buffer;
}

static void free_dma_buffer(struct vio_dev *vdev, size_t size, void *vaddr,
			    dma_addr_t dma_handle)
{
	// DMA unmap
	dma_unmap_single(&vdev->dev, dma_handle, size, DMA_BIDIRECTIONAL);

	// deallocate memory
	kzfree(vaddr);
}

static struct ibmvmc_buffer * get_valid_hmc_buffer_locked(u8 hmc_index)
{
	unsigned long i;
	struct ibmvmc_buffer *buffer;
	struct ibmvmc_buffer *ret_buf = NULL;

	if(hmc_index > ibmvmc.max_hmc_index)
		return NULL;

	buffer = hmcs[hmc_index].buffer;

	for(i=0; i<ibmvmc_max_buf_pool_size; i++) {
		if((buffer[i].valid) && (buffer[i].free) &&
		   (buffer[i].owner == VMC_BUF_OWNER_ALPHA)) {
			buffer[i].free = 0;
			ret_buf = &(buffer[i]);
			break;
		}
	}

	return ret_buf;
}

static struct ibmvmc_buffer * get_free_hmc_buffer_locked(u8 hmc_index)
{
	unsigned long i;
	struct ibmvmc_buffer *buffer;
	struct ibmvmc_buffer *ret_buf = NULL;

	if(hmc_index > ibmvmc.max_hmc_index) {
		info(LOG_LEVEL_NORM, "get_free_hmc_buffer: "
				"invalid hmc_index = 0x%x\n", hmc_index);
		return NULL;
	}

	buffer = hmcs[hmc_index].buffer;

	for(i=0; i<ibmvmc_max_buf_pool_size; i++) {
		if((buffer[i].free) && (buffer[i].owner == VMC_BUF_OWNER_ALPHA)) {
			buffer[i].free = 0;
			ret_buf = &(buffer[i]);
			break;
		}
	}

	return ret_buf;
}

static void return_hmc_buffer(struct ibmvmc_hmc *hmc,
		struct ibmvmc_buffer *buffer)
{
	unsigned long flags;
	spin_lock_irqsave(&(hmc->lock), flags);
	buffer->free = 1;
	spin_unlock_irqrestore(&(hmc->lock), flags);
}

static void count_hmc_buffers(u8 hmc_index, unsigned int *valid,
		unsigned int *free)
{
	unsigned long i;
	struct ibmvmc_buffer *buffer;
	unsigned long flags;

	if(hmc_index > ibmvmc.max_hmc_index)
		return;

	if(valid == NULL || free == NULL)
		return;

	*valid = 0; *free = 0;

	buffer = hmcs[hmc_index].buffer;
	spin_lock_irqsave(&(hmcs[hmc_index].lock), flags);

	for(i=0; i<ibmvmc_max_buf_pool_size; i++) {
		if(buffer[i].valid) {
			*valid = *valid + 1;
			if(buffer[i].free) {
				*free = *free + 1;
			}
		}
	}

	spin_unlock_irqrestore(&(hmcs[hmc_index].lock), flags);
}

static struct ibmvmc_hmc * ibmvmc_get_free_hmc(void)
{
	unsigned long i;
	unsigned long flags;

	/*
	 * Find an available HMC connection.
	 */
	for(i = 0; i <= ibmvmc.max_hmc_index; i++) {
		spin_lock_irqsave(&(hmcs[i].lock), flags);
		if(hmcs[i].state == ibmhmc_state_free) {
			hmcs[i].index = i;
			hmcs[i].state = ibmhmc_state_initial;
			spin_unlock_irqrestore(&(hmcs[i].lock), flags);
			return(&hmcs[i]);
		}
		spin_unlock_irqrestore(&(hmcs[i].lock), flags);
	}

	return NULL;
}

static int ibmvmc_return_hmc(struct ibmvmc_hmc *hmc)
{
	unsigned long i;
	struct ibmvmc_buffer *buffer;
	struct crq_server_adapter *adapter;
	struct vio_dev *vdev;
	unsigned long flags;

	if((hmc == NULL) || (hmc->adapter == NULL))
		return -EIO;

	adapter = hmc->adapter;
	vdev = to_vio_dev(adapter->dev);

	spin_lock_irqsave(&(hmc->lock), flags);
	hmc->index = 0;
	hmc->state = ibmhmc_state_free;
	hmc->queue_head = 0;
	hmc->queue_tail = 0;
	hmc->file = NULL;
	buffer = hmc->buffer;
	for(i=0; i<ibmvmc_max_buf_pool_size; i++) {
		if(buffer[i].valid) {
			free_dma_buffer(vdev,
					    ibmvmc.max_mtu,
					    buffer[i].real_addr_local,
					    buffer[i].dma_addr_local);
			info(LOG_LEVEL_TRACE, "Forgot buffer id 0x%lx\n", i);
		}
		memset(&buffer[i], 0, sizeof(struct ibmvmc_buffer));

		hmc->queue_outbound_msgs[i] = VMC_INVALID_BUFFER_ID;
	}

	spin_unlock_irqrestore(&(hmc->lock), flags);

	return 0;
}

static int send_open(struct ibmvmc_buffer *buffer,
		     struct ibmvmc_hmc *hmc)
{
	int rc = 0;
	struct crq_msg_ibmvmc crq_msg;
	__be64 *crq_as_u64 = (__be64 *)&crq_msg;
	struct crq_server_adapter *adapter;

	if((hmc == NULL) || (hmc->adapter == NULL))
		return -EIO;

	adapter = hmc->adapter;

	info(LOG_LEVEL_TRACE, "ibmvmc send_open: 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx\n",
	     (unsigned long)buffer->size, (unsigned long)adapter->liobn,
	     (unsigned long)buffer->dma_addr_local, (unsigned long)adapter->riobn,
	     (unsigned long)buffer->dma_addr_remote);

	rc = h_copy_rdma(buffer->size,
			 adapter->liobn,
			 buffer->dma_addr_local,
			 adapter->riobn,
			 buffer->dma_addr_remote);
	if (rc) {
		err("Error: In send_open, h_copy_rdma rc 0x%x\n", rc);
		return -EIO;
	}

	hmc->state = ibmhmc_state_opening;

	crq_msg.valid = 0x80;
	crq_msg.type = VMC_MSG_OPEN;
	crq_msg.status = 0;
	crq_msg.var1.rsvd = 0;
	crq_msg.hmc_session = hmc->session;
	crq_msg.hmc_index = hmc->index;
	crq_msg.var2.buffer_id = cpu_to_be16(buffer->id);
	crq_msg.rsvd = 0;
	crq_msg.var3.rsvd = 0;

	ibmvmc_send_crq(adapter, be64_to_cpu(crq_as_u64[0]),
			be64_to_cpu(crq_as_u64[1]));

	return rc;
}

static int send_close(struct ibmvmc_hmc *hmc)
{
	int rc = 0;
	struct crq_msg_ibmvmc crq_msg;
	__be64 *crq_as_u64 = (__be64 *)&crq_msg;
	struct crq_server_adapter *adapter;

	info(LOG_LEVEL_NORM, "CRQ send: close\n");

	if((hmc == NULL) || (hmc->adapter == NULL))
		return -EIO;

	adapter = hmc->adapter;

	crq_msg.valid = 0x80;
	crq_msg.type = VMC_MSG_CLOSE;
	crq_msg.status = 0;
	crq_msg.var1.rsvd = 0;
	crq_msg.hmc_session = hmc->session;
	crq_msg.hmc_index = hmc->index;
	crq_msg.var2.rsvd = 0;
	crq_msg.rsvd = 0;
	crq_msg.var3.rsvd = 0;

	ibmvmc_send_crq(adapter, be64_to_cpu(crq_as_u64[0]),
			be64_to_cpu(crq_as_u64[1]));

	return rc;
}

static int ibmvmc_send_capabilities(struct crq_server_adapter *adapter)
{
	struct crq_msg_ibmvmc_admin crq_msg;
	__be64 *crq_as_u64 = (__be64 *)&crq_msg;

	info(LOG_LEVEL_TRACE, "CRQ send: capabilities\n");
	crq_msg.valid = 0x80;
	crq_msg.type = VMC_MSG_CAP;
	crq_msg.status = 0;
	crq_msg.rsvd[0] = 0;
	crq_msg.rsvd[1] = 0;
	crq_msg.max_hmc = ibmvmc_max_hmcs;
	crq_msg.max_mtu = cpu_to_be32(ibmvmc_max_mtu);
	crq_msg.pool_size = cpu_to_be16(ibmvmc_max_buf_pool_size);
	crq_msg.crq_size = cpu_to_be16(adapter->queue.size);
	crq_msg.version = cpu_to_be16(IBMVMC_PROTOCOL_VERSION);

	ibmvmc_send_crq(adapter, be64_to_cpu(crq_as_u64[0]),
			be64_to_cpu(crq_as_u64[1]));

	ibmvmc.state = ibmvmc_state_capabilities;

	return 0;
}

static int ibmvmc_send_add_buffer_resp(struct crq_server_adapter *adapter,
				u8 status, u8 hmc_session, u8 hmc_index, u16 buffer_id)
{
	struct crq_msg_ibmvmc crq_msg;
	__be64 *crq_as_u64 = (__be64 *)&crq_msg;

	info(LOG_LEVEL_TRACE, "CRQ send: add_buffer_resp\n");
	crq_msg.valid = 0x80;
	crq_msg.type = VMC_MSG_ADD_BUF_RESP;
	crq_msg.status = status;
	crq_msg.var1.rsvd = 0;
	crq_msg.hmc_session = hmc_session;
	crq_msg.hmc_index = hmc_index;
	crq_msg.var2.buffer_id = cpu_to_be16(buffer_id);
	crq_msg.rsvd = 0;
	crq_msg.var3.rsvd = 0;

	ibmvmc_send_crq(adapter, be64_to_cpu(crq_as_u64[0]),
			be64_to_cpu(crq_as_u64[1]));

	return 0;
}

static int ibmvmc_send_rem_buffer_resp(struct crq_server_adapter *adapter,
				u8 status, u8 hmc_session, u8 hmc_index, u16 buffer_id)
{
	struct crq_msg_ibmvmc crq_msg;
	__be64 *crq_as_u64 = (__be64 *)&crq_msg;

	info(LOG_LEVEL_TRACE, "CRQ send: rem_buffer_resp\n");
	crq_msg.valid = 0x80;
	crq_msg.type = VMC_MSG_REM_BUF_RESP;
	crq_msg.status = status;
	crq_msg.var1.rsvd = 0;
	crq_msg.hmc_session = hmc_session;
	crq_msg.hmc_index = hmc_index;
	crq_msg.var2.buffer_id = cpu_to_be16(buffer_id);
	crq_msg.rsvd = 0;
	crq_msg.var3.rsvd = 0;

	ibmvmc_send_crq(adapter, be64_to_cpu(crq_as_u64[0]),
			be64_to_cpu(crq_as_u64[1]));

	return 0;
}

static int send_msg(struct crq_server_adapter *adapter,
		    struct ibmvmc_buffer *buffer,
		    struct ibmvmc_hmc *hmc, int msg_len)
{
	int rc = 0;
	struct crq_msg_ibmvmc crq_msg;
	__be64 *crq_as_u64 = (__be64 *)&crq_msg;

	info(LOG_LEVEL_TRACE, "CRQ send: rdma to HV\n");
	rc = h_copy_rdma(msg_len,
			 adapter->liobn,
			 buffer->dma_addr_local,
			 adapter->riobn,
			 buffer->dma_addr_remote);
	if (rc) {
		err("Error: In send_msg, h_copy_rdma rc 0x%x\n", rc);
		return rc;
	}

	crq_msg.valid = 0x80;
	crq_msg.type = VMC_MSG_SIGNAL;
	crq_msg.status = 0;
	crq_msg.var1.rsvd = 0;
	crq_msg.hmc_session = hmc->session;
	crq_msg.hmc_index = hmc->index;
	crq_msg.var2.buffer_id = cpu_to_be16(buffer->id);
	crq_msg.var3.msg_len = cpu_to_be32(msg_len);
	info(LOG_LEVEL_TRACE, "CRQ send: msg to HV 0x%llx 0x%llx\n",
			be64_to_cpu(crq_as_u64[0]), be64_to_cpu(crq_as_u64[1]));

	buffer->owner = VMC_BUF_OWNER_HV;
	ibmvmc_send_crq(adapter, be64_to_cpu(crq_as_u64[0]),
			be64_to_cpu(crq_as_u64[1]));

	return rc;
}

static int ibmvmc_open(struct inode *inode, struct file *file)
{
	int retval = 0;
	unsigned long index = 0;
	struct ibmvmc_hmc *hmc;
	unsigned int valid = 0, free = 0;

	info(LOG_LEVEL_NORM, "vmc_open:  inode = 0x%lx, file = 0x%lx, "
			"state = 0x%x\n", (unsigned long)inode,
			(unsigned long)file, ibmvmc.state);

	if(ibmvmc.state == ibmvmc_state_failed) {
		warn("vmc_open: state_failed\n");
		return -EIO;
	}

	if(ibmvmc.state < ibmvmc_state_ready) {
		warn("vmc_open: not state_ready\n");
		return -EBUSY;
	}

	/* Device is busy until capabilities have been exchanged and we
	 * have a generic buffer for each possible HMC connection.
	 */
	for(index = 0; index <= ibmvmc.max_hmc_index; index++) {
		valid = 0;
		count_hmc_buffers(index, &valid, &free);
		if(valid == 0) {
			warn("vmc_open: buffers not ready for index %ld\n", index);
			return -EBUSY;
		}
	}

	/* Get an hmc object, and transition to ibmhmc_state_initial */
	hmc = ibmvmc_get_free_hmc();
	if(hmc == NULL) {
		warn("vmc_open: free hmc not found\n");
		return -EIO;
	}

	hmc->session = hmc->session+1;
	if(hmc->session == 0xff) hmc->session = 1;

	file->private_data = hmc;
	hmc->adapter = &ibmvmc_adapter;
	hmc->file = file;

	return retval;
}

static int ibmvmc_close(struct inode *inode, struct file *file)
{
	int rc = 0;
	struct crq_server_adapter *adapter;
	struct ibmvmc_hmc *hmc;

	info(LOG_LEVEL_NORM, "close:  file = 0x%lx, state = 0x%x\n",
	     (unsigned long)file, ibmvmc.state);

	hmc = file->private_data;
	if(!hmc)
		return -EIO;

	adapter = hmc->adapter;
	if(!adapter)
		return -EIO;

	if(ibmvmc.state == ibmvmc_state_failed) {
		warn("close: state_failed\n");
		return -EIO;
	}

	if(ibmvmc.state < ibmvmc_state_ready) {
		warn("close: not state_ready\n");
		return -EBUSY;
	}

	rc = send_close(hmc);
	if(rc) {
		warn("close: send_close failed.\n");
	}

	return rc;
}

static ssize_t ibmvmc_read(struct file *file, char *buf, size_t nbytes, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	ssize_t	n;
	struct ibmvmc_hmc *hmc;
	struct crq_server_adapter *adapter;
	struct ibmvmc_buffer *buffer;
	ssize_t retval = 0;
	unsigned long flags;

	info(LOG_LEVEL_TRACE, "read: file = 0x%lx, buf = 0x%lx, nbytes = 0x%lx\n",
	     (unsigned long)file, (unsigned long) buf, (unsigned long)nbytes);

	if (nbytes == 0) return 0;

	if(nbytes > ibmvmc.max_mtu) {
		warn("read: nbytes invalid 0x%x\n", (unsigned int)nbytes);
		return -EINVAL;
	}

	hmc = file->private_data;
	if(!hmc) {
		warn("read: no hmc\n");
		return -EIO;
	}

	adapter = hmc->adapter;
	if(!adapter) {
		warn("read: no adapter\n");
		return -EIO;
	}


	do {
		spin_lock_irqsave(&(hmc->lock), flags);
		if(hmc->queue_tail != hmc->queue_head) {
			/* Data is available */
			break;
		}
		spin_unlock_irqrestore(&(hmc->lock), flags);

                if (file->f_flags & O_NONBLOCK) {
                        retval = -EAGAIN;
                        goto out;
                }
                if (signal_pending(current)) {
                        retval = -ERESTARTSYS;
                        goto out;
                }
                set_current_state(TASK_INTERRUPTIBLE);
		add_wait_queue(&ibmvmc_read_wait, &wait);
                schedule();
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&ibmvmc_read_wait, &wait);
        } while (1);

	buffer = &(hmc->buffer[hmc->queue_outbound_msgs[hmc->queue_tail]]);
	hmc->queue_tail++;
	if(hmc->queue_tail == ibmvmc_max_buf_pool_size)
		hmc->queue_tail = 0;
	spin_unlock_irqrestore(&(hmc->lock), flags);

	nbytes = min(nbytes, (size_t)buffer->msg_len);
	n = copy_to_user((void *)buf, buffer->real_addr_local, nbytes);
	info(LOG_LEVEL_TRACE, "read: copy to user nbytes = 0x%lx.\n", nbytes);
	return_hmc_buffer(hmc, buffer);
	retval = nbytes;

	if (n) {
		warn("read: copy to user failed.\n");
		retval = -EFAULT;
	}

 out:
	return (retval);
}

static unsigned int ibmvmc_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	struct ibmvmc_hmc *hmc;

	hmc = file->private_data;
	if(!hmc)
		return 0;

	poll_wait(file, &ibmvmc_read_wait, wait);

	if(hmc->queue_head != hmc->queue_tail) {
		mask |= POLLIN | POLLRDNORM;
	}

	return mask;
}

static ssize_t ibmvmc_write(struct file *file, const char *buffer,
	  size_t count, loff_t *ppos)
{
	int		ret = 0;
	size_t		bytes;
	const char 	*p = buffer;
	size_t		c = count;
	struct ibmvmc_buffer *vmc_buffer;
	unsigned char *buf;
	struct crq_server_adapter *adapter;
	struct ibmvmc_hmc *hmc;
	unsigned long flags;

	hmc = file->private_data;
	if(!hmc)
		return -EIO;

	spin_lock_irqsave(&(hmc->lock), flags);
	if(hmc->state == ibmhmc_state_free) {
		/* HMC connection is not valid (possibly was reset under us). */
		ret = -EIO;
		goto out;
	}

	adapter = hmc->adapter;
	if(!adapter) {
		ret = -EIO;
		goto out;
	}

	if(count > ibmvmc.max_mtu) {
		warn("ibmvmc_write: invalid buffer size 0x%lx\n",
		     (unsigned long)count);
		ret = -EIO;
		goto out;
	}

	/* Waiting for the open resp message to the ioctl(1) - retry */
	if(hmc->state == ibmhmc_state_opening) {
		ret = -EBUSY;
		goto out;
	}

	/* Make sure the ioctl() was called & the open msg sent, and that
	 * the HMC connection has not failed.
	 */
	if(hmc->state != ibmhmc_state_ready) {
		ret = -EIO;
		goto out;
	}

	vmc_buffer = get_valid_hmc_buffer_locked(hmc->index);
	if(vmc_buffer == NULL) {
		/* No buffer available for the msg send, or we have not yet
		 * completed the open/open_resp sequence.  Retry until this is
		 * complete.
		 */
		ret = -EBUSY;
		goto out;
	}
	if(vmc_buffer->real_addr_local == NULL) {
		err("ibmvmc_write: no buffer storage assigned\n");
		ret = -EIO;
		goto out;
	}
	buf = vmc_buffer->real_addr_local;

	while (c > 0) {
		bytes = min(c, (size_t)vmc_buffer->size);

		bytes -= copy_from_user(buf, p, bytes);
		if (!bytes) {
			ret = -EFAULT;
			goto out;
		}
		c -= bytes;
		p += bytes;
	}
	if (p == buffer) {
		goto out;
	}

	file->f_dentry->d_inode->i_mtime = CURRENT_TIME;
	mark_inode_dirty(file->f_dentry->d_inode);

	info(LOG_LEVEL_TRACE, "write: file = 0x%lx, count = 0x%lx\n",
	     (unsigned long)file, (unsigned long)count);

	send_msg(adapter, vmc_buffer, hmc, count);
	ret = p - buffer;
 out:
	spin_unlock_irqrestore(&(hmc->lock), flags);
	return (ssize_t)(ret);
}

static long ibmvmc_ioctl(struct file *file,
	  unsigned int cmd, unsigned long arg)
{
	struct ibmvmc_buffer *buffer;
	size_t bytes;
	struct ibmvmc_hmc *hmc;
	long rc = 0;
	int cnt_buffers[2], n;
	char print_buffer[HMC_ID_LEN+1];
	unsigned long flags;

	hmc = file->private_data;
	info(LOG_LEVEL_TRACE, "ioctl: file=0x%lx, cmd=0x%x, arg=0x%lx, hmc=0x%lx\n",
	     (unsigned long) file, cmd,
	     arg, (unsigned long) hmc);

	if(!hmc) {
		warn("ioctl: no hmc\n");
		return -EIO;
	}

	if(hmc->state == ibmhmc_state_free || hmc->state == ibmhmc_state_failed) {
		return -EIO;
	}

	switch (cmd) {
	case VMC_IOCTL_SETHMCID:
		spin_lock_irqsave(&(hmc->lock), flags);
		buffer = get_valid_hmc_buffer_locked(hmc->index);
		spin_unlock_irqrestore(&(hmc->lock), flags);

		if(buffer == NULL || (buffer->real_addr_local == NULL)) {
			warn("ioctl: no buffer available\n");
			return -EIO;
		}

		if(hmc->state != ibmhmc_state_initial) {
			warn("ioctl: invalid state to send open message 0x%x\n",
			     hmc->state);
			return -EIO;
		}

		bytes = copy_from_user(hmc->hmc_id, (void *) arg, HMC_ID_LEN);
		if(bytes) {
			return -EFAULT;
		}

		/* Make sure buffer is NULL terminated before trying to print it */
		memset(print_buffer, 0, HMC_ID_LEN+1);
		strncpy(print_buffer, hmc->hmc_id, HMC_ID_LEN);
		info(LOG_LEVEL_NORM, "ioctl: Set HMC ID: \"%s\"\n", print_buffer);

		memcpy(buffer->real_addr_local, hmc->hmc_id, HMC_ID_LEN);
		/* RDMA over ID, send open msg, change state to ibmhmc_state_opening */
		rc = send_open(buffer, hmc);
		break;
	case VMC_IOCTL_DEBUG:
		count_hmc_buffers(hmc->index, &cnt_buffers[0], &cnt_buffers[1]);
		info(LOG_LEVEL_TRACE, "ioctl: debug - alloc buffs 0x%x, "
				"avail buffs 0x%x\n", cnt_buffers[0], cnt_buffers[1]);
		n = copy_to_user((void *) arg, cnt_buffers, 8);
		if (!n) {
			return -EFAULT;
		}
		break;
	default:
		warn("ibmvmc: unknown ioctl 0x%x\n", cmd);
		return -EINVAL;
	}

	return rc;
}

static struct file_operations ibmvmc_fops = {
	.owner		= THIS_MODULE,
	.read		= ibmvmc_read,
	.write		= ibmvmc_write,
	.poll		= ibmvmc_poll,
	.unlocked_ioctl	= ibmvmc_ioctl,
	.open           = ibmvmc_open,
	.release        = ibmvmc_close,
};


static int ibmvmc_add_buffer(struct crq_server_adapter *adapter, struct crq_msg_ibmvmc *crq)
{
	int rc = 0;
	u8 hmc_index, hmc_session;
	u16 buffer_id;
	struct ibmvmc_buffer *buffer;
	unsigned long flags;

	if(crq == NULL) return -1;

	hmc_session = crq->hmc_session;
	hmc_index = crq->hmc_index;
	buffer_id = be16_to_cpu(crq->var2.buffer_id);

	if(hmc_index > ibmvmc.max_hmc_index) {
		err("add_buffer: invalid hmc_index = 0x%x\n", hmc_index);
		ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_INVALID_HMC_INDEX,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}

	if(buffer_id >= ibmvmc.max_buffer_pool_size) {
		err("add_buffer: invalid buffer_id = 0x%x\n", buffer_id);
		ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_INVALID_BUFFER_ID,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}

	spin_lock_irqsave(&(hmcs[hmc_index].lock), flags);
	buffer = &(hmcs[hmc_index].buffer[buffer_id]);

	if(buffer->real_addr_local || buffer->dma_addr_local) {
		warn("add_buffer: buffer already allocated for buffer_id = 0x%lx\n",
		     (unsigned long) buffer_id);
		spin_unlock_irqrestore(&(hmcs[hmc_index].lock), flags);
		ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_INVALID_BUFFER_ID,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}

	buffer->real_addr_local = alloc_dma_buffer(to_vio_dev(adapter->dev),
						       ibmvmc.max_mtu,
						       &(buffer->dma_addr_local));

	if(buffer->real_addr_local == NULL) {
		err("add_buffer: alloc_dma_buffer failed.\n");
		spin_unlock_irqrestore(&(hmcs[hmc_index].lock), flags);
		ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_INTERFACE_FAILURE,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}

	buffer->dma_addr_remote = be32_to_cpu(crq->var3.lioba);
	buffer->size = ibmvmc.max_mtu;
	buffer->owner = crq->var1.owner;
	buffer->free = 1;
	mb();
	buffer->valid = 1;
	buffer->id = buffer_id;

	info(LOG_LEVEL_TRACE, "add_buffer: successfully added a buffer:\n");
	info(LOG_LEVEL_TRACE, "            index: %d, session: %d, buffer: 0x%x, "
			"owner: %d\n",
			hmc_index, hmc_session, buffer_id, buffer->owner);
	info(LOG_LEVEL_TRACE, "            local: 0x%x, remote: 0x%x\n",
			(u32)buffer->dma_addr_local, (u32)buffer->dma_addr_remote);
	spin_unlock_irqrestore(&(hmcs[hmc_index].lock), flags);

	ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_SUCCESS, hmc_session,
				    hmc_index, buffer_id);

	return rc;
}

/*
 * The hypervisor requested that we pick an unused buffer, and return it.  Before
 * sending the buffer back, we free any storage associated with the buffer.
 */
static int ibmvmc_rem_buffer(struct crq_server_adapter *adapter, struct crq_msg_ibmvmc *crq)
{
	int rc = 0;
	u8 hmc_index, hmc_session;
	u16 buffer_id = 0;
	struct ibmvmc_buffer *buffer;
	unsigned long flags;

	if(crq == NULL) return -1;

	hmc_session = crq->hmc_session;
	hmc_index = crq->hmc_index;

	if(hmc_index > ibmvmc.max_hmc_index) {
		warn("rem_buffer: invalid hmc_index = 0x%x\n", hmc_index);
		ibmvmc_send_rem_buffer_resp(adapter, VMC_MSG_INVALID_HMC_INDEX,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}

	spin_lock_irqsave(&(hmcs[hmc_index].lock), flags);
	buffer = get_free_hmc_buffer_locked(hmc_index);
	if(buffer == NULL) {
		info(LOG_LEVEL_NORM, "rem_buffer: unable to find buffer to remove\n");
		spin_unlock_irqrestore(&(hmcs[hmc_index].lock), flags);
		ibmvmc_send_rem_buffer_resp(adapter, VMC_MSG_NO_BUFFER, hmc_session,
					    hmc_index, VMC_INVALID_BUFFER_ID);
		return -1;
	}

	buffer_id = buffer->id;

	if(buffer->valid) {
		free_dma_buffer(to_vio_dev(adapter->dev),
				    ibmvmc.max_mtu,
				    buffer->real_addr_local,
				    buffer->dma_addr_local);
	}
	memset(buffer, 0, sizeof(struct ibmvmc_buffer));
	spin_unlock_irqrestore(&(hmcs[hmc_index].lock), flags);

	info(LOG_LEVEL_TRACE, "rem_buffer: removed buffer 0x%x.\n", buffer_id);
	ibmvmc_send_rem_buffer_resp(adapter, VMC_MSG_SUCCESS, hmc_session,
				    hmc_index, buffer_id);

	return rc;
}

static int recv_msg(struct crq_server_adapter *adapter, struct crq_msg_ibmvmc *crq)
{
	int rc = 0;
	u8 hmc_index, hmc_session;
	u16 buffer_id;
	struct ibmvmc_buffer *buffer;
	unsigned long msg_len;
	struct ibmvmc_hmc *hmc;
	unsigned long flags;

	if(crq == NULL) return -1;

	/* Hypervisor writes CRQs directly into our memory in big endian */
	info(LOG_LEVEL_TRACE, "recv_msg: msg from HV 0x%016llx 0x%016llx\n",
			be64_to_cpu(*((unsigned long *)crq)),
			be64_to_cpu(*(((unsigned long *)crq)+1)));

	hmc_session = crq->hmc_session;
	hmc_index = crq->hmc_index;
	buffer_id = be16_to_cpu(crq->var2.buffer_id);
	msg_len = be32_to_cpu(crq->var3.msg_len);

	if(hmc_index > ibmvmc.max_hmc_index) {
		err("recv_msg: invalid hmc_index = 0x%x\n", hmc_index);
		ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_INVALID_HMC_INDEX,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}

	if(buffer_id >= ibmvmc.max_buffer_pool_size) {
		err("recv_msg: invalid buffer_id = 0x%x\n", buffer_id);
		ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_INVALID_BUFFER_ID,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}

	hmc = &hmcs[hmc_index];
	spin_lock_irqsave(&(hmc->lock), flags);

	if(hmc->state == ibmhmc_state_free) {
		err("recv_msg: invalid hmc state = 0x%x\n", hmc->state);
		/* HMC connection is not valid (possibly was reset under us). */
		spin_unlock_irqrestore(&(hmc->lock), flags);
		return -1;
	}

	buffer = &(hmc->buffer[buffer_id]);

	if((buffer->valid == 0) || buffer->owner == VMC_BUF_OWNER_ALPHA) {
		err("recv_msg: buffer not valid, or not owned by HV.  0x%x 0x%x\n",
		    buffer->valid, buffer->owner);
		spin_unlock_irqrestore(&(hmc->lock), flags);
		return -1;
	}

	/* RDMA the data into the partition. */
	rc = h_copy_rdma(msg_len,
			 adapter->riobn,
			 buffer->dma_addr_remote,
			 adapter->liobn,
			 buffer->dma_addr_local);

	info(LOG_LEVEL_TRACE, "recv_msg: msg_len = 0x%x, buffer_id = 0x%x, "
			"queue_head = 0x%x, hmc_idx = 0x%x\n", (unsigned int)msg_len,
			(unsigned int)buffer_id, (unsigned int)hmc->queue_head,
			(unsigned int)hmc_index);
	buffer->msg_len = msg_len;
	buffer->free = 0;
	buffer->owner = VMC_BUF_OWNER_ALPHA;

	if(rc) {
		err("failure in recv_msg: h_copy_rdma = 0x%x\n", rc);
		spin_unlock_irqrestore(&(hmc->lock), flags);
		return -1;
	}

	/* Must be locked because read operates on the same data */
	hmc->queue_outbound_msgs[hmc->queue_head] = buffer_id;
	hmc->queue_head++;
	if(hmc->queue_head == ibmvmc_max_buf_pool_size) {
		hmc->queue_head = 0;
	}
	if(hmc->queue_head == hmc->queue_tail) {
		err("recv_msg: outbound buffer queue wrapped.\n");
	}

	spin_unlock_irqrestore(&(hmc->lock), flags);

	wake_up_interruptible(&ibmvmc_read_wait);
	return 0;
}

static void ibmvmc_process_capabilities(struct crq_msg_ibmvmc *crqp)
{
	struct crq_msg_ibmvmc_admin *crq = (struct crq_msg_ibmvmc_admin *)crqp;

	if((be16_to_cpu(crq->version) >> 8) != (IBMVMC_PROTOCOL_VERSION >> 8)) {
		err("ibmvmc: init failed, incompatible versions (0x%x 0x%x)\n",
		    be16_to_cpu(crq->version), IBMVMC_PROTOCOL_VERSION);
		ibmvmc.state = ibmvmc_state_failed;
		return;
	}

	ibmvmc.max_mtu = min((u32) ibmvmc_max_mtu, be32_to_cpu(crq->max_mtu));
	ibmvmc.max_buffer_pool_size = min((u16) ibmvmc_max_buf_pool_size,
						(u16) be16_to_cpu(crq->pool_size));
	ibmvmc.max_hmc_index = min((u8) ibmvmc_max_hmcs, crq->max_hmc) - 1;
	ibmvmc.state = ibmvmc_state_ready;

	info(LOG_LEVEL_NORM, "capabilites: max_mtu=0x%x, max_buf_pool_size=0x%x, "
			"max_hmc_index=0x%x\n", ibmvmc.max_mtu,
			ibmvmc.max_buffer_pool_size, ibmvmc.max_hmc_index);
}

static int ibmvmc_validate_hmc_session(struct crq_msg_ibmvmc *crq)
{
	unsigned char hmc_index;

	hmc_index = crq->hmc_index;

	if(crq->hmc_session == 0) {
		return 0;
	}

	if(hmc_index > ibmvmc.max_hmc_index) {
		return -1;
	}

	if(hmcs[hmc_index].session != crq->hmc_session) {
		warn("message session value is incorrect (expected 0x%x, "
				"recv 0x%x - msg dropped\n", hmcs[hmc_index].session,
				crq->hmc_session);
		return -1;
	}

	return 0;
}

static void ibmvmc_reset(void)
{
	int i;

	info(LOG_LEVEL_NORM, "*** Reset to initial state.\n");
	for(i=0; i<ibmvmc_max_hmcs; i++) {
		if(hmcs[i].state != ibmhmc_state_free) {
			ibmvmc_return_hmc(&hmcs[i]);
		}
	}
	ibmvmc.state = ibmvmc_state_crqinit;
}

static void ibmvmc_process_open_resp(struct crq_msg_ibmvmc *crq)
{
	unsigned char hmc_index;
	unsigned short buffer_id;

	hmc_index = crq->hmc_index;
	if(hmc_index > ibmvmc.max_hmc_index) {
		/* Why would PHYP give an index > max negotiated? */
		ibmvmc_reset();
	}

	if(crq->status) {
		warn("open_resp: failed - status 0x%x\n", crq->status);
		ibmvmc_return_hmc(&hmcs[hmc_index]);
		return;
	}

	if(hmcs[hmc_index].state == ibmhmc_state_opening) {
		buffer_id = be16_to_cpu(crq->var2.buffer_id);
		if(buffer_id >= ibmvmc.max_buffer_pool_size) {
			err("process_open_resp: invalid buffer_id = 0x%x\n",
					buffer_id);
			hmcs[hmc_index].state = ibmhmc_state_failed;
		} else {
			return_hmc_buffer(&(hmcs[hmc_index]), &(hmcs[hmc_index].buffer[buffer_id]));
			hmcs[hmc_index].state = ibmhmc_state_ready;
			info(LOG_LEVEL_TRACE, "open_resp: set hmc state = "
					"ready (0x%x)\n",
					hmcs[hmc_index].state);
		}
	} else {
		warn("open_resp: invalid hmc state (0x%x)\n",
				hmcs[hmc_index].state);
	}
}

/*
 * If the close fails, simply reset the entire driver as the state of the VMC
 * must be in tough shape.
 */
static void ibmvmc_process_close_resp(struct crq_msg_ibmvmc *crq)
{
	unsigned char hmc_index;

	hmc_index = crq->hmc_index;
	if(hmc_index > ibmvmc.max_hmc_index) {
		ibmvmc_reset();
		return;
	}

	if(crq->status) {
		warn("close_resp: failed - status 0x%x\n", crq->status);
		ibmvmc_reset();
		return;
	}

	ibmvmc_return_hmc(&hmcs[hmc_index]);
}

static void ibmvmc_crq_process(struct crq_server_adapter *adapter, struct crq_msg_ibmvmc *crq)
{
	switch (crq->type) {
	case VMC_MSG_CAP_RESP:
		info(LOG_LEVEL_TRACE, "CRQ recv: capabilities resp (0x%x)\n", crq->type);
		if(ibmvmc.state == ibmvmc_state_capabilities) {
			ibmvmc_process_capabilities(crq);
		} else {
			warn("capabilities msg invalid in state 0x%x\n", ibmvmc.state);
		}
		break;
	case VMC_MSG_OPEN_RESP:
		info(LOG_LEVEL_TRACE, "CRQ recv: open resp (0x%x)\n", crq->type);
		if(ibmvmc_validate_hmc_session(crq) == 0)
			ibmvmc_process_open_resp(crq);
		break;
	case VMC_MSG_ADD_BUF:
		info(LOG_LEVEL_TRACE, "CRQ recv: add buf (0x%x)\n", crq->type);
		if(ibmvmc_validate_hmc_session(crq) == 0)
			ibmvmc_add_buffer(adapter, crq);
		break;
	case VMC_MSG_REM_BUF:
		info(LOG_LEVEL_TRACE, "CRQ recv: rem buf (0x%x)\n", crq->type);
		if(ibmvmc_validate_hmc_session(crq) == 0)
			ibmvmc_rem_buffer(adapter, crq);
		break;
	case VMC_MSG_SIGNAL:
		info(LOG_LEVEL_TRACE, "CRQ recv: signal msg (0x%x)\n", crq->type);
		if(ibmvmc_validate_hmc_session(crq) == 0)
			recv_msg(adapter, crq);
		break;
	case VMC_MSG_CLOSE_RESP:
		info(LOG_LEVEL_TRACE, "CRQ recv: close resp (0x%x)\n", crq->type);
		if(ibmvmc_validate_hmc_session(crq) == 0)
			ibmvmc_process_close_resp(crq);
		break;
	case VMC_MSG_CAP:
	case VMC_MSG_OPEN:
	case VMC_MSG_CLOSE:
	case VMC_MSG_ADD_BUF_RESP:
	case VMC_MSG_REM_BUF_RESP:
		warn("CRQ recv: unexpected msg (0x%x)\n", crq->type);
		break;
	default:
		warn("CRQ recv: unknown msg (0x%x)\n", crq->type);
		break;
	}
}

static void ibmvmc_handle_crq(struct crq_msg_ibmvmc *crq, struct crq_server_adapter *adapter)
{
	switch (crq->valid) {
	case 0xC0:		/* initialization */
		switch (crq->type) {
		case 0x01:	/* Initialization message */
			info(LOG_LEVEL_TRACE, "CRQ recv: CRQ init msg - state 0x%x\n",
					ibmvmc.state);
			if(ibmvmc.state == ibmvmc_state_crqinit) {
				/* Send back a response */
				if (ibmvmc_send_crq(adapter, 0xC002000000000000, 0) == 0) {
					ibmvmc_send_capabilities(adapter);
				} else {
					err("Unable to send init rsp\n");
				}
			} else {
				err("Invalid state 0x%x mtu = 0x%x\n", ibmvmc.state,
						ibmvmc.max_mtu);
			}

			break;
		case 0x02:	/* Initialization response */
			info(LOG_LEVEL_TRACE, "CRQ recv: initialization resp msg - "
					"state 0x%x\n", ibmvmc.state);
			if(ibmvmc.state == ibmvmc_state_crqinit) {
				ibmvmc_send_capabilities(adapter);
			}
			break;
		default:
			warn("Unknown crq message type 0x%lx\n",
			    (unsigned long) crq->type);
		}
		break;
	case 0xFF:		/* Hypervisor telling us the connection is closed */
		warn("CRQ recv: virtual adapter failed - resetting.\n");
		ibmvmc_reset();
		break;
	case 0x80:		/* real payload */
		ibmvmc_crq_process(adapter, crq);
		break;
	default:
		warn("CRQ recv: unknown msg 0x%02x.\n", crq->valid);
		break;
	}
}

static void ibmvmc_task(struct work_struct *work)
{
	struct crq_server_adapter *adapter = container_of(work, struct crq_server_adapter, work);
	struct vio_dev *vdev = to_vio_dev(adapter->dev);
	struct crq_msg_ibmvmc *crq;
	int done = 0;

	while (!done) {
		/* Pull all the valid messages off the CRQ */
		while ((crq = crq_queue_next_crq(&adapter->queue)) != NULL) {
			ibmvmc_handle_crq(crq, adapter);
			crq->valid = 0x00;
		}

		vio_enable_interrupts(vdev);
		crq = crq_queue_next_crq(&adapter->queue);
		if (crq != NULL) {
			vio_disable_interrupts(vdev);
			ibmvmc_handle_crq(crq, adapter);
			crq->valid = 0x00;
		} else {
			done = 1;
		}
	}
}

static int ibmvmc_init_crq_queue(struct crq_server_adapter *adapter)
{
	int rc;
	int retrc;
	struct vio_dev *vdev = to_vio_dev(adapter->dev);
	struct crq_queue *queue = &adapter->queue;

	queue->msgs = (struct crq_msg_ibmvmc *)get_zeroed_page(GFP_KERNEL);

	if (!queue->msgs) {
		goto malloc_failed;
	}
	queue->size = PAGE_SIZE / sizeof(*queue->msgs);

	queue->msg_token = dma_map_single(adapter->dev, queue->msgs,
					    queue->size * sizeof(*queue->msgs),
					    DMA_BIDIRECTIONAL);

	if (dma_mapping_error(adapter->dev, queue->msg_token)) {
		goto map_failed;
	}

	retrc = rc = plpar_hcall_norets(H_REG_CRQ,
					vdev->unit_address,
					queue->msg_token, PAGE_SIZE);
	if (rc == H_RESOURCE) {
		rc = ibmvmc_reset_crq_queue(adapter);
	}

	if (rc == 2) {
		warn("Partner adapter not ready\n");
		retrc = 0;
	} else if (rc != 0) {
		err("Error %d opening adapter\n", rc);
		goto reg_crq_failed;
	}

	queue->cur = 0;
	spin_lock_init(&queue->lock);

	adapter->work_queue = create_singlethread_workqueue(ibmvmc_workq_name);
	if (adapter->work_queue == NULL) {
		err("couldn't allocate work queue\n");
		goto create_workqueue_failed;
	}

	INIT_WORK(&adapter->work, ibmvmc_task);

	if (request_irq(vdev->irq,
			ibmvmc_handle_event,
			0, "ibmvmc", (void *)adapter) != 0) {
		err("couldn't register irq 0x%x\n",
				vdev->irq);
		goto req_irq_failed;
	}

	rc = vio_enable_interrupts(vdev);
	if (rc != 0) {
		err("Error %d enabling interrupts!!!\n", rc);
		goto req_irq_failed;
	}

	return retrc;

req_irq_failed:
	/* Cannot have any work since we either never got our IRQ registered,
	 * or never got interrupts enabled */
	destroy_workqueue(adapter->work_queue);
create_workqueue_failed:
	h_free_crq(vdev->unit_address);
reg_crq_failed:
	dma_unmap_single(adapter->dev,
			queue->msg_token,
			queue->size * sizeof(*queue->msgs), DMA_BIDIRECTIONAL);
map_failed:
	free_page((unsigned long)queue->msgs);
malloc_failed:
	return -1;
}

/* Fill in the liobn and riobn fields on the adapter */
static int read_dma_window(struct vio_dev *vdev,
				struct crq_server_adapter *adapter)
{
	const __be32 *dma_window;
	const __be32 *prop;

	/* TODO Using of_parse_dma_window would be better, but it doesn't give
	 * a way to read multiple windows without already knowing the size of
	 * a window or the number of windows */
	dma_window =
		(const __be32 *)vio_get_attribute(vdev, "ibm,my-dma-window",
						NULL);
	if(!dma_window) {
		err("Couldn't find ibm,my-dma-window property\n");
		return -1;
	}

	adapter->liobn = be32_to_cpu(*dma_window);
	dma_window++;

	prop = (const __be32 *)vio_get_attribute(vdev, "ibm,#dma-address-cells",
						NULL);
	if (!prop) {
		warn("Couldn't find ibm,#dma-address-cells property\n");
		dma_window++;
	} else {
		dma_window += be32_to_cpu(*prop);
	}

	prop = (const __be32 *)vio_get_attribute(vdev, "ibm,#dma-size-cells",
						NULL);
	if (!prop) {
		warn("Couldn't find ibm,#dma-size-cells property\n");
		dma_window++;
	} else {
		dma_window += be32_to_cpu(*prop);
	}

	// dma_window should point to the second window now
	adapter->riobn = be32_to_cpu(*dma_window);

	return 0;
}

static int ibmvmc_probe(struct vio_dev *vdev, const struct vio_device_id *id)
{
	struct crq_server_adapter *adapter = &ibmvmc_adapter;
	int rc;

	info(LOG_LEVEL_NORM, "Probe for UA 0x%x\n", vdev->unit_address);

	dev_set_drvdata(&vdev->dev, NULL);
	memset(adapter, 0, sizeof(*adapter));
	adapter->dev = &vdev->dev;
	sprintf(adapter->name,"%s:%x", ibmvmc_driver_name, vdev->unit_address);

	rc = read_dma_window(vdev, adapter);
	if (rc != 0) {
		ibmvmc.state = ibmvmc_state_failed;
		return -1;
	}

	info(LOG_LEVEL_TRACE, "Probe: liobn 0x%x, riobn 0x%x\n", adapter->liobn,
			adapter->riobn);

	rc = ibmvmc_init_crq_queue(adapter);
	if (rc != 0) {
		err("Error initializing CRQ.  rc = 0x%x\n", rc);
		ibmvmc.state = ibmvmc_state_failed;
		return -1;
	}

	ibmvmc.state = ibmvmc_state_crqinit;

	/* Try to send an initialization message.  Note that this is allowed
	 * to fail if the other end is not acive.  In that case we just wait
	 * for the other side to initialize.
	 */
	if (ibmvmc_send_crq(adapter, 0xC001000000000000LL, 0) != 0
			&& rc != H_RESOURCE) {
		warn("Failed to send initialize CRQ message\n");
	}

	dev_set_drvdata(&vdev->dev, adapter);

	return 0;
}

static int ibmvmc_remove(struct vio_dev *vdev)
{
	struct crq_server_adapter *adapter = dev_get_drvdata(&vdev->dev);
	info(LOG_LEVEL_NORM, "entering remove for UA 0x%x\n", vdev->unit_address);
	ibmvmc_release_crq_queue(adapter);

	return 0;
}

static struct vio_device_id ibmvmc_device_table[] = {
	{ "ibm,vmc", "IBM,vmc" },
	{ "", "" }
};

MODULE_DEVICE_TABLE(vio, ibmvmc_device_table);

static struct vio_driver ibmvmc_driver = {
	.name        = ibmvmc_driver_name,
	.id_table    = ibmvmc_device_table,
	.probe       = ibmvmc_probe,
	.remove      = ibmvmc_remove,
};

static void __init ibmvmc_scrub_module_parms(void)
{
	/* don't care about log_level */

	if(ibmvmc_max_mtu > MAX_MTU) {
		warn("Max MTU reduced to %d\n", MAX_MTU);
		ibmvmc_max_mtu = MAX_MTU;
	} else if(ibmvmc_max_mtu < MIN_MTU) {
		warn("Max MTU increased to %d\n", MIN_MTU);
		ibmvmc_max_mtu = MIN_MTU;
	}

	if(ibmvmc_max_buf_pool_size > MAX_BUF_POOL_SIZE) {
		warn("Max buffer pool size reduced to %d\n", MAX_BUF_POOL_SIZE);
		ibmvmc_max_buf_pool_size = MAX_BUF_POOL_SIZE;
	} else if(ibmvmc_max_buf_pool_size < MIN_BUF_POOL_SIZE) {
		warn("Max buffer pool size increased to %d\n", MIN_BUF_POOL_SIZE);
		ibmvmc_max_buf_pool_size = MIN_BUF_POOL_SIZE;
	}

	if(ibmvmc_max_hmcs > MAX_HMCS) {
		warn("Max HMCs reduced to %d\n", MAX_HMCS);
		ibmvmc_max_hmcs = MAX_HMCS;
	} else if(ibmvmc_max_hmcs < MIN_HMCS) {
		warn("Max HMCs increased to %d\n", MIN_HMCS);
		ibmvmc_max_hmcs = MIN_HMCS;
	}
}

static int __init ibmvmc_module_init(void)
{
	int rc, i, j;

	ibmvmc.state = ibmvmc_state_initial;
	info(LOG_LEVEL_MIN, "ibmvmc version %s\n", IBMVMC_DRIVER_VERSION);

	/* Dynamically allocate major number */
        if (alloc_chrdev_region(&ibmvmc_chrdev, 0, 1, ibmvmc_driver_name)) {
                err("unable to allocate a dev_t\n");
                rc = -EIO;
		goto alloc_chrdev_failed;
        }
	info(LOG_LEVEL_NORM, "ibmvmc node %d:%d\n", MAJOR(ibmvmc_chrdev),
			MINOR(ibmvmc_chrdev));

	/* Initialize data structures */
	memset(hmcs, 0, sizeof(struct ibmvmc_hmc) * MAX_HMCS);
	for(i=0; i<MAX_HMCS; i++) {
		spin_lock_init(&hmcs[i].lock);
		hmcs[i].state = ibmhmc_state_free;
		for(j=0; j<MAX_BUF_POOL_SIZE; j++) {
			hmcs[i].queue_outbound_msgs[j] = VMC_INVALID_BUFFER_ID;
		}
	}

	/* Sanity check module parms */
	ibmvmc_scrub_module_parms();

	/*
	 * Initialize some reasonable values.  Might be negotiated smaller values
	 * during the capabilities exchange.
	 */
	ibmvmc.max_mtu = ibmvmc_max_mtu;
	ibmvmc.max_buffer_pool_size = ibmvmc_max_buf_pool_size;
	ibmvmc.max_hmc_index = ibmvmc_max_hmcs - 1;

	/* Once cdev_add is complete, apps can start trying to use the vmc.
	 * They will get EBUSY on open until after the probe has completed.
	 */
	cdev_init(&ibmvmc.cdev, &ibmvmc_fops);
	ibmvmc.cdev.owner = THIS_MODULE;
	ibmvmc.cdev.ops = &ibmvmc_fops;
	rc = cdev_add(&ibmvmc.cdev, ibmvmc_chrdev, 1);
	if (rc) {
		err("unable to add cdev: %d\n", rc);
		goto cdev_add_failed;
	}

	rc = vio_register_driver(&ibmvmc_driver);

	if (rc) {
		err("rc %d from vio_register_driver\n", rc);
		goto vio_reg_failed;
	}

	return 0;

vio_reg_failed:
	cdev_del(&ibmvmc.cdev);
cdev_add_failed:
	unregister_chrdev_region(ibmvmc_chrdev, 1);
alloc_chrdev_failed:
	return rc;
}

static void __exit ibmvmc_module_exit(void)
{
	info(LOG_LEVEL_NORM, "ibmvmc_module_exit\n");
	vio_unregister_driver(&ibmvmc_driver);
	cdev_del(&ibmvmc.cdev);
	unregister_chrdev_region(ibmvmc_chrdev, 1);
}

module_init(ibmvmc_module_init);
module_exit(ibmvmc_module_exit);

