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

#include <linux/utsname.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/genhd.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/percpu.h>
#include <linux/devfs_fs_kernel.h>

#include <asm/processor.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/vio.h>

#define IBMVMC_DRIVER_VERSION "1.0"

MODULE_DESCRIPTION("IBM VMC");
MODULE_AUTHOR("Steven Royer <seroyer@us.ibm.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(IBMVMC_DRIVER_VERSION);

#define BU_MODE

#define err(format, arg...) printk(KERN_ERR __FILE__ ": " format , ## arg)
#define info(format, arg...) printk(KERN_INFO __FILE__ ": " format  , ## arg)
#define warn(format, arg...) printk(KERN_WARNING __FILE__ ": " format , ## arg)

/*
 * Static global variables
 */
static DECLARE_WAIT_QUEUE_HEAD(vmc_read_wait);

char ibmvmc_driver_name[] = "ibmvmc";
#define IBMVMC_VERSION    0x0101

#define MAX_BUF_POOL_SIZE 64
#define MAX_HMC_INDEX     2
#define MAX_MTU           4*4096
#define HMC_ID_LEN        32

#define VMC_INVALID_BUFFER_ID 0xFFFF

#define VMC_IOCTL_SETHMCID 1
#define VMC_IOCTL_DEBUG    2

#define VMC_MSG_CAP          0x01
#define VMC_MSG_CAP_RESP     0x81
#define VMC_MSG_OPEN         0x02
#define VMC_MSG_OPEN_RESP    0x82
#define VMC_MSG_CLOSE        0x03
#define VMC_MSG_CLOSE_RESP   0x83
#define VMC_MSG_ADD_BUF      0x04
#define VMC_MSG_ADD_BUF_RESP 0x84
#define VMC_MSG_REM_BUF      0x05
#define VMC_MSG_REM_BUF_RESP 0x85
#define VMC_MSG_SIGNAL       0x06

#define VMC_MSG_SUCCESS 0
#define VMC_MSG_INVALID_HMC_INDEX 1
#define VMC_MSG_INVALID_BUFFER_ID 2
#define VMC_MSG_CLOSED_HMC        3
#define VMC_MSG_INTERFACE_FAILURE 4
#define VMC_MSG_NO_BUFFER         5

#define VMC_BUF_OWNER_ALPHA 0
#define VMC_BUF_OWNER_HV    1

enum ibmvmc_states {
	ibmvmc_state_initial =      0,
	ibmvmc_state_crqinit =      1,
	ibmvmc_state_capabilities = 2,
	ibmvmc_state_ready        = 3,
	ibmvmc_state_failed       = 4,
};

enum ibmhmc_states {
	ibmhmc_state_free =    0, /* HMC connection not established                 */
	ibmhmc_state_initial = 1, /* HMC connection established, due to open() call */
	ibmhmc_state_opening = 2, /* open msg sent to HV, due to ioctl(1) call      */
	ibmhmc_state_ready   = 3, /* HMC connection ready, open resp msg from HV    */
	ibmhmc_state_failed  = 4, /* HMC connection failure                         */
};

struct ibmvmc_buffer {
	unsigned char valid;    /* 1 when DMA storage allocated to buffer          */
	unsigned char free;     /* 1 when buffer available for the Alpha Partition */
	unsigned char owner;
	unsigned short id;
	unsigned long size;
	unsigned long msg_len;
	dma_addr_t dma_addr_local;
	dma_addr_t dma_addr_remote;
	void *real_addr_local;
};

struct ibmvmc {
	u32 state;
	u32 max_mtu;
	u32 max_buffer_pool_size;
	u32 max_hmc_index;
	struct crq_server_adapter *adapter;
	spinlock_t lock;
	struct ibmvmc_buffer buffer[MAX_BUF_POOL_SIZE];
} ibmvmc;

struct ibmvmc_hmc {
	u8 session;
	u8 index;
	u32 state;
	struct file *file;
	struct crq_server_adapter *adapter;
	spinlock_t lock;
	unsigned char hmc_id[HMC_ID_LEN];
	struct ibmvmc_buffer buffer[MAX_BUF_POOL_SIZE];
	unsigned short queue_outbound_msgs[MAX_BUF_POOL_SIZE];
	int queue_head, queue_tail;
} hmcs[MAX_HMC_INDEX];

struct crq_msg_ibmvmc_admin {
	u8 valid;     /* RPA Defined           */
	u8 type;      /* ibmvmc msg type       */
	u8 status;    /* Response msg status   */
	u8 rsvd[2];
	u8 max_hmc;
	u16 pool_size;
	u32 max_mtu;
	u16 crq_size;
	u16 version;
};

struct crq_msg_ibmvmc {
	unsigned char valid;     /* RPA Defined           */
	unsigned char type;      /* ibmvmc msg type       */
	unsigned char status;    /* Response msg status   */
	union {
		unsigned char rsvd;  /* Reserved              */
		unsigned char owner;
	} var1;
	unsigned char hmc_session;
	unsigned char hmc_index;
	union {
		unsigned short rsvd;
		unsigned short buffer_id;
	} var2;
	unsigned int rsvd;
	union {
		unsigned int rsvd;
		unsigned int lioba;
		unsigned int msg_len;
	} var3;
};

extern long h_copy_rdma(long length,
			unsigned long sliobn, unsigned long slioba,
			unsigned long dliobn, unsigned long dlioba);

struct crq_server_adapter ibmvmc_adapter;

#if 0
static struct ibmvmc_buffer * get_free_channel_buffer(void)
{
	unsigned long i;
	struct ibmvmc_buffer *buffer;
	struct ibmvmc_buffer *ret_buf = NULL;

	buffer = ibmvmc.buffer;
	spin_lock(&(ibmvmc.lock));

	for(i=0; i<MAX_BUF_POOL_SIZE; i++) {
		if((buffer[i].valid) && (buffer[i].free) &&
		   (buffer[i].owner == VMC_BUF_OWNER_ALPHA)) {
			buffer[i].free = 0;
			ret_buf = &(buffer[i]);
			break;
		}
	}

	spin_unlock(&(ibmvmc.lock));
	return ret_buf;
}

static void return_channel_buffer(struct ibmvmc_buffer *buffer)
{
	spin_lock(&(ibmvmc.lock));
	buffer->free = 1;
	spin_unlock(&(ibmvmc.lock));
}
#endif

static struct ibmvmc_buffer * get_valid_hmc_buffer_locked(u8 hmc_index)
{
	unsigned long i;
	struct ibmvmc_buffer *buffer;
	struct ibmvmc_buffer *ret_buf = NULL;

	if(hmc_index > ibmvmc.max_hmc_index)
		return NULL;

	buffer = hmcs[hmc_index].buffer;

	for(i=0; i<MAX_BUF_POOL_SIZE; i++) {
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
		info("get_free_hmc_buffer: invalid hmc_index = 0x%x\n", hmc_index);
		return NULL;
	}

	buffer = hmcs[hmc_index].buffer;

	for(i=0; i<MAX_BUF_POOL_SIZE; i++) {
		if((buffer[i].free) && (buffer[i].owner == VMC_BUF_OWNER_ALPHA)) {
			buffer[i].free = 0;
			ret_buf = &(buffer[i]);
			break;
		}
	}

	return ret_buf;
}

static void return_hmc_buffer(struct ibmvmc_hmc *hmc, struct ibmvmc_buffer *buffer)
{
	spin_lock(&(hmc->lock));
	buffer->free = 1;
	spin_unlock(&(hmc->lock));
}

static void count_hmc_buffers(u8 hmc_index, unsigned int *valid, unsigned int *free)
{
	unsigned long i;
	struct ibmvmc_buffer *buffer;

	if(hmc_index > MAX_HMC_INDEX)
		return;

	if(valid == NULL || free == NULL)
		return;

	*valid = 0; *free = 0;

	if(hmc_index == 0xFF) {
		buffer = ibmvmc.buffer;
	} else {
		buffer = hmcs[hmc_index].buffer;
	}
	spin_lock(&(hmcs[hmc_index].lock));

	for(i=0; i<MAX_BUF_POOL_SIZE; i++) {
		if(buffer[i].valid) {
			*valid = *valid + 1;
			if(buffer[i].free) {
				*free = *free + 1;
			}
		}
	}

	spin_unlock(&(hmcs[hmc_index].lock));
}

static struct ibmvmc_hmc * ibmvmc_get_free_hmc(void)
{
	unsigned long i;

	/*
	 * Find an available HMC connection.
	 */
	for(i = 0; i <= ibmvmc.max_hmc_index; i++) {
		spin_lock(&(hmcs[i].lock));
		if(hmcs[i].state == ibmhmc_state_free) {
			hmcs[i].index = i;
			hmcs[i].state = ibmhmc_state_initial;
			spin_unlock(&(hmcs[i].lock));
			return(&hmcs[i]);
		}
		spin_unlock(&(hmcs[i].lock));
	}

	return NULL;
}

static int ibmvmc_return_hmc(struct ibmvmc_hmc *hmc)
{
	unsigned long i;
	struct ibmvmc_buffer *buffer;
	struct crq_server_adapter *adapter;

	if((hmc == NULL) || (hmc->adapter == NULL))
		return -EIO;

	adapter = hmc->adapter;

	spin_lock(&(hmc->lock));
	hmc->index = 0;
	hmc->state = ibmhmc_state_free;
	hmc->queue_head = 0;
	hmc->queue_tail = 0;
	hmc->file = NULL;
	buffer = hmc->buffer;
	for(i=0; i<MAX_BUF_POOL_SIZE; i++) {
		if(buffer[i].valid) {
			vio_free_consistent(adapter->dev,
					    ibmvmc.max_mtu,
					    buffer[i].real_addr_local,
					    buffer[i].dma_addr_local);
		}
		memset(&buffer[i], 0, sizeof(struct ibmvmc_buffer));

		hmc->queue_outbound_msgs[i] = VMC_INVALID_BUFFER_ID;
	}

	spin_unlock(&(hmc->lock));

	return 0;
}

static int send_open(struct ibmvmc_buffer *buffer,
		     struct ibmvmc_hmc *hmc)
{
	int rc = 0;
	struct crq_msg_ibmvmc crq_msg;
	struct crq_server_adapter *adapter;

	if((hmc == NULL) || (hmc->adapter == NULL))
		return -EIO;

	adapter = hmc->adapter;

	info("ibmvmc send_open: 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx\n",
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
	crq_msg.var2.buffer_id = buffer->id;
	crq_msg.rsvd = 0;
	crq_msg.var3.rsvd = 0;

	crq_send(adapter, *((unsigned long *)&crq_msg),
		 *(((unsigned long *)&crq_msg)+1));

	return rc;
}

static int send_close(struct ibmvmc_hmc *hmc)
{
	int rc = 0;
	struct crq_msg_ibmvmc crq_msg;
	struct crq_server_adapter *adapter;

	info("CRQ send: close\n");

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

	crq_send(adapter, *((unsigned long *)&crq_msg),
		 *(((unsigned long *)&crq_msg)+1));

	return rc;
}

int ibmvmc_send_capabilities(struct crq_server_adapter *adapter)
{
	struct crq_msg_ibmvmc_admin crq_msg;

	info("CRQ send: capabilities\n");
	crq_msg.valid = 0x80;
	crq_msg.type = VMC_MSG_CAP;
	crq_msg.status = 0;
	crq_msg.rsvd[0] = 0;
	crq_msg.rsvd[1] = 0;
	crq_msg.max_hmc = MAX_HMC_INDEX;
	crq_msg.max_mtu = MAX_MTU;
	crq_msg.pool_size = MAX_BUF_POOL_SIZE;
	crq_msg.crq_size = adapter->queue.size;
	crq_msg.version = IBMVMC_VERSION;

	crq_send(adapter, *((unsigned long *)&crq_msg),
		 *(((unsigned long *)&crq_msg)+1));

	ibmvmc.state = ibmvmc_state_capabilities;

	return 0;
}

int ibmvmc_send_add_buffer_resp(struct crq_server_adapter *adapter,
				u8 status, u8 hmc_session, u8 hmc_index, u16 buffer_id)
{
	struct crq_msg_ibmvmc crq_msg;

	info("CRQ send: add_buffer_resp\n");
	crq_msg.valid = 0x80;
	crq_msg.type = VMC_MSG_ADD_BUF_RESP;
	crq_msg.status = status;
	crq_msg.var1.rsvd = 0;
	crq_msg.hmc_session = hmc_session;
	crq_msg.hmc_index = hmc_index;
	crq_msg.var2.buffer_id = buffer_id;
	crq_msg.rsvd = 0;
	crq_msg.var3.rsvd = 0;

	crq_send(adapter,
		 *((unsigned long *)&crq_msg), *(((unsigned long *)&crq_msg)+1));

	return 0;
}

int ibmvmc_send_rem_buffer_resp(struct crq_server_adapter *adapter,
				u8 status, u8 hmc_session, u8 hmc_index, u16 buffer_id)
{
	struct crq_msg_ibmvmc crq_msg;

	info("CRQ send: rem_buffer_resp\n");
	crq_msg.valid = 0x80;
	crq_msg.type = VMC_MSG_REM_BUF_RESP;
	crq_msg.status = status;
	crq_msg.var1.rsvd = 0;
	crq_msg.hmc_session = hmc_session;
	crq_msg.hmc_index = hmc_index;
	crq_msg.var2.buffer_id = buffer_id;
	crq_msg.rsvd = 0;
	crq_msg.var3.rsvd = 0;

	crq_send(adapter,
		 *((unsigned long *)&crq_msg), *(((unsigned long *)&crq_msg)+1));

	return 0;
}

static int send_msg(struct crq_server_adapter *adapter,
		    struct ibmvmc_buffer *buffer,
		    struct ibmvmc_hmc *hmc, int msg_len)
{
	int rc = 0;
	struct crq_msg_ibmvmc crq_msg;

	info("CRQ send: rdma to HV\n");
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
	crq_msg.var2.buffer_id = buffer->id;
	crq_msg.var3.msg_len = msg_len;
	info("CRQ send: msg to HV 0x%lx 0x%lx\n",
	     *((unsigned long *)&crq_msg),
	     *(((unsigned long *)&crq_msg)+1));

	buffer->owner = VMC_BUF_OWNER_HV;
	crq_send(adapter, *((unsigned long *)&crq_msg),
		 *(((unsigned long *)&crq_msg)+1));

	return rc;
}

static int ibmvmc_open(struct inode *inode, struct file *file)
{
	int retval = 0;
	struct ibmvmc_hmc *hmc;
	unsigned int valid, free;

	info("vmc_open:  inode = 0x%lx, file = 0x%lx, state = 0x%x\n",
	     (unsigned long)inode, (unsigned long)file, ibmvmc.state);

	if(ibmvmc.state == ibmvmc_state_failed) {
		warn("vmc_open: state_failed\n");
		return -EIO;
	}

	/* Device is busy until capabilities have been exchanged and we
	 * have a generic buffer for each possible HMC connection.
	 */
	count_hmc_buffers(0xFF, &valid, &free);
	if(ibmvmc.state < ibmvmc_state_ready || valid < ibmvmc.max_hmc_index) {
		warn("vmc_open: not state_ready\n");
		return -EBUSY;
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

	info("close:  file = 0x%lx, state = 0x%x\n",
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

	rc = ibmvmc_return_hmc(hmc);

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

	info("read: file = 0x%lx, buf = 0x%lx, nbytes = 0x%lx\n",
	     (unsigned long)file, (unsigned long) buf, (unsigned long)nbytes);

	if (nbytes == 0) return 0;

	if(nbytes > ibmvmc.max_mtu) {
		info("read: nbytes invalid 0x%x\n", (unsigned int)nbytes);
		return -EINVAL;
	}

	hmc = file->private_data;
	if(!hmc) {
		info("read: no hmc\n");
		return -EIO;
	}

	adapter = hmc->adapter;
	if(!adapter) {
		info("read: no adapter\n");
		return -EIO;
	}


	do {
		spin_lock(&(hmc->lock));
		if(hmc->queue_tail != hmc->queue_head) {
			/* Data is available */
			break;
		}
		spin_unlock(&(hmc->lock));

                if (file->f_flags & O_NONBLOCK) {
                        retval = -EAGAIN;
                        goto out;
                }
                if (signal_pending(current)) {
                        retval = -ERESTARTSYS;
                        goto out;
                }
                set_current_state(TASK_INTERRUPTIBLE);
		add_wait_queue(&vmc_read_wait, &wait);
                schedule();
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&vmc_read_wait, &wait);
        } while (1);

	buffer = &(hmc->buffer[hmc->queue_outbound_msgs[hmc->queue_tail]]);
	hmc->queue_tail++;
	if(hmc->queue_tail == MAX_BUF_POOL_SIZE)
		hmc->queue_tail = 0;
	spin_unlock(&(hmc->lock));

	nbytes = min(nbytes, buffer->msg_len);
	n = copy_to_user((void *)buf, buffer->real_addr_local, nbytes);
	info("read: copy to user nbytes = 0x%lx.\n", nbytes);
	return_hmc_buffer(hmc, buffer);
	retval = nbytes;

	if (n) {
		info("read: copy to user failed.\n");
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

	poll_wait(file, &vmc_read_wait, wait);

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

	hmc = file->private_data;
	if(!hmc)
		return -EIO;

	spin_lock(&(hmc->lock));
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
		warn("ibmvmc_write: no buffer storage assigned\n");
		ret = -EIO;
		goto out;
	}
	buf = ibmvmc_buffer->real_addr_local;

	while (c > 0) {
		bytes = min(c, ibmvmc_buffer->size);

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

	info("write: file = 0x%lx, count = 0x%lx\n",
	     (unsigned long)file, (unsigned long)count);

	send_msg(adapter, ibmvmc_buffer, hmc, count);
	ret = p - buffer;
 out:
	spin_unlock(&(hmc->lock));
	return (ssize_t)(ret);
}

static int ibmvmc_ioctl(struct inode *inode, struct file *file,
	  unsigned int cmd, unsigned long arg)
{
	struct ibmvmc_buffer *buffer;
	size_t bytes;
	struct ibmvmc_hmc *hmc;
	int rc = 0;
	int cnt_buffers[2], n;

	info("ioctl: inode=0x%lx, file=0x%lx, cmd=0x%x, arg=0x%lx, hmc=0x%lx\n",
	     (unsigned long) inode, (unsigned long) file, cmd,
	     arg, (unsigned long) hmc);

	hmc = file->private_data;
	if(!hmc) {
		info("ioctl: no hmc\n");
		return -EIO;
	}

	if(hmc->state == ibmhmc_state_free || hmc->state == ibmhmc_state_failed) {
		return -EIO;
	}

	switch (cmd) {
	case VMC_IOCTL_SETHMCID:
		spin_lock(&(hmc->lock));
		buffer = get_valid_hmc_buffer_locked(hmc->index);
		spin_unlock(&(hmc->lock));
		// DRENG buffer = get_free_channel_buffer();

		if(buffer == NULL || (buffer->real_addr_local == NULL)) {
			info("ioctl: no buffer available\n");
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

		memcpy(buffer->real_addr_local, hmc->hmc_id, HMC_ID_LEN);
		/* RDMA over ID, send open msg, change state to ibmhmc_state_opening */
		rc = send_open(buffer, hmc);
		break;
	case VMC_IOCTL_DEBUG:
		count_hmc_buffers(hmc->index, &cnt_buffers[0], &cnt_buffers[1]);
		info("ioctl: debug - alloc buffs 0x%x, avail buffs 0x%x\n",
		     cnt_buffers[0], cnt_buffers[1]);
		n = copy_to_user((void *) arg, cnt_buffers, 8);
		if (!n) {
			return -EFAULT;
		}
		break;
	default:
		printk("ibmvmc: unknown ioctl 0x%x\n", cmd);
		return -EINVAL;
	}

	return rc;
}

struct file_operations ibmvmc_fops = {
	.read		= ibmvmc_read,
	.write		= ibmvmc_write,
	.poll		= ibmvmc_poll,
	.ioctl		= ibmvmc_ioctl,
	.open           = ibmvmc_open,
	.release        = ibmvmc_close,
};


int ibmvmc_add_buffer(struct crq_server_adapter *adapter, struct crq_msg *crqp)
{
	int rc = 0;
	struct crq_msg_ibmvmc *crq = (struct crq_msg_ibmvmc *)crqp;
	u8 hmc_index, hmc_session;
	u16 buffer_id;
	struct ibmvmc_buffer *buffer;

	if(crq == NULL) return -1;

	hmc_session = crq->hmc_session;
	hmc_index = crq->hmc_index;
	buffer_id = crq->var2.buffer_id;

	if(hmc_index > MAX_HMC_INDEX && hmc_index != 0xFF) {
		err("add_buffer: invalid hmc_index = 0x%x\n", hmc_index);
		ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_INVALID_HMC_INDEX,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}
#if 0
	// DRENG
	if(hmc_index <= MAX_HMC_INDEX && hmcs[hmc_index].state == ibmhmc_state_free) {
		info("add_buffer: hmc has been freed - dropping buffer\n");
		ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_CLOSED_HMC,
					    hmc_session, hmc_index, buffer_id);
		return 0;
	}
#endif
	if(buffer_id >= ibmvmc.max_buffer_pool_size) {
		err("add_buffer: invalid buffer_id = 0x%x\n", buffer_id);
		ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_INVALID_BUFFER_ID,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}

	if(hmc_index == 0xFF) {
		buffer = &(ibmvmc.buffer[buffer_id]);
	} else {
		buffer = &(hmcs[hmc_index].buffer[buffer_id]);
	}

	if(buffer->real_addr_local || buffer->dma_addr_local) {
		warn("add_buffer: buffer already allocated for buffer_id = 0x%lx\n",
		     (unsigned long) buffer_id);
		ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_INVALID_BUFFER_ID,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}

	buffer->real_addr_local = vio_alloc_consistent(adapter->dev,
						       ibmvmc.max_mtu,
						       &(buffer->dma_addr_local));

	if(buffer->real_addr_local == NULL) {
		err("add_buffer: vio_alloc_consistent failed.\n");
		ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_INTERFACE_FAILURE,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}

	buffer->dma_addr_remote = crq->var3.lioba;
	buffer->size = ibmvmc.max_mtu;
	buffer->owner = crq->var1.owner;
	buffer->free = 1;
	mb();
	buffer->valid = 1;
	buffer->id = buffer_id;

	info("add_buffer: successfully added a buffer.\n");
	ibmvmc_send_add_buffer_resp(adapter, VMC_MSG_SUCCESS, hmc_session,
				    hmc_index, buffer_id);

	return rc;
}

/*
 * The hypervisor requested that we pick an unused buffer, and return it.  Before
 * sending the buffer back, we free any storage associated with the buffer.
 */
int ibmvmc_rem_buffer(struct crq_server_adapter *adapter, struct crq_msg *crqp)
{
	int rc = 0;
	struct crq_msg_ibmvmc *crq = (struct crq_msg_ibmvmc *)crqp;
	u8 hmc_index, hmc_session;
	u16 buffer_id = 0;
	struct ibmvmc_buffer *buffer;

	if(crq == NULL) return -1;

	hmc_session = crq->hmc_session;
	hmc_index = crq->hmc_index;

	if(hmc_index > ibmvmc.max_hmc_index) {
		warn("rem_buffer: invalid hmc_index = 0x%x\n", hmc_index);
		ibmvmc_send_rem_buffer_resp(adapter, VMC_MSG_INVALID_HMC_INDEX,
					    hmc_session, hmc_index, buffer_id);
		return -1;
	}

	spin_lock(&(hmcs[hmc_index].lock));
	buffer = get_free_hmc_buffer_locked(hmc_index);
	if(buffer == NULL) {
		info("rem_buffer: unable to find buffer to remove\n");
		spin_unlock(&(hmcs[hmc_index].lock));
		ibmvmc_send_rem_buffer_resp(adapter, VMC_MSG_NO_BUFFER, hmc_session,
					    hmc_index, VMC_INVALID_BUFFER_ID);
		return -1;
	}

	buffer_id = buffer->id;

	if(buffer->valid) {
		vio_free_consistent(adapter->dev,
				    ibmvmc.max_mtu,
				    buffer->real_addr_local,
				    buffer->dma_addr_local);
	}
	memset(buffer, 0, sizeof(struct ibmvmc_buffer));
	spin_unlock(&(hmcs[hmc_index].lock));

	info("rem_buffer: removed buffer 0x%x.\n", buffer_id);
	ibmvmc_send_rem_buffer_resp(adapter, VMC_MSG_SUCCESS, hmc_session,
				    hmc_index, buffer_id);

	return rc;
}

static int recv_msg(struct crq_server_adapter *adapter, struct crq_msg *crqp)
{
	int rc = 0;
	struct crq_msg_ibmvmc *crq = (struct crq_msg_ibmvmc *)crqp;
	u8 hmc_index, hmc_session;
	u16 buffer_id;
	struct ibmvmc_buffer *buffer;
	unsigned long msg_len;
	struct ibmvmc_hmc *hmc;

	if(crq == NULL) return -1;

	hmc_session = crq->hmc_session;
	hmc_index = crq->hmc_index;
	buffer_id = crq->var2.buffer_id;
	msg_len = crq->var3.msg_len;

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
	spin_lock(&(hmc->lock));

	if(hmc->state == ibmhmc_state_free) {
		err("recv_msg: invalid hmc state = 0x%x\n", hmc->state);
		/* HMC connection is not valid (possibly was reset under us). */
		spin_unlock(&(hmc->lock));
		return -1;
	}

	buffer = &(hmc->buffer[buffer_id]);

	if((buffer->valid == 0) || buffer->owner == VMC_BUF_OWNER_ALPHA) {
		err("recv_msg: buffer not valid, or not owned by HV.  0x%x 0x%x\n",
		    buffer->valid, buffer->owner);
		spin_unlock(&(hmc->lock));
		return -1;
	}

	/* RDMA the data into the partition. */
	rc = h_copy_rdma(msg_len,
			 adapter->riobn,
			 buffer->dma_addr_remote,
			 adapter->liobn,
			 buffer->dma_addr_local);

	info("recv_msg: msg_len = 0x%x, buffer_id = 0x%x, queue_head = 0x%x, hmc_idx = 0x%x\n",
	     (unsigned int)msg_len, (unsigned int)buffer_id,
	     (unsigned int)hmc->queue_head, (unsigned int)hmc_index);
	info("recv_msg: msg from HV 0x%lx 0x%lx\n",
	     *((unsigned long *)crq),
	     *(((unsigned long *)crq)+1));
	buffer->msg_len = msg_len;
	buffer->free = 0;
	buffer->owner = VMC_BUF_OWNER_ALPHA;

	if(rc) {
		err("failure in recv_msg: h_copy_rdma = 0x%x\n", rc);
		spin_unlock(&(hmc->lock));
		return -1;
	}

	/* No lock needed as the function cannot be executed by 2 threads. */
	hmc->queue_outbound_msgs[hmc->queue_head] = buffer_id;
	hmc->queue_head++;
	if(hmc->queue_head == MAX_BUF_POOL_SIZE)
		hmc->queue_head = 0;
	if(hmc->queue_head == hmc->queue_tail)
		err("recv_msg: outbound buffer queue wrapped.\n");

	spin_unlock(&(hmc->lock));

	wake_up_interruptible(&vmc_read_wait);
	return 0;
}

void ibmvmc_process_capabilities(struct crq_msg *crqp)
{
	struct crq_msg_ibmvmc_admin *crq = (struct crq_msg_ibmvmc_admin *)crqp;

	if((crq->version >> 8) != (IBMVMC_VERSION >> 8)) {
		err("ibmvmc: init failed, incompatible versions (0x%x 0x%x)\n",
		    crq->version, IBMVMC_VERSION);
		ibmvmc.state = ibmvmc_state_failed;
		return;
	}

	ibmvmc.max_mtu = min((u32) MAX_MTU, crq->max_mtu);
	ibmvmc.max_buffer_pool_size = min((u16) MAX_BUF_POOL_SIZE,
						  crq->pool_size);
	ibmvmc.max_hmc_index = min((u8) MAX_HMC_INDEX, crq->max_hmc) - 1;
	ibmvmc.state = ibmvmc_state_ready;

	info("capabilites: max_mtu=0x%x, max_buf_pool_size=0x%x, max_hmc_index=0x%x\n",
	     ibmvmc.max_mtu, ibmvmc.max_buffer_pool_size,
	     ibmvmc.max_hmc_index);
}

static int ibmvmc_validate_hmc_session(struct crq_msg *crqp)
{
	struct crq_msg_ibmvmc *crq = (struct crq_msg_ibmvmc *)crqp;
	unsigned char hmc_index;

	hmc_index = crq->hmc_index;

	if(crq->hmc_session == 0) {
		return 0;
	}
#if 0
	// DRENG
	if(hmc_index == 0xFF)
		return 0;
#endif

	if(hmc_index > ibmvmc.max_hmc_index) {
		return -1;
	}

	if(hmcs[hmc_index].session != crq->hmc_session) {
		warn("message session value is incorrect (expected 0x%x, recv 0x%x - msg dropped\n", hmcs[hmc_index].session, crq->hmc_session);
		return -1;
	}

	return 0;
}

static void ibmvmc_reset(void)
{
	int i;

	info("*** Reset to initial state.\n");
	for(i=0; i<MAX_HMC_INDEX; i++) {
		if(hmcs[i].state != ibmhmc_state_free) {
			vmc_return_hmc(&hmcs[i]);
		}
	}
	ibmvmc.state = ibmvmc_state_crqinit;
}

void ibmvmc_process_open_resp(struct crq_msg *crqp)
{
	struct crq_msg_ibmvmc *crq = (struct crq_msg_ibmvmc *)crqp;
	unsigned char hmc_index;
	unsigned short buffer_id;

	hmc_index = crq->hmc_index;
	if(hmc_index > ibmvmc.max_hmc_index && hmc_index != 0xFF) {
		ibmvmc_reset();
	}

	if(crq->status) {
		warn("open_resp: failed - status 0x%x\n", crq->status);
		vmc_return_hmc(&hmcs[hmc_index]);
		return;
	} else {
		if(hmcs[hmc_index].state == ibmhmc_state_opening) {
			buffer_id = crq->var2.buffer_id;
			if(buffer_id >= ibmvmc.max_buffer_pool_size) {
				err("process_open_resp: invalid buffer_id = 0x%x\n",
				    buffer_id);
				hmcs[hmc_index].state = ibmhmc_state_failed;
			} else {
				return_hmc_buffer(&(hmcs[hmc_index]), &(hmcs[hmc_index].buffer[buffer_id]));
				// DRENG return_channel_buffer(&(ibmvmc.buffer[buffer_id]));
				hmcs[hmc_index].state = ibmhmc_state_ready;
				info("open_resp: set hmc state = ready (0x%x)\n",
				     hmcs[hmc_index].state);
			}
		} else {
			warn("open_resp: invalid hmc state (0x%x)\n",
			     hmcs[hmc_index].state);
		}

	}
}

/*
 * If the close fails, simply reset the entire driver as the state of the VMC
 * must be in tough shape.
 */
void ibmvmc_process_close_resp(struct crq_msg *crqp)
{
	struct crq_msg_ibmvmc_admin *crq = (struct crq_msg_ibmvmc_admin *)crqp;

	if(crq->status) {
		warn("close_resp: failed - status 0x%x\n", crq->status);
		ibmvmc_reset();
	}
}

void ibmvmc_crq_process(struct crq_server_adapter *adapter, struct crq_msg *crq)
{
	switch (crq->data[0]) {
	case VMC_MSG_CAP_RESP:
		info("CRQ recv: capabilities resp (0x%x)\n", crq->data[0]);
		if(ibmvmc.state == ibmvmc_state_capabilities) {
			ibmvmc_process_capabilities(crq);
		} else {
			warn("capabilities msg invalid in state 0x%x\n", ibmvmc.state);
		}
		break;
	case VMC_MSG_OPEN_RESP:
		info("CRQ recv: open resp (0x%x)\n", crq->data[0]);
		if(ibmvmc_validate_hmc_session(crq) == 0)
			ibmvmc_process_open_resp(crq);
		break;
	case VMC_MSG_ADD_BUF:
		info("CRQ recv: add buf (0x%x)\n", crq->data[0]);
		if(ibmvmc_validate_hmc_session(crq) == 0)
			ibmvmc_add_buffer(adapter, crq);
		break;
	case VMC_MSG_REM_BUF:
		info("CRQ recv: rem buf (0x%x)\n", crq->data[0]);
		if(ibmvmc_validate_hmc_session(crq) == 0)
			ibmvmc_rem_buffer(adapter, crq);
		break;
	case VMC_MSG_SIGNAL:
		info("CRQ recv: signal msg (0x%x)\n", crq->data[0]);
		if(ibmvmc_validate_hmc_session(crq) == 0)
			recv_msg(adapter, crq);
		break;
	case VMC_MSG_CLOSE_RESP:
		info("CRQ recv: close resp (0x%x)\n", crq->data[0]);
		if(ibmvmc_validate_hmc_session(crq) == 0)
			ibmvmc_process_close_resp(crq);
		break;
	case VMC_MSG_CAP:
	case VMC_MSG_OPEN:
	case VMC_MSG_CLOSE:
	case VMC_MSG_ADD_BUF_RESP:
	case VMC_MSG_REM_BUF_RESP:
		warn("CRQ recv: unexpected msg (0x%x)\n", crq->data[0]);
		break;
	default:
		warn("CRQ recv: unknown msg (0x%x)\n", crq->data[0]);
		break;
	}
}

void ibmvmc_crq_handle(struct crq_server_adapter *adapter, struct crq_msg *crq)
{
	switch (crq->valid) {
	case 0xC0:		/* initialization */
		switch (crq->data[0]) {
		case 0x01:	/* Initialization message */
			info("CRQ recv: CRQ init msg - state 0x%x\n", ibmvmc.state);
			if(ibmvmc.state == ibmvmc_state_crqinit) {
				/* Send back a response */
				if (crq_send(adapter, 0xC002000000000000, 0) == 0) {
					ibmvmc_send_capabilities(adapter);
				} else {
					err("Unable to send init rsp\n");
				}
			} else {
				err("Invalid state 0x%x mtu = 0x%x\n", ibmvmc.state, ibmvmc.max_mtu);
			}

			break;
		case 0x02:	/* Initialization response */
			info("CRQ recv: initialization resp msg\n");
			if(ibmvmc.state == ibmvmc_state_crqinit) {
				ibmvmc_send_capabilities(adapter);
			}
			break;
		default:
			err("Unknown crq message type 0x%lx\n",
			    (unsigned long) crq->data[0]);
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


static int ibmvmc_probe(struct vio_dev *dev, const struct vio_device_id *id)
{
	struct crq_server_adapter *adapter = &ibmvmc_adapter;
	int rc;
#ifndef BU_MODE
	unsigned int *dma_window;
	unsigned int dma_window_property_size;
#endif

	info("Probe for UA 0x%x\n", dev->unit_address);
	memset(adapter, 0, sizeof(*adapter));
	adapter->dev = dev;
	dev->driver_data = adapter;
	sprintf(adapter->name,"%s:%x", ibmvmc_driver_name, dev->unit_address);
	adapter->lock = SPIN_LOCK_UNLOCKED;

#ifndef BU_MODE
	dma_window =
		(unsigned int *)vio_get_attribute(dev, "ibm,my-dma-window",
						  &dma_window_property_size);
	if(!dma_window) {
		warn("Couldn't find ibm,my-dma-window property\n");
	}

	adapter->liobn = dma_window[0];
	adapter->riobn = dma_window[3];
#else
	adapter->liobn = 0x19000002;
	adapter->riobn = 0x13000002;
#endif

	adapter->queue.crq_handler = ibmvmc_crq_handle;
	INIT_WORK(&adapter->crq_work, crq_task, adapter);

	rc = crq_initialize(adapter);
	if (rc != 0) {
		err("Error initializing CRQ.  rc = 0x%x\n", rc);
		ibmvmc.state = ibmvmc_state_failed;
		return -1;
	}

	ibmvmc.state = ibmvmc_state_crqinit;

	rc = h_vio_signal(adapter->dev->unit_address, 1);
	if (rc != 0) {
		err("Error enabling interrupts.  rc = 0x%x\n", rc);
		ibmvmc.state = ibmvmc_state_failed;
		return -1;
	}

        rc = vio_enable_interrupts(adapter->dev);
        if (rc != 0) {
                err("Error enabling interrupts.  rc = 0x%x\n", rc);
		ibmvmc.state = ibmvmc_state_failed;
		return -1;
        }

        adapter->queue.cur = 0;
        adapter->queue.lock = SPIN_LOCK_UNLOCKED;

	return 0;
}

static int ibmvmc_remove(struct vio_dev *dev)
{
	info("entering remove for UA 0x%x\n", dev->unit_address);

	return 0;
}

static struct vio_device_id ibmvmc_device_table[] __devinitdata= {
	{ "vmc", "IBM,vmc" },
	{ 0,}
};

MODULE_DEVICE_TABLE(vio, ibmvmc_device_table);

static struct vio_driver ibmvmc_driver = {
	.name        = ibmvmc_driver_name,
	.id_table    = ibmvmc_device_table,
	.probe       = ibmvmc_probe,
	.remove      = ibmvmc_remove,
};

static int __init ibmvmc_module_init(void)
{
	int rc, i, j;

	info("ibmvmc version %d.%d mod_init\n",
	     IBMVMC_VERSION >> 8, IBMVMC_VERSION & 0xFF);

	rc = vio_register_driver(&ibmvmc_driver);

	if (rc) {
		warn("rc %d from vio_register_driver\n",rc);
	}

        if (register_chrdev (60, "ibmvmc", &vmc_fops)) {
                printk (KERN_WARNING "ibmvmc" ": unable to get major %d\n", 60);
                return -EIO;
        }
	devfs_mk_cdev(MKDEV(60, 0),
                      S_IFCHR | S_IRUSR | S_IWUSR, "ibmvmc");

	/* Initialize data structures */
	memset(hmcs, 0, sizeof(struct ibmvmc_hmc) * MAX_HMC_INDEX);
	for(i=0; i<MAX_HMC_INDEX; i++) {
		hmcs[i].lock = SPIN_LOCK_UNLOCKED;
		hmcs[i].state = ibmhmc_state_free;
		for(j=0; j<MAX_BUF_POOL_SIZE; j++) {
			hmcs[i].queue_outbound_msgs[j] = VMC_INVALID_BUFFER_ID;
		}
	}

	ibmvmc.lock = SPIN_LOCK_UNLOCKED;

	/*
	 * Initialize some reasonable values.  Might be negotiated smaller values
	 * during the capabilities exchange.
	 */
	ibmvmc.max_mtu = MAX_MTU;
	ibmvmc.max_buffer_pool_size = MAX_BUF_POOL_SIZE;
	ibmvmc.max_hmc_index = MAX_HMC_INDEX;

	return rc;
}

static __exit void ibmvmc_module_exit(void)
{
	info("ibmvmc_module_exit\n");
	vio_unregister_driver(&ibmvmc_driver);
}

module_init(ibmvmc_module_init);
module_exit(ibmvmc_module_exit);


