/* TODO - update this header
 * IBM PowerPC Virtual Communications Channel Support.
 *
 *    Copyright (c) 2014 IBM Corp.
 *     Steven Royer seroyer@us.ibm.com
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 */
#ifndef IBMVMC_H
#define IBMVMC_H

#include <linux/types.h>
#include <linux/workqueue.h>

#include <asm/vio.h>

#define IBMVMC_PROTOCOL_VERSION    0x0101

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

struct crq_msg {
	u8 valid;
	u8 type;
	u8 data[14];
};

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

struct crq_server_adapter;

/* an RPA command/response transport queue */
struct crq_queue {
	struct crq_msg_ibmvmc *msgs;
	int size, cur;
	dma_addr_t msg_token;
	spinlock_t lock;
	void (*crq_handler)(struct crq_server_adapter *adapter, struct crq_msg *crq);
};

struct crq_server_adapter
{
	char name[16];
	struct vio_dev *dev;
	struct crq_queue queue;
	spinlock_t lock;
	unsigned int liobn;
	unsigned int riobn;
	struct work_struct crq_work;
};

#endif
