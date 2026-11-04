/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NVMe controller side definitions (NVM Express Base Specification 1.4,
 * NVMe over PCIe Transport) used by the NVMe endpoint function.
 */

#ifndef ZEPHYR_DRIVERS_PCIE_ENDPOINT_FUNCTIONS_PCIE_EPF_NVME_H_
#define ZEPHYR_DRIVERS_PCIE_ENDPOINT_FUNCTIONS_PCIE_EPF_NVME_H_

#include <stdint.h>
#include <zephyr/sys/util.h>

/* Controller registers (BAR0) */
#define NVME_REG_CAP     0x0000
#define NVME_REG_VS      0x0008
#define NVME_REG_INTMS   0x000c
#define NVME_REG_INTMC   0x0010
#define NVME_REG_CC      0x0014
#define NVME_REG_CSTS    0x001c
#define NVME_REG_NSSR    0x0020
#define NVME_REG_AQA     0x0024
#define NVME_REG_ASQ     0x0028
#define NVME_REG_ACQ     0x0030
#define NVME_REG_CMBLOC  0x0038
#define NVME_REG_CMBSZ   0x003c
#define NVME_REG_DBS     0x1000

/* Doorbell stride 4 B (CAP.DSTRD = 0) */
#define NVME_SQ_DB(qid) (NVME_REG_DBS + (qid) * 8U)
#define NVME_CQ_DB(qid) (NVME_REG_DBS + (qid) * 8U + 4U)

/* CAP */
#define NVME_CAP_MQES(x)   ((uint64_t)(x) & 0xffffU)
#define NVME_CAP_CQR       BIT64(16)
#define NVME_CAP_TO(x)     (((uint64_t)(x) & 0xffU) << 24)
#define NVME_CAP_CSS_NVM   BIT64(37)
#define NVME_CAP_MPSMIN(x) (((uint64_t)(x) & 0xfU) << 48)
#define NVME_CAP_MPSMAX(x) (((uint64_t)(x) & 0xfU) << 52)

#define NVME_VS(maj, min, ter) (((maj) << 16) | ((min) << 8) | (ter))

/* CC */
#define NVME_CC_EN          BIT(0)
#define NVME_CC_CSS(cc)     (((cc) >> 4) & 0x7U)
#define NVME_CC_MPS(cc)     (((cc) >> 7) & 0xfU)
#define NVME_CC_AMS(cc)     (((cc) >> 11) & 0x7U)
#define NVME_CC_SHN(cc)     (((cc) >> 14) & 0x3U)
#define NVME_CC_IOSQES(cc)  (((cc) >> 16) & 0xfU)
#define NVME_CC_IOCQES(cc)  (((cc) >> 20) & 0xfU)

/* CSTS */
#define NVME_CSTS_RDY           BIT(0)
#define NVME_CSTS_CFS           BIT(1)
#define NVME_CSTS_SHST_MASK     (3U << 2)
#define NVME_CSTS_SHST_OCCUR    (1U << 2)
#define NVME_CSTS_SHST_CMPLT    (2U << 2)

/* AQA */
#define NVME_AQA_ASQS(aqa) ((aqa) & 0xfffU)
#define NVME_AQA_ACQS(aqa) (((aqa) >> 16) & 0xfffU)

/* Queue entry sizes as log2 */
#define NVME_SQES 6
#define NVME_CQES 4

/* Submission queue entry: 16 dwords, little endian */
struct nvme_sqe {
	uint32_t dw[16];
};

#define NVME_SQE_OPC(e)   ((e)->dw[0] & 0xffU)
#define NVME_SQE_FUSE(e)  (((e)->dw[0] >> 8) & 0x3U)
#define NVME_SQE_PSDT(e)  (((e)->dw[0] >> 14) & 0x3U)
#define NVME_SQE_CID(e)   ((uint16_t)((e)->dw[0] >> 16))
#define NVME_SQE_NSID(e)  ((e)->dw[1])
#define NVME_SQE_PRP1(e)  ((uint64_t)(e)->dw[6] | ((uint64_t)(e)->dw[7] << 32))
#define NVME_SQE_PRP2(e)  ((uint64_t)(e)->dw[8] | ((uint64_t)(e)->dw[9] << 32))
#define NVME_SQE_CDW(e, n) ((e)->dw[(n)])

/* Completion queue entry */
struct nvme_cqe {
	uint32_t dw0;
	uint32_t dw1;
	uint16_t sqhd;
	uint16_t sqid;
	uint16_t cid;
	/* bit 0 phase tag, 15:1 status field */
	uint16_t status;
};

BUILD_ASSERT(sizeof(struct nvme_sqe) == 64);
BUILD_ASSERT(sizeof(struct nvme_cqe) == 16);

/*
 * Status field without the phase tag: SC 7:0, SCT 10:8, CRD 12:11, M 13,
 * DNR 14. Stored shifted left by one in the CQE.
 */
#define NVME_SCT_GENERIC 0x0U
#define NVME_SCT_CMD     0x1U
#define NVME_SCT_MEDIA   0x2U
#define NVME_STATUS(sct, sc) ((uint16_t)(((sct) << 8) | (sc)))
#define NVME_STATUS_DNR      BIT(14)

#define NVME_SC_SUCCESS            NVME_STATUS(NVME_SCT_GENERIC, 0x00)
#define NVME_SC_INVALID_OPCODE     NVME_STATUS(NVME_SCT_GENERIC, 0x01)
#define NVME_SC_INVALID_FIELD      NVME_STATUS(NVME_SCT_GENERIC, 0x02)
#define NVME_SC_DATA_XFER_ERROR    NVME_STATUS(NVME_SCT_GENERIC, 0x04)
#define NVME_SC_INTERNAL           NVME_STATUS(NVME_SCT_GENERIC, 0x06)
#define NVME_SC_ABORT_SQ_DELETED   NVME_STATUS(NVME_SCT_GENERIC, 0x08)
#define NVME_SC_INVALID_NS         NVME_STATUS(NVME_SCT_GENERIC, 0x0b)
#define NVME_SC_PRP_INVALID_OFFSET NVME_STATUS(NVME_SCT_GENERIC, 0x13)
#define NVME_SC_LBA_RANGE          NVME_STATUS(NVME_SCT_GENERIC, 0x80)
#define NVME_SC_NS_NOT_READY       NVME_STATUS(NVME_SCT_GENERIC, 0x82)

#define NVME_SC_CQ_INVALID         NVME_STATUS(NVME_SCT_CMD, 0x00)
#define NVME_SC_QID_INVALID        NVME_STATUS(NVME_SCT_CMD, 0x01)
#define NVME_SC_QUEUE_SIZE         NVME_STATUS(NVME_SCT_CMD, 0x02)
#define NVME_SC_ABORT_LIMIT        NVME_STATUS(NVME_SCT_CMD, 0x03)
#define NVME_SC_AER_LIMIT          NVME_STATUS(NVME_SCT_CMD, 0x05)
#define NVME_SC_INVALID_VECTOR     NVME_STATUS(NVME_SCT_CMD, 0x08)
#define NVME_SC_INVALID_LOG_PAGE   NVME_STATUS(NVME_SCT_CMD, 0x09)
#define NVME_SC_QUEUE_DELETION     NVME_STATUS(NVME_SCT_CMD, 0x0c)
#define NVME_SC_FEAT_NOT_SAVEABLE  NVME_STATUS(NVME_SCT_CMD, 0x0d)
#define NVME_SC_FEAT_NOT_CHANGEABLE NVME_STATUS(NVME_SCT_CMD, 0x0e)

#define NVME_SC_WRITE_FAULT        NVME_STATUS(NVME_SCT_MEDIA, 0x80)
#define NVME_SC_READ_ERROR         NVME_STATUS(NVME_SCT_MEDIA, 0x81)

/* Admin opcodes */
#define NVME_ADM_DELETE_SQ   0x00
#define NVME_ADM_CREATE_SQ   0x01
#define NVME_ADM_GET_LOG     0x02
#define NVME_ADM_DELETE_CQ   0x04
#define NVME_ADM_CREATE_CQ   0x05
#define NVME_ADM_IDENTIFY    0x06
#define NVME_ADM_ABORT       0x08
#define NVME_ADM_SET_FEAT    0x09
#define NVME_ADM_GET_FEAT    0x0a
#define NVME_ADM_AER         0x0c
#define NVME_ADM_KEEP_ALIVE  0x18

/* NVM opcodes */
#define NVME_CMD_FLUSH        0x00
#define NVME_CMD_WRITE        0x01
#define NVME_CMD_READ         0x02
#define NVME_CMD_WRITE_ZEROES 0x08
#define NVME_CMD_DSM          0x09

/* Identify CNS */
#define NVME_CNS_NS             0x00
#define NVME_CNS_CTRL           0x01
#define NVME_CNS_NS_ACTIVE_LIST 0x02
#define NVME_CNS_NS_DESC_LIST   0x03
#define NVME_CNS_CS_NS          0x05
#define NVME_CNS_CS_CTRL        0x06

#define NVME_CSI_NVM 0x00

/* Namespace identification descriptor types */
#define NVME_NIDT_EUI64 0x01
#define NVME_NIDT_NGUID 0x02
#define NVME_NIDT_CSI   0x04

/* Log pages */
#define NVME_LOG_ERROR    0x01
#define NVME_LOG_SMART    0x02
#define NVME_LOG_FW_SLOT  0x03

/* Features */
#define NVME_FEAT_ARBITRATION   0x01
#define NVME_FEAT_POWER_MGMT    0x02
#define NVME_FEAT_TEMP_THRESH   0x04
#define NVME_FEAT_ERR_RECOVERY  0x05
#define NVME_FEAT_VOLATILE_WC   0x06
#define NVME_FEAT_NUM_QUEUES    0x07
#define NVME_FEAT_IRQ_COALESCE  0x08
#define NVME_FEAT_IRQ_CONFIG    0x09
#define NVME_FEAT_WRITE_ATOMIC  0x0a
#define NVME_FEAT_ASYNC_EVENT   0x0b

/* Identify Controller byte offsets */
#define NVME_IDC_VID     0
#define NVME_IDC_SSVID   2
#define NVME_IDC_SN      4
#define NVME_IDC_MN      24
#define NVME_IDC_FR      64
#define NVME_IDC_RAB     72
#define NVME_IDC_MDTS    77
#define NVME_IDC_CNTLID  78
#define NVME_IDC_VER     80
#define NVME_IDC_CNTRLTYPE 111
#define NVME_IDC_OACS    256
#define NVME_IDC_ACL     258
#define NVME_IDC_AERL    259
#define NVME_IDC_FRMW    260
#define NVME_IDC_LPA     261
#define NVME_IDC_ELPE    262
#define NVME_IDC_NPSS    263
#define NVME_IDC_WCTEMP  266
#define NVME_IDC_CCTEMP  268
#define NVME_IDC_SQES    512
#define NVME_IDC_CQES    513
#define NVME_IDC_MAXCMD  514
#define NVME_IDC_NN      516
#define NVME_IDC_ONCS    520
#define NVME_IDC_VWC     525
#define NVME_IDC_SGLS    536
#define NVME_IDC_SUBNQN  768
#define NVME_IDC_PSD0    2048

#define NVME_ONCS_WRITE_ZEROES BIT(3)

/* Identify Namespace byte offsets */
#define NVME_IDN_NSZE    0
#define NVME_IDN_NCAP    8
#define NVME_IDN_NUSE    16
#define NVME_IDN_NLBAF   25
#define NVME_IDN_FLBAS   26
#define NVME_IDN_NGUID   104
#define NVME_IDN_EUI64   120
#define NVME_IDN_LBAF0   128

/* SMART / Health log byte offsets */
#define NVME_SMART_TEMP           1
#define NVME_SMART_AVAIL_SPARE    3
#define NVME_SMART_SPARE_THRESH   4
#define NVME_SMART_DATA_READ      32
#define NVME_SMART_DATA_WRITTEN   48
#define NVME_SMART_HOST_READS     64
#define NVME_SMART_HOST_WRITES    80
#define NVME_SMART_POWER_CYCLES   112
#define NVME_SMART_POWER_ON_HOURS 128
#define NVME_SMART_MEDIA_ERRORS   160

#define NVME_IDENTIFY_SIZE 4096
#define NVME_LOG_SMART_SIZE 512

#endif /* ZEPHYR_DRIVERS_PCIE_ENDPOINT_FUNCTIONS_PCIE_EPF_NVME_H_ */
