/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __DT_BINDINGS_INTERCONNECT_QCOM_KALAMA_H
#define __DT_BINDINGS_INTERCONNECT_QCOM_KALAMA_H

#define MASTER_GPU_TCU				0
#define MASTER_SYS_TCU				1
#define MASTER_APPSS_PROC				2
#define MASTER_LLCC				3
#define MASTER_GIC_AHB				4
#define MASTER_QDSS_BAM				5
#define MASTER_QSPI_0				6
#define MASTER_QUP_1				7
#define MASTER_QUP_2				8
#define MASTER_A1NOC_SNOC				9
#define MASTER_A2NOC_SNOC				10
#define MASTER_CAMNOC_HF				11
#define MASTER_CAMNOC_ICP				12
#define MASTER_CAMNOC_SF				13
#define MASTER_GEM_NOC_CNOC				14
#define MASTER_GEM_NOC_PCIE_SNOC				15
#define MASTER_GFX3D				16
#define MASTER_LPASS_GEM_NOC				17
#define MASTER_LPASS_LPINOC				18
#define MASTER_LPIAON_NOC				19
#define MASTER_MDP				20
#define MASTER_MSS_PROC				21
#define MASTER_MNOC_HF_MEM_NOC				22
#define MASTER_MNOC_SF_MEM_NOC				23
#define MASTER_COMPUTE_NOC				24
#define MASTER_ANOC_PCIE_GEM_NOC				25
#define MASTER_SNOC_GC_MEM_NOC				26
#define MASTER_SNOC_SF_MEM_NOC				27
#define MASTER_CDSP_HCP				28
#define MASTER_VIDEO				29
#define MASTER_VIDEO_CV_PROC				30
#define MASTER_VIDEO_PROC				31
#define MASTER_VIDEO_V_PROC				32
#define MASTER_CNOC_CFG				33
#define MASTER_CNOC_MNOC_CFG				34
#define MASTER_PCIE_ANOC_CFG				35
#define MASTER_QUP_CORE_0				36
#define MASTER_QUP_CORE_1				37
#define MASTER_QUP_CORE_2				38
#define MASTER_CRYPTO				39
#define MASTER_IPA				40
#define MASTER_LPASS_PROC				41
#define MASTER_CDSP_PROC				42
#define MASTER_SP				43
#define MASTER_GIC				44
#define MASTER_PCIE_0				45
#define MASTER_PCIE_1				46
#define MASTER_QDSS_ETR				47
#define MASTER_QDSS_ETR_1				48
#define MASTER_SDCC_2				49
#define MASTER_SDCC_4				50
#define MASTER_UFS_MEM				51
#define MASTER_USB3_0				52
#define SLAVE_EBI1				512
#define SLAVE_AHB2PHY_SOUTH				513
#define SLAVE_AHB2PHY_NORTH				514
#define SLAVE_AOSS				515
#define SLAVE_APPSS				516
#define SLAVE_CAMERA_CFG				517
#define SLAVE_CLK_CTL				518
#define SLAVE_RBCPR_CX_CFG				519
#define SLAVE_RBCPR_MMCX_CFG				520
#define SLAVE_RBCPR_MXA_CFG				521
#define SLAVE_RBCPR_MXC_CFG				522
#define SLAVE_CPR_NSPCX				523
#define SLAVE_CRYPTO_0_CFG				524
#define SLAVE_CX_RDPM				525
#define SLAVE_DISPLAY_CFG				526
#define SLAVE_GFX3D_CFG				527
#define SLAVE_I2C				528
#define SLAVE_IMEM_CFG				529
#define SLAVE_IPA_CFG				530
#define SLAVE_IPC_ROUTER_CFG				531
#define SLAVE_CNOC_MSS				532
#define SLAVE_MX_RDPM				533
#define SLAVE_PCIE_0_CFG				534
#define SLAVE_PCIE_1_CFG				535
#define SLAVE_PDM				536
#define SLAVE_PIMEM_CFG				537
#define SLAVE_PRNG				538
#define SLAVE_QDSS_CFG				539
#define SLAVE_QSPI_0				540
#define SLAVE_QUP_1				541
#define SLAVE_QUP_2				542
#define SLAVE_SDCC_2				543
#define SLAVE_SDCC_4				544
#define SLAVE_SPSS_CFG				545
#define SLAVE_TCSR				546
#define SLAVE_TLMM				547
#define SLAVE_TME_CFG				548
#define SLAVE_UFS_MEM_CFG				549
#define SLAVE_USB3_0				550
#define SLAVE_VENUS_CFG				551
#define SLAVE_VSENSE_CTRL_CFG				552
#define SLAVE_A1NOC_SNOC				553
#define SLAVE_A2NOC_SNOC				554
#define SLAVE_GEM_NOC_CNOC				555
#define SLAVE_SNOC_GEM_NOC_GC				556
#define SLAVE_SNOC_GEM_NOC_SF				557
#define SLAVE_LLCC				558
#define SLAVE_LPASS_GEM_NOC				559
#define SLAVE_LPIAON_NOC_LPASS_AG_NOC				560
#define SLAVE_LPICX_NOC_LPIAON_NOC				561
#define SLAVE_MNOC_HF_MEM_NOC				562
#define SLAVE_MNOC_SF_MEM_NOC				563
#define SLAVE_CDSP_MEM_NOC				564
#define SLAVE_MEM_NOC_PCIE_SNOC				565
#define SLAVE_ANOC_PCIE_GEM_NOC				566
#define SLAVE_CNOC_CFG				567
#define SLAVE_DDRSS_CFG				568
#define SLAVE_LPASS_QTB_CFG				569
#define SLAVE_CNOC_MNOC_CFG				570
#define SLAVE_NSP_QTB_CFG				571
#define SLAVE_PCIE_ANOC_CFG				572
#define SLAVE_QUP_CORE_0				573
#define SLAVE_QUP_CORE_1				574
#define SLAVE_QUP_CORE_2				575
#define SLAVE_BOOT_IMEM				576
#define SLAVE_IMEM				577
#define SLAVE_SERVICE_MNOC				578
#define SLAVE_SERVICE_PCIE_ANOC				579
#define SLAVE_PCIE_0				580
#define SLAVE_PCIE_1				581
#define SLAVE_QDSS_STM				582
#define SLAVE_TCU				583
#define MASTER_LLCC_DISP				1000
#define MASTER_MDP_DISP				1001
#define MASTER_MNOC_HF_MEM_NOC_DISP				1002
#define MASTER_ANOC_PCIE_GEM_NOC_DISP				1003
#define SLAVE_EBI1_DISP				1512
#define SLAVE_LLCC_DISP				1513
#define SLAVE_MNOC_HF_MEM_NOC_DISP				1514
#define MASTER_LLCC_CAM_IFE_0				2000
#define MASTER_CAMNOC_HF_CAM_IFE_0				2001
#define MASTER_CAMNOC_ICP_CAM_IFE_0				2002
#define MASTER_CAMNOC_SF_CAM_IFE_0				2003
#define MASTER_MNOC_HF_MEM_NOC_CAM_IFE_0				2004
#define MASTER_MNOC_SF_MEM_NOC_CAM_IFE_0				2005
#define MASTER_ANOC_PCIE_GEM_NOC_CAM_IFE_0				2006
#define SLAVE_EBI1_CAM_IFE_0				2512
#define SLAVE_LLCC_CAM_IFE_0				2513
#define SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_0				2514
#define SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_0				2515
#define MASTER_LLCC_CAM_IFE_1				3000
#define MASTER_CAMNOC_HF_CAM_IFE_1				3001
#define MASTER_CAMNOC_ICP_CAM_IFE_1				3002
#define MASTER_CAMNOC_SF_CAM_IFE_1				3003
#define MASTER_MNOC_HF_MEM_NOC_CAM_IFE_1				3004
#define MASTER_MNOC_SF_MEM_NOC_CAM_IFE_1				3005
#define MASTER_ANOC_PCIE_GEM_NOC_CAM_IFE_1				3006
#define SLAVE_EBI1_CAM_IFE_1				3512
#define SLAVE_LLCC_CAM_IFE_1				3513
#define SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_1				3514
#define SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_1				3515
#define MASTER_LLCC_CAM_IFE_2				4000
#define MASTER_CAMNOC_HF_CAM_IFE_2				4001
#define MASTER_CAMNOC_ICP_CAM_IFE_2				4002
#define MASTER_CAMNOC_SF_CAM_IFE_2				4003
#define MASTER_MNOC_HF_MEM_NOC_CAM_IFE_2				4004
#define MASTER_MNOC_SF_MEM_NOC_CAM_IFE_2				4005
#define MASTER_ANOC_PCIE_GEM_NOC_CAM_IFE_2				4006
#define SLAVE_EBI1_CAM_IFE_2				4512
#define SLAVE_LLCC_CAM_IFE_2				4513
#define SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_2				4514
#define SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_2				4515

#endif
