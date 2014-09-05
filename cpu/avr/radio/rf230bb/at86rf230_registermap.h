/**
 * @file
 * @brief This file contains the common register definitions for all AT86RF23x.
 */
/*   Copyright (c) 2008, Swedish Institute of Computer Science
  All rights reserved.

  Additional fixes for AVR contributed by:

	Colin O'Flynn coflynn@newae.com
	Eric Gnoske egnoske@gmail.com
	Blake Leverett bleverett@gmail.com
	Mike Vidales mavida404@gmail.com
	Kevin Brown kbrown3@uccs.edu
	Nate Bohlmann nate@elfwerks.com
	Yannic Schröder schroeder@ibr.cs.tu-bs.de

   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PHY23x_REGISTERMAP_EXTERNAL_H
#define PHY23x_REGISTERMAP_EXTERNAL_H

#define HAVE_REGISTER_MAP (1)

/* ###########################################################################
 * # 0x01: TRX_STATUS                                                        #
 * ###########################################################################
 */

/** Offset for register TRX_STATUS */
#define RG_TRX_STATUS                    (0x01)

/** Access parameters for sub-register CCA_DONE in register @ref RG_TRX_STATUS */
#define SR_CCA_DONE                  0x01, 0x80, 7

/** Access parameters for sub-register CCA_STATUS in register @ref RG_TRX_STATUS */
#define SR_CCA_STATUS                0x01, 0x40, 6

#define SR_reserved_01_3             0x01, 0x20, 5

/** Access parameters for sub-register TRX_STATUS in register @ref RG_TRX_STATUS */
#define SR_TRX_STATUS                0x01, 0x1f, 0
/** Constant P_ON for sub-register @ref SR_TRX_STATUS */
#define P_ON                     (0x00)
/** Constant BUSY_RX for sub-register @ref SR_TRX_STATUS */
#define BUSY_RX                  (0x01)
/** Constant BUSY_TX for sub-register @ref SR_TRX_STATUS */
#define BUSY_TX                  (0x02)
/** Constant RX_ON for sub-register @ref SR_TRX_STATUS */
#define RX_ON                    (0x06)
/** Constant TRX_OFF for sub-register @ref SR_TRX_STATUS */
#define TRX_OFF                  (0x08)
/** Constant PLL_ON for sub-register @ref SR_TRX_STATUS */
#define PLL_ON                   (0x09)
/** Constant SLEEP for sub-register @ref SR_TRX_STATUS */
#define SLEEP                    (0x0F)
/** Constant PREP_DEEP_SLEEP for sub-register @ref SR_TRX_STATUS */
#define PREP_DEEP_SLEEP          (0x10)
/** Constant BUSY_RX_AACK for sub-register @ref SR_TRX_STATUS */
#define BUSY_RX_AACK             (0x11)
/** Constant BUSY_TX_ARET for sub-register @ref SR_TRX_STATUS */
#define BUSY_TX_ARET             (0x12)
/** Constant RX_AACK_ON for sub-register @ref SR_TRX_STATUS */
#define RX_AACK_ON               (0x16)
/** Constant TX_ARET_ON for sub-register @ref SR_TRX_STATUS */
#define TX_ARET_ON               (0x19)
/** Constant RX_ON_NOCLK for sub-register @ref SR_TRX_STATUS */
#define RX_ON_NOCLK              (0x1C)
/** Constant RX_AACK_ON_NOCLK for sub-register @ref SR_TRX_STATUS */
#define RX_AACK_ON_NOCLK         (0x1D)
/** Constant BUSY_RX_AACK_NOCLK for sub-register @ref SR_TRX_STATUS */
#define BUSY_RX_AACK_NOCLK       (0x1E)
/** Constant STATE_TRANSITION for sub-register @ref SR_TRX_STATUS */
#define STATE_TRANSITION         (0x1F)

/* ###########################################################################
 * # 0x02: TRX_STATE                                                         #
 * ###########################################################################
 */

/** Offset for register TRX_STATE */
#define RG_TRX_STATE                     (0x02)

/** Access parameters for sub-register TRAC_STATUS in register @ref RG_TRX_STATE */
#define SR_TRAC_STATUS               0x02, 0xe0, 5

/** Access parameters for sub-register TRX_CMD in register @ref RG_TRX_STATE */
#define SR_TRX_CMD                   0x02, 0x1f, 0
/** Constant CMD_NOP for sub-register @ref SR_TRX_CMD */
#define CMD_NOP                  (0x00)
/** Constant CMD_TX_START for sub-register @ref SR_TRX_CMD */
#define CMD_TX_START             (0x02)
/** Constant CMD_FORCE_TRX_OFF for sub-register @ref SR_TRX_CMD */
#define CMD_FORCE_TRX_OFF        (0x03)
/** Constant CMD_FORCE_PLL_ON for sub-register @ref SR_TRX_CMD */
#define CMD_FORCE_PLL_ON         (0x04)
/** Constant CMD_RX_ON for sub-register @ref SR_TRX_CMD */
#define CMD_RX_ON                (0x06)
/** Constant CMD_TRX_OFF for sub-register @ref SR_TRX_CMD */
#define CMD_TRX_OFF              (0x08)
/** Constant CMD_PLL_ON for sub-register @ref SR_TRX_CMD */
#define CMD_PLL_ON               (0x09)
/** Constant CMD_PREP_DEEP_SLEEP for sub-register @ref SR_TRX_CMD */
#define CMD_PREP_DEEP_SLEEP      (0x10)
/** Constant CMD_RX_AACK_ON for sub-register @ref SR_TRX_CMD */
#define CMD_RX_AACK_ON           (0x16)
/** Constant CMD_TX_ARET_ON for sub-register @ref SR_TRX_CMD */
#define CMD_TX_ARET_ON           (0x19)

/* ###########################################################################
 * # 0x03: TRX_CTRL_0                                                        #
 * ###########################################################################
 */

/** Offset for register TRX_CTRL_0 */
#define RG_TRX_CTRL_0                    (0x03)

/** Access parameters for sub-register PAD_IO in register @ref RG_TRX_CTRL_0 */
#define SR_PAD_IO                    0x03, 0xc0, 6

/** Access parameters for sub-register PAD_IO_CLKM in register @ref RG_TRX_CTRL_0 */
#define SR_PAD_IO_CLKM               0x03, 0x30, 4
/** Constant CLKM_2mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_2mA                 (0)
/** Constant CLKM_4mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_4mA                 (1)
/** Constant CLKM_6mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_6mA                 (2)
/** Constant CLKM_8mA for sub-register @ref SR_PAD_IO_CLKM */
#define CLKM_8mA                 (3)

/** Access parameters for sub-register TOM_EN in register @ref RG_TRX_CTRL_0 */
#define SR_TOM_EN                    0x03, 0x80, 7

/** Access parameters for sub-register PMU_EN in register @ref RG_TRX_CTRL_0 */
#define SR_PMU_EN                    0x03, 0x20, 5

/** Access parameters for sub-register PMU_IF_INVERSE in register @ref RG_TRX_CTRL_0 */
#define SR_PMU_IF_INVERSE            0x03, 0x10, 4

/** Access parameters for sub-register CLKM_SHA_SEL in register @ref RG_TRX_CTRL_0 */
#define SR_CLKM_SHA_SEL              0x03, 0x08, 3

/** Access parameters for sub-register CLKM_CTRL in register @ref RG_TRX_CTRL_0 */
#define SR_CLKM_CTRL                 0x03, 0x07, 0
/** Constant CLKM_no_clock for sub-register @ref SR_CLKM_CTRL */
#define CLKM_no_clock            (0)
/** Constant CLKM_1MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_1MHz                (1)
/** Constant CLKM_2MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_2MHz                (2)
/** Constant CLKM_4MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_4MHz                (3)
/** Constant CLKM_8MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_8MHz                (4)
/** Constant CLKM_16MHz for sub-register @ref SR_CLKM_CTRL */
#define CLKM_16MHz               (5)

/* ###########################################################################
 * # 0x04: TRX_CTRL_1                                                        #
 * ###########################################################################
 */

/** Offset for register TRX_CTRL_1 */
#define RG_TRX_CTRL_1                    (0x04)

/** Access parameters for sub-register PA_EXT_EN in register @ref RG_TRX_CTRL_1 */
#define SR_PA_EXT_EN                 0x04, 0x80, 7

/** Access parameters for sub-register IRQ_2_EXT_EN in register @ref RG_TRX_CTRL_1 */
#define SR_IRQ_2_EXT_EN              0x04, 0x40, 6

/** Access parameters for sub-register TX_AUTO_CRC_ON in register @ref RG_TRX_CTRL_1 */
#define SR_TX_AUTO_CRC_ON_NEW        0x04, 0x20, 5

/** Access parameters for sub-register RX_BL_CTRL in register @ref RG_TRX_CTRL_1 */
#define SR_RX_BL_CTRL                0x04, 0x10, 4

/** Access parameters for sub-register SPI_CMD_MODE in register @ref RG_TRX_CTRL_1 */
#define SR_SPI_CMD_MODE              0x04, 0x0C, 2

/** Access parameters for sub-register IRQ_MASK_MODE in register @ref RG_TRX_CTRL_1 */
#define SR_IRQ_MASK_MODE             0x04, 0x02, 1

/** Access parameters for sub-register IRQ_POLARITY in register @ref RG_TRX_CTRL_1 */
#define SR_IRQ_POLARITY              0x04, 0x01, 0

/* ###########################################################################
 * # 0x05: PHY_TX_PWR                                                        #
 * ###########################################################################
 */

/** Offset for register PHY_TX_PWR */
#define RG_PHY_TX_PWR                    (0x05)

/** Access parameters for sub-register TX_AUTO_CRC_ON in register @ref RG_PHY_TX_PWR */
#define SR_TX_AUTO_CRC_ON            0x05, 0x80, 7

#define SR_reserved_05_2             0x05, 0x70, 4

/** Access parameters for sub-register TX_PWR in register @ref RG_PHY_TX_PWR */
#define SR_TX_PWR                    0x05, 0x0f, 0

/* ###########################################################################
 * # 0x06: PHY_RSSI                                                          #
 * ###########################################################################
 */

/** Offset for register PHY_RSSI */
#define RG_PHY_RSSI                      (0x06)

/** Access parameters for sub-register RX_CRC_VALID in register @ref RG_PHY_RSSI */
#define SR_RX_CRC_VALID              0x06, 0x80, 7

/** Access parameters for sub-register RND_VALUE in register @ref RG_PHY_RSSI */
#define SR_RND_VALUE                 0x06, 0x60, 5

/** Access parameters for sub-register RSSI in register @ref RG_PHY_RSSI */
#define SR_RSSI                      0x06, 0x1f, 0

/* ###########################################################################
 * # 0x07: PHY_ED_LEVEL                                                      #
 * ###########################################################################
 */

/** Offset for register PHY_ED_LEVEL */
#define RG_PHY_ED_LEVEL                  (0x07)

/** Access parameters for sub-register ED_LEVEL in register @ref RG_PHY_ED_LEVEL */
#define SR_ED_LEVEL                  0x07, 0xff, 0

/* ###########################################################################
 * # 0x08: PHY_CC_CCAL                                                      #
 * ###########################################################################
 */

/** Offset for register PHY_CC_CCA */
#define RG_PHY_CC_CCA                    (0x08)

/** Access parameters for sub-register CCA_REQUEST in register @ref RG_PHY_CC_CCA */
#define SR_CCA_REQUEST               0x08, 0x80, 7

/** Access parameters for sub-register CCA_MODE in register @ref RG_PHY_CC_CCA */
#define SR_CCA_MODE                  0x08, 0x60, 5

/** Access parameters for sub-register CHANNEL in register @ref RG_PHY_CC_CCA */
#define SR_CHANNEL                   0x08, 0x1f, 0

/* ###########################################################################
 * # 0x09: CCA_THRES                                                         #
 * ###########################################################################
 */

/** Offset for register CCA_THRES */
#define RG_CCA_THRES                     (0x09)

/** Access parameters for sub-register CCA_CS_THRES in register @ref RG_CCA_THRES */
#define SR_CCA_CS_THRES              0x09, 0xf0, 4

/** Access parameters for sub-register CCA_ED_THRES in register @ref RG_CCA_THRES */
#define SR_CCA_ED_THRES              0x09, 0x0f, 0

/* ###########################################################################
 * # 0x0a: RX_CTRL                                                           #
 * ###########################################################################
 */

/** Offset for register RX_CTRL */
#define RG_RX_CTRL                       (0x0a)

/* ###########################################################################
 * # 0x0b: SFD_VALUE                                                         #
 * ###########################################################################
 */

/** Offset for register SFD_VALUE */
#define RG_SFD_VALUE                     (0x0b)

/* ###########################################################################
 * # 0x0c: TRX_CTRL_2                                                        #
 * ###########################################################################
 */

/** Offset for register TRX_CTRL_2 */
#define RG_TRX_CTRL_2                    (0x0c)

/* ###########################################################################
 * # 0x0d: ANT_DIV                                                           #
 * ###########################################################################
 */

/** Offset for register ANT_DIV */
#define RG_ANT_DIV                       (0x0d)

/* ###########################################################################
 * # 0x0e: IRQ_MASK                                                          #
 * ###########################################################################
 */

/** Offset for register IRQ_MASK */
#define RG_IRQ_MASK                      (0x0e)

/** Access parameters for sub-register IRQ_MASK in register @ref RG_IRQ_MASK */
#define SR_IRQ_MASK                  0x0e, 0xff, 0

/* ###########################################################################
 * # 0x0f: IRQ_STATUS                                                        #
 * ###########################################################################
 */

/** Offset for register IRQ_STATUS */
#define RG_IRQ_STATUS                    (0x0f)

/** Access parameters for sub-register IRQ_7_BAT_LOW in register @ref RG_IRQ_STATUS */
#define SR_IRQ_7_BAT_LOW             0x0f, 0x80, 7

/** Access parameters for sub-register IRQ_6_TRX_UR in register @ref RG_IRQ_STATUS */
#define SR_IRQ_6_TRX_UR              0x0f, 0x40, 6

/** Access parameters for sub-register IRQ_5 in register @ref RG_IRQ_STATUS */
#define SR_IRQ_5                     0x0f, 0x20, 5

/** Access parameters for sub-register IRQ_4 in register @ref RG_IRQ_STATUS */
#define SR_IRQ_4                     0x0f, 0x10, 4

/** Access parameters for sub-register IRQ_3_TRX_END in register @ref RG_IRQ_STATUS */
#define SR_IRQ_3_TRX_END             0x0f, 0x08, 3

/** Access parameters for sub-register IRQ_2_RX_START in register @ref RG_IRQ_STATUS */
#define SR_IRQ_2_RX_START            0x0f, 0x04, 2

/** Access parameters for sub-register IRQ_1_PLL_UNLOCK in register @ref RG_IRQ_STATUS */
#define SR_IRQ_1_PLL_UNLOCK          0x0f, 0x02, 1

/** Access parameters for sub-register IRQ_0_PLL_LOCK in register @ref RG_IRQ_STATUS */
#define SR_IRQ_0_PLL_LOCK            0x0f, 0x01, 0

/* ###########################################################################
 * # 0x10: VREG_CTRL                                                         #
 * ###########################################################################
 */

/** Offset for register VREG_CTRL */
#define RG_VREG_CTRL                     (0x10)

/** Access parameters for sub-register AVREG_EXT in register @ref RG_VREG_CTRL */
#define SR_AVREG_EXT                 0x10, 0x80, 7

/** Access parameters for sub-register AVDD_OK in register @ref RG_VREG_CTRL */
#define SR_AVDD_OK                   0x10, 0x40, 6

/** Access parameters for sub-register AVREG_TRIM in register @ref RG_VREG_CTRL */
#define SR_AVREG_TRIM                0x10, 0x30, 4
/** Constant AVREG_1_80V for sub-register @ref SR_AVREG_TRIM */
#define AVREG_1_80V              (0)
/** Constant AVREG_1_75V for sub-register @ref SR_AVREG_TRIM */
#define AVREG_1_75V              (1)
/** Constant AVREG_1_84V for sub-register @ref SR_AVREG_TRIM */
#define AVREG_1_84V              (2)
/** Constant AVREG_1_88V for sub-register @ref SR_AVREG_TRIM */
#define AVREG_1_88V              (3)

/** Access parameters for sub-register DVREG_EXT in register @ref RG_VREG_CTRL */
#define SR_DVREG_EXT                 0x10, 0x08, 3

/** Access parameters for sub-register DVDD_OK in register @ref RG_VREG_CTRL */
#define SR_DVDD_OK                   0x10, 0x04, 2

/** Access parameters for sub-register DVREG_TRIM in register @ref RG_VREG_CTRL */
#define SR_DVREG_TRIM                0x10, 0x03, 0
/** Constant DVREG_1_80V for sub-register @ref SR_DVREG_TRIM */
#define DVREG_1_80V              (0)
/** Constant DVREG_1_75V for sub-register @ref SR_DVREG_TRIM */
#define DVREG_1_75V              (1)
/** Constant DVREG_1_84V for sub-register @ref SR_DVREG_TRIM */
#define DVREG_1_84V              (2)
/** Constant DVREG_1_88V for sub-register @ref SR_DVREG_TRIM */
#define DVREG_1_88V              (3)

/* ###########################################################################
 * # 0x11: BATMON                                                            #
 * ###########################################################################
 */

/** Offset for register BATMON */
#define RG_BATMON                        (0x11)

#define SR_reserved_11_1             0x11, 0xc0, 6

/** Access parameters for sub-register BATMON_OK in register @ref RG_BATMON */
#define SR_BATMON_OK                 0x11, 0x20, 5

/** Access parameters for sub-register BATMON_HR in register @ref RG_BATMON */
#define SR_BATMON_HR                 0x11, 0x10, 4

/** Access parameters for sub-register BATMON_VTH in register @ref RG_BATMON */
#define SR_BATMON_VTH                0x11, 0x0f, 0

/* ###########################################################################
 * # 0x12: XOSC_CTRL                                                         #
 * ###########################################################################
 */

/** Offset for register XOSC_CTRL */
#define RG_XOSC_CTRL                     (0x12)

/** Access parameters for sub-register XTAL_MODE in register @ref RG_XOSC_CTRL */
#define SR_XTAL_MODE                 0x12, 0xf0, 4

/** Access parameters for sub-register XTAL_TRIM in register @ref RG_XOSC_CTRL */
#define SR_XTAL_TRIM                 0x12, 0x0f, 0

/* ###########################################################################
 * # 0x13: CC_CTRL_0                                                         #
 * ###########################################################################
 */

/** Offset for register CC_CTRL_0 */
#define RG_CC_CTRL_0                     (0x13)

/** Access parameters for sub-register CC_NUMBER in register @ref RG_CC_CTRL_0 */
#define SR_CC_NUMBER                 0x13, 0xff, 0

/* ###########################################################################
 * # 0x14: CC_CTRL_1                                                         #
 * ###########################################################################
 */

/** Offset for register CC_CTRL_1 */
#define RG_CC_CTRL_1                     (0x14)

#define SR_reserved_14_1           0x14, 0xf0, 4

/** Access parameters for sub-register CC_BAND in register @ref RG_CC_CTRL_1 */
#define SR_CC_BAND                 0x14, 0x0f, 0

/* ###########################################################################
 * # 0x15: RX_SYN                                                            #
 * ###########################################################################
 */

/** Offset for register RX_SYN */
#define RG_RX_SYN                        (0x15)

/** Access parameters for sub-register RX_PDT_DIS in register @ref RG_RX_SYN */
#define SR_RX_PDT_DIS              0x15, 0x80, 7

#define SR_reserved_15_1           0x15, 0x70, 4

/** Access parameters for sub-register RX_PDT_LEVEL in register @ref RG_RX_SYN */
#define SR_RX_PDT_LEVEL            0x15, 0x0f, 0

/* ###########################################################################
 * # 0x16: TRX_RPC                                                           #
 * ###########################################################################
 */

/** Offset for register TRX_RPC */
#define RG_TRX_RPC                       (0x16)

/* ###########################################################################
 * # 0x17: XAH_CTRL_1                                                        #
 * ###########################################################################
 */

/** Offset for register XAH_CTRL_1 */
#define RG_XAH_CTRL_1                    (0x17)

/** Access parameters for sub-register ARET_TX_TS_EN in register @ref RG_XAH_CTRL_1 */
#define SR_ARET_TX_TS_EN             0x17, 0x80, 7

#define SR_reserved_17_2             0x17, 0x40, 6

/** Access parameters for sub-register AACK_FLTR_RES_FT in register @ref RG_XAH_CTRL_1 */
#define SR_AACK_FLTR_RES_FT          0x17, 0x20, 5

/** Access parameters for sub-register AACK_UPLD_RES_FT in register @ref RG_XAH_CTRL_1 */
#define SR_AACK_UPLD_RES_FT          0x17, 0x10, 4

#define reserved_18_5                0x17, 0x08, 3

/** Access parameters for sub-register AACK_ACK_TIME in register @ref RG_XAH_CTRL_1 */
#define SR_AACK_ACK_TIME             0x17, 0x04, 2

/** Access parameters for sub-register AACK_PROM_MODE in register @ref RG_XAH_CTRL_1 */
#define SR_AACK_PROM_MODE            0x17, 0x02, 1

/** Access parameters for sub-register AACK_SPC_EN in register @ref RG_XAH_CTRL_1 */
#define SR_AACK_SPC_EN               0x17, 0x01, 0

/* ###########################################################################
 * # 0x18: FTN_CTRL                                                          #
 * ###########################################################################
 */

/** Offset for register FTN_CTRL */
#define RG_FTN_CTRL                      (0x18)

/** Access parameters for sub-register FTN_START in register @ref RG_FTN_CTRL */
#define SR_FTN_START                 0x18, 0x80, 7

#define SR_reserved_18_2             0x18, 0x40, 6

/** Access parameters for sub-register FTNV in register @ref RG_FTN_CTRL */
#define SR_FTNV                      0x18, 0x3f, 0

/* ###########################################################################
 * # 0x19: XAH_CTRL_2                                                        #
 * ###########################################################################
 */

/** Offset for register XAH_CTRL_2 */
#define RG_XAH_CTRL_2                    (0x19)

/* ###########################################################################
 * # 0x1a: PLL_CF                                                            #
 * ###########################################################################
 */

/** Offset for register PLL_CF */
#define RG_PLL_CF                        (0x1a)

/** Access parameters for sub-register PLL_CF_START in register @ref RG_PLL_CF */
#define SR_PLL_CF_START              0x1a, 0x80, 7

#define SR_reserved_1a_2             0x1a, 0x70, 4

/** Access parameters for sub-register PLL_CF in register @ref RG_PLL_CF */
#define SR_PLL_CF                    0x1a, 0x0f, 0

/* ###########################################################################
 * # 0x1b: PLL_DCU                                                           #
 * ###########################################################################
 */

/** Offset for register PLL_DCU */
#define RG_PLL_DCU                       (0x1b)

/** Access parameters for sub-register PLL_DCU_START in register @ref RG_PLL_DCU */
#define SR_PLL_DCU_START             0x1b, 0x80, 7

#define SR_reserved_1b_2             0x1b, 0x40, 6

/** Access parameters for sub-register PLL_DCUW in register @ref RG_PLL_DCU */
#define SR_PLL_DCUW                  0x1b, 0x3f, 0

/* ###########################################################################
 * # 0x1c: PART_NUM                                                          #
 * ###########################################################################
 */

/** Offset for register PART_NUM */
#define RG_PART_NUM                      (0x1c)

/** Access parameters for sub-register PART_NUM in register @ref RG_PART_NUM */
#define SR_PART_NUM                  0x1c, 0xff, 0
/** Constant RF230 for sub-register @ref SR_PART_NUM */
#define RF230                    (0x02)
/** Constant RF231 for sub-register @ref SR_PART_NUM */
#define RF231                    (0x03)
/** Constant RF232 for sub-register @ref SR_PART_NUM */
#define RF232                    (0x0a)
/** Constant RF233 for sub-register @ref SR_PART_NUM */
#define RF233                    (0x0b)

/* ###########################################################################
 * # 0x1d: VERSION_NUM                                                       #
 * ###########################################################################
 */

/** Offset for register VERSION_NUM */
#define RG_VERSION_NUM                   (0x1d)

/** Access parameters for sub-register VERSION_NUM in register @ref RG_VERSION_NUM */
#define SR_VERSION_NUM               0x1d, 0xff, 0
/** Constant RF230_REV_A for sub-register @ref SR_VERSION_NUM */
#define RF230_REV_A              (0x01)
/** Constant RF230_REV_B for sub-register @ref SR_VERSION_NUM */
#define RF230_REV_B              (0x02)
/** Constant RF231_REV_A for sub-register @ref SR_VERSION_NUM */
#define RF231_REV_A              (0x02)
/** Constant RF232_REV_A for sub-register @ref SR_VERSION_NUM */
#define RF232_REV_A              (0x02)
/** Constant RF233_REV_A for sub-register @ref SR_VERSION_NUM */
#define RF233_REV_A              (0x01)
/** Constant RF233_REV_B for sub-register @ref SR_VERSION_NUM */
#define RF233_REV_B              (0x02)

/* ###########################################################################
 * # 0x1e: MAN_ID_0                                                          #
 * ###########################################################################
 */

/** Offset for register MAN_ID_0 */
#define RG_MAN_ID_0                      (0x1e)

/** Access parameters for sub-register MAN_ID_0 in register @ref RG_MAN_ID_0 */
#define SR_MAN_ID_0                  0x1e, 0xff, 0

/* ###########################################################################
 * # 0x1f: MAN_ID_1                                                          #
 * ###########################################################################
 */

/** Offset for register MAN_ID_1 */
#define RG_MAN_ID_1                      (0x1f)

/** Access parameters for sub-register MAN_ID_1 in register @ref RG_MAN_ID_1 */
#define SR_MAN_ID_1                  0x1f, 0xff, 0

/* ###########################################################################
 * # 0x20: SHORT_ADDR_0                                                      #
 * ###########################################################################
 */

/** Offset for register SHORT_ADDR_0 */
#define RG_SHORT_ADDR_0                  (0x20)

/** Access parameters for sub-register SHORT_ADDR_0 in register @ref RG_SHORT_ADDR_0 */
#define SR_SHORT_ADDR_0              0x20, 0xff, 0

/* ###########################################################################
 * # 0x21: SHORT_ADDR_1                                                      #
 * ###########################################################################
 */

/** Offset for register SHORT_ADDR_1 */
#define RG_SHORT_ADDR_1                  (0x21)

/** Access parameters for sub-register SHORT_ADDR_1 in register @ref RG_SHORT_ADDR_1 */
#define SR_SHORT_ADDR_1              0x21, 0xff, 0

/* ###########################################################################
 * # 0x22: PAN_ID_0                                                          #
 * ###########################################################################
 */

/** Offset for register PAN_ID_0 */
#define RG_PAN_ID_0                      (0x22)

/** Access parameters for sub-register PAN_ID_0 in register @ref RG_PAN_ID_0 */
#define SR_PAN_ID_0                  0x22, 0xff, 0

/* ###########################################################################
 * # 0x23: PAN_ID_1                                                          #
 * ###########################################################################
 */

/** Offset for register PAN_ID_1 */
#define RG_PAN_ID_1                      (0x23)

/** Access parameters for sub-register PAN_ID_1 in register @ref RG_PAN_ID_1 */
#define SR_PAN_ID_1                  0x23, 0xff, 0

/* ###########################################################################
 * # 0x24: IEEE_ADDR_0                                                       #
 * ###########################################################################
 */

/** Offset for register IEEE_ADDR_0 */
#define RG_IEEE_ADDR_0                   (0x24)

/** Access parameters for sub-register IEEE_ADDR_0 in register @ref RG_IEEE_ADDR_0 */
#define SR_IEEE_ADDR_0               0x24, 0xff, 0

/* ###########################################################################
 * # 0x25: IEEE_ADDR_1                                                       #
 * ###########################################################################
 */

/** Offset for register IEEE_ADDR_1 */
#define RG_IEEE_ADDR_1                   (0x25)

/** Access parameters for sub-register IEEE_ADDR_1 in register @ref RG_IEEE_ADDR_1 */
#define SR_IEEE_ADDR_1               0x25, 0xff, 0

/* ###########################################################################
 * # 0x26: IEEE_ADDR_2                                                       #
 * ###########################################################################
 */

/** Offset for register IEEE_ADDR_2 */
#define RG_IEEE_ADDR_2                   (0x26)

/** Access parameters for sub-register IEEE_ADDR_2 in register @ref RG_IEEE_ADDR_2 */
#define SR_IEEE_ADDR_2               0x26, 0xff, 0

/* ###########################################################################
 * # 0x27: IEEE_ADDR_3                                                       #
 * ###########################################################################
 */

/** Offset for register IEEE_ADDR_3 */
#define RG_IEEE_ADDR_3                   (0x27)

/** Access parameters for sub-register IEEE_ADDR_3 in register @ref RG_IEEE_ADDR_3 */
#define SR_IEEE_ADDR_3               0x27, 0xff, 0

/* ###########################################################################
 * # 0x28: IEEE_ADDR_4                                                       #
 * ###########################################################################
 */

/** Offset for register IEEE_ADDR_4 */
#define RG_IEEE_ADDR_4                   (0x28)

/** Access parameters for sub-register IEEE_ADDR_4 in register @ref RG_IEEE_ADDR_4 */
#define SR_IEEE_ADDR_4               0x28, 0xff, 0

/* ###########################################################################
 * # 0x29: IEEE_ADDR_5                                                       #
 * ###########################################################################
 */

/** Offset for register IEEE_ADDR_5 */
#define RG_IEEE_ADDR_5                   (0x29)

/** Access parameters for sub-register IEEE_ADDR_5 in register @ref RG_IEEE_ADDR_5 */
#define SR_IEEE_ADDR_5               0x29, 0xff, 0

/* ###########################################################################
 * # 0x2a: IEEE_ADDR_6                                                       #
 * ###########################################################################
 */

/** Offset for register IEEE_ADDR_6 */
#define RG_IEEE_ADDR_6                   (0x2a)

/** Access parameters for sub-register IEEE_ADDR_6 in register @ref RG_IEEE_ADDR_6 */
#define SR_IEEE_ADDR_6               0x2a, 0xff, 0

/* ###########################################################################
 * # 0x2b: IEEE_ADDR_7                                                       #
 * ###########################################################################
 */

/** Offset for register IEEE_ADDR_7 */
#define RG_IEEE_ADDR_7                   (0x2b)

/** Access parameters for sub-register IEEE_ADDR_7 in register @ref RG_IEEE_ADDR_7 */
#define SR_IEEE_ADDR_7               0x2b, 0xff, 0

/* ###########################################################################
 * # 0x2c: XAH_CTRL_0                                                        #
 * ###########################################################################
 */

/** Offset for register XAH_CTRL_0 */
#define RG_XAH_CTRL_0                    (0x2c)

/** Access parameters for sub-register MAX_FRAME_RETRIES in register @ref RG_XAH_CTRL_0 */
#define SR_MAX_FRAME_RETRIES         0x2c, 0xf0, 4

/** Access parameters for sub-register MAX_CSMA_RETRIES in register @ref RG_XAH_CTRL_0 */
#define SR_MAX_CSMA_RETRIES          0x2c, 0x0e, 1

#define SR_reserved_2c_3             0x2c, 0x01, 0

/* ###########################################################################
 * # 0x2d: CSMA_SEED_0                                                       #
 * ###########################################################################
 */

/** Offset for register CSMA_SEED_0 */
#define RG_CSMA_SEED_0                   (0x2d)

/** Access parameters for sub-register CSMA_SEED_0 in register @ref RG_CSMA_SEED_0 */
#define SR_CSMA_SEED_0               0x2d, 0xff, 0

/* ###########################################################################
 * # 0x2e: CSMA_SEED_1                                                       #
 * ###########################################################################
 */

/** Offset for register CSMA_SEED_1 */
#define RG_CSMA_SEED_1                   (0x2e)

/** Access parameters for sub-register AACK_FVN_MODE in register @ref RG_CSMA_SEED_1 */
#define SR_AACK_FVN_MODE             0x2e, 0xc0, 6

/** Access parameters for AACK_SET_PD bit in register @ref RG_CSMA_SEED_1 */
#define SR_AACK_SET_PD               0x2e, 0x20, 5

/** Access parameters for sub-register I_AM_COORD in register @ref RG_CSMA_SEED_1 */
#define SR_I_AM_COORD                0x2e, 0x08, 3

/** Access parameters for sub-register CSMA_SEED_1 in register @ref RG_CSMA_SEED_1 */
#define SR_CSMA_SEED_1               0x2e, 0x07, 0

/* ###########################################################################
 * # 0x2f: CSMA_BE                                                           #
 * ###########################################################################
 */

/** Offset for register CSMA_BE */
#define RG_CSMA_BE                       (0x2f)

/* ###########################################################################
 * # 0x30: reserved                                                          #
 * ###########################################################################
 */

/* ###########################################################################
 * # 0x31: reserved                                                          #
 * ###########################################################################
 */

/* ###########################################################################
 * # 0x32: reserved                                                          #
 * ###########################################################################
 */

/* ###########################################################################
 * # 0x33: reserved                                                          #
 * ###########################################################################
 */

/* ###########################################################################
 * # 0x34: reserved                                                          #
 * ###########################################################################
 */

/* ###########################################################################
 * # 0x35: reserved                                                          #
 * ###########################################################################
 */

/* ###########################################################################
 * # 0x36: TST_CTRL_DIGI                                                     #
 * ###########################################################################
 */

/** Offset for register TST_CTRL_DIGI */
#define RG_TST_CTRL_DIGI                 (0x36)

#define SR_reserved_36_1             0x36, 0xF0, 4

/** Access parameters for sub-register TST_CTRL_DIG in register @ref RG_TST_CTRL_DIGI */
#define SR_TST_CTRL_DIG              0x36, 0x0F, 0
/** Constant TST_CTRL_DIG_NO_MODE for sub-register @ref SR_TST_CTRL_DIG */
#define TST_CTRL_DIG_NO_MODE     (0x0)
/** Constant TST_CTRL_DIG_CW_TEST for sub-register @ref SR_TST_CTRL_DIG */
#define TST_CTRL_DIG_CW_TEST     (0xF)


/* ###########################################################################
 * # 0x37: reserved                                                          #
 * ###########################################################################
 */

/* ###########################################################################
 * # 0x38: reserved                                                          #
 * ###########################################################################
 */

/* ###########################################################################
 * # 0x39: reserved                                                          #
 * ###########################################################################
 */

/* ###########################################################################
 * # 0x3a: reserved                                                          #
 * ###########################################################################
 */

/* ###########################################################################
 * # 0x3b: PHY_TX_TIME / PHY_PMU_VALUE                                       #
 * ###########################################################################
 */

/** Offset for register PHY_TX_TIME */
#define RG_PHY_TX_TIME                   (0x3b)

#define SR_reserved_3b_1             0x3b, 0xF0, 4

/** Access parameters for sub-register IRC_TX_TIME in register @ref RG_PHY_TX_TIME */
#define SR_IRC_TX_TIME               0x3b, 0x0F, 0

/** Offset for register PHY_PMU_VALUE */
#define RG_PHY_PMU_VALUE                 (0x3b)

/** Access parameters for sub-register PMU_VALUE in register @ref RG_PHY_PMU_VALUE */
#define SR_PMU_VALUE                 0x3b, 0xFF, 0

/* ###########################################################################
 * # 0x3c: TST_AGC                                                           #
 * ###########################################################################
 */

/** Offset for register TST_AGC */
#define RG_TST_AGC                       (0x3c)

#define SR_reserved_3c_1             0x3c, 0xC0, 6

/** Access parameters for sub-register AGC_HOLD_SEL in register @ref RG_TST_AGC */
#define SR_AGC_HOLD_SEL              0x3c, 0x20, 5

/** Access parameters for sub-register AGC_RST in register @ref RG_TST_AGC */
#define SR_AGC_RST                   0x3c, 0x10, 4

/** Access parameters for sub-register AGC_OFF in register @ref RG_TST_AGC */
#define SR_AGC_OFF                   0x3c, 0x08, 3

/** Access parameters for sub-register AGC_HOLD in register @ref RG_TST_AGC */
#define SR_AGC_HOLD                  0x3c, 0x04, 2

/** Access parameters for sub-register GC in register @ref RG_TST_AGC */
#define SR_GC                        0x3c, 0x03, 0

/* ###########################################################################
 * # 0x3d: TST_SDM                                                           #
 * ###########################################################################
 */

/** Offset for register TST_SDM */
#define RG_TST_SDM                       (0x3d)

/** Access parameters for sub-register MOD_SEL in register @ref RG_TST_SDM */
#define SR_MOD_SEL                   0x3d, 0x80, 7

/** Access parameters for sub-register MOD in register @ref RG_TST_SDM */
#define SR_MOD                       0x3d, 0x40, 6

/** Access parameters for sub-register TX_RX in register @ref RG_TST_SDM */
#define SR_TX_RX                     0x3d, 0x20, 5

/** Access parameters for sub-register TX_RX_SEL in register @ref RG_TST_SDM */
#define SR_TX_RX_SEL                 0x3d, 0x10, 4

#define SR_reserved_3d_5             0x3d, 0x0F, 0

#endif /* PHY23x_REGISTERMAP_EXTERNAL_H */
