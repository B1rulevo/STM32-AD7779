#ifndef AD7779_REGS_H_
#define AD7779_REGS_H_

// AD7779 registers
#define AD7779_REG_CH_CONFIG(ch)    (0x00 + (ch))   // Channel Configuration
#define AD7779_REG_CH_DISABLE     0x08      // Disable clocks to ADC channel
#define AD7779_REG_CH_SYNC_OFFSET(ch)   (0x09 + (ch))   // Channel SYNC Offset
#define AD7779_REG_GENERAL_USER_CONFIG_1  0x11      // General User Config 1
#define AD7779_REG_GENERAL_USER_CONFIG_2  0x12      // General User Config 2
#define AD7779_REG_GENERAL_USER_CONFIG_3  0x13      // General User Config 3
#define AD7779_REG_DOUT_FORMAT      0x14      // Data out format
#define AD7779_REG_ADC_MUX_CONFIG   0x15      // Main ADC meter and reference Mux control
#define AD7779_REG_GLOBAL_MUX_CONFIG    0x16      // Global diagnostics mux
#define AD7779_REG_GPIO_CONFIG      0x17      // GPIO config
#define AD7779_REG_GPIO_DATA      0x18      // GPIO Data
#define AD7779_REG_BUFFER_CONFIG_1    0x19      // Buffer Config 1
#define AD7779_REG_BUFFER_CONFIG_2    0x1A      // Buffer Config 2
#define AD7779_REG_CH_OFFSET_UPPER_BYTE(ch) (0x1C + (ch) * 6) // Channel offset upper byte
#define AD7779_REG_CH_OFFSET_MID_BYTE(ch) (0x1D + (ch) * 6) // Channel offset middle byte
#define AD7779_REG_CH_OFFSET_LOWER_BYTE(ch) (0x1E + (ch) * 6) // Channel offset lower byte
#define AD7779_REG_CH_GAIN_UPPER_BYTE(ch) (0x1F + (ch) * 6) // Channel gain upper byte
#define AD7779_REG_CH_GAIN_MID_BYTE(ch)   (0x20 + (ch) * 6) // Channel gain middle byte
#define AD7779_REG_CH_GAIN_LOWER_BYTE(ch) (0x21 + (ch) * 6) // Channel gain lower byte
#define AD7779_REG_CH_ERR_REG(ch)   (0x4C + (ch))   // Channel Status Register
#define AD7779_REG_CH0_1_SAT_ERR    0x54      // Channel 0/1 DSP errors
#define AD7779_REG_CH2_3_SAT_ERR    0x55      // Channel 2/3 DSP errors
#define AD7779_REG_CH4_5_SAT_ERR    0x56      // Channel 4/5 DSP errors
#define AD7779_REG_CH6_7_SAT_ERR    0x57      // Channel 6/7 DSP errors
#define AD7779_REG_CHX_ERR_REG_EN   0x58      // Channel 0-7 Error Reg Enable
#define AD7779_REG_GEN_ERR_REG_1    0x59      // General Errors Register 1
#define AD7779_REG_GEN_ERR_REG_1_EN   0x5A      // General Errors Register 1 Enable
#define AD7779_REG_GEN_ERR_REG_2    0x5B      // General Errors Register 2
#define AD7779_REG_GEN_ERR_REG_2_EN   0x5C      // General Errors Register 2 Enable
#define AD7779_REG_STATUS_REG_1     0x5D      // Error Status Register 1
#define AD7779_REG_STATUS_REG_2     0x5E      // Error Status Register 2
#define AD7779_REG_STATUS_REG_3     0x5F      // Error Status Register 3
#define AD7779_REG_SRC_N_MSB      0x60      // Decimation Rate (N) MSB
#define AD7779_REG_SRC_N_LSB      0x61      // Decimation Rate (N) LSB
#define AD7779_REG_SRC_IF_MSB     0x62      // Decimation Rate (IF) MSB
#define AD7779_REG_SRC_IF_LSB     0x63      // Decimation Rate (IF) LSB
#define AD7779_REG_SRC_UPDATE     0x64      // SRC load source and load update

// AD7779 registers configurations
// AD7779_REG_CHx_CONFIG
#define AD7779_CH_GAIN(x)     (((x) & 0x3) << 6)
#define AD7779_CH_REF_MONITOR (1 << 5)
#define AD7779_CH_RX        (1 << 4)

// AD7779_REG_CH_DISABLE
#define AD7779_CH_DISABLE(x)      (1 << (x))

// AD7779_REG_GENERAL_USER_CONFIG_1
#define AD7779_ALL_CH_DIS_MCLK_EN   (1 << 7)
#define AD7779_MOD_POWERMODE      (1 << 6)
#define AD7779_PDB_VCM        (1 << 5)
#define AD7779_PDB_REFOUT_BUF     (1 << 4)
#define AD7779_PDB_SAR        (1 << 3)
#define AD7779_PDB_RC_OSC     (1 << 2)
#define AD7779_SOFT_RESET(x)      (((x) & 0x3) << 0)

// AD7779_REG_GENERAL_USER_CONFIG_2
#define AD7779_FILTER_MODE      (1 << 6)
#define AD7779_SAR_DIAG_MODE_EN     (1 << 5)
#define AD7779_SDO_DRIVE_STR(x)     (((x) & 0x3) << 3)
#define AD7779_DOUT_DRIVE_STR(x)    (((x) & 0x3) << 1)
#define AD7779_SPI_SYNC       (1 << 0)

// AD7779_REG_GENERAL_USER_CONFIG_3
#define AD7779_CONVST_SAR_DEBURRING_1_5MCLK (0x02 << 6)
#define AD7779_CONVST_SAR_DEBURRING_NONE (0x03 << 6)
#define AD7779_SPI_SLAVE_MODE_EN    (1 << 4)
#define AD7779_CLK_QUAL_DIS     (1 << 0)

// AD7779_REG_DOUT_FORMAT
#define AD7779_DOUT_FORMAT(x)     (((x) & 0x3) << 6)
#define AD7779_DOUT_HEADER_FORMAT   (1 << 5)
#define AD7779_DCLK_CLK_DIV(x)      (((x) & 0x7) << 1)

// AD7779_REG_ADC_MUX_CONFIG
#define AD7779_REF_MUX_CTRL(x) (((x) & 0x3) << 6)
#define AD7779_MTR_MUX_CTRL(x) (((x) & 0x0F) << 2)

// AD7779_REG_GLOBAL_MUX_CONFIG
#define AD7779_GLOBAL_MUX_CTRL(x)   (((x) & 0x1F) << 3)

// AD7779_REG_BUFFER_CONFIG_1
#define AD7779_REF_BUF_POS_EN     (1 << 4)
#define AD7779_REF_BUF_NEG_EN     (1 << 3)

// AD7779_REG_BUFFER_CONFIG_2
#define AD7779_REFBUFP_PREQ     (1 << 7)
#define AD7779_REFBUFN_PREQ     (1 << 6)
#define AD7779_PDB_ALDO1_OVRDRV     (1 << 2)
#define AD7779_PDB_ALDO2_OVRDRV     (1 << 1)
#define AD7779_PDB_DLDO_OVRDRV      (1 << 0)

// AD7779_REG_CHX_ERR_REG_EN
#define AD7779_OUTPUT_SAT_TEST_EN   (1 << 7)
#define AD7779_FILTER_SAT_TEST_EN       (1 << 6)
#define AD7779_MOD_SAT_TEST_EN    (1 << 5)
#define AD7779_AINM_UV_TEST_EN        (1 << 4)
#define AD7779_AINM_OV_TEST_EN      (1 << 3)
#define AD7779_AINP_UV_TEST_EN      (1 << 2)
#define AD7779_AINP_OV_TEST_EN  (1 << 1)
#define AD7779_REG_DET_TEST_EN      (1 << 0)

// AD7779_REG_GEN_ERR_REG_1_EN
#define AD7779_MEMMAP_CRC_TEST_EN   (1 << 5)
#define AD7779_ROM_CRC_TEST_EN      (1 << 4)
#define AD7779_SPI_CLK_COUNT_TEST_EN    (1 << 3)
#define AD7779_SPI_INVALID_READ_TEST_EN   (1 << 2)
#define AD7779_SPI_INVALID_WRITE_TEST_EN  (1 << 1)
#define AD7779_SPI_CRC_TEST_EN      (1 << 0)

// AD7779_REG_GEN_ERR_REG_2_EN
#define AD7779_RESET_DETECT_TEST_EN   (1 << 5)
#define AD7779_LDO_PSM_TEST_EN(x) (((x) & 0x03) << 2)
#define AD7779_LDO_PSM_TRIP_TEST_EN(x) (((x) & 0x03) << 0)

// AD7779 error flags
// GEN_ERR_REG1
#define AD7779_GEN1_ERR_MEMMAP_CRC_TEST (1 << 5)
#define AD7779_GEN1_ERR_ROM_CRC_TEST (1 << 4)
#define AD7779_GEN1_ERR_SPI_CLK_COUNT_TEST (1 << 3)
#define AD7779_GEN1_ERR_SPI_INVALID_READ_TEST (1 << 2)
#define AD7779_GEN1_ERR_SPI_INVALID_WRITE_TEST (1 << 1)
#define AD7779_GEN1_ERR_SPI_CRC_TEST (1 << 0)

// GEN_ERR_REG2
#define AD7779_GEN2_ERR_RESET_DETECTED (1 << 5)
#define AD7779_GEN2_ERR_EXT_MCLK_SWITCH (1 << 4)
#define AD7779_GEN2_ERR_ALDO1_PSM (1 << 2)
#define AD7779_GEN2_ERR_ALDO2_PSM (1 << 1)
#define AD7779_GEN2_ERR_DLDO_PSM (1 << 0)

// General errors
#define AD7779_GEN_ERR_MEMMAP_CRC_TEST AD7779_GEN1_ERR_MEMMAP_CRC_TEST
#define AD7779_GEN_ERR_ROM_CRC_TEST AD7779_GEN1_ERR_ROM_CRC_TEST
#define AD7779_GEN_ERR_SPI_CLK_COUNT_TEST AD7779_GEN1_ERR_SPI_CLK_COUNT_TEST
#define AD7779_GEN_ERR_SPI_INVALID_READ_TEST AD7779_GEN1_ERR_SPI_INVALID_READ_TEST
#define AD7779_GEN_ERR_SPI_INVALID_WRITE_TEST AD7779_GEN1_ERR_SPI_INVALID_WRITE_TEST
#define AD7779_GEN_ERR_SPI_CRC_TEST AD7779_GEN1_ERR_SPI_CRC_TEST
#define AD7779_GEN_ERR_RESET_DETECTED (AD7779_GEN2_ERR_RESET_DETECTED << 8)
#define AD7779_GEN_ERR_EXT_MCLK_SWITCH (AD7779_GEN2_ERR_EXT_MCLK_SWITCH << 8)
#define AD7779_GEN_ERR_ALDO1_PSM (AD7779_GEN2_ERR_ALDO1_PSM << 8)
#define AD7779_GEN_ERR_ALDO2_PSM (AD7779_GEN2_ERR_ALDO2_PSM << 8)
#define AD7779_GEN_ERR_DLDO_PSM (AD7779_GEN2_ERR_DLDO_PSM << 8)

// Channel errors
#define AD7779_CH_ERR_AINM_UV (1 << 4)
#define AD7779_CH_ERR_AINM_OV (1 << 3)
#define AD7779_CH_ERR_AINP_UV (1 << 2)
#define AD7779_CH_ERR_AINP_OV (1 << 1)
#define AD7779_CH_ERR_REF_DET (1 << 0)

// Channel DSP errors
#define AD7779_CH0246_ERR_DSP_MOD_SAT (1 << 2)
#define AD7779_CH1357_ERR_DSP_MOD_SAT (1 << 5)
#define AD7779_CH0246_ERR_DSP_FILTER_SAT (1 << 1)
#define AD7779_CH1357_ERR_DSP_FILTER_SAT (1 << 4)
#define AD7779_CH0246_ERR_DSP_OUTPUT_SAT (1 << 0)
#define AD7779_CH1357_ERR_DSP_OUTPUT_SAT (1 << 3)

// Flag check tool
#define AD7779_FLAGCHECK(reg_data, flag) (((reg_data) & (flag)) > 0)

#endif /* AD7779_REGS_H_ */
