#pragma once

// AD7779 /DRDY interrupt priority
#define AD7779_DRDY_INT_MAIN_PRIO 9
#define AD7779_DRDY_INT_SUB_PRIO 0

// AD7779 CRC polynomial
#define AD7779_CRC_POLY			0x07



// AD7779 configuration enumeration
typedef enum
{
	AD7779_INT_REG,
	AD7779_SD_CONV,
	AD7779_SAR_CONV,
} ad7779_spi_op_mode;

typedef enum
{
	AD7779_CH0,
	AD7779_CH1,
	AD7779_CH2,
	AD7779_CH3,
	AD7779_CH4,
	AD7779_CH5,
	AD7779_CH6,
	AD7779_CH7,
} ad7779_ch;

typedef enum
{
	AD7779_ENABLE,
	AD7779_DISABLE,
} ad7779_state;

typedef enum
{
	AD7779_GAIN_1,
	AD7779_GAIN_2,
	AD7779_GAIN_4,
	AD7779_GAIN_8,
} ad7779_gain;

typedef enum
{
	AD7779_DOUT_FORMAT_4LINES,
	AD7779_DOUT_FORMAT_2LINES,
	AD7779_DOUT_FORMAT_1LINE
}ad7779_dout_format;

typedef enum
{
	AD7779_HEADER_STATUS,
	AD7779_HEADER_CRC
}ad7779_dout_header_format;

typedef enum
{
	AD7779_LDO_PSM_TEST_DISABLE,
	AD7779_LDO_PSM_TEST_AREGxCAP,
	AD7779_LDO_PSM_TEST_DREGCAP,
	AD7779_LDO_PSM_TEST_ALL
}ad7779_ldo_psm_test_en;

typedef enum
{
	AD7779_LDO_PSM_TRIP_TEST_DISABLE,
	AD7779_LDO_PSM_TRIP_TEST_AREG1CAP,
	AD7779_LDO_PSM_TRIP_TEST_AREG2CAP,
	AD7779_LDO_PSM_TRIP_TEST_DREGCAP
}ad7779_ldo_psm_trip_test_en;

typedef enum
{
	AD7779_DCLK_DIV_1,
	AD7779_DCLK_DIV_2,
	AD7779_DCLK_DIV_4,
	AD7779_DCLK_DIV_8,
	AD7779_DCLK_DIV_16,
	AD7779_DCLK_DIV_32,
	AD7779_DCLK_DIV_64,
	AD7779_DCLK_DIV_128,
} ad7779_dclk_div;

typedef enum
{
	AD7779_REFMUX_EXTREF,
	AD7779_REFMUX_INTREF,
	AD7779_REFMUX_EXTPWR_AVDD1X_AVSSX,
	AD7779_REFMUX_EXTREF_INVERSE
}ad7779_ref_mux;

typedef enum
{
	AD7779_MTRMUX_280mV = 0x02,
	AD7779_MTRMUX_EXTREF = 0x03,
	AD7779_MTRMUX_EXTREF_INVERSE = 0x04,
	AD7779_MTRMUX_EXTREF_ALLNEG = 0x05,
	AD7779_MTRMUX_INTREF = 0x06,
	AD7779_MTRMUX_INTREF_INVERSE = 0x07,
	AD7779_MTRMUX_INTREF_ALLPOS = 0x08,
	AD7779_MTRMUX_EXTREF_ALLPOS = 0x09
}ad7779_mtr_mux;

typedef enum
{
	AD7779_HIGH_RES,
	AD7779_LOW_PWR,
} ad7779_pwr_mode;

typedef enum
{
	AD7779_EXT_REF,
	AD7779_INT_REF,
} ad7779_ref_type;

typedef enum
{
	AD7779_AUXAINP_AUXAINN,
	AD7779_DVBE_AVSSX,
	AD7779_REF1P_REF1N,
	AD7779_REF2P_REF2N,
	AD7779_REF_OUT_AVSSX,
	AD7779_VCM_AVSSX,
	AD7779_AREG1CAP_AVSSX_ATT,
	AD7779_AREG2CAP_AVSSX_ATT,
	AD7779_DREGCAP_DGND_ATT,
	AD7779_AVDD1A_AVSSX_ATT,
	AD7779_AVDD1B_AVSSX_ATT,
	AD7779_AVDD2A_AVSSX_ATT,
	AD7779_AVDD2B_AVSSX_ATT,
	AD7779_IOVDD_DGND_ATT,
	AD7779_AVDD4_AVSSX,
	AD7779_DGND_AVSS1A_ATT,
	AD7779_DGND_AVSS1B_ATT,
	AD7779_DGND_AVSSX_ATT,
	AD7779_AVDD4_AVSSX_ATT,
	AD7779_REF1P_AVSSX,
	AD7779_REF2P_AVSSX,
	AD7779_AVSSX_AVDD4_ATT,
} ad7779_sar_mux;

// Hardware
void AD7779_Interface_Initialize(void); // Initialize the GPIO/SPI/EXTI for AD7779
void AD7779_HardwareReset(void); // Hardware reset AD7779 by output a reset signal
uint8_t AD7779_ReadRegister(uint16_t address); // Read the register from AD7779
void AD7779_WriteRegister(uint8_t address, uint8_t data); // Write the register from AD7779

// Collection
void AD7779_Collection_Start(void); // Enable the interrupt for start collection from AD7779
void AD7779_Collection_Stop(void); // Disable the interrupt for stop collection from AD7779

// Communication and transmission
void AD7779_Set_SPIOperationMode(ad7779_spi_op_mode mode); // Set the SPI operation mode

// Power mode
void AD7779_Set_PowerMode(ad7779_pwr_mode mode); // Set the power mode of AD7779
ad7779_pwr_mode AD7779_Get_PowerMode(void); // Get the power mode of AD7779

// Output data rate(ODR)
void AD7779_Set_OutputRate(double odr_kHz); // Set the output data rate (sample rate) by SRC_N/SRC_IF

// Output format/DCLK frequency division
void AD7779_Set_OutputFormat(ad7779_dout_format format); // Set DOUT format
void AD7779_Set_OutputHeaderFormat(ad7779_dout_header_format format); // Set DOUT header format
void AD7779_Set_DCLKDivision(ad7779_dclk_div div); // Set DCLK frequency division

// Main ADC(Σ-Δ) reference/multiplexing control
void AD7779_Set_SigmaDelta_ReferenceMultiplexing(ad7779_ref_mux mux); // Set Σ-Δ reference multiplexing
void AD7779_Set_SigmaDelta_ADCMultiplexing(ad7779_mtr_mux mux); // Set Σ-Δ ADC multiplexing

// Global SAR diagnosis multiplexing
void AD7779_Set_SAR_Multiplexing(ad7779_sar_mux mux); // Set SAR ADC multiplexing

// GPIOs
void AD7779_GPIO_SetMode(uint8_t gpio_pin, uint8_t gpio_mode); // Set GPIO mode
void AD7779_GPIO_WritePin(uint8_t gpio_pin, GPIO_PinState state); // Write GPIO pin
GPIO_PinState AD7779_GPIO_ReadPin(uint8_t gpio_pin); // Read GPIO pin

// General errors
uint16_t AD7779_Get_GeneralError(void); // Get general error flags
void AD7779_Set_GeneralErrorCheckEnable(uint16_t error_en, ad7779_ldo_psm_test_en psm_en, ad7779_ldo_psm_trip_test_en psm_trip_en); // Enable/disable the general error check

// Channel enable/disable
void AD7779_EnableChannel(ad7779_ch channel); // Enable the channel
void AD7779_DisableChannel(ad7779_ch channel); // Disable the channel
void AD7779_Set_ChannelEnable(ad7779_ch channel, ad7779_state state); // Enable/disable channel

// Channel special function
void AD7779_Set_ChannelPGAGain(ad7779_ch channel, ad7779_gain gain); // Set the PGA (programmable gain amplifier) gain of channel
void AD7779_Set_ChannelReferenceMonitor(ad7779_ch channel, ad7779_state state); // Enable/disable reference moniter function of channel
void AD7779_Set_ChannelMuxRxMode(ad7779_ch channel, ad7779_state state); // Enable/disable multiplexing RX mode

// Channel synchrony offset
void AD7779_Set_ChannelSyncOffset(ad7779_ch channel, uint8_t offset); // Set the synchrony offset of channel

// Channel offset/gain 
void AD7779_Set_ChannelOffset(ad7779_ch channel, uint32_t offset); // Set the offset of channel
void AD7779_Set_ChannelGain(ad7779_ch channel, uint32_t gain); // Set the gain of channel

// Channel errors
uint8_t AD7779_Get_ChannelError(ad7779_ch channel); // Get the error status of channel
uint8_t AD7779_Get_ChannelDSPError(ad7779_ch channel); // Get the DSP error status of channel
void AD7779_Set_ChannelErrorCheckEnable(uint8_t error_en); // Enable/disable the channel error check

// Get channels value in interrupt
void AD7779_Get_SigmaDelta_Value(int32_t* values); // Get all channels of Σ-Δ ADC's values
void AD7779_Get_SigmaDelta_Original(uint32_t* datas); // Get all channels of Σ-Δ ADC's original data return from SPI
