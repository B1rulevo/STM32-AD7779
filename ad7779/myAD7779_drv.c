#include "myAD7779_drv.h"

#include "ad7779_regs.h"

#include "spi.h"

#include "string.h"

extern volatile u8 data_fl;

static void AD7779_send(u8 *dat, u16 sz)
{
  CS_LOW();
  HAL_SPI_Transmit(&hspi1, dat, sz, SPI_BASE_TIMEOUT);
  CS_HIGH();
}

static void AD7779_receive(u8 *dat, u16 sz)
{
  CS_LOW();
  HAL_SPI_Receive(&hspi1, dat, sz, SPI_BASE_TIMEOUT);
  CS_HIGH();
}

static void ad7779_read_write(u8 writ_byte, u8 *read_byte)
{
  HAL_SPI_TransmitReceive(&hspi1, &writ_byte, read_byte, 1, SPI_BASE_TIMEOUT);
}

static void ad7779_read_write_16bit(u8 writ_byte, u8 *read_byte)
{
  HAL_SPI_TransmitReceive(&hspi1, &writ_byte, read_byte, 1, SPI_BASE_TIMEOUT);
}

static inline void AD7779_ReadWrite(u16 write, u16 *read)
{
  HAL_SPI_TransmitReceive(&hspi1, (u8 *)&write, (u8 *)read, 1, SPI_BASE_TIMEOUT);
}

static void ad7779_write_reg(u8 reg, u8 dat)
{
  u8 tx[2];

  tx[0] = reg & 0x7F;      // MSB = 0 → WRITE
  tx[1] = dat;

  AD7779_send(tx, 2);
}

static void AD7779_WriteRegister(u16 address, u8 data)
{
  u16 dummy;
  CS_LOW();
  AD7779_ReadWrite( ((address << 8) + data) , &dummy);
  CS_HIGH();
}

//static void AD7779_ReadRegister(uint16_t address, u8 *data)
//{
//  CS_LOW();
//  AD7779_ReadWrite( ((address << 8) | 0x8000), data);
//  CS_HIGH();
//}

static void AD7779_ReadRegister(uint16_t address, uint8_t *data)
{
  uint16_t rx;

  CS_LOW();
  AD7779_ReadWrite((address << 8) | 0x8000, &rx);
  CS_HIGH();

  *data = (uint8_t)(rx & 0xFF);
}

static void ad7779_read_reg(u8 reg, u8 *buf)
{
  u8 tx[2];
  u8 rx[2];

  tx[0] = reg | 0x80;      // MSB = 1 → READ
  tx[1] = 0x00;

  CS_LOW();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, SPI_BASE_TIMEOUT);
  CS_HIGH();

  *buf = rx[1];
}

static void ad7779_read_reg_16bit(u16 reg, u16 *buf)
{
  u16 val;

  CS_LOW();
  AD7779_ReadWrite( ((reg << 8) | 0x8000), &val);
  val = ( val & 0x00FF);
  CS_HIGH();

  *buf = val;
}

static void AD7779_Set_PowerMode(ad7779_pwr_mode mode)
{
  u8 cfg;
  AD7779_ReadRegister(AD7779_REG_GENERAL_USER_CONFIG_1, &cfg);

  if (mode == AD7779_HIGH_RES) {
    cfg |= AD7779_MOD_POWERMODE;
  } else {
    cfg &= ~AD7779_MOD_POWERMODE;
  }

  AD7779_WriteRegister(AD7779_REG_GENERAL_USER_CONFIG_1, cfg);
}

ad7779_pwr_mode AD7779_Get_PowerMode(void)
{
  u8 config;
  AD7779_ReadRegister(AD7779_REG_GENERAL_USER_CONFIG_1, &config);

  if (AD7779_FLAGCHECK(config, AD7779_MOD_POWERMODE)) return AD7779_HIGH_RES;
  else return AD7779_LOW_PWR;
}

//static void AD7779_Set_SigmaDelta_ADCMultiplexing(ad7779_mtr_mux mux)
//{
//  uint8_t config = AD7779_ReadRegister(AD7779_REG_ADC_MUX_CONFIG);
//
//  config &= ~AD7779_MTR_MUX_CTRL(0x0F);
//  config |= AD7779_MTR_MUX_CTRL(mux);
//
//  AD7779_WriteRegister(AD7779_REG_ADC_MUX_CONFIG, config);
//}

static void AD7779_Set_SPIOperationMode(ad7779_spi_op_mode mode)
{
  u8 gen2_data, gen3_data;
  AD7779_ReadRegister(AD7779_REG_GENERAL_USER_CONFIG_2, &gen2_data); //AD7779_REG_GENERAL_USER_CONFIG_2
  AD7779_ReadRegister(AD7779_REG_GENERAL_USER_CONFIG_3, &gen3_data); //AD7779_REG_GENERAL_USER_CONFIG_3

  switch (mode) {
  case AD7779_INT_REG:
    gen2_data &= ~AD7779_SAR_DIAG_MODE_EN;
    gen3_data &= ~AD7779_SPI_SLAVE_MODE_EN;
    break;
  case AD7779_SD_CONV:
    gen2_data &= ~AD7779_SAR_DIAG_MODE_EN;
    gen3_data |= AD7779_SPI_SLAVE_MODE_EN;
    break;
  case AD7779_SAR_CONV:
    gen2_data |= AD7779_SAR_DIAG_MODE_EN;
    gen3_data &= ~AD7779_SPI_SLAVE_MODE_EN;
    break;
  }

  AD7779_WriteRegister(AD7779_REG_GENERAL_USER_CONFIG_2, gen2_data);
  AD7779_WriteRegister(AD7779_REG_GENERAL_USER_CONFIG_3, gen3_data);
}

static void AD7779_Set_ChannelGain(ad7779_ch channel, uint32_t gain)
{
  uint8_t lower, mid, upper;
  lower = gain & 0x0000FF;
  mid = (gain & 0x00FF00) >> 8;
  upper = (gain & 0xFF0000) >> 16;

  AD7779_WriteRegister(AD7779_REG_CH_GAIN_UPPER_BYTE(channel), upper);
  AD7779_WriteRegister(AD7779_REG_CH_GAIN_MID_BYTE(channel)  , mid  );
  AD7779_WriteRegister(AD7779_REG_CH_GAIN_LOWER_BYTE(channel), lower);
}

static void AD7779_Set_ChannelOffset(ad7779_ch channel, uint32_t offset) {
  uint8_t lower, mid, upper;
  lower = offset & 0x0000FF;
  mid = (offset & 0x00FF00) >> 8;
  upper = (offset & 0xFF0000) >> 16;

  AD7779_WriteRegister(AD7779_REG_CH_OFFSET_UPPER_BYTE(channel), upper);
  AD7779_WriteRegister(AD7779_REG_CH_OFFSET_MID_BYTE(channel), mid);
  AD7779_WriteRegister(AD7779_REG_CH_OFFSET_LOWER_BYTE(channel), lower);
}

void AD7779_Set_OutputRate(double odr_kHz)
{
  uint8_t src_n_lower = 0x00, src_n_upper = 0x00;
  uint8_t src_if_lower = 0x00, src_if_upper = 0x00;

  uint16_t src_n = 0x0000, src_if = 0x0000;

  // Get the fmod from mode register
  // High-precision = 2048, Low-power = 512
  double f_mod = 512.0;
  if (AD7779_Get_PowerMode() == AD7779_HIGH_RES) f_mod = 2048.0;

  // Calculate the SRC value
  double n = f_mod / odr_kHz;

  src_n = (uint16_t)n; // Convert to integer for floor
  src_if = (uint16_t)((n - (double)src_n) * 65536.0); // Convert the decimal part

  // Split lower/upper bits
  src_n_lower = src_n & 0x00FF;
  src_n_upper = (src_n & 0xFF00) >> 8;

  src_if_lower = src_if & 0x00FF;
  src_if_upper = (src_if & 0xFF00) >> 8;

  // Write registers
  AD7779_WriteRegister(AD7779_REG_SRC_N_MSB, src_n_upper);
  AD7779_WriteRegister(AD7779_REG_SRC_N_LSB, src_n_lower);
  AD7779_WriteRegister(AD7779_REG_SRC_IF_MSB, src_if_upper);
  AD7779_WriteRegister(AD7779_REG_SRC_IF_LSB, src_if_lower);

  // Update
  AD7779_WriteRegister(AD7779_REG_SRC_UPDATE, 0x01);
  HAL_Delay(1);
  AD7779_WriteRegister(AD7779_REG_SRC_UPDATE, 0x00);
}

static void AD7779_Set_SigmaDelta_ReferenceMultiplexing(ad7779_ref_mux mux)
{
  uint8_t config;
  AD7779_ReadRegister(AD7779_REG_ADC_MUX_CONFIG, &config);

  config &= ~AD7779_REF_MUX_CTRL(0x03);
  config |= AD7779_REF_MUX_CTRL(mux);

  AD7779_WriteRegister(AD7779_REG_ADC_MUX_CONFIG, config);
}

static void AD7779_Set_SigmaDelta_ADCMultiplexing(ad7779_mtr_mux mux)
{
  uint8_t config;
  AD7779_ReadRegister(AD7779_REG_ADC_MUX_CONFIG, &config);

  config &= ~AD7779_MTR_MUX_CTRL(0x0F);
  config |= AD7779_MTR_MUX_CTRL(mux);

  AD7779_WriteRegister(AD7779_REG_ADC_MUX_CONFIG, config);
}

void AD7779_Set_ChannelMuxRxMode(ad7779_ch channel, ad7779_state state) {
  uint8_t config;
  AD7779_ReadRegister(AD7779_REG_CH_CONFIG(channel), &config);

  if (state == AD7779_ENABLE) config |= AD7779_CH_RX;
  else config &= ~AD7779_CH_RX;

  AD7779_WriteRegister(AD7779_REG_CH_CONFIG(channel), config);
}

void ad7779_init(void)
{
    /* 1. Аппаратный RESET */
  HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);                       // ≥10 мкс, даём с запасом
  HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(2);                       // время выхода из reset

  u8 dummy;
  AD7779_ReadRegister(AD7779_REG_GEN_ERR_REG_2, &dummy);

  AD7779_Set_SigmaDelta_ReferenceMultiplexing(AD7779_REFMUX_EXTREF);
  AD7779_Set_SigmaDelta_ADCMultiplexing(AD7779_MTRMUX_280mV);

  AD7779_Set_ChannelMuxRxMode(AD7779_CH0, AD7779_DISABLE);
  AD7779_Set_ChannelMuxRxMode(AD7779_CH1, AD7779_DISABLE);
  AD7779_Set_ChannelMuxRxMode(AD7779_CH2, AD7779_DISABLE);
  AD7779_Set_ChannelMuxRxMode(AD7779_CH3, AD7779_DISABLE);

//  AD7779_Set_ChannelMuxRxMode(AD7779_CH0, AD7779_ENABLE);
//  AD7779_Set_ChannelMuxRxMode(AD7779_CH1, AD7779_ENABLE);
//  AD7779_Set_ChannelMuxRxMode(AD7779_CH2, AD7779_ENABLE);
//  AD7779_Set_ChannelMuxRxMode(AD7779_CH3, AD7779_ENABLE);

  AD7779_Set_PowerMode(AD7779_HIGH_RES);

//  AD7779_Set_ChannelOffset(AD7779_CH0, 0xFFFFFF - 184);
//  AD7779_Set_ChannelOffset(AD7779_CH1, 0);
//  AD7779_Set_ChannelOffset(AD7779_CH2, 0xFFFFFF - 111);
//  AD7779_Set_ChannelOffset(AD7779_CH3, 0xFFFFFF - 130);
//  AD7779_Set_ChannelOffset(AD7779_CH4, 0xFFFFFF - 149);
//  AD7779_Set_ChannelOffset(AD7779_CH5, 0xFFFFFF - 104);
//  AD7779_Set_ChannelOffset(AD7779_CH6, 0xFFFFFF - 110);
//  AD7779_Set_ChannelOffset(AD7779_CH7, 0xFFFFFF - 184);

  AD7779_Set_ChannelOffset(AD7779_CH0, 0);
  AD7779_Set_ChannelOffset(AD7779_CH1, 0);
  AD7779_Set_ChannelOffset(AD7779_CH2, 0);
  AD7779_Set_ChannelOffset(AD7779_CH3, 0);
  AD7779_Set_ChannelOffset(AD7779_CH4, 0);
  AD7779_Set_ChannelOffset(AD7779_CH5, 0);
  AD7779_Set_ChannelOffset(AD7779_CH6, 0);
  AD7779_Set_ChannelOffset(AD7779_CH7, 0);

  AD7779_Set_ChannelGain(AD7779_CH0, 0x555555);
  AD7779_Set_ChannelGain(AD7779_CH1, 0x555555);
  AD7779_Set_ChannelGain(AD7779_CH2, 0x555555);
  AD7779_Set_ChannelGain(AD7779_CH3, 0x555555);
  AD7779_Set_ChannelGain(AD7779_CH4, 0x555555);
  AD7779_Set_ChannelGain(AD7779_CH5, 0x555555);
  AD7779_Set_ChannelGain(AD7779_CH6, 0x555555);
  AD7779_Set_ChannelGain(AD7779_CH7, 0x555555);



//  AD7779_Set_SigmaDelta_ADCMultiplexing(AD7779_MTRMUX_EXTREF);
  AD7779_Set_OutputRate( (double)4.0 );

  AD7779_Set_SPIOperationMode(AD7779_SD_CONV);

  data_fl = 0;
}

void AD7779_Get_SigmaDelta_Value(int32_t * values)
{
  uint16_t data[16];
  uint16_t dummy;

  CS_LOW();
  AD7779_ReadWrite(0x8000, &dummy);
  for (uint8_t i = 0; i < 16; i++) AD7779_ReadWrite(0x8000, &data[i]);
  CS_HIGH();

  for (uint8_t ch = 0; ch < 8; ch++)
  {
    uint8_t pointer = ch << 1;
    values[ch] = (((int32_t)data[pointer] & 0x00FF) << 16) + (data[pointer + 1]);
    if ((data[pointer] & 0x0080) > 0) values[ch] | 0xFF000000;
  }
}

void AD7779_Get_SigmaDelta_Original(int32_t *datas)
{
    uint16_t raw[16];   // 17 слов = 17 байт данных (в младших 8 битах)

    CS_LOW();
    for (uint8_t i = 0; i < 16; i++) {
        AD7779_ReadWrite(0x8000, &raw[i]);
    }
    CS_HIGH();

    // для отладки — оставим
    uint8_t test_bytes[32];
    memcpy(test_bytes, (uint8_t *)raw, 32);

    for (uint8_t ch = 0; ch < 8; ch++) {

        // Индекс первого байта канала
        uint8_t base = ch * 2;

        // Достаём реальные байты
        uint8_t b0 = raw[base + 0] & 0xFF; // D23..16
        uint8_t b1 = (raw[base + 1] & 0xFF00) >> 8; // D15..8
        uint8_t b2 = raw[base + 1] & 0xFF; // D7..0

        // Склеиваем + sign extend через сдвиг
        uint32_t val =
            ((uint32_t)b0 << 24) |
            ((uint32_t)b1 << 16) |
            ((uint32_t)b2 << 8);

        val >>= 8;   // арифметический сдвиг → расширение знака

        if (val & 0x00800000) {
            val |= 0xFF000000;
        }

        datas[ch] = (int32_t)val;
    }
}
