#include "usart_helpers.h"

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#define I2C I2C1
#define CAM_I2C_ADDRESS (0xBA >> 1)


// working config:
// i2c  @ 10kHz
// address 0xBA

static void cam_i2c_hwinit(void)
{
	const uint32_t SCL_PORT = GPIOB;
	const uint32_t SCL_PIN = GPIO6;
	const uint32_t SCL_AF = GPIO_AF4;
	const uint32_t SDA_PORT = GPIOB;
	const uint32_t SDA_PIN = GPIO7;
	const uint32_t SDA_AF = GPIO_AF4;

	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_I2C1EN);

//	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, GPIO8);//SCL
//	gpio_set_af(GPIOA, GPIO_AF4, GPIO8);
//	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);

//	gpio_set_output_options(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, GPIO9);//SDA
//	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
//	gpio_set_af(GPIOC, GPIO_AF4, GPIO9);

	gpio_set_output_options(SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, SCL_PIN);//SCL
	gpio_mode_setup(SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SCL_PIN);
	gpio_set_af(SCL_PORT, SCL_AF, SCL_PIN);

	gpio_set_output_options(SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, SDA_PIN);//SDA
	gpio_mode_setup(SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SDA_PIN);
	gpio_set_af(SDA_PORT, SDA_AF, SDA_PIN);
}

static void tim3_setup(void)
{
//	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO8);
//	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
//	gpio_set_af(GPIOC, GPIO_AF2, GPIO8);

	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO9);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOC, GPIO_AF2, GPIO9);

	timer_reset(TIM3);
	timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_TOGGLE);
	timer_enable_oc_output(TIM3, TIM_OC4);
	timer_set_oc_polarity_high(TIM3, TIM_OC4);
	timer_set_oc_value(TIM3, TIM_OC4, 5);
	timer_set_prescaler(TIM3, 0);	// 42Mhz/10000hz - 1
	timer_set_period(TIM3, 11);
	timer_enable_counter(TIM3);
}


static void cam_i2c_init(void)
{
	const int frequency = 10000;
	cam_i2c_hwinit();

	i2c_reset(I2C);

	i2c_peripheral_disable(I2C);

	i2c_set_clock_frequency(I2C, I2C_CR2_FREQ_21MHZ);

	uint32_t ccr = rcc_apb1_frequency/(frequency*2);
	i2c_set_ccr(I2C, ccr);

	i2c_peripheral_enable(I2C);
}

static void cam_i2c_write(uint32_t i2c, uint8_t regaddr, const uint8_t *data)
{
	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Send destination address. */
	i2c_send_7bit_address(i2c, CAM_I2C_ADDRESS, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	(void)I2C_SR2(i2c);

	/* Sending the register address. */
	i2c_send_data(i2c, regaddr);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF)); /* Await ByteTransferedFlag. */

	/* Sending the data */
	i2c_send_data(i2c, data[0]);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));

	/* Send STOP condition. */
	i2c_send_stop(i2c);
}

static void cam_i2c_read(uint8_t regaddr, uint8_t *data)
{
	LOG_PRINTF_S("Hey: %04X %04X %04X %04X\n", I2C_SR1(I2C), I2C_SR2(I2C), I2C_CR1(I2C), I2C_CR2(I2C));
	/* Send START condition. */
	i2c_send_start(I2C);
//	delay_ms_busy_loop(1);
	LOG_PRINTF_S("Y3\n");
	/* Waiting for START is send and switched to master mode. */
	while (!(I2C_SR1(I2C) & I2C_SR1_SB)) {
		LOG_PRINTF_S("d\n");
	}
//LOG_PRINTF_S("DD\n");
	(void)I2C_SR1(I2C);

	i2c_send_7bit_address(I2C, CAM_I2C_ADDRESS, I2C_WRITE);
//LOG_PRINTF_S("DX\n");
	while (!(I2C_SR1(I2C) & I2C_SR1_ADDR));LOG_PRINTF_S("Hey: %04X %04X %04X %04X\n", I2C_SR1(I2C), I2C_SR2(I2C), I2C_CR1(I2C), I2C_CR2(I2C));
LOG_PRINTF_S("DA\n");
	/* Cleaning ADDR condition sequence. */
	(void)I2C_SR2(I2C);
	i2c_send_data(I2C, regaddr);
	while (!(I2C_SR1(I2C) & (I2C_SR1_BTF | I2C_SR1_TxE)));

//	LOG_PRINTF_S("Y1\n");
	/* Send START condition. */
	i2c_send_start(I2C);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(I2C) & I2C_SR1_SB)));
//		&& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	i2c_send_7bit_address(I2C, CAM_I2C_ADDRESS, I2C_READ);
	LOG_PRINTF_S("a2");

//	LOG_PRINTF_S("X1\n");
	/* Waiting for address is transferred. */
	while (!(I2C_SR1(I2C) & I2C_SR1_ADDR)) {
//		delay_ms_busy_loop(1000);
//		LOG_PRINTF_S("starting...%04X %04X %04X %04X\n", I2C_SR1(i2c), I2C_SR2(i2c), I2C_CR1(i2c), I2C_CR2(i2c));
	}
	/* Cleaning ADDR condition sequence. */
	(void)I2C_SR2(I2C);
	LOG_PRINTF_S("a1");
	i2c_enable_ack(I2C);

	/* Now the slave should begin to send us the first byte. Await BTF. */
	while (!(I2C_SR1(I2C) & (I2C_SR1_RxNE)));

	data[0] = i2c_get_data(I2C);

	// Stop before last byte is received
	i2c_send_stop(I2C);

//	LOG_PRINTF_S("X\n");
	/* Now the slave should begin to send us the first byte. Await BTF. */
	while (!(I2C_SR1(I2C) & (I2C_SR1_RxNE)));

	data[1] = i2c_get_data(I2C);
}

/* Set STM32 to 168 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

	// GPIO
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);

	// periphery
	rcc_periph_clock_enable(RCC_USART6); // USART
	rcc_periph_clock_enable(RCC_I2C3);
	rcc_periph_clock_enable(RCC_TIM3);
}

void camera_main()
{
	clock_setup();

	// USART TX
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
	gpio_set_af(GPIOC, GPIO_AF8, GPIO6 | GPIO7);

	/* Set GPIO12-15 (in GPIO port D) to 'output push-pull'. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);

		delay_ms_busy_loop(100);

	usart_init(USART6, 921600);
	tim3_setup();
		delay_ms_busy_loop(1000);

int i = 0;
//while (1) {
	cam_i2c_init();

		delay_ms_busy_loop(500);

//	LOG_PRINTF_S("going to read %d\n", i++);

	uint16_t data = 0;
	cam_i2c_read(0x01, &data);

	LOG_PRINTF_S("CAM_DATA: %04X\n", data)

	LOG_FLUSH();
//}
	while(1);
}
