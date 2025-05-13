
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

/**
 * Macro definition for the RS485 node path in Zephyr.
 */
#define RS485_NODE DT_PATH(zephyr_user)
#define RESPONSE_SIZE 9
#define RECEIVE_TIMEOUT 100 // milliseconds

/**
 * Defines GPIO pin specifications for RS485 communication.
 *
 * The re_pin specifies the GPIO pin for RS485 receive enable.
 * The de_pin specifies the GPIO pin for RS485 driver enable.
 */
static const struct gpio_dt_spec re_pin = GPIO_DT_SPEC_GET(RS485_NODE, re_gpios);
static const struct gpio_dt_spec de_pin = GPIO_DT_SPEC_GET(RS485_NODE, de_gpios);

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart1));

/**
 * Array representing a request command for read temperature an relative humidity.
 */
uint8_t requestCommand[] = {0x11, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC6, 0x9B};

static uint8_t rx_buf[12];
static uint8_t sensor_data[RESPONSE_SIZE];
static size_t sensor_data_len = 0;

/**
 * Define a semaphore with initial count and maximum count.
 *
 * @param rx_sem The name of the semaphore.
 * @param initial_count The initial count of the semaphore.
 * @param max_count The maximum count the semaphore can reach.
 *
 * @returns None
 */
K_SEM_DEFINE(rx_sem, 0, 1);
K_SEM_DEFINE(tx_sem, 1, 1); // Semaphore for transmission, initialized as available

/**
 * Calculates the CRC-16 checksum for the given data.
 *
 * @param data Pointer to the data array.
 * @param length Length of the data array.
 *
 * @returns The CRC-16 checksum value.
 */
uint16_t calc_crc16(const uint8_t *data, uint8_t length)
{
	uint16_t crc = 0xFFFF;
	uint16_t c = 0xFFFF;

	for (uint8_t i = 0; i < length; i++)
	{
		c = data[i] & 0xFFFF;
		crc ^= c;
		for (uint8_t j = 0; j < 8; j++)
		{
			if (crc & 0x0001)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}

	return crc;
}

/**
 * Callback function for UART events handling.
 *
 * @param dev Pointer to the UART device structure.
 * @param evt Pointer to the UART event structure.
 * @param user_data Pointer to user data.
 *
 * @returns None
 */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type)
	{
	case UART_RX_RDY:
	{
		// Append new bytes to sensor_data buffer
		const uint8_t *data = &evt->data.rx.buf[evt->data.rx.offset];
		size_t len = evt->data.rx.len;

		for (size_t i = 0; i < len && sensor_data_len < RESPONSE_SIZE; i++)
		{
			sensor_data[sensor_data_len++] = data[i];
		}

		if (sensor_data_len >= RESPONSE_SIZE)
		{
			k_sem_give(&rx_sem);  // Señal al hilo principal
			uart_rx_disable(dev); // Deshabilitar la recepción
		}
		break;
	}
	case UART_RX_DISABLED:
		// Restart reception
		uart_rx_enable(dev, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);
		break;

	case UART_RX_BUF_REQUEST:

		break;

	case UART_RX_BUF_RELEASED:
		break;

	case UART_RX_STOPPED:
		printk("UART RX stopped\n");
		break;

	default:
		break;
	}
}

int main(void)
{
	int ret;
	if (!gpio_is_ready_dt(&de_pin) || !gpio_is_ready_dt(&re_pin))
	{
		printk("GPIO not ready\n");
		return 0;
	}
	ret = gpio_pin_configure_dt(&de_pin, GPIO_OUTPUT_INACTIVE);	 // DE LOW
	ret |= gpio_pin_configure_dt(&re_pin, GPIO_OUTPUT_INACTIVE); // RE LOW

	if (ret < 0)
	{
		printk("GPIO config failed\n");
		return 0;
	}

	if (!device_is_ready(uart))
	{
		printk("UART device not ready\r\n");
		return 1;
	}
	ret = uart_callback_set(uart, uart_cb, NULL);
	if (ret)
	{
		printk("Failed to set UART callback\n");
		return 1;
	}
	ret = uart_rx_enable(uart, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);
	if (ret)
	{
		printk("Failed to enable UART RX\n");
		return 1;
	}

	while (1)
	{
		// Acquire transmission semaphore before starting
		k_sem_take(&tx_sem, K_FOREVER);  // Wait indefinitely for permission to transmit

		gpio_pin_set_dt(&re_pin, 1); // RE HIGH
		gpio_pin_set_dt(&de_pin, 1); // DE HIGH
		k_msleep(200);
		ret = uart_tx(uart, requestCommand, sizeof(requestCommand), SYS_FOREVER_MS);
		k_msleep(10);
		gpio_pin_set_dt(&re_pin, 0); // RE LOW
		gpio_pin_set_dt(&de_pin, 0); // DE LOW

		if (k_sem_take(&rx_sem, K_MSEC(1000))) // full_response_received
		{
			printk("Modbus RTU response:\n");
			for (int i = 0; i < RESPONSE_SIZE; i++)
			{
				printk("0x%02X ", sensor_data[i]);
			}
			printk("\n");

			sensor_data_len = 0;

			uart_rx_enable(uart, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);

			int16_t raw_temperature = sensor_data[3] << 8 | sensor_data[4];
			uint16_t raw_humidity = sensor_data[5] << 8 | sensor_data[6];
			float temperature = raw_temperature / 100.0;
			float humidity = raw_humidity / 100.0;
			int temp_int = (int)temperature;							// Integer part
			int temp_frac = (int)((temperature - temp_int) * 100);		// Fractional part
			int moisture_int = (int)humidity;							// Integer part
			int moisture_frac = (int)((humidity - moisture_int) * 100); // Fractional part
			printk("---- Soil Data ----\n");
			printk("Temperature: %d.%02d °C\n", temp_int, temp_frac);
			printk("Moisture: %d.%02d%%\n", moisture_int, moisture_frac);
		}
		else
		{
			printk("No full response received yet.\n");
		}
		k_sem_give(&tx_sem); // Release semaphore after one transmission cycle
		printk("-------------------\n");
		k_msleep(8000); // Wait before next request
	}
}
