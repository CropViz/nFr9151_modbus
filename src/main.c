#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

// Pines RS485: ajusta los alias o pines reales según tu overlay

#define RS485_NODE DT_PATH(zephyr_user)
#define RESPONSE_SIZE 9
#define RECEIVE_TIMEOUT 100 // milliseconds

static const struct gpio_dt_spec re_pin = GPIO_DT_SPEC_GET(RS485_NODE, re_gpios);
static const struct gpio_dt_spec de_pin = GPIO_DT_SPEC_GET(RS485_NODE, de_gpios);

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart1));

uint8_t requestCommand[] = {0x11, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC6, 0x9B};

static uint8_t rx_buf[12];
static uint8_t sensor_data[RESPONSE_SIZE];
static size_t sensor_data_len = 0;
static bool full_response_received = false;

// // Función para calcular el CRC16 Modbus
// uint16_t ModRTU_CRC(uint8_t *buf, int len)
// {
// 	uint16_t crc = 0xFFFF;

// 	for (int pos = 0; pos < len; pos++)
// 	{
// 		crc ^= (uint16_t)buf[pos];

// 		for (int i = 8; i != 0; i--)
// 		{
// 			if ((crc & 0x0001) != 0)
// 			{
// 				crc >>= 1;
// 				crc ^= 0xA001;
// 			}
// 			else
// 			{
// 				crc >>= 1;
// 			}
// 		}
// 	}
// 	return crc;
// }

// Calcula el CRC16 (MODBUS) de un arreglo de bytes
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
			full_response_received = true;
			// uart_rx_disable(dev); // Optional: stop receiving after full response
			uart_rx_enable(dev, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);
		}
		break;
	}
	case UART_RX_DISABLED:
		// Restart reception
		uart_rx_enable(dev, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);
		break;

	case UART_RX_BUF_REQUEST:
		// Provide a new buffer if needed
		break;

	case UART_RX_BUF_RELEASED:
		break;

	case UART_RX_STOPPED:
		// printk("UART RX stopped due to error: %d\n", evt->data.rx_stop.reason);
		// uart_rx_enable(dev, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);
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
	ret = gpio_pin_configure_dt(&de_pin, GPIO_OUTPUT_INACTIVE);	 // DE LOW initially
	ret |= gpio_pin_configure_dt(&re_pin, GPIO_OUTPUT_INACTIVE); // RE LOW (receiver enabled)

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
		gpio_pin_set_dt(&re_pin, 1); // RE HIGH (receiver disabled)
		gpio_pin_set_dt(&de_pin, 1); // DE HIGH (transmit enabled)
		k_msleep(200);
		ret = uart_tx(uart, requestCommand, sizeof(requestCommand), SYS_FOREVER_MS);
		k_msleep(10);				 // 20
		gpio_pin_set_dt(&re_pin, 0); // RE HIGH (receiver disabled)
		gpio_pin_set_dt(&de_pin, 0); // DE HIGH (transmit enabled)

		if (full_response_received)
		{
			printk("Modbus RTU response:\n");
			for (int i = 0; i < RESPONSE_SIZE; i++)
			{
				printk("0x%02X ", sensor_data[i]);
			}
			printk("\n");

			// // Calcular y verificar CRC
			// uint16_t receivedCRC = (sensor_data[8] << 8) | sensor_data[7]; // orden inverso
			// uint16_t calculatedCRC = calc_crc16(sensor_data, 9);		   // CRC de primeros 7 bytes

			// if (receivedCRC != calculatedCRC)
			// {
			// 	printk("CRC check failed.");
			// }

			// Reset for next read
			full_response_received = false;
			sensor_data_len = 0;

			// // Re-enable reception if disabled
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
		printk("-------------------\n");
		k_msleep(8000); // Wait before next request
	}
}
