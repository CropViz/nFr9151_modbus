#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

// Pines RS485: ajusta los alias o pines reales según tu overlay

#define RS485_NODE DT_PATH(zephyr_user)
static const struct gpio_dt_spec re_pin = GPIO_DT_SPEC_GET(RS485_NODE, re_gpios);
static const struct gpio_dt_spec de_pin = GPIO_DT_SPEC_GET(RS485_NODE, de_gpios);

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart1));

uint8_t requestCommand[] = {0x11, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC6, 0x9B};

// #define RECEIVE_BUFF_SIZE 10
// static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};
// #define RECEIVE_TIMEOUT 100

// static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
// {
// 	switch (evt->type)
// 	{
// 	case UART_RX_RDY:
// 		if ((evt->data.rx.len) == 1)
// 		{
// 			printk("Received %d bytes: ", evt->data.rx.len);
// 			for (int i = 0; i < evt->data.rx.len; i++)
// 			{
// 				printk("%02X ", evt->data.rx.buf[i]);
// 			}
// 			printk("\n");
// 		}
// 		break;
// 	case UART_RX_DISABLED:
// 		uart_rx_enable(dev, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
// 		break;
// 	default:
// 		break;
// 	}
// }

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

	// /* STEP 8 - Register the UART callback function */
	// ret = uart_callback_set(uart, uart_cb, NULL);
	// if (ret)
	// {
	// 	return 1;
	// }
	// /* STEP 9.2 - Send the data over UART by calling uart_tx() */

	// /* STEP 10.3  - Start receiving by calling uart_rx_enable() and pass it the address of the
	//  * receive  buffer */
	// ret = uart_rx_enable(uart, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
	// if (ret)
	// {
	// 	return 1;
	// }

	while (1)
	{
		gpio_pin_set_dt(&re_pin, 1); // RE HIGH (receiver disabled)
		gpio_pin_set_dt(&de_pin, 1); // DE HIGH (transmit enabled)
		k_msleep(200);
		ret = uart_tx(uart, requestCommand, sizeof(requestCommand), SYS_FOREVER_MS);
		k_msleep(10);				 // 20
		gpio_pin_set_dt(&re_pin, 0); // RE HIGH (receiver disabled)
		gpio_pin_set_dt(&de_pin, 0); // DE HIGH (transmit enabled)

		if (ret)
		{
			printk("Error sending data: %d\n", ret);
			return 1;
		}

		k_msleep(8000); // Wait before next request
	}
}
