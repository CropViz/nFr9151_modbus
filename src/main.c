#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#define SLEEP_TIME_MS 1000

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define RS485_NODE DT_PATH(zephyr_user)
static const struct gpio_dt_spec re_pin = GPIO_DT_SPEC_GET(RS485_NODE, re_gpios);
static const struct gpio_dt_spec de_pin = GPIO_DT_SPEC_GET(RS485_NODE, de_gpios);

static const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));
static const struct device *uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));

static struct k_sem tx_done_sem_uart0;
static struct k_sem tx_done_sem_uart1;
static struct k_sem rx_ready_sem_uart1;

static uint8_t uart0_tx_buf[] = "Message from UART0\r\n";
static uint8_t uart1_tx_buf[] = {0x11, 0x03, 0x00, 0x00, 0x00, 0x02, 0x45, 0xC8};

#define UART1_RX_BUF_SIZE 64
static uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];

static void uart_cb_uart0(const struct device *dev, struct uart_event *evt, void *user_data)
{
	if (evt->type == UART_TX_DONE)
	{
		k_sem_give(&tx_done_sem_uart0);
	}
}

static void uart_cb_uart1(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type)
	{
	case UART_TX_DONE:
		k_sem_give(&tx_done_sem_uart1);
		break;

	case UART_RX_RDY:
		printk("UART1 Received %d bytes:\n", evt->data.rx.len);
		for (int i = 0; i < evt->data.rx.len; i++)
		{
			printk("0x%02X ", evt->data.rx.buf[i]);
		}
		printk("\n");
		k_sem_give(&rx_ready_sem_uart1);
		break;

	case UART_RX_DISABLED:
		// Restart RX
		uart_rx_enable(uart1, uart1_rx_buf, sizeof(uart1_rx_buf), 100);
		break;

	default:
		break;
	}
}

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&de_pin) || !gpio_is_ready_dt(&re_pin))
	{
		printk("GPIO not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	ret |= gpio_pin_configure_dt(&de_pin, GPIO_OUTPUT_INACTIVE); // DE LOW initially
	ret |= gpio_pin_configure_dt(&re_pin, GPIO_OUTPUT_INACTIVE); // RE LOW (receiver enabled)

	if (ret < 0)
	{
		printk("GPIO config failed\n");
		return 0;
	}

	if (!device_is_ready(uart0) || !device_is_ready(uart1))
	{
		printk("UART devices not ready!\n");
		return 0;
	}

	k_sem_init(&tx_done_sem_uart0, 0, 1);
	k_sem_init(&tx_done_sem_uart1, 0, 1);
	k_sem_init(&rx_ready_sem_uart1, 0, 1);

	uart_callback_set(uart0, uart_cb_uart0, NULL);
	uart_callback_set(uart1, uart_cb_uart1, NULL);

	// Enable RX on UART1
	ret = uart_rx_enable(uart1, uart1_rx_buf, sizeof(uart1_rx_buf), 100);
	if (ret < 0)
	{
		printk("Failed to enable UART1 RX\n");
		return 0;
	}

	while (1)
	{
		// Send data
		gpio_pin_set_dt(&re_pin, 1); // RE HIGH (receiver disabled)
		gpio_pin_set_dt(&de_pin, 1); // DE HIGH (transmit enabled)
		k_msleep(20);
		int ret0 = uart_tx(uart0, uart0_tx_buf, strlen(uart0_tx_buf), SYS_FOREVER_MS);
		int ret1 = uart_tx(uart1, uart1_tx_buf, sizeof(uart1_tx_buf), SYS_FOREVER_MS);

		if (ret0 < 0 || ret1 < 0)
		{
			printk("UART TX failed\n");
			return 0;
		}
		gpio_pin_set_dt(&re_pin, 0); // RE LOW (receiver enabled)
		gpio_pin_set_dt(&de_pin, 0); // DE LOW (transmit disabled)
		k_msleep(20);
		k_sem_take(&tx_done_sem_uart0, K_FOREVER);
		k_sem_take(&tx_done_sem_uart1, K_FOREVER);

		printk("UART0 and UART1 transmissions complete\n");
		// printk("uart1_tx_buf: ");
		// for (size_t i = 0; i < sizeof(uart1_tx_buf); i++)
		// {
		// 	printk("%02X ", uart1_tx_buf[i]);
		// }
		// printk("\n");
		// Wait for Modbus reply
		if (k_sem_take(&rx_ready_sem_uart1, K_SECONDS(5)) == 0)
		{
			printk("Modbus response received and processed.\n");
		}
		else
		{
			printk("No Modbus response.\n");
		}

		gpio_pin_toggle_dt(&led);
		k_msleep(SLEEP_TIME_MS);
	}
}
