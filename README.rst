==============================
Modbus RTU over UART with nRF91
==============================

Overview
========
This project demonstrates a basic implementation of Modbus RTU communication over RS485 using the nRF9151 development kit and Zephyr RTOS. The application:

- Uses `UART1` to send Modbus commands and receive responses from a Modbus slave (e.g., soil moisture sensor).
- Controls RS485 direction via two GPIO pins: `DE` and `RE`.
- Uses `UART0` for debug messages over a USB or UART console.
- Implements UART transmission and reception using callbacks and semaphores.

Hardware Setup
==============
- **Board**: nRF9151 Development Kit
- **Modbus RTU Module (RS485)**
  - `DI` (Data In): Connected to `TX` of `UART1`
  - `RO` (Receiver Out): Connected to `RX` of `UART1` (via level shifter or voltage divider)
  - `DE`: Connected to `GPIO0.23` (Direction Enable)
  - `RE`: Connected to `GPIO0.22` (Receive Enable)

.. note::

    If your Modbus module uses 5V logic, ensure to use level shifters or voltage dividers to protect the nRF9151, which uses 1.8V logic.

Features
========
- Sends Modbus RTU request (example: Read Holding Registers)
- Waits for Modbus RTU response using UART receive callback
- Handles UART TX/RX using semaphores
- Controls transmission/reception direction using GPIOs
- Toggles an onboard LED as a heartbeat, only for testing purposes

How It Works
============
1. **GPIO Initialization**: Configures `RE` and `DE` pins as outputs to control Modbus direction.
2. **UART Initialization**: Sets up both UART0 and UART1 with callbacks.
3. **Transmission**: When sending a Modbus frame, the application sets `DE=1` and `RE=0`.
4. **Reception**: After TX, sets `DE=0` and `RE=1` to listen for a response.
5. **Semaphores**: Used to block the main thread until TX or RX completes.

Build and Flash
===============
1. Clone this repository into a Zephyr workspace.
2. Add the required configuration to `prj.conf`.
3. Build and flash with:

   .. code-block:: bash

      west build -b nrf9151dk_nrf9151
      west flash

Required Configuration
======================
`prj.conf` should include:

.. code-block:: none

   CONFIG_GPIO=y
   CONFIG_UART_ASYNC_API=y
   CONFIG_SERIAL=y
   CONFIG_UART_INTERRUPT_DRIVEN=y
   CONFIG_HEAP_MEM_POOL_SIZE=1024

Device Tree Overlay
===================
Your `boards/nrf9151dk_nrf9151.overlay` should include:

.. code-block:: dts

   / {
       aliases {
           led0 = &gpio0  // Or whichever LED is on your board
       };
   };

   &uart1 {
       status = "okay";
   };

   &gpio0 {
       de_pin: de-pin {
           gpios = <&gpio0 23 GPIO_ACTIVE_HIGH>;
       };

       re_pin: re-pin {
           gpios = <&gpio0 22 GPIO_ACTIVE_HIGH>;
       };
   };

License
=======
SPDX-License-Identifier: Apache-2.0

Author
======
JZ - 2025