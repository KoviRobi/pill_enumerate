// vim: tabstop=4 softtabstop=4 shiftwidth=4 noexpandtab
// let g:syntastic_c_compiler_options=" -I libopencm3/include -DSTM32F1 -Wall"
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include "pins.h"
#include "cdcacm.h"

// Set TRIANGLE if you want only to test if two pins are connected in one
// direction (0->1..*, 1->2..*, 2->3..* etc, hence triangle)
//#define TRIANGLE
// otherwise we will probe both ways, useful for diodes
//
// in/out scan pins
//   25   26   27   28   29   30   31   32   33   38   39   40   41
// PB10 PB13 PB14 PB15  PA8  PA9 PA10 PA11 PA12 PA15  PB3  PB4  PB5
#define gpio_scan_pins_num (sizeof(gpio_scan_pins)/sizeof(gpio_scan_pins[0]))
const int gpio_scan_pins[][2] = {
	PB12, PB13, PB14, PB15, PA8,  PA9,
	PA10, PB3,  PB5,  PB6,  PB7,  PB8,
	PB9
	/*PA11, PA12,*/
	/*PA15, PB4*/
};
int pin_matrix[gpio_scan_pins_num][gpio_scan_pins_num];
int pin_matrix_prev[gpio_scan_pins_num][gpio_scan_pins_num];

static usbd_device *usbd_dev;

const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x05ac,
	.idProduct = 0x2227,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};


const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
		.iface_assoc = &uart_assoc,
		.altsetting = uart_comm_iface,
}, {
	.num_altsetting = 1,
		.altsetting = uart_data_iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = sizeof(ifaces)/sizeof(ifaces[0]),
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"KoviRobi",
	"Pin Enumerator",
	"ABC",
	"Pin Enumerator UART Port",
};

void sys_tick_handler(void)
{
	int any_set = false;
	//if (gpio_get(GPIOB, GPIO12) & GPIO12) gpio_clear(GPIOC, GPIO13);
	//else gpio_set(GPIOC, GPIO13);
	//
	const int* prev_out = NULL;
	for (unsigned int out_id = 0;
			out_id < gpio_scan_pins_num;
			++out_id) {
		const int* out = gpio_scan_pins[out_id];

		// Set current pin to output (high), previous pin to input (pull down)
		if (prev_out != NULL) {
			gpio_set_mode(prev_out[0],
					GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
					prev_out[1]);
			gpio_clear(prev_out[0], prev_out[1]);
		}
		prev_out = out;

		gpio_set_mode(out[0],
				GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
				out[1]);
		gpio_set(out[0], out[1]);

		// Scan all the other pins
#ifdef TRIANGLE
		for (unsigned int in_id = out_id+1; // to avoid 0->1 and 1->0
				in_id < gpio_scan_pins_num;
				++in_id) {
#else
		for (unsigned int in_id = 0;
				in_id < gpio_scan_pins_num;
				++in_id) {
			if (in_id == out_id) continue;
#endif
			const int* in = gpio_scan_pins[in_id];
			if (gpio_get(in[0], in[1]) & in[1]) any_set = true;
			pin_matrix[out_id][in_id] = gpio_get(in[0], in[1]) & in[1];
		}
	}
	if (prev_out != NULL) {
		gpio_set_mode(prev_out[0],
				GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
				prev_out[1]);
		gpio_clear(prev_out[0], prev_out[1]);
	}

	if (any_set) gpio_clear(GPIOC, GPIO13);
	else gpio_set(GPIOC, GPIO13);
}

static int int_to_buf(int i, char* buf, int maxlen) {
     int written = 0;
     do {
             if (written > maxlen) break;
             buf[written++] = (i%10) + '0';
             i /= 10;

     } while (i > 0);
     // Reverse digits
     for (int j = 0; j < written; ++j) {
             buf[j] = buf[written-j-1];
     }
     return written;
}

sized_buf serial_write() {

	static char bufptr[gpio_scan_pins_num*gpio_scan_pins_num*12];
	sized_buf buf = { bufptr, 0 };

	for (unsigned int i = 0; i < gpio_scan_pins_num; ++i)
		for (unsigned int j = 0; j < gpio_scan_pins_num; ++j) {
			int cur = pin_matrix[i][j];
			int prev = pin_matrix_prev[i][j];
			if (cur != prev) {
				buf.len += int_to_buf(i, &buf.ptr[buf.len], 4);
				buf.ptr[buf.len++] = '-';
				buf.ptr[buf.len++] = cur?'>':'x';
				buf.len += int_to_buf(j, &buf.ptr[buf.len], 4);
				buf.ptr[buf.len++] = '\r';
				buf.ptr[buf.len++] = '\n';
				pin_matrix_prev[i][j] = pin_matrix[i][j];
			}
		}
	return buf;
}

static void usb_set_config(usbd_device *dev, uint16_t wValue)
{
	cdcacm_set_config(dev, wValue);
}


static void setup_clock(void) {
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* SysTick interrupt every N clock pulses: set reload to N-1
	 * Period: N / (72 MHz / 8 )
	 * */
	//systick_set_reload(899999); // 100 ms
	//systick_set_reload(89999); // 10 ms
	systick_set_reload(8999); // 1 ms
	systick_interrupt_enable();
	systick_counter_enable();
}

static void setup_gpio(void) {
	// Built-in LED on blue pill board, PC13
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	gpio_set(GPIOC, GPIO13);

	// Set all the pins to input
	for (unsigned int pin_id = 0;
			pin_id < gpio_scan_pins_num;
			++pin_id) {
		const int* pin = gpio_scan_pins[pin_id];
		gpio_set_mode(pin[0],
				GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
				pin[1]);
		gpio_clear(pin[0], pin[1]);
	}
}

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

int main(void)
{
	setup_clock();
	setup_gpio();

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings,
			sizeof(usb_strings)/sizeof(char *),
			usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usb_set_config);

	while (1)
		usbd_poll(usbd_dev);
}
