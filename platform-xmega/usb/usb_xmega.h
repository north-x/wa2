#pragma once

#include <avr/io.h>
#include <util/delay.h>
#include "usb.h"

#ifndef F_USB
#define F_USB 48000000
#endif

/// Configure the XMEGA's clock for use with USB.
void usb_configure_clock(void);
void usb_enable_SOF_DFLL(void);

/// Copy data from program memory to the ep0 IN buffer
const uint8_t* usb_ep0_from_progmem(const uint8_t* addr, uint16_t size);

void usb_ep_enable(uint8_t ep, uint8_t type, usb_size bufsize);

typedef union USB_EP_pair{
	union{
		struct{
			USB_EP_t out;
			USB_EP_t in;
		};
		USB_EP_t ep[2];
	};
} __attribute__((packed)) USB_EP_pair_t;

extern USB_EP_pair_t usb_xmega_endpoints[];
extern const uint8_t usb_num_endpoints;

/** Like __attribute__(align(2)), but actually works. 
    From http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=121033
 */
#define GCC_FORCE_ALIGN_2  __attribute__((section (".data,\"aw\",@progbits\n.p2align 1;")))

#define USB_ENDPOINTS(NUM_EP) \
	const uint8_t usb_num_endpoints = (NUM_EP); \
	USB_EP_pair_t usb_xmega_endpoints[(NUM_EP)+1] GCC_FORCE_ALIGN_2;


/// Length of the device's unique internal serial number, in bits, if present on the selected microcontroller model.
#define INTERNAL_SERIAL_LENGTH_BITS    (8 * (1 + (offsetof(NVM_PROD_SIGNATURES_t, COORDY1) - offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0))))

/// Start address of the internal serial number, in the appropriate address space, if present on the selected microcontroller model.
#define INTERNAL_SERIAL_START_ADDRESS  offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0)

USB_StringDescriptor* USB_Device_GetInternalSerialDescriptor(void);
void USB_enter_bootloader(void);