/*
 * Copyright (c) 2014, Manuel Vetterli
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */ 

#include <avr/pgmspace.h>
#include "sys/process.h"
#include "usb_support.h"
#include "loconet.h"
#include "ln_interface.h"
#include "usb/usb_xmega.h"
#include "usb/cdc_standard.h"

PROCESS(usb_process, "USB Handler");

USB_ENDPOINTS(3);

__attribute__((__aligned__(4))) uint8_t usbserial_buf_in[64];
__attribute__((__aligned__(4))) uint8_t usbserial_buf_out[64];

LnBuf ln_buf;
uint8_t usbserial_sending_in = 0;

const USB_DeviceDescriptor PROGMEM device_descriptor = {
	.bLength = sizeof(USB_DeviceDescriptor),
	.bDescriptorType = USB_DTYPE_Device,

	.bcdUSB                 = 0x0200,
	.bDeviceClass           = USB_CSCP_CDCClass,
	.bDeviceSubClass        = USB_CSCP_NoDeviceSubclass,
	.bDeviceProtocol        = USB_CSCP_NoDeviceProtocol,

	.bMaxPacketSize0        = 64,
	.idVendor               = 0x03EB,
	.idProduct              = 0x2044,
	.bcdDevice              = 0x0001,

	.iManufacturer          = 0x01,
	.iProduct               = 0x02,
	.iSerialNumber          = 0xDC,

	.bNumConfigurations     = 1
};

typedef struct ConfigDesc {
	USB_ConfigurationDescriptor Config;
	USB_InterfaceDescriptor CDC_control_interface;

	CDC_FunctionalHeaderDescriptor CDC_functional_header;
	CDC_FunctionalACMDescriptor CDC_functional_ACM;
	CDC_FunctionalUnionDescriptor CDC_functional_union;
	USB_EndpointDescriptor CDC_notification_endpoint;

	USB_InterfaceDescriptor CDC_data_interface;
	USB_EndpointDescriptor CDC_out_endpoint;
	USB_EndpointDescriptor CDC_in_endpoint;
} __attribute__((packed)) ConfigDesc;

const ConfigDesc PROGMEM configuration_descriptor = {
	.Config = {
		.bLength = sizeof(USB_ConfigurationDescriptor),
		.bDescriptorType = USB_DTYPE_Configuration,
		.wTotalLength  = sizeof(ConfigDesc),
		.bNumInterfaces = 2,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = USB_CONFIG_ATTR_BUSPOWERED,
		.bMaxPower = USB_CONFIG_POWER_MA(50)
	},
	.CDC_control_interface = {
		.bLength = sizeof(USB_InterfaceDescriptor),
		.bDescriptorType = USB_DTYPE_Interface,
		.bInterfaceNumber = INTERFACE_CDC_CONTROL,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = CDC_INTERFACE_CLASS,
		.bInterfaceSubClass = CDC_INTERFACE_SUBCLASS_ACM,
		.bInterfaceProtocol = 0,
		.iInterface = 0,
	},
	.CDC_functional_header = {
		.bLength = sizeof(CDC_FunctionalHeaderDescriptor),
		.bDescriptorType = USB_DTYPE_CSInterface,
		.bDescriptorSubtype = CDC_SUBTYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.CDC_functional_ACM = {
		.bLength = sizeof(CDC_FunctionalACMDescriptor),
		.bDescriptorType = USB_DTYPE_CSInterface,
		.bDescriptorSubtype = CDC_SUBTYPE_ACM,
		.bmCapabilities = 0x00,
	},
	.CDC_functional_union = {
		.bLength = sizeof(CDC_FunctionalUnionDescriptor),
		.bDescriptorType = USB_DTYPE_CSInterface,
		.bDescriptorSubtype = CDC_SUBTYPE_UNION,
		.bMasterInterface = INTERFACE_CDC_CONTROL,
		.bSlaveInterface = INTERFACE_CDC_DATA,
	},
	.CDC_notification_endpoint = {
		.bLength = sizeof(USB_EndpointDescriptor),
		.bDescriptorType = USB_DTYPE_Endpoint,
		.bEndpointAddress = USB_EP_CDC_NOTIFICATION,
		.bmAttributes = (USB_EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
		.wMaxPacketSize = 8,
		.bInterval = 0xFF
	},
	.CDC_data_interface = {
		.bLength = sizeof(USB_InterfaceDescriptor),
		.bDescriptorType = USB_DTYPE_Interface,
		.bInterfaceNumber = INTERFACE_CDC_DATA,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = CDC_INTERFACE_CLASS_DATA,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,
	},
	.CDC_out_endpoint = {
		.bLength = sizeof(USB_EndpointDescriptor),
		.bDescriptorType = USB_DTYPE_Endpoint,
		.bEndpointAddress = USB_EP_CDC_OUT,
		.bmAttributes = (USB_EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
		.wMaxPacketSize = 64,
		.bInterval = 0x05
	},
	.CDC_in_endpoint = {
		.bLength = sizeof(USB_EndpointDescriptor),
		.bDescriptorType = USB_DTYPE_Endpoint,
		.bEndpointAddress = USB_EP_CDC_IN,
		.bmAttributes = (USB_EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
		.wMaxPacketSize = 64,
		.bInterval = 0x05
	},
};

const USB_StringDescriptor PROGMEM language_string = {
	.bLength = USB_STRING_LEN(1),
	.bDescriptorType = USB_DTYPE_String,
	.bString = {USB_LANGUAGE_EN_US},
};

const USB_StringDescriptor PROGMEM manufacturer_string = {
	.bLength = USB_STRING_LEN(11),
	.bDescriptorType = USB_DTYPE_String,
	.bString = u"Atmel Corp."
};

const USB_StringDescriptor PROGMEM product_string = {
	.bLength = USB_STRING_LEN(3),
	.bDescriptorType = USB_DTYPE_String,
	.bString = u"WA2"
};

void usb_process_init(void)
{
	//USB_ConfigureClock();
	
	USB.INTCTRLA = USB_BUSEVIE_bm | USB_INTLVL_MED_gc;
	USB.INTCTRLB = USB_TRNIE_bm | USB_SETUPIE_bm;
	
	usb_init();
	initLnBuf(&ln_buf);
	process_start(&usb_process, NULL);
	usb_attach();
}

void usb_serial_init(void)
{
	usb_ep_enable(USB_EP_CDC_NOTIFICATION, USB_EP_TYPE_BULK_gc, 8);
	usb_ep_enable(USB_EP_CDC_OUT, USB_EP_TYPE_BULK_gc, 64);
	usb_ep_enable(USB_EP_CDC_IN, USB_EP_TYPE_BULK_gc, 64);

	usb_ep_start_out(USB_EP_CDC_OUT, usbserial_buf_out, 64);
}

void usbserial_disable()
{
	usb_disable_ep(USB_EP_CDC_NOTIFICATION);
	usb_disable_ep(USB_EP_CDC_OUT);
	usb_disable_ep(USB_EP_CDC_IN);
}

void usb_cb_completion(void)
{
	if (usb_ep_pending(USB_EP_CDC_OUT))
	{
		usb_ep_handled(USB_EP_CDC_OUT);
		usb_size len = usb_ep_out_length(USB_EP_CDC_OUT);
		
		// Add received bytes to buffer
		for (usb_size index=0;index<len;index++)
		{
			addByteLnBuf(&ln_buf, usbserial_buf_out[index]);
		}
		
		// Ready to receive next packet
		usb_ep_start_out(USB_EP_CDC_OUT, usbserial_buf_out, 64);
	}

	if (usb_ep_pending(USB_EP_CDC_IN))
	{
		usb_ep_handled(USB_EP_CDC_IN);
		usbserial_sending_in = false;
	}
}

void sendLocoNetPacketUSB(lnMsg *LnPacket)
{
	uint8_t index, size;
	
	if (!usbserial_sending_in) {
		size = getLnMsgSize(LnPacket);
		
		// alternative: memcpy(usbserial_buf_in[usbserial_active_rx_buf], usbserial_buf_out, len);
		for (index=0;index<size;index++)
		{
			usbserial_buf_in[index] = LnPacket->data[index];
		}
		usb_ep_start_in(USB_EP_CDC_IN, usbserial_buf_in, size, false);
		usbserial_sending_in = true;
	}
}

PROCESS_THREAD(usb_process, ev, data)
{
	lnMsg *LnPacket;
	
	PROCESS_BEGIN();
	
	while (1)
	{
		LnPacket = recvLnMsg(&ln_buf);
		
		if (LnPacket)
		{
			sendLocoNetPacket(LnPacket);
		}
				
		PROCESS_PAUSE();
	}
	
	PROCESS_END();
}

uint16_t usb_cb_get_descriptor(uint8_t type, uint8_t index, const uint8_t** ptr)
{
	const void* address = NULL;
	uint16_t size    = 0;

	switch (type)
	{
		case USB_DTYPE_Device:
			address = &device_descriptor;
			size    = sizeof(USB_DeviceDescriptor);
			break;
		case USB_DTYPE_Configuration:
			address = &configuration_descriptor;
			size    = sizeof(ConfigDesc);
			break;
		case USB_DTYPE_String:
			switch (index)
			{
				case 0x00:
					address = &language_string;
					break;
				case 0x01:
					address = &manufacturer_string;
					break;
				case 0x02:
					address = &product_string;
					break;
				case 0xDC:
					*ptr = (uint8_t *) USB_Device_GetInternalSerialDescriptor();
					return ((USB_StringDescriptor*)address)->bLength;
					break;
			}
			size = pgm_read_byte(&((USB_StringDescriptor*)address)->bLength);
			break;
	}

	*ptr = usb_ep0_from_progmem(address, size);
	return size;
}

void usb_cb_reset(void)
{

}

bool usb_cb_set_configuration(uint8_t config)
{
	if (config <= 1)
	{
		usb_serial_init();
		enableLocoNetMaster(true);
		return true;
	}
	else
	{
		return false;
	}
}

CDC_LineEncoding line_encoding;

void usb_cb_control_setup(void)
{
	switch (usb_setup.bRequest)
	{
		case CDC_GET_LINE_ENCODING:
			if (usb_setup.bmRequestType==(USB_REQDIR_DEVICETOHOST|USB_REQTYPE_CLASS|USB_RECIPIENT_INTERFACE))
			{
				memcpy(ep0_buf_in,&line_encoding,sizeof(CDC_LineEncoding));
				usb_ep0_in(sizeof(CDC_LineEncoding));
				return usb_ep0_out();
			}
			break;
		case CDC_SET_LINE_ENCODING:
			if (usb_setup.bmRequestType==(USB_REQDIR_HOSTTODEVICE|USB_REQTYPE_CLASS|USB_RECIPIENT_INTERFACE))
			{
				memcpy(&line_encoding,ep0_buf_out,sizeof(CDC_LineEncoding));
				usb_ep0_in(0);
				return usb_ep0_out();
			}
			break;
		case CDC_SET_CONTROL_LINE_STATE:
			if (usb_setup.bmRequestType==(USB_REQDIR_HOSTTODEVICE|USB_REQTYPE_CLASS|USB_RECIPIENT_INTERFACE))
			{
				usb_ep0_in(0);
				return usb_ep0_out();
			}
			break;
	}
	return usb_ep0_stall();
}

void usb_cb_control_in_completion(void)
{

}

void usb_cb_control_out_completion(void)
{

}

bool usb_cb_set_interface(uint16_t interface, uint16_t altsetting)
{
	return false;
}
