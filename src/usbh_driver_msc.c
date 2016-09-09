/*
 * This file is part of the libusbhost library
 * hosted at http://github.com/libusbhost/libusbhost
 *
 * Copyright (C) 2015 Amir Hammad <amir.hammad@hotmail.com>
 *
 *
 * libusbhost is free software: you can redistribute it and/or modify
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
 *
 * MSC and SCSI definitions are partly adopted of libopencm3 library, file: usb_msc.c
 *
 */

#include "usbh_driver_msc.h"
#include "driver/usbh_device_driver.h"
#include "usart_helpers.h"

#include <string.h>
#include <stdint.h>

#include "mini_endian.h"

#include <libopencm3/usb/usbstd.h>

enum STATES {
	STATE_INACTIVE,
	STATE_IDLE,
	STATE_GET_MAX_LUN_REQUEST,
	STATE_ENUM_START = STATE_GET_MAX_LUN_REQUEST,
	STATE_GET_MAX_LUN_COMPLETE,

	STATE_READ_CAPACITY_REQUEST,
	STATE_READ_CAPACITY_COMPLETE,

	STATE_MODE_SENSE_6_REQUEST,
	STATE_MODE_SENSE_6_COMPLETE,

	STATE_REQUEST_SENSE_REQUEST,
	STATE_REQUEST_SENSE_COMPLETE,

	STATE_INQUIRY_REQUEST,
	STATE_INQUIRY_COMPLETE,

	STATE_TEST_UNIT_READY_REQUEST,
	STATE_TEST_UNIT_READY_COMPLETE,

	STATE_READ10_COMPLETE,
	STATE_WRITE10_COMPLETE,
};

static const enum STATES enum_states[] = {
	// IMPLICIT GET_MAX_LUN
	STATE_READ_CAPACITY_REQUEST,
	STATE_TEST_UNIT_READY_REQUEST,
	STATE_INQUIRY_REQUEST,
	STATE_MODE_SENSE_6_REQUEST,
	STATE_REQUEST_SENSE_REQUEST,
	STATE_IDLE
};

enum STATES_TRANSACTION {
	STATE_TRANSACTION_IDLE,

	STATE_TRANSACTION_CBW_COMPLETE,
	STATE_TRANSACTION_CSW_COMPLETE,

	STATE_TRANSACTION_READ_CAPACITY_COMPLETE,
	STATE_TRANSACTION_READ_10_COMPLETE,
	STATE_TRANSACTION_WRITE_10_COMPLETE,
	STATE_TRANSACTION_MODE_SENSE_6_COMPLETE,
	STATE_TRANSACTION_REQUEST_SENSE_COMPLETE,
	STATE_TRANSACTION_INQUIRY_COMPLETE
};

/* Command Block Wrapper */
#define CBW_SIGNATURE				0x43425355
#define CBW_STATUS_SUCCESS			0
#define CBW_STATUS_FAILED			1
#define CBW_STATUS_PHASE_ERROR			2

/* Command Status Wrapper */
#define CSW_SIGNATURE				0x53425355
#define CSW_STATUS_SUCCESS			0
#define CSW_STATUS_FAILED			1
#define CSW_STATUS_PHASE_ERROR			2

/* Implemented SCSI Commands */
#define SCSI_TEST_UNIT_READY			0x00
#define SCSI_REQUEST_SENSE			0x03
#define SCSI_FORMAT_UNIT			0x04
#define SCSI_READ_6				0x08
#define SCSI_WRITE_6				0x0A
#define SCSI_INQUIRY				0x12
#define SCSI_MODE_SENSE_6			0x1A
#define SCSI_SEND_DIAGNOSTIC			0x1D
#define SCSI_READ_CAPACITY			0x25
#define SCSI_READ_10				0x28


/* Required SCSI Commands */

/* Optional SCSI Commands */
#define SCSI_REPORT_LUNS			0xA0
#define SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL	0x1E
#define SCSI_MODE_SELECT_6			0x15
#define SCSI_MODE_SELECT_10			0x55
#define SCSI_MODE_SENSE_10			0x5A
#define SCSI_READ_12				0xA8
#define SCSI_READ_FORMAT_CAPACITIES		0x23
#define SCSI_READ_TOC_PMA_ATIP			0x43
#define SCSI_START_STOP_UNIT			0x1B
#define SCSI_SYNCHRONIZE_CACHE			0x35
#define SCSI_VERIFY				0x2F
#define SCSI_WRITE_10				0x2A
#define SCSI_WRITE_12				0xAA

/* The sense codes */
enum sbc_sense_key {
	SBC_SENSE_KEY_NO_SENSE			= 0x00,
	SBC_SENSE_KEY_RECOVERED_ERROR		= 0x01,
	SBC_SENSE_KEY_NOT_READY			= 0x02,
	SBC_SENSE_KEY_MEDIUM_ERROR		= 0x03,
	SBC_SENSE_KEY_HARDWARE_ERROR		= 0x04,
	SBC_SENSE_KEY_ILLEGAL_REQUEST		= 0x05,
	SBC_SENSE_KEY_UNIT_ATTENTION		= 0x06,
	SBC_SENSE_KEY_DATA_PROTECT		= 0x07,
	SBC_SENSE_KEY_BLANK_CHECK		= 0x08,
	SBC_SENSE_KEY_VENDOR_SPECIFIC		= 0x09,
	SBC_SENSE_KEY_COPY_ABORTED		= 0x0A,
	SBC_SENSE_KEY_ABORTED_COMMAND		= 0x0B,
	SBC_SENSE_KEY_VOLUME_OVERFLOW		= 0x0D,
	SBC_SENSE_KEY_MISCOMPARE		= 0x0E
};

enum sbc_asc {
	SBC_ASC_NO_ADDITIONAL_SENSE_INFORMATION	= 0x00,
	SBC_ASC_PERIPHERAL_DEVICE_WRITE_FAULT	= 0x03,
	SBC_ASC_LOGICAL_UNIT_NOT_READY		= 0x04,
	SBC_ASC_UNRECOVERED_READ_ERROR		= 0x11,
	SBC_ASC_INVALID_COMMAND_OPERATION_CODE	= 0x20,
	SBC_ASC_LBA_OUT_OF_RANGE		= 0x21,
	SBC_ASC_INVALID_FIELD_IN_CDB		= 0x24,
	SBC_ASC_WRITE_PROTECTED			= 0x27,
	SBC_ASC_NOT_READY_TO_READY_CHANGE	= 0x28,
	SBC_ASC_FORMAT_ERROR			= 0x31,
	SBC_ASC_MEDIUM_NOT_PRESENT		= 0x3A
};

enum sbc_ascq {
	SBC_ASCQ_NA				= 0x00,
	SBC_ASCQ_FORMAT_COMMAND_FAILED		= 0x01,
	SBC_ASCQ_INITIALIZING_COMMAND_REQUIRED	= 0x02,
	SBC_ASCQ_OPERATION_IN_PROGRESS		= 0x07
};

// CBW
struct _usb_msc_cbw {
	uint32_t dCBWSignature;
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t  bmCBWFlags;
	uint8_t  bCBWLUN;
	uint8_t  bCBWCBLength;
	uint8_t  CBWCB[16];
} __attribute__((packed));


// CSW
struct _usb_msc_csw {
	uint32_t dCSWSignature;
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t  bCSWStatus;
} __attribute__((packed));


//
struct _msc_device {
	usbh_device_t *usbh_device;
	enum STATES state_next;
	uint8_t device_id;

	uint8_t buffer[USBH_MSC_BUFFER];

	uint16_t endpoint_in_maxpacketsize;
	uint8_t endpoint_in_address;
	uint8_t endpoint_in_toggle;

	uint16_t endpoint_out_maxpacketsize;
	uint8_t endpoint_out_address;
	uint8_t endpoint_out_toggle;

	struct {
		uint32_t block_count; // lba
		uint32_t block_size;
	} properties;

	// only one transaction can be in progress, because the following
	// field is used to hold data (also for retry mechanism: e.g. cbw)
	struct {
		uint32_t opcode;
		uint32_t data_length;
		void *data_ptr;
		enum STATES_TRANSACTION state_next;
		struct _usb_msc_cbw cbw;
		msc_callback_t callback;
		void *callback_arg;
	} transaction;

	int enumSeq;
};
typedef struct _msc_device msc_device_t;

static msc_device_t msc_device[USBH_MSC_MAX_DEVICES];
static const msc_config_t *msc_config;

static bool initialized = false;

static void transaction_event(usbh_device_t *dev, usbh_packet_callback_data_t cb_data);
static void event(usbh_device_t *dev, usbh_packet_callback_data_t cb_data);
static void cbw_send(msc_device_t *msc);

void msc_driver_init(const msc_config_t *config)
{
	if (!config) {
		return;
	}
	initialized = true;
	uint32_t i;
	msc_config = config;
	for (i = 0; i < USBH_MSC_MAX_DEVICES; i++) {
		msc_device[i].state_next = STATE_INACTIVE;
	}
}

bool msc_idle(uint8_t device_id)
{
	return msc_device[device_id].state_next == STATE_IDLE;
}

static void update_state(msc_device_t *msc)
{
	msc->state_next = enum_states[msc->enumSeq++];
}

/**
 *
 *
 */
static void *init(void *usbh_dev)
{
	if (!initialized) {
		LOG_PRINTF("\r\n%s/%d : driver not initialized\r\n", __FILE__, __LINE__);
		return 0;
	}

	uint32_t i;
	msc_device_t *drvdata = 0;

	// find free data space for msc device
	for (i = 0; i < USBH_MSC_MAX_DEVICES; i++) {
		if (msc_device[i].state_next == STATE_INACTIVE) {
			drvdata = &msc_device[i];
			drvdata->device_id = i;
			drvdata->endpoint_in_address = 0;
			drvdata->endpoint_in_toggle = 0;
			drvdata->endpoint_out_address = 0;
			drvdata->endpoint_out_toggle = 0;
			drvdata->transaction.state_next = STATE_TRANSACTION_IDLE;
			drvdata->usbh_device = (usbh_device_t *)usbh_dev;
			drvdata->enumSeq = 0;
			break;
		}
	}

	return drvdata;
}

/**
 * Returns true if all needed data are parsed
 */
static bool analyze_descriptor(void *drvdata, void *descriptor)
{
	msc_device_t *msc = (msc_device_t *)drvdata;
	uint8_t desc_type = ((uint8_t *)descriptor)[1];
	switch (desc_type) {
	case USB_DT_ENDPOINT:
		{
			struct usb_endpoint_descriptor *ep = (struct usb_endpoint_descriptor*)descriptor;
			if ((ep->bmAttributes&0x03) == USB_ENDPOINT_ATTR_BULK) {

				// EP IN
				if (ep->bEndpointAddress & (1 << 7)) {
					if (msc->endpoint_in_address == 0) {
						// update address and max packet size for endpoint
						msc->endpoint_in_address = ep->bEndpointAddress&0x7f;
						if (ep->wMaxPacketSize < USBH_MSC_BUFFER) {
							msc->endpoint_in_maxpacketsize = ep->wMaxPacketSize;
						} else {
							msc->endpoint_in_maxpacketsize = USBH_MSC_BUFFER;
						}
					}
				} else {
					if (msc->endpoint_out_address == 0) {
						// update address and max packet size for endpoint
						msc->endpoint_out_address = ep->bEndpointAddress&0x7f;
						if (ep->wMaxPacketSize < USBH_MSC_BUFFER) {
							msc->endpoint_out_maxpacketsize = ep->wMaxPacketSize;
						} else {
							msc->endpoint_out_maxpacketsize = USBH_MSC_BUFFER;
						}
					}
				}

			}
		}
		break;

	default:
		break;
	}

	if (msc->endpoint_in_address
	&&	msc->endpoint_out_address) {
		msc->state_next = STATE_ENUM_START;
		return true;
	}
	return false;
}

static void finish_transaction(msc_device_t *msc)
{
	usbh_packet_t packet;
	packet.toggle = &msc->endpoint_in_toggle;
	packet.address = msc->usbh_device->address;
	packet.speed = msc->usbh_device->speed;
	packet.callback = transaction_event;
	packet.callback_arg = msc->usbh_device;
	packet.endpoint_size_max = msc->endpoint_in_maxpacketsize;
	packet.endpoint_address = msc->endpoint_in_address;
	packet.endpoint_type = USBH_ENDPOINT_TYPE_BULK;
	packet.data.in = msc->buffer;
	packet.datalen = sizeof(struct _usb_msc_csw);

	msc->transaction.state_next = STATE_TRANSACTION_CSW_COMPLETE;
	usbh_read(msc->usbh_device, &packet);
}

static void transaction_event(usbh_device_t *dev, usbh_packet_callback_data_t cb_data)
{
	msc_device_t *msc = (msc_device_t *)dev->drvdata;
	switch (msc->transaction.state_next) {
	case STATE_TRANSACTION_CBW_COMPLETE:
		{
			switch (cb_data.status) {
			case USBH_PACKET_CALLBACK_STATUS_OK:
				{
					usbh_packet_t packet;

					// fill common entries
					packet.address = msc->usbh_device->address;
					packet.speed = msc->usbh_device->speed;
					packet.callback = transaction_event;
					packet.callback_arg = msc->usbh_device;
					packet.endpoint_type = USBH_ENDPOINT_TYPE_BULK;

					packet.datalen = msc->transaction.data_length;

					// 8th bit represents direction of the transfer
					if (msc->transaction.cbw.bmCBWFlags & 0x80) {
						packet.toggle = &msc->endpoint_in_toggle;
						packet.endpoint_size_max = msc->endpoint_in_maxpacketsize;
						packet.endpoint_address = msc->endpoint_in_address;
						packet.data.in = msc->transaction.data_ptr;
					} else {
						packet.toggle = &msc->endpoint_out_toggle;
						packet.endpoint_size_max = msc->endpoint_out_maxpacketsize;
						packet.endpoint_address = msc->endpoint_out_address;
						packet.data.out = msc->transaction.data_ptr;
					}

					LOG_PRINTF("CBW succeed\r\n");
					switch (msc->transaction.opcode) {
					case SCSI_READ_10:
						LOG_PRINTF("READ10 %d\r\n", msc->transaction.data_length);

						msc->transaction.state_next = STATE_TRANSACTION_READ_10_COMPLETE;
						usbh_read(msc->usbh_device, &packet);
						break;

					case SCSI_WRITE_10:
						LOG_PRINTF("WRITE10 %d\r\n", msc->transaction.data_length);

						msc->transaction.state_next = STATE_TRANSACTION_WRITE_10_COMPLETE;
						usbh_write(msc->usbh_device, &packet);
						break;

					case SCSI_READ_CAPACITY:
						msc->transaction.state_next = STATE_TRANSACTION_READ_CAPACITY_COMPLETE;
						usbh_read(msc->usbh_device, &packet);
						break;

					case SCSI_MODE_SENSE_6:
						msc->transaction.state_next = STATE_TRANSACTION_MODE_SENSE_6_COMPLETE;
						usbh_read(msc->usbh_device, &packet);
						break;

					case SCSI_REQUEST_SENSE:
						msc->transaction.state_next = STATE_TRANSACTION_REQUEST_SENSE_COMPLETE;
						usbh_read(msc->usbh_device, &packet);
						break;

					case SCSI_INQUIRY:
						msc->transaction.state_next = STATE_TRANSACTION_INQUIRY_COMPLETE;
						usbh_read(msc->usbh_device, &packet);
						break;

					case SCSI_TEST_UNIT_READY:
						finish_transaction(msc);
						break;

					default:
						ERROR_S("FAIL");
						break;
					}
				}
				break;

				case USBH_PACKET_CALLBACK_STATUS_EAGAIN:
					cbw_send(msc);
					break;

			default:
				ERROR_S("FAIL");
				break;
			}
		}
		break;

	case STATE_TRANSACTION_INQUIRY_COMPLETE:
	case STATE_TRANSACTION_REQUEST_SENSE_COMPLETE:
	case STATE_TRANSACTION_MODE_SENSE_6_COMPLETE:
		switch (cb_data.status) {
		case USBH_PACKET_CALLBACK_STATUS_OK:
			finish_transaction(msc);
			break;

		default:
			LOG_PRINTF("Trans length: %d\r\n", cb_data.transferred_length);
			ERROR(cb_data.status);
		}
		break;

	case STATE_TRANSACTION_READ_10_COMPLETE:
		switch (cb_data.status) {
		case USBH_PACKET_CALLBACK_STATUS_OK:
			{
				finish_transaction(msc);
			}
			break;

		default:
			LOG_PRINTF("Trans length: %d\r\n", cb_data.transferred_length);
			ERROR(cb_data.status);
		}
		break;

	case STATE_TRANSACTION_WRITE_10_COMPLETE:
		switch (cb_data.status) {
		case USBH_PACKET_CALLBACK_STATUS_OK:
			{
				LOG_PRINTF("%d\n", cb_data.status);
				finish_transaction(msc);
			}
			break;

		case USBH_PACKET_CALLBACK_STATUS_EAGAIN:
			{
				usbh_packet_t packet;

				// fill common entries
				packet.address = msc->usbh_device->address;
				packet.speed = msc->usbh_device->speed;
				packet.callback = transaction_event;
				packet.callback_arg = msc->usbh_device;
				packet.endpoint_type = USBH_ENDPOINT_TYPE_BULK;
				packet.toggle = &msc->endpoint_out_toggle;
				packet.endpoint_size_max = msc->endpoint_out_maxpacketsize;
				packet.endpoint_address = msc->endpoint_out_address;

				packet.data.out = msc->transaction.data_ptr;
				packet.datalen = msc->transaction.data_length;

				msc->transaction.state_next = STATE_TRANSACTION_WRITE_10_COMPLETE;

				usbh_write(msc->usbh_device, &packet);
			}
			break;

		default:
			LOG_PRINTF("Trans length: %d\r\n", cb_data.transferred_length);
			ERROR(cb_data.status);
		}
		break;


	case STATE_TRANSACTION_READ_CAPACITY_COMPLETE:
		switch (cb_data.status) {
		case USBH_PACKET_CALLBACK_STATUS_OK:
			{
				uint32_t *buf = (uint32_t *)msc->buffer;
				msc->properties.block_count = be32toh(*buf);
				buf++;
				msc->properties.block_size = be32toh(*buf);
				LOG_PRINTF("\r\n");
				LOG_PRINTF("BLOCK COUNT = %5u\r\n", msc->properties.block_count);
				LOG_PRINTF("BLOCK SIZE =  %5u\r\n", msc->properties.block_size);
				LOG_PRINTF("CAPACITY = %llu bytes\r\n", msc->properties.block_count*(uint64_t)msc->properties.block_size);

				finish_transaction(msc);
			}
			break;

		default:
			ERROR_S("FAIL");
		}
		break;

	case STATE_TRANSACTION_CSW_COMPLETE:
		switch (cb_data.status) {
		case USBH_PACKET_CALLBACK_STATUS_OK:
			{
				struct _usb_msc_csw *csw = (struct _usb_msc_csw *)msc->buffer;
				LOG_PRINTF("\r\n CSW: ");
				LOG_PRINTF("CSW status=%d\r\n", csw->bCSWStatus);
				msc->transaction.state_next = STATE_TRANSACTION_IDLE;
				event(dev, cb_data);
			}
			break;
		default:
			ERROR_S("FAIL");
			break;
		}
		break;

	case STATE_TRANSACTION_IDLE:
		ERROR_S("Epic fail, callback called in idle state\r\n");
		break;
	}

}

static void event(usbh_device_t *dev, usbh_packet_callback_data_t cb_data)
{
	msc_device_t *msc = (msc_device_t *)dev->drvdata;
	switch (msc->state_next) {
	case STATE_GET_MAX_LUN_COMPLETE:
		{
			switch (cb_data.status) {
			case USBH_PACKET_CALLBACK_STATUS_OK:
				LOG_PRINTF("\r\nMAXLUN= %d\r\n", msc->buffer[0]);
				update_state(msc);

				if (msc_config->notify_connected) {
					msc_config->notify_connected(msc->device_id);
				}
				break;

			case USBH_PACKET_CALLBACK_STATUS_EFATAL:
			case USBH_PACKET_CALLBACK_STATUS_EAGAIN:
			case USBH_PACKET_CALLBACK_STATUS_ERRSIZ:
				ERROR(cb_data.status);
				msc->state_next = STATE_INACTIVE;
				break;
			}
		}
		break;

	case STATE_TEST_UNIT_READY_COMPLETE:
		update_state(msc);
		break;

	case STATE_READ_CAPACITY_COMPLETE:
		update_state(msc);
		break;

	case STATE_MODE_SENSE_6_COMPLETE:
		update_state(msc);
		break;

	case STATE_REQUEST_SENSE_COMPLETE:
		update_state(msc);
		break;

	case STATE_INQUIRY_COMPLETE:
		update_state(msc);
		break;

	case STATE_READ10_COMPLETE:
		{
			msc->state_next = STATE_IDLE;
			if (msc->transaction.callback) {
				msc->transaction.callback(msc->transaction.callback_arg);
			}
		}
		break;

	case STATE_WRITE10_COMPLETE:
		{
			msc->state_next = STATE_IDLE;
			if (msc->transaction.callback) {
				msc->transaction.callback(msc->transaction.callback_arg);
			}
		}
		break;

	case STATE_INACTIVE:
		{
			LOG_PRINTF("MSC inactive");
		}
		break;

	default:
		{
			LOG_PRINTF("Unknown state\r\n");
		}
		break;
	}
}

static void cbw_send(msc_device_t *msc)
{
	msc->transaction.cbw.dCBWSignature = CBW_SIGNATURE; // CONSTANT
	msc->transaction.cbw.dCBWTag = 0; // We do not use Tag
	msc->transaction.cbw.bCBWLUN = 0; // CONSTANT We do not use other LUN than 0
	msc->transaction.cbw.dCBWDataTransferLength = msc->transaction.data_length;// Data length

	usbh_packet_t packet;
	packet.toggle = &msc->endpoint_out_toggle;
	packet.address = msc->usbh_device->address;
	packet.speed = msc->usbh_device->speed;
	packet.callback = transaction_event;
	packet.callback_arg = msc->usbh_device;
	packet.endpoint_size_max = msc->endpoint_out_maxpacketsize;
	packet.endpoint_address = msc->endpoint_out_address;
	packet.endpoint_type = USBH_ENDPOINT_TYPE_BULK;
	packet.data.out = &msc->transaction.cbw;
	packet.datalen = sizeof(msc->transaction.cbw);

	msc->transaction.opcode = msc->transaction.cbw.CBWCB[0];
	msc->transaction.state_next = STATE_TRANSACTION_CBW_COMPLETE;
	usbh_write(msc->usbh_device, &packet);
}

static void read_capacity(msc_device_t *msc)
{
	msc->transaction.cbw.bmCBWFlags = 0x80; // Direction read
	msc->transaction.cbw.bCBWCBLength = 10;
	memset(&msc->transaction.cbw.CBWCB[0], 0, msc->transaction.cbw.bCBWCBLength);
	memset(&msc->transaction.cbw.CBWCB[msc->transaction.cbw.bCBWCBLength], 0, sizeof(msc->transaction.cbw.CBWCB) - msc->transaction.cbw.bCBWCBLength);
	msc->transaction.cbw.CBWCB[0] = SCSI_READ_CAPACITY;

	msc->transaction.data_ptr = msc->buffer;
	msc->transaction.data_length = 8;
	cbw_send(msc);
}

static void mode_sense_6(msc_device_t *msc)
{
	msc->transaction.cbw.bmCBWFlags = 0x80; // Direction read
	msc->transaction.cbw.bCBWCBLength = 6;
	memset(&msc->transaction.cbw.CBWCB[0], 0, msc->transaction.cbw.bCBWCBLength);
	memset(&msc->transaction.cbw.CBWCB[msc->transaction.cbw.bCBWCBLength], 0, sizeof(msc->transaction.cbw.CBWCB) - msc->transaction.cbw.bCBWCBLength);
	msc->transaction.cbw.CBWCB[0] = SCSI_MODE_SENSE_6;

	msc->transaction.data_ptr = msc->buffer;
	msc->transaction.data_length = 10;
	cbw_send(msc);
}

static void test_unit_ready(msc_device_t *msc)
{
	msc->transaction.cbw.bmCBWFlags = 0x80; // Direction read
	msc->transaction.cbw.bCBWCBLength = 6;
	memset(&msc->transaction.cbw.CBWCB[0], 0, msc->transaction.cbw.bCBWCBLength);
	memset(&msc->transaction.cbw.CBWCB[msc->transaction.cbw.bCBWCBLength], 0, sizeof(msc->transaction.cbw.CBWCB) - msc->transaction.cbw.bCBWCBLength);
	msc->transaction.cbw.CBWCB[0] = SCSI_TEST_UNIT_READY;

	msc->transaction.data_ptr = NULL;
	msc->transaction.data_length = 0;
	cbw_send(msc);
}

static void request_sense(msc_device_t *msc)
{
	msc->transaction.cbw.bmCBWFlags = 0x80; // Direction read
	msc->transaction.cbw.bCBWCBLength = 6;
	memset(&msc->transaction.cbw.CBWCB[0], 0, msc->transaction.cbw.bCBWCBLength);
	memset(&msc->transaction.cbw.CBWCB[msc->transaction.cbw.bCBWCBLength], 0, sizeof(msc->transaction.cbw.CBWCB) - msc->transaction.cbw.bCBWCBLength);
	msc->transaction.cbw.CBWCB[0] = SCSI_REQUEST_SENSE;

	msc->transaction.data_ptr = msc->buffer;
	msc->transaction.data_length = 8;
	cbw_send(msc);
}

static void inquiry(msc_device_t *msc)
{
	struct _inquiry_cb {
		uint8_t opcode;
		uint8_t cmdt_evpd;
		uint8_t reserved;
		uint16_t alloc_length;
		uint8_t control;
	} __attribute__((packed));

	msc->transaction.cbw.bmCBWFlags = 0x80; // Direction read
	msc->transaction.cbw.bCBWCBLength = 6;

	struct _inquiry_cb *inquiry_cb = (struct _inquiry_cb *)&msc->transaction.cbw.CBWCB[0];
	memset(&msc->transaction.cbw.CBWCB[0], 0, msc->transaction.cbw.bCBWCBLength);
	memset(&msc->transaction.cbw.CBWCB[msc->transaction.cbw.bCBWCBLength], 0, sizeof(msc->transaction.cbw.CBWCB) - msc->transaction.cbw.bCBWCBLength);
	inquiry_cb->opcode = SCSI_INQUIRY;
	inquiry_cb->alloc_length = 96;

	msc->transaction.data_ptr = msc->buffer;
	msc->transaction.data_length = inquiry_cb->alloc_length;
	cbw_send(msc);
}

/**
 * \param time_curr_us - monotically rising time (see usbh_hubbed.h)
 *		unit is microseconds
 */
static void poll(void *drvdata, uint32_t time_curr_us)
{
	(void)time_curr_us;

	msc_device_t *msc = (msc_device_t *)drvdata;
	usbh_device_t *dev = msc->usbh_device;

	if (msc->transaction.state_next != STATE_TRANSACTION_IDLE) {
		return;
	}

	switch (msc->state_next) {
	case STATE_ENUM_START:
		{
			LOG_PRINTF("\r\nmsc CONFIGURED\r\n");
			struct usb_setup_data setup_data;
			setup_data.bmRequestType = USB_REQ_TYPE_IN | USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE;
			setup_data.bRequest = 254; //0xfe=254=GET_MAX_LUN request
			setup_data.wValue = 0;
			setup_data.wIndex = 0;
			setup_data.wLength = 1;

			msc->state_next = STATE_GET_MAX_LUN_COMPLETE;
			device_control(dev, event, &setup_data, msc->buffer);

			if (msc_config->notify_connected) {
				msc_config->notify_connected(msc->device_id);
			}
		}
		break;

	case STATE_TEST_UNIT_READY_REQUEST:
		LOG_PRINTF("~~~ TEST_UNIT_READY\n");
		msc->state_next = STATE_TEST_UNIT_READY_COMPLETE;
		test_unit_ready(msc);
		break;

	case STATE_READ_CAPACITY_REQUEST:
		LOG_PRINTF("~~~ READ_CAPACITY\n");
		msc->state_next = STATE_READ_CAPACITY_COMPLETE;
		read_capacity(msc);
		break;

	case STATE_MODE_SENSE_6_REQUEST:
		LOG_PRINTF("~~~ MODE_SENSE_6\n");
		msc->state_next = STATE_MODE_SENSE_6_COMPLETE;
		mode_sense_6(msc);
		break;

	case STATE_REQUEST_SENSE_REQUEST:
		LOG_PRINTF("~~~ REQUEST_SENSE\n");
		msc->state_next = STATE_REQUEST_SENSE_COMPLETE;
		request_sense(msc);
		break;

	case STATE_INQUIRY_REQUEST:
		LOG_PRINTF("~~~ INQUIRY\n");
		msc->state_next = STATE_INQUIRY_COMPLETE;
		inquiry(msc);
		break;

	default:
		{
			// do nothing - probably transfer is in progress
		}
		break;
	}
}

static void remove(void *drvdata)
{
	LOG_PRINTF("Removing msc\r\n");

	msc_device_t *msc = (msc_device_t *)drvdata;
	if (msc_config->notify_disconnected) {
		msc_config->notify_disconnected(msc->device_id);
	}
	msc->state_next = STATE_INACTIVE;
	msc->transaction.state_next= STATE_TRANSACTION_IDLE;
}

void msc_read10(uint8_t device_id, void *data, uint32_t block_count, uint32_t lba, msc_callback_t callback, void *arg)
{
	msc_device_t *msc = &msc_device[device_id];
	struct _read10 {
		uint8_t opcode;
		uint8_t flags;
		uint32_t lba_address;
		uint8_t group;
		uint16_t length; // in blocks
		uint8_t control;
	} __attribute__((packed));
	struct _read10 *read10 = (struct _read10 *)&msc->transaction.cbw.CBWCB[0];
	memset(read10, 0, sizeof(*read10));

	read10->opcode = SCSI_READ_10;
	read10->flags = 0;
	read10->lba_address = htobe32(lba);
	read10->group = 0;
	read10->length = htobe16(block_count);
	read10->control = 0;

	msc->transaction.cbw.bCBWCBLength = 10;
	msc->transaction.cbw.bmCBWFlags = 0x80; // Direction read

	memset(&msc->transaction.cbw.CBWCB[msc->transaction.cbw.bCBWCBLength], 0, sizeof(msc->transaction.cbw.CBWCB) - msc->transaction.cbw.bCBWCBLength);

	msc->transaction.data_length = block_count * msc->properties.block_size;
	msc->transaction.data_ptr = data;
	msc->transaction.callback = callback;
	msc->transaction.callback_arg = arg;

	msc->state_next = STATE_READ10_COMPLETE;

	cbw_send(msc);
}

void msc_write10(uint8_t device_id, void *data, uint32_t block_count, uint32_t lba, msc_callback_t callback, void *arg)
{
	msc_device_t *msc = &msc_device[device_id];

	struct _write10 {
		uint8_t opcode;
		uint8_t flags;
		uint32_t lba_address;
		uint8_t group;
		uint16_t length; // in blocks
		uint8_t control;
	} __attribute__((packed)) ;

	struct _write10 *write10 = (struct _write10 *)&msc->transaction.cbw.CBWCB[0];
	memset(write10, 0, sizeof(*write10));
	write10->opcode = SCSI_WRITE_10;
	write10->flags = 0;
	write10->lba_address = htobe32(lba);
	write10->group = 0;
	write10->length = htobe16(block_count);
	write10->control = 0;

	msc->transaction.cbw.bCBWCBLength = 10;
	msc->transaction.cbw.bmCBWFlags = 0x00; // Direction write

	memset(&msc->transaction.cbw.CBWCB[msc->transaction.cbw.bCBWCBLength], 0, sizeof(msc->transaction.cbw.CBWCB) - msc->transaction.cbw.bCBWCBLength);

	msc->transaction.data_length = block_count * msc->properties.block_size;
	msc->transaction.data_ptr = data;
	msc->transaction.callback = callback;
	msc->transaction.callback_arg = arg;

	msc->state_next = STATE_WRITE10_COMPLETE;

	cbw_send(msc);
}

bool msc_present(uint8_t device_id)
{
	if (device_id >= sizeof(msc_device)/sizeof(msc_device[0])) {
		return false;
	}
	return msc_device[device_id].state_next != STATE_INACTIVE;
}

bool msc_initialized(void)
{
	return initialized;
}

static const usbh_dev_driver_info_t driver_info = {
	.deviceClass = 0x00,
	.deviceSubClass = 0x00,
	.deviceProtocol = 0x00,
	.idVendor = -1,
	.idProduct = -1,
	.ifaceClass = 8, // mass storage
	.ifaceSubClass = 6, // SCSI
	.ifaceProtocol = 80 // Bulk only (BBB) transport
};

const usbh_dev_driver_t usbh_msc_driver = {
	.init = init,
	.analyze_descriptor = analyze_descriptor,
	.poll = poll,
	.remove = remove,
	.info = &driver_info
};
