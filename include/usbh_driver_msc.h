#ifndef USBH_DRIVER_MSC_H_
#define USBH_DRIVER_MSC_H_

#include "usbh_hubbed.h"

#include <stdint.h>
#include <stdlib.h>

BEGIN_DECLS

struct _msc_config {
	void (*update)(uint8_t device_id, uint8_t data);
	void (*notify_connected)(uint8_t device_id);
	void (*notify_disconnected)(uint8_t device_id);
};
typedef struct _msc_config msc_config_t;

struct _msc_packet {

};

void msc_driver_init(const msc_config_t *config);
void msc_read10(uint8_t device_id, void *data, uint32_t block_count, uint32_t lba);
bool msc_ready(uint8_t device_id);
extern const usbh_dev_driver_t usbh_msc_driver;

END_DECLS

#endif

