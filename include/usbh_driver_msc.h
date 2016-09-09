#ifndef USBH_DRIVER_MSC_H_
#define USBH_DRIVER_MSC_H_

#include "usbh_core.h"

#include <stdint.h>
#include <stdlib.h>

BEGIN_DECLS

struct _msc_config {
	/**
	 * @brief Device connected callback
	 */
	void (*notify_connected)(uint8_t device_id);

	/**
	 * @brief Device disconnected callback
	 */
	void (*notify_disconnected)(uint8_t device_id);
};
typedef struct _msc_config msc_config_t;

typedef void(*msc_callback_t)(void *) ;

/**
 * @brief MSC driver initialization routine
 *
 * Calls to @ref usbh_poll are not allowed before this function is called
 * with the correct configuration when the structure @ref usbh_msc_driver is
 * passed to @ref usbh_init
 *
 * @param config
 */
void msc_driver_init(const msc_config_t *config);

/**
 * @brief Read from the msc using SCSI_WRITE_10 command
 *
 * This function uses SCSI_READ_10 command.
 *
 * @param device_id
 * @param data
 * @param block_count
 * @param lba
 * @param callback
 * @param arg argument passed to callback @ref msc_callback_t
 */
void msc_read10(uint8_t device_id, void *data, uint32_t block_count, uint32_t lba, msc_callback_t callback, void *arg);

/**
 * @brief Write to the msc using SCSI_WRITE_10 command
 *
 * Data should remain valid for the duration of the transfer (retry mechanism).
 * When the callback is called, data can be freed or reused
 *
 * This function uses SCSI_WRITE_10 command.
 *
 * @param device_id
 * @param data
 * @param block_count
 * @param lba
 * @param callback
 * @param arg argument passed to callback @ref msc_callback_t
 */
void msc_write10(uint8_t device_id, void *data, uint32_t block_count, uint32_t lba, msc_callback_t callback, void *arg);

/**
 * @brief Checks whether the device is ready to process user's functions
 *
 * Idle state means, that no transaction is being processed. When msc device is idle,
 * functions like msc_writeX, msc_readX can be called.
 *
 * @param device_id
 * @returns true when idle, false otherwise
 */
bool msc_idle(uint8_t device_id);

/**
 * @brief Checks whether the device is present
 * @param device_id
 * @returns true when the device with th provided @ref device_id is connected
 */
bool msc_present(uint8_t device_id);

/**
 * @brief Check whether this device driver was initialized via @ref msc_driver_init
 *
 * @returns true when initialized, false otherwise
 */
bool msc_initialized(void);

/**
 * @brief MSC device driver's structure
 *
 * User should not modify this structure. Instead the pointer to this structure
 * should be passed to @ref usbh_init function.
 *
 * @see usbh_init
 */
extern const usbh_dev_driver_t usbh_msc_driver;

END_DECLS

#endif

