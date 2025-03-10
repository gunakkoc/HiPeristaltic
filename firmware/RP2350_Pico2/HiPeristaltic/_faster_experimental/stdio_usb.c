/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tusb.h"
#include "pico/stdio_usb.h"

// these may not be set if the user is providing tud support (i.e. LIB_TINYUSB_DEVICE is 1 because
// the user linked in tinyusb_device) but they haven't selected CDC

#include "pico/binary_info.h"
#include "pico/time.h"
#include "pico/stdio/driver.h"
#include "pico/mutex.h"
#include "hardware/irq.h"
#include "device/usbd_pvt.h" // for usbd_defer_func

void stdio_usb_tud_task(){
    tud_task();
}

int stdio_usb_out_available(){
    return tud_cdc_write_available();
}

void stdio_usb_out_chars(const char *buf, int length) {
    //write multiple chars
    tud_cdc_write(buf, (uint32_t)length);
}

void stdio_usb_out_char(char ch) {
    //write single char
    tud_cdc_write_char(ch);
}

void stdio_usb_out_flush(void) {
    //flushes all
    do {
        tud_task();
    } while (tud_cdc_write_flush());
}

uint32_t stdio_usb_out_flush_single(void) {
    //returns non-zero if there is something left to be flushed
    return tud_cdc_write_flush();
}

int stdio_usb_in_chars(char *buf, int length) {
    //if (stdio_usb_connected() && tud_cdc_available())
    int count = (int) tud_cdc_read(buf, (uint32_t) length);
    return count;
}

void stdio_usb_in_char(char *buf) {
    //if (stdio_usb_connected() && tud_cdc_available())
    tud_cdc_read(buf, 1);
}

int stdio_usb_in_available() {
    return tud_cdc_available();
}


bool stdio_usb_connected(void) {
#if PICO_STDIO_USB_CONNECTION_WITHOUT_DTR
    return tud_ready();
#else
    // this actually checks DTR
    return tud_cdc_connected();
#endif
}

bool stdio_usb_init(void) {
#if !PICO_NO_BI_STDIO_USB
    bi_decl_if_func_used(bi_program_feature("USB stdin / stdout"));
#endif
    tusb_init();
    bool rc = true;
    if (rc) {
        stdio_set_driver_enabled(&stdio_usb, true);
    }
    return rc;
}

bool stdio_usb_deinit(void) {
    assert(tud_inited()); // we expect the caller to have initialized when calling sdio_usb_init
    bool rc = true;
    stdio_set_driver_enabled(&stdio_usb, false);

#if PICO_STDIO_USB_DEINIT_DELAY_MS != 0
    sleep_ms(PICO_STDIO_USB_DEINIT_DELAY_MS);
#endif
    return rc;
}

stdio_driver_t stdio_usb = {
    .out_chars = stdio_usb_out_chars,
    .out_flush = stdio_usb_out_flush,
    .in_chars = stdio_usb_in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = PICO_STDIO_USB_DEFAULT_CRLF
#endif
};

