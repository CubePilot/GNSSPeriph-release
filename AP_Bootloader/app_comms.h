/*
  application -> bootloader communication structure This is put into
  the start of RAM by AP_Periph to facilitate firmware upload with
  UAVCAN
 */

#pragma once

#define APP_BOOTLOADER_COMMS_MAGIC 0xc544ad9a

#ifndef APP_COMMS_RAM_START
#define APP_COMMS_RAM_START HAL_RAM0_START
#endif

#if defined(HAL_BOOTLOADER_FALLBACK) && HAL_BOOTLOADER_FALLBACK
#define IS_FALLBACK_BL 1
#else
#define IS_FALLBACK_BL 0
#endif
struct app_bootloader_comms {
    uint32_t magic;
    uint32_t reserved[3];
    uint8_t fallback_bl;
    uint8_t server_node_id;
    uint8_t my_node_id;
    uint8_t path[201];
};
