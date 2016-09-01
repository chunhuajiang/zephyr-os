/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements gcc-specific startup code for the cc2538.
 */

#include <stdint.h>

#define FLASH_CCA_BOOTLDR_CFG_DISABLE           0xEFFFFFFF ///< Disable backdoor function
#define FLASH_CCA_BOOTLDR_CFG_ENABLE            0xF0FFFFFF ///< Enable backdoor function
#define FLASH_CCA_BOOTLDR_CFG_ACTIVE_HIGH       0x08000000 ///< Selected pin on pad A active high
#define FLASH_CCA_BOOTLDR_CFG_PORT_A_PIN_M      0x07000000 ///< Selected pin on pad A mask
#define FLASH_CCA_BOOTLDR_CFG_PORT_A_PIN_S      24         ///< Selected pin on pad A shift
#define FLASH_CCA_IMAGE_VALID                   0x00000000 ///< Indicates valid image in flash

#define FLASH_CCA_CONF_BOOTLDR_BACKDOOR_PORT_A_PIN  3      ///< Select Button on SmartRF06 Eval Board

typedef struct
{
    uint32_t ui32BootldrCfg;
    uint32_t ui32ImageValid;
    uint32_t ui32ImageVectorAddr;
    uint8_t  ui8lock[32];
} flash_cca_lock_page_t;

extern void *_vector_table[];

__attribute__((__section__(".flashcca"), used))
const flash_cca_lock_page_t flash_cca_lock_page =
{
    FLASH_CCA_BOOTLDR_CFG_ENABLE | (FLASH_CCA_CONF_BOOTLDR_BACKDOOR_PORT_A_PIN << FLASH_CCA_BOOTLDR_CFG_PORT_A_PIN_S),
    FLASH_CCA_IMAGE_VALID,
    (uint32_t) &_vector_table,
    {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    }
};
