/** 
 * ZuluSCSI™ - Copyright (c) 2022-2025 Rabbit Hole Computing™
 * 
 * ZuluSCSI™ firmware is licensed under the GPL version 3 or any later version. 
 * 
 * https://www.gnu.org/licenses/gpl-3.0.html
 * ----
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. 
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/

// Accelerated SCSI subroutines for SCSI initiator/host side communication

#include "scsi_accel_host.h"
#include "BlueSCSI_platform.h"
#include "BlueSCSI_log.h"
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/structs/iobank0.h>
#include <hardware/sync.h>

#ifdef PLATFORM_HAS_INITIATOR_MODE
# include "scsi_accel_host_RP2MCU.pio.h"

#define SCSI_PIO pio0
#define SCSI_SM 0

static struct {
    // PIO configurations
    uint32_t pio_offset_async_read;
    pio_sm_config pio_cfg_async_read;
    uint32_t pio_offset_async_write;
    pio_sm_config pio_cfg_async_write;
} g_scsi_host;

enum scsidma_state_t { SCSIHOST_IDLE = 0,
                       SCSIHOST_READ,
                       SCSIHOST_WRITE };
static volatile scsidma_state_t g_scsi_host_state;

static void scsi_accel_host_config_gpio()
{
    if (g_scsi_host_state == SCSIHOST_IDLE)
    {
        iobank0_hw->io[SCSI_IO_DB0].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB1].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB2].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB3].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB4].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB5].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB6].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB7].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DBP].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IN_REQ].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_OUT_ACK].ctrl = GPIO_FUNC_SIO;
    }
    else if (g_scsi_host_state == SCSIHOST_READ)
    {
        // 10001000000000000111111111
        // ACK REQ          PDB
        //
        pio_sm_set_pins(SCSI_PIO, SCSI_SM, SCSI_IO_DATA_MASK | 1 << SCSI_IN_REQ | 1 << SCSI_OUT_ACK);
        pio_sm_set_consecutive_pindirs(SCSI_PIO, SCSI_SM, 0, 9, false);  // DBP Input
        pio_sm_set_consecutive_pindirs(SCSI_PIO, SCSI_SM, SCSI_IN_REQ, 1, false);  // REQ Input
        pio_sm_set_consecutive_pindirs(SCSI_PIO, SCSI_SM, SCSI_OUT_ACK, 1, true);  // ACK Output

        iobank0_hw->io[SCSI_IO_DB0].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB1].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB2].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB3].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB4].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB5].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB6].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DB7].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IO_DBP].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_IN_REQ].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_OUT_ACK].ctrl = GPIO_FUNC_PIO0;
    }
    else if (g_scsi_host_state == SCSIHOST_WRITE)
    {
        // Data bus pins driven by PIO output, ACK as PIO sideset output, REQ as input
        pio_sm_set_pins(SCSI_PIO, SCSI_SM, SCSI_IO_DATA_MASK | 1 << SCSI_IN_REQ | 1 << SCSI_OUT_ACK);
#if defined(BLUESCSI_ULTRA_WIDE)
        pio_sm_set_consecutive_pindirs(SCSI_PIO, SCSI_SM, 0, 18, true);  // DB0-DB15+DBP+DBP1 Output
#else
        pio_sm_set_consecutive_pindirs(SCSI_PIO, SCSI_SM, 0, 9, true);   // DB0-DB7+DBP Output
#endif
        pio_sm_set_consecutive_pindirs(SCSI_PIO, SCSI_SM, SCSI_IN_REQ, 1, false);  // REQ Input
        pio_sm_set_consecutive_pindirs(SCSI_PIO, SCSI_SM, SCSI_OUT_ACK, 1, true);  // ACK Output

        iobank0_hw->io[SCSI_IO_DB0].ctrl  = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB1].ctrl  = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB2].ctrl  = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB3].ctrl  = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB4].ctrl  = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB5].ctrl  = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB6].ctrl  = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB7].ctrl  = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DBP].ctrl  = GPIO_FUNC_PIO0;
#if defined(BLUESCSI_ULTRA_WIDE)
        iobank0_hw->io[SCSI_IO_DB8].ctrl  = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB9].ctrl  = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB10].ctrl = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB11].ctrl = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB12].ctrl = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB13].ctrl = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB14].ctrl = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DB15].ctrl = GPIO_FUNC_PIO0;
        iobank0_hw->io[SCSI_IO_DBP1].ctrl = GPIO_FUNC_PIO0;
#endif
        iobank0_hw->io[SCSI_IN_REQ].ctrl  = GPIO_FUNC_SIO;
        iobank0_hw->io[SCSI_OUT_ACK].ctrl = GPIO_FUNC_PIO0;

        // Enable output on external data bus buffer
        sio_hw->gpio_set = (1 << SCSI_DATA_DIR);
    }
}

uint32_t scsi_accel_host_read(uint8_t *buf, uint32_t count, int *parityError, int busWidth, volatile int *resetFlag)
{
    // Currently this method just reads from the PIO RX fifo directly in software loop.
    // The SD card access is parallelized using DMA, so there is limited benefit from using DMA here.
    g_scsi_host_state = SCSIHOST_READ;

    int cd_start = SCSI_IN(CD);
    int msg_start = SCSI_IN(MSG);

    pio_sm_init(SCSI_PIO, SCSI_SM, g_scsi_host.pio_offset_async_read, &g_scsi_host.pio_cfg_async_read);
    scsi_accel_host_config_gpio();
    pio_sm_set_enabled(SCSI_PIO, SCSI_SM, true);

    // Set the number of bytes to read, must be divisible by 2.
    assert((count & 1) == 0);
    if (busWidth == 0)
    {
        // 8-bit bus
        pio_sm_put(SCSI_PIO, SCSI_SM, count - 1);
    }
    else
    {
        // 16-bit bus
        pio_sm_put(SCSI_PIO, SCSI_SM, (count / 2) - 1);
    }

    // Read results from PIO RX FIFO
    uint8_t *dst = buf;
    uint8_t *end = buf + count;
    uint32_t paritycheck = 0xFFFFFFFF;
    uint32_t prev_rx_time = platform_millis();
    while (dst < end)
    {
        uint32_t available = pio_sm_get_rx_fifo_level(SCSI_PIO, SCSI_SM);

        if (available == 0)
        {
            // No new data has been received by PIO, check if there is a need to abort

            bool abort = false;
            if (*resetFlag)
            {
                dbgmsg("scsi_accel_host_read: Aborting due to reset request");
                abort = true;
            }
            else if ((platform_millis() - prev_rx_time) > 10000)
            {
                dbgmsg("scsi_accel_host_read: Aborting due to timeout");
                abort = true;
            }
            else
            {
                // Some drives such as ST-296N may have glitches on phase signals in between
                // byte transfers. This is allowed by SCSI spec, and officially we should only
                // check the phase signals when REQ is active. However the PIO logic currently
                // does not do this. Instead, when we detect a phase change, wait for 10 milliseconds
                // to see if it is real.
                int debounce = 100;
                while (debounce > 0 && (!SCSI_IN(IO) || SCSI_IN(CD) != cd_start || SCSI_IN(MSG) != msg_start))
                {
                    debounce--;
                    platform_delay_us(100);
                }

                if (debounce == 0)
                {
                    dbgmsg("scsi_accel_host_read: aborting because target switched transfer phase (IO: ",
                        (int)SCSI_IN(IO), ", CD: ", (int)SCSI_IN(CD), ", MSG: ", (int)SCSI_IN(MSG), ")");
                    abort = true;
                }
            }

            if (abort)
            {
                count = dst - buf;
                break;
            }
        }

        if (busWidth == 0)
        {
            // 8-bit bus
            // For normal BlueSCSI, there are two bytes per PIO word.
            // For wide BlueSCSI, there is one byte per PIO word.

            while (available > 0)
            {
                available--;
                uint32_t word = pio_sm_get(SCSI_PIO, SCSI_SM);
                paritycheck ^= word;
                word = ~word;

#ifdef BLUESCSI_ULTRA_WIDE
                *dst++ = word & 0xFF;
#else
                *dst++ = word & 0xFF;
                *dst++ = word >> 16;
#endif
            }
        }
        else
        {
            // 16-bit bus
            while (available > 0)
            {
                available--;
                uint32_t word = pio_sm_get(SCSI_PIO, SCSI_SM);
                paritycheck ^= word;
                word = ~word;

                *dst++ = word & 0xFF;
                *dst++ = (word >> 8) & 0xFF;
            }
        }
    }

#ifdef BLUESCSI_ULTRA_WIDE
    bool parity_ok;
    if (busWidth == 0)
        parity_ok = scsi_check_parity(paritycheck);
    else
        parity_ok = scsi_check_parity_16bit(paritycheck);
#else
    bool parity_ok = scsi_check_parity(paritycheck & 0xFFFF) && scsi_check_parity(paritycheck >> 16);
#endif

    // Check parity errors in whole block
    // This doesn't detect if there is even number of parity errors in block.
    if (!parity_ok)
    {
        logmsg("Parity error in scsi_accel_host_read(): ", paritycheck);
        *parityError = 1;
    }
    g_scsi_host_state = SCSIHOST_IDLE;
    SCSI_RELEASE_DATA_REQ();
    scsi_accel_host_config_gpio();
    pio_sm_set_enabled(SCSI_PIO, SCSI_SM, false);

    return count;
}


uint32_t scsi_accel_host_write(const uint8_t *data, uint32_t count, int busWidth, volatile int *resetFlag)
{
    // This method writes the data to the PIO TX fifo in a software loop.
    // The PIO drives the data bus, asserts ACK and waits for the target
    // handshake, while the CPU pushes the next words into the fifo.
    g_scsi_host_state = SCSIHOST_WRITE;

    int cd_start = SCSI_IN(CD);
    int msg_start = SCSI_IN(MSG);

    uint32_t words;
    if (busWidth == 0)
    {
        // 8-bit bus: one byte per PIO word
        words = count;
    }
    else
    {
        // 16-bit bus: two bytes per PIO word
        assert((count & 1) == 0);
        words = count / 2;
    }
    if (words == 0)
    {
        g_scsi_host_state = SCSIHOST_IDLE;
        return count;
    }

    pio_sm_init(SCSI_PIO, SCSI_SM, g_scsi_host.pio_offset_async_write, &g_scsi_host.pio_cfg_async_write);
    scsi_accel_host_config_gpio();
    pio_sm_set_enabled(SCSI_PIO, SCSI_SM, true);

    // Set the number of words to send
    pio_sm_put(SCSI_PIO, SCSI_SM, words - 1);

    // Push data words to PIO TX FIFO
    const uint8_t *src = data;
    uint32_t sent = 0;
    uint32_t prev_tx_time = platform_millis();
    bool abort = false;

    while (sent < words)
    {
        // TX FIFO is only 4 words deep, and pio_sm_put() silently drops
        // the value if the fifo is full, so only push when there is room.
        if (pio_sm_get_tx_fifo_level(SCSI_PIO, SCSI_SM) < 4)
        {
            uint32_t word = 0;
            if (busWidth == 0)
            {
                // 8-bit bus: the parity lookup table contains the GPIO value
                // for the 8 data bits + 1 parity bit
                word = g_scsi_parity_lookup[*src++];
            }
#ifdef BLUESCSI_ULTRA_WIDE
            else
            {
                // 16-bit bus: generate parity for the 16 data bits + 2 parity bits
                uint16_t w = (uint16_t)src[0] | ((uint16_t)src[1] << 8);
                src += 2;
                word = scsi_generate_parity(w);
            }
#endif
            pio_sm_put(SCSI_PIO, SCSI_SM, word);
            sent++;
            prev_tx_time = platform_millis();
        }
        else
        {
            // TX FIFO full, check if there is a need to abort

            if (*resetFlag)
            {
                dbgmsg("scsi_accel_host_write: Aborting due to reset request");
                abort = true;
                break;
            }
            else if ((platform_millis() - prev_tx_time) > 10000)
            {
                dbgmsg("scsi_accel_host_write: Aborting due to timeout");
                abort = true;
                break;
            }
            else
            {
                // Some drives such as ST-296N may have glitches on phase signals in between
                // byte transfers. Debounce phase changes before aborting.
                int debounce = 100;
                while (debounce > 0 && (SCSI_IN(IO) || SCSI_IN(CD) != cd_start || SCSI_IN(MSG) != msg_start))
                {
                    debounce--;
                    platform_delay_us(100);
                }

                if (debounce == 0)
                {
                    dbgmsg("scsi_accel_host_write: aborting because target switched transfer phase (IO: ",
                        (int)SCSI_IN(IO), ", CD: ", (int)SCSI_IN(CD), ", MSG: ", (int)SCSI_IN(MSG), ")");
                    abort = true;
                    break;
                }
            }
        }
    }

    // Wait for the completion marker pushed by the PIO after the last word
    if (!abort)
    {
        uint32_t prev_rx_time = platform_millis();
        while (pio_sm_get_rx_fifo_level(SCSI_PIO, SCSI_SM) == 0)
        {
            if (*resetFlag)
            {
                dbgmsg("scsi_accel_host_write: Aborting due to reset request");
                abort = true;
                break;
            }
            else if ((platform_millis() - prev_rx_time) > 10000)
            {
                dbgmsg("scsi_accel_host_write: Aborting due to timeout");
                abort = true;
                break;
            }
            else
            {
                int debounce = 100;
                while (debounce > 0 && (SCSI_IN(IO) || SCSI_IN(CD) != cd_start || SCSI_IN(MSG) != msg_start))
                {
                    debounce--;
                    platform_delay_us(100);
                }

                if (debounce == 0)
                {
                    dbgmsg("scsi_accel_host_write: aborting because target switched transfer phase (IO: ",
                        (int)SCSI_IN(IO), ", CD: ", (int)SCSI_IN(CD), ", MSG: ", (int)SCSI_IN(MSG), ")");
                    abort = true;
                    break;
                }
            }
        }

        // Drain the completion marker
        while (pio_sm_get_rx_fifo_level(SCSI_PIO, SCSI_SM) > 0)
        {
            pio_sm_get(SCSI_PIO, SCSI_SM);
        }
    }

    if (abort)
    {
        // Estimate how many bytes have been fully transferred.
        // Words still in the TX FIFO plus the word currently being
        // processed by the PIO have not completed the handshake.
        uint32_t in_flight = pio_sm_get_tx_fifo_level(SCSI_PIO, SCSI_SM) + 1;
        uint32_t done = (sent > in_flight) ? sent - in_flight : 0;
        count = done * (busWidth == 0 ? 1 : 2);
    }

    g_scsi_host_state = SCSIHOST_IDLE;
    SCSI_RELEASE_DATA_REQ();
    scsi_accel_host_config_gpio();
    pio_sm_set_enabled(SCSI_PIO, SCSI_SM, false);

    return count;
}


void scsi_accel_host_init()
{
    g_scsi_host_state = SCSIHOST_IDLE;
    scsi_accel_host_config_gpio();

    // Load PIO programs
    pio_clear_instruction_memory(SCSI_PIO);

    // Asynchronous / synchronous SCSI read
#if defined (BLUESCSI_ULTRA_WIDE)
    g_scsi_host.pio_offset_async_read = pio_add_program(SCSI_PIO, &scsi_host_async_read_wide_program);
#else
    g_scsi_host.pio_offset_async_read = pio_add_program(SCSI_PIO, &scsi_host_async_read_program);
#endif
    //    wait 0 gpio REQ             side 1  ; Wait for REQ low
    uint16_t instr = pio_encode_wait_gpio(false, SCSI_IN_REQ) | pio_encode_sideset(1, 1);
    SCSI_PIO->instr_mem[g_scsi_host.pio_offset_async_read + 2] = instr;
    instr =   pio_encode_wait_gpio(true, SCSI_IN_REQ) | pio_encode_sideset(1, 0);
    SCSI_PIO->instr_mem[g_scsi_host.pio_offset_async_read + 5] = instr;
    g_scsi_host.pio_cfg_async_read = scsi_host_async_read_program_get_default_config(g_scsi_host.pio_offset_async_read);
    sm_config_set_in_pins(&g_scsi_host.pio_cfg_async_read, SCSI_IO_DB0);
    sm_config_set_sideset_pins(&g_scsi_host.pio_cfg_async_read, SCSI_OUT_ACK);
    sm_config_set_out_shift(&g_scsi_host.pio_cfg_async_read, true, false, 32);
    sm_config_set_in_shift(&g_scsi_host.pio_cfg_async_read, true, true, 32);

    // Asynchronous / synchronous SCSI write
#if defined (BLUESCSI_ULTRA_WIDE)
    g_scsi_host.pio_offset_async_write = pio_add_program(SCSI_PIO, &scsi_host_async_write_wide_program);
#else
    g_scsi_host.pio_offset_async_write = pio_add_program(SCSI_PIO, &scsi_host_async_write_program);
#endif
    //    wait 0 gpio REQ             side 1  ; Wait for REQ low
    instr = pio_encode_wait_gpio(false, SCSI_IN_REQ) | pio_encode_sideset(1, 1);
    SCSI_PIO->instr_mem[g_scsi_host.pio_offset_async_write + 3] = instr;
    //    wait 1 gpio REQ             side 0  ; Wait for REQ high
    instr = pio_encode_wait_gpio(true, SCSI_IN_REQ) | pio_encode_sideset(1, 0);
    SCSI_PIO->instr_mem[g_scsi_host.pio_offset_async_write + 14] = instr;
    g_scsi_host.pio_cfg_async_write = scsi_host_async_write_program_get_default_config(g_scsi_host.pio_offset_async_write);
#if defined(BLUESCSI_ULTRA_WIDE)
    sm_config_set_out_pins(&g_scsi_host.pio_cfg_async_write, SCSI_IO_DB0, 18);
#else
    sm_config_set_out_pins(&g_scsi_host.pio_cfg_async_write, SCSI_IO_DB0, 9);
#endif
    sm_config_set_sideset_pins(&g_scsi_host.pio_cfg_async_write, SCSI_OUT_ACK);
    sm_config_set_out_shift(&g_scsi_host.pio_cfg_async_write, true, false, 32);
    sm_config_set_in_shift(&g_scsi_host.pio_cfg_async_write, true, false, 32);
}

#endif // PLATFORM_HAS_INITIATOR_MODE
