/* SAE J2716 CRC-4 implementation (polynomial 0x0D).
 *
 * Used by both the encoder (to append the correct CRC nibble) and the
 * decoder (to verify the received CRC nibble). */
#ifndef SENT_SENT_CRC_H
#define SENT_SENT_CRC_H

#include <stddef.h>
#include <stdint.h>

#include "sent/sent_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Compute a 4-bit CRC over data_nibbles using the SAE J2716 algorithm.
 * mode controls whether the status nibble is mixed in before the data.
 * init_seed: 0x05 for legacy, 0x03 for the APR2016 recommendation.
 * Returns the 4-bit CRC result (0x0–0xF). */
uint8_t sent_crc4_j2716(const uint8_t* data_nibbles,
                        size_t nibble_count,
                        sent_crc_mode_t mode,
                        uint8_t status_nibble,
                        uint8_t init_seed);

/* Compute the SAE J2716 Enhanced Serial Message 6-bit CRC.
 * message_bits contains the 24 ESM CRC input bits in receive order, MSB-first:
 * [frame 7 bit2, frame 7 bit3, ... frame 18 bit2, frame 18 bit3].
 * Uses polynomial x^6 + x^4 + x^3 + 1 and seed 0x15. */
uint8_t sent_crc6_esm_j2716(uint32_t message_bits);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_SENT_CRC_H */
