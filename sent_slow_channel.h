#ifndef SENT_SENT_SLOW_CHANNEL_H
#define SENT_SENT_SLOW_CHANNEL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENT_SLOW_FORMAT_NONE = 0,
    SENT_SLOW_FORMAT_SHORT = 1,
    SENT_SLOW_FORMAT_ENHANCED_12_8 = 2,
    SENT_SLOW_FORMAT_ENHANCED_16_4 = 3,
} sent_slow_format_t;

typedef struct {
    sent_slow_format_t format;
    uint8_t message_id;
    uint16_t data;
} sent_slow_message_t;

typedef struct {
    uint8_t esm_frame_counter;
    uint8_t esm_active;
    uint8_t esm_b3_shift;
    uint8_t esm_b2_shift;
    uint8_t esm_rx_crc;
    uint8_t esm_config_bit;
    uint8_t esm_crc_bits;
    uint32_t esm_crc_message;
    uint16_t esm_data_bits;
    uint16_t esm_id_bits;
    uint8_t esm_data_high_bits;

    uint8_t short_active;
    uint8_t short_bit_count;
    uint16_t short_bits;
} sent_slow_channel_t;

void sent_slow_channel_init(sent_slow_channel_t* ctx);
void sent_slow_channel_reset(sent_slow_channel_t* ctx);

bool sent_slow_channel_process_status(sent_slow_channel_t* ctx,
                                      uint8_t status_nibble,
                                      sent_slow_message_t* out_message);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_SENT_SLOW_CHANNEL_H */
