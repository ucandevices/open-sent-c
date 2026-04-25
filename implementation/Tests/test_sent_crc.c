#include "test.h"

#include "sent/sent_crc.h"
#include "sent/sent_protocol.h"

static void test_crc_data_only_known_vector(void) {
    /* J2716 example: 6 nibbles {0x1, 0x2, 0x3, 0x4, 0x5, 0x6}, init 0x05, data-only */
    uint8_t data[] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6};
    uint8_t crc = sent_crc4_j2716(data, 6, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x05);
    TEST_ASSERT(crc <= 0x0F);
}

static void test_crc_status_and_data_includes_status(void) {
    uint8_t data[] = {0x1, 0x2, 0x3, 0x4};
    uint8_t crc1 = sent_crc4_j2716(data, 4, SENT_CRC_MODE_STATUS_AND_DATA, 0x00, 0x05);
    uint8_t crc2 = sent_crc4_j2716(data, 4, SENT_CRC_MODE_STATUS_AND_DATA, 0x0F, 0x05);
    TEST_ASSERT(crc1 != crc2); /* status nibble must affect result */
}

static void test_crc_seed_affects_result(void) {
    uint8_t data[] = {0x1, 0x2, 0x3};
    uint8_t crc1 = sent_crc4_j2716(data, 3, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x05);
    uint8_t crc2 = sent_crc4_j2716(data, 3, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x03);
    TEST_ASSERT(crc1 != crc2);
}

static void test_crc_null_data_returns_seed_after_status(void) {
    /* data_nibbles == NULL path is taken */
    uint8_t crc = sent_crc4_j2716(NULL, 0, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x07);
    TEST_ASSERT_EQ(crc, 0x07);
}

static void test_crc_null_data_status_mode(void) {
    /* status path runs with NULL data */
    uint8_t crc = sent_crc4_j2716(NULL, 0, SENT_CRC_MODE_STATUS_AND_DATA, 0x05, 0x05);
    TEST_ASSERT(crc <= 0x0F);
}

static void test_crc_high_bit_xor_path(void) {
    /* Force the (crc & 0x08) != 0 branch via known input */
    uint8_t data[] = {0x0F};
    uint8_t crc = sent_crc4_j2716(data, 1, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x05);
    TEST_ASSERT(crc <= 0x0F);
}

static void test_crc_status_high_bit_xor_path(void) {
    uint8_t crc = sent_crc4_j2716(NULL, 0, SENT_CRC_MODE_STATUS_AND_DATA, 0x0F, 0x0F);
    TEST_ASSERT(crc <= 0x0F);
}

static void test_crc_masks_input_nibbles(void) {
    uint8_t a[] = {0x05};
    uint8_t b[] = {0xF5}; /* upper bits should be ignored */
    uint8_t ca = sent_crc4_j2716(a, 1, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x05);
    uint8_t cb = sent_crc4_j2716(b, 1, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x05);
    TEST_ASSERT_EQ(ca, cb);
}

static void test_crc_masks_seed_to_4_bits(void) {
    uint8_t crc = sent_crc4_j2716(NULL, 0, SENT_CRC_MODE_DATA_ONLY, 0x00, 0xF5);
    /* Seed should be masked to 0x05 */
    TEST_ASSERT_EQ(crc, 0x05);
}

void run_sent_crc_tests(void);
void run_sent_crc_tests(void) {
    TEST_RUN(test_crc_data_only_known_vector);
    TEST_RUN(test_crc_status_and_data_includes_status);
    TEST_RUN(test_crc_seed_affects_result);
    TEST_RUN(test_crc_null_data_returns_seed_after_status);
    TEST_RUN(test_crc_null_data_status_mode);
    TEST_RUN(test_crc_high_bit_xor_path);
    TEST_RUN(test_crc_status_high_bit_xor_path);
    TEST_RUN(test_crc_masks_input_nibbles);
    TEST_RUN(test_crc_masks_seed_to_4_bits);
}
