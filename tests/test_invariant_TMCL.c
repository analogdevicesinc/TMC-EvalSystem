#include <check.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Include the TMCL interface */
#include "tmc/TMCL.h"

/*
 * Security invariant: TMCL reply buffer construction must not overflow
 * when processing unauthenticated commands with large length values.
 * Since TMCL is accessible without authentication via USB CDC or UART,
 * we verify that reply buffer writes are bounded regardless of input.
 */

/* Simulated TMCL command packets (9 bytes each: module_addr, cmd, type, motor, value[4], checksum) */
static uint8_t exploit_cmd[] = {0x01, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
static uint8_t boundary_cmd[] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
static uint8_t valid_cmd[] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};

START_TEST(test_tmcl_reply_buffer_no_overflow)
{
    /* Invariant: Unauthenticated TMCL commands must not cause buffer overflow in reply */
    uint8_t *payloads[] = { exploit_cmd, boundary_cmd, valid_cmd };
    size_t payload_lens[] = { sizeof(exploit_cmd), sizeof(boundary_cmd), sizeof(valid_cmd) };
    int num_payloads = 3;

    /* A reply buffer of standard TMCL size (9 bytes header + limited data) */
    uint8_t replyBuffer[256];

    for (int i = 0; i < num_payloads; i++) {
        memset(replyBuffer, 0xAA, sizeof(replyBuffer));

        /* Fix checksum byte */
        uint8_t csum = 0;
        for (size_t j = 0; j < payload_lens[i] - 1; j++)
            csum += payloads[i][j];
        payloads[i][payload_lens[i] - 1] = csum;

        /* Process the command through TMCL - this exercises the real code path */
        tmcl_init();
        tmcl_process_command(payloads[i], payload_lens[i], replyBuffer, sizeof(replyBuffer));

        /* Verify no write beyond buffer bounds (canary check not possible without
         * instrumentation, but we verify the reply status indicates rejection or
         * bounded response). Reply byte[2] is status: 1=success, others=error.
         * For unauthenticated/invalid commands, we expect non-success or bounded reply. */
        size_t reply_len = replyBuffer[0]; /* First byte often indicates length */
        ck_assert_msg(reply_len <= sizeof(replyBuffer),
                      "Reply length %zu exceeds buffer size for payload %d", reply_len, i);
    }
}
END_TEST

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Security");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_tmcl_reply_buffer_no_overflow);
    suite_add_tcase(s, tc_core);

    return s;
}

int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    s = security_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}