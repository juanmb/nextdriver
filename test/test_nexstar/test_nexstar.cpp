#include <ArduinoFake.h>
#include <stdio.h>
#include <stdint.h>
#include <unity.h>
#include "nexstar_base.h"

struct TestPosition {
    uint32_t uint;
    char bin[3];
};


void test_uint32To24bits(void) {
    char out[3];
    uint32To24bits(0x12345678, out);

    char expected[3] = {0x12, 0x34, 0x56};
    TEST_ASSERT_EQUAL_MEMORY(expected, out, 3);

    uint32To24bits(0, out);
    char expected0[3] = {0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL_MEMORY(expected0, out, 3);
}

void test_uint32From24bits(void) {
    char in[3] = {0x12, 0x34, 0x56};
    uint32_t expected = 0x12345600;
    uint32_t out = uint32From24bits(in);
    TEST_ASSERT_EQUAL(expected, out);
}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_uint32To24bits);
    RUN_TEST(test_uint32From24bits);
    UNITY_END();

    return 0;
}
