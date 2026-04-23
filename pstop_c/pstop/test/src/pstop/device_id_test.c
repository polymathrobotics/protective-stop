
#include "pstop/device_id.h"

#include <unity/unity.h>

#include "pstop/test_utils.h"

static
void
test_device_init(void)
{
    device_id_t device;
    for(int i = 0; i < DEVICE_ID_LENGTH; ++i) {
        device.data[i] = i;
    }

    device_id_init(&device);
    for(int i = 0; i < DEVICE_ID_LENGTH; ++i) {
        TEST_ASSERT_EQUAL(device.data[i], 0U);
    }
}

static
void
test_device_cmp(void)
{
    device_id_t lhs;
    device_id_set(&lhs, "testing");

    device_id_t rhs;
    device_id_set(&rhs, "testing");

    TEST_ASSERT_EQUAL(0, device_id_cmp(&lhs, &rhs));
    device_id_set(&rhs, "test");
    TEST_ASSERT_NOT_EQUAL(0, device_id_cmp(&lhs, &rhs));
}

int
main_device_id_test(void)
{
    UnityBegin("device_id_test.c");

    RUN_TEST(test_device_init);
    RUN_TEST(test_device_cmp);

    return UNITY_END();
}
