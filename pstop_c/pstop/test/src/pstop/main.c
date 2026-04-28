
#include <unity/unity.h>

extern void main_device_id_test(void);
extern void main_pstop_client_test(void);
extern void main_machine_test(void);
extern void main_machine_timeout_test(void);

void setUp(void) {}

void tearDown(void) {}

int
main(void)
{
    UnityBegin("PSTOP");

    main_device_id_test();
    main_pstop_client_test();
    main_machine_test();
    main_machine_timeout_test();

    return UNITY_END();
}
