
#include <unity/unity.h>

extern int main_device_id_test(void);
extern int main_pstop_client_test(void);
extern int main_machine_test(void);

void setUp(void) {}

void tearDown(void) {}

int
main(void)
{
    main_device_id_test();
    main_pstop_client_test();
    main_machine_test();

    return 0;
}