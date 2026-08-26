
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <unity/unity.h>

extern void main_req_2_01_test(void);
extern void main_req_2_02_test(void);
extern void main_req_2_03_test(void);

extern void main_req_2_06_test(void);
extern void main_req_2_07_test(void);
extern void main_req_2_08_test(void);

extern void main_req_3_01_test(void);
extern void main_req_3_02_test(void);
extern void main_req_3_03_test(void);
extern void main_req_3_04_test(void);
extern void main_req_3_05_test(void);
extern void main_req_3_06_test(void);
extern void main_req_3_07_test(void);
extern void main_req_3_08_test(void);
extern void main_req_3_09_test(void);
extern void main_req_3_10_test(void);
extern void main_req_3_11_test(void);
extern void main_req_3_12_test(void);
extern void main_req_3_13_test(void);
extern void main_req_3_14_test(void);
extern void main_req_3_15_test(void);
extern void main_req_3_16_test(void);
extern void main_req_3_17_test(void);
extern void main_req_3_18_test(void);
extern void main_req_3_19_test(void);
extern void main_req_3_20_test(void);

void setUp(void) {}

void tearDown(void) {}

int
main(void)
{
    UnityBegin("PSTOP Requirements");

    main_req_2_01_test();
    main_req_2_02_test();
    main_req_2_03_test();

    main_req_2_06_test();
    main_req_2_07_test();
    main_req_2_08_test();

    main_req_3_01_test();
    main_req_3_02_test();
    main_req_3_03_test();
    main_req_3_04_test();
    main_req_3_05_test();
    main_req_3_06_test();
    main_req_3_07_test();
    main_req_3_08_test();
    main_req_3_09_test();
    main_req_3_10_test();
    main_req_3_11_test();
    main_req_3_12_test();
    main_req_3_13_test();
    main_req_3_14_test();
    main_req_3_15_test();
    main_req_3_16_test();
    main_req_3_17_test();
    main_req_3_18_test();
    main_req_3_19_test();
    main_req_3_20_test();

    return UNITY_END();
}
