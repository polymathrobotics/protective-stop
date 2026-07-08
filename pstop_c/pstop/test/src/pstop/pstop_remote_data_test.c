
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "pstop/pstop_remote_data.h"

#include <string.h>

#include <unity/unity.h>

static
void
assert_empty_client(pstop_remote_data_t *client)
{
    TEST_ASSERT_EQUAL(0U, client->remote_data.remote_id.data);
    TEST_ASSERT_EQUAL(0U, client->remote_data.last_timestamp);
    TEST_ASSERT_EQUAL(0U, client->remote_data.heartbeat_ms);
    TEST_ASSERT_EQUAL(0U, client->remote_data.msg_counter);
}

static
void
test_client_init(void)
{
    pstop_remote_data_t client;
    memset(&client, 0x12, sizeof(pstop_remote_data_t));

    pstop_remote_init(&client);
    assert_empty_client(&client);
}

static
void
test_clients_init(void)
{
    pstop_remote_data_t client_data[5];
    pstop_remotes_t clients = {
        .remotes = client_data,
        .max_remotes = 5
    };

    memset(client_data, 0x13, sizeof(pstop_remote_data_t) * 5);

    pstop_remotes_init(&clients);
    for(int i = 0; i < 5; ++i) {
        assert_empty_client(&client_data[i]);
    }
}

static
void
test_get_free_client(void)
{
    pstop_remote_data_t client_data[2];
    pstop_remotes_t clients = {
        .remotes = client_data,
        .max_remotes = 2
    };

    pstop_remotes_init(&clients);
    TEST_ASSERT_EQUAL(0U, pstop_remote_num_active(&clients));

    pstop_remote_data_t *c1 = pstop_remote_get_free_remote(&clients);
    TEST_ASSERT_NOT_NULL(c1);
    TEST_ASSERT_EQUAL(1U, pstop_remote_num_active(&clients));
    assert_empty_client(c1);

    pstop_remote_data_t *c2 = pstop_remote_get_free_remote(&clients);
    TEST_ASSERT_NOT_NULL(c2);
    TEST_ASSERT_EQUAL(2U, pstop_remote_num_active(&clients));
    assert_empty_client(c2);

    pstop_remote_data_t *c3 = pstop_remote_get_free_remote(&clients);
    TEST_ASSERT_NULL(c3);
    TEST_ASSERT_EQUAL(2U, pstop_remote_num_active(&clients));
}

static
void
test_remove_client(void)
{
    pstop_remote_data_t client_data[2];
    pstop_remotes_t clients = {
        .remotes = client_data,
        .max_remotes = 2
    };

    pstop_remotes_init(&clients);
    TEST_ASSERT_EQUAL(0U, pstop_remote_num_active(&clients));

    device_id_t c1_id;
    c1_id.data = 1234;

    device_id_t c2_id;
    c2_id.data = 1236;
    pstop_remote_data_t *c1 = pstop_remote_get_free_remote(&clients);
    pstop_remote_data_t *c2 = pstop_remote_get_free_remote(&clients);

    device_id_copy(&(c1->remote_data.remote_id), &c1_id);
    device_id_copy(&(c2->remote_data.remote_id), &c2_id);

    // remove first client and make sure second client was copied over
    TEST_ASSERT_EQUAL(2U, pstop_remote_num_active(&clients));
    pstop_remote_deactivate(c1);
    TEST_ASSERT_EQUAL(1U, pstop_remote_num_active(&clients));

    // validate that the client is no longer in the list
    TEST_ASSERT_NULL(pstop_remote_get(&clients, &c1_id));

    // verify that the other client is still available
    c2 = pstop_remote_get(&clients, &c2_id);
    TEST_ASSERT_NOT_NULL(c2);
    TEST_ASSERT_EQUAL(0, device_id_cmp(&c2_id, &(c2->remote_data.remote_id)));

    // now remove last client
    pstop_remote_deactivate(c2);
    TEST_ASSERT_EQUAL(0U, pstop_remote_num_active(&clients));
}

void
test_num_stopped(void)
{
    pstop_remote_data_t client_data[2];
    pstop_remotes_t clients = {
        .remotes = client_data,
        .max_remotes = 2
    };

    pstop_remotes_init(&clients);

    TEST_ASSERT_EQUAL(0U, pstop_remote_num_stopped(&clients));
    pstop_remote_data_t *c1 = pstop_remote_get_free_remote(&clients);

    c1->remote_state = PSTOP_REMOTE_STOPPED;
    TEST_ASSERT_EQUAL(1U, pstop_remote_num_stopped(&clients));
    pstop_remote_deactivate(c1);
    TEST_ASSERT_EQUAL(0U, pstop_remote_num_stopped(&clients));
}

void
main_pstop_client_test(void)
{
    UnitySetTestFile("pstop_remote_test.c");

    RUN_TEST(test_client_init);
    RUN_TEST(test_clients_init);
    RUN_TEST(test_get_free_client);
    RUN_TEST(test_remove_client);
    RUN_TEST(test_num_stopped);
}
