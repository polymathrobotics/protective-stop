
#include "pstop/pstop_client.h"

#include <string.h>

#include <unity/unity.h>

#include "pstop/test_utils.h"

static
void
assert_empty_client(pstop_client_data_t *client)
{
    for(int i = 0; i < DEVICE_ID_LENGTH; ++i) {
        TEST_ASSERT_EQUAL(0U, client->client_id.data[i]);
    }
    TEST_ASSERT_EQUAL(0U, client->last_timestamp);
    TEST_ASSERT_EQUAL(0U, client->heartbeat_ms);
    TEST_ASSERT_EQUAL(0U, client->msg_counter);
    TEST_ASSERT_EQUAL(0U, client->clock_drift);
}

static
void
test_client_init(void)
{
    pstop_client_data_t client;
    memset(&client, 0x12, sizeof(pstop_client_data_t));

    pstop_client_init(&client);
    assert_empty_client(&client);
}

static
void
test_clients_init(void)
{
    pstop_client_data_t client_data[5];
    pstop_clients_t clients = {
        .clients = client_data,
        .max_clients = 5
    };

    memset(client_data, 0x13, sizeof(pstop_client_data_t) * 5);

    pstop_clients_init(&clients);
    for(int i = 0; i < 5; ++i) {
        assert_empty_client(&client_data[i]);
    }
}

static
void
test_get_free_client(void)
{
    pstop_client_data_t client_data[2];
    pstop_clients_t clients = {
        .clients = client_data,
        .max_clients = 2
    };

    pstop_clients_init(&clients);
    TEST_ASSERT_EQUAL(0U, pstop_client_num_active(&clients));

    pstop_client_data_t *c1 = pstop_client_get_free_client(&clients);
    TEST_ASSERT_NOT_NULL(c1);
    TEST_ASSERT_EQUAL(1U, pstop_client_num_active(&clients));
    assert_empty_client(c1);

    pstop_client_data_t *c2 = pstop_client_get_free_client(&clients);
    TEST_ASSERT_NOT_NULL(c2);
    TEST_ASSERT_EQUAL(2U, pstop_client_num_active(&clients));
    assert_empty_client(c2);

    pstop_client_data_t *c3 = pstop_client_get_free_client(&clients);
    TEST_ASSERT_NULL(c3);
    TEST_ASSERT_EQUAL(2U, pstop_client_num_active(&clients));
}

static
void
test_remove_client(void)
{
    pstop_client_data_t client_data[2];
    pstop_clients_t clients = {
        .clients = client_data,
        .max_clients = 2
    };

    pstop_clients_init(&clients);
    TEST_ASSERT_EQUAL(0U, pstop_client_num_active(&clients));

    device_id_t c1_id;
    device_id_set_str(&c1_id, "test");

    device_id_t c2_id;
    device_id_set_str(&c2_id, "test2");
    pstop_client_data_t *c1 = pstop_client_get_free_client(&clients);
    pstop_client_data_t *c2 = pstop_client_get_free_client(&clients);

    device_id_copy(&(c1->client_id), &c1_id);
    device_id_copy(&(c2->client_id), &c2_id);

    // remove first client and make sure second client was copied over
    TEST_ASSERT_EQUAL(2U, pstop_client_num_active(&clients));
    pstop_client_deactivate(c1);
    TEST_ASSERT_EQUAL(1U, pstop_client_num_active(&clients));

    // validate that the client is no longer in the list
    TEST_ASSERT_NULL(pstop_client_get(&clients, &c1_id));

    // verify that the other client is still available
    c2 = pstop_client_get(&clients, &c2_id);
    TEST_ASSERT_NOT_NULL(c2);
    TEST_ASSERT_EQUAL(0, device_id_cmp(&c2_id, &(c2->client_id)));

    // now remove last client
    pstop_client_deactivate(c2);
    TEST_ASSERT_EQUAL(0U, pstop_client_num_active(&clients));
}

void
main_pstop_client_test(void)
{
    UnitySetTestFile("pstop_client_test.c");

    RUN_TEST(test_client_init);
    RUN_TEST(test_clients_init);
    RUN_TEST(test_get_free_client);
    RUN_TEST(test_remove_client);
}
