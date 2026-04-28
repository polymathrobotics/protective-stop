# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
import pytest
from protective_stop_remote.app import config_app


@pytest.fixture
def client():
    with config_app().test_client() as client:
        yield client


def test_data_endpoint(client):
    response = client.get('/data')
    assert response.status_code == 200
    assert response.json['status_summary'] == 'Not Connected'
