# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

import pytest

import src.core.db
import src.core.embedding
from src.core.db import VDMSClient


@pytest.fixture
def vdms_client(mocker, tmp_path):
    """
    A pytest fixture to use mock VDMS_Client object
    """
    mocker.patch("src.core.db.VDMS_Client", return_value=None)
    mocker.patch("src.core.embedding.vCLIPEmbeddings", return_value=object)

    mock_vdms = mocker.MagicMock()
    mock_vdms.add_videos.return_value = None
    mocker.patch("src.core.db.VDMS", return_value=mock_vdms)

    client = VDMSClient(host="localhost", port=22222, collection_name="test-index", embedder=object)

    assert client.client == None
    assert client.video_db == mock_vdms

    return client


def test_vdms_client_props(vdms_client, tmp_path):
    """
    Test the VDMS vector DB Class instantiation and method calls
    """
    assert vdms_client.host == "localhost"
    assert vdms_client.port == 22222
    assert vdms_client.video_collection == "test-index"
    assert vdms_client.video_embedder == object
    assert vdms_client.video_metadata_path == tmp_path
    assert vdms_client.embedding_dimensions == 512
    assert vdms_client.video_search_type == "similarity"
    assert vdms_client.constraints is None
    src.core.embedding.vCLIPEmbeddings.assert_called_once_with(model="super-model")


def test_vdms_client_conn(vdms_client):
    """
    Test the various methods in VDMSClient class
    """
    src.core.db.VDMS_Client.assert_called_once_with(host=vdms_client.host, port=vdms_client.port)
    src.core.db.VDMS.assert_called_once()


def test_store_embedding(vdms_client, mocker, tmp_path):
    """
    Test create_embedding methods of VDMSClient
    """
    mock_data = {"video_temp_path": tmp_path, "timestamp": "time", "clip_duration": 30}
    mock_metadata = {"video": mock_data}
    paths = [mock_data["video_temp_path"]]
    mocker.patch("src.core.db.read_config", return_value=mock_metadata)
    vdms_client.store_embeddings()
    src.core.db.read_config.assert_called_once_with(vdms_client.video_metadata_path, type="json")
    vdms_client.video_db.add_videos.assert_called_once_with(
        metadatas=[mock_data],
        paths=paths,
        start_time=[mock_data["timestamp"]],
        clip_duration=[mock_data["clip_duration"]],
    )
