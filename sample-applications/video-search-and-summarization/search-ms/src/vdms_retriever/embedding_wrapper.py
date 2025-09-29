# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

from pydantic import BaseModel
from langchain_core.embeddings import Embeddings
import requests
from typing import Any, List
from src.utils.common import logger, settings
from urllib.parse import urlparse

def should_use_no_proxy(url: str) -> bool:
    no_proxy = settings.no_proxy_env
    hostname = urlparse(url).hostname
    logger.debug(
        f"Checking no_proxy for hostname: {hostname} against no_proxy domains: {no_proxy}"
    )
    if hostname:
        for domain in no_proxy.split(","):
            if hostname.endswith(domain):
                logger.debug(f"Hostname {hostname} matches no_proxy domain {domain}")
                return True
    return False


class vCLIPEmbeddingsWrapper(BaseModel, Embeddings):
    """Wrapper for vCLIP Embeddings that makes API calls to /embeddings endpoint."""

    api_url: str
    model_name: str
    num_frames: int

    def embed_documents(self, texts: List[str]) -> List[List[float]]:
        logger.debug(f"Embedding documents: {texts}")
        try:
            response = requests.post(
                f"{self.api_url}",
                json={
                    "model": self.model_name,
                    "input": {"type": "text", "text": texts},
                    "encoding_format": "float",
                },
                proxies=(
                    None
                    if should_use_no_proxy(self.api_url)
                    else {"http": settings.http_proxy, "https": settings.https_proxy}
                ),
            )
            logger.debug(f"Response status code: {response.status_code}")
            response.raise_for_status()
            embeddings = response.json()["embedding"]
            return embeddings
        except requests.RequestException as ex:
            logger.error(f"Error in embed_documents: {ex}")
            raise Exception("Error creating embedding") from ex

    def embed_query(self, text: str) -> List[float]:
        logger.debug(f"Embedding query: {text}")
        try:
            response = requests.post(
                f"{self.api_url}",
                json={
                    "model": self.model_name,
                    "input": {"type": "text", "text": text},
                    "encoding_format": "float",
                },
                proxies=(
                    None
                    if should_use_no_proxy(self.api_url)
                    else {"http": settings.http_proxy, "https": settings.https_proxy}
                ),
            )
            logger.debug(f"Response status code: {response.status_code}")
            response.raise_for_status()
            embedding = response.json()["embedding"]
            return embedding
        except requests.RequestException as ex:
            logger.error(f"Error in embed_query: {ex}")
            raise Exception("Error creating embedding") from ex

    def embed_video(self, paths: List[str], **kwargs: Any) -> List[List[float]]:
        logger.debug(f"Embedding videos: {paths} with kwargs: {kwargs}")
        try:
            video_features = []
            for path in paths:
                segment_config = {
                    "startOffsetSec": kwargs["start_time"][0],
                    "clip_duration": kwargs["clip_duration"][0],
                    "num_frames": self.num_frames,
                }
                logger.debug(f"Segment config for {path}: {segment_config}")
                response = requests.post(
                    f"{self.api_url}",
                    json={
                        "model": self.model_name,
                        "input": {
                            "type": "video_file",
                            "video_path": path,
                            "segment_config": segment_config,
                        },
                        "encoding_format": "float",
                    },
                    proxies=(
                        None
                        if should_use_no_proxy(self.api_url)
                        else {
                            "http": settings.http_proxy,
                            "https": settings.https_proxy,
                        }
                    ),
                )
                logger.debug(f"Response status code: {response.status_code}")
                response.raise_for_status()
                embedding = response.json()["embedding"]
                # logger.debug(f"Received embedding for {path}: {embedding}")
                video_features.append(embedding)
            return video_features
        except requests.RequestException as ex:
            logger.error(f"Error in embed_video: {ex}")
            raise Exception("Error creating embedding") from ex


    def get_embedding_length(self) -> int:
        logger.debug(
            f"Getting embedding length self.api_url: {self.api_url} self.model_name: {self.model_name}"
        )
        try:
            if settings.EMBEDDING_LENGTH == 0:
                response = requests.post(
                    f"{self.api_url}",
                    json={
                        "model": self.model_name,
                        "input": {"type": "text", "text": ["sample_text"]},
                        "encoding_format": "float",
                    },
                    proxies=(
                        None
                        if should_use_no_proxy(self.api_url)
                        else {"http": settings.http_proxy, "https": settings.https_proxy}
                    ),
                )
                logger.debug(f"Response status code: {response.status_code}")
                response.raise_for_status()
                embedding = response.json()["embedding"]
                settings.EMBEDDING_LENGTH = len(embedding[0])

            logger.debug(f"Embedding dimension: {settings.EMBEDDING_LENGTH}")
            return settings.EMBEDDING_LENGTH
        except requests.RequestException as ex:
            logger.error(f"Error in get_embedding_length: {ex}")
            raise Exception("Error getting embedding length") from ex
