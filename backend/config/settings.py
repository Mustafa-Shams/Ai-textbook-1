from pydantic_settings import BaseSettings
from typing import Optional
import os

class Settings(BaseSettings):
    # API Configuration
    API_HOST: str = os.getenv("API_HOST", "0.0.0.0")
    API_PORT: int = int(os.getenv("API_PORT", "8000"))

    # Qdrant Configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_book")

    # Database Configuration
    DATABASE_URL: str = os.getenv("DATABASE_URL", "")

    # OpenRouter Configuration
    OPENROUTER_API_KEY: str = os.getenv("OPENROUTER_API_KEY", "")
    LLM_MODEL: str = os.getenv("LLM_MODEL", "qwen/qwen-2-72b-instruct")

    # Qwen Embedding Model
    QWEN_EMBEDDING_MODEL: str = os.getenv("QWEN_EMBEDDING_MODEL", "Alibaba-NLP/qwen2-embedding")
    QWEN_API_KEY: str = os.getenv("QWEN_API_KEY", "")

    # Rate Limiting
    RATE_LIMIT: str = os.getenv("RATE_LIMIT", "10/minute")

    # GitHub Pages Configuration
    GITHUB_PAGES_URL: str = os.getenv("GITHUB_PAGES_URL", "")

    # Logging
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")

    class Config:
        env_file = ".env"

settings = Settings()