import numpy as np
from typing import List
import logging
from openai import OpenAI
from config.settings import settings

logger = logging.getLogger(__name__)

class EmbeddingService:
    """
    Service to handle text embeddings using OpenAI's embedding API.
    Uses text-embedding-ada-002 model for generating embeddings.
    This is a lighter alternative to local sentence transformers.
    """

    def __init__(self):
        self.client = OpenAI(
            api_key=settings.OPENAI_API_KEY,
            base_url=settings.OPENAI_BASE_URL  # For OpenRouter
        )
        self.model = "text-embedding-ada-002"  # Compatible with OpenRouter

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using OpenAI API
        """
        try:
            # Truncate text if it's too long (OpenAI has token limits)
            max_length = 8192  # Conservative limit for embeddings
            if len(text) > max_length:
                text = text[:max_length]
                logger.warning(f"Text truncated to {max_length} characters for embedding")

            response = self.client.embeddings.create(
                input=text,
                model=self.model
            )

            embedding = response.data[0].embedding
            return embedding
        except Exception as e:
            logger.error(f"Error generating embedding with OpenAI API: {e}")
            # Fallback: return a simple embedding based on character counts
            # This is just a placeholder - in production, you'd want proper error handling
            return [0.0] * 1536  # OpenAI embeddings are 1536-dimensional

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts
        For efficiency, we'll call the API individually for each text
        In a production environment, you might want to batch these calls
        """
        embeddings = []
        for text in texts:
            embedding = self.embed_text(text)
            embeddings.append(embedding)
        return embeddings

    def similarity(self, embedding1: List[float], embedding2: List[float]) -> float:
        """
        Calculate cosine similarity between two embeddings
        """
        emb1 = np.array(embedding1)
        emb2 = np.array(embedding2)

        # Calculate cosine similarity
        dot_product = np.dot(emb1, emb2)
        norm1 = np.linalg.norm(emb1)
        norm2 = np.linalg.norm(emb2)

        if norm1 == 0 or norm2 == 0:
            return 0.0

        return float(dot_product / (norm1 * norm2))