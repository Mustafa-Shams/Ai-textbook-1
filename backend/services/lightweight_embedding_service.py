import numpy as np
from typing import List
import logging
from config.settings import settings

logger = logging.getLogger(__name__)

class EmbeddingService:
    """
    Lightweight service to handle text embeddings without heavy dependencies.
    Uses a simple approach that works within Railway's size constraints.
    """

    def __init__(self):
        # For now, we'll implement a basic embedding approach
        # In production, you might want to use a remote embedding service
        logger.info("Initialized lightweight embedding service")

        # Check if we have the API key needed for embeddings
        if not settings.OPENROUTER_API_KEY or settings.OPENROUTER_API_KEY == "":
            logger.warning("OPENROUTER_API_KEY not found in settings")

    def embed_text(self, text: str) -> List[float]:
        """
        Generate a simple embedding for text using character-level features.
        This is a lightweight alternative to transformer-based embeddings.
        """
        try:
            # Create a simple embedding based on character frequencies
            # This is a basic approach for Railway deployment
            if not text:
                return [0.0] * 384  # Match expected vector size for Qdrant

            # Normalize the text
            text = text.lower()

            # Create a simple character-based embedding
            # Using first 384 features based on character patterns
            embedding = []

            # Character frequency approach
            char_counts = {}
            for char in text:
                char_counts[char] = char_counts.get(char, 0) + 1

            # Create features based on ASCII values and frequencies
            for i in range(384):
                if i < len(text):
                    # Use character at position i
                    char_val = ord(text[i]) / 255.0  # Normalize ASCII value
                    embedding.append(char_val)
                else:
                    # Pad with a function of the text properties
                    padding_val = (hash(text) + i) % 1000 / 1000.0
                    embedding.append(padding_val)

            # Normalize the embedding to unit vector
            emb_array = np.array(embedding)
            norm = np.linalg.norm(emb_array)
            if norm > 0:
                emb_array = emb_array / norm
            return emb_array.tolist()

        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            # Return a default embedding
            return [0.0] * 384

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts
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