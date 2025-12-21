import numpy as np
from typing import List, Union
import logging
from transformers import AutoTokenizer, AutoModel
import torch

logger = logging.getLogger(__name__)

class EmbeddingService:
    """
    Service to handle text embeddings using a sentence transformer model.
    Uses sentence-transformers/all-MiniLM-L6-v2 model for generating embeddings.
    """

    def __init__(self):
        self.model_name = "sentence-transformers/all-MiniLM-L6-v2"
        self.tokenizer = None
        self.model = None
        self._load_model()

    def _load_model(self):
        """Load the sentence transformer embedding model"""
        try:
            logger.info(f"Loading sentence transformer embedding model: {self.model_name}")
            self.tokenizer = AutoTokenizer.from_pretrained(self.model_name)

            # Add pad token if it doesn't exist
            if self.tokenizer.pad_token is None:
                self.tokenizer.pad_token = self.tokenizer.eos_token

            self.model = AutoModel.from_pretrained(self.model_name)
            logger.info("Sentence transformer embedding model loaded successfully")
        except Exception as e:
            logger.error(f"Failed to load sentence transformer embedding model: {e}")
            raise

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text
        """
        try:
            inputs = self.tokenizer(
                text,
                return_tensors="pt",
                padding=True,
                truncation=True,
                max_length=512
            )

            with torch.no_grad():
                outputs = self.model(**inputs)

            # Use the mean of the last hidden state as the embedding
            embedding = outputs.last_hidden_state.mean(dim=1).squeeze().numpy()

            # Convert to list and return
            return embedding.tolist()
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

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