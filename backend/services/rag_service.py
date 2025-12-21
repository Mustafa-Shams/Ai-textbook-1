import logging
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
import uuid
from config.settings import settings

logger = logging.getLogger(__name__)

class RAGService:
    """
    Service to handle Retrieval-Augmented Generation functionality.
    Connects to Qdrant vector database to search for relevant content.
    """

    def __init__(self, embedding_service):
        self.embedding_service = embedding_service
        self.client = None
        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self._connect_to_qdrant()

    def _connect_to_qdrant(self):
        """Connect to Qdrant vector database"""
        try:
            if settings.QDRANT_URL and settings.QDRANT_API_KEY:
                # For Qdrant Cloud instances, we need to pass the full URL with API key
                # The URL should be the base URL without port for cloud instances
                self.client = QdrantClient(
                    url=settings.QDRANT_URL,
                    api_key=settings.QDRANT_API_KEY,
                    timeout=30
                )
            else:
                logger.warning("QDRANT_URL or QDRANT_API_KEY not set, RAG functionality will be limited")
                self.client = None
                return

            # Verify connection and collection exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                logger.info(f"Creating collection: {self.collection_name}")
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE)
                )

            logger.info(f"Connected to Qdrant collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            logger.info("Running in fallback mode without RAG functionality")
            self.client = None

    async def search_similar_content(self, query: str, limit: int = 5) -> List[Dict]:
        """
        Search for similar content in the vector database based on the query
        """
        try:
            # Check if Qdrant client is available
            if self.client is None:
                logger.warning("Qdrant client not available, returning empty results")
                # Return a default response when RAG is not available
                return [{
                    "content": "RAG functionality is currently unavailable. Using fallback response.",
                    "source": "system",
                    "score": 1.0,
                    "metadata": {}
                }]

            # Generate embedding for the query
            query_embedding = self.embedding_service.embed_text(query)

            # Search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                with_payload=True
            )

            # Format results
            formatted_results = []
            for result in search_results:
                formatted_results.append({
                    "content": result.payload.get("content", ""),
                    "source": result.payload.get("source_path", ""),
                    "score": result.score,
                    "metadata": result.payload.get("metadata", {})
                })

            return formatted_results

        except Exception as e:
            logger.error(f"Error searching for similar content: {e}")
            # Return a default response when there's an error
            return [{
                "content": "Error retrieving content from knowledge base. Using fallback response.",
                "source": "system",
                "score": 1.0,
                "metadata": {}
            }]

    async def add_document_chunk(self, content: str, source_path: str, metadata: Dict = None) -> str:
        """
        Add a document chunk to the vector database
        """
        try:
            # Check if Qdrant client is available
            if self.client is None:
                logger.warning("Qdrant client not available, skipping document addition")
                return str(uuid.uuid4())  # Return a fake ID for compatibility

            # Generate embedding for the content
            embedding = self.embedding_service.embed_text(content)

            # Create a unique ID for the point
            point_id = str(uuid.uuid4())

            # Prepare payload
            payload = {
                "content": content,
                "source_path": source_path,
                "metadata": metadata or {}
            }

            # Upsert the point to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload=payload
                    )
                ]
            )

            return point_id

        except Exception as e:
            logger.error(f"Error adding document chunk: {e}")
            raise

    async def batch_add_document_chunks(self, chunks: List[Dict]) -> List[str]:
        """
        Add multiple document chunks to the vector database
        """
        try:
            # Check if Qdrant client is available
            if self.client is None:
                logger.warning("Qdrant client not available, skipping document addition")
                # Return fake IDs for compatibility
                return [str(uuid.uuid4()) for _ in chunks]

            points = []
            ids = []

            for chunk in chunks:
                content = chunk["content"]
                source_path = chunk["source_path"]
                metadata = chunk.get("metadata", {})

                # Generate embedding for the content
                embedding = self.embedding_service.embed_text(content)

                # Create a unique ID for the point
                point_id = str(uuid.uuid4())
                ids.append(point_id)

                # Prepare payload
                payload = {
                    "content": content,
                    "source_path": source_path,
                    "metadata": metadata
                }

                # Create point structure
                point = PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=payload
                )
                points.append(point)

            # Batch upsert to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            return ids

        except Exception as e:
            logger.error(f"Error batch adding document chunks: {e}")
            raise