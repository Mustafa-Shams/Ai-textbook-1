#!/usr/bin/env python3
"""
Script to reset the Qdrant collection and re-ingest proper documentation
"""
import os
import sys
import asyncio
from pathlib import Path

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, str(Path.cwd()))

from qdrant_client import QdrantClient
from dotenv import load_dotenv
from ingest import DocumentIngestor

async def reset_and_reingest():
    print("Resetting Qdrant collection and re-ingesting documentation...")

    # Load environment variables
    load_dotenv()

    # Connect to Qdrant
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        timeout=30
    )

    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_book")

    # Delete the existing collection
    try:
        client.delete_collection(collection_name)
        print(f"Deleted collection: {collection_name}")
    except Exception as e:
        print(f"Collection {collection_name} might not exist yet, or error occurred: {e}")

    print(f"Collection {collection_name} will be recreated with correct dimensions on next use")

    # Re-ingest the proper documentation
    print("\nStarting re-ingestion of proper documentation...")
    docs_path = Path('../website/docs').resolve()
    print(f'Processing documentation from: {docs_path}')

    ingestor = DocumentIngestor()
    total_chunks = await ingestor.ingest_directory(docs_path)
    print(f'\nSUCCESS: Re-ingestion completed! Total chunks added: {total_chunks}')

    # Verify by searching for a test query
    print('\nTesting search functionality...')
    test_results = await ingestor.rag_service.search_similar_content('Physical AI and Humanoid Robotics', limit=3)
    print(f'Found {len(test_results)} relevant results for test query')
    for i, result in enumerate(test_results):
        print(f'  Result {i+1}: Score: {result["score"]:.3f}, Source: {result["source"]}')

if __name__ == "__main__":
    asyncio.run(reset_and_reingest())