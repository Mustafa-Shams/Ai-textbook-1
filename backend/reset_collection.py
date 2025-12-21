#!/usr/bin/env python3
"""
Script to reset the Qdrant collection with the correct vector dimension
"""
import os
import sys
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def reset_collection():
    print("Resetting Qdrant collection...")

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

if __name__ == "__main__":
    reset_collection()