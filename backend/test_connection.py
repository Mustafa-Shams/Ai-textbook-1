#!/usr/bin/env python3
"""
Test script to verify Qdrant and database connections with the provided credentials
"""

import asyncio
import os
from qdrant_client import QdrantClient
import asyncpg
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

async def test_qdrant_connection():
    """Test Qdrant connection"""
    print("Testing Qdrant connection...")

    try:
        qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
            timeout=10
        )

        # Test connection by getting collections
        collections = qdrant_client.get_collections()
        print(f"SUCCESS: Qdrant connection successful!")
        print(f"Available collections: {[col.name for col in collections.collections]}")

        # Check if our collection exists
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_book")
        collection_exists = any(col.name == collection_name for col in collections.collections)

        if collection_exists:
            print(f"SUCCESS: Collection '{collection_name}' exists")
        else:
            print(f"INFO: Collection '{collection_name}' does not exist (will be created automatically when needed)")

        return True

    except Exception as e:
        print(f"ERROR: Qdrant connection failed: {e}")
        return False

async def test_database_connection():
    """Test database connection"""
    print("\nTesting database connection...")

    try:
        db_url = os.getenv("DATABASE_URL")
        if not db_url:
            print("❌ DATABASE_URL not set in environment")
            return False

        conn = await asyncpg.connect(dsn=db_url)
        version = await conn.fetchval("SELECT version()")
        await conn.close()

        print(f"SUCCESS: Database connection successful!")
        print(f"PostgreSQL version: {version.split(',')[0] if version else 'Unknown'}")

        return True

    except Exception as e:
        print(f"ERROR: Database connection failed: {e}")
        return False

async def main():
    print("Starting connection tests...\n")

    qdrant_ok = await test_qdrant_connection()
    db_ok = await test_database_connection()

    print(f"\nSummary:")
    print(f"- Qdrant connection: {'SUCCESS' if qdrant_ok else 'FAILED'}")
    print(f"- Database connection: {'SUCCESS' if db_ok else 'FAILED'}")

    if qdrant_ok and db_ok:
        print("\nSUCCESS: All connections are working! You can now run the RAG system.")
    else:
        print("\nWARNING: Some connections failed. Please check your configuration.")

if __name__ == "__main__":
    asyncio.run(main())