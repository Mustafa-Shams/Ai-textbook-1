#!/usr/bin/env python3
"""
Test script to verify the RAG functionality with the newly ingested content
"""
import asyncio
from services.embedding_service import EmbeddingService
from services.rag_service import RAGService

async def test_rag_search():
    """Test that RAG search can find content related to 'module 2'"""
    print("Testing RAG search functionality...")

    embedding_service = EmbeddingService()
    rag_service = RAGService(embedding_service)

    # Test search for module 2 content
    query = "What is module 2 about?"
    results = await rag_service.search_similar_content(query)

    print(f"Query: {query}")
    print(f"Found {len(results)} results:")

    for i, result in enumerate(results[:3]):  # Show first 3 results
        print(f"  Result {i+1}:")
        print(f"    Source: {result['source']}")
        print(f"    Score: {result['score']:.3f}")
        print(f"    Preview: {result['content'][:100]}...")
        print()

    # Test another query about specific content
    query2 = "sensors in robotics"
    results2 = await rag_service.search_similar_content(query2)

    print(f"Query: {query2}")
    print(f"Found {len(results2)} results:")

    for i, result in enumerate(results2[:2]):  # Show first 2 results
        print(f"  Result {i+1}:")
        print(f"    Source: {result['source']}")
        print(f"    Score: {result['score']:.3f}")
        print(f"    Preview: {result['content'][:100]}...")
        print()

if __name__ == "__main__":
    asyncio.run(test_rag_search())