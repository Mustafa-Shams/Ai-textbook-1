#!/usr/bin/env python3
"""
Test script to verify the internal knowledge base is working
"""
import asyncio
from backend.services.internal_knowledge_service import internal_kb_service

def test_internal_knowledge():
    print("Testing internal knowledge base...")
    print(f"Total chunks loaded: {len(internal_kb_service.chunks)}")

    # Show some statistics
    modules = set(chunk.module for chunk in internal_kb_service.chunks)
    print(f"Modules found: {list(modules)}")

    sources = set(chunk.source for chunk in internal_kb_service.chunks)
    print(f"Sources: {list(sources)[:10]}...")  # Show first 10 sources

    # Test search functionality
    print("\nTesting search functionality...")
    query = "What is ROS 2?"
    results = internal_kb_service.search_similar_content(query, limit=3)

    print(f"Query: '{query}'")
    print(f"Found {len(results)} results:")

    for i, result in enumerate(results):
        print(f"  Result {i+1}:")
        print(f"    Source: {result['source']}")
        print(f"    Score: {result['score']:.3f}")
        print(f"    Preview: {result['content'][:100]}...")
        print()

    # Test another query
    query2 = "Physical AI"
    results2 = internal_kb_service.search_similar_content(query2, limit=2)

    print(f"Query: '{query2}'")
    print(f"Found {len(results2)} results:")

    for i, result in enumerate(results2):
        print(f"  Result {i+1}:")
        print(f"    Source: {result['source']}")
        print(f"    Score: {result['score']:.3f}")
        print(f"    Preview: {result['content'][:100]}...")
        print()

if __name__ == "__main__":
    test_internal_knowledge()