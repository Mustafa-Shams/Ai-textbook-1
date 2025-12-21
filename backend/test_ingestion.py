#!/usr/bin/env python3
"""
Test script to verify the ingestion process works with your Qdrant Cloud instance
"""

import asyncio
import os
from pathlib import Path

# Add the backend directory to the path so we can import our modules
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import using absolute imports
from services.embedding_service import EmbeddingService
from services.rag_service import RAGService
from config.settings import settings
from ingest import DocumentIngestor

async def test_ingestion():
    """Test the ingestion process with a sample document"""
    print("Testing ingestion process...")

    try:
        # Create a sample markdown content for testing
        sample_content = """
# Introduction to Physical AI

Physical AI is an interdisciplinary field combining artificial intelligence with physical systems.
This includes robotics, embodied intelligence, and systems that interact with the physical world.

## Key Concepts

- Embodied Intelligence: AI systems that exist in and interact with physical environments
- Sensorimotor Learning: Learning through interaction with the environment
- Multi-modal Perception: Processing different types of sensory input

## ROS2 Fundamentals

Robot Operating System 2 (ROS2) provides frameworks for robotics development:
- Nodes and topics for communication
- Services and actions for interaction
- Packages and workspaces for organization

## Humanoid Robotics

Humanoid robots are designed to resemble and interact with humans:
- Bipedal locomotion
- Human-like manipulation
- Social interaction capabilities
        """

        # Create a temporary test file
        test_file = Path("test_doc.md")
        with open(test_file, 'w', encoding='utf-8') as f:
            f.write(sample_content)

        # Create ingestor
        ingestor = DocumentIngestor()

        # Process the test file
        chunks_count = await ingestor.process_file(test_file, Path("."))

        print(f"SUCCESS: Processed {chunks_count} chunks from test document")

        # Clean up
        if test_file.exists():
            test_file.unlink()

        return True

    except Exception as e:
        print(f"ERROR: Ingestion test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_ingestion())
    if success:
        print("\nSUCCESS: Ingestion process is working correctly!")
    else:
        print("\nERROR: Ingestion process failed!")