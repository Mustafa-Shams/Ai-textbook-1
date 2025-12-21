#!/usr/bin/env python3
"""
Script to ingest documentation files and store them in the vector database.
Walks through the docs directory, splits markdown files using Recursive Character Splitter,
generates embeddings with Qwen, and uploads to Qdrant.
"""

import os
import asyncio
import argparse
import logging
from pathlib import Path
from typing import List, Dict
import re

# Import our services
from services.embedding_service import EmbeddingService
from services.rag_service import RAGService
from config.settings import settings

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DocumentIngestor:
    """
    Class to handle document ingestion from markdown files into vector database
    Uses Recursive Character Splitter approach for chunking
    """

    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.rag_service = RAGService(self.embedding_service)

    def read_markdown_file(self, file_path: Path) -> str:
        """Read content from a markdown file"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            return content
        except Exception as e:
            logger.error(f"Error reading file {file_path}: {e}")
            return ""

    def split_markdown_content_recursive(self, content: str, chunk_size: int = 1000, overlap: int = 200) -> List[Dict]:
        """
        Split markdown content using Recursive Character Splitter approach
        This preserves document structure while creating appropriately sized chunks
        """
        chunks = []

        # Define separators in order of preference (from most specific to least)
        separators = ["\n\n", "\n", " ", ""]

        # First, split by headers to maintain context
        header_split = re.split(r'(\n#{1,6}\s.*?\n|\n#{1,6}\s.*)', content)

        # Process each section separately to maintain context
        for i in range(0, len(header_split), 2):
            header = header_split[i-1] if i > 0 and i-1 < len(header_split) and header_split[i-1].startswith('\n') else ""
            text_content = header_split[i] if i < len(header_split) else ""

            # Combine header and content
            section = header + text_content if header else text_content

            # Now apply recursive splitting
            section_chunks = self._recursive_split(section, chunk_size, overlap, separators)
            for chunk in section_chunks:
                if chunk.strip():  # Only add non-empty chunks
                    chunks.append({
                        "content": chunk,
                        "metadata": {"chunk_size": len(chunk)}
                    })

        return chunks

    def _recursive_split(self, text: str, chunk_size: int, overlap: int, separators: List[str]) -> List[str]:
        """
        Recursively split text using different separators
        """
        # If text is smaller than chunk size, return as is
        if len(text) <= chunk_size:
            return [text]

        # Try each separator in order
        for sep in separators:
            if sep == "":
                # If we're down to character-level splitting, just split by chunk_size
                chunks = []
                for i in range(0, len(text), chunk_size - overlap):
                    end = min(i + chunk_size, len(text))
                    chunks.append(text[i:end])
                return chunks

            # Split by the current separator
            splits = text.split(sep) if sep != " " else re.split(r'\s+', text)

            # Check if any split is still too large
            too_large_found = False
            final_chunks = []
            current_chunk = ""

            for split in splits:
                if len(split) > chunk_size:
                    # This split is still too large, need to split further with next separator
                    too_large_found = True
                    # Add current chunk if it exists
                    if current_chunk.strip():
                        final_chunks.append(current_chunk.strip())
                        current_chunk = ""

                    # Recursively split this large piece
                    sub_splits = self._recursive_split(split, chunk_size, overlap, separators[separators.index(sep)+1:])
                    final_chunks.extend(sub_splits)
                else:
                    # Check if adding this split would exceed chunk size
                    test_chunk = current_chunk + sep + split if current_chunk else split
                    if len(test_chunk) <= chunk_size:
                        current_chunk = test_chunk
                    else:
                        # Current chunk is full, save it and start new one
                        if current_chunk.strip():
                            final_chunks.append(current_chunk.strip())

                        # If the current split is too large by itself, we need to split it further
                        if len(split) > chunk_size:
                            sub_splits = self._recursive_split(split, chunk_size, overlap, separators[separators.index(sep)+1:])
                            final_chunks.extend(sub_splits)
                        else:
                            current_chunk = split

            # Add the last chunk
            if current_chunk.strip():
                final_chunks.append(current_chunk.strip())

            # If no chunk was too large, we're done with this separator
            if not too_large_found:
                return final_chunks

        # Fallback: if all separators fail, just split by character
        chunks = []
        for i in range(0, len(text), chunk_size - overlap):
            end = min(i + chunk_size, len(text))
            chunks.append(text[i:end])
        return chunks

    def get_all_markdown_files(self, docs_dir: Path) -> List[Path]:
        """Get all markdown files from the docs directory"""
        markdown_files = []

        for file_path in docs_dir.rglob("*.md"):
            if not any(part.startswith('.') for part in file_path.parts):
                markdown_files.append(file_path)

        return markdown_files

    async def process_file(self, file_path: Path, docs_dir: Path) -> int:
        """Process a single markdown file and add to vector database"""
        try:
            logger.info(f"Processing file: {file_path}")

            # Read file content
            content = self.read_markdown_file(file_path)
            if not content:
                return 0

            # Calculate relative path for source tracking
            relative_path = file_path.relative_to(docs_dir)

            # Split content into chunks using recursive approach
            chunks = self.split_markdown_content_recursive(content)

            # Prepare chunks for database insertion
            db_chunks = []
            for i, chunk in enumerate(chunks):
                db_chunk = {
                    "content": chunk["content"],
                    "source_path": str(relative_path),
                    "metadata": {
                        **chunk["metadata"],
                        "chunk_index": i,
                        "total_chunks": len(chunks)
                    }
                }
                db_chunks.append(db_chunk)

            # Add chunks to vector database
            chunk_ids = await self.rag_service.batch_add_document_chunks(db_chunks)

            logger.info(f"Successfully processed {len(chunk_ids)} chunks from {file_path}")
            return len(chunk_ids)

        except Exception as e:
            logger.error(f"Error processing file {file_path}: {e}")
            raise

    async def ingest_directory(self, docs_dir: Path) -> int:
        """Ingest all markdown files from a directory"""
        logger.info(f"Starting ingestion from directory: {docs_dir}")

        # Get all markdown files
        markdown_files = self.get_all_markdown_files(docs_dir)
        logger.info(f"Found {len(markdown_files)} markdown files to process")

        total_chunks = 0

        # Process each file
        for file_path in markdown_files:
            try:
                chunks_count = await self.process_file(file_path, docs_dir)
                total_chunks += chunks_count
            except Exception as e:
                logger.error(f"Failed to process {file_path}: {e}")
                continue  # Continue with other files

        logger.info(f"Ingestion complete. Total chunks added: {total_chunks}")
        return total_chunks

async def main():
    parser = argparse.ArgumentParser(description="Ingest documentation files into vector database")
    parser.add_argument("--docs-path", type=Path, default=Path("../docs"),
                       help="Path to the docs directory (default: ../docs)")
    parser.add_argument("--chunk-size", type=int, default=1000,
                       help="Maximum chunk size in characters (default: 1000)")
    parser.add_argument("--overlap", type=int, default=200,
                       help="Overlap between chunks in characters (default: 200)")

    args = parser.parse_args()

    # Verify docs directory exists
    if not args.docs_path.exists():
        logger.error(f"Docs directory does not exist: {args.docs_path}")
        return 1

    if not args.docs_path.is_dir():
        logger.error(f"Path is not a directory: {args.docs_path}")
        return 1

    # Create ingestor and run
    ingestor = DocumentIngestor()
    total_chunks = await ingestor.ingest_directory(args.docs_path)

    logger.info(f"Successfully ingested {total_chunks} chunks into the vector database")
    return 0

if __name__ == "__main__":
    exit_code = asyncio.run(main())
    exit(exit_code)