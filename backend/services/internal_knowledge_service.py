"""
Internal Knowledge Base Service for the Physical AI & Humanoid Robotics textbook
This service loads all documentation content into memory to provide answers without external connections.
"""
import os
import glob
from pathlib import Path
import logging
import re
from typing import List, Dict, Tuple
from dataclasses import dataclass
import difflib
from collections import Counter
import math

logger = logging.getLogger(__name__)

@dataclass
class KnowledgeChunk:
    """Represents a chunk of knowledge from documentation"""
    id: str
    content: str
    source: str
    title: str
    module: str

class InternalKnowledgeService:
    """Service that provides access to internal documentation knowledge base"""

    def __init__(self):
        self.chunks: List[KnowledgeChunk] = []
        self.module_map: Dict[str, str] = {}
        self._load_documentation()

    def _load_documentation(self):
        """Load all documentation files from website/docs into memory"""
        logger.info("Loading documentation into internal knowledge base...")

        # Get the path to website/docs relative to the backend directory
        docs_path = Path(__file__).parent.parent.parent / "website" / "docs"

        if not docs_path.exists():
            logger.error(f"Documentation path does not exist: {docs_path}")
            return

        # Find all markdown files
        md_files = list(docs_path.rglob("*.md"))
        logger.info(f"Found {len(md_files)} documentation files to load")

        chunk_id = 0

        for md_file in md_files:
            try:
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Extract title from the file (first H1 or from frontmatter)
                title = self._extract_title(content)
                source = str(md_file.relative_to(docs_path.parent))

                # Determine module from path
                module = self._extract_module_from_path(str(md_file))

                # Split content into chunks (to simulate the RAG approach)
                content_chunks = self._split_content(content, chunk_size=500)

                for i, chunk_content in enumerate(content_chunks):
                    chunk = KnowledgeChunk(
                        id=f"{source}_chunk_{i}",
                        content=chunk_content,
                        source=source,
                        title=f"{title} - Chunk {i+1}" if len(content_chunks) > 1 else title,
                        module=module
                    )
                    self.chunks.append(chunk)

                logger.info(f"Loaded {len(content_chunks)} chunks from {source}")

            except Exception as e:
                logger.error(f"Error loading documentation file {md_file}: {e}")

        logger.info(f"Loaded {len(self.chunks)} knowledge chunks from documentation")

    def _extract_title(self, content: str) -> str:
        """Extract title from markdown content"""
        # First try to extract from frontmatter
        lines = content.split('\n')
        if len(lines) > 0 and lines[0].strip() == '---':
            # Look for title in frontmatter
            for i in range(1, len(lines)):
                if lines[i].strip() == '---':
                    break
                if lines[i].startswith('title:'):
                    title = lines[i][6:].strip().strip('"').strip("'")
                    return title

        # If no title in frontmatter, look for first H1
        for line in lines:
            if line.startswith('# '):
                return line[2:].strip()

        # If no H1, return filename
        return "Untitled Document"

    def _extract_module_from_path(self, file_path: str) -> str:
        """Extract module name from file path"""
        path_parts = Path(file_path).parts
        for part in path_parts:
            if part.startswith('module-'):
                return part
        return "general"

    def _split_content(self, content: str, chunk_size: int = 500) -> List[str]:
        """Split content into chunks of specified size"""
        # Remove frontmatter if present
        lines = content.split('\n')
        if len(lines) > 0 and lines[0].strip() == '---':
            # Skip frontmatter
            end_frontmatter = -1
            for i, line in enumerate(lines[1:], 1):
                if line.strip() == '---':
                    end_frontmatter = i
                    break
            if end_frontmatter > 0:
                content = '\n'.join(lines[end_frontmatter + 1:])

        # Split content into chunks
        sentences = re.split(r'(?<=[.!?])\s+', content)
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            if len(current_chunk) + len(sentence) <= chunk_size:
                current_chunk += sentence + " "
            else:
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())
                current_chunk = sentence + " "

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        # If no chunks were created (content too short), return the whole content
        if not chunks:
            chunks = [content[:chunk_size]]

        return chunks

    def search_similar_content(self, query: str, limit: int = 5) -> List[Dict]:
        """Search for content similar to the query using keyword matching"""
        if not query:
            return []

        query_lower = query.lower()
        query_tokens = self._tokenize_text(query_lower)

        results = []

        for chunk in self.chunks:
            # Calculate a score based on keyword matches and string similarity
            score = self._calculate_content_score(query_lower, query_tokens, chunk.content.lower())

            if score > 0:  # Include results that have some relevance
                results.append({
                    'content': chunk.content,
                    'source': chunk.source,
                    'score': score,
                    'title': chunk.title
                })

        # Sort by score (descending)
        results.sort(key=lambda x: x['score'], reverse=True)

        # Return top results
        return results[:limit]

    def _calculate_content_score(self, query_lower: str, query_tokens: List[str], content_lower: str) -> float:
        """Calculate a relevance score for content based on query"""
        if not query_tokens:
            return 0.0

        # Calculate keyword match score
        keyword_matches = 0
        for token in query_tokens:
            if token in content_lower:
                keyword_matches += 1

        keyword_score = keyword_matches / len(query_tokens) if query_tokens else 0.0

        # Calculate string similarity for broader context matching
        if len(content_lower) > 0:
            # Use a more lenient similarity calculation
            similarity = 0
            for query_token in query_tokens[:3]:  # Limit to first 3 tokens to avoid performance issues
                if len(query_token) > 2:  # Only consider meaningful words
                    # Count occurrences of query token in content
                    token_count = content_lower.count(query_token)
                    if token_count > 0:
                        similarity += token_count * len(query_token) / len(content_lower)

            # Combine keyword score with similarity score
            total_score = keyword_score * 0.7 + min(similarity, 1.0) * 0.3
        else:
            total_score = keyword_score

        return total_score

    def _tokenize_text(self, text: str) -> List[str]:
        """Tokenize text into words, removing punctuation and common stop words"""
        # Split text into tokens, removing punctuation
        tokens = re.findall(r'\b\w+\b', text)

        # Remove common stop words to focus on meaningful terms
        stop_words = {
            'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for',
            'of', 'with', 'by', 'is', 'are', 'was', 'were', 'be', 'been', 'being',
            'have', 'has', 'had', 'do', 'does', 'did', 'will', 'would', 'could',
            'should', 'may', 'might', 'must', 'can', 'this', 'that', 'these', 'those'
        }

        return [token for token in tokens if token not in stop_words and len(token) > 2]

    def _calculate_similarity(self, query_tokens: List[str], content: str) -> float:
        """Calculate similarity between query and content using TF-IDF approach"""
        content_tokens = self._tokenize_text(content)

        if not content_tokens:
            return 0.0

        # Calculate term frequencies in content
        content_freq = Counter(content_tokens)

        # Calculate query term frequencies
        query_freq = Counter(query_tokens)

        # Find common terms
        common_terms = set(query_freq.keys()) & set(content_freq.keys())

        if not common_terms:
            return 0.0

        # Calculate similarity based on common terms and their frequencies
        similarity = 0.0
        for term in common_terms:
            # Weight by frequency in both query and content
            query_weight = query_freq[term] / len(query_tokens)
            content_weight = content_freq[term] / len(content_tokens)
            similarity += query_weight * content_weight

        # Normalize by total unique terms to avoid bias toward longer content
        total_terms = len(set(query_tokens + content_tokens))
        if total_terms > 0:
            similarity = similarity / math.sqrt(total_terms)

        return min(similarity, 1.0)  # Cap at 1.0

    def get_all_content(self) -> str:
        """Get all documentation content as a single string"""
        return " ".join([chunk.content for chunk in self.chunks])

    def get_module_content(self, module_name: str) -> str:
        """Get content for a specific module"""
        module_chunks = [chunk.content for chunk in self.chunks if chunk.module == module_name]
        return " ".join(module_chunks)

# Create a global instance
internal_kb_service = InternalKnowledgeService()