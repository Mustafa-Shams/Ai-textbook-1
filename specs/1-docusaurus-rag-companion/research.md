# Research Document: Docusaurus RAG Companion

**Feature**: 1-docusaurus-rag-companion
**Created**: 2025-12-21

## Decision: Local Qwen Embedding Model Setup

**Research Task**: Determine optimal setup for local Qwen embedding model
**Status**: Resolved

**Decision**: Use the Qwen2-Embedding model from Alibaba's Qwen series via Hugging Face Transformers
**Rationale**:
- Direct compatibility with Python ecosystem
- Good performance for text embeddings
- Can run on CPU with reasonable performance
- No external API dependencies for embeddings

**Implementation**:
```python
from transformers import AutoTokenizer, AutoModel
import torch

model_name = "Alibaba-NLP/qwen2-embedding"
tokenizer = AutoTokenizer.from_pretrained(model_name)
model = AutoModel.from_pretrained(model_name)

def get_embedding(text):
    inputs = tokenizer(text, return_tensors="pt", padding=True, truncation=True)
    with torch.no_grad():
        outputs = model(**inputs)
    # Use the embedding from the last hidden state
    embedding = outputs.last_hidden_state.mean(dim=1).squeeze().numpy()
    return embedding
```

**Alternatives Considered**:
- Using Alibaba Cloud DashScope API: Would create external dependency
- Other embedding models (e.g., SentenceTransformers): Qwen is specifically requested
- Running via Docker: More complex than direct library usage

## Decision: Qdrant Cloud Configuration

**Research Task**: Determine optimal Qdrant Cloud setup
**Status**: Resolved

**Decision**: Use Qdrant Cloud with cosine similarity and 1536-dimensional vectors
**Rationale**:
- Qdrant is specifically requested in the feature description
- Cosine similarity works well for text embeddings
- 1536 matches typical embedding dimensions for modern models

**Implementation**:
- Sign up at cloud.qdrant.io
- Create collection with cosine distance
- Use qdrant-client Python library
- Store metadata with source information

**Configuration**:
```python
from qdrant_client import QdrantClient
from qdrant_client.http import models

client = QdrantClient(
    url="YOUR_CLUSTER_URL",
    api_key="YOUR_API_KEY"
)

# Create collection
client.create_collection(
    collection_name="physical_ai_book",
    vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE)
)
```

## Decision: Rate Limiting Strategy

**Research Task**: Determine optimal rate limiting approach for public API
**Status**: Resolved

**Decision**: Use slowapi with in-memory storage for development, Redis for production
**Rationale**:
- slowapi is designed specifically for FastAPI applications
- Easy to implement and configure
- Supports multiple storage backends
- Good documentation and community support

**Implementation**:
```python
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

limiter = Limiter(key_func=get_remote_address)
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

@app.post("/chat")
@limiter.limit("10/minute")
async def chat_endpoint(request: Request, ...):
    # endpoint implementation
```

**Default**: 10 requests per minute per IP address

## Decision: Markdown Processing Strategy

**Research Task**: Determine best approach for processing Docusaurus markdown files
**Status**: Resolved

**Decision**: Use markdown and mistune libraries to parse markdown while preserving structure
**Rationale**:
- Need to maintain document hierarchy and context
- Must handle Docusaurus-specific features (headers, code blocks)
- Need to extract content without losing semantic meaning

**Implementation**:
```python
import markdown
from pathlib import Path
import re

def extract_markdown_sections(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Split by headers to maintain context
    sections = re.split(r'^##\s+(.+)$', content, flags=re.MULTILINE)
    # Process sections to maintain hierarchy
    # Return list of content chunks with metadata
```

**Considerations**:
- Handle nested headers properly
- Preserve code blocks and special formatting
- Maintain file path information for citations