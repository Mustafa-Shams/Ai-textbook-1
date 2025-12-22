# Quickstart Guide: Docusaurus RAG Companion

**Feature**: 1-docusaurus-rag-companion
**Created**: 2025-12-21

## Development Setup

### Prerequisites

Before starting, ensure you have the following installed:

- **Python 3.11+**: Required for the backend FastAPI application
- **Poetry**: Dependency management for Python
- **Node.js 18+**: Required for Docusaurus frontend
- **Git**: Version control
- **Access to APIs**:
  - Qdrant Cloud account (for vector database)
  - OpenRouter API key (for LLM responses)
  - (Optional) Alibaba Cloud account for Qwen model access

### Backend Setup

1. **Clone and navigate to backend directory**
   ```bash
   cd backend
   ```

2. **Install dependencies with Poetry**
   ```bash
   poetry install
   ```

3. **Activate the virtual environment**
   ```bash
   poetry shell
   ```

4. **Set up environment variables**
   Create a `.env` file in the backend directory with the following:
   ```env
   # Qdrant Configuration
   QDRANT_URL=your-qdrant-cluster-url
   QDRANT_API_KEY=your-qdrant-api-key
   QDRANT_COLLECTION_NAME=physical_ai_book

   # Neon Postgres Configuration
   DATABASE_URL=postgresql://username:password@neon-host:5432/database-name

   # OpenRouter Configuration
   OPENROUTER_API_KEY=your-openrouter-api-key
   LLM_MODEL=qwen/qwen-2-72b-instruct  # or your preferred model

   # Qwen Embedding Model (local)
   QWEN_EMBEDDING_MODEL=Alibaba-NLP/qwen2-embedding

   # Rate Limiting
   RATE_LIMIT="10/minute"

   # Application
   API_HOST=0.0.0.0
   API_PORT=8000
   ```

5. **Run the backend server**
   ```bash
   python -m backend.main
   ```

### Frontend Integration

1. **Navigate to the Docusaurus website directory**
   ```bash
   cd website  # or wherever your Docusaurus site is located
   ```

2. **Install dependencies**
   ```bash
   npm install
   # or if using yarn
   yarn install
   ```

3. **Integrate the ChatWidget component**
   The ChatWidget should be added to your Docusaurus layout to appear on all pages:
   - Create `src/components/ChatWidget` directory
   - Add the ChatWidget React component
   - Modify `src/theme/Layout` to include the widget globally

4. **Start the Docusaurus development server**
   ```bash
   npm run start
   ```

### Initial Data Setup

1. **Run the ingestion script to populate the vector database**
   ```bash
   # From the backend directory with poetry shell active
   python -m backend.scripts.ingest_docs --docs-path /path/to/docusaurus/docs
   ```

2. **Verify data ingestion**
   - Check that documents have been chunked and stored in Qdrant
   - Verify that embeddings were generated successfully
   - Test semantic search functionality

### Testing the Setup

1. **API Health Check**
   - Visit `https://ai-textbook-1-production.up.railway.app/health` to verify the backend is running
   - Should return `{"status": "healthy"}`

2. **Test Chat Endpoint**
   ```bash
   curl -X POST https://ai-textbook-1-production.up.railway.app/chat \
     -H "Content-Type: application/json" \
     -d '{"message": "Hello, test message"}'
   ```

3. **Test Frontend Integration**
   - Open your Docusaurus site in a browser
   - Verify the chat widget appears
   - Test the tooltip functionality by selecting text

## Architecture Overview

### Backend Components

```
backend/
├── main.py                 # FastAPI application entry point
├── api/                    # API route definitions
│   ├── chat.py             # Chat and RAG endpoints
│   ├── sessions.py         # Session management
│   └── health.py           # Health check endpoint
├── services/               # Business logic
│   ├── rag.py              # RAG pipeline implementation
│   ├── embeddings.py       # Qwen embedding generation
│   ├── vector_db.py        # Qdrant interaction
│   └── llm.py              # OpenRouter integration
├── models/                 # Data models and schemas
│   ├── chat.py             # Chat-related Pydantic models
│   └── embeddings.py       # Embedding models
├── database/               # Database models and connections
│   ├── models.py           # SQLAlchemy models
│   └── session.py          # Database session management
├── scripts/                # Utility scripts
│   └── ingest_docs.py      # Documentation ingestion script
└── config/                 # Configuration settings
    └── settings.py         # Settings management
```

### Frontend Components

```
website/src/components/
└── ChatWidget/
    ├── ChatWidget.tsx      # Main chat interface
    ├── useTextSelection.ts # Text selection hook
    ├── Tooltip.tsx         # Tooltip for selected text
    └── MessageList.tsx     # Chat message display
```

## Environment Configuration

### Required Environment Variables

| Variable | Purpose | Example |
|----------|---------|---------|
| `QDRANT_URL` | Qdrant Cloud cluster URL | `https://your-cluster.us-east.qdrant.io` |
| `QDRANT_API_KEY` | Qdrant Cloud API key | `your-api-key` |
| `DATABASE_URL` | Neon Postgres connection string | `postgresql://user:pass@ep-...` |
| `OPENROUTER_API_KEY` | OpenRouter API key | `sk-or-...` |
| `QWEN_EMBEDDING_MODEL` | Local Qwen embedding model | `Alibaba-NLP/qwen2-embedding` |

### Optional Environment Variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `API_HOST` | Host for FastAPI server | `0.0.0.0` |
| `API_PORT` | Port for FastAPI server | `8000` |
| `RATE_LIMIT` | Rate limiting configuration | `10/minute` |
| `LOG_LEVEL` | Logging level | `INFO` |

## Common Tasks

### Running Tests
```bash
# Backend tests
poetry run pytest

# Frontend tests
cd website && npm run test
```

### Building for Production
```bash
# Backend (containerized)
docker build -t rag-companion .

# Frontend
cd website && npm run build
```

### Data Management
```bash
# Re-ingest all documentation
python -m backend.scripts.ingest_docs --docs-path /path/to/docs --force

# Clear vector database
python -m backend.scripts.clear_vectors
```