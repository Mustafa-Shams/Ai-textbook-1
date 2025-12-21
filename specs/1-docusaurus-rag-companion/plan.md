# Implementation Plan: Docusaurus RAG Companion

**Feature**: 1-docusaurus-rag-companion
**Created**: 2025-12-21
**Status**: Draft
**Spec**: specs/1-docusaurus-rag-companion/spec.md

## Technical Context

### Architecture Overview
- **Frontend**: React component integrated with Docusaurus site
- **Backend**: FastAPI server handling RAG operations
- **Embeddings**: Local Qwen embedding model (as clarified)
- **Vector DB**: Qdrant Cloud for semantic search
- **Relational DB**: Neon Postgres for chat history
- **LLM**: OpenRouter API for responses
- **Authentication**: Public access with rate limiting

### Technology Stack
- **Backend**: Python 3.11+, FastAPI, SQLAlchemy/AsyncPG
- **Frontend**: React, TypeScript, Docusaurus integration
- **Vector DB**: Qdrant (Cloud)
- **Relational DB**: Neon Postgres
- **Embeddings**: Local Qwen model
- **LLM**: OpenRouter API
- **Packaging**: Poetry

### Dependencies & Integration Points
- **Docusaurus Integration**: Component injection into existing site
- **Markdown Processing**: Parsing of existing `/docs` folder
- **External APIs**: OpenRouter, Qdrant Cloud
- **Database**: Neon Postgres connection

### Unknowns
- Local Qwen embedding model setup specifics
- Qdrant Cloud connection details
- OpenRouter API configuration
- Rate limiting implementation specifics

## Constitution Check

### Alignment with Project Principles
- ✅ **Maintainability**: Using established patterns (FastAPI, React)
- ✅ **Performance**: Caching and efficient vector search
- ✅ **Security**: Rate limiting and input validation
- ✅ **User Experience**: Non-intrusive tooltip interaction
- ✅ **Scalability**: Cloud-based vector DB and Postgres

### Potential Violations
- **Data Privacy**: Need to ensure no user data is stored unnecessarily
- **API Limits**: Rate limiting to prevent abuse of external services

## Gates

### Pre-Implementation Requirements
- [ ] Local Qwen embedding model installed and tested
- [ ] Qdrant Cloud account and collection created
- [ ] Neon Postgres database provisioned
- [ ] OpenRouter API access configured
- [ ] Development environment with Poetry set up

### Success Criteria for Each Phase
- Phase 1: Backend infrastructure connects to all services
- Phase 2: Ingestion pipeline processes docs successfully
- Phase 3: API endpoints return valid RAG responses
- Phase 4: Frontend component integrates with Docusaurus

## Phase 0: Research & Resolution

### research.md

#### Decision: Local Qwen Embedding Model Setup
- **Rationale**: As clarified in spec, using local Qwen model for embeddings
- **Implementation**: Use transformers library with Qwen embedding model
- **Requirements**: GPU support may be needed for performance

#### Decision: Qdrant Cloud Configuration
- **Rationale**: Cloud-based vector database for semantic search
- **Implementation**: Use qdrant-client library to connect to cloud instance
- **Requirements**: API key and cluster endpoint from Qdrant Cloud

#### Decision: Rate Limiting Strategy
- **Rationale**: Public access requires protection against abuse
- **Implementation**: Use slowapi with in-memory or Redis-based rate limiting
- **Default**: 10 requests per minute per IP

#### Decision: Markdown Processing Strategy
- **Rationale**: Need to parse Docusaurus docs structure
- **Implementation**: Use markdown libraries to extract content while preserving hierarchy
- **Considerations**: Handle code blocks, headers, and nested content properly

## Phase 1: Design & Contracts

### data-model.md

#### Database Schema (Neon Postgres)

**chat_sessions**
- id: UUID (primary key)
- created_at: timestamp
- updated_at: timestamp
- title: string (auto-generated from first query)

**chat_messages**
- id: UUID (primary key)
- session_id: UUID (foreign key to chat_sessions)
- role: enum (user|assistant)
- content: text
- timestamp: timestamp
- context_used: JSON (optional, for selected text context)

#### Vector Database Schema (Qdrant)

**Collection: physical_ai_book**
- id: UUID
- vector: float array (embedding dimensions)
- payload: {
  - content: string (chunked text),
  - source_path: string (file path),
  - heading_hierarchy: string[] (headers above content),
  - metadata: JSON (additional info)
}

### API Contracts

#### OpenAPI Contract

**POST /chat**
- Request: {message: string, session_id?: string, selected_context?: string}
- Response: {response: string, session_id: string, sources: string[]}
- Rate Limit: 10 requests/minute per IP

**GET /sessions/{session_id}**
- Response: {messages: ChatMessage[], title: string}

**GET /sessions**
- Response: {sessions: {id: string, title: string, updated_at: string}[]}

### quickstart.md

#### Development Setup

1. **Prerequisites**
   ```bash
   # Python 3.11+
   # Poetry installed
   # Node.js for Docusaurus
   ```

2. **Backend Setup**
   ```bash
   cd backend
   poetry install
   poetry shell
   # Set environment variables (see .env.example)
   python -m backend.main
   ```

3. **Frontend Integration**
   ```bash
   # In website directory
   npm install # or yarn
   # Add ChatWidget to Docusaurus layout
   npm run start
   ```

### Agent Context Update

#### New Technologies to Add
- FastAPI: Modern Python web framework
- Qdrant: Vector database client
- Transformers: ML model library for Qwen embeddings
- Slowapi: Rate limiting for FastAPI
- Docusaurus integration: React component injection

## Phase 2: Implementation Phases

### Phase 1: Backend Infrastructure
- Setup Poetry environment
- Initialize Neon Postgres connection
- Initialize Qdrant Client
- Setup OpenRouter client
- Implement basic API structure

### Phase 2: RAG Pipeline & Ingestion
- Create ingestion script for Docusaurus docs
- Implement Markdown-aware chunking
- Generate embeddings with local Qwen model
- Store vectors in Qdrant

### Phase 3: API Endpoint Development
- Implement main /chat endpoint with RAG logic
- Add session management endpoints
- Implement rate limiting
- Add error handling and validation

### Phase 4: Frontend Integration
- Create ChatWidget React component
- Implement text selection hook
- Integrate with Docusaurus layout
- Add tooltip interaction for selected text

## Risk Analysis

### Technical Risks
- **Qwen Model Performance**: Local model may be slow without GPU
- **Vector DB Costs**: Qdrant Cloud usage may be expensive
- **API Rate Limits**: External services may have request limits

### Mitigation Strategies
- **Caching**: Cache frequent queries to reduce API calls
- **Optimization**: Optimize chunking and embedding processes
- **Monitoring**: Track API usage and costs

## Success Metrics

### Technical Metrics
- Response time under 5 seconds for queries
- 95% uptime during peak hours
- Support for 100 concurrent users

### User Experience Metrics
- 90% of queries return relevant results
- Tooltip interaction appears within 200ms of selection
- Session restoration works 99% of the time

## Deployment Strategy

### Staging Environment
1. Deploy backend to cloud service (e.g., Railway, Heroku)
2. Connect to staging databases
3. Test with subset of documentation

### Production Deployment
1. Deploy backend to production cloud service
2. Connect to production databases
3. Integrate frontend component into Docusaurus build
4. Monitor performance and usage

## Operational Readiness

### Monitoring
- API response times and error rates
- Database connection health
- Vector DB query performance
- Rate limiting effectiveness

### Maintenance
- Regular model updates for embeddings
- Database cleanup for old sessions
- Documentation sync process