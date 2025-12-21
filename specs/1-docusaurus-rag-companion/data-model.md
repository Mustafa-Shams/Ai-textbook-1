# Data Model: Docusaurus RAG Companion

**Feature**: 1-docusaurus-rag-companion
**Created**: 2025-12-21

## Database Schema (Neon Postgres)

### chat_sessions
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, NOT NULL, DEFAULT gen_random_uuid() | Unique identifier for the chat session |
| created_at | TIMESTAMP | NOT NULL, DEFAULT CURRENT_TIMESTAMP | When the session was created |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT CURRENT_TIMESTAMP | When the session was last updated |
| title | VARCHAR(255) | NOT NULL | Auto-generated title based on first query |

### chat_messages
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, NOT NULL, DEFAULT gen_random_uuid() | Unique identifier for the message |
| session_id | UUID | NOT NULL, FOREIGN KEY (chat_sessions.id) | Reference to parent chat session |
| role | VARCHAR(20) | NOT NULL, CHECK (role IN ('user', 'assistant')) | Message sender (user or assistant) |
| content | TEXT | NOT NULL | The actual message content |
| timestamp | TIMESTAMP | NOT NULL, DEFAULT CURRENT_TIMESTAMP | When the message was created |
| context_used | JSONB | NULL | Optional context used for the response (selected text, retrieved chunks) |

## Vector Database Schema (Qdrant)

### Collection: physical_ai_book

#### Vector Configuration
- **Size**: 1536 (dimension of embedding vectors)
- **Distance**: Cosine similarity
- **HNSW config**: Enabled for fast approximate search

#### Payload Structure
```json
{
  "content": "string - The text content of the chunk",
  "source_path": "string - File path in the docs directory",
  "heading_hierarchy": "string[] - Array of headings above this content",
  "chunk_index": "integer - Position of this chunk in the document",
  "metadata": {
    "word_count": "integer - Number of words in chunk",
    "language": "string - Programming language if code chunk",
    "level": "integer - Heading level if applicable"
  }
}
```

## Entity Relationships

### ChatSession
- **Description**: Represents a conversation between user and AI assistant
- **Fields**:
  - id: UUID
  - created_at: timestamp
  - updated_at: timestamp
  - title: string
- **Relationships**:
  - One-to-Many: Has many ChatMessage entities

### ChatMessage
- **Description**: Represents a single message in a conversation
- **Fields**:
  - id: UUID
  - session_id: UUID (foreign key)
  - role: enum (user|assistant)
  - content: text
  - timestamp: timestamp
  - context_used: JSONB
- **Relationships**:
  - Many-to-One: Belongs to ChatSession

### DocumentChunk (Vector DB)
- **Description**: Represents a chunk of document content with embedding
- **Fields**:
  - id: UUID (Qdrant point ID)
  - vector: float[1536] (embedding vector)
  - payload: object (as defined above)
- **Relationships**: None (standalone in vector DB)

## Validation Rules

### ChatSession
- Title must be 3-255 characters
- Cannot have more than 1000 messages per session
- Session must be updated when new message is added

### ChatMessage
- Content must not be empty
- Role must be either 'user' or 'assistant'
- Session must exist before creating message
- Context_used must be valid JSON if present

### DocumentChunk
- Content must be 50-2000 characters (for optimal RAG performance)
- Source path must exist in the documentation directory
- Embedding must be generated successfully before storage

## State Transitions

### ChatSession
- **Active**: When session is created and receiving messages
- **Inactive**: When session hasn't been updated for 30 days
- **Archived**: When session is cleaned up based on retention policy

### ChatMessage
- **Pending**: When message is being processed by LLM
- **Complete**: When response is generated and stored
- **Error**: When processing fails (retries may be attempted)

## Indexes

### Database Indexes
- chat_sessions.updated_at (for retrieving recent sessions)
- chat_messages.session_id (for querying messages by session)
- chat_messages.timestamp (for ordering messages)
- chat_messages.session_id, chat_messages.timestamp (composite for session queries)

### Vector Database Indexes
- Default HNSW index on vector field (for semantic search)
- Payload index on source_path (for filtering by document)
- Payload index on heading_hierarchy (for context-aware search)