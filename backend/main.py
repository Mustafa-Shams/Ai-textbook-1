from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import os
import asyncio
import logging
import uuid
from datetime import datetime
from contextlib import asynccontextmanager
import asyncpg

# Import the services and models we'll need
from services.rag_service import RAGService
from services.lightweight_embedding_service import EmbeddingService
from services.llm_service import LLMService
from config.settings import settings

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global variables to hold service instances
rag_service = None
embedding_service = None
llm_service = None
db_pool = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize services on startup and clean up on shutdown"""
    global rag_service, embedding_service, llm_service, db_pool

    # Initialize services
    embedding_service = EmbeddingService()
    rag_service = RAGService(embedding_service)
    llm_service = LLMService()

    # Initialize database connection pool
    if settings.DATABASE_URL:
        db_pool = await asyncpg.create_pool(settings.DATABASE_URL)
        # Create chat_history table if it doesn't exist
        async with db_pool.acquire() as conn:
            await conn.execute('''
                CREATE TABLE IF NOT EXISTS chat_history (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    session_id UUID NOT NULL,
                    user_msg TEXT NOT NULL,
                    bot_response TEXT NOT NULL,
                    timestamp TIMESTAMP DEFAULT NOW()
                )
            ''')
    else:
        logger.warning("DATABASE_URL not set, chat history will not be saved")

    logger.info("Services initialized")

    yield  # This is where the application runs

    # Cleanup
    if db_pool:
        await db_pool.close()
    logger.info("Shutting down services")

# Create FastAPI app with lifespan
app = FastAPI(
    title="Docusaurus RAG Companion API",
    description="API for the Docusaurus RAG Companion feature",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware to allow requests from localhost (for development) and configured origins (for production)
cors_origins = [
    "http://localhost:3000", "http://127.0.0.1:3000",
    "http://localhost:3001", "http://127.0.0.1:3001",
    "http://localhost:3002", "http://127.0.0.1:3002",
    "http://localhost:3003", "http://127.0.0.1:3003",
    "http://localhost:3004", "http://127.0.0.1:3004",
    "https://mustafa-shams.github.io/Ai-textbook-1/",
    "https://ai-textbook-1-production.up.railway.app"  # Allow requests from the same Railway domain
]

# Add GitHub Pages URL from environment variable if available
if settings.GITHUB_PAGES_URL:
    cors_origins.append(settings.GITHUB_PAGES_URL)

app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models
class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    session_id: str
    sources: List[str] = []

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy"}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Main chat endpoint that handles user queries with RAG capabilities.

    If selected_text is provided, bypass Vector Search and inject selection directly into System Prompt.
    Otherwise, perform Qdrant Similarity Search using the user's query embedded with Qwen.
    Call OpenRouter API and save the interaction to Neon Postgres chat_history table.
    """
    session_id = request.session_id or str(uuid.uuid4())

    try:
        # Logic: If request.selected_text is present, bypass Vector Search and inject selection directly
        if request.selected_text:
            context = request.selected_text
            sources = ["selected_text"]
        else:
            # Logic: Perform Qdrant Similarity Search using the user's query embedded with Qwen
            search_results = await rag_service.search_similar_content(request.query)
            context = " ".join([result["content"] for result in search_results])
            sources = [result["source"] for result in search_results]

        # Generate response using LLM
        response = await llm_service.generate_response(
            query=request.query,
            context=context,
            session_id=session_id
        )

        # Save the interaction to Neon Postgres chat_history table
        if db_pool:
            try:
                async with db_pool.acquire() as conn:
                    await conn.execute('''
                        INSERT INTO chat_history (session_id, user_msg, bot_response)
                        VALUES ($1, $2, $3)
                    ''', session_id, request.query, response)
            except Exception as db_error:
                logger.error(f"Failed to save chat history: {db_error}")
                # Continue anyway, don't fail the response if DB save fails

        return ChatResponse(
            response=response,
            session_id=session_id,
            sources=sources
        )

    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=settings.API_HOST, port=settings.API_PORT)