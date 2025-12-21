# Physical AI & Humanoid Robotics RAG System

## Overview
This repository contains a comprehensive RAG (Retrieval-Augmented Generation) system for the Physical AI and Humanoid Robotics textbook. The system includes:

- **Frontend**: Docusaurus-based documentation website with floating AI chatbot
- **Backend**: FastAPI server with RAG capabilities
- **Vector Database**: Qdrant Cloud for document storage and retrieval
- **LLM**: OpenRouter API for responses

## Architecture
- **Frontend**: Static website hosted on GitHub Pages
- **Backend**: Python FastAPI server (must be hosted separately from GitHub Pages)
- **Database**: Qdrant Cloud vector database
- **LLM**: OpenRouter API (supports various models)

## Deployment Information
⚠️ **Important**: This system requires both frontend and backend components. GitHub Pages alone cannot host the backend API server. See [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md) for complete deployment instructions.

## Features
- Floating circular chatbot with modern UI design
- Text selection to "Ask AI" for instant explanations
- RAG-powered responses based on textbook content
- Professional, structured responses with proper formatting
- Support for complex technical queries about robotics

## Local Development
1. Backend: `cd backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000`
2. Frontend: `cd website && npm run start`

## Backend Requirements
The backend server needs:
- Qdrant Cloud credentials
- OpenRouter API key
- Proper CORS configuration for your domain

See `.env` file in backend directory for required environment variables.

For complete deployment instructions, please refer to [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md). 
