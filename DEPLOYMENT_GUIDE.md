# Deployment Guide: RAG Chatbot for Physical AI & Humanoid Robotics

## Overview
This guide explains how to properly deploy the RAG (Retrieval-Augmented Generation) chatbot system, which consists of:
- **Frontend**: Static website hosted on GitHub Pages
- **Backend**: API server that must be hosted separately
- **Vector Database**: Qdrant Cloud (already configured)
- **LLM Service**: OpenRouter API (already configured)

## Architecture
```
Internet User
    ↓ (HTTPS)
GitHub Pages (Frontend)
    ↓ (CORS-enabled API calls)
Your Backend Server (API + Services)
    ↓ (Database connections)
Qdrant Cloud (Vector DB) ←→ OpenRouter API (LLM)
```

## Why GitHub Pages Alone Won't Work

GitHub Pages is a **static hosting service** that can only serve:
- HTML, CSS, JavaScript files
- Static assets (images, documents)
- Client-side code that runs in the browser

GitHub Pages **cannot** run:
- Python/Node.js backend servers
- Database services
- Real-time processing services

Your RAG system requires a backend server to:
- Process user queries
- Connect to Qdrant vector database
- Call OpenRouter API for LLM responses
- Handle document retrieval and generation

## Solution: Two-Part Deployment

### Part 1: Frontend (GitHub Pages)
The Docusaurus website will be deployed to GitHub Pages as normal.

### Part 2: Backend (Separate Hosting Service)
The FastAPI backend server must be deployed to a platform that supports Python applications:

## Backend Hosting Options

### Option A: Railway (Recommended for beginners)
1. Create account at [railway.app](https://railway.app)
2. Connect your GitHub repository
3. Railway will automatically deploy your backend
4. Update frontend to use the new backend URL

### Option B: Render
1. Create account at [render.com](https://render.com)
2. Create a new Web Service
3. Connect to your GitHub repository
4. Set build command: `pip install -r requirements.txt`
5. Set start command: `uvicorn main:app --host 0.0.0.0 --port $PORT`

### Option C: Heroku
1. Create account at [heroku.com](https://heroku.com)
2. Use Heroku CLI to deploy
3. Add environment variables for your services

### Option D: Self-hosting
Deploy to any VPS or cloud server that supports Python

## Configuration Changes Needed

### 1. Update Backend Environment Variables
Your backend needs these environment variables on the hosting platform:
```
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=physical_ai_book
DATABASE_URL=your_neon_database_url
OPENROUTER_API_KEY=your_openrouter_api_key
LLM_MODEL=mistralai/mistral-7b-instruct
```

### 2. Update CORS Settings
In `backend/main.py`, add your GitHub Pages URL to CORS origins:
```python
allow_origins=[
    "https://yourusername.github.io",  # Add this for GitHub Pages
    "http://localhost:3000",
    "http://localhost:3001",
    # ... other local ports
],
```

### 3. Update Frontend Backend URL
After deploying your backend, update the frontend to use the production backend URL:

**Option 1: Environment Variable in Build**
Set environment variable when building the frontend:
```bash
BACKEND_URL=https://your-backend-url.onrender.com npm run build
```

**Option 2: Hardcode in Production**
Update `website/src/theme/LayoutEnhancer.tsx`:
```jsx
<ChatWidget backendUrl="https://your-backend-url.onrender.com" />
```

## Step-by-Step Deployment Process

### Step 1: Deploy Backend First
1. Choose a backend hosting platform (Railway/Render recommended)
2. Deploy the backend server with all environment variables
3. Test the backend API: `https://your-backend-url/chat`
4. Ensure it responds to test requests

### Step 2: Update Frontend Configuration
1. Update CORS settings in backend to include your GitHub Pages URL
2. Update frontend to point to your backend URL
3. Test locally with the production backend

### Step 3: Deploy Frontend to GitHub Pages
1. Build the frontend with correct backend URL
2. Push to GitHub with GitHub Pages enabled
3. Verify the chatbot connects to your backend

## Testing the Complete Setup

After both deployments:
1. Visit your GitHub Pages site
2. Open browser developer tools (F12) → Network tab
3. Use the chatbot and watch for API calls to your backend
4. Verify responses come through correctly
5. Test the highlight feature

## Troubleshooting Common Issues

### Issue: "CORS error" or "Failed to fetch"
- **Cause**: Backend doesn't allow requests from your GitHub Pages URL
- **Solution**: Add your GitHub Pages URL to `allow_origins` in `main.py`

### Issue: "Connection refused" or "Backend not responding"
- **Cause**: Frontend is pointing to wrong backend URL
- **Solution**: Update backend URL in frontend configuration

### Issue: "API key error" or "Authentication failed"
- **Cause**: Environment variables not set on backend hosting
- **Solution**: Verify all API keys are set as environment variables

### Issue: "No context found" for all queries
- **Cause**: Backend can't connect to Qdrant Cloud
- **Solution**: Verify QDRANT_URL and QDRANT_API_KEY are correct

## Sample Production Configuration

### backend/.env (for hosting platform):
```
QDRANT_URL=https://your-cluster.region.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=physical_ai_book
DATABASE_URL=postgresql://user:pass@ep-xxx.eu-west-2.aws.neon.tech/dbname
OPENROUTER_API_KEY=your_openrouter_api_key
LLM_MODEL=mistralai/mistral-7b-instruct
API_HOST=0.0.0.0
API_PORT=8000
```

### Frontend URL in LayoutEnhancer.tsx:
```jsx
<ChatWidget backendUrl="https://your-backend-production-url.railway.app" />
```

## Cost Considerations

- **GitHub Pages**: Free
- **Backend Hosting**: $5-15/month depending on usage
- **Qdrant Cloud**: Free tier available, then ~$10-30/month
- **OpenRouter API**: Pay-per-use, typically $1-10/month for moderate usage
- **Total**: $7-65/month depending on usage and chosen platforms

## Recommended Setup for Minimal Cost
- GitHub Pages: Free hosting for frontend
- Railway: Free tier for backend (with occasional payment for extended use)
- Qdrant Cloud: Free tier
- OpenRouter: Pay-per-use
- **Total**: $0-15/month depending on usage

This architecture allows you to have a professional RAG chatbot system with static frontend hosting while maintaining all the powerful backend functionality.