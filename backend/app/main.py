from fastapi import FastAPI, HTTPException, Body
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import os
from dotenv import load_dotenv
from .services.rag_service import RAGService
from .services.ingestion import IngestionService

load_dotenv()

app = FastAPI()

# Allow CORS for local development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Adjust in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

rag_service = RAGService()
ingestion_service = IngestionService()

class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    sources: List[str]

@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    try:
        response, sources = await rag_service.get_response(request.message, request.selected_text)
        return ChatResponse(response=response, sources=sources)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/ingest")
async def ingest_docs():
    try:
        count = await ingestion_service.ingest_docs()
        return {"message": f"Successfully ingested {count} documents."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
