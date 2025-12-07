import os
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dotenv import load_dotenv
from openai import OpenAI
from qdrant_client import QdrantClient, models

# Load environment variables
load_dotenv()

# --- Configuration ---
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
OPENROUTER_MODEL = "openai/gpt-4o" # Default model, can be configured later

# Initialize clients
app = FastAPI()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    host=QDRANT_HOST,
    api_key=QDRANT_API_KEY,
)

# Initialize OpenRouter (OpenAI-compatible) client
# Using base_url to point to OpenRouter
openai_client = OpenAI(
    base_url="https://openrouter.ai/api/v1",
    api_key=OPENROUTER_API_KEY,
)

# --- Models ---
class ChatRequest(BaseModel):
    query: str
    selected_text: str = None # Optional selected text from the book

class HealthCheckResponse(BaseModel):
    status: str = "ok"

# --- Endpoints ---
@app.get("/health", response_model=HealthCheckResponse)
async def health_check():
    return HealthCheckResponse()

@app.post("/chat")
async def chat_with_rag(request: ChatRequest):
    if not request.query:
        raise HTTPException(status_code=400, detail="Query cannot be empty")

    # 1. Retrieve relevant documents from Qdrant
    # This is a placeholder for actual retrieval logic.
    # We would typically embed the query first, then search Qdrant.
    # For now, let's assume we retrieve some dummy context.

    # TODO: Implement embedding generation for the query
    # For demonstration, we'll skip embedding the query and directly
    # retrieve dummy data. In a real RAG system, you'd use a text embedding model.
    # For example:
    # query_embedding = openai_client.embeddings.create(input=request.query, model="text-embedding-ada-002").data[0].embedding

    # Dummy retrieval
    retrieved_context = "This is a placeholder context from the book relevant to your query."
    if request.selected_text:
        retrieved_context = f"User selected text: '{request.selected_text}'. " + retrieved_context

    # 2. Construct prompt for LLM
    messages = [
        {"role": "system", "content": "You are a helpful assistant that answers questions about the book 'Physical AI & Humanoid Robotics'. Use the provided context to answer questions."},
        {"role": "user", "content": f"Context from the book: {retrieved_context}\n\nQuestion: {request.query}"}
    ]

    # 3. Generate response using LLM
    try:
        completion = openai_client.chat.completions.create(
            model=OPENROUTER_MODEL,
            messages=messages
        )
        response_content = completion.choices[0].message.content
        return {"response": response_content}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"LLM API error: {str(e)}")

# --- Content Ingestion Placeholder (for later) ---
# This part would involve parsing Docusaurus markdown files,
# chunking them, creating embeddings, and uploading to Qdrant.
# We'll develop this separately later.
@app.post("/ingest-content")
async def ingest_content():
    # TODO: Implement content ingestion from Docusaurus book
    # This will involve:
    # 1. Reading markdown files from the book's docs directory
    # 2. Splitting content into manageable chunks
    # 3. Generating embeddings for each chunk using an embedding model (e.g., OpenAI's text-embedding-ada-002)
    # 4. Uploading chunks and their embeddings to Qdrant
    return {"message": "Content ingestion endpoint (to be implemented)"}
