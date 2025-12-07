import os
import google.generativeai as genai
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from typing import Tuple, List, Optional
from dotenv import load_dotenv

load_dotenv()

class RAGService:
    def __init__(self):
        genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
        self.model = genai.GenerativeModel('gemini-1.5-flash')
        self.qdrant_client = AsyncQdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
        )
        self.collection_name = "huminiod_robotics"

    async def get_embedding(self, text: str) -> List[float]:
        # Gemini embedding model
        result = genai.embed_content(
            model="models/text-embedding-004",
            content=text,
            task_type="retrieval_query"
        )
        return result['embedding']

    async def get_response(self, message: str, selected_text: Optional[str] = None) -> Tuple[str, List[str]]:
        query_text = message
        if selected_text:
            query_text = f"Context: {selected_text}\nQuestion: {message}"

        embedding = await self.get_embedding(query_text)

        search_result = await self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=embedding,
            limit=5
        )

        context = "\n\n".join([hit.payload["content"] for hit in search_result])
        sources = [hit.payload["source"] for hit in search_result]

        system_instruction = "You are a helpful assistant for the book 'Physical AI & Humanoid Robotics'. Answer the user's question based strictly on the provided context."
        
        prompt = f"""{system_instruction}

Context:
{context}

User Selection (if any):
{selected_text if selected_text else "None"}

Question:
{message}
"""
        response = await self.model.generate_content_async(prompt)
        
        return response.text, sources
