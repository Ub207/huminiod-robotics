import os
import glob
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
import google.generativeai as genai
import uuid
import asyncio
from dotenv import load_dotenv

load_dotenv()

class IngestionService:
    def __init__(self):
        genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
        self.qdrant_client = AsyncQdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
        )
        self.collection_name = "huminiod_robotics"
        self.docs_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../huminiod-robotics/docs"))

    async def get_embedding(self, text: str):
        # Add retry logic or rate limit handling if needed
        try:
             result = genai.embed_content(
                model="models/text-embedding-004",
                content=text,
                task_type="retrieval_document"
            )
             return result['embedding']
        except Exception as e:
            print(f"Error embedding chunk: {e}")
            return None

    async def ensure_collection(self):
        collections = await self.qdrant_client.get_collections()
        if not any(c.name == self.collection_name for c in collections.collections):
            await self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE), # text-embedding-004 is 768 dim
            )

    async def ingest_docs(self) -> int:
        await self.ensure_collection()
        
        files = glob.glob(os.path.join(self.docs_path, "**/*.md"), recursive=True)
        points = []
        
        total_chunks = 0
        
        for file_path in files:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
                
            chunks = [c.strip() for c in content.split("\n\n") if len(c.strip()) > 50]
            
            for chunk in chunks:
                embedding = await self.get_embedding(chunk)
                if embedding:
                    points.append(models.PointStruct(
                        id=str(uuid.uuid4()),
                        vector=embedding,
                        payload={
                            "content": chunk,
                            "source": os.path.basename(file_path)
                        }
                    ))
                    total_chunks += 1
                    
                # Rate limit protection (Gemini free tier has limits)
                await asyncio.sleep(0.5) 

        if points:
            # Batch upsert to avoid request size limits
            batch_size = 50
            for i in range(0, len(points), batch_size):
                await self.qdrant_client.upsert(
                    collection_name=self.collection_name,
                    points=points[i:i+batch_size]
                )
            
        return total_chunks
