---
sidebar_position: 2
---

# Memory Systems

## The Importance of Memory in AI Agents

Memory is crucial for AI agents in robotics because it enables them to:
- Learn from past experiences
- Remember important information about the environment
- Build models of the world over time
- Avoid repeating mistakes
- Develop more sophisticated behaviors through experience

## Types of Memory in Robotics

### 1. Episodic Memory
Stores specific experiences and events, tagged with temporal and spatial context:
- "At 2:30 PM, in room B, I encountered an obstacle here"
- "The door was locked when I tried to enter at this location"

### 2. Semantic Memory
Stores general knowledge and facts:
- "Tables are typically 0.7-0.8 meters high"
- "Red traffic lights mean stop"
- "Humans prefer to maintain 1 meter of personal space"

### 3. Procedural Memory
Stores learned skills and procedures:
- How to walk up stairs
- How to pick up objects of various shapes
- Navigation strategies for different environments

## Memory Architecture for Robotics

### Short-term Memory
- High-resolution, detailed information
- Limited capacity and duration
- Used for immediate processing and decision-making
- Implemented with working memory systems

### Long-term Memory
- Lower resolution but much larger capacity
- Persistent storage of learned information
- Organized for efficient retrieval
- Implemented with vector databases like Qdrant

## Vector Databases for Robot Memory

Vector databases like Qdrant excel at storing robotics memories because they:

1. **Enable Semantic Search**: Find relevant memories based on meaning rather than exact text matching
2. **Handle Multimodal Data**: Store not just text but also image, audio, and other embeddings
3. **Scale Efficiently**: Handle millions of memory entries with reasonable performance
4. **Support Metadata**: Store contextual information with each memory

### Implementation Example with Qdrant

```python
import numpy as np
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import OpenAI
import os
import uuid
from datetime import datetime

class RobotMemorySystem:
    def __init__(self, collection_name="robot_memory"):
        self.client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
        )
        self.collection_name = collection_name
        self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        
        # Create collection if it doesn't exist
        self._ensure_collection()
        
    def _ensure_collection(self):
        collections = self.client.get_collections()
        if not any(c.name == self.collection_name for c in collections.collections):
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
            )
    
    async def get_embedding(self, text: str):
        """Get embedding from OpenAI"""
        response = await self.openai_client.embeddings.create(
            input=text,
            model="text-embedding-3-small"
        )
        return response.data[0].embedding
    
    async def store_memory(self, content: str, metadata: dict = None):
        """Store a memory with its embedding"""
        if metadata is None:
            metadata = {}
        
        # Add timestamp
        metadata["timestamp"] = datetime.now().isoformat()
        
        # Create embedding
        vector = await self.get_embedding(content)
        
        # Store in Qdrant
        point = models.PointStruct(
            id=str(uuid.uuid4()),
            vector=vector,
            payload={
                "content": content,
                "metadata": metadata
            }
        )
        
        self.client.upsert(
            collection_name=self.collection_name,
            points=[point]
        )
        
        return point.id
    
    async def retrieve_memories(self, query: str, limit: int = 5):
        """Retrieve relevant memories based on a query"""
        query_vector = await self.get_embedding(query)
        
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit
        )
        
        return [
            {
                "content": result.payload["content"],
                "metadata": result.payload["metadata"],
                "score": result.score
            }
            for result in search_results
        ]
    
    async def store_experience(self, sensors_data: dict, action_taken: str, outcome: str, context: dict = None):
        """Store a complete experience episode"""
        experience = f"""
        SENSORS: {sensors_data}
        ACTION: {action_taken}
        OUTCOME: {outcome}
        """
        
        metadata = {
            "type": "experience",
            "action": action_taken,
            "outcome": outcome
        }
        
        if context:
            metadata.update(context)
        
        return await self.store_memory(experience, metadata)
```

## Memory Management Strategies

### 1. Forgetting Mechanisms
Not all memories are equally valuable; implement strategies to manage memory:

```python
def forget_old_memories(self, days_to_keep=30):
    """Remove memories older than specified days"""
    cutoff_time = datetime.now() - timedelta(days=days_to_keep)
    
    # Delete points with older timestamps
    self.client.delete(
        collection_name=self.collection_name,
        points_selector=models.Filter(
            must=[
                models.FieldCondition(
                    key="metadata.timestamp",
                    range=models.Range(
                        lt=cutoff_time.timestamp()
                    )
                )
            ]
        )
    )
```

### 2. Memory Consolidation
Combine similar experiences to reduce redundancy:

```python
async def consolidate_similar_memories(self, threshold=0.8):
    """Find and merge highly similar memories"""
    all_points = self.client.scroll(
        collection_name=self.collection_name,
        limit=10000  # Adjust based on your needs
    )
    
    for point1, point2 in itertools.combinations(all_points[0], 2):
        # Calculate similarity (simplified approach)
        similarity = self._calculate_similarity(point1.vector, point2.vector)
        
        if similarity > threshold:
            # Merge similar memories
            merged_content = self._merge_memories(
                point1.payload["content"], 
                point2.payload["content"]
            )
            
            # Update one and delete the other
            # Implementation would depend on specific requirements
```

## Implementing Memory for Different Modalities

### Visual Memory
Store embeddings of visual scenes for later recognition:

```python
async def store_visual_memory(self, image_path: str, description: str):
    """Store visual information with its semantic description"""
    # In a real implementation, you'd create image embeddings
    # For now, we'll use the description as the basis for search
    content = f"IMAGE: {description} - Captured at {datetime.now()}"
    
    metadata = {
        "type": "visual",
        "image_path": image_path,
        "timestamp": datetime.now().isoformat()
    }
    
    return await self.store_memory(content, metadata)
```

### Spatial Memory
Maintain maps of locations and their properties:

```python
async def store_location_memory(self, location: dict, description: str):
    """Store information about a specific location"""
    content = f"LOCATION: {location} - {description}"
    
    metadata = {
        "type": "location",
        "coordinates": location,
        "timestamp": datetime.now().isoformat()
    }
    
    return await self.store_memory(content, metadata)

async def find_relevant_locations(self, query: str):
    """Find locations relevant to a query"""
    memories = await self.retrieve_memories(query)
    return [m for m in memories if m["metadata"].get("type") == "location"]
```

## Memory Integration with ROS

Connect the memory system to ROS topics and services:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

class MemoryIntegrationNode(Node):
    def __init__(self):
        super().__init__('memory_integration_node')
        self.memory_system = RobotMemorySystem()
        
        # Subscriptions for data to store in memory
        self.sensor_subscriber = self.create_subscription(
            String,
            '/robot_experiences',
            self.store_sensor_experience,
            10
        )
        
        self.location_subscriber = self.create_subscription(
            Pose,
            '/robot_location',
            self.store_location,
            10
        )
        
        # Service to query memories
        self.query_service = self.create_service(
            String,
            'query_memory',
            self.query_memory_callback
        )
    
    def store_sensor_experience(self, msg):
        """Store sensor experiences in memory"""
        # Extract action and outcome from message
        # In practice, you'd parse more complex data
        future = self.memory_system.store_memory(msg.data, {"type": "sensor_data"})
        # Handle future appropriately
    
    def store_location(self, msg):
        """Store location information in memory"""
        location_data = {
            "x": msg.position.x,
            "y": msg.position.y,
            "z": msg.position.z
        }
        # Store location in memory system
    
    def query_memory_callback(self, request, response):
        """Handle memory queries"""
        # Implementation would retrieve and format memories
        response.data = "Query result"
        return response
```

## Ethical Considerations in Robot Memory

### Privacy
- Respect privacy when storing data that might contain personal information
- Implement appropriate access controls
- Consider data retention policies

### Bias
- Monitor for biases that might develop in memory systems
- Implement techniques to prevent bias propagation
- Regularly audit memory contents

## Practical Exercise

1. Implement a Qdrant-based memory system for a simple robot
2. Store different types of memories (episodic, semantic, procedural)
3. Implement retrieval functions for different query types
4. Create a ROS node that integrates the memory system
5. Test memory retrieval in various scenarios

## Next Steps

With our memory system established, we'll explore how AI agents plan actions and make decisions in the next section.