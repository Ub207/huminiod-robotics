---
sidebar_position: 6
---

# AI-ROS Integration

## Connecting Intelligence to Physics

In physical AI and humanoid robotics, the real power emerges when cognitive capabilities (AI agents) are tightly integrated with physical systems (ROS). This integration allows AI to perceive the physical world through robot sensors and act upon it through robot actuators.

## Architecture of AI-ROS Integration

The integration typically follows this pattern:

```
AI Agent ↔ ROS Bridge ↔ ROS Nodes ↔ Robot Hardware
```

The "ROS Bridge" can be implemented in several ways:
1. Direct integration where AI code runs as ROS nodes
2. Middleware services that translate between AI APIs and ROS
3. Hybrid approaches combining both

## Implementing Direct AI-ROS Integration

Let's create a ROS 2 node that incorporates AI capabilities to make decisions for our humanoid robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from openai import OpenAI
import base64
import json
import os

class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')
        
        # Initialize OpenAI client
        self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        
        # Create subscribers for sensor data
        self.camera_subscriber = self.create_subscription(
            Image,
            '/head_camera/rgb_image',
            self.camera_callback,
            10
        )
        
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Create publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create publisher for AI status
        self.ai_status_publisher = self.create_publisher(
            String,
            '/ai_status',
            10
        )
        
        # Timer for periodic decision making
        self.timer = self.create_timer(5.0, self.make_decision)
        
        # Store current robot state
        self.current_image = None
        self.current_joint_states = None
        self.ai_model = "gpt-4-vision-preview"

    def camera_callback(self, msg):
        # Process image data - in real implementation, you'd extract image from msg
        # For now, we'll just store that we have new image data
        self.current_image = msg
        self.get_logger().info('Received new camera image')

    def joint_state_callback(self, msg):
        # Store current joint states
        self.current_joint_states = msg
        self.get_logger().info(f'Received joint states for {len(msg.name)} joints')

    def make_decision(self):
        if not self.current_image:
            self.get_logger().info('No image data available for AI processing')
            return

        # Prepare data for AI decision
        prompt = """
        You are an AI controlling a humanoid robot. Based on the robot's vision and joint states,
        decide what the robot should do next. Possible actions:
        
        1. Move forward
        2. Turn left
        3. Turn right
        4. Stop and analyze surroundings
        5. Wave to detected human
        
        Respond with ONLY a JSON object containing:
        {
          "action": "forward|left|right|stop|wave",
          "reason": "brief explanation of why this action was chosen",
          "expected_outcome": "what the robot expects to happen"
        }
        """
        
        # In a real implementation, we would encode the image and send it to the AI
        # For this example, we'll simulate the AI response
        ai_response = self.simulate_ai_decision(prompt)
        
        # Publish AI status
        status_msg = String()
        status_msg.data = f'AI Decision: {ai_response["action"]} - {ai_response["reason"]}'
        self.ai_status_publisher.publish(status_msg)
        
        # Execute the action
        self.execute_action(ai_response["action"])

    def simulate_ai_decision(self, prompt):
        # In a real implementation, this would call the AI model
        # For now, we'll return a simulated response
        return {
            "action": "forward",
            "reason": "Clear path detected ahead",
            "expected_outcome": "Move toward target location"
        }

    def execute_action(self, action):
        cmd = Twist()
        
        if action == "forward":
            cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        elif action == "left":
            cmd.angular.z = 0.5  # Turn left
        elif action == "right":
            cmd.angular.z = -0.5  # Turn right
        elif action == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif action == "wave":
            # Trigger a wave motion through a custom topic
            self.get_logger().info('Triggering wave motion')
            return
        
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(f'Published command: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    ai_decision_node = AIDecisionNode()
    
    try:
        rclpy.spin(ai_decision_node)
    except KeyboardInterrupt:
        pass
    
    ai_decision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Handling Complex AI Interactions

In humanoid robotics, we often need to handle complex interactions that require memory and planning. Here's an example that implements a simple memory system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from qdrant_client import QdrantClient
from qdrant_client.http import models
import uuid

class AIMemoryNode(Node):
    def __init__(self):
        super().__init__('ai_memory_node')
        
        # Initialize Qdrant client for memory storage
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
        )
        self.memory_collection = "robot_memory"
        
        # Create subscription for experiences to remember
        self.experience_subscriber = self.create_subscription(
            String,
            '/robot_experiences',
            self.experience_callback,
            10
        )
        
        # Create subscription for queries about past experiences
        self.query_subscriber = self.create_subscription(
            String,
            '/memory_query',
            self.query_callback,
            10
        )
        
        # Create publisher for memory responses
        self.memory_publisher = self.create_publisher(
            String,
            '/memory_response',
            10
        )
        
        # Ensure memory collection exists
        self.ensure_memory_collection()

    def ensure_memory_collection(self):
        collections = self.qdrant_client.get_collections()
        if not any(c.name == self.memory_collection for c in collections.collections):
            self.qdrant_client.create_collection(
                collection_name=self.memory_collection,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE)
            )

    async def get_embedding(self, text: str):
        # Get embedding from OpenAI
        # In a real implementation, you'd use the OpenAI API
        # For this example, we'll simulate
        return [0.1] * 1536  # Mock embedding

    def experience_callback(self, msg):
        # Store the experience in vector database
        experience_text = msg.data
        self.get_logger().info(f'Remembering experience: {experience_text}')
        
        # In real implementation, you'd create an embedding and store it
        # For now we'll just log
        pass

    def query_callback(self, msg):
        query_text = msg.data
        self.get_logger().info(f'Searching memory for: {query_text}')
        
        # In real implementation, you'd search the vector database
        # For now we'll just respond with a mock answer
        response = String()
        response.data = f"Based on memory, I recall that when you asked about '{query_text}', there was no relevant past experience."
        self.memory_publisher.publish(response)

def main(args=None):
    rclpy.init(args=args)
    ai_memory_node = AIMemoryNode()
    
    try:
        rclpy.spin(ai_memory_node)
    except KeyboardInterrupt:
        pass
    
    ai_memory_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Planning with AI Agents

AI agents can also be used for higher-level planning. Here's an example of task planning:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class AIPlannerNode(Node):
    def __init__(self):
        super().__init__('ai_planner_node')
        
        # Create subscription for high-level goals
        self.goal_subscriber = self.create_subscription(
            String,
            '/high_level_goal',
            self.goal_callback,
            10
        )
        
        # Create publisher for action sequences
        self.action_publisher = self.create_publisher(
            String,
            '/action_sequence',
            10
        )

    def goal_callback(self, msg):
        goal = msg.data
        self.get_logger().info(f'Received goal: {goal}')
        
        # Plan actions to achieve the goal
        action_plan = self.create_action_plan(goal)
        
        # Publish the action sequence
        plan_msg = String()
        plan_msg.data = json.dumps(action_plan)
        self.action_publisher.publish(plan_msg)
        
        self.get_logger().info(f'Published action plan: {action_plan}')

    def create_action_plan(self, goal):
        # In a real implementation, this would call an AI model to generate a plan
        # For this example, we'll return a mock plan
        
        plans = {
            "navigate to kitchen": [
                {"action": "turn", "params": {"degrees": 90}},
                {"action": "move_forward", "params": {"distance": 3.0}},
                {"action": "turn", "params": {"degrees": -90}},
                {"action": "move_forward", "params": {"distance": 5.0}}
            ],
            "pick up object": [
                {"action": "approach_object", "params": {"object_id": "target"}},
                {"action": "grasp", "params": {"arm": "right", "force": 50.0}},
                {"action": "lift", "params": {"height": 0.2}}
            ],
            "wave to person": [
                {"action": "look_at", "params": {"entity": "person"}},
                {"action": "raise_arm", "params": {"arm": "right", "angle": 90}},
                {"action": "wave", "params": {"repetitions": 3}}
            ]
        }
        
        # Return the plan if we have one, otherwise a default response
        return plans.get(goal, [{"action": "unknown_goal", "params": {"goal": goal}}])

def main(args=None):
    rclpy.init(args=args)
    ai_planner_node = AIPlannerNode()
    
    try:
        rclpy.spin(ai_planner_node)
    except KeyboardInterrupt:
        pass
    
    ai_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for AI-ROS Integration

### 1. Error Handling
Always implement robust error handling when connecting AI systems to ROS:

```python
# Example of safe AI integration with error handling
def safe_ai_call(self, prompt):
    try:
        response = self.openai_client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}]
        )
        return response.choices[0].message.content
    except Exception as e:
        self.get_logger().error(f'AI service error: {e}')
        # Implement fallback behavior
        return self.fallback_behavior()
```

### 2. Latency Considerations
AI calls can be slow, so consider implementing:
- Caching for repeated requests
- Asynchronous processing
- Fallback behaviors when AI is unavailable

### 3. Safety Checks
Always validate AI outputs before sending to hardware:
- Check joint angle limits
- Validate velocities
- Ensure safe movement patterns

## Practical Exercise

Create a simple ROS node that integrates with an AI service to recognize objects in images from a robot's camera. The node should:

1. Subscribe to the robot's camera topic
2. Send images to an AI service for object recognition
3. Publish the recognized objects to a new topic
4. Include error handling for when the AI service is unavailable

## Next Steps

With AI integrated into our ROS system, we now need to create simulation environments where we can safely test and train our intelligent humanoid robots. In the next module, we'll explore Gazebo and Unity for creating digital twins.