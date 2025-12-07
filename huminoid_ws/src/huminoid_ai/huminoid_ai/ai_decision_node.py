import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from openai import OpenAI
import base64
import json
import os
from cv_bridge import CvBridge
import numpy as np
from qdrant_client import QdrantClient
from qdrant_client.http import models


class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')
        
        # Initialize OpenAI client
        self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        
        # Initialize Qdrant client for memory
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL", "http://localhost:6333"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.memory_collection = "robot_memory"
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Create subscribers for sensor data
        self.camera_subscriber = self.create_subscription(
            Image,
            '/huminoid/head_camera/image_raw',
            self.camera_callback,
            10
        )
        
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/huminoid/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/huminoid/scan',
            self.lidar_callback,
            10
        )
        
        # Create publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/huminoid/cmd_vel',
            10
        )
        
        # Create publisher for AI status
        self.ai_status_publisher = self.create_publisher(
            String,
            '/huminoid/ai_status',
            10
        )
        
        # Timer for periodic decision making
        self.timer = self.create_timer(2.0, self.make_decision)
        
        # Store current robot state
        self.current_image = None
        self.current_joint_states = None
        self.current_lidar_data = None
        self.ai_model = os.getenv("AI_MODEL", "gpt-4o")
        
        # Initialize memory collection if it doesn't exist
        self._initialize_memory()
        
        self.get_logger().info('AI Decision Node initialized')

    def _initialize_memory(self):
        """Initialize the memory collection in Qdrant"""
        try:
            collections = self.qdrant_client.get_collections()
            if not any(c.name == self.memory_collection for c in collections.collections):
                self.qdrant_client.create_collection(
                    collection_name=self.memory_collection,
                    vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
                )
                self.get_logger().info(f'Created memory collection: {self.memory_collection}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize memory: {e}')

    async def get_embedding(self, text: str):
        """Get embedding from OpenAI"""
        try:
            response = await self.openai_client.embeddings.create(
                input=text,
                model="text-embedding-3-small"
            )
            return response.data[0].embedding
        except Exception as e:
            self.get_logger().error(f'Error getting embedding: {e}')
            # Return mock embedding in case of error
            return [0.0] * 1536

    def camera_callback(self, msg):
        """Process camera images from robot"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            self.get_logger().info('Received camera image')
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def joint_state_callback(self, msg):
        """Process joint state information"""
        self.current_joint_states = msg
        joint_info = {name: pos for name, pos in zip(msg.name, msg.position)}
        self.get_logger().info(f'Received joint states for {len(msg.name)} joints')

    def lidar_callback(self, msg):
        """Process LiDAR data"""
        self.current_lidar_data = msg.ranges
        self.get_logger().info(f'Received LiDAR data with {len(msg.ranges)} points')

    def make_decision(self):
        """Main decision making function"""
        if not all([self.current_joint_states, self.current_lidar_data]):
            self.get_logger().info('Waiting for complete sensor data')
            return

        # Prepare context for AI
        context = self._prepare_context()
        
        # Get AI decision
        ai_response = self._get_ai_decision(context)
        
        if ai_response:
            # Publish AI status
            status_msg = String()
            status_msg.data = f'AI Decision: {ai_response.get("action", "unknown")} - {ai_response.get("reason", "")}'
            self.ai_status_publisher.publish(status_msg)
            
            # Execute the action
            self._execute_action(ai_response)

    def _prepare_context(self):
        """Prepare context for AI decision making"""
        context = {
            "timestamp": self.get_clock().now().seconds_nanoseconds(),
            "robot_state": {
                "joint_positions": {name: pos for name, pos in 
                                   zip(self.current_joint_states.name, 
                                       self.current_joint_states.position)} if self.current_joint_states else {},
                "lidar_summary": self._summarize_lidar() if self.current_lidar_data else {}
            }
        }
        return context

    def _summarize_lidar(self):
        """Create a summary of LiDAR data"""
        if not self.current_lidar_data:
            return {}
        
        # Get distances in different directions (simplified)
        ranges = np.array(self.current_lidar_data)
        ranges = ranges[np.isfinite(ranges)]  # Remove invalid values
        
        if len(ranges) == 0:
            return {}
        
        return {
            "min_distance": float(np.min(ranges)),
            "front_distance": float(ranges[len(ranges)//2]) if len(ranges) > 0 else float('inf'),
            "left_distance": float(ranges[len(ranges)//4]) if len(ranges) > 0 else float('inf'),
            "right_distance": float(ranges[3*len(ranges)//4]) if len(ranges) > 0 else float('inf'),
            "obstacle_count": int(np.sum(ranges < 1.0))  # Count obstacles within 1m
        }

    def _get_ai_decision(self, context):
        """Get decision from AI model"""
        try:
            prompt = f"""
            You are an AI controlling a humanoid robot. Based on the current sensor data:
            {json.dumps(context, indent=2)}
            
            Decide what the robot should do next. Possible actions:
            1. Move forward slowly
            2. Turn left
            3. Turn right
            4. Stop and analyze surroundings
            5. Wait for human instruction
            
            Respond with ONLY a JSON object containing:
            {{
              "action": "forward|left|right|stop|wait",
              "reason": "brief explanation of why this action was chosen",
              "confidence": "high|medium|low"
            }}
            """
            
            response = self.openai_client.chat.completions.create(
                model=self.ai_model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that makes decisions for a humanoid robot based on sensor data. Respond only with properly formatted JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=200
            )
            
            # Parse the response
            content = response.choices[0].message.content.strip()
            
            # Remove any markdown formatting if present
            if content.startswith('```json'):
                content = content[7:]  # Remove ```json
            if content.endswith('```'):
                content = content[:-3]  # Remove ```
            
            decision = json.loads(content)
            return decision
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing AI response: {e}')
            return {"action": "stop", "reason": "AI response parsing error", "confidence": "low"}
        except Exception as e:
            self.get_logger().error(f'Error getting AI decision: {e}')
            return {"action": "stop", "reason": "AI communication error", "confidence": "low"}

    def _execute_action(self, decision):
        """Execute the action decided by AI"""
        cmd = Twist()
        
        action = decision.get("action", "stop")
        
        if action == "forward":
            cmd.linear.x = 0.3  # Move forward slowly
            cmd.angular.z = 0.0
        elif action == "left":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn left
        elif action == "right":
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5  # Turn right
        elif action == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif action == "wait":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Default to stop for unknown actions
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(f'Published command: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    ai_decision_node = AIDecisionNode()
    
    try:
        rclpy.spin(ai_decision_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_decision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()