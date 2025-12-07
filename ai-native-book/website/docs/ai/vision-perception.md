---
sidebar_position: 4
---

# Vision and Perception

## Perception in Physical AI

Perception is the foundation of physical AI, enabling robots to understand and interpret their environment. For humanoid robots, perception systems must process multiple sensory modalities to create a coherent understanding of the physical world.

## Multi-Modal Perception

Humanoid robots typically integrate several types of sensors:

1. **Vision**: Cameras, depth sensors, thermal imaging
2. **Proprioception**: Joint encoders, IMUs, force/torque sensors
3. **Audio**: Microphones for sound localization and speech
4. **Tactile**: Pressure sensors for touch and grasp
5. **Range**: LiDAR, ultrasonic sensors for distance measurement

## AI-Enhanced Vision Processing

### Image Understanding with Large Models

Large vision-language models can provide rich scene understanding:

```python
from openai import OpenAI
import base64
from PIL import Image
import io
import cv2

class AIVisionSystem:
    def __init__(self):
        self.client = OpenAI()
        self.system_prompt = """
        You are an AI assistant that analyzes images from a humanoid robot's perspective.
        Describe the scene in detail, focusing on:
        1. Objects present and their positions
        2. People and their activities
        3. Navigable paths and obstacles
        4. Potential interaction points
        5. Safety considerations
        
        Respond with a structured JSON containing:
        {
          "objects": [{"name": str, "position": str, "distance": float}],
          "people": [{"action": str, "location": str}],
          "navigation": {"clear_paths": int, "obstacles": int, "safety": str},
          "interactions": [{"type": str, "location": str}],
          "summary": str
        }
        """
    
    def encode_image(self, image_path):
        """Encode image for API submission"""
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')
    
    def compress_image(self, image_path, max_size=512):
        """Compress image to reduce API costs"""
        img = Image.open(image_path)
        
        # Maintain aspect ratio
        if img.width > img.height:
            new_width = min(max_size, img.width)
            new_height = int(img.height * new_width / img.width)
        else:
            new_height = min(max_size, img.height)
            new_width = int(img.width * new_height / img.height)
        
        img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)
        
        # Save to bytes
        buffer = io.BytesIO()
        img.save(buffer, format="JPEG", quality=85)
        return buffer.getvalue()
    
    def analyze_scene(self, image_path):
        """Analyze a scene using AI vision"""
        # Compress image
        compressed_image = self.compress_image(image_path)
        
        # Encode for API
        base64_image = base64.b64encode(compressed_image).decode('utf-8')
        
        response = self.client.chat.completions.create(
            model="gpt-4-vision-preview",
            messages=[
                {
                    "role": "system",
                    "content": self.system_prompt
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": "Analyze this robot's camera image."
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}"
                            }
                        }
                    ]
                }
            ],
            max_tokens=1000
        )
        
        try:
            result = eval(response.choices[0].message.content)
            return result
        except:
            # Handle case where response isn't valid JSON
            return {
                "objects": [],
                "people": [],
                "navigation": {"clear_paths": 0, "obstacles": 0, "safety": "unknown"},
                "interactions": [],
                "summary": "Could not parse vision analysis"
            }
```

### Real-time Object Detection

For real-time applications, traditional computer vision combined with AI can be more efficient:

```python
import cv2
import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image

class RealTimeVision:
    def __init__(self):
        # Load a pre-trained object detection model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.eval()
        
        # Classes of interest for humanoid robots (COCO dataset)
        self.target_classes = {
            'person', 'bottle', 'cup', 'chair', 'couch', 'potted plant', 
            'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 
            'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase'
        }
    
    def detect_objects(self, image):
        """Detect objects in an image"""
        # Convert OpenCV image (BGR) to RGB
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Convert to PIL for model processing
        pil_image = Image.fromarray(rgb_image)
        
        # Run detection
        results = self.model(pil_image)
        
        # Convert results to our format
        detections = []
        for detection in results.xyxy[0]:  # x1, y1, x2, y2, conf, cls
            x1, y1, x2, y2, confidence, class_id = detection
            
            # Get class name
            class_name = self.model.names[int(class_id)]
            
            # Only include relevant objects
            if class_name in self.target_classes:
                detection_dict = {
                    'class': class_name,
                    'confidence': float(confidence),
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                    'center': [(float(x1) + float(x2)) / 2, (float(y1) + float(y2)) / 2]
                }
                
                detections.append(detection_dict)
        
        return detections
    
    def track_objects(self, current_detections, previous_detections, max_distance=50):
        """Perform simple object tracking using bounding box overlap"""
        # Implementation would track objects between frames
        # For now, return current detections
        return current_detections
```

## Depth Perception and 3D Understanding

### Depth Estimation

Understanding 3D structure is crucial for navigation and manipulation:

```python
import open3d as o3d
import numpy as np

class DepthProcessor:
    def __init__(self):
        # For RGB-D input, process depth and color together
        pass
    
    def estimate_depth(self, rgb_image, depth_image=None):
        """Process depth information from RGB or RGB-D"""
        if depth_image is not None:
            # Use provided depth image
            return self._process_depth_image(rgb_image, depth_image)
        else:
            # Use monocular depth estimation
            return self._estimate_monocular_depth(rgb_image)
    
    def _process_depth_image(self, rgb_image, depth_image):
        """Process stereo or structured light depth"""
        # Convert to point cloud
        pcd = o3d.geometry.PointCloud()
        
        # Create coordinate grids
        height, width = depth_image.shape
        fx, fy = 525, 525  # Camera intrinsic parameters (example)
        cx, cy = width/2, height/2
        
        # Generate points
        points = []
        colors = []
        
        for v in range(height):
            for u in range(width):
                z = depth_image[v, u] / 1000.0  # Convert to meters
                if z > 0 and z < 10:  # Valid depth range
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])
                    
                    # Get color
                    colors.append(rgb_image[v, u] / 255.0)
        
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
        
        return pcd
    
    def segment_objects_by_depth(self, point_cloud, cluster_distance=0.05):
        """Segment objects using Euclidean clustering"""
        # Perform clustering
        labels = np.array(point_cloud.cluster_dbscan(
            eps=cluster_distance,  # Cluster distance threshold
            min_points=50,        # Minimum points per cluster
            print_progress=False
        ))
        
        # Create clusters
        max_label = labels.max()
        clusters = []
        
        for i in range(max_label + 1):
            cluster_indices = np.where(labels == i)[0]
            if len(cluster_indices) > 100:  # Filter small clusters
                cluster_points = np.asarray(point_cloud.points)[cluster_indices]
                clusters.append({
                    'points': cluster_points,
                    'center': np.mean(cluster_points, axis=0),
                    'size': len(cluster_points)
                })
        
        return clusters
```

## Sensor Fusion

Combine multiple sensors for robust perception:

```python
class SensorFusion:
    def __init__(self):
        self.vision_system = AIVisionSystem()
        self.realtime_vision = RealTimeVision()
        self.depth_processor = DepthProcessor()
        
        # Maintain state across sensor readings
        self.tracked_objects = {}
        self.environment_map = {}
    
    def fuse_sensors(self, camera_data, depth_data, lidar_data, imu_data):
        """Fuse data from multiple sensors"""
        fused_perception = {
            'timestamp': camera_data['timestamp'],
            'objects': [],
            'spatial_map': {},
            'navigation_info': {},
            'safety_assessment': {}
        }
        
        # Process visual data
        if 'image' in camera_data:
            # Real-time object detection
            vision_detections = self.realtime_vision.detect_objects(
                camera_data['image']
            )
            
            # AI-enhanced scene analysis (when computational budget allows)
            if camera_data.get('analyze_with_ai', False):
                # This would be done less frequently due to cost
                ai_analysis = self.vision_system.analyze_scene(camera_data['image_path'])
                fused_perception['ai_analysis'] = ai_analysis
        
        # Process depth data
        if depth_data is not None:
            point_cloud = self.depth_processor._process_depth_image(
                camera_data['image'],
                depth_data
            )
            clusters = self.depth_processor.segment_objects_by_depth(point_cloud)
            
            # Add depth-based objects
            for cluster in clusters:
                fused_perception['objects'].append({
                    'type': 'depth_segment',
                    'position': cluster['center'],
                    'size': cluster['size']
                })
        
        # Process LiDAR data
        if lidar_data is not None:
            # Process laser scan for navigation
            fused_perception['navigation_info'] = self._process_lidar_navigation(lidar_data)
            
            # Identify obstacles and free space
            obstacles = self._identify_obstacles(lidar_data)
            free_space = self._identify_free_space(lidar_data)
            
            fused_perception['spatial_map'] = {
                'obstacles': obstacles,
                'free_space': free_space
            }
        
        # Incorporate IMU data for motion understanding
        if imu_data is not None:
            fused_perception['motion_state'] = self._interpret_imu(imu_data)
        
        return fused_perception
    
    def _process_lidar_navigation(self, lidar_data):
        """Process LiDAR data for navigation"""
        # Implement navigation-specific processing
        # Return information relevant to path planning
        return {
            'clear_front': lidar_data[320] > 1.0,  # Distance at front (assuming 640 points, front is at index 320)
            'left_clear': lidar_data[160] > 1.0,   # Left side
            'right_clear': lidar_data[480] > 1.0,  # Right side
            'obstacle_distances': [d for d in lidar_data if d < 2.0]  # All close obstacles
        }
    
    def _identify_obstacles(self, lidar_data):
        """Identify obstacle positions from LiDAR"""
        obstacles = []
        for i, distance in enumerate(lidar_data):
            if distance < 0.5:  # Obstacle threshold
                angle = i * (2 * 3.14159 / len(lidar_data)) - 3.14159  # Convert index to angle
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                obstacles.append({'x': x, 'y': y, 'distance': distance})
        return obstacles
    
    def _interpret_imu(self, imu_data):
        """Interpret IMU data for robot state"""
        return {
            'acceleration': imu_data.linear_acceleration,
            'angular_velocity': imu_data.angular_velocity,
            'orientation': imu_data.orientation,
            'is_stable': abs(imu_data.linear_acceleration.z) > 9.5  # Assuming z is gravity
        }
```

## Implementation in ROS

Create ROS nodes that handle perception tasks:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan, Imu
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import numpy as np
from cv_bridge import CvBridge

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Initialize sensor fusion system
        self.sensor_fusion = SensorFusion()
        self.cv_bridge = CvBridge()
        
        # Subscriptions for all sensor types
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Publishers for processed data
        self.object_publisher = self.create_publisher(
            String,  # In practice, use custom message type
            '/perceived_objects',
            10
        )
        
        self.navigation_publisher = self.create_publisher(
            String,  # In practice, use custom message type
            '/navigation_map',
            10
        )
        
        # Store latest sensor data for fusion
        self.latest_image = None
        self.latest_depth = None
        self.latest_lidar = None
        self.latest_imu = None
        
        # Timer for sensor fusion
        self.fusion_timer = self.create_timer(0.1, self.fuse_sensors_callback)
    
    def image_callback(self, msg):
        """Process camera image"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = {
                'image': cv_image,
                'timestamp': msg.header.stamp
            }
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def depth_callback(self, msg):
        """Process depth image"""
        try:
            # Convert ROS image to numpy array
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "32FC1")  # 32-bit float
            self.latest_depth = np.array(depth_image, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f'Error processing depth: {e}')
    
    def lidar_callback(self, msg):
        """Process LiDAR scan"""
        try:
            self.latest_lidar = np.array(msg.ranges)
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR: {e}')
    
    def imu_callback(self, msg):
        """Process IMU data"""
        try:
            self.latest_imu = msg
        except Exception as e:
            self.get_logger().error(f'Error processing IMU: {e}')
    
    def fuse_sensors_callback(self):
        """Fusion timer callback"""
        if not all([self.latest_image, self.latest_lidar, self.latest_imu]):
            return  # Wait for all sensors
        
        try:
            # Perform sensor fusion
            fused_data = self.sensor_fusion.fuse_sensors(
                self.latest_image,
                self.latest_depth,
                self.latest_lidar,
                self.latest_imu
            )
            
            # Publish processed data
            objects_msg = String()
            objects_msg.data = str(fused_data.get('objects', []))
            self.object_publisher.publish(objects_msg)
            
            nav_msg = String()
            nav_msg.data = str(fused_data.get('spatial_map', {}))
            self.navigation_publisher.publish(nav_msg)
            
            self.get_logger().info(f'Fused perception data: {len(fused_data.get("objects", []))} objects detected')
        except Exception as e:
            self.get_logger().error(f'Error in sensor fusion: {e}')

def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()
    
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    
    perception_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Perception for Human Interaction

Special considerations for perceiving and understanding humans:

```python
class HumanPerception:
    def __init__(self):
        # Models for human detection and pose estimation
        self.pose_model = self._load_pose_model()
        self.face_model = self._load_face_model()
    
    def detect_humans(self, image):
        """Detect humans in the environment"""
        # Use pose estimation to detect humans
        poses = self.pose_model.estimate_poses(image)
        
        humans = []
        for pose in poses:
            if self._is_human_pose(pose):
                humans.append({
                    'position': self._estimate_position_from_pose(pose),
                    'pose': pose,
                    'gaze_direction': self._estimate_gaze_direction(pose),
                    'intention': self._infer_intention(pose)
                })
        
        return humans
    
    def _is_human_pose(self, pose):
        """Verify that detected pose is likely human"""
        # Check for human-like proportions and joint angles
        return True  # Implementation would have detailed checks
    
    def _estimate_position_from_pose(self, pose):
        """Estimate human position in 3D space"""
        # Use key body joints to estimate position relative to robot
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}
    
    def _infer_intention(self, pose):
        """Infer human intention from pose"""
        # Analyze pose to understand if person is reaching, pointing, etc.
        return "neutral"
    
    def process_human_interaction(self, humans, robot_state):
        """Process detected humans for interaction"""
        interaction_targets = []
        
        for human in humans:
            # Determine if human is paying attention to robot
            if self._is_attention_to_robot(human, robot_state):
                interaction_targets.append({
                    'human': human,
                    'attention_level': self._calculate_attention(human, robot_state),
                    'recommended_action': self._recommend_interaction_action(human)
                })
        
        return interaction_targets
    
    def _is_attention_to_robot(self, human, robot_state):
        """Check if human is paying attention to robot"""
        return True  # Implementation would check gaze direction, etc.
    
    def _calculate_attention(self, human, robot_state):
        """Calculate level of attention"""
        return 0.8  # High attention
    
    def _recommend_interaction_action(self, human):
        """Recommend appropriate interaction"""
        # Return appropriate interaction based on human state
        return "greet"  # or "wait", "approach", etc.
```

## Privacy and Ethical Considerations

### Privacy-Preserving Perception
- Blur or mask personal identifying information
- Local processing when possible
- Data retention policies
- User consent mechanisms

### Bias Mitigation
- Diverse training data
- Regular bias audits
- Fairness metrics

## Practical Exercise

1. Implement a basic object detection system using YOLO
2. Add depth information to detected objects
3. Create a ROS node that publishes detected object positions
4. Integrate with the memory system to remember object locations
5. Test the system with simulated sensor data

## Next Steps

With vision and perception systems established, we'll explore how to integrate these components with the broader AI agent architecture to enable intelligent behavior in humanoid robots.