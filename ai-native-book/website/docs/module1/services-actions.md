---
sidebar_position: 4
---

# Services and Actions

## Services in ROS 2

While topics enable asynchronous communication through publish/subscribe, **services** provide synchronous request/response communication. This is useful for tasks that require a response, such as calibration, configuration, or complex computations.

### Creating a Service Server

Let's create a service that could handle a humanoid robot's calibration routine:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class CalibrationService(Node):
    def __init__(self):
        super().__init__('calibration_service')
        self.srv = self.create_service(
            SetBool, 
            'calibrate_robot', 
            self.calibrate_robot_callback
        )

    def calibrate_robot_callback(self, request, response):
        self.get_logger().info(f'Calibrating robot based on request: {request.data}')
        
        # Simulate calibration process
        success = self.perform_calibration()
        
        response.success = success
        response.message = f'Calibration {"succeeded" if success else "failed"}'
        return response

    def perform_calibration(self):
        # In real implementation, this would calibrate joints, sensors, etc.
        self.get_logger().info('Performing robot calibration sequence...')
        return True  # Simulate success

def main(args=None):
    rclpy.init(args=args)
    calibration_service = CalibrationService()
    
    try:
        rclpy.spin(calibration_service)
    except KeyboardInterrupt:
        pass
    
    calibration_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client

Here's how to create a client that calls the calibration service:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        self.cli = self.create_client(SetBool, 'calibrate_robot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, data):
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    calibration_client = CalibrationClient()
    
    response = calibration_client.send_request(True)
    calibration_client.get_logger().info(f'Result: {response.message}')
    
    calibration_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions in ROS 2

While services are good for short, synchronous tasks, **actions** are designed for long-running tasks that provide continuous feedback and can be canceled. This is perfect for humanoid robot movements, navigation, or complex manipulation tasks.

### Creating an Action Server

Let's create an action server for a humanoid walking gait:

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class WalkActionServer(Node):
    def __init__(self):
        super().__init__('walk_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,  # In practice, you'd define a custom action for walking
            'walk_forward',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        # Accept or reject a goal
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accept or reject a cancel request
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Simulate walking process
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            
            # Simulate walking time
            from time import sleep
            sleep(0.5)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    walk_action_server = WalkActionServer()
    
    try:
        rclpy.spin(walk_action_server)
    except KeyboardInterrupt:
        pass
    
    walk_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating an Action Client

Here's how to call the walking action:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class WalkActionClient(Node):
    def __init__(self):
        super().__init__('walk_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'walk_forward')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = WalkActionClient()

    action_client.send_goal(10)
    
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
```

## When to Use Each Communication Pattern

### Topics (Publish/Subscribe)
- Sensor data streaming (camera images, joint states, IMU data)
- Continuous monitoring (robot status, battery level)
- Broadcasting information where multiple nodes might be interested

### Services (Request/Response)
- Calibration procedures
- Configuration changes
- Complex computations that return a result
- Tasks that should complete immediately

### Actions (Goal/Feedback/Result)
- Navigation to a specific location
- Complex manipulation tasks
- Humanoid walking or motion sequences
- Any task that takes significant time and may need cancellation

## Practical Exercise

Create a simple action server and client that simulates a humanoid robot moving its arm to a specific position. The action should:
1. Accept a goal with target joint angles
2. Provide feedback during the movement
3. Return the final result when the movement is complete

## Next Steps

Now that we understand all three communication patterns in ROS 2, we'll look at URDF (Unified Robot Description Format) to model our humanoid robot.