---
sidebar_position: 3
---

# Planning and Decision Making

## The Decision-Making Pipeline

Planning and decision-making form the cognitive core of AI agents in robotics. The process typically involves:

1. **Situation Assessment**: Understanding the current state of the robot and environment
2. **Goal Formulation**: Defining what the robot should achieve
3. **Plan Generation**: Creating a sequence of actions to reach the goal
4. **Plan Execution**: Carrying out the planned actions
5. **Monitoring and Adaptation**: Adjusting the plan based on feedback

## Hierarchical Planning

Robotic tasks often require multiple levels of planning:

### High-Level (Task) Planning
- Long-term objectives like "visit all rooms in the building"
- Abstract actions that may be refined later
- Consideration of resource constraints

### Mid-Level (Motion) Planning
- Path planning through configuration space
- Avoiding obstacles in the environment
- Generating smooth trajectories

### Low-Level (Control) Planning
- Joint-level commands
- Balance and stability maintenance
- Real-time feedback control

## AI-Based Planning Approaches

### 1. Large Language Model (LLM) Planning

LLMs can generate high-level plans and reasoning:

```python
import asyncio
from openai import AsyncOpenAI
import json

class LLMPlanner:
    def __init__(self):
        self.client = AsyncOpenAI()
        self.system_prompt = """
        You are a helpful assistant that creates plans for humanoid robots.
        Given a goal and current situation, create a plan consisting of specific actions.
        Respond with ONLY a JSON object with this structure:
        {
          "plan": [
            {
              "action": "action_name",
              "parameters": {"param1": "value1", ...},
              "reasoning": "Why this action is appropriate"
            }
          ],
          "confidence": "high|medium|low"
        }
        
        Actions available:
        - navigate_to: Move robot to location
        - pick_up: Pick up an object
        - place: Place an object at location
        - ask_for_help: Request assistance from human
        - wait: Wait for specified duration
        """
    
    async def create_plan(self, goal: str, current_state: dict):
        """Generate a plan using LLM"""
        user_prompt = f"""
        Goal: {goal}
        Current State: {json.dumps(current_state, indent=2)}
        """
        
        response = await self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.3
        )
        
        try:
            plan_data = json.loads(response.choices[0].message.content)
            return plan_data
        except json.JSONDecodeError:
            # Handle case where LLM didn't return valid JSON
            return {
                "plan": [{"action": "wait", "parameters": {"duration": 1.0}, "reasoning": "Could not parse plan"}],
                "confidence": "low"
            }
```

### 2. Integration with Traditional Planners

Combine AI planning with traditional robotics planners:

```python
class HybridPlanner:
    def __init__(self):
        self.llm_planner = LLMPlanner()
        self.motion_planner = MotionPlanner()  # Traditional path planner
        self.action_executor = ActionExecutor()  # ROS action clients
    
    async def create_comprehensive_plan(self, goal: str, robot_state: dict):
        """Create a plan combining AI reasoning with traditional motion planning"""
        # Get high-level plan from LLM
        high_level_plan = await self.llm_planner.create_plan(goal, robot_state)
        
        # Convert high-level actions to executable commands
        executable_plan = []
        for action in high_level_plan['plan']:
            if action['action'] == 'navigate_to':
                # Use traditional motion planner to create detailed trajectory
                trajectory = self.motion_planner.plan_path(
                    current_pose=robot_state['pose'],
                    target_pose=action['parameters']['target']
                )
                
                # Add detailed motion commands to executable plan
                for step in trajectory:
                    executable_plan.append({
                        "type": "follow_trajectory",
                        "trajectory": step,
                        "reasoning": action['reasoning']
                    })
            else:
                # Directly add other actions that don't require detailed planning
                executable_plan.append(action)
        
        return executable_plan
```

## Decision Making Under Uncertainty

Robots operate in uncertain environments, requiring probabilistic decision-making:

```python
from dataclasses import dataclass
from typing import List
import numpy as np

@dataclass
class ActionOption:
    name: str
    parameters: dict
    expected_outcome: str
    success_probability: float
    execution_time: float  # in seconds
    cost: float  # energy, time, risk

class ProbabilisticDecisionMaker:
    def __init__(self):
        self.current_belief_state = {}
        self.utility_weights = {
            "success": 1.0,
            "time": 0.5,
            "energy": 0.3,
            "risk": 2.0
        }
    
    def calculate_expected_utility(self, action: ActionOption) -> float:
        """Calculate expected utility of an action"""
        # Success utility scaled by probability
        success_utility = self.utility_weights["success"] * action.success_probability
        
        # Penalty for time and energy
        time_penalty = self.utility_weights["time"] * action.execution_time
        energy_penalty = self.utility_weights["energy"] * action.cost
        
        # Risk component
        risk_penalty = self.utility_weights["risk"] * (1 - action.success_probability)
        
        expected_utility = success_utility - time_penalty - energy_penalty - risk_penalty
        return expected_utility
    
    def select_best_action(self, action_options: List[ActionOption]) -> ActionOption:
        """Select action with highest expected utility"""
        utilities = [self.calculate_expected_utility(action) for action in action_options]
        best_idx = np.argmax(utilities)
        return action_options[best_idx]
```

## Reactive vs. Deliberative Planning

### Reactive Systems
- Respond immediately to environmental changes
- Good for real-time control and safety
- Limited to predefined responses

### Deliberative Systems
- Consider future states and consequences
- Better for complex multi-step tasks
- More computationally expensive

### Hybrid Approach
```python
class HybridDecisionSystem:
    def __init__(self):
        self.reactive_layer = ReactiveLayer()  # For immediate responses
        self.deliberative_layer = DeliberativeLayer()  # For planning
        self.monitor = EnvironmentMonitor()
    
    def decide_action(self, goal: str, state: dict):
        # Check for immediate threats (reactive)
        emergency_action = self.reactive_layer.check_emergency(state)
        if emergency_action:
            return emergency_action
        
        # Otherwise, use deliberative planning
        return self.deliberative_layer.plan_action(goal, state)
    
    def monitor_and_adapt(self):
        """Continuously monitor environment and adapt plan"""
        current_state = self.monitor.get_state()
        
        # Check if current plan is still appropriate
        if not self.deliberative_layer.plan_is_valid(current_state):
            # Recalculate plan
            new_plan = self.deliberative_layer.replan()
            return new_plan
        
        return None  # Continue with current plan
```

## Planning with Memory Integration

Use robot's memories to improve planning:

```python
class MemoryEnhancedPlanner:
    def __init__(self, memory_system):
        self.memory_system = memory_system
        self.llm_planner = LLMPlanner()
    
    async def create_contextual_plan(self, goal: str, state: dict):
        """Create a plan that incorporates past experiences"""
        # Retrieve relevant memories
        relevant_memories = await self.memory_system.retrieve_memories(
            f"similar goal: {goal} in similar state: {state}",
            limit=5
        )
        
        # Include past experiences in the context
        context = {
            "current_state": state,
            "goal": goal,
            "past_experiences": [mem["content"] for mem in relevant_memories]
        }
        
        # Generate plan with historical context
        plan = await self.llm_planner.create_plan(goal, context)
        
        # Evaluate if past experiences suggest modifications
        for memory in relevant_memories:
            if memory["metadata"].get("outcome") == "failure":
                # Adjust plan based on past failures
                plan = self._adapt_plan_for_known_issues(plan, memory)
        
        return plan
    
    def _adapt_plan_for_known_issues(self, plan, failure_memory):
        """Modify plan based on past failures"""
        # Implementation would adjust plan based on failure patterns
        # For example, if a certain approach failed before, try an alternative
        return plan
```

## Implementation in ROS

Create a ROS node that integrates planning and decision-making:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
from your_interfaces.action import NavigateToPose, ManipulateObject

class PlanningDecisionNode(Node):
    def __init__(self):
        super().__init__('planning_decision_node')
        
        # Publishers and subscribers
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, 'robot_pose', self.pose_callback, 10
        )
        
        # Action clients for executing plans
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manip_client = ActionClient(self, ManipulateObject, 'manipulate_object')
        
        # Initialize planning components
        self.planner = HybridPlanner()
        self.decision_maker = ProbabilisticDecisionMaker()
        
        # Goal handling
        self.current_goal = None
        self.current_plan = []
        
        # Planning timer
        self.planning_timer = self.create_timer(1.0, self.plan_callback)
    
    def pose_callback(self, msg):
        """Update robot pose for planning"""
        self.current_pose = msg
    
    def plan_callback(self):
        """Main planning loop"""
        if not self.current_goal:
            return
        
        # Get current state
        current_state = {
            "pose": self.current_pose,
            "battery": self.get_battery_level(),
            "task_queue": self.get_active_tasks()
        }
        
        # Generate or update plan
        if not self.current_plan or self.should_replan(current_state):
            future_plan = self.planner.create_comprehensive_plan(
                self.current_goal, current_state
            )
            # Handle the async future appropriately
            # self.current_plan = future_plan.result()
    
    def should_replan(self, state):
        """Determine if replanning is needed"""
        # Check for environmental changes, new goals, plan failure, etc.
        return False  # Implementation would check various conditions
    
    async def execute_plan_step(self, action):
        """Execute a single step of the plan"""
        if action["type"] == "navigate":
            # Execute navigation action
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = action["pose"]
            
            self.nav_client.wait_for_server()
            future = self.nav_client.send_goal_async(goal_msg)
            # Add feedback handling
        elif action["type"] == "manipulate":
            # Execute manipulation action
            goal_msg = ManipulateObject.Goal()
            goal_msg.object_id = action["object_id"]
            goal_msg.action = action["manipulation_type"]
            
            self.manip_client.wait_for_server()
            future = self.manip_client.send_goal_async(goal_msg)
            # Add feedback handling

def main(args=None):
    rclpy.init(args=args)
    node = PlanningDecisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety and Ethics in Decision Making

### Fail-Safe Mechanisms
- Always have a safe default action
- Implement timeouts on all operations
- Verify action safety before execution

### Ethical Decision Making
- Prioritize human safety above all goals
- Consider privacy and consent
- Respect human autonomy

## Practical Exercise

1. Implement an LLM-based planner for a simple navigation task
2. Integrate it with a traditional path planner
3. Add memory-based learning to improve future decisions
4. Create a ROS node that executes the plan
5. Test the system in simulation with various scenarios

## Next Steps

With planning and decision-making established, we'll explore how AI agents process sensor data for perception in the next section.