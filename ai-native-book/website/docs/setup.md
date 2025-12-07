---
sidebar_position: 3
---

# Setup & Prerequisites

## System Requirements

Before starting with the Physical AI & Humanoid Robotics textbook, ensure your system meets the following requirements:

### Minimum System Specs
- **OS**: Ubuntu 22.04 LTS, Windows 10/11 (WSL2), or macOS 12+
- **RAM**: 16GB (32GB recommended for simulation)
- **Storage**: 50GB free space
- **GPU**: Optional but recommended for complex simulations

### Required Software
- Git
- Python 3.10 or higher
- Node.js 18+ and npm 8+
- Docker and Docker Compose
- ROS 2 Humble Hawksbill (for Module 1)

## Installation Steps

### 1. Clone the Repository
```bash
git clone https://github.com/huminiod-robotics/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Install ROS 2
For Ubuntu 22.04:
```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

For Windows users, install WSL2 with Ubuntu 22.04 and follow the Ubuntu steps inside WSL.

### 3. Set up Docusaurus Development Environment
```bash
cd ai-native-book/website
npm install
```

### 4. Install Python Dependencies
```bash
cd backend
pip install -r requirements.txt
```

### 5. Configure Environment Variables
Create a `.env` file in the backend directory with the following:
```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

### 6. Initialize Qdrant Database
If you don't have a Qdrant instance, you can run one locally with Docker:
```bash
docker run -d --name qdrant-container -p 6333:6333 -p 6334:6334 qdrant/qdrant
```

### 7. Start the Development Server
To start the documentation site:
```bash
cd ai-native-book/website
npm start
```

To start the backend:
```bash
cd backend
python -m app.main
```

## Troubleshooting

### Common Issues

- **ROS 2 Installation**: Ensure your system locale is set to UTF-8 (en_US.UTF-8)
- **Python Virtual Environment**: Use a virtual environment to avoid conflicts
- **Port Conflicts**: The default ports are 3000 (frontend), 8000 (backend), and 6333 (Qdrant)

## Next Steps

After completing the setup, proceed to Module 1: The Robotic Nervous System (ROS 2) to begin your journey into Physical AI.