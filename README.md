# Automated Mobile Robot with Object Detection

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.7+](https://img.shields.io/badge/Python-3.7%2B-blue)](https://www.python.org/)
[![GitHub Stars](https://img.shields.io/github/stars/Roshan20222/Automated-Mobile-Robot-Object-Detection?style=flat-square)](https://github.com/Roshan20222/Automated-Mobile-Robot-Object-Detection/stargazers)

## Overview

This project implements an **Object Detection Based Automated Mobile Robot** with a **4-DOF Robotic Arm** for autonomous pick and place operations. The system leverages computer vision algorithms (YOLO v5/v8) and machine learning to identify, locate, and manipulate objects in real-time without human intervention.

### Key Features

- **Real-time Object Detection**: Using YOLOv5/v8 pre-trained CNN models
- **Autonomous Navigation**: Dijkstra's algorithm for optimal path planning
- **Robotic Arm Control**: 4-DOF robotic arm with precise inverse kinematics
- **Color-based Sorting**: Detection and sorting of objects by color
- **Distance Measurement**: Ultrasonic sensors for accurate object positioning
- **Live Video Processing**: Real-time image processing and decision making

## System Architecture

```
┌─────────────────────────────────────────┐
│      Raspberry Pi (Main Controller)     │
│  - Image Processing                     │
│  - Object Detection (YOLO)              │
│  - Path Planning (Dijkstra)             │
│  - Motor Control Signals                │
└─────────────────────────────────────────┘
         │           │           │
    ┌────┴────┐  ┌──┴──┐  ┌────┴────┐
    │          │  │      │  │         │
   Camera    Motor Servo  Ultrasonic
   Module    Drivers Motors Sensors
    │          │  │      │  │
    └─────┬────┴──┴──────┴──┘
          │
    Mobile Robot Vehicle
    with Robotic Arm
```

## Hardware Components

| Component | Model/Specification | Purpose |
|-----------|---------------------|----------|
| Microcontroller | Raspberry Pi 4 (4GB RAM) | Main processing unit |
| Microcontroller | Arduino Mega 2560 | Motor & sensor control |
| Motors (DC) | 4x DC Motors | Vehicle wheel actuation |
| Motors (Servo) | 4x MG90S Servo Motors | Robotic arm joints |
| Motor Drivers | L298N | DC motor speed/direction control |
| Camera | Raspberry Pi Camera Module (5MP) + USB Webcam | Visual input |
| Distance Sensor | HC-SR04 Ultrasonic Sensor | Object distance measurement |
| Battery | LiPo Battery (11.1V) | Power supply |
| Power Regulator | Buck Converter | 5V voltage regulation |
| Frame | 3D Printed Acrylic | Mechanical structure |

## Software Components

### Languages & Frameworks
- **Python 3.7+**: Main programming language
- **YOLO v5/v8**: Object detection framework
- **OpenCV**: Image processing library
- **Arduino IDE**: Firmware development

### Key Libraries
```python
- ultralytics (YOLO)
- opencv-python
- numpy
- scipy
- serial (PySerial)
```

## Project Structure

```
Automated-Mobile-Robot-Object-Detection/
├── README.md
├── LICENSE
├── requirements.txt
├── .gitignore
├─┠─ src/
│  ├── main.py                 # Main robot control script
│  ├── object_detection.py     # YOLO detection module
│  ├─┠─ path_planning.py       # Dijkstra's algorithm implementation
│  ├── robotic_arm.py         # Inverse kinematics & arm control
│  ├── motor_control.py       # DC motor control via L298N
│  ├── sensor_fusion.py       # Ultrasonic & camera sensor integration
│  ├── image_processing.py    # Color detection & preprocessing
│  ├── communication.py       # Serial communication with Arduino
│  ├┠─ utils.py               # Utility functions
├── models/
│  ├┠─ yolov5_trained.pt      # Trained YOLO model weights
├── data/
│  ├── dataset/               # Training dataset
│  ├── annotations/           # CVAT annotations
│  ├┠─ test_results/          # Test outputs & logs
├── config/
│  ├── yolo_config.yaml       # YOLO configuration
│  ├┠─ robot_params.yaml      # Robot calibration parameters
├── firmware/
│  ├┠─ arduino_sketch.ino     # Arduino motor control firmware
├── docs/
│  ├── INSTALLATION.md        # Setup instructions
│  ├── USAGE.md               # Usage guide
│  ├── HARDWARE_SETUP.md      # Hardware assembly guide
│  ├┠─ TRAINING.md            # Model training instructions
├── tests/
│  ├── test_detection.py
│  ├── test_kinematics.py
│  ├┠─ test_movement.py
├┠─ requirements.txt
```

## Installation & Setup

### Prerequisites
- Raspberry Pi 4 with Raspbian OS (32-bit or 64-bit)
- Python 3.7 or higher
- Arduino IDE (for firmware upload)
- Git

### Step 1: Clone the Repository
```bash
git clone https://github.com/Roshan20222/Automated-Mobile-Robot-Object-Detection.git
cd Automated-Mobile-Robot-Object-Detection
```

### Step 2: Install Dependencies
```bash
pip install -r requirements.txt
```

### Step 3: Hardware Setup
Refer to `docs/HARDWARE_SETUP.md` for detailed hardware assembly instructions.

### Step 4: Configure Parameters
Edit `config/robot_params.yaml` with your robot's calibration parameters:
- Motor PWM pins
- Servo angles
- Camera resolution
- YOLO model path

### Step 5: Run the Main Script
```bash
python src/main.py
```

## Algorithm Overview

### 1. Object Detection
- Captures frames from camera/webcam
- Pre-processes images (resize, normalize)
- Uses YOLO v5/v8 for real-time detection
- Returns bounding boxes and confidence scores

### 2. Path Planning
- Converts camera frame to grid representation
- Identifies robot and object positions
- Applies Dijkstra's algorithm for optimal path
- Generates waypoints for navigation

### 3. Robot Navigation
- Follows planned path using motor control
- Avoids obstacles using ultrasonic sensors
- Adjusts trajectory based on visual feedback

### 4. Robotic Arm Control
- Calculates inverse kinematics for target pose
- Controls 4 servo motors for arm positioning
- Implements gripper open/close logic
- Executes pick and place operations

## Model Training

To train a custom YOLO model on your dataset:

```bash
cd docs
cat TRAINING.md
```

Quick training example:
```python
from ultralytics import YOLO

# Load a pretrained model
model = YOLO('yolov5s.pt')

# Train the model
results = model.train(data='dataset.yaml', epochs=50, imgsz=640)
```

## Performance Metrics

- **Detection Accuracy**: 92%+ (on test dataset)
- **Frame Rate**: 15-20 FPS (Raspberry Pi 4)
- **Path Planning Time**: < 200ms
- **Arm Positioning Accuracy**: ±2mm
- **Robot Speed**: 0.2 m/s

## Usage Examples

### Basic Operation
```python
from src.robot_controller import RobotController

robot = RobotController(config_path='config/robot_params.yaml')
robot.start()
robot.pick_and_place()
robot.stop()
```

### Custom Object Detection
```python
from src.object_detection import ObjectDetector

detector = ObjectDetector(model_path='models/yolov5_trained.pt')
frame = detector.capture_frame()
detections = detector.detect(frame)
```

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit changes (`git commit -m 'Add improvement'`)
4. Push to branch (`git push origin feature/improvement`)
5. Open a Pull Request

## Results & Analysis

This project successfully demonstrates:
- Real-time object detection using deep learning
- Autonomous robot navigation and control
- Precise robotic arm manipulation
- Integration of multiple hardware components
- Computer vision-based decision making

See `data/test_results/` for detailed performance graphs and test outputs.

## Known Limitations
- Limited by Raspberry Pi processing power
- Requires well-lit environment for optimal detection
- Arm reach limited to ~45cm radius
- No real-time mapping (SLAM) implementation

## Future Enhancements
- [ ] Integration of stereo vision for better depth perception
- [ ] LiDAR-based SLAM for autonomous exploration
- [ ] Machine learning-based gripper pressure control
- [ ] Mobile app for remote robot control
- [ ] Multi-robot coordination
- [ ] ROS (Robot Operating System) integration

## Troubleshooting

### Issue: Low Detection Accuracy
**Solution**: Retrain model with more diverse dataset or adjust YOLO confidence threshold

### Issue: Motor Not Responding
**Solution**: Check serial communication, verify Arduino firmware upload, test GPIO pins

### Issue: Camera Feed Lag
**Solution**: Reduce image resolution, disable preview, optimize YOLO model size

For more troubleshooting, see `docs/INSTALLATION.md`

## References

1. Redmon, J., & Farhadi, A. (2018). YOLOv3: An Incremental Improvement. arXiv preprint arXiv:1804.02767.
2. Corke, P., et al. (2013). Computer Vision-based Object Recognition and Grasping for Autonomous Robotic Manipulation. IEEE Transactions on Robotics.
3. Dijkstra, E. W. (1959). A note on two problems in connexion with graphs. Numerische Mathematik.

## Author

**Roshan Pandey**
- GitHub: [@Roshan20222](https://github.com/Roshan20222)
- Email: pandeyroshan2021@outlook.com
- Institution: Tribhuvan University, Kathmandu Engineering College

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

Special thanks to:
- Project Supervisors: Er. Anmol Ratna Bajracharya
- Department of Electronics, Communication and Information Engineering
- Tribhuvan University, Kathmandu Engineering College

## Citation

If you use this project in your research, please cite:
```bibtex
@project{roshan2024autonomousrobot,
  title={Automated Mobile Robot with Object Detection Based Autonomous Pick and Place System},
  author={Pandey, Roshan},
  year={2025},
  url={https://github.com/Roshan20222/Automated-Mobile-Robot-Object-Detection}
}
```

---

**Last Updated**: November 2025
**Status**: Active Development
