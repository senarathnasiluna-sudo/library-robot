Autonomous Book Sorting Robot in a Library System
This project presents an autonomous robot designed to assist with library management by identifying, sorting and organizing books automatically. It combines robotics, computer vision and artificial intelligence to reduce the need for manual work in libraries and make the sorting process faster and more accurate.
The system was developed and tested in a simulated environment using Webots. It integrates several modules, including book detection, navigation, barcode and color recognition and battery monitoring. Together, these components allow the robot to operate independently and carry out book management tasks with minimal human input.


Project Overview
Libraries often face challenges when organizing and maintaining large collections of books. Manual sorting and cataloging can be slow, repetitive and error prone. This project aims to solve those issues by building an autonomous robot capable of recognizing books, classifying them and placing them in the correct locations.
The robot can:
1.	Detect and identify books using computer vision tools such as YOLO, OpenCV, and ZBar.
2.	Navigate within a library using LiDAR mapping and the A* path planning algorithm.
3.	Sort books into color-coded shelf categories.
4.	Monitor its battery level and automatically return to a charging station when needed.
5.	Check book information and availability from an Excel-based library database.
This project demonstrates how artificial intelligence, perception systems, and robotic control can work together to create a practical automation solution for library management.




System Architecture
The robot’s system is organized into three main parts: perception, processing and actuation.
Perception Module
The perception module allows the robot to see and understand its surroundings. It captures live images through the onboard camera and processes them using a trained YOLO object detection model to locate books. In addition, color detection in the HSV color space and barcode recognition with ZBar provide more precise book identification.
Processing Module
The processing module interprets the sensor data to make decisions. It uses LiDAR sensors to map the environment and applies the A* algorithm to calculate the shortest and safest paths for movement. It also connects to an Excel file through the pandas library to check book details, availability and categories.
Actuation Module
The actuation module handles the robot’s physical actions. It controls the robot arm and gripper using inverse kinematics for accurate book pickup and placement. It also manages the robot’s battery level, ensuring that when power runs low, the robot automatically stops its current task and navigates to the charging dock.

Key Features
Book Detection and Classification
The system uses a YOLOv8 model for real-time book detection. It achieved around 94% accuracy under normal conditions and maintained reliable performance under different lighting situations. Color detection with OpenCV helps the robot identify shelf categories, while barcode scanning with ZBar provides a unique identifier for each book.
Autonomous Navigation
Navigation is managed through LiDAR mapping and A* path planning. The robot builds a 2D map of the library, locates its position, and calculates an optimal route to reach the target shelf while avoiding obstacles. This enables smooth and intelligent movement throughout the environment.


Battery Management
Battery monitoring is built into the system. The robot continuously checks its power level and when it drops below a certain threshold, automatically moves to a predefined charging station. Once the battery is fully charged, it returns to its previous task.
Book Availability Checker
The robot includes a Python-based book management program. It reads data from an Excel sheet using pandas and allows the system to verify whether a particular book exists in the library database. The program displays full details of the book, including its name, ID, category and barcode.
Artificial Intelligence and Decision-Making
The robot combines deep learning and rule-based logic. YOLO provides visual perception, while rule-based control handles energy management and task scheduling. Together, these techniques allow the robot to make real-time decisions and perform its operations without manual control.

Testing and Evaluation
Each module of the system was tested separately and then combined for integrated performance	evaluation.
Below is a summary of the key results.

| Module                | Description                                     | Accuracy                            | Notes                        |
| --------------------- | ----------------------------------------------- | ----------------------------------- | ---------------------------- |
| YOLO Object Detection | Detection under varied lighting and orientation | ~91%                                | Consistent performance       |
| Color Detection       | HSV-based recognition                           | 93% (normal light), 84% (low light) | Stable under most conditions |
| Barcode Recognition   | ZBar decoding                                   | 85%                                 | Errors only in blurry images |
| Battery Monitoring    | Automatic charging and recovery                 | 100%                                | Fully reliable               |
| Navigation            | LiDAR + A* algorithm                            | 80–85%                              | Minor alignment issues       |
| Book Availability     | Excel-based search                              | 100%                                | Accurate and responsive      |

Technologies Used
•	Webots – Simulation environment for testing the robot
•	Python – Main programming language for control and logic
•	YOLOv8 – Deep learning model for object detection
•	OpenCV – Computer vision library for color and image processing
•	ZBar – Library for barcode scanning
•	LiDAR sensors and A* – Mapping and path planning
•	Pandas and Excel – Data management and book availability checking
•	Tiago Robot Model (PAL Robotics) – Simulated robot platform
•	AI Techniques – Deep learning, rule-based decision making, and path planning

System Workflow
1.	The robot initializes and scans the environment using LiDAR and camera sensors.
2.	It detects books through the YOLO model and identifies their color and barcode.
3.	It plans the shortest path to the correct shelf using A* and moves toward it.
4.	The robotic arm picks up or places the book using precise motion control.
5.	The system continuously monitors battery levels during operation.
6.	When the battery is low, the robot navigates to the charging dock.
7.	After recharging, it resumes the interrupted task automatically.


Future Improvements
Several improvements can be made to enhance the robot’s performance and bring it closer to real-world use:
1.	Transfer the system from simulation to a real robot for practical deployment.
2.	Add a more advanced robotic arm for smoother and faster pick-and-place operations.
3.	Use improved machine learning models, such as CNNs or newer YOLO versions, for better detection accuracy.
4.	Implement real-time SLAM (Simultaneous Localization and Mapping) for dynamic mapping in changing environments.
5.	Integrate cloud connectivity for remote monitoring, logging, and data analysis.
6.	Combine all modules under a unified ROS2 framework for better synchronization and scalability.

Why This Project Matters
The Autonomous Book Sorting Robot represents a practical example of how artificial intelligence and robotics can simplify routine human work.
By automating repetitive tasks such as sorting and locating books, this system can significantly reduce human effort, minimize errors and increase operational efficiency in libraries.
Beyond libraries, this concept can be extended to warehouses, offices and other environments where object identification and sorting are essential. It demonstrates the potential of combining AI, computer vision and robotics to build intelligent systems that can think, act and adapt in real-time.

