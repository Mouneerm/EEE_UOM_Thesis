# EEE_UOM_Thesis
Computer Vision algorithm implemented in a Raspberry Pi to monitor the performance of Public Transport in Mauritius


## Introduction to the problem:

Mauritius has experienced significant economic growth, but it has led to traffic congestion issues. Encouraging public transport use, especially buses, is a potential solution, but residents avoid buses due to the main following reasons:

1 . Slow driving in an attempt to maximise the number of potential passengers.

2 . Safety concerns: accidents sometimes happen due to speeding.

3 . Unpredictable waiting times.

The proposed solution involves an Artificial Intelligence powered camera system to assess bus speed and safety, coupled with a GPS system using the Internet of Things (IoT) to provide real-time bus location information for passengers.


## System Architecture:

The proposed system uses a Raspberry Pi 4 along with sensors to record key bus traveling characteristics in real time, such as speed, GPS coordinates and distance between vehicles in front. If the distance between the bus and the vehicle immediately in front of it is below the braking  distance, the driver will be penalized for dangerous driving while if the distance between both vehicles is too far and the bus is driving slowly for a certain amount of time, the driver will be warned and penalized for slow driving.


![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/a2effaf6-d5a6-4f96-8852-f0f88160429c)


To sum everything up, RPi4 will be used as the heart of the system combined with a camera and a Neo-6m V2 GPS module. Moreover, an easy and intuitive interface is implemented. Hence, the design contains an I2C display with a rotary encoder for the bus driver to select his respective bus route and also an app for time management.


## Overall implementation:

The following block diagram demonstrates how the system works and is 
interconnected:

![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/04373084-a6b6-4068-8e0a-47929abc1534)


The schematics are shown below and note that the red wires represent the power rail, the black wires represent ground, and the blue wires are for communication or GPIO output. Unfortunately, there is not any options on USB connectivity for the RPi4 in Fritzing and the USB camera connection is not shown.

![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/e93df27b-8196-4bea-b331-dfdb0c0aff62)

The overall system flow chart is shown here:

![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/d61e5c7e-8fd5-4570-90ac-fd0de8b7b971)


Key points from the flowchart description:

USD (Upper Bound Stopping Distance) is the upper limit for the stopping distance, indicating that the preceding vehicle is considered too far.
LSD (Lower Bound Stopping Distance) is the lower limit for the stopping distance, signifying that the preceding vehicle is considered too close.
"Distance = None" implies no vehicle is detected in the region of interest.

SSD Algorithm Limitation:

The SSD algorithm can detect traffic signs but cannot read them, potentially leading to misinterpretation of "slow driving" within the speed limit and triggering false "slow" alerts.
Addressing Slow Driving Issue:

To address the slow driving issue, a "normal" state is added, assuming that if the bus is driving slowly within the speed limit, it will not be overtaken.
The "Normal" state becomes false when no vehicle is detected or is too far in front, and the algorithm checks for potential overtaking to trigger the "Slow" state.

Differentiation in Braking Distance:

The algorithm is designed to differentiate between whether the preceding vehicle is slowing down or if the bus is being overtaken based on the range of braking distances.
GPS Module Integration:

A separate process interfaces with the GPS module and sends data to the cloud because the GPS module updates data at 1 second intervals, while the SSD algorithm operates at around 4 FPS.
Implementing both in a single loop would limit the FPS to 1, affecting system performance.
The use of the Python Multiprocessing library creates a separate process, preventing the algorithm from waiting for GPS data and improving overall system performance.

Multiprocessing Benefits:

Multiprocessing does not cause instability in the Global Interpreter Lock, allowing the OS of the RPi4 to utilize another CPU core for the secondary process.
One drawback is increased RAM usage, but this is not a limitation for the prototype.

## Hardware implementation:

To hold everything, a PLA prototype is designed, and 3D printed. The following is the design from Fusion 360:

![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/57dd955d-d740-4018-8f1b-5e5063e1c0e1)

The sensors and other modules are connected based on the schematic shown above.

## Software implementation on Raspberry Pi 4:

The following packages first need to be installed for OpenCV to work on the Rpi4:
sudo apt-get -y install libxvidcore-dev libx264-dev
sudo apt-get -y install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get -y install qt4-dev-tools libatlas-base-dev
sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev

SSD MobileNet-V2 trained with Deep Drive 100K dataset was chosen for high FPS and reasonable accuracy.

## Software implementation App: which shows GPS coordinate of the Public Transport:

The speed and GPS coordinates are successfully collected and the latter is 
being updated in the Firebase server. Moreover, an android app was developed using the Kivy framework in python to 
successfully display the current GPS location of public transports on the chosen bus route.


![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/dea7edb4-6e2a-463d-8b1e-998321e72684)

Synchronising via google Firebase API:



## Results and testing:

Distance and speed estimation in region of interest and the App displaying real-time location of bus obtained via Firebase API:

![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/b859d50b-fc47-4cc6-81de-d53bb7fbe3eb)

“Normal” = False state detected when speed is low while there is no vehicle ahead and picture is uploaded in the appropriate Firebase directory

![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/2cb200a8-a48c-427a-a3bf-15f967c5af30)

“Slow” state detected when speed is low while vehicle has overtaken and picture is uploaded in the appropriate Firebase directory

![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/c8d794b8-a244-4919-a261-e948eaf3460e)


## Future works:
One major issue with the prototype during testing is that it triggers false alarms in curve roads. The following can be done to solve this issue:
1.	Implement a lane detection algorithm.
2.	Add a gyroscope sensor
3.	Manually add GPS coordinates where the algorithm must be paused.
However, adding another AI algorithm to the program is too much for the RPi4 and manually add GPS coordinates is quite a daunting task. Implementing a gyroscope sensor may be the proper solution as it is computationally cheap, it can detect angular acceleration when turning but must be calibrated well so that it must not be sensitive to vibrations.

Another improvement is to make the project more adaptable so that in can be used in any scenario where driving performance needs to be monitored.


