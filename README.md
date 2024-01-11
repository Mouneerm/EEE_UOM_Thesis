# EEE_UOM_Thesis
Computer Vision algorithm implemented in a Raspberry Pi to monitor the performance of Public Transport in Mauritius


## Introduction to the problem:

Mauritius has experienced significant economic growth, but it has led to traffic congestion issues. The transport system lags due to an excessive number of vehicles, inefficient public transport, poor road traffic management, population growth, poor roadway conditions, economic development, urbanization, and unforeseen circumstances. Peak hours exacerbate the problem. Encouraging public transport use, especially buses, is a potential solution, but residents avoid buses due to the main following reasons:

1 . Slow driving in an attempt to maximise the number of potential passengers.

2 . Safety concerns; accidents sometimes happen due to speeding

3 . Unpredictable waiting times.

The proposed solution involves an Artificial Intelligence-powered camera system to assess bus speed and safety, coupled with a GPS system using the Internet of Things (IoT) to provide real-time bus location information for passengers.

## System Architecture:

To solve the problems mentioned above, the propose system uses a single-board computer (SBC) or microcontroller along with sensors to record key bus traveling characteristics in realtime, such as speed, GPS coordinates and distance between vehicles in front. If the distance between the bus and the vehicle immediately in front of it is below the braking  distance, the driver will be penalized for dangerous driving while if the distance between both vehicles is too far and the bus is driving slowly for a certain amount of time, the driver will be warned and penalized for slow driving.


![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/a2effaf6-d5a6-4f96-8852-f0f88160429c)


To sum everything up, RPi4 will be used as the heart of the system combined with a camera and a Neo-6m V2 GPS module. Moreover, an easy and intuitive interface is implemented. Hence, the design contains an I2C display with a rotary encoder for the bus driver to select his respective bus route and also an app for time management.

## Overall implementation:

The following block diagram demonstrates how the overall system works and is 
interconnected:

![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/04373084-a6b6-4068-8e0a-47929abc1534)


The schematics are shown below and note that the red wires represent the power rail, the black wires represent ground and the blue wires are for communication or GPIO output. Unfortunately, there is not any options on USB connectivity for the RPi4 in Fritzing and the USB camera connection is not shown.

![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/e93df27b-8196-4bea-b331-dfdb0c0aff62)

The overall system flow chart is shown here:

![image](https://github.com/Mouneerm/EEE_UOM_Thesis/assets/45911394/d61e5c7e-8fd5-4570-90ac-fd0de8b7b971)

The following should be noted from the flowchart:
•	USD is the upper bound stopping distance, above which the preceding vehicle is considered too far
•	LSD is the lower bound stopping distance below which the preceding vehicle is considered too close.
•	“Distance = None” means that no vehicle is detected in the region of interest.

The SSD algorithm can detect traffic signs but cannot unfortunately read them. Hence the program can misinterpret “slow driving” when driving within the speed limit and trigger false “slow” alert. To solve this issue, a “normal” state is added if the bus is driving slowly because of the speed limit, it is being assumed that it will not be overtaken. On the contrary, if the bus is driving purposely slowly, it will be overtaken sooner or later. The “Normal” state is False when no vehicle is detected or is too far in front of the bus. At this state, the algorithm keeps checking if suddenly the bus is being overtaken and it will trigger the “Slow” state. Moreover, the algorithm is designed to differentiate between whether the preceding vehicle is slowing down or whether the bus is overtaken by different range of braking distance. 
It can be seen that a separate process interfaces with the GPS module and sending the data to the cloud. This is because the GPS module takes one second to update the data sequence while the SSD algorithm operates at around 4 FPS. If both are implemented in a single loop sequence, the algorithm waits for the GPS to receive data and the FPS would be limited to 1 FPS hence, restricting the system’s performance. The separate process was created by using the built-in Python Multiprocessing library. Moreover, multi-processing does not cause instability in the Global Interpreter Lock as for example, the OS of the RPi4 can utilize another CPU core for the secondary process. However, one main drawback is that more RAM is used which fortunately is not a limitation for the prototype.

