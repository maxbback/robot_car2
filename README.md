# robot_car2
Multi sensor autodriving car

Current key objectives:
- Avoid objects by moving forward and turning around (sensors is only in front and on the sides)
- Variable speed, slow down when closer to objects
status: meet

Next key level of objective:
- Follow the walls
- Avoid getting trapped, one sugestion is to reverse movements if we are stuck

Comming objectives
- attach camera, use it for recognition
- map room

Core components:
2 Motors + battery pack for motors
Motor controller board
Raspberry pi
3 HC-SR04 ultrasonci sensors
My starting kit was a camjam/edukit complemented with 3 HC-SR04 and labboard
You also need a bunch of cabels

To handle this I am using two nodes
- A distance dection node, wich is a bit of the smart part identifying where to to go and keeping track of distnace
-- It is populating following event topics
--- distance the closest distance to an obstacle
--- direction a proposed direction based on the closest obstacle
--- frontdistance (obviuous)
--- leftdistance (obvious)
--- rightdistance (obvious)
- Second node is autodrive that controlles the motors and subscribes to distance and direction topics
-- Distance information is used to controle the speed
-- Direction is the guidance for which direction to drive to

Current improvement areas is
- When direction is not forward use distance to optimize where to go, try turn and if forward distance is ok go forward a litle
- But in general a  smarter algorithm for movement is needed
Click tp see the video when it tries to avoid crashing
[![Watch the video](image_car2/robot_car2_front.jpeg)](https://youtu.be/d5yBfNilb-g)
Below is some good pictures
[![Watch the video](image_car2/robot_car2_front.jpeg)](image_car2/robot_car2_front.jpeg)
[![Watch the video](image_car2/IMG_2048.jpeg)](image_car2/IMG_2048.jpeg)
[![Watch the video](image_car2/IMG_2049.jpeg)](image_car2/IMG_2049.jpeg)
[![Watch the video](image_car2/IMG_2050.jpeg)](image_car2/IMG_2050.jpeg)
[![Watch the video](https://youtu.be/d5yBfNilb-g)](https://youtu.be/d5yBfNilb-g)


