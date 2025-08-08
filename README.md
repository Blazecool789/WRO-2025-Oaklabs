# WRO-2025-Oaklabs
Future Engineers 2025 Oaklabs Github Repository

Our team, Oaklabs, designed a robot for the WRO 2025 Future Engineers competition that is both simple and advanced. We wanted it to be fast, reliable, and able to handle different challenges on the field. Every part of our robot was chosen carefully so it works well in competition.

We use two main controllers: an Arduino Uno and a Raspberry Pi 5. The Arduino controls motors and sensors that need quick reactions, while the Raspberry Pi handles bigger tasks like image processing and decision-making. This way, both controllers share the work and avoid delays.

The Arduino runs the movement system using a brushed DC motor and a high-torque servo for turning. It also has two ultrasonic sensors to measure distances from the field’s edges, so the robot can move without hitting boundaries.

The Raspberry Pi has a wide-angle camera and another ultrasonic sensor for backup. The camera can detect colors, blocks, and line patterns using Python and OpenCV. This helps the robot make smart choices based on what it sees.

The Arduino and Raspberry Pi communicate through a fast wired serial link. For example, if the Pi sees a red block within 5 cm, it sends “R” to the Arduino, which turns the servo right. If it’s green, it sends “L” for a left turn.

In the open challenge phase, the robot follows line colors instead of avoiding obstacles. Blue–orange means “turn clockwise” and orange–blue means “turn counterclockwise.”

For the final parking task, the Pi counts blue and orange lines. When it reaches 12 of each, it waits for a pink mark. When found, it sends “P” to the Arduino to run the parking sequence.

Our design uses teamwork between sensors, motors, and controllers. By combining fast reactions with smart decision-making, our robot can adapt to different situations and perform each task accurately. We believe this will give Oaklabs a strong chance to succeed in WRO 2025.
