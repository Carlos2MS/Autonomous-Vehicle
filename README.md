# Autonomous-Vehicle
Self-driving vehicle that can navigate complex tracks while maximizing efficiency. This project will be put to the test in a variety of challenges to demonstrate its many capabilities. The robot, outfitted with sensors and programmed with an Arduino should be able to recognize and follow directions, start and stop through wireless communication, be able to sense obstacles in its path, and so on.

Equipment:
1. Arduino UNO R4 Wifi
2. Ultrasensor
3. Tww IR sensors
4. Two DC motors
5. Huskey Camera
6. Baterry Power source
7. Two Wheel Encoders

Challenges:

Bronze:
In summary: Traverse the track, don’t crash into any obstacles, report events wirelessly.​
Details:
The track will consist of a line forming a loop with a total track length (perimeter) of at least 3m. 
The track itself is a line on a background of contrasting colour (such that it the line can be detected by the IR sensors). A light line on a dark background will be used (if you are building a section of track for testing at home then masking tape on dark card as a background would be suitable). In the weeks leading up to the demo a track will be avilable in the labs.
The track will form a loop (see image below; the track may not look precisely like this but it will approximate the shape, and in particular you will include at least two right-angle turns). The total length will be at least 3m.
The PC control program must:
Provide the user with start and stop buttons that can be used to begin and end the buggy's run on the track.
Provide an output area that displays telemetry received from the buggy during the run. 
The buggy must
Start the run on receiving a GO command via WiFi  from the controlling PC
Stop the run on receiving a STOP command via WiFi from the controlling PC
Traverse the track twice without derailing, using the IR sensors to follow the line of the track
Pause for obstacles as detected by the US rangefinder. The stopping distance is up to you (but about 10cm is reasonable).
Report to the controlling PC when obstacles are detected and cleared (a simple "obstacle seen" message is sufficient, but you may choose to do something more details, e.g. "stopping for obstacle at 5cm distance") and (periodically) an estimate of distance travelled calculated via the wheel encoder. The reporting does not have to display within the Processing graphics window (you can use the console).

Silver:
In addition to the Bronze challenge requirements , in the silver challenge:  
The buggy still traverses the line track from the Bronze challenge. However, its speed is to be controlled through a PID controller. Two modes of speed control are to be implemented: (1) via entering the reference speed into the GUI (should be possible to update at any time during the demo) and (2) via reference object placed in sight of the ultrasonic distance sensor. For the second mode, the buggy’s control strategy should be to keep a constant distance from the object (15 cm). The object is moving forward at varying speeds (no backward motion, so the buggy does not need to reverse at any point). The telemetry reported back to the GUI should contain the distance of the object, the current mode of control, the current reference speed, and the current speed of the buggy.

Gold:
In addition to the Bronze challenge and Silver challenge requirements, in the gold challenge is a creative section using the Huskey camara.
