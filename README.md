# os-ev3-project


ROBOT NAME: Big Chungus

## **Description of the architecture of the robot**

The robot we are presenting is based on the Lego MINDSTORMS EV3 Education kit. Here, many sensors and actuators are presented, in order to communicate efficiently with an ARM9 microcontroller. Following the guidelines of the project, we know that not all of them may be used in the same, but this number is limited to four sensors and up to four engines. This restriction is not only given by the project but even from the microcontroller bricks, which can host a maximum of four input ports and four output ports allowing up to four NXT bricks to be connected.

Given these instructions, we selected three sensors from the kit, which are the following;

 - An ultrasonic sensor
 - A gyroscope sensor
 - A touch sensor

During the first tests, we have chosen to include even the compass sensor, which could have been useful to help the robot to navigate correctly. This was giving a substantial contribution in giving a correct reference for the straightness of the robot.

Then, when we tested it in the official EURECOM arena, we saw it behaving in a strange way such as losing the correspondence of the starting position. After some failed tests, we understood that the ARENA is built using some components such as iron, which disrupt the signal of the sensor, causing random generation of data. In order to prove it, we tried to build a structure that was lifting the component, where the interference from the arena should have been way lower. The results were quite positive but not good enough in our opinion, so we finally decided to drop this option out of our selected sensors.

Regarding the motors, our robot is moving thanks to two big servomotors, linked by an axle, which is used to stabilize the movement of the machine. Lastly, we used the middle-size servomotor in order to drop the obstacles during the competition. To sum up, we are using three motors;

 - Two big servomotors
 - One middle size servomotor

Finally, to increase the stabilization of the robot, some LEGO components comprehended in the kit provided, have been used, such as the rotatory ball for compensating the backward traction and some pieces to decrease the likelihood of overturning.

Pictures of our robots are provided from the following links;

[Front view](https://www.dropbox.com/s/ojfkgyokqhrmki3/20220118_164013.jpg?dl=0)

[Lateral view - Left](https://www.dropbox.com/s/fed29vdfzdbb8pn/20220118_164036.jpg?dl=0)

[Lateral view - Right](https://www.dropbox.com/s/sm1opu0qdoxzp8i/20220118_164051.jpg?dl=0)

[Back view](https://www.dropbox.com/s/4uyfd5a9hzxng9s/20220118_164104.jpg?dl=0)

## **Algorithms**


The algorithm our robot is using in order to compete in the EURECOM arena could be divided into different machine states. Each state makes several checks and steps in order to make a change state.

The main flows are based on the structure of the arena, following the straights and the different turns. When we were in the design phase of the project we found a metaphor that could apply in our case. We believe that our robot acts as a pilot who has studied in deep the circuit and is ready to race in an international competition against others. 

Our robot knows exactly the path of the circuit since it is not changing dynamically. It was unnecessary to make it learn the path to follow during the race. So the approach we used to implement the algorithm for our robot is based on states. Each state, except for the handling errors ones, represents a circuit turn or straight.

The basic flow, supposing that we are in the best case so. there are no obstacles, no rivals, no issue with the data taken in input by the sensors, no malfunction with the motors, and no inertia, it is very linear.
The stages the robot has to complete is, starting from the state start, are a series of 'forward states', from 1 to 6, which define the number of centimeters the robot is supposed to run (set them up during design time). Furthermore, some state machine cases take into consideration only some of the sensors available. For instance, in some of the parts of the arena, we decided not to check the proximity from the robot to the next obstacles, taking into account some suppositions which were verified as true during the testing phase. This decision saved us a lot of work that could have been too specific for some extreme cases, overfitting our robot model.

Some other machine states are designed to handle specific cases which were likely to happen to take into consideration a lot of variables that can disrupt the robot from the original behavior, such as the inertia of the motors. In these cases, what we can actually do is correct the error(s) made by the sensors and motors. **For instance, we know from reference [1] and [2] that the gyroscope accumulates error each time it is [...].** Since we are making a lot of operations, and we want the robot responsiveness to below, we have to take into consideration a high margin of error.  For this reason, we designed three machine states which can correct these mistakes made by the sensors;

 -  Proximity correction
 -  Angle correction
 -  Proximity correction before angle correction

Starting from the top, proximity correction is a state machine when the range settled in design mode is not respected by the robot based on the ultrasonic sensor's value. If the proximity is greater to the object than expected, it means we have to get closer, otherwise, we have to get further back. To do that, either we calculate a dynamic value based on the distance max and the proximity, or we use fixed distance.  Testing it, we observed that the second case is more precise, so we assume that is the best option. When we say that is it more precise, we mean that the is less likely for the robot to hit an object
before realizing that the proximity condition is not valid anymo


Another problem we faced during the design and testing phase, as mentioned before, is the error accumulated by the gyro. This is the sensor we use in order to calibrate the robot and to set the 'zero angle', so the angle of the start that we suppose is the only one we can trust, giving the continuous movement of the robot, that does not allow the sensor to be accurate (reference, how much time does it need to give a trustable value?). In this case, we analyze the same issue we faced with the proximity correction, but regarding the angle. The algorithm is found in this state machine whenever the angle data provided by the gyroscope is not comprehended in the range settled in design mode. To be as precise as possible, we subdivided the solving method into cycles. At every cycle, until the angle is not in the range we want it to be, we rotate the robot for a specific angle, given by (expected_angle - gyro_angle) / 2.

Lastly, in this report, we would like to present a more specific case wherever the robot is stuck in front of one object whatever it is. In this case, the first thing we want to do is get further from the obstacle with the inner while iteration. 

All the three methods presented, at the end of their correction, move the algorithm to the next state machine specified as a parameter from the parent method. For more information regarding the implementation of the state machine, we invite you to take a look at the code and the comments added.


## **Source code and instructions**

## **Videos / pictures**

## **Contributions**
