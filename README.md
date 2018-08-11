# Description

Automatic and manual control of a ping pong ball on a curved truss mounted on a servo stand. The ball position is captured using
a camera and fedback to the controller responsible for tracking the reference input.

## Mode Control

Can be achieved by declaring the appropriate pre-processor constant value in the main file.

## Compute platform

The hardware used is an arduino uno and with a joystick for command input whose underlying implementation is a 2 axis potentiometer. The position acquitison is done using a Pixy camera and the tracking is setup using the accompanying PixyMon software.

### [Manual mode video](https://www.youtube.com/watch?v=Gwn9X6mq8wU)
### [Automatic mode video](https://www.youtube.com/watch?v=DCVG3v2UdTw)
