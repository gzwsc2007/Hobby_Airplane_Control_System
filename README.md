[Project website on Blogger](http://ansonproj.blogspot.com/2015/06/hobby-airplane-control-system-hacs.html)

The Hobby Airplane Control System (HACS) is a personal project about developing a powerful embedded control system for hobby airplanes. This system should be able to gather critical flight data, and send them back to a ground station client, which display these information to the pilot in an intuitive way.

In addition to flight data acquisition, HACS should also be able to stablize and ultimately autonomously control the airplane under pilot instruction. To develope a feedback controller, I will first need to learn the mathematical model of the airplane. This will be accomplished by conducting flight experiment, and then use the obtained data to perform grey-box system identification.

Currently I am still developing both the hardware and software platform for the system. I have designed a shield PCB for the STM32F4 nucleo board, and am currently writing drivers for the various sensors to be used.
