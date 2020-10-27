***
FAQ
***
* **Is ur_rtde realtime capable?**

The servo, speed and force commands should be realtime capable. I am currently working on
a guide for how to set up a realtime kernel and a ur_rtde realtime communication test, that can be used for
determining if the communication latency is low enough for realtime communication. Once done you can find the guide
here :ref:`Realtime Setup Guide <realtime-setup-guide>`

* **Can I use ur_rtde interface through MATLAB?**

Yes! because ur_rtde have python bindings and MATLAB supports integration with Python,
these can be called directly through MATLAB. Please see the section :ref:`Use with MATLAB <use-with-matlab>`

* **How do I use my Robotiq gripper together with ur_rtde?**

Currently 3 different ways of controlling a Robotiq gripper with ur_rtde exists. Please see the section
:ref:`Use with Robotiq Gripper <use-with-robotiq-gripper>` for more information.

* **Can I pause the motion of the robot?**

You cannot pause the motion of the move, servo and speed commands. However you can stop them. For the servo or speed movements
you can stop the motion by using either servoStop() for a servo command and speedStop() for a speed
command.

For the move commands you need to set the async flag to true, in order to be able to stop them, using either
stopJ() or stopL(). see :ref:`Move Asynchronous Example <move-asynchronous-example>`.

* **Can I set a custom reference frame for the robot, eg. wrt. a feature?**

Currently it has not been implemented in the rtde_control script, but will be in an upcoming release. In the meantime please see
`Move with respect to a custom feature/frame <https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/urscript-move-with-respect-to-a-custom-featureframe-20115/>`_.