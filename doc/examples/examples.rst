********
Examples
********
This section contains examples of how to use the :ref:`RTDE Control Interface <rtde-control-api>` and the
:ref:`RTDE Receive Interface <rtde-receive-api>`.

.. warning::
   It is your own responsibility to verify that the movements performed by these examples are collision-free and safe
   to execute on the robot. When in doubt use the simulator provided by Universal Robots.

Basic use
=========
Simple example using the RTDE Control Interface to move the robot to a pose with the **moveL** command.

.. code-block:: c++

   // The constructor simply takes the IP address of the Robot
   RTDEControlInterface rtde_control("127.0.0.1");
   // First argument is the pose 6d vector followed by speed and acceleration
   rtde_control.moveL({-0.143, -0.435, 0.20, -0.001, 3.12, 0.04}, 0.5, 0.2);

Simple example using the RTDE Receive Interface to get the joint positions of the robot

.. code-block:: c++

   // The constructor takes a list of variables that should be available and IP address of the robot
   RTDEReceiveInterface rtde_receive({"actual_q"}, "127.0.0.1");
   std::vector<double> joint_positions = rtde_receive.getActualQ();

.. note::
   When using an e-Series robot data will be received at the maximum available frequency (500Hz), for a CB3
   robot the frequency will be (125Hz).

Forcemode Example
=================
This example will start moving the robot downwards with -20N in the z-axis for 1 second, followed by a move
upwards with 20N in the z-axis for 1 second.

.. code-block:: c++

   #include <rtde_control_interface.h>
   #include <iostream> // only needed for the printout
   #include <thread> // only needed for the delay

   int main(int argc, char* argv[])
   {
      RTDEControlInterface rtde_control("127.0.0.1");
      std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
      std::vector<int> selection_vector = {0, 0, 1, 0, 0, 0};
      std::vector<double> wrench_down = {0, 0, -20, 0, 0, 0};
      std::vector<double> wrench_up = {0, 0, 20, 0, 0, 0};
      int force_type = 2;
      std::vector<double> limits = {2, 2, 1.5, 1, 1, 1};

      rtde_control.forceModeStart(task_frame, selection_vector, wrench_down, force_type, limits);
      std::cout << std::endl << "Going Down!" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::cout << std::endl << "Going Up!" << std::endl << std::endl;
      rtde_control.forceModeUpdate(wrench_up);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      rtde_control.forceModeStop();
   }