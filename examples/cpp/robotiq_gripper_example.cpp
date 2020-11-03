#include <iostream>
#include <thread>
#include <chrono>

#include "robotiq_gripper.h"

using namespace std;
using namespace ur_rtde;

/**
 * Print object detection status of gripper
 */
void printStatus(int Status)
{
	switch (Status)
	{
	case RobotiqGripper::MOVING: std::cout << "moving"; break;
	case RobotiqGripper::STOPPED_OUTER_OBJECT: std::cout << "outer object detected"; break;
	case RobotiqGripper::STOPPED_INNER_OBJECT: std::cout << "inner object detected"; break;
	case RobotiqGripper::AT_DEST: std::cout << "at destination"; break;
	}

	std::cout << std::endl;
}


int main(int argc, char* argv[])
{
	std::cout << "Gripper test" << std::endl;
	ur_rtde::RobotiqGripper Gripper("127.0.0.1", 63352, true);
	Gripper.connect();

	// Test emergency release functionality
	if (!Gripper.isActive())
	{
		Gripper.emergencyRelease(RobotiqGripper::OPEN);
	}
	std::cout << "Fault status: 0x" << std::hex << Gripper.faultStatus() << std::dec << std::endl;
	std::cout << "activating gripper" << std::endl;
	Gripper.activate();

	// Test setting of position units and conversion of position values
	Gripper.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_DEVICE);
	std::cout << "OpenPosition: " << Gripper.getOpenPosition() <<
		"  ClosedPosition: " << Gripper.getClosedPosition() << std::endl;
	Gripper.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_NORMALIZED);
	std::cout << "OpenPosition: " << Gripper.getOpenPosition() <<
		"  ClosedPosition: " << Gripper.getClosedPosition() << std::endl;

	// Test of move functionality with normalized values (0.0 - 1.0)
	auto Status = Gripper.move(1, 1, 0, RobotiqGripper::WAIT_FINISHED);
	printStatus(Status);
	Status = Gripper.move(0, 1, 0, RobotiqGripper::WAIT_FINISHED);
	printStatus(Status);

	// We preset force and and speed so we don't need to pass it to the
	// following move functions
	Gripper.setForce(0.0);
	Gripper.setSpeed(0.5);

	// We switch the position unit the mm and define te position range of
	// our gripper
	Gripper.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_MM);
	Gripper.setPositionRange_mm(10, 50);
	std::cout << "OpenPosition: " << Gripper.getOpenPosition() <<
		"  ClosedPosition: " << Gripper.getClosedPosition() << std::endl;
	Gripper.move(50);
	Status = Gripper.waitForMotionComplete();
	printStatus(Status);

	Gripper.move(10);
	Status = Gripper.waitForMotionComplete();
	printStatus(Status);

	std::cout << "moving to open position" << std::endl;
	Status = Gripper.open();
	Status = Gripper.waitForMotionComplete();
	printStatus(Status);

	// Test async move - start move and then wait for completion
	Gripper.close(0.02, 0, RobotiqGripper::START_MOVE);
	Status = Gripper.waitForMotionComplete();
	printStatus(Status);

	Status = Gripper.open(1.0, 0.0, RobotiqGripper::WAIT_FINISHED);

	Gripper.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_DEVICE);
	Gripper.setUnit(RobotiqGripper::SPEED, RobotiqGripper::UNIT_DEVICE);
	Gripper.setUnit(RobotiqGripper::FORCE, RobotiqGripper::UNIT_DEVICE);

	std::cout << "OpenPosition: " << Gripper.getOpenPosition() <<
		"  ClosedPosition: " << Gripper.getClosedPosition() << std::endl;

	Gripper.move(255, 5, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	while (RobotiqGripper::MOVING == Gripper.objectDetectionStatus())
	{
		std::cout << "waiting..." << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	printStatus(Gripper.objectDetectionStatus());


	std::cout << "disconnecting" << std::endl;
	Gripper.disconnect();
}
