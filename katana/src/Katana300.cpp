/*
 * Katana300.cpp
 *
 *  Created on: Dec 16, 2011
 *      Author: rah
 */

#include <katana/Katana300.h>

namespace katana {

Katana300::Katana300() :
	Katana() {

	MoveToAngel = getMotorAngles();
	setLimits();
}

Katana300::~Katana300() {
}

void Katana300::setLimits() {

	kni->setMotorAccelerationLimit(0, 1);
	kni->setMotorVelocityLimit(0, 30);

	for (size_t i = 1; i < NUM_MOTORS; i++) {
		// These two settings probably only influence KNI functions like moveRobotToEnc(),
		// openGripper() and so on, and not the spline trajectories. We still set them
		// just to be sure.
		kni->setMotorAccelerationLimit(i, 1);
		kni->setMotorVelocityLimit(i, 25);
	}

}

void Katana300::freezeRobot() {
	boost::recursive_mutex::scoped_lock lock(kni_mutex);
	kni->freezeRobot();
}

bool Katana300::moveJoint(int jointIndex, double turningAngle) {

	MoveToAngel[jointIndex] = turningAngle;

	return Katana::moveJoint(jointIndex,turningAngle);

}

void Katana300::refreshMotorStatus() {
	Katana::refreshEncoders();
	Katana::refreshMotorStatus();
}

bool Katana300::allJointsReady() {

	std::vector<double> MotorAngels = getMotorAngles();

	for (size_t i = 0; i < NUM_JOINTS; i++) {
		//Velocity 0 might be a problem
		if (motor_status_[i] == MSF_MOTCRASHED)
			return false;
		else if ( MoveToAngel[i] < (MotorAngels[i] + 0.01) &&  MoveToAngel[i] > (MotorAngels[i] - 0.01) ) {
			if(motor_velocities_[i] != 0){
				return false;
			}
		}
		else
			return false;
	}

	return true;
}

bool Katana300::allMotorsReady() {

	std::vector<double> MotorAngels = getMotorAngles();

	for (size_t i = 0; i < NUM_JOINTS; i++) {
		//Velocity 0 might be a problem
		if (motor_status_[i] == MSF_MOTCRASHED)
			return false;
		else if ( MoveToAngel[i] < (MotorAngels[i] + 10) &&  MoveToAngel[i] > (MotorAngels[i] - 10) ) {
			if(motor_velocities_[i] != 0){
				return false;
			}
		}
		else
			return false;
	}

	return true;

}

}

