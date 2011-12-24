/*
 * katana300.h
 *
 *  Created on: Dec 13, 2011
 *      Author: rah
 */

#ifndef KATANA300_H_
#define KATANA300_H_

#include <ros/ros.h>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread.hpp>

#include <kniBase.h>

#include <katana/SpecifiedTrajectory.h>
#include <katana/AbstractKatana.h>
#include <katana/KNIConverter.h>
#include <katana/Katana.h>


namespace katana
{

class Katana300 : public Katana
{
private:
	std::vector<double> MoveToAngel;

public:
	Katana300();
	virtual ~Katana300();

	virtual void setLimits();

	virtual void freezeRobot();
	virtual bool moveJoint(int jointIndex, double turningAngle);

	virtual void refreshMotorStatus();
	virtual bool allJointsReady();
	virtual bool allMotorsReady();

};





}


#endif /* KATANA300_H_ */
