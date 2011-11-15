/*
 * IGazeboRosKatanaAction.h
 *
 *  Created on: 07.11.2011
 *      Author: karl
 */

#ifndef IGAZEBOROSKATANAACTION_H_
#define IGAZEBOROSKATANAACTION_H_

struct GRKAPoint {
  double position;
  double velocity;
};

class IGazeboRosKatanaAction
{
public:
  virtual ~IGazeboRosKatanaAction();
  virtual GRKAPoint getNextDesiredPoint() = 0;
  virtual void setCurrentPoint(GRKAPoint point) = 0;
  virtual bool hasActiveGoal() = 0;
  virtual void cancleGoal();

};

#endif /* IGAZEBOROSKATANAACTION_H_ */
