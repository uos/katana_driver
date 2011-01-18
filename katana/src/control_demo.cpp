/**********************************************************************************
 *   Katana Native Interface - A C++ interface to the robot arm Katana.
 *   Copyright (C) 2005-2008 Neuronics AG
 *   Check out the AUTHORS file for detailed contact information.
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 **********************************************************************************/
#include "kniBase.h"
#include <iostream>
#include <cstdio>
#include <memory>
#include <vector>
#include <fstream>
#include <pthread.h>
#include "katana/keyboard.h"

using namespace std;


//////////////////////////////////////////////////////////////////////////////////
//Thread structs:
pthread_mutex_t mutex;

struct TPoint
{
  double X, Y, Z;
  double phi, theta, psi;
};

struct Tpos
{
  static vector<int> x, y, z, u, v, w;
  static const int xArr[], yArr[], zArr[], uArr[], vArr[], wArr[];
};

// Katana object
CLMBase* kni;

// positions, hard-coded. Use values from file instead.
const int Tpos::xArr[] = {30206, -23393, -3066, 14454, 30000, 30000};
const int Tpos::yArr[] = {24327, -7837, -16796, 5803, 30290, 31000};
const int Tpos::zArr[] = {24327, -7837, -16796, 5802, 30290, 10924};
const int Tpos::uArr[] = {5333, -13791, -9985, 11449, 30996, 12063};
const int Tpos::vArr[] = {-3799, -5703, -11676, 8210, 30995, 12063};
const int Tpos::wArr[] = {-3799, -5703, -11676, 8210, 30995, 30992};
vector<int> Tpos::x(xArr, xArr + sizeof(xArr) / sizeof(*xArr));
vector<int> Tpos::y(yArr, yArr + sizeof(yArr) / sizeof(*yArr));
vector<int> Tpos::z(zArr, zArr + sizeof(zArr) / sizeof(*zArr));
vector<int> Tpos::u(uArr, uArr + sizeof(uArr) / sizeof(*uArr));
vector<int> Tpos::v(vArr, vArr + sizeof(vArr) / sizeof(*vArr));
vector<int> Tpos::w(wArr, wArr + sizeof(wArr) / sizeof(*wArr));
vector<TPoint> points(0);
void StartPointlistMovement();
void StartProgram();
pthread_t tid;
void* RunProgram(void*);
pid_t threadPid;
int retVal = 0;
bool progRunning = false;
const double PI = 3.14159265358979323846;

//////////////////////////////////////////////////////////////////////////////////
void DisplayHelp()
{
  cout << "-------------------------------------------	\n";
  cout << "?: Display this help\n";
  cout << "c: Calibrate the Katana\n";
  cout << "e: Read the current encoder values\n";
  cout << "E: Read the current force values\n";
  cout << "o: Switch motors off/on (Default: On)\n";
  cout << "x: Read the current position\n";
  cout << "v: Set the velocity limits for all motors seperately\n";
  cout << "V: Set the velocity limits for all motors (or for the TCP if in linear movement mode)\n";
  cout << "a: Set the acceleration limits for all motors seperately\n";
  cout << "A: Set the acceleration limits for all motors\n";
  cout << ",: Set the force limits for all motors\n";
  cout << "w: Read the velocity limits of all motors	\n";
  cout << "W: Read the acceleration limits of all motors	\n";
  cout << "q: Read the Sensors\n";
  cout << "y: Set a new position using IK\n";
  cout << "l: Switch on/off linear movements\n";
  cout << "<: Add a point to the point list\n";
  cout << ">: Move to a specific point\n";
  cout << " : (space) Move to the next point in the point list\n";
  cout << "=: write pointlist to file\n";
  cout << "(: calculate DK from any encoders\n";
  cout << "f: read pointlist from file\n";
  cout << "g: Open Gripper\n";
  cout << "h: Close Gripper\n";
  cout << "n: Set the speed collision limit for all motors seperately\n";
  cout << "N: Set the speed collision limit for all motors\n";
  cout << "s: Set the position collision limit for all motors seperately\n";
  cout << "S: Set the position collision limit for all motors\n";
  cout << "t: Switch collision limit on\n";
  cout << "T: Switch collision limit off\n";
  cout << "u: Unblock motors after crash\n";
  cout << "d: Move motor to degrees\n";
  cout << "z: Set TCP offset\n\n\n";
  cout << "Keyboard of Katana, use the following keys:\n\n";
  cout << "1: Move motor1 +1000 encoders\n";
  cout << "2: Move motor1 -1000 encoders\n";
  cout << "3: Move motor2 +1000 encoders\n";
  cout << "4: Move motor2 -1000 encoders\n";
  cout << "5: Move motor3 +1000 encoders\n";
  cout << "6: Move motor3 -1000 encoders\n";
  cout << "7: Move motor4 +1000 encoders\n";
  cout << "8: Move motor4 -1000 encoders\n";
  cout << "9: Move motor5 +1000 encoders\n";
  cout << "0: Move motor5 -1000 encoders\n";
  cout << "/: Move motor6 +1000 encoders\n";
  cout << "*: Move motor6 -1000 encoders\n";
  cout << "$: Start/Stop Program\n";
  cout << "p: Start/Stop movement through points list\n\n";
}


//////////////////////////////////////////////////////////////////////////////////
void moveMotor(short motorNum, int stepSize)
{
  const TKatMOT* motors = kni->GetBase()->GetMOT();
  int pos = kni->getMotorEncoders(motorNum) + stepSize;

  if (motors->arr[motorNum].checkEncoderInRange(pos))
  {
    cout << "\nMoving to encoder position " << pos << endl;
    kni->moveMotorToEnc(motorNum, pos);
  }
  else
  {
    cout << "\nEncoder target value out of range! \n" << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  if (argc != 3)
  {
    cout << "---------------------------------\n";
    cout << "usage: control CONFIGFILE IP_ADDR\n";
    cout << "---------------------------------\n";
    return 0;
  }

  cout << "--------------------\n";
  cout << "CONTROL DEMO STARTED\n";
  cout << "--------------------\n";

  //----------------------------------------------------------------//
  //open device: a serial port is opened in this case
  //----------------------------------------------------------------//
  auto_ptr<CCdlSocket> device;
  auto_ptr<CCplSerialCRC> protocol;

  try
  {
    int port = 5566;
    device.reset(new CCdlSocket(argv[2], port));

    cout << "-------------------------------------------\n";
    cout << "success:  port " << port << " open\n";
    cout << "-------------------------------------------\n";

    //--------------------------------------------------------//
    //init protocol:
    //--------------------------------------------------------//

    protocol.reset(new CCplSerialCRC());
    cout << "-------------------------------------------\n";
    cout << "success: protocol class instantiated\n";
    cout << "-------------------------------------------\n";
    protocol->init(device.get()); //fails if no response from Katana
    cout << "-------------------------------------------\n";
    cout << "success: communication with Katana initialized\n";
    cout << "-------------------------------------------\n";

    //--------------------------------------------------------//
    //init robot:
    //--------------------------------------------------------//
    kni = new CLMBase();
    kni->create(argv[1], protocol.get());

  }
  catch (Exception &e)
  {
    cout << "ERROR: " << e.message() << endl;
    return -1;
  }
  cout << "-------------------------------------------\n";
  cout << "success: katana initialized\n";
  cout << "-------------------------------------------\n";

  DisplayHelp();
  short counter = 0;
  bool loop = true;
  bool IsOff = false;
  bool useLinearMode = false;
  vector<int> encoders(kni->getNumberOfMotors(), 0);
  //pthread_mutex_init(&mutex, 0);
  //pthread_mutex_lock(&mutex);
  //pthread_mutex_unlock(&mutex);
  //set sensor controller values
  /*TSctDesc  sctdesc[1] = {{15, 8, 16}};		//sctID,resol,count
   CSctBase* sctarr     = new CSctBase[1];		//create sensor-ctrl class
   TKatSCT   katsct     = {1, sctarr, sctdesc};	//fill TKatSCT structure
   */
  CSctBase* sensctrl = &kni->GetBase()->GetSCT()->arr[0];
  int limit;

  //set linear velocity to 60
  kni->setMaximumLinearVelocity(60);

  while (loop)
  {
    double arr_pos[6];
    int input = _getch();
    if (progRunning)
    {
      //Thread killen:
      progRunning = false;
      continue;
    }

    try
    {

      switch (input)
      {
        case '(':
        {
          vector<double> pose_result(6, 0);
          vector<int> etc;
          etc.push_back(20000);
          etc.push_back(-20000);
          etc.push_back(-20000);
          etc.push_back(20000);
          etc.push_back(20000);
          etc.push_back(20000);
          kni->getCoordinatesFromEncoders(pose_result, etc);
          cout.precision(6);
          cout << "\n------------------------------------\n";
          cout << "X: " << pose_result.at(0) << "\n";
          cout << "Y: " << pose_result.at(1) << "\n";
          cout << "Z: " << pose_result.at(2) << "\n";
          cout << "phi: " << pose_result.at(3) << "\n";   // in radian
          cout << "theta: " << pose_result.at(4) << "\n";
          cout << "psi: " << pose_result.at(5) << "\n";
          cout << "------------------------------------\n";
        }
          break;

        case '1':
          moveMotor(0, 1000);
          break;
        case '2':
          moveMotor(0, -1000);
          break;
        case '3':
          moveMotor(1, 1000);
          break;
        case '4':
          moveMotor(1, -1000);
          break;
        case '5':
          moveMotor(2, 1000);
          break;
        case '6':
          moveMotor(2, -1000);
          break;
        case '7':
          moveMotor(3, 1000);
          break;
        case '8':
          moveMotor(3, -1000);
          break;
        case '9':
          moveMotor(4, 1000);
          break;
        case '0':
          moveMotor(4, -1000);
          break;
        case '/':
          moveMotor(5, 1000);
          break;
        case '*':
          moveMotor(5, -1000);
          break;
        case '$':
          StartProgram();
          break;
        case '?':
          DisplayHelp();
          break;
        case 'c':
          // must be called first, otherwise moving motors gives: ERROR: FirmwareException : 'Axis not calibrated (axis 1)'
          cout << "\nCalibrating Katana, please wait for termination... \n" << flush;
          kni->calibrate();
          break;
        case 'e':
          // e: Read the current encoder values
          // (e.g.: 24801 -27258 -27248 23513 25374 25863; after calibration: 28219 -30500 -30500 30500 30500 30500)
          cout << "\nEncoder values: " << endl;
          kni->getRobotEncoders(encoders.begin(), encoders.end());
          for (vector<int>::iterator i = encoders.begin(); i != encoders.end(); ++i)
          {
            cout << *i << " ";
          }
          cout << endl;
          break;
        case 'E':
          // E: Read the current force values (e.g.:  -1 4 -30 -7 -7 -11)
          cout << "\nForce values:" << endl;
          for (int i = 0; i < kni->getNumberOfMotors(); ++i)
          {
            cout << " " << (int)kni->getForce(i + 1);
          }
          cout << endl;
          break;
        case 'o':
          // o: Switch motors off/on (Default: On)
          if (IsOff)
          {
            kni->switchRobotOn();
            IsOff = false;
            cout << "\nMotors on\n";
          }
          else
          {
            kni->switchRobotOff();
            IsOff = true;
            cout << "\nMotors off\n";
          }
          break;
        case 'x':
        {
          TPoint p;
          kni->getCoordinates(p.X, p.Y, p.Z, p.phi, p.theta, p.psi);
          cout.precision(6);
          cout << "\n------------------------------------\n";
          cout << "X: " << p.X << "\n";
          cout << "Y: " << p.Y << "\n";
          cout << "Z: " << p.Z << "\n";
          cout << "phi: " << p.phi << "\n";
          cout << "theta: " << p.theta << "\n";
          cout << "psi: " << p.psi << "\n";
          cout << "------------------------------------\n";
          break;
        }
        case 'q':
        {
          cout << "\nCurrent Sensor values:" << endl;
          const TSctDAT *data = sensctrl->GetDAT();
          bool *change = new bool[data->cnt];
          byte *lastarr = new byte[data->cnt];
          sensctrl->recvDAT();
          for (int k = 0; k < data->cnt; k++)
          { //init stuff
            change[k] = false;
            lastarr[k] = (byte)data->arr[k];
          }
          sensctrl->recvDAT();
          for (int i = 0; i < data->cnt; i++)
          {
            cout.width(5);
            cout << data->arr[i] << " "; //printout data
          }
          cout << "\n";
          break;
        }
        case 'v':
        {
          short velocity;
          cout << "\n\nSet maximum velocity for motors to: \n";
          for (short motorNumber = 0; motorNumber < kni->getNumberOfMotors(); ++motorNumber)
          {
            cout << motorNumber + 1 << ": ";
            cin >> velocity;
            kni->setMotorVelocityLimit(motorNumber, velocity);
          }
        }
          break;
        case 'V':
        {
          if (useLinearMode)
          {
            double velocity;
            cout << "\n\nSet the TCP velocity to: ";
            cin >> velocity;
            kni->setMaximumLinearVelocity(velocity);
            kni->setRobotVelocityLimit(static_cast<short> (velocity));
          }
          else
          {
            short velocity;
            cout << "\n\nSet maximum velocity for all motors to: ";
            cin >> velocity;
            kni->setRobotVelocityLimit(velocity);
            kni->setMaximumLinearVelocity(static_cast<double> (velocity));
          }
          cout << endl;
        }
          break;
        case 'a':
        {
          short acceleration;
          cout << "\n\nSet maximum velocity for motors to: \n";
          for (short motorNumber = 0; motorNumber < kni->getNumberOfMotors(); ++motorNumber)
          {
            cout << motorNumber + 1 << ": ";
            cin >> acceleration;
            kni->setMotorAccelerationLimit(motorNumber, acceleration);
          }
        }
          break;
        case 'A':
        {
          short acceleration;
          cout << "\n\nSet maximum acceleration for all motors to: ";
          cin >> acceleration;
          cout << endl;
          kni->setRobotAccelerationLimit(acceleration);
        }
          break;
        case ',':
        {
          short limit;
          cout << "\nSet force limit for all motors to (%): ";
          cin >> limit;
          cout << endl;
          kni->setForceLimit(0, limit);
        }
          break;
        case 'w':
        {
          cout << "\nCurrent velocity limits:" << endl;
          for (short motorNumber = 0; motorNumber < kni->getNumberOfMotors(); ++motorNumber)
            cout << motorNumber + 1 << ": " << kni->getMotorVelocityLimit(motorNumber) << endl;
          cout << "linear: " << kni->getMaximumLinearVelocity() << endl;
          break;
        }
        case 'W':
        {
          cout << "\nCurrent acceleration limits:" << endl;
          for (short motorNumber = 0; motorNumber < kni->getNumberOfMotors(); ++motorNumber)
            cout << motorNumber + 1 << ": " << kni->getMotorAccelerationLimit(motorNumber) << endl;
          break;
        }
        case 'l':
          if (useLinearMode)
          {
            cout << "Switching to inverse kinematics movement mode\n";
          }
          else
          {
            cout << "Switching to linear movement mode\n";
          }
          useLinearMode = !useLinearMode;
          break;
        case 'y':
          cout << "\n\nInsert cartesian parameters: \n";
          cout << "X: ";
          cin >> arr_pos[0];
          cout << "Y: ";
          cin >> arr_pos[1];
          cout << "Z: ";
          cin >> arr_pos[2];
          cout << "phi: ";
          cin >> arr_pos[3];
          cout << "theta: ";
          cin >> arr_pos[4];
          cout << "psi: ";
          cin >> arr_pos[5];
          if (useLinearMode)
          {
            kni->moveRobotLinearTo(arr_pos[0], arr_pos[1], arr_pos[2], arr_pos[3], arr_pos[4], arr_pos[5]);
          }
          else
          {
            kni->moveRobotTo(arr_pos[0], arr_pos[1], arr_pos[2], arr_pos[3], arr_pos[4], arr_pos[5]);
          }
          break;
        case '<':
          cout.precision(6);
          TPoint point;
          kni->getCoordinates(point.X, point.Y, point.Z, point.phi, point.theta, point.psi);
          cout << "Point: ";
          cout << "  X=" << point.X;
          cout << ", Y=" << point.Y;
          cout << ", Z=" << point.Z;
          cout << ", phi=" << point.phi;
          cout << ", theta=" << point.theta;
          cout << ", psi=" << point.psi;
          cout << " ... added to point list as number ";
          cout << points.size() << endl;
          points.push_back(point);
          break;
        case '>':
          cout.width(10);
          cout.precision(3);
          cout << "\nMoving to point? ";
          size_t pointNumber;
          cin >> pointNumber;
          if (pointNumber >= points.size())
          {
            cout << "Invalid point number. You have only " << points.size() << " points in your list" << endl;
            break;
          }
          cout.width(6);
          cout << " x=" << points[pointNumber].X;
          cout.width(6);
          cout << " y=" << points[pointNumber].Y;
          cout.width(6);
          cout << " z=" << points[pointNumber].Z;
          cout.width(6);
          cout << " phi=" << points[pointNumber].phi;
          cout.width(6);
          cout << " theta=" << points[pointNumber].theta;
          cout.width(6);
          cout << " psi=" << points[pointNumber].psi;
          cout << endl;
          if (useLinearMode)
          {
            kni->moveRobotLinearTo(points[pointNumber].X, points[pointNumber].Y, points[pointNumber].Z,
                                      points[pointNumber].phi, points[pointNumber].theta, points[pointNumber].psi);
          }
          else
          {
            kni->moveRobotTo(points[pointNumber].X, points[pointNumber].Y, points[pointNumber].Z,
                                points[pointNumber].phi, points[pointNumber].theta, points[pointNumber].psi);
          }
          break;
        case 3:
        case 4:
        case 27:
          loop = false;
          continue;
        case ' ':
          cout.width(10);
          cout.precision(3);
          cout << "Moving to point " << counter << ": ";
          cout.width(6);
          cout << " x=" << points[counter].X;
          cout.width(6);
          cout << " y=" << points[counter].Y;
          cout.width(6);
          cout << " z=" << points[counter].Z;
          cout.width(6);
          cout << " phi=" << points[counter].phi;
          cout.width(6);
          cout << " theta=" << points[counter].theta;
          cout.width(6);
          cout << " psi=" << points[counter].psi;
          cout << endl;
          if (useLinearMode)
          {
            kni->moveRobotLinearTo(points[counter].X, points[counter].Y, points[counter].Z, points[counter].phi,
                                      points[counter].theta, points[counter].psi);
          }
          else
          {
            kni->moveRobotTo(points[counter].X, points[counter].Y, points[counter].Z, points[counter].phi,
                                points[counter].theta, points[counter].psi);
          }
          counter++;
          counter = counter % ((short)points.size());
          break;
        case 'g':
          cout << "Opening gripper...\n";
          kni->openGripper();
          break;
        case 'h':
          cout << "Close gripper...\n";
          kni->closeGripper();
          break;
        case 'n':
          cout << "Set speed collision limit for motors to: \n";
          for (short motorNumber = 0; motorNumber < kni->getNumberOfMotors(); ++motorNumber)
          {
            cout << motorNumber + 1 << ": ";
            cin >> limit;
            kni->setSpeedCollisionLimit(motorNumber, limit);
          }
          break;
        case 'N':
          cout << "Set speed collision limit for all motors to: \n";
          cin >> limit;
          for (short motorNumber = 0; motorNumber < kni->getNumberOfMotors(); ++motorNumber)
          {
            kni->setSpeedCollisionLimit(motorNumber, limit);
          }
          break;
        case 's':
          cout << "Set position collision limit for motors to: \n";
          for (short motorNumber = 0; motorNumber < kni->getNumberOfMotors(); ++motorNumber)
          {
            cout << motorNumber + 1 << ": ";
            cin >> limit;
            kni->setPositionCollisionLimit(motorNumber, limit);
          }
          break;
        case 'S':
          cout << "Set position collision limit for all motors to: \n";
          cin >> limit;
          for (short motorNumber = 0; motorNumber < kni->getNumberOfMotors(); ++motorNumber)
          {
            kni->setPositionCollisionLimit(motorNumber, limit);
          }
          break;
        case 't':
          cout << "Collision detection enabled\n";
          kni->enableCrashLimits();
          break;
        case 'T':
          cout << "WARNING: Collision detection disabled\n";
          kni->disableCrashLimits();
          break;
        case 'u':
          cout << "Unblocking motors\n";
          kni->unBlock();
          break;
        case 'f':
        {
          cout << "Loading which file?\n";
          string filename;
          cin >> filename;
          ifstream listfile(filename.c_str());
          if (!listfile)
          {
            cout << "File not found or access denied." << endl;
            break;
          }
          string line;
          vector<string> tokens;
          const string delimiter = ",";
          int lines = 0;
          while (!listfile.eof())
          {
            listfile >> line;
            string::size_type lastPos = line.find_first_not_of(delimiter, 0);
            string::size_type pos = line.find_first_of(delimiter, lastPos);
            while (string::npos != pos || string::npos != lastPos)
            {
              tokens.push_back(line.substr(lastPos, pos - lastPos));
              lastPos = line.find_first_not_of(delimiter, pos);
              pos = line.find_first_of(delimiter, lastPos);
            }
            TPoint point;
            point.X = atof((tokens.at(0)).data());
            point.Y = atof((tokens.at(1)).data());
            point.Z = atof((tokens.at(2)).data());
            point.phi = atof((tokens.at(3)).data());
            point.theta = atof((tokens.at(4)).data());
            point.psi = atof((tokens.at(5)).data());
            points.push_back(point);
            ++lines;
            tokens.clear();
          }

          cout << lines << " points loaded.\n";
          break;
        }
        case '=':
        {
          cout << "Which file? WARNING: Will be overwritten!\n";
          string filename;
          cin >> filename;
          ofstream listfile(filename.c_str(), ios_base::out);
          int count = 0;
          for (vector<TPoint>::iterator iter = points.begin(); iter != points.end(); ++iter)
          {
            listfile.precision(8);
            if (count != 0)
              listfile << endl;

            listfile << iter->X << "," << iter->Y << "," << iter->Z << ",";
            listfile << iter->phi << "," << iter->theta << "," << iter->psi;
            ++count;
          }
          cout << count << " points saved.\n";
          listfile.close();
          break;
        }
        case 'p':
        {
          cout << "Start playback!\n";
          for (int i = 1; i > 0; ++i)
          {
            if (useLinearMode)
            {
              kni->moveRobotLinearTo(points[i % points.size()].X, points[i % points.size()].Y, points[i
                  % points.size()].Z, points[i % points.size()].phi, points[i % points.size()].theta, points[i
                  % points.size()].psi, true, 10000);
            }
            else
            {
              kni->moveRobotTo(points[i % points.size()].X, points[i % points.size()].Y,
                                  points[i % points.size()].Z, points[i % points.size()].phi,
                                  points[i % points.size()].theta, points[i % points.size()].psi, true, 10000);
            }
            if (i % 100 == 0)
            {
              cout << i << ", " << flush;
            }
          }
          break;
        }
        case 'd':
        {
          long motor;
          double degrees;
          cout << "\nMoving motor to degrees\n";
          cout << " motor: ";
          cin >> motor;
          cout << " degrees: ";
          cin >> degrees;
          if ((motor > 0) && (motor < 7))
          {
            kni->movDegrees(motor - 1, degrees);
          }
          else
          {
            cout << "motor has to be a number from 1 to 6\n";
          }
          break;
        }
        case 'z':
        {
          double xoff, yoff, zoff, psioff;
          cout << "X offset (m): ";
          cin >> xoff;
          cout << "Y offset (m): ";
          cin >> yoff;
          cout << "Z offset (m): ";
          cin >> zoff;
          cout << "psi offset around x axis (rad): ";
          cin >> psioff;
          kni->setTcpOffset(xoff, yoff, zoff, psioff);
          break;
        }
        default:
          cout << "\n'" << input << "' is not a valid command.\n" << endl;
          break;
      }

    }
    catch (Exception &e)
    {
      cout << "\nERROR: " << e.message() << endl;
    }

  }
  return 0;
}
//////////////////////////////////////////////////////////////////////////////////
void StartProgram()
{
  progRunning = true;
  pthread_create(&tid, NULL, RunProgram, (void*)&retVal);
  pthread_detach(tid);
}
//////////////////////////////////////////////////////////////////////////////////
void* RunProgram(void*)
{
  cout << "\nProgram running...type any key to stop after the next cycle\n";
  while (progRunning)
  {
    if (progRunning)
      kni->moveRobotToEnc(Tpos::x, true);
    if (progRunning)
      kni->moveRobotToEnc(Tpos::y, true);
    if (progRunning)
      kni->moveRobotToEnc(Tpos::z, true);
    if (progRunning)
      kni->moveRobotToEnc(Tpos::u, true);
    if (progRunning)
      kni->moveRobotToEnc(Tpos::v, true);
    if (progRunning)
      kni->moveRobotToEnc(Tpos::w, true);
  }
  pthread_exit((void*)&retVal);
  return ((void*)&retVal);
}
//////////////////////////////////////////////////////////////////////////////////

