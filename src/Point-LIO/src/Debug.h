#include <iostream>
#include <iomanip>
#include "Estimator.h"
#include "parameters.h"
#include "li_initialization.h"

Vector3d rot_ang;
Vector3d ang_vel_cor;

double linvel_max = 0;
// Trajectory
std::vector<Eigen::Vector3d> trajectory;
double _length_traversed;


template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

void getEulerAngle()
{
    rot_ang = SO3ToEuler(kf_output.x_.rot);
}


void CorrAngularVel()
{
    ang_vel_cor = ang_vel_meas - kf_output.x_.bg;
}

void getLinVelMax()
{
    double linvel = sqrt(pow(kf_output.x_.vel(0), 2) + pow(kf_output.x_.vel(1), 2) + pow(kf_output.x_.vel(2), 2));
    if (linvel > linvel_max)
    {
        linvel_max = linvel;
    }
}



void CalLengthTrans()
{
    double length_traversed = 0.;
    Eigen::Vector3d p_curr = Eigen::Vector3d(0., 0., 0.);
    Eigen::Vector3d p_prev = Eigen::Vector3d(0., 0., 0.);
    // Update trajectory
    trajectory.push_back(kf_output.x_.pos);
      for (const auto& t : trajectory) {
    if (p_prev == Eigen::Vector3d(0., 0., 0.)) {
      p_prev = t;
      continue;
    }
    p_curr = t;
    double l = sqrt(pow(p_curr[0] - p_prev[0], 2) + pow(p_curr[1] - p_prev[1], 2) + pow(p_curr[2] - p_prev[2], 2));

    if (l >= 0.1) {
      length_traversed += l;
      p_prev = p_curr;
    }
  }
  _length_traversed = length_traversed;
}

void debug()
{
      // Print to terminal
  printf("\033[2J\033[1;1H");

  std::cout << std::endl
            << "+-------------------------------------------------------------------+" << std::endl;
  std::cout << "|                         Point_LIO Odometry"  << "               |"
            << std::endl;
  std::cout << "+-------------------------------------------------------------------+" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Position     {W}  [xyz] :: " + to_string_with_precision(kf_output.x_.pos(0), 4) + " "
                                + to_string_with_precision(kf_output.x_.pos(1), 4) + " "
                                + to_string_with_precision(kf_output.x_.pos(2), 4)
    << "|" << std::endl;
    getEulerAngle();
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "EulerAngle   {W}  [rpy] :: " + to_string_with_precision(rot_ang(0), 4) + " "
                                + to_string_with_precision(rot_ang(1), 4) + " "
                                + to_string_with_precision(rot_ang(2), 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Lin Velocity {W}  [xyz] :: " + to_string_with_precision(kf_output.x_.vel(0), 4) + " "
                                + to_string_with_precision(kf_output.x_.vel(1), 4) + " "
                                + to_string_with_precision(kf_output.x_.vel(2), 4)
    << "|" << std::endl;
    CorrAngularVel();
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Ang Velocity {B}  [xyz] :: " + to_string_with_precision(ang_vel_cor(0), 4) + " "
                                + to_string_with_precision(ang_vel_cor(1), 4) + " "
                                + to_string_with_precision(ang_vel_cor(2), 4)
    << "|" << std::endl;

  getLinVelMax();
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
   <<  "Lin Velocity Max        :: " + to_string_with_precision(linvel_max, 4) + " "
    << "|" << std::endl;

  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Accel Bias        [xyz] :: " + to_string_with_precision(kf_output.x_.ba(0), 8) + " "
                                + to_string_with_precision(kf_output.x_.ba(1), 8) + " "
                                + to_string_with_precision(kf_output.x_.ba(2), 8)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Gyro Bias         [xyz] :: " + to_string_with_precision(kf_output.x_.bg(0), 8) + " "
                                + to_string_with_precision(kf_output.x_.bg(1), 8) + " "
                                + to_string_with_precision(kf_output.x_.bg(2), 8)
    << "|" << std::endl;
    CalLengthTrans();
      std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Distance Traveled  :: " + to_string_with_precision(_length_traversed, 4) + " meters"
    << "|" << std::endl;

    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
        << "Distance to Origin :: "
      + to_string_with_precision( sqrt(pow(kf_output.x_.pos(0),2) +
                                       pow(kf_output.x_.pos(1),2) +
                                       pow(kf_output.x_.pos(2),2)), 4) + " meters"
    << "|" << std::endl;

    std::cout << "+-------------------------------------------------------------------+" << std::endl;

}