/**
 * @file   AccRamp.h
 * @author Michael Schmidpeter
 * @date   2018-07-16
 * @brief  todo
 * 
 * PROJECT: rona
 * @see https://github.com/schmiddey/rona
 */


#ifndef ACCRAMP_H_
#define ACCRAMP_H_

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>


namespace rona{


class AccRamp{
public:
  AccRamp(const double max_acc_lin, const double max_acc_ang, const double rate)
  {
    _max_acc_lin = std::abs(max_acc_lin);
    _max_acc_ang = std::abs(max_acc_ang);
    _rate = std::abs(rate);

    _delta_lin = _max_acc_lin / rate;
    _delta_ang = _max_acc_ang / rate;

    _last_cmd.linear.x = 0.0;
    _last_cmd.linear.y = 0.0;
    _last_cmd.linear.z = 0.0;
    _last_cmd.angular.x = 0.0;
    _last_cmd.angular.y = 0.0;
    _last_cmd.angular.z = 0.0;
  }
  ~AccRamp()
  { }

  geometry_msgs::Twist toc_ramp(const geometry_msgs::Twist& cmd)
  {
    // Eigen::Vector3d lin_cmd(cmd.linear.x,  cmd.linear.y, 0.0);
    // Eigen::Vector3d lin_old_cmd(_last_cmd.linear.x,  _last_cmd.linear.y, 0.0);


    // double norm = 0.0;

    // _last_cmd.linear.x = 0;
    // _last_cmd.linear.y = 0;


    // _last_cmd.angular.z = 0;

    //hack -> lin movement is not interpreded as vector -> hack!!! todo do it better :)
    double delta_lx = _last_cmd.linear.x - cmd.linear.x;
    double delta_ly = _last_cmd.linear.y - cmd.linear.y;


    double sgn_lin_x = delta_lx >= 0.0 ? -1.0 : 1.0;
    double sgn_lin_y = delta_ly >= 0.0 ? -1.0 : 1.0;

    if(std::abs(delta_lx) > _delta_lin)
    {
      _last_cmd.linear.x += sgn_lin_x * _delta_lin;
    }
    else
    {
      _last_cmd.linear.x = cmd.linear.x;
    }

    if(std::abs(delta_ly) > _delta_lin)
    {
      _last_cmd.linear.y += sgn_lin_y * _delta_lin;
    }
    else
    {
      _last_cmd.linear.y = cmd.linear.y;
    }


    double delta_a = _last_cmd.angular.z - cmd.angular.z;
    double sgn_ang = delta_a >= 0.0 ? -1.0 : 1.0;

    if(std::abs(delta_a) > _delta_ang)
    {
      _last_cmd.angular.z += sgn_ang * _delta_ang;
    }
    else
    {
      _last_cmd.angular.z = cmd.angular.z;
    }

    return _last_cmd;
  }
  
private:

  double _max_acc_lin;
  double _max_acc_ang;
  double _rate;  
  
  double _delta_lin;
  double _delta_ang;

  geometry_msgs::Twist _last_cmd;
};

}

#endif  //ACCRAMP_H_