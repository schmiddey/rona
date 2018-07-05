/*
 * LinPwrTransfere.h
 *
 *  Created on: 13.02.2018
 *      Author: m1ch1
 */

#ifndef LINPWRTRANSFERE_H_
#define LINPWRTRANSFERE_H_


#include "Controller_base.h"
#include <cmath>
#include <string>
#include <Eigen/Dense>
namespace controller
{


/**
 * @bug if parabola_lin_fac <= 1 then max speed is never reached... maybe scale first with max vel ... and lower ramp/parabola salcing
 */
class LinPwrTransfere: public controller::Controller_base
{
public:
  LinPwrTransfere(
         double max_vel_lin,
         double max_vel_ang,
         double pwr_ratio_lin_fac,    //value between 0..1 -> 0 lin -> 0.5 lin + pwr  -> 1.0 only pwr
         double pwr_ratio_ang_fac,    //value between 0..1 -> 0 lin -> 0.5 lin + pwr  -> 1.0 only pwr
         double lin_slope = 1.0,
         double ang_slope = 1.0
         ) :
    _max_vel_lin      (max_vel_lin      ),
    _max_vel_ang      (max_vel_ang      ),
    _pwr_ratio_lin_fac(pwr_ratio_lin_fac),
    _pwr_ratio_ang_fac(pwr_ratio_ang_fac),
    _lin_slope        (std::abs(lin_slope)),
    _ang_slope        (std::abs(ang_slope))
  {
    //prove max values...
    _max_vel_ang = std::abs(_max_vel_ang);
    _max_vel_lin = std::abs(_max_vel_lin);
    _pwr_ratio_lin_fac = std::abs(_pwr_ratio_lin_fac);
    _pwr_ratio_ang_fac = std::abs(_pwr_ratio_ang_fac);

    this->fix_max(_pwr_ratio_lin_fac, 1.0);
    this->fix_max(_pwr_ratio_ang_fac, 1.0);
  }

   virtual controller::velocity control(const double linear_x, const double linear_y, const double angular)
   {
     controller::velocity vel;

     vel.angular  = this->transfere_fcn(_pwr_ratio_ang_fac, angular , _max_vel_ang, _ang_slope);

     Eigen::Vector3d lin(linear_x, linear_y, 0.0);
     double norm = lin.norm();

//     std::cout << "lin_x" << linear_x << std::endl;
//     std::cout << "lin_y" << linear_y << std::endl;
//
//     std::cout << "norm1: " << norm << std::endl;

     norm = this->transfere_fcn(_pwr_ratio_lin_fac, norm, _max_vel_lin, _lin_slope);
//     vel.linear_y = this->transfere_fcn(_pwr_ratio_lin_fac, linear_y, _max_vel_lin);

     if(lin.norm() > 0.0)
     {
       norm /= lin.norm();
       lin *= norm;
     }


//     std::cout << "norm: " << norm << std::endl;
//
//     std::cout << "lin.norm: " << lin.norm() << std::endl;

     vel.linear_x = lin.x();
     vel.linear_y = lin.y();

     return vel;
   }

private: //functions
   inline double transfere_fcn(const double scale, const double value, const double max_val, const double slope)
   {
     double ret = (1 - scale) * slope * std::abs(value) + ((1-(1-scale)) * std::pow(std::abs(value), _pwr));

     int sgn = (value > 0) - (value < 0);

     ret = std::abs(ret);

     ret *= max_val;

     this->fix_max(ret, max_val);

     return ret * (double)sgn;
   }

   inline void fix_max(double& value, const double max_val)
   {
     if(value > max_val)
       value = max_val;
   }

private: //dataelements
   double _max_vel_lin      ;
   double _max_vel_ang      ;
   double _pwr_ratio_lin_fac;
   double _pwr_ratio_ang_fac;
   double _lin_slope;
   double _ang_slope;

   int _pwr = 10;
};

} /* namespace controller */


#endif /* LINPWRTRANSFERE_H_ */
