/*
 * PIController.h
 *
 *  Created on: 11.03.2016
 *      Author: m1ch1
 */

#ifndef PDCONTROLLER_H_
#define PDCONTROLLER_H_

#include "Controller_base.h"

namespace controller
{

class PDController: public Controller_base
{
public:
   PDController(double max_vel_lin,
                double max_vel_ang,
                double lin_fac,
                double T_v,
                double T_ab,
                double K_p)
   {
      _max_vel_lin  = max_vel_lin ;
      _max_vel_ang  = max_vel_ang ;
      _lin_fac      = lin_fac     ;
      _T_v          = T_v           ;
      _T_ab         = T_ab         ;
      _K_p          = K_p         ;

      _ang_err_old = 0.0;

   }
   virtual ~PDController()
   {}

   virtual controller::velocity control(const double linear_x, const double linear_y, const double angular)
   {
      controller::velocity vel;

      double ang = this->pd_control(angular);

      vel.angular = ang;
      vel.linear_x = this->lin(_lin_fac, linear_x, _max_vel_lin);
      _ang_err_old = angular;

      return vel;
   }

   virtual void setMaxVel(const double linear, const double angular)
   {
     _max_vel_lin = linear;
     _max_vel_ang = angular;    
   }

private: //functions
   double lin(const double scale, const double value, const double max_value)
   {
      double max_val = std::abs(max_value);
      int sng = value > 0 ? 1 : -1;
      double tmp = 0;
      tmp = scale * value;
      return std::abs(tmp) > max_val ? (max_val * sng) : tmp;
   }

   double pd_control(const double error)
   {
      //return _ang_vel_old + _K_R * error - (_K_R* (1 - (_T/_T_n))) * _ang_err_old;
      //double y = _K_p * ( error + ((_T_v/_T_ab) * (error - _ang_err_old)));
//      std::cout << "-----------" << std::endl;
//      std::cout << "T_v" <<  _T_v << std::endl;
//      std::cout << "T_ab" << _T_ab << std::endl;
//      std::cout << "K_p" <<  _K_p << std::endl;



      double y = _K_p * ( error + ((_T_v/_T_ab) * (error - _ang_err_old)));

      //minmax
      int sng = y > 0 ? 1 : -1;
      y = std::abs(y) > _max_vel_ang ? _max_vel_ang * sng : y;

      return y;
   }

private:
   double _max_vel_lin;
   double _max_vel_ang;
   double _lin_fac;

   double _T_v;
   double _T_ab;
   double _K_p;

   double _ang_err_old;

};


} /* namespace controller */

#endif /* PICONTROLLER_H_ */
