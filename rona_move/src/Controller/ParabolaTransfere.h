/*
 * ParabolaTransfere.h
 *
 *  Created on: 04.11.2014
 *      Author: m1ch1
 */

#ifndef OHM_AUTONOMY_SRC_OHM_PATH_CONTROL_SRC_CONTROLLER_PARABOLATRANSFERE_H_
#define OHM_AUTONOMY_SRC_OHM_PATH_CONTROL_SRC_CONTROLLER_PARABOLATRANSFERE_H_

#include "Controller_base.h"
#include <cmath>
#include <string>
#include <boost/lexical_cast.hpp>

namespace controller
{


/**
 * @bug if parabola_lin_fac <= 1 then max speed is never reached... maybe scale first with max vel ... and lower ramp/parabola salcing
 */
class ParabolaTransfere: public controller::Controller_base
{
public:
   ParabolaTransfere(
         double max_vel_lin,
         double max_vel_ang,
         double parabola_lin_fac,
         double parabola_ang_fac);
   virtual ~ParabolaTransfere();

   virtual controller::velocity control(const double linear_x, const double linear_y, const double angular);

private: //functions
   double parabola(const double scale, const double value, const double max_vale);

private: //dataelements
   double _max_vel_lin;
   double _max_vel_ang;
   double _parabola_ang_fac;
   double _parabola_lin_fac;
};

} /* namespace controller */

#endif /* OHM_AUTONOMY_SRC_OHM_PATH_CONTROL_SRC_CONTROLLER_PARABOLATRANSFERE_H_ */
