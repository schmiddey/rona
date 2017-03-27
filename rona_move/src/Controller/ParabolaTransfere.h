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

class ParabolaTransfere: public controller::Controller_base
{
public:
   ParabolaTransfere(
         double max_vel_lin,
         double max_vel_ang,
         double parabola_lin_fac,
         double parabola_ang_fac);
   virtual ~ParabolaTransfere();

   virtual controller::velocity control(double linear, double angular);

private: //functions
   double parabola(double scale, double value, double max_vale);

private: //dataelements
   double _max_vel_lin;
   double _max_vel_ang;
   double _parabola_ang_fac;
   double _parabola_lin_fac;
};

} /* namespace controller */

#endif /* OHM_AUTONOMY_SRC_OHM_PATH_CONTROL_SRC_CONTROLLER_PARABOLATRANSFERE_H_ */
