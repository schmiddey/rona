/*
 * Controller_base.h
 *
 *  Created on: 24.10.2014
 *      Author: m1ch1
 */

#ifndef _CONTROLLER_BASE_H_
#define _CONTROLLER_BASE_H_

namespace controller
{

typedef struct {
   double linear_x;
   double linear_y;
   double angular;
} velocity;

class Controller_base
{
public:
   Controller_base() {}
   virtual ~Controller_base() {}

   virtual controller::velocity control(const double linear_x, const double linear_y, const double angular) = 0;

   virtual void setMaxVel(const double linear, const double angular) = 0;

};

} /* namespace controller */

#endif /* _CONTROLLER_BASE_H_ */
