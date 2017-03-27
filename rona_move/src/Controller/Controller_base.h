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
   double linear;
   double angular;
} velocity;

class Controller_base
{
public:
   Controller_base() {}
   virtual ~Controller_base() {}

   virtual controller::velocity control(double linear, double angular) = 0;

};

} /* namespace controller */

#endif /* _CONTROLLER_BASE_H_ */
