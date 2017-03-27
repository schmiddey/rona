/*
 * BasicAnalyser.h
 *
 *  Created on: 23.03.2015
 *      Author: m1ch1
 */

#ifndef OHM_AUTONOMY_SRC_OHM_PATH_CONTROL_SRC_PATHANALYSER_BASICANALYSER_H_
#define OHM_AUTONOMY_SRC_OHM_PATH_CONTROL_SRC_PATHANALYSER_BASICANALYSER_H_

#include "PathAnalyser_base.h"
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <string>
#include <boost/lexical_cast.hpp>
#include <limits>


namespace analyser
{

class BasicAnalyser: public PathAnalyser_base
{
public:
   BasicAnalyser(
         double target_r,
         double target_r_last,
         unsigned int cos_pwr_n,
         double cos_fac_n,
         double ang_reachd_range,
         double end_approach);
   virtual ~BasicAnalyser();

   analyser::diff_scale analyse(analyser::pose current_pose);

   double getDetectionRadius() const;

private: //functions
   /**
    * @brief contains the transfere function with a cos(mx)^(2..n)
    *
    * @param cos_pwr    -> power of cos(mx) must be >= 2
    * @param cos_factor -> factor of x in cos
    * @param value      -> value to transfers
    * @return  scalefactor 0..1
    */
   double scaleFnk_cos_n(unsigned int cos_pwr, double cos_factor, double value);

   int getDirection(Vector3d p, Vector3d ori);

   double getLinFactor_dist(double distance);

   double getLinScaleFactor_ang_n(double angDiff_scale);

   void local_reachedFinalGoal(bool state);

private: //dataelements
   //analyser parameter
   double _target_radius;
   double _target_radius_last;
   double _curr_target_radius;
   unsigned int _cos_pwr_n;
   double _cos_fac_n;
   double _ang_reached_range;
   double _end_approach;

   bool _reachedLastPose;
};

} /* namespace analyser */

#endif /* OHM_AUTONOMY_SRC_OHM_PATH_CONTROL_SRC_PATHANALYSER_BASICANALYSER_H_ */
