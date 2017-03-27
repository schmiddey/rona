/*
 * MotionPredictor.h
 *
 *  Created on: 10.09.2015
 *      Author: m1ch1
 */

#ifndef ALGORITHM_MULTI_ROBOT_MOTIONPREDICTOR_H_
#define ALGORITHM_MULTI_ROBOT_MOTIONPREDICTOR_H_

#include <vector>
#include <cmath>
#include <memory>

#include "motion_types.h"
#include "../Map/GridMap.h"
#include "../Map/Operations.h"


namespace rona
{
namespace motion
{

class MotionPredictor
{
public:
   /**
    * @brief Constructor
    *
    * @param average_vel in [m/s]
    */
   MotionPredictor(double average_vel) : _average_vel(average_vel) {}
   virtual ~MotionPredictor() {}

   /**
    * @brief computes the time for each path element from the first one in [ms]
    *
    * @param path to compute time
    * @return std::vector<unsigned int> containing time in [ms] needed to reach the corresponding path point
    */
   std::vector<double> computeMotionPrediction(const rona::map::Path& path)
   {
      std::vector<double> motion;
      //init first element //no time needed already on it
      motion.push_back(0);

      if(path.size() < 2)
      {

         return motion;
      }

      double curr_dist = 0;

      for(unsigned int i=1; i<path.size(); ++i)
      {
         rona::map::Point2D a = path[i-1].pos;
         rona::map::Point2D b = path[i].pos;

         curr_dist += rona::map::Operations::computeDistance(a,b);

         motion.push_back(this->computeTime(curr_dist));
      }

      //std::cout << "pathlenth: " << curr_dist << std::endl;

      return motion;
   }

   /**
    * @brief add the given time to the motion time to get absolut araival time
    *
    * @param motion   computed motion
    * @param time     current time in [s] as double
    * @return motion with absolut time values
    */
   std::vector<double> toAbsolutTimeMotion(std::vector<double> motion, double time, unsigned int curr_idx)
   {
      double time_value;
      std::vector<double> abs_motion(motion.size());

      //error
      if(curr_idx >= motion.size())
      {
         std::cout << "invalid index at compute Absolut Motion" << std::endl;
         abs_motion.clear();
         return abs_motion;
      }

      time_value = motion[curr_idx];

      for(unsigned int i=0; i<motion.size(); ++i)
      {
         abs_motion[i] = motion[i] + time - time_value;
      }
      return abs_motion;
   }


   /**
    *
    * @return current average_vel in [m/s]
    */
   double getAverageVel() const
   {
      return _average_vel;
   }

   /**
    * @brief set average_vel in [m/s]
    *
    * @param averageVel in [m/s]
    */
   void setAverageVel(double averageVel)
   {
      _average_vel = averageVel;
   }

private: //functions

   /**
    * @brief computes time ro reach given length with given average vel
    * @param path_length lenth of way to compute time
    * @return time in [s] to reach given length
    */
   double computeTime(double path_length)
   {
      //unsigned int t = 0;
      double t_s = 0;

      if(_average_vel == 0.0)
         return t_s;

      t_s = path_length / _average_vel;

      return t_s;
   }

private: //dataelements
   double _average_vel;
};

}
} /* namespace apps */

#endif /* ALGORITHM_MULTI_ROBOT_MOTIONPREDICTOR_H_ */
