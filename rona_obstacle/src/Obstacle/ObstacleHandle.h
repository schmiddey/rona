/*
 * ObstacleHandle.h
 *
 *  Created on: 29.02.2016
 *      Author: m1ch1
 */

#ifndef OBSTACLEHANDLE_H_
#define OBSTACLEHANDLE_H_

#include <iostream>
#include <vector>
#include <map>
#include <memory>

#include <rona_lib/Utility.h>
#include <rona_lib/Map/Operations.h>

#include "ObstacleEventHandler.h"

namespace rona
{

enum class ObstacleState { NEW = 0,
                           RENEW,
                           EXISTING };

struct ObstacleInfo{
   ObstacleState state;
   double fitAccuracy;  ///< smaller value -> better fit
   unsigned int id;
};

class ObstacleHandle
{
public:
   /**
    * @brief Constructor
    *
    * @param obstacle         -> as polygon
    * @param id               -> id from polygon
    * @param scale            -> for obstacle, for proving if other obstacle is the same
    * @param duration_valid   -> time which obstacle is valid in [s], negative value means endless validity
    */
   ObstacleHandle(map::Polygon obstacle, unsigned int id, double scale, double duration_valid)
   {
      if(duration_valid > 0) //if < 0 then no timer needed
      {
         //activate oneshot
         _timer = _nh.createTimer(ros::Duration(duration_valid), &ObstacleHandle::timerInvalid_callback, this, true);
      }

      _eventHandler = ObstacleEventHandler::getInstance();

      _isValid = true;
      _renewConuter = 0;
      _id = id;
      _duration_valid = duration_valid;
      _scale = scale;
      _obstacle = obstacle;
      _obstacleScaled = map::Operations::scale(_obstacle, _scale);
   }

   virtual ~ObstacleHandle() { }




   inline bool isValid() const
   {
      return _isValid;
   }
   inline void setValid(const bool valid)
   {
      _isValid = valid;
      if(_isValid)
      {
         _renewConuter++;
         _timer.setPeriod(ros::Duration(_duration_valid));
      }

   }

   inline unsigned int getRenewCnt() const { return _renewConuter; }

   /**
    * @todo compute fitting accuracy - value...
    * @param polygon
    * @return
    */
   inline ObstacleInfo isSameObstacle(map::Polygon& polygon)
   {
      ObstacleInfo info;
      info.id = _id;
      //prove if all new polygon points are in sclaed poygon(old obstacle)
      unsigned int cnt = 0;
      for(auto e : polygon.points)
      {
         if(map::Operations::pointInPolygon(_obstacleScaled, e))
            cnt++;
      }

      ///@todo check more properties of the polygon ... center point ... area ... shape ...
      //compute fitAccuracy
      // fitAccuracy = center-center dist + area variation
      map::Point2D c_new = map::Operations::computeCentroid(polygon);
      map::Point2D c_old = map::Operations::computeCentroid(_obstacle);
      double c_dist = map::Operations::computeDistance(c_new, c_old);
      double area_new = map::Operations::computeArea(polygon);
      double area_old = map::Operations::computeArea(_obstacle);
      double area_diff = std::abs(area_old - area_new);
      //double area_diff_fac = ara_d
      info.fitAccuracy = c_dist * c_dist + area_diff; //c_dist^2 cause ara and dist ca same weight...

      if(cnt < polygon.points.size())
      {//not enough points in polygon
         info.state =  ObstacleState::NEW;
      } else if(this->isValid())
      {
         info.state =  ObstacleState::EXISTING;
      } else
      {
         info.state =  ObstacleState::RENEW;
      }

      return info;
   }

   inline const unsigned int getId() const { return _id; }
   inline const map::Polygon getObstacle() const { return _obstacle; }
   inline const map::Polygon getObstacleScaled() const { return _obstacleScaled; }

private: //functions
   inline void timerInvalid_callback(const ros::TimerEvent& e)
   {
      //_timer.stop();
      //fire drop Event
      _eventHandler->dropObstacle_event(_id);
      _isValid = false;
   }

private: //data elements
   ros::NodeHandle _nh;
   ros::Timer _timer;

   std::shared_ptr<ObstacleEventHandler> _eventHandler;

   bool _isValid;

   unsigned int _renewConuter;
   unsigned int _id;

   double _duration_valid;

   map::Polygon _obstacleScaled;
   map::Polygon _obstacle;
   double _scale;
};

} /* namespace rona */

#endif /* OBSTACLEHANDLE_H_ */
