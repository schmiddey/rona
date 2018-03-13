/*
 * Utility.h
 *
 *  Created on: 17.12.2015
 *      Author: m1ch1
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <rona_lib/Map/map_types.h>

#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

namespace rona {

class Utility {
public:
   static inline bool isEqual(const double x, const double y)
   {
      const double error_factor = 1.0;
      return x == y || std::abs(x-y) < std::abs(std::min(x, y)) * std::numeric_limits<double>::epsilon() * error_factor;
   }
   static inline bool isEqual(const float x, const float y)
   {
      const float error_factor = 1.0f;
      return x == y || std::abs(x-y) < std::abs(std::min(x, y)) * std::numeric_limits<float>::epsilon() * error_factor;
   }
//   template<typename T>
//   static inline bool isEqual(const T x, const T y)
//   {
//      const T error_factor = 1.0;
//      return x == y || std::abs(x-y) < std::abs(std::min(x, y)) * std::numeric_limits<T>::epsilon() * error_factor;
//   }

  static inline double computeDist(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
  {
   double x = b.x - a.x;
   double y = b.y - a.y;
   return std::sqrt(x*x + y*y);
  }


   /**
    * @brief returns proved value by abs_min_value if(abs(val) < min) return 0
    *
    * @param val
    * @param abs_min_value
    * @return vlaue or 0 if value > abs_min_value
    */
   static inline double proveMin(const double val, const double abs_min_value)
   {
      if(std::abs(val) < std::abs(abs_min_value))
         return 0;
      else
         return val;
   }

//   static inline double sng(const double val)
//   {
//      return static_cast<double>( (val>0.0) - (val<0.0) );
//   }

   /**
    * @brief returns transform
    *
    * @param tf_listener     -> valid tf listener
    * @param target_frame    -> target frame e.g map_frame
    * @param source_frame    -> source frame e.g robot_frame
    * @param d               -> wait for transform duration, default 1s
    * @return  Stamped Transform
    */
   static inline tf::StampedTransform getTransform(tf::TransformListener& tf_listener, std::string target_frame, std::string source_frame, ros::Duration d = ros::Duration(5))
   {
      tf::StampedTransform tf;
      try {
         ros::Time time = ros::Time(0);
         tf_listener.lookupTransform(target_frame, source_frame, time, tf);
         tf_listener.waitForTransform(target_frame, source_frame, time, d);

      } catch (tf::TransformException& e)
      {
         ROS_ERROR("Unable to get Transform from %s to %s", target_frame.c_str(), source_frame.c_str());
         ROS_ERROR("what(): %s", e.what());
         return tf;
      }
      return tf;
   }

   /**
    * @brief hack to transform polygon...
    *
    * @param tf_listener
    * @param target_frame
    * @param poly
    * @return
    */
   static inline geometry_msgs::PolygonStamped transformPoylgon(tf::TransformListener& tf_listener, std::string target_frame, geometry_msgs::PolygonStamped poly)
   {
      geometry_msgs::PolygonStamped poly_out;
      poly_out.header.frame_id = target_frame;
      //poly_out.header.stamp = ros::Time();

      for(auto e : poly.polygon.points)
      {
         geometry_msgs::PointStamped p;
         geometry_msgs::PointStamped p_out;
         //Point32 to PointStamped
         p.point.x = static_cast<double>(e.x);
         p.point.y = static_cast<double>(e.y);
         p.point.z = static_cast<double>(e.z);
         p.header.frame_id = poly.header.frame_id;
         p.header.stamp = poly.header.stamp;
         try{
            tf_listener.waitForTransform(target_frame, poly.header.frame_id, poly.header.stamp, ros::Duration(1));
            tf_listener.transformPoint(target_frame, p, p_out);
         } catch (tf::TransformException& e) {
            ROS_ERROR("Exeption at tranformPoint at polygon transform from %s to %s: %s", poly.header.frame_id.c_str(), target_frame.c_str(), e.what());
         }
         geometry_msgs::Point32 p_poly;
         //PointStamped to Point32
         p_poly.x = static_cast<float>(p_out.point.x);
         p_poly.y = static_cast<float>(p_out.point.y);
         p_poly.z = static_cast<float>(p_out.point.z);

         poly_out.polygon.points.push_back(p_poly);
         poly_out.header.stamp = p_out.header.stamp;
      }
      return poly_out;
   }

   /**
    * @brief returns transform as 2d tf
    *
    * @param tf_listener     -> valid tf listener
    * @param target_frame    -> target frame e.g map_frame
    * @param source_frame    -> source frame e.g robot_frame
    * @param d               -> wait for transform duration, default 1s
    * @return  Poin2D with 2d tf
    */
   static inline map::Point2D getTransformPoint2D(tf::TransformListener& tf_listener, std::string target_frame, std::string source_frame, ros::Duration d = ros::Duration(5))
   {
      return Utility::toPoint2D(Utility::getTransform(tf_listener, target_frame, source_frame, d));
   }

   static inline nav_msgs::Path toRosPath(map::Path& path, std::string frame_id = std::string("map"))
   {
      nav_msgs::Path ros_path;
      ros_path.header.frame_id = frame_id;
      for(unsigned int i=0; i<path.size(); ++i)
      {
         ros_path.poses.push_back(Utility::toRosPoseStamped(path[i].pos,frame_id));
         ros_path.poses[i].header.seq = i;
      }

      if(ros_path.poses.empty())
      {//if empty just add one empty pose
         geometry_msgs::PoseStamped p;
         ros_path.poses.push_back(p);
      } else if(ros_path.poses.size() == 1)
      {//pushback same pose to prevent a path with length 1 is seen as empty path
         ros_path.poses.push_back(ros_path.poses[0]);
      }

      return ros_path;
   }

   /**
    * @todo check if id needed
    * @param ros_path
    * @return
    */
   static inline map::Path toRonaPath(const nav_msgs::Path& ros_path)//, std::weak_ptr<map::GridMap> map_wp)
   {
      //if(map_wp.expired())
      //{
      //   ROS_ERROR("map not valid... return empty path");
      //   return map::Path(0);
      //}
      //std::shared_ptr<map::GridMap> map = map_wp.lock();
      map::Path path(ros_path.poses.size());

      for(unsigned int i=0; i<ros_path.poses.size(); ++i)
      {
         map::Node tmp_node;
         tmp_node.pos = Utility::toPoint2D(ros_path.poses[i]);
         tmp_node.id = 0;//map->getGrid()->  toIdx(tmp_node.pos); // todo... hier add to idx and pose...blablabla

         path[i] = tmp_node;
      }

      return path;
   }

   //static inline std::vector<double> toAppsMotion()

//   static inline std_msgs::UInt32MultiArray toRosUInt32Array(std::vector<unsigned int> uint_array)
//   {
//      std_msgs::UInt32MultiArray ros_array;
//
//      for(unsigned int i=0; i<uint_array.size(); ++i)
//      {
//         ros_array.data.push_back(uint_array[i]);
//      }
//
//      return ros_array;
//   }

   static inline map::Point2D toPoint2D(const geometry_msgs::PoseStamped& ros_pose)
   {
      map::Point2D pose;
      pose.x = ros_pose.pose.position.x;
      pose.y = ros_pose.pose.position.y;

      return pose;
   }

   static inline map::Point2D toPoint2D(const geometry_msgs::Pose& ros_pose)
   {
      map::Point2D pose;
      pose.x = ros_pose.position.x;
      pose.y = ros_pose.position.y;

      return pose;
   }

   static inline map::Point2D toPoint2D(const geometry_msgs::Point& ros_point)
   {
      map::Point2D pose;
      pose.x = ros_point.x;
      pose.y = ros_point.y;

      return pose;
   }


   static inline map::Point2D toPoint2D(const tf::StampedTransform& tf)
   {
      map::Point2D pos;

      pos.x = tf.getOrigin().x();
      pos.y = tf.getOrigin().y();

      return pos;
   }

   static inline geometry_msgs::PoseStamped toRosPoseStamped(const map::Point2D& pos, std::string frame_id = std::string("map"))
   {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = frame_id;
      p.header.stamp = ros::Time();
      p.pose.position.x = pos.x;
      p.pose.position.y = pos.y;
      p.pose.position.z = 0;
      p.pose.orientation.w = 1;
      p.pose.orientation.x = 0;
      p.pose.orientation.y = 0;
      p.pose.orientation.z = 0;

      return p;
   }

   static inline geometry_msgs::Point toRosPoint(const map::Point2D pos)
   {
      geometry_msgs::Point p;
      p.x = pos.x;
      p.y = pos.y;
      p.z = 0;
      return p;
   }


   static inline map::Polygon toPolygon(const geometry_msgs::PolygonStamped ros_pol)
   {
      map::Polygon pol;
      for(auto e : ros_pol.polygon.points)
      {
         rona::map::Point2D p(e.x,e.y);
         pol.points.push_back(p);
      }
      return pol;
   }

   static inline geometry_msgs::PolygonStamped toRosPolygon(const map::Polygon pol, std::string frame_id = std::string("map"))
   {
      geometry_msgs::PolygonStamped ros_pol;
      ros_pol.header.frame_id = frame_id;
      ros_pol.header.stamp = ros::Time();
      for(auto e : pol.points)
      {
         geometry_msgs::Point32 p;
         p.x = e.x;
         p.y = e.y;
         p.z = 0; // not used;
         ros_pol.polygon.points.push_back(p);
      }
      return ros_pol;
   }

   static inline map::Polygon toPolygon(const map::Rect2D rect)
   {
      map::Polygon pol;

      pol.points.resize(4);

      pol.points[0] = rect.p;
      pol.points[1] = map::Point2D(rect.p.x, rect.p.y + rect.h);
      pol.points[2] = map::Point2D(rect.p.x + rect.w, rect.p.y + rect.h);
      pol.points[3] = map::Point2D(rect.p.x + rect.w, rect.p.y);

      return pol;
   }

   static inline visualization_msgs::Marker toRosMarker(const map::Polygon pol, std_msgs::ColorRGBA col, unsigned int id = 0, std::string frame_id = std::string("map"))
   {
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = ros::Time();

      if(pol.points.empty()) // for savety :)
         return marker;

      marker.color = col;
      marker.ns = frame_id;
      marker.id = id;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      map::Polygon polygon = pol;
      polygon.points.push_back(polygon.points[0]);    //to close last line to start...

      for(auto& e : polygon.points)
      {
         marker.points.push_back(toRosPoint(e));
      }

      return marker;
   }

   static inline visualization_msgs::Marker toRosMarker(const map::Rect2D rect, std_msgs::ColorRGBA col, unsigned int id = 0, std::string frame_id = std::string("map"))
   {
      map::Polygon pol = toPolygon(rect);

      return toRosMarker(pol, col, id, frame_id);
   }



private:
};



}  // namespace rona

#endif /* UTILITY_H_ */
