/*
 * map_dummy_generator.cpp
 *
 *  Created on: 27.01.2015
 *      Author: chris
 */
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>


int main(int argc,char **argv)
{
   ros::init(argc, argv, "map_dummy");
   ros::NodeHandle nh;

   ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);

   const unsigned int width  = 10;
   const unsigned int height = 10;
   const float        res    = 1;

   // create occupancy grid for testing
   nav_msgs::OccupancyGrid map_dummy;

   // set header to message
   map_dummy.header.seq             = 0;
   map_dummy.header.stamp           = ros::Time::now();
   map_dummy.header.frame_id        = "/map";

   // fill up informations
   map_dummy.info.height            = width;
   map_dummy.info.width             = height;
   map_dummy.info.resolution        = res;
   map_dummy.data.resize(width*height);

   map_dummy.info.origin.position.x =  -5.0;
   map_dummy.info.origin.position.y =  -5.0;

   // fill data to grid
   for(    unsigned int h=0 ; h<width  ; h++) {
      for( unsigned int w=0 ; w<height ; w++)
      {

         const unsigned int idx = h*height + w;
//         std::cout << idx << std::endl;

         map_dummy.data[idx]    = -1;

         // free space
         if((w>3/res) && (w<7/res) && (h>3/res) && (h<7/res)){
            map_dummy.data[idx] = 0;
         }

//         // obstacles
         if(((w>3/res) && (w<7/res)) && ((h==3/res))) {
            map_dummy.data[idx] = 100;
         }

         if(((w>3/res) && (w<7/res)) && ((h==7/res))) {
            map_dummy.data[idx] = 100;
         }
//         if(((w>3/res) && (w<7/res)) && ((h==7/res))) {
//            map_dummy.data[idx] = 100;
//         }

//         if(((h<7/res) && (h>3/res)) && ((w==7/res))) {
//            map_dummy.data[idx] = 100;
//         }
//
         if(w<=4/res)
            map_dummy.data[idx] = 0;
//
//         if(w==0)
//            map_dummy.data[idx] = 100;
      }
   }

   ros::Rate l(0.2);
   while(ros::ok())
   {
      map_pub.publish(map_dummy);
      ros::spinOnce();
      l.sleep();
   }


}

