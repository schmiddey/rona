/*
 * Grid.h
 *
 *  Created on: 16.12.2015
 *      Author: m1ch1
 */

#ifndef GRID_H_
#define GRID_H_

///@todo remove this define
#define USE_ROS
#define USE_OPENCV

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <cassert>
#include <cstdint>

//ros stuff
#ifdef USE_ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#endif

//opencv stuff
#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

#include <rona_lib/Map/map_types.h>
#include <rona_lib/Utility.h>


namespace rona
{
namespace map
{



/**
 * @class Grid
 *
 * @brief class for grid Handling
 *
 *
 * @todo implement usage of rotated origins ( maybe not important )
 */
class Grid
{
public:
   /**
    *
    */
   Grid() = default;

#ifdef USE_ROS
   /**
    * @brief Constructor for occupancy_grid from ros
    *
    * @param occ_grid
    */
   Grid(const nav_msgs::OccupancyGrid& occ_grid)
   {
      Point2D origin(occ_grid.info.origin.position.x, occ_grid.info.origin.position.y);
      this->init(occ_grid.info.width, occ_grid.info.height, occ_grid.info.resolution, origin);

      //copy data
      _data.resize(occ_grid.data.size());
      //std::copy(occ_grid.data.begin(), occ_grid.data.end(), _data);
      _data = std::vector<uint8_t>(occ_grid.data.begin(), occ_grid.data.end());

      //prove if data is correct
      assert(_data.size() == _width * _height);
   }

   /**
    * @brief converts grid to nav_msgs::GridCells, only returns pixels which are min_vel <= vel <= max_vel
    * @param min_vel -> min vel of cell which returned;
    * @param max_vel -> max vel of cell which returned;
    * @return
    */
   nav_msgs::GridCells toGridCells(uint8_t min_vel, uint8_t max_vel, std::string frame_id = "map")
   {
      nav_msgs::GridCells gridCells;
      gridCells.header.frame_id = frame_id;
      gridCells.header.stamp = ros::Time();
      gridCells.cell_width = _cellSize;
      gridCells.cell_height = _cellSize;

      for(unsigned int i=0; i<_data.size(); ++i)
      {
         if(_data[i] >= min_vel && _data[i] <= max_vel)
         {
            gridCells.cells.push_back(rona::Utility::toRosPoint(this->toPoint2D(i)));
         }
      }
      return gridCells;
   }


#endif
   /**
    * @brief
    *
    * @param data
    * @param width
    * @param height
    * @param cellSize
    * @param origin
    */
   Grid(uint8_t* data, unsigned int width, unsigned int height, double cellSize, Point2D origin)
   {
      this->init(width, height, cellSize, origin);

      //copy data
      _data.resize(_width * _height);
      //std::copy(data, data + _width * _height, _data);
      _data = std::vector<uint8_t>(data, data + width * height);
   }


   /**
    * @brief cpy constructor
    *
    * @param grid
    * @todo check if working :D
    */
   Grid(const Grid& grid) = default;

   /**
    * @brief move constructor
    *
    * @param grid
    * @todo check if working :D
    */
   //Grid(const Grid&& grid) = default;

   /**
    * @brief copy constructor from a shard_ptr
    *
    * @param grid
    */
   Grid(const std::weak_ptr<Grid> grid);

   /**
    * @brief Destructor
    */
   virtual ~Grid();

#ifdef USE_OPENCV
   inline cv::Mat toCvMat()
   {
      cv::Mat tmp_img(cv::Size(this->getWidth(), this->getHeight()),
                      CV_8UC1,
                      (unsigned char*) &this->getData()[0], //address of first vector element
                      cv::Mat::AUTO_STEP);
      cv::Mat ret;
      tmp_img.copyTo(ret);
      return ret;
   }
#endif

   /**
    * @returns data vector
    *
    * @todo check if const is possible
    *
    * @return
    */
   std::vector<uint8_t>& getData() { return _data; }

   /**
    * @brief proves if a given Grid is compatible to the current Grid (e.G. for costMaps)
    * @param grid
    * @return true if compatible else false
    */
   bool isCompatible(const Grid& grid) const;

   /**
    * @brief computes the Pixel coordinates of a given Point2D, using the cellsize and origin
    *
    * @param p
    * @return
    */
   inline Pixel toPixel(const Point2D& p) const
   {
      int tmp_x;
      int tmp_y;
      tmp_x = std::round((p.x - _cellSize * 0.5) / _cellSize);
      tmp_y = std::round((p.y - _cellSize * 0.5) / _cellSize);

      //translate to origin
      tmp_x -= _origin.x;
      tmp_y -= _origin.y;

      Pixel tmp;
      tmp.x = tmp_x;
      tmp.y = tmp_y;

      return tmp;
   }

   inline Pixel toPixel(unsigned int idx) const
   {
      Pixel tmp;
      tmp.x = idx % _width;
      tmp.y = idx / _width;
      return tmp;
   }

   /**
    * @brief computes the Point2D pos of a given Pixel, using the cellsize and origin
    *
    * @param p
    * @return
    */
   inline Point2D toPoint2D(const Pixel& p) const
   {
      Point2D tmp;
      tmp.x = (double)p.x * _cellSize + _cellSize * 0.5; //todo prove right way
      tmp.y = (double)p.y * _cellSize + _cellSize * 0.5;

      //translate to origin
      tmp.x += (double)_origin.x * _cellSize;
      tmp.y += (double)_origin.y * _cellSize;
      return tmp;
   }

   inline Point2D toPoint2D(const unsigned int idx) const
   {
      return this->toPoint2D(this->toPixel(idx));
   }

   inline unsigned int toIdx(const Pixel pix) const
   {
      return pix.y * _width + pix.x;
   }

   inline unsigned int toIdx(const Point2D p) const
   {
      return this->toIdx(this->toPixel(p));
   }

   // getter functions
   inline double getCellSize() const           { return _cellSize; }
   inline unsigned int getWidth() const        { return _width; }
   inline unsigned int getHeight() const       { return _height; }
   inline unsigned int getCellCnt() const      { return _data.size(); }
   //inline uint8_t getOccupiedMinValue() const  { return _occupied_min_value; }
   //inline uint8_t getOccupiedMaxValue() const  { return _occupied_max_value; }

   inline PixelSigned getOrigin() const        { return _origin; }
   inline Point2D getOriginPoint2D() const     { return Point2D((double)_origin.x * _cellSize + _cellSize * 0.5, (double)_origin.y * _cellSize + _cellSize * 0.5); }


   // setter functions
   //inline void setOccupiedMinValue(const uint8_t val)  { _occupied_min_value = val; }
   //inline void setOccupiedMaxValue(const uint8_t val)  { _occupied_max_value = val; }




private: //functions
   /**
    * @brief inits all values, which must be set in every Grid, copy data must be done otherwise
    *
    * @param width
    * @param height
    * @param cellSize
    * @param origin
    */
    void init(const unsigned int width, const unsigned int height, const double cellSize, const Point2D origin);

private:
   std::vector<uint8_t> _data;

   PixelSigned _origin;
   double _cellSize;
   unsigned int _width;
   unsigned int _height;

};

} /* namespace map */
} /* namespace rona */

#endif /* GRID_H_ */
