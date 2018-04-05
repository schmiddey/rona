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

struct ResizePrio{
  //high prio
  uint8_t p0_low = 1;
  uint8_t p0_high = 100;

  //mid prio
  uint8_t p1_low = 128;
  uint8_t p1_high = 255;

  //low prio
  uint8_t p2_low = 0;
  uint8_t p2_high = 0;
};

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

  enum enum_resize : unsigned int{
    RESIZE_2X2 = 2,
    RESIZE_4X4 = 4,
    RESIZE_8X8 = 8
  };

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

  nav_msgs::OccupancyGrid toRosOccGrid(std::string frame_id = "map")
  {
    nav_msgs::OccupancyGrid occ_grid;
    occ_grid.header.frame_id           = "map";
    occ_grid.header.stamp              = ros::Time::now();

    occ_grid.info.resolution           = this->getCellSize();
    occ_grid.info.width                = this->getWidth();
    occ_grid.info.height               = this->getHeight();
    occ_grid.info.origin.orientation.w = 0.0;
    occ_grid.info.origin.orientation.x = 0.0;
    occ_grid.info.origin.orientation.y = 0.0;
    occ_grid.info.origin.orientation.z = 0.0;
    occ_grid.info.origin.position.x    = this->getOrigin().x;
    occ_grid.info.origin.position.y    = this->getOrigin().y;
    occ_grid.info.origin.position.z    = 0.0;
    occ_grid.data.resize(this->getWidth() * this->getHeight());

    auto& data = this->getData();

    for(unsigned int i=0; i<data.size(); ++i)
    {
      occ_grid.data[i] = static_cast<int8_t>(data[i]);
    }

    return occ_grid;
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
  // Grid(uint8_t* data, unsigned int width, unsigned int height, double cellSize, Point2D origin)
  // {
  //    this->init(width, height, cellSize, origin);

  //    //copy data
  //    _data.resize(_width * _height);
  //    //std::copy(data, data + _width * _height, _data);
  //    _data = std::vector<uint8_t>(data, data + width * height);
  // }

  /**
   * @brief Construct a new Grid object -> empty
   * 
   * @param width 
   * @param height 
   * @param cellSize 
   * @param origin 
   */
  Grid(const unsigned int width, const unsigned int height, double cellSize, const Point2D& origin)
  {
    this->init(width, height, cellSize, origin);
    _data.resize(_width * _height);
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
   * @return
   */
  std::vector<uint8_t>& getData() { return _data; }
   
  const std::vector<uint8_t>& getData() const { return _data; }
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
    Pixel tmp;
    //transform by origin
    double tmp_x = p.x - _origin.x;
    double tmp_y = p.y - _origin.y;

    //convert to pixel
    // tmp.x = std::round(tmp_x / _cellSize);
    // tmp.y = std::round(tmp_y / _cellSize);

    // not rounding seems to be correct...
    tmp.x = tmp_x / _cellSize;
    tmp.y = tmp_y / _cellSize;

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
    //tranform to bottom left
    tmp.x = static_cast<double>(p.x) + 0.5;   //HACK + 0.5 fits best (rounding error...)
    tmp.y = static_cast<double>(p.y) + 0.5;   //HACK + 0.5 fits best (rounding error...)
    
    //convert from pixel to m
    tmp.x *= _cellSize;
    tmp.y *= _cellSize;

    //transform by origin
    tmp.x += _origin.x;
    tmp.y += _origin.y;

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

  inline Point2D getOrigin() const { return _origin; }

  static inline std::shared_ptr<Grid> resize(const std::weak_ptr<Grid> grid, const unsigned int scale ,const ResizePrio& prio = ResizePrio())
  {
    if(grid.expired()) 
    {
      return std::make_shared<Grid>();
    }
    if(scale < 2)
    {
      //do no scaling
      return std::make_shared<Grid>(grid);
    }
    
    auto sgrid = grid.lock();
    //get new width and height
    unsigned int w_mod = sgrid->getWidth() % scale;
    unsigned int h_mod = sgrid->getHeight() % scale;
    unsigned int w = w_mod ? (sgrid->getWidth() / scale) + 1 : (sgrid->getWidth() / scale);
    unsigned int h = h_mod ? (sgrid->getHeight() / scale) + 1 : (sgrid->getHeight() / static_cast<unsigned int>(scale));

    std::cout << "-- scale: " << scale << std::endl;
    std::cout << "new width : " << w << std::endl;
    std::cout << "new height: " << h << std::endl;

    auto resize = std::make_shared<Grid>(w, h, sgrid->getCellSize() * static_cast<double>(scale), sgrid->getOrigin());
    std::cout << "sgrid->getCellSize(): " << sgrid->getCellSize() << std::endl;
    auto& data = resize->getData();

    //set new data
    for (unsigned int i = 0; i < data.size(); i++)
    {
      //get data from origin
      Pixel start_re = resize->toPixel(i);
      Pixel start(start_re.x * scale, start_re.y * scale);
      
      std::vector<uint8_t> cells;
      cells.reserve(scale * scale);

      for (unsigned int x = start.x; x < start.x + scale; x++)
      {
        for (unsigned int y = start.y; y < start.y + scale; y++)
        {
          if(x >= sgrid->getWidth() || y >= sgrid->getHeight())
          {
            break;
          }
          cells.push_back(sgrid->getData()[sgrid->toIdx(Pixel(x,y))]);
        }
      }

      uint8_t cell = cells.front();

      for(auto& e : cells)
      {
        if(e >= prio.p0_low && e <= prio.p0_high)
        {
          cell = e;
          continue;
        }
        else if(e >= prio.p1_low && e <= prio.p1_high)
        {
          if(cell >= prio.p0_low && cell <= prio.p0_high)
            continue;
          cell = e;
        }
        else if(e >= prio.p2_low && e <= prio.p2_high)
        {
          if(cell >= prio.p0_low && cell <= prio.p0_high)
            continue;
          if(cell >= prio.p1_low && cell <= prio.p1_high)
            continue;
          cell = e;
        }
      }

      data[i] = cell;
    }



    return resize;
  }

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

  // PixelSigned _origin;
  Point2D _origin;
  double _cellSize;
  unsigned int _width;
  unsigned int _height;

};

} /* namespace map */
} /* namespace rona */

#endif /* GRID_H_ */
