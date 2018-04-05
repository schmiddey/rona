/*
 * GridMap.h
 *
 *  Created on: 09.12.2015
 *      Author: m1ch1
 */

#ifndef GRIDMAP_H_
#define GRIDMAP_H_

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <stdexcept>
#include <cstdint>
#include <cmath>


#include <rona_lib/Map/Map.h>
#include <rona_lib/Map/Grid.h>


#ifdef USE_ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#endif

namespace rona
{
namespace map
{

/**
 * @class GridMap
 *
 * @brief Class for PathPlanner containing a grid. Plus functions for Graph representation
 */
class GridMap: public Map
{
public:

   /**
    * @brief default constructor not used
    */
   GridMap() = delete;

   GridMap(GridMap&& grid) = default;
   GridMap(const GridMap& grid) = default;

  /**
   * @brief Copy Constructor based on weak_ptr to Gridmap
   * 
   * @param map weakptr to Gridmap
   */
   GridMap(const std::weak_ptr<GridMap> map);


   /**
    *
    * @param grid
    */
   GridMap(std::weak_ptr<Grid> grid);
#ifdef USE_ROS
   /**
    *
    * @param occ_grid
    */
   GridMap(const nav_msgs::OccupancyGrid& occ_grid)
   {
      _grid = std::shared_ptr<Grid>(new Grid(occ_grid));
      this->init();
   }
#endif
   /**
    *
    * @param data
    * @param width
    * @param height
    * @param cellSize
    * @param origin
    */
  //  GridMap(uint8_t* data, unsigned int width, unsigned int height, double cellSize, Point2D origin);

   /**
    * @brief Constuctor for loading GridMap from file
    *
    * @todo implement may be with load func
    * @param file
    */
   GridMap(std::string file) = delete;

   virtual ~GridMap();

   virtual std::vector<Node> getChildNodes(unsigned int id);
   virtual void serialize(std::string file);
   virtual void load(std::string file);
   virtual unsigned int getSize()
   {
      return this->getGrid()->getCellCnt();
   }

   /**
    * @brief sets a new Grid
    *
    * @param grid
    */
   void setGrid(const std::weak_ptr<Grid> grid)
   {
      if(grid.expired())
         return;
      _grid = grid.lock();
   }

  void resize(const unsigned int resize)
  {
    _grid = Grid::resize(_grid, resize);
  }
   /**
    * @brief returns shared_ptr of current Grid
    *
    * @return shared_ptr of current Grid
    */
   std::shared_ptr<Grid> getGrid() const { return _grid; }


   /**
    * @brief
    *
    * @param identifier
    * @param grid
    * @return true is successful else false;
    */
   bool addCostMap(std::string identifier, std::weak_ptr<Grid> grid);

   /**
    * @brief
    *
    * @param identifier
    *
    * @return true is successful else false;
    */
   bool removeCostMap(std::string identifier);

   std::weak_ptr<Grid> getCostMap(std::string identifier)
   {
      std::weak_ptr<Grid> wp_grid;
      try {
         wp_grid = _costMaps.at(identifier);
      } catch (std::out_of_range& e) {
      }
      return wp_grid;
   }

   /**
    * @brief proves if a Gridcell is occupied (input index)
    * @param idx
    * @return true if occupied else false
    */
   inline bool isOccupied(unsigned int idx)
   {
      if(_grid->getData()[idx] >= _occupiedMinValue && _grid->getData()[idx] <= _occupiedMaxValue)
         return true;
      else
         return false;
   }

   /**
    * @brief proves if a Gridcell is occupied (input Pixel)
    * @param p
    * @return true if occupied else false
    */
   inline bool isOccupied(Pixel p)
   {
      return this->isOccupied(_grid->toIdx(p));
   }

   /**
    * @brief computes from all costmaps the cost of the given node-id
    *
    * @param id from node / gridcell
    * @return cost
    */
   double computeCost(unsigned int id);

   inline uint8_t getOccupiedMinValue() const  { return _occupiedMinValue; }
   inline uint8_t getOccupiedMaxValue() const  { return _occupiedMaxValue; }

   inline void setOccupiedMinValue(const uint8_t val)  { _occupiedMinValue = val; }
   inline void setOccupiedMaxValue(const uint8_t val)  { _occupiedMaxValue = val; }

private:
   void init();

protected:
   //current Map
   std::shared_ptr<Grid> _grid;     ///< map data as Grid
   //costmaps
   std::map<std::string, std::shared_ptr<Grid> > _costMaps;    ///< container of costmaps as Grids

   uint8_t _occupiedMinValue;       ///< min-value of a cell defined as occupied
   uint8_t _occupiedMaxValue;       ///< max-value of a cell defined as occupied

   std::vector<double> _parentDist; ///< contains distance to parent (8 elements)
};


} /* namespace map */
} /* namespace rona */

#endif /* GRIDMAP_H_ */
