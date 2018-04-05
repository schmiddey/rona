/*
 * Grid.cpp
 *
 *  Created on: 16.12.2015
 *      Author: m1ch1
 */

#include <rona_lib/Map/Grid.h>

namespace rona
{
namespace map
{

Grid::Grid(const std::weak_ptr<Grid> grid)
{
   if(grid.expired())
   {
      std::cerr << "WARNING shared_prt grid object not valid" << std::endl;
      return;
   }
   auto sg = grid.lock();
   this->init(sg->getWidth(), sg->getHeight(), sg->getCellSize(), sg->getOrigin());

   //copy data via std::vector =
   _data = sg->getData();
}


Grid::~Grid()
{ }

bool Grid::isCompatible(const Grid& grid) const
{
   if(rona::Utility::isEqual(grid.getCellSize(),this->getCellSize()) &&
      grid.getWidth()    == this->getWidth()    &&
      grid.getHeight()   == this->getHeight() )
   {
      return true;
   }
   else
   {
      return false;
   }
}




void Grid::init(const unsigned int width, const unsigned int height, const double cellSize, const Point2D origin)
{
  _width = width;
  _height = height;
  _cellSize = cellSize;
  _origin = origin;
}

} /* namespace map */
} /* namespace rona */
