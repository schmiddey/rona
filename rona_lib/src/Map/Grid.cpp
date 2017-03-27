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
   this->init(sg->getWidth(), sg->getHeight(), sg->getCellSize(), sg->getOriginPoint2D());

   //copy data via std::vector =
   _data = sg->getData();
}


Grid::~Grid()
{
   //todo
}

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

   //round todo

   //compute from to Pixel
   _origin.x = std::round((origin.x - _cellSize * 0.5) / _cellSize);
   _origin.y = std::round((origin.y - _cellSize * 0.5) / _cellSize);
}

} /* namespace map */
} /* namespace rona */
