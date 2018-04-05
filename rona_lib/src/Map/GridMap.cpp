/*
 * GridMap.cpp
 *
 *  Created on: 09.12.2015
 *      Author: m1ch1
 */

#include <rona_lib/Map/GridMap.h>

namespace rona
{
namespace map
{


namespace{
const double PIXEL_SHORT_COST  = 1;
const double PIXEL_LONG_COST   = ::sqrt(PIXEL_SHORT_COST*PIXEL_SHORT_COST + PIXEL_SHORT_COST*PIXEL_SHORT_COST);
}

GridMap::GridMap(std::weak_ptr<Grid> grid)
{
   if(grid.expired())
   {
      std::cerr << "no valid grid given... exit()" << std::endl;
      exit(EXIT_FAILURE);
   }
   else
   {
      _grid = grid.lock();
   }

   this->init();
}

// GridMap::GridMap(uint8_t* data, unsigned int width, unsigned int height, double cellSize, Point2D origin)
// {
//    _grid = std::shared_ptr<Grid>(new Grid(data, width, height, cellSize, origin));
//    this->init();
// }

GridMap::GridMap(const std::weak_ptr<GridMap> map)
{
   if(map.expired())
   {
      std::cerr << "no valid map given... exit()" << std::endl;
      exit(EXIT_FAILURE);
   }
   auto smap = map.lock();
   _grid = std::shared_ptr<Grid>(new Grid(smap->getGrid()));

   this->init();
   _occupiedMinValue = smap->getOccupiedMinValue();
   _occupiedMaxValue = smap->getOccupiedMaxValue();

   //deep copy grid-costmaps
   for(auto& e : smap->_costMaps)
   {
      _costMaps.insert(std::make_pair(e.first, std::shared_ptr<Grid>(new Grid(e.second))));
   }
}

GridMap::~GridMap()
{
}

std::vector<Node> GridMap::getChildNodes(unsigned int id)
{
   //std::cout << "debug: getChildNodes called" << std::endl;
   std::vector<Node> nodes;
   Pixel p = _grid->toPixel(id);

   std::vector<Pixel> tmp_vec(8);

   tmp_vec[0] = (Pixel(p.x-1, p.y-1));
   tmp_vec[1] = (Pixel(p.x,   p.y-1));
   tmp_vec[2] = (Pixel(p.x+1, p.y-1));
   tmp_vec[3] = (Pixel(p.x-1, p.y  ));
   tmp_vec[4] = (Pixel(p.x+1, p.y  ));
   tmp_vec[5] = (Pixel(p.x-1, p.y+1));
   tmp_vec[6] = (Pixel(p.x,   p.y+1));
   tmp_vec[7] = (Pixel(p.x+1, p.y+1));

   for(unsigned int i=0; i<tmp_vec.size(); ++i)
   {
      //prove border cases
      if((int)tmp_vec[i].x >= 0 && (int)tmp_vec[i].y >= 0 && tmp_vec[i].x <= (_grid->getWidth() - 1) && tmp_vec[i].y <= (_grid->getHeight() - 1))
      {
         unsigned int succ_id = _grid->toIdx(tmp_vec[i]);
         //prove occupied
         if(!this->isOccupied(succ_id))
         {//Free
            Node node;
            node.id = succ_id;
            node.pos = _grid->toPoint2D(tmp_vec[i]);

            // only add one edge from gridMap for distance and cost (edge from parent id)
            Edge edge;
            edge.id = id; //parent id
            edge.distance = _parentDist[i];
            edge.cost = this->computeCost(succ_id);

            //insert edge
            node.edges_map.insert(std::make_pair(edge.id, edge));
            nodes.push_back(node);
         }
      }
   }

   return nodes;
}

void GridMap::serialize(std::string file)
{
   //todo
}

void GridMap::load(std::string file)
{
   //todo
}

bool GridMap::addCostMap(std::string identifier, std::weak_ptr<Grid> grid)
{
   if(grid.expired())
      return false;
   auto tmp_grid = grid.lock();

   try {
      _costMaps.at(identifier) = tmp_grid;
   } catch (std::out_of_range& e) {
      //insert
      _costMaps.insert(std::make_pair(identifier, tmp_grid));
   }
   return true;
}


bool GridMap::removeCostMap(std::string identifier)
{
   try {
      _costMaps.at(identifier);
   } catch (std::out_of_range& e) {
      return false;
   }
   _costMaps.erase(identifier);
   return true;
}

double GridMap::computeCost(unsigned int id)
{
   if(!_costMaps.size())
      return 0.0;

   double cost = 0;
   for(auto e : _costMaps)
   {
      cost += e.second->getData()[id];
   }

   //norm to 1 and average value
   ///@todo give higher costs higher prio... not average..

   return cost / 255 * _costMaps.size();
}

void GridMap::init()
{
   _occupiedMinValue = 10;
   _occupiedMaxValue = 255;

   _parentDist.resize(8);

   _parentDist[0] = PIXEL_LONG_COST  * _grid->getCellSize();
   _parentDist[1] = PIXEL_SHORT_COST * _grid->getCellSize();
   _parentDist[2] = PIXEL_LONG_COST  * _grid->getCellSize();
   _parentDist[3] = PIXEL_SHORT_COST * _grid->getCellSize();
   _parentDist[4] = PIXEL_SHORT_COST * _grid->getCellSize();
   _parentDist[5] = PIXEL_LONG_COST  * _grid->getCellSize();
   _parentDist[6] = PIXEL_SHORT_COST * _grid->getCellSize();
   _parentDist[7] = PIXEL_LONG_COST  * _grid->getCellSize();
//   _parentDist[0] = PIXEL_SHORT_COST * _grid->getCellSize();
//   _parentDist[1] = PIXEL_SHORT_COST * _grid->getCellSize();
//   _parentDist[2] = PIXEL_SHORT_COST * _grid->getCellSize();
//   _parentDist[3] = PIXEL_SHORT_COST * _grid->getCellSize();
//   _parentDist[4] = PIXEL_SHORT_COST * _grid->getCellSize();
//   _parentDist[5] = PIXEL_SHORT_COST * _grid->getCellSize();
//   _parentDist[6] = PIXEL_SHORT_COST * _grid->getCellSize();
//   _parentDist[7] = PIXEL_SHORT_COST * _grid->getCellSize();
}

} /* namespace map */
} /* namespace rona */


