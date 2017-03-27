/*
 * Astar.h
 *
 *  Created on: 22.12.2015
 *      Author: m1ch1
 */

#ifndef ASTAR_H_
#define ASTAR_H_

#include <iostream>
#include <cassert>

#include <map>
#include <unordered_map>

#include <vector>
#include <memory>
#include <stdexcept>
#include <cstdint>
#include <cmath>
#include <queue>

#include <cassert>

#include <rona_lib/Map/Map.h>



namespace rona
{
namespace planner
{

struct Node_astar {
   Node_astar() : f(0), h(0), g(0), parentId(0), parentDist(0), isInvalid(true)
   { }

   double f;
   double h;
   double g;

   map::Node node;

   unsigned int parentId;     ///< id from parent, for tracing (ptr to parent)
   double parentDist;         ///< dist parent @todo check if needed
   std::weak_ptr<Node_astar> parent;

   bool isInvalid;
};

class Comp{
public:
   bool operator() (const std::shared_ptr<Node_astar>& l, const std::shared_ptr<Node_astar>& r)
   {
      return l->f > r->f;
   }
};

class KeyHash
{
public:
   std::size_t operator()(const unsigned int i) const
   {
      return i;
   }
};


class AStar
{
public:
   AStar();
   AStar(std::weak_ptr<map::Map> map);
   virtual ~AStar();

   std::vector<map::Node> computePath(const map::Node n_start, const map::Node n_end);
   std::vector<map::Node> recomputePath(const map::Node n_start)
   {
      std::cout << "replan called" << std::endl;
      if(_replan_rdy)
      {
         std::cout << "execute replan" << std::endl;
         return computePath(n_start, _current_target);
      }
      return std::vector<map::Node>(0);
   }

   void setMap(std::weak_ptr<map::Map> map);
private:

   inline double compute_g(const std::shared_ptr<Node_astar>& node, const std::shared_ptr<Node_astar>& prenode)
   {
      try {
         //todo
         return prenode->g + node->node.edges_map.at(prenode->node.id).distance;// + node->node.edges_map.at(prenode->node.id).cost;
      } catch (std::out_of_range& e) {
         std::cerr << "invalid element access with parent id.... normaly must exist ... this output should never be dispalyed :) " << std::endl;
         assert(false);
         return -1;
      }
   }

   /**
    * @todo add costs !!!!!!
    * @param n
    * @param end
    * @return
    */
   inline double compute_h(const std::shared_ptr<Node_astar>& node, const std::shared_ptr<Node_astar>& end)
   {
      //euclidian distance
      map::Point2D a = node->node.pos;
      map::Point2D b = end->node.pos;

      double x = b.x - a.x;
      double y = b.y - a.y;

      double cost_scale = 1;
      try{
      cost_scale += node->node.edges_map.at(node->parentId).cost;
      } catch (std::out_of_range& e) {
         std::cerr << "invalid element access with parent id.... normaly must exist ... this output should never be dispalyed :) " << std::endl;
         assert(false);
         return -1;
      }

      if(cost_scale > 1.0001)
      {
         cost_scale *= 10;
      }

      return std::sqrt(x*x + y*y) * cost_scale;
   }

   /**
    * @todo maybe add scale for g and h...
    * @param node
    * @return
    */
   inline double compute_f(const std::shared_ptr<Node_astar>& node)
   {
      return node->g + (node->h); //todo may add weight values
   }

   inline bool contain(std::map<unsigned int, std::shared_ptr<Node_astar>>& map, unsigned int id)
   {
      try {
         map.at(id);
         return true;
      } catch (std::out_of_range& e) {
         return false;
      }
   }

   inline bool contain(std::unordered_map<unsigned int, std::shared_ptr<Node_astar>,KeyHash>& map, unsigned int id)
   {
      try {
         map.at(id);
         return true;
      } catch (std::out_of_range& e) {
         return false;
      }
   }

   inline bool contain(std::unordered_map<unsigned int, std::shared_ptr<Node_astar>>& map, unsigned int id)
   {
      try {
         map.at(id);
         return true;
      } catch (std::out_of_range& e) {
         return false;
      }
   }

   inline bool contain(std::vector<std::shared_ptr<Node_astar>>& map, unsigned int id)
   {
      if(map[id])
      {
         return true;
      }
      return false;
   }

   inline void pushOpenList()
   {

   }

   inline void tracePath(std::weak_ptr<Node_astar> parent, std::vector<map::Node>& path)
   {
      if(parent.expired())
      {
         //std::cerr << "AStarTracePath: no valid parent ptr given... exit()" << std::endl;
         exit(EXIT_FAILURE);
      }
      std::shared_ptr<Node_astar> par = parent.lock();

      path.push_back(par->node);

      if(par->parent.expired())
      {//start node has no parents...
         //debug
         //std::cout << "debug: TracePath rdy: last id: " << par->node.id << std::endl;
         return;
      }

      this->tracePath(par->parent, path);
   }

private:
   std::shared_ptr<map::Map> _map; ///< shard_ptr to map

   map::Node _current_target;
   bool _replan_rdy;
};

} /* namespace planner */
} /* namespace rona */

#endif /* ASTAR_H_ */
