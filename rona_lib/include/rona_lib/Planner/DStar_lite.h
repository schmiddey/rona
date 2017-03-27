/*
 * DStarlite.h
 *
 *  Created on: 19.02.2016
 *      Author: m1ch1
 */

#ifndef DSTAR_LITE_H_
#define DSTAR_LITE_H_


#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <stdexcept>
#include <algorithm>
#include <limits>

#include <memory>

#include <rona_lib/Map/Map.h>

namespace rona
{
namespace planner
{


using f_type = double;
using k_type = std::vector<f_type>;


class DNode// : public apps::Element_base<unsigned int>
{
public:
   DNode() : h(0),
      g(0),
      rhs(0),
      key(2),
      isExpanded(false),
      isDeleted(false)
   {
      key[0] = 0;
      key[1] = 0;
   }
   /**
    * copy constructor
    * @param node to copy
    */
   DNode(DNode* node) : key(2)
   {
      h = node->h;
      g = node->g;
      rhs = node->rhs;
      key[0] = node->key[0];
      key[1] = node->key[1];
      mapNode = node->mapNode;
      isExpanded = node->isExpanded;
      isDeleted = node->isDeleted;
      nodes = node->nodes;
      dist_nodes = node->dist_nodes;
   }

   f_type h;
   f_type g;
   f_type rhs;
   k_type key;

   map::Node mapNode;
   bool isExpanded;
   bool isDeleted;
   std::vector<unsigned int> nodes;
   std::vector<f_type> dist_nodes;
};


class DComp{
public:
   bool operator() (const std::shared_ptr<DNode>& l, const std::shared_ptr<DNode>& r)
   {
      return(l->key[0] > r->key[0] || (this->isEqual(l->key[0],r->key[0]) && l->key[1] >= r->key[1]) );
   }
   inline bool firstLesser(const k_type& l, const k_type& r)
   {
      return(l[0] < r[0] || (this->isEqual(l[0],r[0]) && l[1] < r[1]) );
   }

};


class DStar_lite
{
public:
   DStar_lite() : _k(0), _start_id(0), _end_id(0)
   {

   }
   virtual ~DStar_lite()
   {
      //todo
   }

   inline std::weak_ptr<map::Map> getMap() const
   {
      return _map;
   }

   inline void setMap(const std::weak_ptr<map::Map> map)
   {
      if(map.expired())
      {
         std::cerr << "DStar_lite -> invalid map given at setMap()" << std::endl;
         return;
      }
      auto smap = map.lock();
      if(!_map)
      {//first init
         _nodeList.resize(smap->getSize());
         _openL_vec.resize(smap->getSize());
      }
      _map = smap;
   }

   inline void init(map::Node start, map::Node end)
   {
      auto node = std::make_shared<DNode>();
      node->g   = _INF * 0.9;
      node->rhs = _INF;
      node->isExpanded = false;
      node->mapNode = start;
      _start_id = node->mapNode.id;
      _nodeList[_start_id] = node;

      node = this->initNode();
      node->rhs = 0;
      node->g   = _INF;
      node->key = this->compute_key();//todo
      _end_id = node->mapNode.id;
      _nodeList[_end_id] = node;

      _openL.push(_nodeList[_end_id]);
      _openL_vec[_end_id] = node;
   }

   inline void updateNode(std::shared_ptr<DNode> node)
   {

   }

   inline void computeShortestPath()
   {

   }

   inline bool replan(unsigned int id_newPos)
   {

   }

   inline map::Path tracePath()
   {

   }

private: //fct

   inline f_type compute_h(const std::shared_ptr<DNode>& node, const std::shared_ptr<DNode>& end)
   {
      //euclidian distance
      map::Point2D a = node->mapNode.pos;
      map::Point2D b = end->mapNode.pos;

      f_type x = b.x - a.x;
      f_type y = b.y - a.y;

      return std::sqrt(x*x + y*y);
   }

   inline f_type compute_rhs(std::shared_ptr<DNode>& node)
   {
      //todo Hier weiter machen
      //rhs := (minimum g value vom all child-nodes) + costs to move to it
      f_type min_g = _INF;
      unsigned int id = -1;

      if(!node->isExpanded)
         this->expandNode(node);

      for(unsigned int i=0; i<node->nodes.size(); ++i)
      {
         if(_nodeList[node->nodes[i]]->g < min_g)
         {
            min_g = _nodeList[node->nodes[i]]->g;
            id = i;
         }
      }

      if(id > node->dist_nodes.size())
      {//return _INF is important -> nodes with _INF rhs value will not be pushed in openList
         //std::cout << "debug: no valid nodes around" << std::endl;
         return _INF;
      }

      f_type dings = min_g + node->dist_nodes[id];
      return dings;

   }

   inline k_type compute_key()
   {

   }

   inline void expandNode(std::shared_ptr<DNode>& node)
   {

   }

   inline std::shared_ptr<DNode> initNode(map::Node mapNode)
   {
      std::shared_ptr<DNode> node;

      //todo

      return node;
   }


private: //data elements
   const f_type _INF = std::numeric_limits<f_type>::max();

   std::shared_ptr<map::Map> _map;

   // or as deque not vector
   //std::priority_queue<std::shared_ptr<DNode>, std::vector<std::shared_ptr<DNode>>, DComp> _openL;
   std::priority_queue<std::shared_ptr<DNode>, std::deque<std::shared_ptr<DNode>>, DComp> _openL;
   std::vector<std::shared_ptr<DNode>> _openL_vec;

   std::vector<std::shared_ptr<DNode>> _nodeList; ///< contains all nodes by ID

   unsigned int _start_id;
   unsigned int _end_id;

   //unsigned int _cnt;

   DComp _comp;
};

































} /* namespace planner */
} /* namespace rona */

#endif /* DSTAR_LITE_H_ */
