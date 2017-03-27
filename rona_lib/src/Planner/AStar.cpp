/*
 * Astar.cpp
 *
 *  Created on: 22.12.2015
 *      Author: m1ch1
 */

#include <rona_lib/Planner/AStar.h>

namespace rona
{
namespace planner
{

AStar::AStar()
{
   _replan_rdy = false;
}

AStar::AStar(std::weak_ptr<map::Map> map)
{
   if(map.expired())
   {
      std::cerr << "AStarConstuctor: no valid map given... exit()" << std::endl;
      exit(EXIT_FAILURE);
   }
   else
   {
      _map = map.lock();
   }
   _replan_rdy = false;
}

AStar::~AStar()
{
   // TODO Auto-generated destructor stub
}

std::vector<map::Node> AStar::computePath(const map::Node n_start, const map::Node n_end)
{
   if(!_map)
   {
      std::cerr << "AStar Planner->computePath: no valid Map given" << std::endl;
      return std::vector<map::Node>(0);
   }

   //save targetNode for replan fct
   _current_target = n_end;
   _replan_rdy = true;

   ///openlist as std::priorityqueue
   //std::priority_queue<std::shared_ptr<Node_astar>, std::deque<std::shared_ptr<Node_astar>>, Comp>  openList;
   std::priority_queue<std::shared_ptr<Node_astar>, std::vector<std::shared_ptr<Node_astar>>, Comp>  openList;
   //std::map<unsigned int, std::shared_ptr<Node_astar>> openList_map; ///< for proving if node is on openlist
   //std::unordered_map<unsigned int, std::shared_ptr<Node_astar>,KeyHash> openList_map; ///< for proving if node is on openlist
   //std::unordered_map<unsigned int, std::shared_ptr<Node_astar>> openList_map;//_map->getSize()); ///< for proving if node is on openlist
   ///closedList as std::map
   //std::map<unsigned int, std::shared_ptr<Node_astar>> closedList;
   //std::unordered_map<unsigned int, std::shared_ptr<Node_astar>> closedList;//_map->getSize());
   //std::unordered_map<unsigned int, std::shared_ptr<Node_astar>,KeyHash> closedList;//_map->getSize());

   //testVec
   std::vector<std::shared_ptr<Node_astar>> openList_map(_map->getSize());
   std::vector<std::shared_ptr<Node_astar>> closedList(_map->getSize());

   //std::cout << "size: " << _map->getSize() << std::endl;

   std::shared_ptr<Node_astar> start(new Node_astar);
   std::shared_ptr<Node_astar> end(new Node_astar);
   //spwap start and end
   start->node = n_end;
   start->isInvalid = false;
   end->node   = n_start;
   start->isInvalid = false;

   //debug
   //std::cout << "AStarComputePath - call" << std::endl;
   //std::cout << "start id: " << n_start.id << std::endl;
   //std::cout << "end   id: " << n_end.id << std::endl;

   //init openlist push
   //openList.push(start);
   //openList_map.insert(std::make_pair(start->node.id,start));

   //no init push on openlist ... init currNode with first element
   std::shared_ptr<Node_astar> currNode = start;

   unsigned int cnt = 0;

   while(1)
   {
      cnt++;
      if(currNode->node.id == end->node.id)
      {//path found
         //debug
         //std::cout << "debug: Path found, num loop: " << cnt << std::endl;

         std::vector<map::Node> path;

         end = currNode;

         this->tracePath(end,path);

         return path;
      }

      //add currNode in closedList
      //closedList.insert(std::make_pair(currNode->node.id, currNode));
      closedList[currNode->node.id] = currNode;

      //expand Node
      std::vector<map::Node> children = _map->getChildNodes(currNode->node.id);

      //std::cout << "num of childs: " << children.size() << std::endl;

      for(auto e : children)
      {
         //prove closedList
         if(this->contain(closedList,e.id))
         {//nothing to do
            continue;
         }

         //compute g
         std::shared_ptr<Node_astar> tmp_child(new Node_astar);
         tmp_child->node = e;
         tmp_child->isInvalid = false;
         tmp_child->parentId = currNode->node.id;
         tmp_child->parent = currNode;
         tmp_child->parentDist = 0; // todo check if needed
         tmp_child->g = this->compute_g(tmp_child, currNode);

         //prove openlist
         if(this->contain(openList_map, e.id))
         {
            std::shared_ptr<Node_astar> old_node = openList_map.at(tmp_child->node.id);
            if(old_node->g > tmp_child->g)
            {//better then old... -> update old node to invalid and new node
               old_node->isInvalid = true;
               //compute new values of node
               tmp_child->h = this->compute_h(tmp_child, end); // todo check if needed ?? may take from old one
               tmp_child->f = this->compute_f(tmp_child);
               //replace old node with new
               openList_map.at(tmp_child->node.id) = tmp_child;
               //push openlist
               openList.push(tmp_child);
            }
            else
            {//not better then old...
               //do noting
            }
         }
         else
         {//else -> unknown node ... push openlist
            //push new node
            tmp_child->h = this->compute_h(tmp_child, end);
            tmp_child->f = this->compute_f(tmp_child);
            //openList_map.insert(std::make_pair(tmp_child->node.id, tmp_child));
            openList_map[tmp_child->node.id] = tmp_child;
            openList.push(tmp_child);
         }
      }

      //std::cout << "debug: << openlist size: " << openList.size() << std::endl;

      //get next node
      if(!openList.empty())
      {
         do{
            currNode = openList.top();
            openList.pop();
         }while(currNode->isInvalid && !openList.empty());
         //valid openlist Node -> erase from map
         //openList_map.erase(currNode->node.id);
         openList_map[currNode->node.id].reset();
      }

      //must proved after top from openlist because openlist can be filled with invalid nodes
      if(openList.empty())
      {//openList empty no Path found
         //debug
         //std::cout << "debug: openList empty... no path found, num loop: " << cnt << std::endl;
         return std::vector<map::Node>(0);
      }

   }//end while

   //should never be reached until now :)
   return std::vector<map::Node>(0);
}

void AStar::setMap(std::weak_ptr<map::Map> map)
{
   if(map.expired())
   {
      std::cerr << "AStarSetMap: no valid map given... exit()" << std::endl;
      exit(EXIT_FAILURE);
   }
   else
   {
      _map = map.lock();
   }

}

} /* namespace planner */
} /* namespace rona */
