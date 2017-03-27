/*
 * RoadMap.h
 *
 *  Created on: 09.12.2015
 *      Author: m1ch1
 */

#ifndef ROADMAP_H_
#define ROADMAP_H_

#include <rona_lib/Map/Map.h>

namespace rona
{
namespace map
{

class RoadMap: public Map
{
public:
   RoadMap();
   virtual ~RoadMap();

   virtual std::vector<Node> getChildNodes(unsigned int id);
   virtual void serialize(std::string file);
   virtual void load(std::string file);
   virtual unsigned int getSize()
   {
      return 0;
   }

   //todo add functions for manipulate graph add ...


private:
   std::vector<Node> _graph;

};

} /* namespace map */
} /* namespace rona */

#endif /* ROADMAP_H_ */
