/*
 * Map_types.h
 *
 *  Created on: 16.12.2015
 *      Author: m1ch1
 */

#ifndef MAP_TYPES_H_
#define MAP_TYPES_H_

#include <iostream>
#include <map>
#include <unordered_map>
#include <vector>

namespace rona
{
namespace map
{

struct Pixel{
   unsigned int x;
   unsigned int y;

   Pixel() : x(0), y(0) { }
   Pixel(const unsigned int _x, const unsigned int _y) : x(_x) , y(_y) { }
};

struct PixelSigned{
   int x;
   int y;

   PixelSigned() : x(0), y(0) { }
   PixelSigned(const int _x, const int _y) : x(_x) , y(_y) { }
};

struct Point2D{
   double x;
   double y;
   Point2D() : x(0), y(0) { }
   Point2D(const double _x, const double _y) : x(_x), y(_y) { }
};

struct Polygon{
   std::vector<Point2D> points;
   Polygon() { }
   Polygon(const std::vector<Point2D>& p) { points = p; } //cpy
};


struct RectPixel
{
  Pixel p;
  unsigned int w;
  unsigned int h;
  RectPixel() : p(), w(0), h(0) { }
  RectPixel(const Pixel& _p, const unsigned int _w, const unsigned int _h) :
  p(_p), w(_w), h(_h) { }
};

struct Rect2D{
  Point2D p;
  double w;
  double h;
  Rect2D() : p(), w(0), h(0) { }
  Rect2D(const Point2D& _p, const unsigned int _w, const unsigned int _h) :
  p(_p), w(_w), h(_h) { }
};


struct Edge{
   unsigned int id;
   double cost;
   double distance;
};

struct Node{
   unsigned int id;
   std::map<unsigned int, Edge> edges_map;
   //std::unordered_map<unsigned int, Edge> edges_map;

   Point2D pos;
};

using Path = std::vector<Node>;


//Overload output operator
inline std::ostream& operator<<(std::ostream& os, const rona::map::Point2D& p)
{
   os << "(" << p.x << ", " << p.y << ")";
   return os;
}

inline std::ostream& operator<<(std::ostream& os, const rona::map::Pixel& p)
{
   os << "(" << p.x << ", " << p.y << ")";
   return os;
}

inline std::ostream& operator<<(std::ostream& os, const rona::map::PixelSigned& p)
{
   os << "(" << p.x << ", " << p.y << ")";
   return os;
}

inline std::ostream& operator<<(std::ostream& os, const rona::map::Polygon& p)
{
   for(const auto& e : p.points)
   {
      os << "( " << e << " )";
   }
   return os;
}

inline std::ostream& operator<<(std::ostream& os, const rona::map::RectPixel& r)
{
   os << "(p:" << r.p << ", w: " << r.w << ", h: " << r.h << ")" ;
   return os;
}

inline std::ostream& operator<<(std::ostream& os, const rona::map::Rect2D& r)
{
   os << "(p:" << r.p << ", w: " << r.w << ", h: " << r.h << ")" ;
   return os;
}

inline std::ostream& operator<<(std::ostream& os, const rona::map::Path& p)
{
   os << "Path.size(): " << p.size() << " ,Nodes.pos: " << std::endl;
   for(const auto& e : p)
   {
      os << e.pos << std::endl;
   }

   return os;
}

} /* namespace map */
} /* namespace rona */






#endif /* MAP_TYPES_H_ */
