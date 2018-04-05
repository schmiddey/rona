/*
 * MapOperations.h
 *
 *  Created on: 29.12.2014
 *      Author: m1ch1
 */

#ifndef ALGORITHM_PATHFIND_MAP_MAPOPERATIONS_MAPOPERATIONS_H_
#define ALGORITHM_PATHFIND_MAP_MAPOPERATIONS_MAPOPERATIONS_H_

#include <iostream>
#include <memory>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include <rona_lib/Map/map_types.h>
#include <rona_lib/Map/Map.h>
#include <rona_lib/Map/GridMap.h>

namespace rona
{
namespace map
{

//forward declarations;
//class GridMap;
//struct Pixel;


class Operations
{
public:
   Operations() { }
   virtual ~Operations() { }
   static void binarize(Grid& grid, uint8_t min, uint8_t max, uint8_t vel_in, uint8_t vel_out);

   static void inflateRect(Grid& grid, uint8_t val_min, uint8_t val_max, double offset);
   static void distnaceTransformRect(Grid& grid, double offset, uint8_t wall_val);

   static void inflateCirc(Grid& grid, uint8_t val_min, uint8_t val_max, double offset);
   static void distnaceTransformCirc(Grid& grid, double offset, uint8_t wall_val);

   static void drawFilledCircle(Grid& grid, Point2D circ_center, double radius, uint8_t circ_value);
   static void overdrawFilledCircle(Grid& grid, Point2D circ_center, double radius, uint8_t circ_value, uint8_t min_draw, uint8_t max_draw);
   static void drawFilledRect(Grid& grid, Point2D rect_p, double rect_w, double rect_h, uint8_t rect_value);
   static void overdrawFilledRect(Grid& grid, Point2D rect_p, double rect_w, double rect_h, uint8_t rect_value, uint8_t min_draw, uint8_t max_draw);

   static std::shared_ptr<Grid> diff(const Grid& grid_1, const Grid& grid_2);

   static double computeDistance(Point2D a, Point2D b);

   //check maybe in other class
   static double computePathLength(const Path& path, const unsigned int start_idx = 0);

   static void drawFilledPolygon(Grid& grid, const Polygon& polygon, const uint8_t pol_value);
   static bool pointInPolygon(const Polygon& polygon, const Point2D& p);

   static Rect2D getBoundingRect(const Polygon& polygon);
   static Point2D computeCentroid(const Polygon& polygon);
   static double computeArea(const Polygon& polygon);
   static Polygon scale(const Polygon& polygon, double scale_fac);


   inline static unsigned int pixelToIdx(Pixel pix, unsigned int width)
   {
      return pix.y * width + pix.x;
   }

   inline static Pixel idxToPixel(unsigned int idx, unsigned int width)
   {
      Pixel tmp;
      tmp.x = idx % width;
      tmp.y = idx / width;
      return tmp;
   }

};

} /* namespace map */
} /* namespace rona */

#endif /* ALGORITHM_PATHFIND_MAP_MAPOPERATIONS_MAPOPERATIONS_H_ */
