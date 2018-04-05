/*
 * MapOperations.cpp
 *
 *  Created on: 29.12.2014
 *      Author: m1ch1
 */

#include <iostream>

#include <rona_lib/Map/Operations.h>
//#include <opencv2/opencv.hpp>

namespace rona
{
namespace map
{

void Operations::binarize(Grid& grid,
                          uint8_t min,
                          uint8_t max,
                          uint8_t vel_in,
                          uint8_t vel_out)
{
  std::vector<uint8_t>& g = grid.getData();

  for(unsigned int i = 0; i < g.size(); ++i)
  {
    if (g[i] >= min && g[i] <= max) //in range
    {
      g[i] = vel_in;
    }
    else //out range
    {
      g[i] = vel_out;
    }
  }
}

void Operations::inflateRect(Grid& grid, uint8_t val_min, uint8_t val_max, double offset)
{
  //must be int...
  int num = grid.getCellCnt();
  int width = grid.getWidth();
  int height = grid.getHeight();

  uint8_t offset_ = std::round(offset / (double)grid.getCellSize());

  std::vector<uint8_t>& target_grid = grid.getData();
  //copy
  std::vector<uint8_t> tmp_grid = target_grid;

  //read tmp_map ... write origin map
  for(int i=0; i < (int)num; ++i)
  {
    //prove if surrounded by val ... after testing .. very efficient
    if(i > width && i < (width * (height - 1)) &&
      tmp_grid[i - 1] >= val_min && tmp_grid[i - 1] <= val_max &&
      tmp_grid[i + 1] >= val_min && tmp_grid[i + 1] <= val_max &&
      tmp_grid[i - width] >= val_min && tmp_grid[i - width] <= val_max &&
      tmp_grid[i + width] >= val_min && tmp_grid[i + width] <= val_max)
      continue;

    if(tmp_grid[i] >= val_min && tmp_grid[i] <= val_max) // get val points
    {
      int x = i % width;
      int y = i / width;
      for(int yi = y - offset_; yi < y + offset_; ++yi)
      {
        for(int xi = x - offset_; xi < x + offset_; ++xi)
        {
          int idx = yi * width + xi;
          if (idx >= 0 && idx < num)
            target_grid[idx] = tmp_grid[i];
        }
      }
    }
  }
}

void Operations::distnaceTransformRect(Grid &grid, double offset, uint8_t wall_val)
{
  //must be int...
  int num = grid.getCellCnt();
  int width = grid.getWidth();
  int height = grid.getHeight();

  uint8_t offset_ = std::round(offset / (double)grid.getCellSize());

  std::vector<uint8_t>& dat = grid.getData();

  //compute loockuptable:
  int x = offset_;                                                   //middle point
  int y = offset_;                                                   //middle point
  std::vector<uint8_t> lt((2 * offset_ + 1) * (2 * offset_ + 1), 0); //init with 0

  for(unsigned int off = 0; off <= offset_; ++off)
  {
    int val = (offset_ - off) * (255 / offset_);
    val = (val > 254) ? 254 : val;
    for(unsigned int yi = y - off; yi < y + off; ++yi)
    {
      for(unsigned int xi = x - off; xi < x + off; ++xi)
      {
        int idx = yi * (2 * offset_ + 1) + xi;
        lt[idx] = (lt[idx] > val) ? lt[idx] : val;
      }
    }
  }

  //do distance transform:
  for(int i = 0; i < num; ++i)
  {
    if(dat[i] == wall_val) // get wall points
    {
      int x = i % width;
      int y = i / width;

      //prove if surrounded by wall ... after testing .. very efficient
      if(i > width && i < (width * (height - 1)) && 
        dat[i - 1] == wall_val && 
        dat[i + 1] == wall_val && 
        dat[i - width] == wall_val && 
        dat[i + width] == wall_val)
        continue;

      for(int yi = y - offset_, ylt = 0; yi < y + offset_; ++yi, ++ylt)
      {
        int idx_ = yi * width;
        int idxlt_ = ylt * (2 * offset_ + 1);
        for(int xi = x - offset_, xlt = 0; xi < x + offset_; ++xi, ++xlt)
        {
          int idx = idx_ + xi;
          int idxlt = idxlt_ + xlt;

          if(idx >= 0 && idx < num)
          {
            if(dat[idx] < lt[idxlt])
            {
              dat[idx] = lt[idxlt];
            }
          }
        }
      }
    }
  }
}

void Operations::inflateCirc(Grid& grid, uint8_t val_min, uint8_t val_max, double offset)
{
  int num = grid.getCellCnt();
  int width = grid.getWidth();
  int height = grid.getHeight();

  int offset_ = std::round(offset / grid.getCellSize());

  std::vector<uint8_t>& target_grid = grid.getData();
  //copy
  std::vector<uint8_t> tmp_grid = target_grid;

  //compute loockuptable:
  int x = offset_; //middle point
  int y = offset_; //middle point

  unsigned int width_lt = 2 * offset_ + 1;
  std::vector<uint8_t> lt(width_lt * width_lt, 0); //init with 0

  Pixel c(x, y);

  //draw filled circ in lt
  for(unsigned int i = 0; i < width_lt * width_lt; ++i)
  {
    Pixel p = Operations::idxToPixel(i, width_lt);
    //prove if point is in circ
    if (((int)p.x - x) * ((int)p.x - x) + ((int)p.y - y) * ((int)p.y - y) < ((offset_ + 1) * (offset_ + 1)))
    {
      lt[i] = val_max;
    }
  }

  //debug
  // cv::Mat tmp_img(cv::Size(width_lt, width_lt),
  //                 CV_8UC1,
  //                 (unsigned char*) &lt[0], //address of first vector element
  //                 cv::Mat::AUTO_STEP);
  // cv::imwrite("/tmp/lt.png", tmp_img);

  //do inflation
  for(int i = 0; i < num; ++i)
  {
    if(tmp_grid[i] >= val_min && tmp_grid[i] <= val_max) // get wall points
    {
      int x = i % width;
      int y = i / width;

      //prove if surrounded by val ... after testing .. very efficient
      if(i > width && i < (width * (height - 1)) &&
         tmp_grid[i - 1] >= val_min && tmp_grid[i - 1] <= val_max &&
         tmp_grid[i + 1] >= val_min && tmp_grid[i + 1] <= val_max &&
         tmp_grid[i - width] >= val_min && tmp_grid[i - width] <= val_max &&
         tmp_grid[i + width] >= val_min && tmp_grid[i + width] <= val_max)
        continue;

      for(int yi = y - offset_, ylt = 0; yi < y + offset_ + 1; ++yi, ++ylt)
      {
        int idx_ = yi * width;
        int idxlt_ = ylt * (2 * offset_ + 1);
        for(int xi = x - offset_, xlt = 0; xi < x + offset_ + 1; ++xi, ++xlt)
        {
          int idx = idx_ + xi;
          int idxlt = idxlt_ + xlt;

          if(idx >= 0 && idx < num)
          {
            if(tmp_grid[idx] < lt[idxlt])
            {
              target_grid[idx] = lt[idxlt];
            }
          }
        }
      }
    }
  }

  //   cv::Mat mat_lt(cv::Size(width_lt, width_lt),
  //                  CV_8UC1,
  //                  (unsigned char*) lt,
  //                  cv::Mat::AUTO_STEP);
  //
  //   cv::imwrite("/tmp/lt_circ.png", mat_lt);
}

void Operations::distnaceTransformCirc(Grid& grid, double offset, uint8_t wall_val)
{
  int num = grid.getCellCnt();
  int width = grid.getWidth();
  int height = grid.getHeight();

  int offset_ = std::round(offset / (double)grid.getCellSize());

  if(offset_ == 0)
  {//nothing to do
     return;
  }

  std::vector<uint8_t>& target_grid = grid.getData();
  //copy
  std::vector<uint8_t> tmp_grid = target_grid;

  //compute loockuptable:
  int x = offset_;   //middle point
  int y = offset_;   //middle point
  unsigned int width_lt = 2 * offset_ +1;
  std::vector<uint8_t> lt(width_lt * width_lt, 0); //init with 0

  Pixel c(x,y);

  for(unsigned int i = 0; i < width_lt * width_lt; ++i)
  {
    Pixel p = Operations::idxToPixel(i, width_lt);
    //prove if point is in circ
    if(((int)p.x - x) * ((int)p.x - x) + ((int)p.y - y) * ((int)p.y - y) < ((offset_ + 1) * (offset_ + 1)))
    //if((p.x - x)*(p.x - x) + (p.y - y)*(p.y - y) < ((offset_+1)*(offset_+1)))
    {
      //get euclidean distance and normalize to 1
      double x_d = x - (int)p.x;
      double y_d = y - (int)p.y;

      double d = ::sqrt(x_d * x_d + y_d * y_d);

      d /= std::round((double)(offset_ + 1));
      //std::cout << "-> d: " << d << std::endl;
      lt[i] = (uint8_t)(255 * (1 - d));
    }
  }

  //   //debug
  //   cv::Mat tmp_img(cv::Size(width_lt, width_lt),
  //                   CV_8UC1,
  //                   (unsigned char*) &lt[0], //address of first vector element
  //                   cv::Mat::AUTO_STEP);
  //   cv::imwrite("/tmp/lt.png", tmp_img);

  //do dt
  for(int i = 0; i < num; ++i)
  {
    if(tmp_grid[i] == wall_val) // get wall points
    {
      int x = i % width;
      int y = i / width;

      //prove if surrounded by wall ... after testing .. very efficient
      if(i > width && i < (width * (height - 1)) &&
         target_grid[i - 1] == wall_val &&
         target_grid[i + 1] == wall_val &&
         target_grid[i - width] == wall_val &&
         target_grid[i + width] == wall_val)
        continue;

      for(int yi = y - offset_, ylt = 0; yi < y + offset_ + 1; ++yi, ++ylt)
      {
        int idx_ = yi * width;
        int idxlt_ = ylt * (2 * offset_ + 1);
        for(int xi = x - offset_, xlt = 0; xi < x + offset_ + 1; ++xi, ++xlt)
        {
          int idx = idx_ + xi;
          int idxlt = idxlt_ + xlt;

          if(idx >= 0 && idx < num)
          {
            if(target_grid[idx] < lt[idxlt])
            {
              target_grid[idx] = lt[idxlt];
            }
          }
        }
      }
    }
  }

  //   cv::Mat mat_lt(cv::Size(width_lt, width_lt),
  //                  CV_8UC1,
  //                  (unsigned char*) lt,
  //                  cv::Mat::AUTO_STEP);
  //
  //   cv::imwrite("/tmp/lt_dt_circ.png", mat_lt);
}

void Operations::drawFilledCircle(Grid& grid, Point2D circ_center, double radius,  uint8_t circ_value)
{
  Pixel circ_c = grid.toPixel(circ_center);

  unsigned int rad = std::abs(std::round(radius / (double)grid.getCellSize()));

  //define rect around circle:
  unsigned int xc = (circ_c.x <= rad) ? 0 : circ_c.x - rad;
  unsigned int yc = (circ_c.y <= rad) ? 0 : circ_c.y - rad;
  unsigned int wc = 2 * (rad + 1);

  //iterate over all rect pixel an prove if they are in circle
  for(unsigned int y = yc; y < (wc + yc); ++y)
  {
    for(unsigned int x = xc; x < (wc + xc); ++x)
    {
      //prove if it is inside circ
      if((x - circ_c.x) * (x - circ_c.x) + (y - circ_c.y) * (y - circ_c.y) < (rad * rad))
      {
        Pixel p;
        p.x = x;
        p.y = y;

        unsigned int idx = grid.toIdx(p);
        if(idx < grid.getWidth() * grid.getHeight())
          grid.getData()[idx] = circ_value;
      }
    }
  }
}

void Operations::overdrawFilledCircle(Grid& grid,
                                      Point2D circ_center,
                                      double radius,
                                      uint8_t circ_value,
                                      uint8_t min_draw,
                                      uint8_t max_draw)
{
  Pixel circ_c = grid.toPixel(circ_center);

  unsigned int rad = std::abs(std::round(radius / (double)grid.getCellSize()));

  //define rect around circle:
  unsigned int xc = (circ_c.x <= rad) ? 0 : circ_c.x - rad;
  unsigned int yc = (circ_c.y <= rad) ? 0 : circ_c.y - rad;
  unsigned int wc = 2 * (rad + 1);

  //iterate over all rect pixel an prove if they are in circle
  for(unsigned int y = yc; y < (wc + yc); ++y)
  {
    for(unsigned int x = xc; x < (wc + xc); ++x)
    {
      //prove if it is inside circ
      if((x - circ_c.x) * (x - circ_c.x) + (y - circ_c.y) * (y - circ_c.y) < (rad * rad))
      {
        Pixel p;
        p.x = x;
        p.y = y;

        unsigned int idx = grid.toIdx(p);

        if(idx < grid.getWidth() * grid.getHeight() && grid.getData()[idx] >= min_draw && grid.getData()[idx] <= max_draw)
          grid.getData()[idx] = circ_value;
      }
    }
  }
}


void Operations::drawFilledRect(Grid& grid, Point2D rect_p, double rect_w, double rect_h, uint8_t rect_value)
{
  Pixel p = grid.toPixel(rect_p);
  unsigned int w = std::abs(std::round(rect_w / (double)grid.getCellSize()));
  unsigned int h = std::abs(std::round(rect_h / (double)grid.getCellSize()));

  std::vector<uint8_t>& data = grid.getData();

  for(unsigned int y = p.y; y < (p.y + h); ++y)
  {
    for(unsigned int x = p.x; x < (p.x + w); ++x)
    {
      Pixel tmp;
      tmp.x = x;
      tmp.y = y;
      unsigned int idx = grid.toIdx(tmp);

      if(x >= 0 && x < grid.getWidth() && y >= 0 && y < grid.getHeight())
      {
        data[idx] = rect_value;
      }
    }
  }
}



void Operations::overdrawFilledRect(Grid& grid,
                                    Point2D rect_p,
                                    double rect_w,
                                    double rect_h,
                                    uint8_t rect_value,
                                    uint8_t min_draw,
                                    uint8_t max_draw)
{
  Pixel p = grid.toPixel(rect_p);
  unsigned int w = std::abs(std::round(rect_w / (double)grid.getCellSize()));
  unsigned int h = std::abs(std::round(rect_h / (double)grid.getCellSize()));

  for(unsigned int y = p.y; y < (p.y + h); ++y)
  {
    for(unsigned int x = p.x; x < (p.x + w); ++x)
    {
      Pixel tmp;
      tmp.x = x;
      tmp.y = y;
      unsigned int idx = grid.toIdx(tmp);

      if(x >= 0 && x < grid.getWidth() && y >= 0 && y < grid.getHeight() && grid.getData()[idx] >= min_draw && grid.getData()[idx] <= max_draw)
      {
        grid.getData()[idx] = rect_value;
      }
    }
  }
}



std::shared_ptr<Grid> Operations::diff(const Grid& grid_1, const Grid& grid_2)
{
   if(!grid_1.isCompatible(grid_2))
   {
      std::cerr << "diff() grids not compaible" << std::endl;
      return std::shared_ptr<Grid>(new Grid);
   }

   //cpy grid
   std::shared_ptr<Grid> grid(new Grid(grid_2));
   const std::vector<uint8_t>& g1 = grid_1.getData();
   const std::vector<uint8_t>& g2 = grid_2.getData();
   std::vector<uint8_t>& gx = grid->getData();
   for(unsigned int i=0; i<g1.size(); ++i)
   {
      if(g1[i] != g2[i])
         gx[i] = 255;
      else
         gx[i] = 0;
   }
   return grid;
}


double Operations::computeDistance(Point2D a, Point2D b)
{
   double x = b.x - a.x;
   double y = b.y - a.y;

   double ret = std::sqrt(x*x + y*y);

//   if(ret > 0.5)
//   {
//      std::cout << "-- dist: " << ret << " , xa: " << a.x  << " , ya: " << a.y <<" , xb: " << b.x << " , yb: " << b.y << std::endl;
//   }

   return ret;
}

double Operations::computePathLength(const Path& path, const unsigned int start_idx)
{
   double path_length = 0.0;
   if(!path.size())
      return path_length;
   if(start_idx >= path.size() - 1)
   {
      return path_length;
   }

   Point2D tmp_p = path[start_idx].pos;
   for(unsigned int i=start_idx + 1; i<path.size(); ++i)
   {
      double dings = Operations::computeDistance(tmp_p, path[i].pos);
      //std::cout << "----" << i <<  " << curr_l: " << path_length << " , dist: " << dings << std::endl;
      path_length += dings;
      tmp_p = path[i].pos;
   }

   return path_length;
}

void Operations::drawFilledPolygon(Grid& grid, const Polygon& polygon, const uint8_t pol_value)
{
   Rect2D rect = getBoundingRect(polygon);

   Pixel p = grid.toPixel(rect.p);
   unsigned int w = std::abs(std::round(rect.w / (double)grid.getCellSize()));
   unsigned int h = std::abs(std::round(rect.h / (double)grid.getCellSize()));

   std::vector<uint8_t>& data = grid.getData();

   for(unsigned int y = p.y; y < (p.y + h); ++y)
   {
      for(unsigned int x = p.x; x < (p.x + w); ++x)
      {
         Pixel tmp;
         tmp.x = x;
         tmp.y = y;
         unsigned int idx = grid.toIdx(tmp);

         //todo maybe find better way... (prove point in polygon)
         if(x >= 0 && x < grid.getWidth() && y >= 0 && y < grid.getHeight() && pointInPolygon(polygon, grid.toPoint2D(tmp)))
         {
            data[idx] = pol_value;
         }
      }
   }


}

bool Operations::pointInPolygon(const Polygon& polygon, const Point2D& p)
{
//   int i, j, c = 0;
//   for(i = 0, j = nvert-1; i < nvert; j = i++) {
//     if ( ((verty[i]>testy) != (verty[j]>testy)) &&
//     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
//        c = !c;
//   }
//
   unsigned int i,m;
   bool c = false;
   unsigned int size = polygon.points.size();
   const std::vector<Point2D>& ps = polygon.points;
   for(i=0, m=size-1; i<size; m=i++)
   {
      if( ((ps[i].y > p.y) != (ps[m].y > p.y)) &&
          (p.x < (ps[m].x - ps[i].x) * (p.y - ps[i].y) / (ps[m].y - ps[i].y) + ps[i].x) )
         c = !c;
   }
   return c;
}

Rect2D Operations::getBoundingRect(const Polygon& polygon)
{
   Rect2D rect;

   double min_x = std::numeric_limits<double>::max();
   double min_y = std::numeric_limits<double>::max();
   double max_x = std::numeric_limits<double>::max() * -1.0;
   double max_y = std::numeric_limits<double>::max() * -1.0;

   for(auto e : polygon.points)
   {
      if(min_x > e.x)
         min_x = e.x;
      if(min_y > e.y)
         min_y = e.y;
      if(max_x < e.x)
         max_x = e.x;
      if(max_y < e.y)
         max_y = e.y;
   }

   rect.p.x = min_x;
   rect.p.y = min_y;
   rect.w = max_x - min_x;
   rect.h = max_y - min_y;

   return rect;
}



Point2D Operations::computeCentroid(const Polygon& polygon)
{
   //just implemented formula...
   Point2D center;
   if(polygon.points.empty())
      return center;

   Polygon p = polygon;
   p.points.push_back(p.points[0]); //make last element as same as first element... needed for algorithm
   //std::reverse(p.points.begin(),p.points.end());
   double xs = 0;
   double ys = 0;
   for(unsigned int i=0; i<p.points.size() - 1; ++i)
   {
      double fac = (p.points[i].x * p.points[i+1].y - p.points[i+1].x * p.points[i].y);
      xs += (p.points[i].x + p.points[i+1].x) * fac;
      ys += (p.points[i].y + p.points[i+1].y) * fac;
   }

   double area = computeArea(polygon);
   if(area == 0)
      return center;
   center.x = xs / (area * 6);
   center.y = ys / (area * 6);

   //std::cout << "debug: center:" << center << std::endl;

   return center;
}

double Operations::computeArea(const Polygon& polygon)
{
   //Gauss'sche Dreiecksformel, just implemented formula...
   double area = 0;
   if(polygon.points.empty())
         return area;
   Polygon p = polygon;
   p.points.push_back(p.points[0]); //make last element as same as first element... needed for algorithm
   //std::reverse(p.points.begin(),p.points.end());
   for(unsigned int i=0; i<p.points.size() - 1; ++i)
   {
      area += p.points[i].x * p.points[i+1].y - p.points[i+1].x * p.points[i].y;
   }
   //std::cout << "debug: area: " << area * 0.5 << std::endl;
   return std::abs(area * 0.5);
}

Polygon Operations::scale(const Polygon& polygon, double scale_fac)
{
   Polygon scaledPol;

   Point2D c = computeCentroid(polygon);

   Eigen::MatrixXd m(2,polygon.points.size());
   int i = 0;
   for(auto e : polygon.points)
   {
      //translate by c
      m(0,i)   = e.x - c.x;
      m(1,i++) = e.y - c.y;
   }

   Eigen::Matrix2d aff;
   aff(0,0) = (scale_fac);
   aff(1,1) = (scale_fac);

   Eigen::MatrixXd res = aff * m;

   for(unsigned int i=0; i<polygon.points.size(); ++i)
   {
      Point2D p;
      p.x = res(0,i) + c.x;
      p.y = res(1,i) + c.y;

      scaledPol.points.push_back(p);
   }

   return scaledPol;
}

} /* namespace map */
} /* namespace rona */


