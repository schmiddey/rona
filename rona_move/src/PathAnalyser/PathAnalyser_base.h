/*
 * PathAnalyserbase.h
 *
 *  Created on: 24.10.2014
 *      Author: m1ch1
 */

#ifndef _PATHANALYSERBASE_H_
#define _PATHANALYSERBASE_H_

#include <vector>
#include <Eigen/Dense>
//#define EIGEN_USE_NEW_STDVECTOR
//#include <Eigen/StdVector>

using namespace Eigen;

namespace analyser
{

namespace cfg{

struct AnalyserBase_config{
  //general
  double target_radius         = 0.24;
  double target_radius_final   = 0.1;
  double lin_end_approach      = 0.5;
  double ang_reached_range     = 0.008; // aprox 0.5°
  //mecanum
  //todo
//  bool hold_pos                = true;  //for now only in mecanum but should be also in general (also differential)
  double wait_for_rotation     = 4.0;

  //differential
  //todo

};

}


//enum enum_state {
//   IDLE = 0,
//   REACHED_GOAL,
//   MOVING,
//   ABORTED
//};

struct diff_scale{
  double linear_x;
  double linear_y;
  double angular;

  diff_scale() :
    linear_x(0),
    linear_y(0),
    angular(0)
  { }
};

typedef struct {
  Vector3d position;
  Vector3d orientation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} pose;

typedef struct {
  double path_length;
  double path_length_remaining;
  unsigned int num_goals;
  unsigned int current_goal_id;
  //analyser::enum_state state;
  bool reached_goal;
} info;


/**
 * @todo add some kind of update cfg for path controller (allow dyn recfg...)
 */
class PathAnalyser_base
{
public:
  /**
   * @brief constructor
   */
  PathAnalyser_base();

  /**
   * @brief destructor
   */
  virtual ~PathAnalyser_base() { }

  /**
   * @brief set new path
   *
   * @param path -> to analyse incrementelly as R-Value
   *
   * @return void
   */
  void setPath(std::vector<analyser::pose>&& path);

  /**
   * @brief set new path
   *
   * @param path -> to analyse incrementelly as Copy
   *
   * @return void
   */
  void setPath(const std::vector<analyser::pose>& path)
  {
    //do a cpy before...
    auto tmp_cpy = path;
    this->setPath(std::move(tmp_cpy));
  }

  /**
   * @brief clear path
   */
  void clear() { _path.clear(); }


  /**
   * @brief retruns is path is empty -> importatnt if user want to know if empty path was sent
   * @return
   */
  bool isEmpty() { return _path.empty(); }



  /**
   * @param void
   *
   * @return diffscale whitch have to be controled after
   */
  virtual analyser::diff_scale analyse(const analyser::pose& current_pose) = 0;

  virtual void setTargetRadius(const double r, const double r_final) = 0;

  analyser::info getInfo();

  /**
   * @todo write func
   * @return
   */
  static Vector3d quaternion_to_orientationVec(const Quaternion<double>& q) { return q * Vector3d(1,0,0); }

  void setDoEndRotate(const bool doEndRotate) { _do_end_rotate = doEndRotate; }
  bool isDoEndRotate() { return _do_end_rotate; }
  inline bool isReachedFinalGoal() const { return _reached_final_goal; }


protected: //dataelements
  std::vector<analyser::pose> _path;  ///< current path to analyse

protected:  //functions
  inline analyser::pose currentGoal() const { return _path[_currentGoal_index]; }
  void nextGoal();
  inline unsigned int getCurrentGoalIndex() const { return _currentGoal_index; }
  inline bool isLastGoal() const { return _currentGoal_index == _path.size() - 1 ? true : false; }
  inline bool isFirstGoal() const { return _currentGoal_index == 0 ? true : false; }

  inline double getDistToCurrentGoal() const { return _dist_to_current_goal; }
  inline void setDistToCurrentGoal(const double distToCurrentPose) { _dist_to_current_goal = distToCurrentPose; }
  void setReachedFinalGoal(const bool reachedFinalGoal) { _reached_final_goal = reachedFinalGoal; }
  //inline void setState(analyser::enum_state state)
  //{
  //   _state = state;
  //}
  //inline analyser::enum_state getState() { return _state; }
  inline double getPathLengthRest() const { return _path_lenth_rest; }


  inline int getDirection(Vector3d p, Vector3d ori)
  {
    //build cross product to estimate angular direction
    Vector3d directionVec = ori.cross(p);
    double tmp_dir = directionVec(2);
    if(tmp_dir > 0)
      return 1;
    else
      return -1;
  }

private:
  unsigned int _currentGoal_index;
  double _dist_to_current_goal;       ///< must set in analyse() by user of this baseclass
  double _path_lenth;
  double _path_lenth_rest;
  bool _reached_final_goal;           ///< must set in analyse() by user of this baseclass
  bool _do_end_rotate;
  //analyser::enum_state _state;
  //for eigen
//public:
   //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace analyser */

#endif /* _PATHANALYSERBASE_H_ */
