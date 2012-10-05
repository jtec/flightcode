/**
* \file Navigator.h
* \brief 
*
*/

#ifndef _NAVIGATOR_H_
#define _NAVIGATOR_H_

#include "declarationsGlobales.h"
#include "ParameterManager.h"


class Navigator{
public :
  Navigator();
  ~Navigator();
   void tick();
   TypeNavigation navigationData;   // Holds mission parameters (waypoints, safety zones etc.)
   void setWaypoint(float lat, float lon, float alt, float hoverTime, mavion_navigationTask taskType, UINT16 waypointIndex);
   void doTrajectoryAnalysis();
   void introduceComponent(ParameterManager* p);
private:
  void initNavData(TypeNavigation *nav);
  ParameterManager* parameterManager;
  bool hasParameterManager;
};

#endif  // _NAVIGATOR_H_
