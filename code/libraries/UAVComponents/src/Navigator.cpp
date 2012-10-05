/**
* \file Navigator.cpp
* \brief A Navigator object manages the UAV's mission. At the moment, that comprises 
*        keeping the mission parameters (waypoints, tasks to perform at waypoints etc.).
*        and provifing functions to access them.
*
*/


#include "Navigator.h"
#include "navigation.h"
#include <math.h>


/**
* \brief Constructor, returns an instance of Template.
*/
Navigator::Navigator()
{
  // Initialize navigation data.
  this->initNavData(&this->navigationData);
  
  this->hasParameterManager = false;
}

/**
* \brief Destructor.
*/
Navigator::~Navigator()
{
}

/**
* \brief Sets a certain waypoint of the mission to the provided values.
* \param[in] lat - the waypoints longitude [degree], WGS84.
* \param[in] lon - the waypoints latitude [degree], WGS84.
* \param[in] alt - the waypoints altitude [m], WGS84.
* \param[in] task - the task from this waypoint, one of the tasks defined in the 'mavion_navigationTask' enumeration.
* \param[in] hoverTime - the time [s] the UAV is suppsoed to hover at the given waypoint before continuing its mission.
* \param[in] waypointIndex - the index of the waypoint to be defined (starting from 0)
*/
void Navigator::setWaypoint(float lat, float lon, float alt, float hoverTime, mavion_navigationTask taskType, UINT16 waypointIndex){
  // Check if the index fits the waypoint array:
  if(waypointIndex < NB_MAX_WP){
    // Copy waypoint into the waypoint list.
    this->navigationData.latitude[waypointIndex] = lat;
    this->navigationData.longitude[waypointIndex] = lon;
    this->navigationData.altitude[waypointIndex] = alt;
    this->navigationData.timeWP[waypointIndex] = hoverTime;
    this->navigationData.taskType[waypointIndex] = taskType;
    this->navigationData.maxVelocity[waypointIndex] = this->navigationData.defaultMaxVelocity;
    // Since setting a home point in the GCS is not implemented yet, put it between the
    // first and the second waypoint.
    if(waypointIndex == 1){
      // Fixme Not sure if this is always between the 2 points (signs!), but in any case it is
      // sufficiently close for the conversions between global and local coordinates.
      float dLat = fabs(this->navigationData.latitude[1] - this->navigationData.latitude[0])/2;
      float dLon = fabs(this->navigationData.longitude[1] - this->navigationData.longitude[0])/2;
      this->navigationData.latHome = this->navigationData.latitude[0] + dLat;
      this->navigationData.lonHome = this->navigationData.longitude[0] + dLon;
    }
  }
}

/**
* \brief Updates the trajectory. Should be called after mission parameters have been changed.
*/
void Navigator::doTrajectoryAnalysis(){
  trajectoryAnalysis(&this->navigationData);
}

/**
* \brief Keeps the Navigator working.
*/
void Navigator::tick(){
  return;
}

/**
* \brief Gives the Navigator a reference to a ParameterManager object it may use
*        to make its navigation parameters accessible to QGroundControl.
*/
void Navigator::introduceComponent(ParameterManager* p){
  this->parameterManager = p;
  this->hasParameterManager = true;
  // Make the navigation parameters accessible to the ParameterManager
  this->parameterManager->checkInFloat(&(this->navigationData.kpGPS), "nav_kpGPS", 16);
  this->parameterManager->checkInFloat(&(this->navigationData.kdGPS), "nav_kdGPS", 16);
  this->parameterManager->checkInFloat(&(this->navigationData.kiGPS), "nav_kiGPS", 16);
  this->parameterManager->checkInFloat(&(this->navigationData.kd_iGPS), "nav_kdiGPS", 16);
  this->parameterManager->checkInFloat(&(this->navigationData.tau), "nav_tau", 16);
  this->parameterManager->checkInFloat(&(this->navigationData.xsie), "nav_xsie", 16);
  this->parameterManager->checkInFloat(&(this->navigationData.kLine), "nav_kLine", 16); 
  this->parameterManager->checkInFloat(&(this->navigationData.defaultMaxVelocity), "nav_maxVelocity", 16); 
  this->parameterManager->checkInFloat(&(this->navigationData.hoverTimeWPZero), "nav_timeWPZero", 16); 
}

/**
* \brief Sets the navigation data/mission to their default values.
* \param[out] nav - a pointer to the Navigator's navigation data structure.
*/
void Navigator::initNavData(TypeNavigation *nav){
  // Initialize mission item arrays
  for(int i=0; i < NB_MAX_WP; i++){
    nav->taskType[i] = MAVION_NAVTASK_UNDEFINED;
    nav->latitude[i] = 33.3;
    nav->longitude[i] = 44.4;
    nav->altitude[i] = 11.1;
    nav->maxVelocity[i] = 15.0;
    nav->timeWP[i] = 1.0;  
    nav->totalDistance[i] = 1.0;
  }
  
  nav->kpGPS = 3.0;
  nav->kdGPS = 1.6;
  nav->kiGPS = 0.0;
  nav->kd_iGPS = 1.5;
  
  nav->tau = 3.0; // size of transition zone for line following scheme (meters)
  nav->xsie = 55.0; // far field angle of approach to line following (degrees)
  nav->kLine = 0.0;
  
  // Default home position: Toulouse
  nav->latHome = 43.565; 
  nav->lonHome = 1.477;
  
  nav->taskType[0] = MAVION_NAVTASK_HOLD_CURRENT_POSITION;
  nav->latitude[0] = nav->latHome; 
  nav->longitude[0] = nav->lonHome;                                      
  nav->altitude[0] = 2.0;
  nav->maxVelocity[0] = 2.0;
  nav->Vc = 0.0;
  nav->Vcf = 0.0;
  nav->nbTasks = 0; // At the beginning, tasks/waypoints are still to be received from the GCS.
  nav->defaultMaxVelocity = 15.0;
  nav->timeWP[0] = nav->hoverTimeWPZero; 
  nav->taskDone=false;
  nav->previousTask = 0;
  nav->currentTask = 0;
  nav->nextTask = 1;
  nav->nextTask = 1;
  nav->timer = 0;
  nav->altGPS = 0.0;
  nav->altGPSHome = 0.0;
  nav->taskSubProcess = 0;
  nav->totalError = 0.0;
  nav->thd=0.0;
  nav->thp = 0.0;
  nav->thi = 0.0;
  nav->count=0;
  nav->sstar = 0.0;
  nav->hoverTimeWPZero = 30;   // Hover at waypoint zero for 20 s to see if automatic control is working.
  
  // As long as the safety zone cannot be set by the GCS, set it to span both Toulouse
  // and Braunschweig. This way it's no longer a safety zone but does not interfere with the mission.
  nav->yellowAlat = 56.0;
  nav->yellowAlon = -10.0;
  nav->yellowBlat = 36.0;
  nav->yellowBlon = -6.0;
  nav->yellowClat = 40.0;
  nav->yellowClon = 22.0;
  nav->yellowDlat = 58.0;
  nav->yellowDlon = 26.0;
  nav->redAlat = nav->yellowAlat;
  nav->redAlon = nav->yellowAlon;
  nav->redBlat = nav->yellowBlat;
  nav->redBlon = nav->yellowBlon;
  nav->redClat = nav->yellowClat;
  nav->redClon = nav->yellowClon;
  nav->redDlat = nav->yellowDlat;
  nav->redDlon = nav->yellowDlon;
}
