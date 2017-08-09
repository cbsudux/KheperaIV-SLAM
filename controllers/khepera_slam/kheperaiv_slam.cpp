/* Include the controller definition */
#include "kheperaiv_slam.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include<argos3/core/utility/logging/argos_log.h>
#include <argos3/core/simulator/simulator.h>

/******************************************************/

// SinglePositionSLAM params: gives us a nice-size map
static const int MAP_SIZE_PIXELS        = 800;
static const double MAP_SIZE_METERS     =  32;

static const int SCAN_SIZE 		        = 682;
static const int random_seed              = 9999;

/*****************************************************/

vector<double *> trajectory;

static Real leftOdom = 0.0;
static Real rightOdom = 0.0;

/***************************************************/

CKheperaIVSlam::CKheperaIVSlam() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcLidar(NULL),
   m_pcOdometry(NULL),
   m_pcMapbytes(NULL),
   m_pcSlam(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha))
   {
}

/****************************************/
/****************************************/

class KheperaURG04LX : public URG04LX
{

public:
    KheperaURG04LX(void): URG04LX(
        70,          // detectionMargin
        0 )         // offsetMillimeters
    {
    }
};

class Rover : WheeledRobot
{

public:

    Rover() : WheeledRobot(
         21,     // wheelRadiusMillimeters
        52.7)     // halfAxleLengthMillimeters
    {
    }

    Velocities computeVelocities(long * odometry, Velocities & velocities)
    {
        return WheeledRobot::computeVelocities(
            odometry[0],
            odometry[1],
            odometry[2]);
    }

protected:

    void extractOdometry(
        double timestamp,
        double leftWheelOdometry,
        double rightWheelOdometry,
        double & timestampSeconds,
        double & leftWheelDegrees,
        double & rightWheelDegrees)
    {
        // Convert microseconds to seconds, ticks to angles
        timestampSeconds = timestamp / 1e6;
        leftWheelDegrees = ticksToDegrees(leftWheelOdometry);
        rightWheelDegrees = ticksToDegrees(rightWheelOdometry);
    }

    void descriptorString(char * str)
    {
        sprintf(str, "ticks_per_cycle=%d", this->TICKS_PER_CYCLE);
    }

private:

    double ticksToDegrees(double ticks)
    {
        return ticks * (180. / this->TICKS_PER_CYCLE);
    }

    static const int TICKS_PER_CYCLE = 2000;
};

int coords2index(double x,  double y)
{
    return y * MAP_SIZE_PIXELS + x;
}

int mm2pix(double mm)
{
    return (int)(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS));
}

void CKheperaIVSlam::Init(TConfigurationNode& t_node) {


    m_pcMapbytes = new unsigned char[MAP_SIZE_PIXELS * MAP_SIZE_PIXELS];

    // Creating SLAM object
    KheperaURG04LX laser;

    m_pcSlam = random_seed ?
    (SinglePositionSLAM*)new RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed) :
    (SinglePositionSLAM*)new Deterministic_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS);

    // Get sensor/actuator handles
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_KheperaIVProximitySensor    >("kheperaiv_proximity"  );
    m_pcLidar     = GetSensor  <CCI_KheperaIVLIDARSensor	       >("kheperaiv_lidar"      );
    m_pcOdometry  = GetSensor  <CCI_DifferentialSteeringSensor  >("differential_steering");

    /*
     * Parse the configuration file
     *
     * The user defines this part. Here, the algorithm accepts three
     * parameters and it's nice to put them in the config file so we don't
     * have to recompile if we want to try other settings.
     */
    GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
    m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
    GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    m_unTicks = 0;
}

/****************************************/
/****************************************/


void CKheperaIVSlam::ControlStep() {

   const CCI_KheperaIVProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   const CCI_KheperaIVLIDARSensor::TReadings& tLidarReads = m_pcLidar->GetReadings();
   const CCI_DifferentialSteeringSensor::SReading& tOdomReads = m_pcOdometry->GetReading();
   Rover robot = Rover();

   leftOdom += tOdomReads.CoveredDistanceLeftWheel*147.4*10.0;
   rightOdom += tOdomReads.CoveredDistanceRightWheel*147.4*10.0;

   //Counter for ticks
   m_unTicks++;

   // Update with odometry

   if( m_unTicks%2 == 0 )
    {

       long * odometry = new long [3];
       odometry[0] = m_unTicks;
       odometry[1] = leftOdom;
       odometry[2] = rightOdom;

       // Since function  integer array, convert vector<int> to int
       int * scanvals = new int [SCAN_SIZE];

       for (int k=0; k<SCAN_SIZE; ++k)
        {
            scanvals[k] = tLidarReads[k];
        }

       //Pointer to current scan
       int *lidar = scanvals;
       Velocities velocities = robot.computeVelocities(odometry, velocities);
       m_pcSlam->update(lidar, velocities);

       Position position = m_pcSlam->getpos();

       // Add new coordinates to trajectory ;
       double * v = new double[2];
       v[0] = position.x_mm;
       v[1] = position.y_mm;
       trajectory.push_back(v);

     }

   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta ) {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   }
}


void CKheperaIVSlam::Reset()
{
   // Get final map
   m_pcSlam->getmap(m_pcMapbytes);


   // Put trajectory into map as black pixels
   for (int k=0; k<(int)trajectory.size(); ++k)
   {
       double * v = trajectory[k];

       int x = mm2pix(v[0]);
       int y = mm2pix(v[1]);

       delete v;

       m_pcMapbytes[coords2index(x, y)] = 0;
   }

   // Save map and trajectory as PGM file
   char filename[100];
   sprintf(filename, "%s.pgm", "TestMap");
   printf("\nSaving map to file %s\n", filename);

   FILE * output = fopen(filename, "wt");

   fprintf(output, "P2\n%d %d 255\n", MAP_SIZE_PIXELS, MAP_SIZE_PIXELS);

   for (int y=0; y<MAP_SIZE_PIXELS; y++)
   {
       for (int x=0; x<MAP_SIZE_PIXELS; x++)
       {
           fprintf(output, "%d ", m_pcMapbytes[coords2index(x, y)]);
       }
       fprintf(output, "\n");
   }

   printf("\n");

}

void CKheperaIVSlam::Destroy()
{
    if (random_seed)
      {
          delete ((RMHC_SLAM *)m_pcSlam);
      }
    else
      {
          delete ((Deterministic_SLAM *)m_pcSlam);
      }

    delete m_pcMapbytes;

}
/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CKheperaIVSlam, "kheperaiv_slam_controller")
