/* Include the controller definition */
#include "offline_slam.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

void CKheperaIVManualControl::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

CKheperaIVManualControl::CKheperaIVManualControl() :
m_pcWheels(NULL),
m_pcLEDs(NULL),
m_pcOdometry(NULL),
m_pcLidar(NULL),
m_cSLAMData("slam_data.dat", ofstream::out | ofstream::trunc),
m_unTicks(0)
{
}
/****************************************/
/****************************************/

void CKheperaIVManualControl::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the XML tag of the
    * device whose handle we want to have. For a list of allowed values, type at the
    * command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
    *       the lists provided the configuration file at the
    *       <controllers><Khepera_diffusion><actuators> and
    *       <controllers><Khepera_diffusion><sensors> sections. If you forgot to
    *       list a device in the XML and then you request it here, an error occurs.
    */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcOdometry  = GetSensor  <CCI_DifferentialSteeringSensor  >("differential_steering");
    m_pcLidar     = GetSensor  <CCI_KheperaIVLIDARSensor        >("kheperaiv_lidar"      );
    m_pcLEDs   = GetActuator<CCI_LEDsActuator                   >("leds");
   /*
    * Parse the config file
    */
   try {
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }
}

/****************************************/
/****************************************/

Real leftOdom = 0.0;
Real rightOdom = 0.0;

void CKheperaIVManualControl::ControlStep() {
   /* Follow the control vector only if selected */
   const CCI_DifferentialSteeringSensor::SReading& tOdomReads = m_pcOdometry->GetReading();
   const CCI_KheperaIVLIDARSensor::TReadings& tLidarReads = m_pcLidar->GetReadings();

   leftOdom += tOdomReads.CoveredDistanceLeftWheel*147.4*10.0;
   rightOdom += tOdomReads.CoveredDistanceRightWheel*147.4*10.0;

   m_unTicks++;

   if(m_bSelected)
      SetWheelSpeedsFromVector(m_cControl);
   else
      m_pcWheels->SetLinearVelocity(0.0f, 0.0f);


      if(m_unTicks%1 == 0)
      {

          //Avoiding decimal values and converting time to microseconds
          m_cSLAMData << (UInt64)(m_unTicks*100*1000) << " ";

          //unused data
          m_cSLAMData << 0 << " ";

          // odometry

          m_cSLAMData << (UInt64)(leftOdom) << " "<< (UInt64)(rightOdom) << " ";

          // more unused data
          for(size_t j = 0; j < 20; ++j ){
             m_cSLAMData << 0 << " " ;
          }


          for(size_t j = 0; j < tLidarReads.size(); ++j ){
             m_cSLAMData << (UInt32)(tLidarReads[j]*10) << " " ;
          }
          m_cSLAMData << "\n";

     }
}

/****************************************/
/****************************************/

void CKheperaIVManualControl::Select() {
   m_bSelected = true;
   m_pcLEDs->SetAllColors(CColor::RED);
}

/****************************************/
/****************************************/

void CKheperaIVManualControl::Deselect() {
   m_bSelected = false;
   m_pcLEDs->SetAllColors(CColor::BLACK);
}

/****************************************/
/****************************************/

void CKheperaIVManualControl::SetControlVector(const CVector2& c_control) {
   m_cControl = c_control;
}

/****************************************/
/****************************************/

void CKheperaIVManualControl::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

   /* Turning state switching conditions */
   if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
      /* No Turn, heading angle very small */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
   }
   else if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
      /* Hard Turn, heading angle very large */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
   }
   else if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN &&
           Abs(cHeadingAngle) > m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
      /* Soft Turn, heading angle in between the two cases */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
   }

   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }

      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }

      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }

   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CKheperaIVManualControl, "kheperaiv_manual_controller")
