/*
 * NOTE : Using a custom Mobility Model works, but have issues working with PyViz due to lack of Python Bindings.
 * therefore, work on this is halted for now.
 */

#ifndef NS3_SCRATCH_DRONE_ZOOM_MODEL_H
#define NS3_SCRATCH_DRONE_ZOOM_MODEL_H

#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
namespace ns3
{
/** \brief 
 *          Simulation of an implementation of Vega, as described in {dcoss-link}
 * 
 */
class DroneZoomModel : public ConstantVelocityMobilityModel
{
public:

  enum Mode {
    Ascending,
    Descending,
    Static
  };

  Mode mode;

  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);


  virtual TypeId GetInstanceTypeId() const;
  /**
   * Create position located at coordinates, velocity & acceleration set to (0,0,0) 
   */
  DroneZoomModel ();
  virtual ~DroneZoomModel ();

  /** \brief Sanity check for debugging
   */
  void SanityCheck ();

  void setMode(Mode m);

  Mode getMode();

private:

  double m_theta; //!< Heading angle. If vehicle is moving along x-axis, forward it would be 0, if opposite direction, it's PI (3.14)
  
};

}


#endif 