/*
 * NOTE : Using a custom Mobility Model works, but have issues working with PyViz due to lack of Python Bindings.
 * therefore, work on this is halted for now.
 */

#ifndef NS3_SCRATCH_DRONE_MOBILITY_MODEL_H
#define NS3_SCRATCH_DRONE_MOBILITY_MODEL_H

#include "constant-time-circular-motion-model.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
namespace ns3
{
/** \brief 
 *          Simulation of an implementation of Vega, as described in {dcoss-link}
 * 
 */
class DroneMobilityModel : public ConstantTimeCircularMotionModel
{
public:
  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);


  virtual TypeId GetInstanceTypeId() const;
  /**
   * Create position located at coordinates, velocity & acceleration set to (0,0,0) 
   */
  DroneMobilityModel ();
  virtual ~DroneMobilityModel ();

  /** \brief Sanity check for debugging
   */
  void SanityCheck ();

private:

  double m_theta; //!< Heading angle. If vehicle is moving along x-axis, forward it would be 0, if opposite direction, it's PI (3.14)
  

Vector2D mycenter;
double myradius;
};

}


#endif 