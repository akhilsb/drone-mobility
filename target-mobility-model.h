/*
 * NOTE : Using a custom Mobility Model works, but have issues working with PyViz due to lack of Python Bindings.
 * therefore, work on this is halted for now.
 */

#ifndef NS3_SCRATCH_TARGET_MOBILITY_MODEL_H
#define NS3_SCRATCH_TARGET_MOBILITY_MODEL_H

#include "ns3/constant-position-mobility-model.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
namespace ns3
{
/** \brief 
 *          Simulation of an implementation of Vega, as described in {dcoss-link}
 * 
 */
class TargetMobilityModel : public ConstantPositionMobilityModel
{
public:

  bool wasFound;
  ns3::Time arrivalTime;
  double detectionScore;

  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);


  virtual TypeId GetInstanceTypeId() const;
  /**
   * Create position located at coordinates, velocity & acceleration set to (0,0,0) 
   */
  TargetMobilityModel ();
  virtual ~TargetMobilityModel ();

  /** \brief Sanity check for debugging
   */
  void SanityCheck ();

  /** \brief Set whether the target was found or not
   */
  void setFound (bool found);

  /** \brief Get whether the target was found or not
   */
  bool getFound ();

  /** \brief Set the confidence score of the detection from the drone
  */
  void setDetectionScore(double score);

  /** \brief Get the confidence score of the detection from the drone
  */
  double getDetectionScore();

  /** \brief Set whether the target was found or not
   */
  void setArrivalTime (ns3::Time t);
  
};

}


#endif 