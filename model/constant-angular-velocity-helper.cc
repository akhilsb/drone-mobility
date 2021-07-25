#include "ns3/simulator.h"
#include "ns3/rectangle.h"
#include "ns3/box.h"
#include "ns3/log.h"
#include "constant-angular-velocity-helper.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ConstantAngularVelocityHelper");

ConstantAngularVelocityHelper::ConstantAngularVelocityHelper ()
  : m_paused (true)
{
  NS_LOG_FUNCTION (this);
}
ConstantAngularVelocityHelper::ConstantAngularVelocityHelper (const Vector2D &center)
  : center (center),
    m_paused (true)
{
  NS_LOG_FUNCTION (this << center);
}
ConstantAngularVelocityHelper::ConstantAngularVelocityHelper (const Vector2D &center,
                                                const Vector2D &omega, const Vector &position)
  : center (center),
    m_omega (omega),
    m_position(position),
    m_paused (true)
{
  NS_LOG_FUNCTION (this << center << omega);
}

void
ConstantAngularVelocityHelper::SetPosition (const Vector &position)
{
  NS_LOG_FUNCTION (this << position);
  m_position = position;
  m_omega = Vector2D(0.0,1.0);
  m_lastUpdate = Simulator::Now ();
}

Vector
ConstantAngularVelocityHelper::GetCurrentPosition (void) const
{
  NS_LOG_FUNCTION (this);
  return m_position;
}

Vector2D 
ConstantAngularVelocityHelper::GetOmega (void) const
{
  NS_LOG_FUNCTION (this);
  return m_paused ? Vector2D (0.0, 1.0) : m_omega;
}

double ConstantAngularVelocityHelper::GetRadius() const
{
    double dx = m_position.x - center.x;
    double dy = m_position.y - center.y;
    return std::sqrt(dx*dx + dy*dy);
}
void 
ConstantAngularVelocityHelper::SetOmega (const Vector2D &omega)
{
  NS_LOG_FUNCTION (this << omega);
  m_omega = omega;
  m_lastUpdate = Simulator::Now ();
}

void
ConstantAngularVelocityHelper::SetCenter(const Vector2D &cen)
{
    NS_LOG_FUNCTION (this << cen);
    center = cen;
    m_lastUpdate = Simulator::Now ();
}

void
ConstantAngularVelocityHelper::Update (void) const
{
  NS_LOG_FUNCTION (this);
  Time now = Simulator::Now ();
  NS_ASSERT (m_lastUpdate <= now);
  Time deltaTime = now - m_lastUpdate;
  m_lastUpdate = now;
  if (m_paused)
    {
      return;
    }
  double deltaS = deltaTime.GetSeconds ();
  double radius = GetRadius();
  // augment value of theta between 0 and 2*PI
  double theta = GetTheta(m_position);
  theta= theta + deltaS*m_omega.x*m_omega.y;
    
    while (theta >= 2*M_PI)
    {
        theta -= 2*M_PI;
    }
  m_position.x = radius * std::cos(theta);
  m_position.y = radius *std::sin(theta);
}

double 
ConstantAngularVelocityHelper::GetTheta(const Vector &position) const
{
  double radius = GetRadius();
  // To calculate \theta, calculate dot product first and then calculate \the ta
  Vector2D positionVector = Vector2D(position.x-center.x,position.y-center.y);
  Vector2D parallelLineToXAxis = Vector2D(1.0,0.0);
  double dotProduct = positionVector.x*parallelLineToXAxis.x + positionVector.y*parallelLineToXAxis.y;
  double theta = std::acos(dotProduct/radius);
  if(positionVector.y < 0)
  {
    theta = 2*M_PI -theta;
  }
  return theta;
}

// void
// ConstantVelocityHelper::UpdateWithBounds (const Rectangle &bounds) const
// {
//   NS_LOG_FUNCTION (this << bounds);
//   Update ();
//   m_position.x = std::min (bounds.xMax, m_position.x);
//   m_position.x = std::max (bounds.xMin, m_position.x);
//   m_position.y = std::min (bounds.yMax, m_position.y);
//   m_position.y = std::max (bounds.yMin, m_position.y);
// }

// void
// ConstantVelocityHelper::UpdateWithBounds (const Box &bounds) const
// {
//   NS_LOG_FUNCTION (this << bounds);
//   Update ();
//   m_position.x = std::min (bounds.xMax, m_position.x);
//   m_position.x = std::max (bounds.xMin, m_position.x);
//   m_position.y = std::min (bounds.yMax, m_position.y);
//   m_position.y = std::max (bounds.yMin, m_position.y);
//   m_position.z = std::min (bounds.zMax, m_position.z);
//   m_position.z = std::max (bounds.zMin, m_position.z);
// }

void 
ConstantAngularVelocityHelper::Pause (void) const
{
  NS_LOG_FUNCTION (this);
  m_paused = true;
}

void 
ConstantAngularVelocityHelper::Unpause (void) const
{
  NS_LOG_FUNCTION (this);
  m_paused = false;
}

} // namespace ns3