#include "drone-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <iomanip>      // std::setprecision

#include "ns3/double.h"
#include "ns3/vector.h"

#include <cmath>
namespace ns3
{
NS_LOG_COMPONENT_DEFINE ("DroneMobilityModel");
NS_OBJECT_ENSURE_REGISTERED (DroneMobilityModel);

TypeId ns3::DroneMobilityModel::GetTypeId(void) {
	static TypeId tid = TypeId("ns3::DroneMobilityModel")
							.SetParent<ConstantTimeCircularMotionModel>()
							.SetGroupName ("caac")
							.AddConstructor<DroneMobilityModel>()
							.AddAttribute("MyCenter",
										"Center of the circle to revolve around",
										Vector2DValue(Vector2D(0.0,0.0)),
										MakeVector2DAccessor(&DroneMobilityModel::mycenter),
										MakeVector2DChecker())
							.AddAttribute("MyRadius",
										"The maximum surveillance radius that the nodes should patrol",
										DoubleValue(150.0),
										MakeDoubleAccessor(&DroneMobilityModel::myradius),
										MakeDoubleChecker<double>());
	return tid;
}
TypeId ns3::DroneMobilityModel::GetInstanceTypeId() const
{
	return DroneMobilityModel::GetTypeId();
}
ns3::DroneMobilityModel::DroneMobilityModel() {
	m_theta = 0 ; //heading is initialized to zero
	NS_LOG_INFO("DMM Setting Center:  " << mycenter);
	//m_helper.SetCenter(mycenter);
	NS_LOG_INFO("DMM Setting Radius:  " << myradius);
	//m_helper.SetRadius(myradius);
}
ns3::DroneMobilityModel::~DroneMobilityModel() {
}

void DroneMobilityModel::SanityCheck()
{
    std::cout << "Debug Statement";
}

}