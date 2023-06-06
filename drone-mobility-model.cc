#include "drone-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <iomanip>      // std::setprecision

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
							;
	return tid;
}
TypeId ns3::DroneMobilityModel::GetInstanceTypeId() const
{
	return DroneMobilityModel::GetTypeId();
}
ns3::DroneMobilityModel::DroneMobilityModel() {
	m_theta = 0 ; //heading is initialized to zero
}
ns3::DroneMobilityModel::~DroneMobilityModel() {
}

void DroneMobilityModel::SanityCheck()
{
    std::cout << "Debug Statement";
}

}