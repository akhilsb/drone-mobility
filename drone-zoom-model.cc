#include "drone-zoom-model.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <iomanip>      // std::setprecision
#include <string>

#include <cmath>
namespace ns3
{
NS_LOG_COMPONENT_DEFINE ("DroneZoomModel");
NS_OBJECT_ENSURE_REGISTERED (DroneZoomModel);

TypeId ns3::DroneZoomModel::GetTypeId(void) {
	static TypeId tid = TypeId("ns3::DroneZoomModel")
							.SetParent<ConstantVelocityMobilityModel>()
							.SetGroupName ("caac")
							.AddConstructor<DroneZoomModel>()
							;
	return tid;
}
TypeId ns3::DroneZoomModel::GetInstanceTypeId() const
{
	return DroneZoomModel::GetTypeId();
}
ns3::DroneZoomModel::DroneZoomModel() {
	m_theta = 0 ; //heading is initialized to zero
	mode = DroneZoomModel::Mode::Static;
}
ns3::DroneZoomModel::~DroneZoomModel() {
}

void DroneZoomModel::SetMode(std::string m) {
	mode = m;
}

std::string DroneZoomModel::GetMode() {
	return mode;
}

void DroneZoomModel::SanityCheck()
{
    std::cout << "Debug Statement";
}

}