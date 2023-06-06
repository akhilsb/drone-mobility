#include "target-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <iomanip>      // std::setprecision

#include <cmath>
namespace ns3
{
NS_LOG_COMPONENT_DEFINE ("TargetMobilityModel");
NS_OBJECT_ENSURE_REGISTERED (TargetMobilityModel);

TypeId ns3::TargetMobilityModel::GetTypeId(void) {
	static TypeId tid = TypeId("ns3::TargetMobilityModel")
							.SetParent<ConstantPositionMobilityModel>()
							.SetGroupName ("caac")
							.AddConstructor<TargetMobilityModel>()
							;
	return tid;
}
TypeId ns3::TargetMobilityModel::GetInstanceTypeId() const
{
	return TargetMobilityModel::GetTypeId();
}
ns3::TargetMobilityModel::TargetMobilityModel() {
    wasFound = false;
    arrivalTime = Seconds(0);
}
ns3::TargetMobilityModel::~TargetMobilityModel() {
}

void TargetMobilityModel::SanityCheck()
{
    std::cout << "Debug Statement";
}

void TargetMobilityModel::setFound(bool found)
{
    wasFound = found;
}

void TargetMobilityModel::setArrivalTime(ns3::Time t)
{
    arrivalTime = t;
}

}