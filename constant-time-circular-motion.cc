/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006,2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */ 

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/box.h"
#include "target-mobility-model.h"
#include "drone-mobility-model.h"

#include <iostream>
#include <iterator>
#include <cmath>
#include <vector>
#include <random>

using namespace ns3;

double iou_scores[8] = {0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0};

static void 
CourseChange (std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition ();
  Vector vel = mobility->GetVelocity ();
  std::cout << Simulator::Now () << ", model=" << mobility << ", POS: x=" << pos.x << ", y=" << pos.y
            << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
            << ", z=" << vel.z << std::endl;
}

bool isInside (Vector position, double x_min, double x_max, double y_min, double y_max) {
    return
        position.x <= x_max && position.x >= x_min &&
        position.y <= y_max && position.y >= y_min;
}

static void
handleState(NodeContainer drones, NodeContainer targets) {
    double tan_theta = std::tan(47 * M_PI / 180);
    double aspect_ratio = 4.0 / 3.0;
    double x_mul_const = aspect_ratio * tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);
    double y_mul_const = tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);

    Ptr<Node> drone = drones.Get(0);
    Ptr<MobilityModel> drone_mo = drone->GetObject<MobilityModel> ();
    Ptr<DroneMobilityModel> drone_m = DynamicCast<DroneMobilityModel>(drone_mo);
    Vector pos = drone_m->GetPosition();

    double x_min = pos.x - x_mul_const * pos.z;
    double x_max = pos.x + x_mul_const * pos.z;
    double y_min = pos.y - y_mul_const * pos.z;
    double y_max = pos.y + y_mul_const * pos.z;

    uint32_t nNodes = targets.GetN ();
    Ptr<Node> p;
    Ptr<MobilityModel> mo;
    Ptr<TargetMobilityModel> m;

    for (uint32_t i = 0; i < nNodes; i++) {
        p = targets.Get (i);
        mo = p->GetObject<MobilityModel> ();
        m = DynamicCast<TargetMobilityModel>(mo);
        Vector t_pos = m->GetPosition();
        if (isInside(t_pos, x_min, x_max, y_min, y_max) && !(m->getFound())) {
            std::cout << "Found target, node number:  " << i << std::endl;
            m->setFound(true);
        }

    }
}

static void
handleTargetDetection (NodeContainer drones, NodeContainer targets) {
    double tan_theta = std::tan(47 * M_PI / 180);
    double aspect_ratio = 4.0 / 3.0;
    double x_mul_const = aspect_ratio * tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);
    double y_mul_const = tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);

    Ptr<Node> drone = drones.Get(0);
    Ptr<MobilityModel> drone_mo = drone->GetObject<MobilityModel> ();
    Ptr<DroneMobilityModel> drone_m = DynamicCast<DroneMobilityModel>(drone_mo);
    Vector pos = drone_m->GetPosition();

    double x_min = pos.x - x_mul_const * pos.z;
    double x_max = pos.x + x_mul_const * pos.z;
    double y_min = pos.y - y_mul_const * pos.z;
    double y_max = pos.y + y_mul_const * pos.z;

    uint32_t nNodes = targets.GetN ();
    Ptr<Node> p;
    Ptr<MobilityModel> mo;
    Ptr<TargetMobilityModel> m;

    for (uint32_t i = 0; i < nNodes; i++) {
        p = targets.Get (i);
        mo = p->GetObject<MobilityModel> ();
        m = DynamicCast<TargetMobilityModel>(mo);
        Vector t_pos = m->GetPosition();
        if (isInside(t_pos, x_min, x_max, y_min, y_max) && !(m->getFound())) {
            std::cout << "Found target, node number:  " << i << std::endl;
            m->setFound(true);
        }

    }
    
    Simulator::Schedule (Seconds (0.1), &handleTargetDetection, drones, targets);
}

static void
handleTargetMove (NodeContainer targets, std::vector<double> times, std::vector<Vector> positions) {
    uint32_t nNodes = targets.GetN ();
    double t = 0.0;
    Time time = Simulator::Now();
    double now = time.GetSeconds();
    Vector pos;
    Ptr<Node> p;
    Ptr<MobilityModel> mo;
    Ptr<TargetMobilityModel> m;
    for (uint32_t i = 0; i < nNodes; i++) {
        p = targets.Get (i);
        mo = p->GetObject<MobilityModel> ();
        m = DynamicCast<TargetMobilityModel>(mo);
        t = times[i];
        if (abs(now - t) < 0.1) {
            m->setArrivalTime(Seconds(t));
            pos = positions[i];
            m->SetPosition(pos);
        }
  }
  Simulator::Schedule (Seconds (1), &handleTargetMove, targets, times, positions);
}

static void
countNumberOfDetections (NodeContainer targets) {
    uint32_t nNodes = targets.GetN ();
    Ptr<Node> p;
    Ptr<MobilityModel> mo;
    Ptr<TargetMobilityModel> m;

    int count = 0;
    for (uint32_t i = 0; i < nNodes; i++) {
        p = targets.Get (i);
        mo = p->GetObject<MobilityModel> ();
        m = DynamicCast<TargetMobilityModel>(mo);
        if (m->getFound()) {
            count++;
        }
    }
    std::cout << "Number of detections:  " << count << std::endl;
}

int main (int argc, char *argv[])
{
  LogComponentEnable("ConstantTimeCircularMotionModel", LOG_LEVEL_INFO);
  LogComponentEnable("ConstantAngularVelocityHelper", LOG_LEVEL_INFO);
  //LogComponentEnable("ConstantAngularVelocityHelper", LOG_LEVEL_FUNCTION);

  //Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Mode", StringValue ("Time"));
  //Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Time", StringValue ("2s"));
  //Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  //Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Bounds", StringValue ("0|200|0|200"));
  //double max_radius = 750.0;
  //double min_radius = 75.0;
  double seconds = 100.0;

  int simulation_type = 0;  // drone-cycle
  //int simulation_type = 1  // drone-zoom
  //int simulation_type = 2  // hybrid

  //Define bounds of simulation. Assumed to be a square.
  double min = 0.0;
  double max = 400.0;

  //Config::SetDefault("ns3::ConstantTimeCircularMotionModel::MaximumRadius",DoubleValue(750.0));
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);

  ns3::RngSeedManager::SetSeed(3);  // Changes seed from default of 1 to 3
  ns3::RngSeedManager::SetRun(7);   // Changes run number from default of 1 to 7

  Ptr<ExponentialRandomVariable> exp = CreateObject<ExponentialRandomVariable> ();
  exp->SetAttribute ("Mean", DoubleValue (15.0)); // Poisson rate
  exp->SetAttribute ("Bound", DoubleValue (100.0));
  Ptr<UniformRandomVariable> uni = CreateObject<UniformRandomVariable> ();
  uni->SetAttribute ("Min", DoubleValue (min));
  uni->SetAttribute ("Max", DoubleValue (max));
  //double poisson_rate = 0.1;
  
/*
          DroneZoom thoughts:
              Pre: DroneZoom will be a separate mobility module.
              1. Find hb by looking at the SLA requirement for quality. hb will be
                  the highest height for which an IoU array of qualities has a quality greater than the SLA.
              2.  Might need some help in determining hu. For now just set it to 400, the max.
              3. Start at hu
              4. If detection, move down to hb.
                  a. Will want a method to start the drone moving down toward the detection point on a straight line.
                  b. Set a flag to say the drone is currently in DZ mode
                  c. When the drone gets to the height, hb stop.
                  d. Move back to hu on a straight line.
                  e. So we need: helper functions for determining the point where the drone will go (Pythagorean theorem type deal).
                      a parameter for saying the Vector that the drone was at at hu, and a vector that will be dynamically
                      changing to hold where hb will be to get to a height of hb.
*/

/*
    New DZ alg.
        Pre: - Find hb by looking at the SLA requirement for quality. hb will be
                  the highest height for which an IoU array of qualities has a quality greater than the SLA.
             - Have a constant velocity that we will use for descent and ascent  
             - Have a threshold for wuality detection 
             - Running handleTargetDetection already
                a. Change setFound to not be True/False. instead check the detection score, 
                    and if the current detection score is > previous detection score, set new detection score
        1. Schedule event to wait until detection at hu. function inputs(constantVelocity cv, hu)
        2. If detection and Target.detectionScore < threshold:
            a. Calculate unit vector by (D(x,y,z) - T(x,y,z)) / Mag(D(x,y,z) - T(x,y,z))
            b. DroneZoomMobilityModel.setVelocity(cv * unit vector)
            c. Wait until D(z) < hb
            d. Ascend back up to hu by setting velocity to negative of the above velocity


    Thoughts:
        - new handleState scheduled method runs every 0.1 seconds:
            Start with handleTargetDetection
            if target isInside && drone height >= hu
                - DroneZoom.setMode("descent")
                - do descent  
                - schedule handleTargetDetection 
            if DroneZoom.mode is descent and height > hb:
                - schedule handleTargetDetection
            if DroneZoom.mode is descent and height <= hb:
                - schedule handleTargetDetection
                - do ascent 
                - set mode to ascent
            if DroneZoom.mode is ascent and height >= hu:
                - set velocity to zero
                - (?) set position to base position (?) (maybe, lets test it out without first)
                - set mode to "static"
            schedule handleState
*/

  NodeContainer uavs;
  Ptr<ListPositionAllocator> uav_position_allocator = CreateObject<ListPositionAllocator>();
  NodeContainer targets;
  Ptr<ListPositionAllocator> target_position_allocator = CreateObject<ListPositionAllocator>();
  int nodes = 0;

  //Create the UAV
  for(int i=1;i<=1;i++)
  {
      //double theta = 2*M_PI;
      for(int j=1;j<=i;j++)
      {
          //double angle = (theta/i)*j;
          uav_position_allocator->Add(Vector(350,200,50.0));
          nodes +=1;
      }
  }
  uavs.Create (nodes);

  MobilityHelper uav_mobility;

  //Create targets by generating target arrival times according to a poisson process.
  double t = 0.0;
  std::vector<double> times;
  std::vector<Vector> positions;
  nodes = 0;
  while (t < seconds) {
      t += exp->GetValue();
      times.push_back(int (t));
      Vector pos = Vector(int (uni->GetValue()), int (uni->GetValue()), 0.0);
      positions.push_back(pos);
      target_position_allocator->Add(Vector(-200,-200,0));
      nodes +=1;
  }
  targets.Create(nodes);

  MobilityHelper target_mobility;

  target_mobility.SetPositionAllocator(target_position_allocator);
  target_mobility.SetMobilityModel("ns3::TargetMobilityModel");

  target_mobility.Install(targets);
  
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeCallback (&CourseChange));

  switch(simulation_type) {
      case 0: // drone-cycle
        uav_mobility.SetPositionAllocator (uav_position_allocator);
        uav_mobility.SetMobilityModel("ns3::DroneMobilityModel",
                                    "TangentialVelocity",DoubleValue(30.0),
                                    "RadialVelocity",DoubleValue(45.0),
                                    "TimeToFlyInOrbit",TimeValue(Seconds(seconds)),
                                    "Center", Vector2DValue(Vector2D(200,200)),
                                    "Radius", DoubleValue(150.0)
                                    );
        
        //Install the circle mobility module on to the first node
        uav_mobility.Install(uavs);

        Simulator::Schedule (Seconds (seconds), &countNumberOfDetections, targets);
        Simulator::Schedule (Seconds (1), &handleTargetMove, targets, times, positions);
        Simulator::Schedule (Seconds (0.1), &handleTargetDetection, uavs, targets);
        break;
      case 1: //drone-zoom
        Simulator::Schedule (Seconds (0.1), &handleState, uavs, targets);
        break;
      case 2: // hybrid
        break;
      default:
        break;
  }

  

  Simulator::Stop (Seconds (seconds));

  Simulator::Run ();

  Simulator::Destroy ();
  return 0;
}