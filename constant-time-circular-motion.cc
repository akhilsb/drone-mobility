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
#include "target-mobility-model.h"
#include "drone-mobility-model.h"

#include <iostream>
#include <iterator>
#include <vector>
#include <random>

using namespace ns3;

static void 
CourseChange (std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition ();
  Vector vel = mobility->GetVelocity ();
  std::cout << Simulator::Now () << ", model=" << mobility << ", POS: x=" << pos.x << ", y=" << pos.y
            << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
            << ", z=" << vel.z << std::endl;
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
          /*
                Thought: Use the rectangle mobility model to visualize a field of view box. Can create it using the same 
                mobility model as this UAV, and hopefully they just move together. 

                This would probably not work once we try to do drone zoom where the height of the drone will change 
                and therefore the bounding box will change. But I have yet to figure out that level of dynamism anyway.
          */

          //double angle = (theta/i)*j;
          uav_position_allocator->Add(Vector(300,200,50.0));
          nodes +=1;
      }
  }
  uavs.Create (nodes);

  MobilityHelper uav_mobility;
  
  uav_mobility.SetPositionAllocator (uav_position_allocator);
  uav_mobility.SetMobilityModel("ns3::DroneMobilityModel",
                            "TangentialVelocity",DoubleValue(50.0),
                            "RadialVelocity",DoubleValue(100.0),
                            "TimeToFlyInOrbit",TimeValue(Seconds(seconds)),
                            "Center", Vector2DValue(Vector2D(200,200)),
                            "MaximumRadius", DoubleValue(100.0),
                            "Epsilon", DoubleValue(1.0)
                            );
  
  //Install the circle mobility module on to the first node
  uav_mobility.Install(uavs);

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

  Simulator::Schedule (Seconds (1), &handleTargetMove, targets, times, positions);

  Simulator::Stop (Seconds (seconds));

  Simulator::Run ();

  Simulator::Destroy ();
  return 0;
}