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

int main (int argc, char *argv[])
{
  //Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Mode", StringValue ("Time"));
  //Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Time", StringValue ("2s"));
  //Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  //Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Bounds", StringValue ("0|200|0|200"));
  //double max_radius = 750.0;
  double min_radius = 75.0;
  Config::SetDefault("ns3::ConstantTimeCircularMotionModel::MaximumRadius",DoubleValue(750.0));
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);

  NodeContainer c;
  Ptr<ListPositionAllocator> list_position_allocator = CreateObject<ListPositionAllocator>();
  int nodes = 0;
  for(int i=1;i<=1;i++)
  {
      double theta = 2*M_PI;
      for(int j=1;j<=i;j++)
      {
          double angle = (theta/i)*j;
          list_position_allocator->Add(Vector(min_radius*i*std::cos(angle),min_radius*i*std::sin(angle),0.0));
          nodes +=1;
      }
  }
  c.Create (nodes);

  MobilityHelper mobility;
  
  mobility.SetPositionAllocator (list_position_allocator);
  mobility.SetMobilityModel("ns3::ConstantTimeCircularMotionModel",
                            "TangentialVelocity",DoubleValue(50.0),
                            "RadialVelocity",DoubleValue(100.0)
                            );
  mobility.InstallAll ();
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeCallback (&CourseChange));

  Simulator::Stop (Seconds (100.0));

  Simulator::Run ();

  Simulator::Destroy ();
  return 0;
}
