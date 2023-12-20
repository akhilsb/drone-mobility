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
#include "drone-zoom-model.h"

#include <iostream>
#include <iterator>
#include <cmath>
#include <vector>
#include <random>

using namespace ns3;

double iou_scores[8] = {0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2};
Vector drone_cycle_start;
Vector drone_zoom_start;
double drone_speed = 10.0;

static void 
CourseChange (std::string foo, Ptr<const MobilityModel> mobility)
{
  //Vector pos = mobility->GetPosition ();
  //Vector vel = mobility->GetVelocity ();

  //std::cout << Simulator::Now () << ", model=" << mobility << ", POS: x=" << pos.x << ", y=" << pos.y
  //          << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
  //         << ", z=" << vel.z << std::endl;
}

bool isInside (Vector position, double x_min, double x_max, double y_min, double y_max) {
    return
        position.x <= x_max && position.x >= x_min &&
        position.y <= y_max && position.y >= y_min;
}

double getScore(double height) {
    int i = (height / 50) - 1;
    return iou_scores[i];
}

Vector getVelVector(Vector start, Vector end) {
    std::cout << "START:   " << start << "\n";
    std::cout << "END:   " << end << "\n";
    Vector t = Vector(end.x - start.x, end.y - start.y, end.z - start.z);
    std::cout << "t:   " << t << "\n";
    double mag = sqrt(t.x*t.x + t.y*t.y + t.z*t.z);
    t.x = (t.x / mag) * drone_speed;
    t.y = (t.y / mag) * drone_speed;
    t.z = (t.z / mag) * drone_speed;
    std::cout << "normal t:   " << t << "\n";
    return t;
}

static void
handleState(NodeContainer drones, NodeContainer targets, double hu, double hb) {
    std::string ASCENT("ASCENT");
    std::string DESCENT("DESCENT");
    std::string STATIC("STATIC");

    double tan_theta = std::tan(47 * M_PI / 180);
    double aspect_ratio = 4.0 / 3.0;
    double x_mul_const = aspect_ratio * tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);
    double y_mul_const = tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);

    Ptr<Node> drone_n = drones.Get(0);
    Ptr<MobilityModel> drone_mo = drone_n->GetObject<MobilityModel> ();
    Ptr<DroneZoomModel> drone = DynamicCast<DroneZoomModel>(drone_mo);
    Vector pos = drone->GetPosition();
    std::string mode = drone->GetMode();
    if (mode.compare(DESCENT) == 0 && pos.z > hb) {
    } 
    else if (mode.compare(ASCENT) == 0 && pos.z >= hu) {
        drone->SetVelocity(Vector(0,0,0));
        std::cout << "Setting mode to STATIC" << "\n";
        drone->SetMode("STATIC");
    } else {

        double x_min = pos.x - x_mul_const * pos.z;
        double x_max = pos.x + x_mul_const * pos.z;
        double y_min = pos.y - y_mul_const * pos.z;
        double y_max = pos.y + y_mul_const * pos.z;

        uint32_t nNodes = targets.GetN ();
        Ptr<Node> p;
        Ptr<MobilityModel> mo;
        Ptr<TargetMobilityModel> m;
        double score = getScore(pos.z);

        for (uint32_t i = 0; i < nNodes; i++) {
            p = targets.Get (i);
            mo = p->GetObject<MobilityModel> ();
            m = DynamicCast<TargetMobilityModel>(mo);
            Vector t_pos = m->GetPosition();
            if (isInside(t_pos, x_min, x_max, y_min, y_max) && (m->getDetectionScore() < score) && pos.z >= hu) {
                //std::cout << score << " is greater than:  " << m->getDetectionScore() << std::endl;
                std::cout << "Found target, STARTING DESCENT" << std::endl;
                drone->SetMode("DESCENT");
                drone->SetVelocity(getVelVector(pos, t_pos));
                break;
            } else if (mode.compare(DESCENT) == 0 && pos.z <=hb && isInside(t_pos, x_min, x_max, y_min, y_max) && (m->getDetectionScore() < score)) {
                m->setFound(true);
                m->setDetectionScore(score);
                m->setDetectionTime(Simulator::Now());
            }

        }
        if (mode.compare(DESCENT) == 0 && pos.z <=hb) {
            std::cout << "REACHED BOTTOM, STARTING ASCENT" << std::endl;
            drone->SetMode("ASCENT");
            drone->SetVelocity(getVelVector(pos, drone_zoom_start));
        }
    }
    Simulator::Schedule (Seconds (0.1), &handleState, drones, targets, hu, hb);
}

static void
handleTargetDetection (NodeContainer drones, NodeContainer targets) {
    double tan_theta = std::tan(47 * M_PI / 180);
    double aspect_ratio = 4.0 / 3.0;
    double x_mul_const = aspect_ratio * tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);
    double y_mul_const = tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);

    Ptr<Node> drone = drones.Get(0);
    Ptr<MobilityModel> drone_mo = drone->GetObject<MobilityModel> ();
    Ptr<ConstantTimeCircularMotionModel> drone_m = DynamicCast<ConstantTimeCircularMotionModel>(drone_mo);
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
            std::cout << "Found target,   t_position= " << t_pos << "   ,   node number:  " << i << "   |   drone_pos:  " << pos << std::endl;
            double score = getScore(pos.z);
            m->setDetectionScore(score);
            m->setFound(true);
            m->setDetectionTime(Simulator::Now());
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
determineEndReportLatency (NodeContainer targets, int mode, int hu, double inner_rad, double outer_rad) {
    uint32_t nNodes = targets.GetN ();
    Ptr<Node> p;
    Ptr<MobilityModel> mo;
    Ptr<TargetMobilityModel> m;
    uint32_t numFound = 0;
    uint32_t numIn = 0;
    uint32_t numOut = 0;
    double latencies = 0.0;

    double tan_theta = std::tan(47 * M_PI / 180);
    double aspect_ratio = 4.0 / 3.0;
    double x_mul_const = aspect_ratio * tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);
    double y_mul_const = tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);

    double x_min = 400 - x_mul_const * hu; // Hard coded 200 because that's the center of the simulation (200,200)
    double x_max = 400 + x_mul_const * hu;
    double y_min = 400 - y_mul_const * hu;
    double y_max = 400 + y_mul_const * hu;
    if (Simulator::Now().GetSeconds() < 202) {
        std::cout << "Inner rad:  " << inner_rad << " |  Outer rad:  " << (outer_rad) << std::endl;
    }

    for (uint32_t i = 0; i < nNodes; i++) {
        p = targets.Get (i);
        mo = p->GetObject<MobilityModel> ();
        m = DynamicCast<TargetMobilityModel>(mo);
        Vector t_pos = m->GetPosition();
        if (mode==0) { //drone_cycle
            double rad_to_center = std::sqrt((t_pos.x - 400)*(t_pos.x-400) + ((t_pos.y-400)*(t_pos.y-400)));
            if (Simulator::Now().GetSeconds() > 333 && Simulator::Now().GetSeconds() < 335) {
                std::cout << "rad to center for target" << i << " :  " << (rad_to_center) << std::endl;
            }
            if (rad_to_center>inner_rad && rad_to_center<outer_rad) {
                numIn ++;
                //std::cout << "Sanity!!! one, confirming found was set:  " << Simulator::Now().GetSeconds() << std::endl;
                if (Simulator::Now().GetSeconds() > 333 && Simulator::Now().GetSeconds() < 335) {
                    std::cout << "get Founds for i:" << i << " :  " << (m->getFound()) << std::endl;
                }
                
                if (m->getFound()) {
                    ns3::Time lat = m->getLatency();
                    double l = lat.GetSeconds();
                    latencies += l;
                    numFound ++;
                    //std::cout << "Counting a found. Num found: " << numFound << std::endl;
                }
            } else {
                numOut ++;
            }
        } else if (mode == 1) {
            if (isInside(t_pos, x_min, x_max, y_min, y_max)) { // drone_zoom
                numIn ++;
                if (m->getFound()) {
                    ns3::Time lat = m->getLatency();
                    double l = lat.GetSeconds();
                    latencies += l;
                    numFound ++;
                }
            } else {
                numOut ++;
            }
        }
    }
    if (Simulator::Now().GetSeconds() > 333 && Simulator::Now().GetSeconds() < 335) {
        std::cout << "Number Found :  " << numFound << "  |  Number inside:  " << numIn << "  |  Num out:   " << numOut << std::endl;
    }
    if (numIn == numFound) {
        double av_latency = latencies / numFound;
        std::cout << "Number Found :  " << numFound << "  |  Number inside:  " << numIn << "  |  Num out:   " << numOut << std::endl;
        std::cout << "AVERAGE LATENCY=" << av_latency << std::endl;
        Simulator::Stop();
    } else {
        if (Simulator::Now().GetSeconds() > 11000) {
            double av_latency = latencies / numFound;
            std::cout << "EARLY STOP" << std::endl;
            std::cout << "Number Found :  " << numFound << "  |  Number inside:  " << numIn << "  |  Num out:   " << numOut << std::endl;
            std::cout << "AVERAGE LATENCY=" << av_latency << std::endl;
            Simulator::Stop();
        } else {
        //std::cout << "Number of detections:  " << numFound << "  |  Number inside:  " << numIn << std::endl;
            Simulator::Schedule (Seconds (2), &determineEndReportLatency, targets, mode, hu, inner_rad, outer_rad);
        }
    }
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
  double seconds = 200.0;
  int mode = 0;
  int seed = 3;
  int start_x = 400;
  int start_y = 400;
  int start_z = 50;
  int dz_hu = 249;
  double rate = 0.1;
  CommandLine cmd (__FILE__);
  cmd.AddValue("mode", "Drone mode (dronecycle, etc.)", mode);
  cmd.AddValue("seed", "random seed", seed);
  cmd.AddValue("start_x", "Drone start x position", start_x);
  cmd.AddValue("start_y", "Drone start y position", start_y);
  cmd.AddValue("start_z", "Drone start z position", start_z);
  cmd.AddValue("dz_hu", "DroneZoom hu for area calculation", dz_hu);
  cmd.AddValue("rate", "Event rate for poisson process", rate);

  cmd.Parse (argc, argv);

  double tan_theta = std::tan(47 * M_PI / 180);
  double aspect_ratio = 4.0 / 3.0;
  double x_mul_const = aspect_ratio * tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);
  double y_mul_const = tan_theta / std::sqrt(1 + aspect_ratio * aspect_ratio);

  double x_min = start_x - x_mul_const * dz_hu;
  double x_max = start_x + x_mul_const * dz_hu;
  double y_min = start_y - y_mul_const * dz_hu;
  double y_max = start_y + y_mul_const * dz_hu;
  double area = (x_max - x_min) * (y_max - y_min);
  std::cout << "AREA=" << area << std::endl;
  double radius_calc = std::sqrt(area / M_PI);
  std::cout << "Radius=" << radius_calc << std::endl;
  double inner_rad = radius_calc - x_mul_const * start_z;
  double outer_rad = radius_calc + x_mul_const * start_z;

  //double radius_calc = start_x - 400; //center of inspection area is 400,400. So radius will be x_pos - 400


  drone_cycle_start = Vector(400 + radius_calc, start_y,start_z);
  drone_zoom_start = Vector(start_x, start_y,start_z);
  /*
  drone_cycle_start = Vector(350,200,50.0);
  drone_zoom_start = Vector(200,200,400.0);
  */

  int simulation_type = mode;
  //int simulation_type = 0;  // drone-cycle
  //Vector drone_start = drone_cycle_start;
  //int simulation_type = 1;  // drone-zoom
  //int simulation_type = 2;  // hybrid
  //Vector drone_start = drone_zoom_start;
  Vector drone_start;
  if (mode == 0) {
    drone_start = drone_cycle_start;
  } else {
    drone_start = drone_zoom_start;
  }

  //Define bounds of simulation. Assumed to be a square.
  double min = 0.0;
  double max = 800.0;

  double hu = start_z;
  double hb = 50.0;

  //Config::SetDefault("ns3::ConstantTimeCircularMotionModel::MaximumRadius",DoubleValue(750.0));
  

  ns3::RngSeedManager::SetSeed(seed);  // Changes seed from default of 1 to 3
  ns3::RngSeedManager::SetRun(7);   // Changes run number from default of 1 to 7

  Ptr<ExponentialRandomVariable> exp = CreateObject<ExponentialRandomVariable> ();
  //exp->SetAttribute ("Mean", DoubleValue (rate*seconds)); // Poisson rate
  exp->SetAttribute ("Mean", DoubleValue (1/rate));
  exp->SetAttribute ("Bound", DoubleValue (seconds));
  Ptr<UniformRandomVariable> uni = CreateObject<UniformRandomVariable> ();
  uni->SetAttribute ("Min", DoubleValue (min));
  uni->SetAttribute ("Max", DoubleValue (max));
  //double poisson_rate = 0.1;

  Config::SetDefault("ns3::ConstantTimeCircularMotionModel::Center", Vector2DValue(Vector2D(400,400)));
  Config::SetDefault("ns3::ConstantTimeCircularMotionModel::Radius", DoubleValue(radius_calc));
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
          uav_position_allocator->Add(drone_start);
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
      target_position_allocator->Add(Vector(-800,-800,0));
      nodes +=1;
  }
  std::cout << "Total number of nodes:  " << nodes << std::endl;

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
        
        uav_mobility.SetMobilityModel("ns3::ConstantTimeCircularMotionModel",
                                    "TangentialVelocity",DoubleValue(25.0),
                                    "RadialVelocity",DoubleValue(55.0),
                                    "TimeToFlyInOrbit",TimeValue(Seconds(seconds)),
                                    "Center", Vector2DValue(Vector2D(400,400)),
                                    "Radius", DoubleValue(radius_calc)
                                    );
        
        //Install the circle mobility module on to the first node
        uav_mobility.Install(uavs);
        
        Simulator::Schedule (Seconds (1), &handleTargetMove, targets, times, positions);
        Simulator::Schedule (Seconds (0.1), &handleTargetDetection, uavs, targets);
        break;
      case 1: //drone-zoom
        uav_mobility.SetPositionAllocator (uav_position_allocator);
        uav_mobility.SetMobilityModel("ns3::DroneZoomModel"
                                    );
        //Install the circle mobility module on to the first node
        uav_mobility.Install(uavs);
        Simulator::Schedule (Seconds (0.1), &handleState, uavs, targets, hu, hb);
        Simulator::Schedule (Seconds (1), &handleTargetMove, targets, times, positions);
        break;
      case 2: // hybrid
        break;
      default:
        break;
  }
  Simulator::Schedule (Seconds (seconds), &determineEndReportLatency, targets, mode, dz_hu, inner_rad, outer_rad);
  //Simulator::Stop (Seconds (seconds));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;

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
    Experiment thoughts:
        Results should be collected by fixing event rate and varying area coverage 
            and then fixing area coverage and varying event rate. 

        Fix event rate and vary area coverage:
        - DroneZoom can remain the same...
        - For drone cycle:
            1. Input dz start z, then calculate dronezoom's FoV and determine the radius by A=pi*r^2
            2. Determine that FoV and get the inner and outer radius of the field of view
        
        - Edit target fields to include appear time and detect time

        New handleState:
        - After 100 seconds:
        1. Determine possible area where targets could be detected
        2. Loop through targets, and count number of detections above threshold
        potential_targets=false;
        3. If (in_possible_area && not_detected):
            potential_targets=> true
        4. if (potential_targets)
            schedule in 1 second
        5. else 
            schedule event to print results

        New print results:
            count=0
            latencies = 0
            for each target:
                if detection score > threshold
                    latency <- detect_time - appear time
                    latencies += latency
                    count++

            print AVERAGE_LATENCY = latencies / count

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
            if DroneZoom.mode is descent and height > hb:
                - pass
            Do target detection:

            if target isInside && drone height >= hu && droneDetectionScore < current IoU score
                - set target detection score to score(droneHeight) // function to handle getting the IoU score, given the drone height
                - DroneZoom.setMode("descent")
                - do descent  (i.e. get unit vector and set magnitude)
            
            if DroneZoom.mode is descent and height <= hb:
                - set target detection score to score(droneHeight)
                - do ascent (i.e. get unit vector and set magnitude)
                - set mode to ascent

            if DroneZoom.mode is ascent and height >= hu:
                - set velocity to zero
                - (?) set position to base position (?) (maybe, lets test it out without first)
                - set mode to "static"

            schedule handleState
*/
}