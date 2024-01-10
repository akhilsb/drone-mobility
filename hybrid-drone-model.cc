#include "ns3/random-variable-stream.h"
#include <cmath>
#include "ns3/enum.h"
#include "ns3/string.h"
#include "ns3/simulator.h"
#include "ns3/pointer.h"
#include "hybrid-drone-model.h"
#include "ns3/double.h"
#include "ns3/vector.h"

namespace ns3{

NS_LOG_COMPONENT_DEFINE ("HybridDroneModel");
// Circular mobility model with exchange points - 
// a node reaches an exchange point and only then it can switch orbits
// A model such as this is very difficult to analyze and predict, as there 
// arise phase differences between when drones are in transit and 
// when they are in circular motion 
NS_OBJECT_ENSURE_REGISTERED (HybridDroneModel);
    TypeId
    HybridDroneModel::GetTypeId (void)
    {
        static TypeId tid = TypeId ("ns3::HybridDroneModel")
            .SetParent<MobilityModel>()
            .SetGroupName ("Mobility")
            .AddConstructor<HybridDroneModel> ()
            .AddAttribute("TangentialVelocity",
                        "Velocity of a node along the arcs/circumferences of the circular orbit",
                        DoubleValue (5.0),
                        MakeDoubleAccessor(&HybridDroneModel::m_tangential_vel),
                        MakeDoubleChecker<double> ())
            .AddAttribute("RadialVelocity",
                        "Velocity of a node while moving on the radius of the circle",
                        DoubleValue(10.0),
                        MakeDoubleAccessor(&HybridDroneModel::m_radial_vel),
                        MakeDoubleChecker<double> ())
            .AddAttribute("Center",
                        "Center of the circle to revolve around",
                        Vector2DValue(Vector2D(0.0,0.0)),
                        MakeVector2DAccessor(&HybridDroneModel::center),
                        MakeVector2DChecker())
            .AddAttribute("Timestep",
                        "Time step for updating position along the orbit",
                        DoubleValue(1.0),
                        MakeDoubleAccessor(&HybridDroneModel::time_step),
                        MakeDoubleChecker<double>())
            .AddAttribute("InterOrbitDistance",
                        "Distance between 2 successive orbits in the circle",
                        DoubleValue(75.0),
                        MakeDoubleAccessor(&HybridDroneModel::m_orbit_dist),
                        MakeDoubleChecker<double> ())
            .AddAttribute("Radius",
                        "The maximum surveillance radius that the nodes should patrol",
                        DoubleValue(150.0),
                        MakeDoubleAccessor(&HybridDroneModel::m_max_orbit_rad),
                        MakeDoubleChecker<double>())
            .AddAttribute("TimeToFlyInOrbit",
                        "The time to fly in the orbit before switching carrying out orbit switching",
                        TimeValue(Seconds(10.0)),
                        MakeTimeAccessor(&HybridDroneModel::m_time_rotate),
                        MakeTimeChecker())
            .AddAttribute("Epsilon",
                        "The value of epsilon - a node moves to upper/lower orbit with prob. epsilon"
                        "and moves to a random orbit with probablity (1-epsilon)",
                        DoubleValue(0.99),
                        MakeDoubleAccessor(&HybridDroneModel::m_epsilon),
                        MakeDoubleChecker<double>())
            .AddAttribute("WalkOrFlight",
                        "Walk on this iteration or fly to another orbit in this iteration",
                        StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1.0]"),
                        MakePointerAccessor(&HybridDroneModel::m_rw_rf_choice),
                        MakePointerChecker<RandomVariableStream>())
            .AddAttribute("WalkChoice",
                        "A random variable which decides whether we walk to the upper orbit or lower orbit",
                        StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1.0]"),
                        MakePointerAccessor(&HybridDroneModel::m_rw_choice),
                        MakePointerChecker<RandomVariableStream>())
            .AddAttribute("FlightChoice",
                        "A random variable which decides which orbit to move to next",
                        StringValue("ns3::UniformRandomVariable[Min[0.0|Max=2.0]]"),
                        MakePointerAccessor(&HybridDroneModel::m_rf_choice),
                        MakePointerChecker<RandomVariableStream>());
        return tid;
    }
    HybridDroneModel::HybridDroneModel()
    {
        //NS_LOG_INFO("CTCMM Setting Center:  " << Vector2D(400.0,400.0));
        m_helper.SetCenter(Vector2D(400.0,400.0));
        //NS_LOG_INFO("CTCMM Setting Radius:  " << m_max_orbit_rad);
        m_helper.SetRadius(335.336);
        mode = HybridDroneModel::Mode::Static;
        DoInitialize();
        
    }
    void HybridDroneModel::DoInitialize()
    {
        //DoSetPosition(m_initial_position);
        //m_max_orbit_rad = radius
        //NS_LOG_INFO("CTCMM Setting Center:  " << Vector2D(400.0,400.0));
        //m_helper.SetCenter(Vector2D(400.0,400.0));
        //NS_LOG_INFO("CTCMM Setting Radius:  " << 335.336);
        //m_helper.SetRadius(335.336);
        
        number_of_orbits = m_max_orbit_rad/m_orbit_dist;
    }
    void HybridDroneModel::DoConfigureAngVelHelper(const Vector &position)
    {
        //NS_LOG_INFO("Center: " << center);
        //NS_LOG_INFO("CTTCMM Radius: " << m_max_orbit_rad);
        // set angular velocity in the circular mobility model
        //Vector pos_vector = Vector(center.x-position.x,center.y-position.y,-1*position.z);
        //double current_orbit_radius = std::sqrt(pos_vector.x*pos_vector.x + pos_vector.y*pos_vector.y);
        NS_LOG_INFO("DCAH Setting Radius:  " << m_max_orbit_rad);
        m_helper.SetRadius(m_max_orbit_rad);
        double current_orbit_radius = m_max_orbit_rad;
        double omega = m_tangential_vel/current_orbit_radius;
        // orbital number
        double odd_or_even = std::fmod(current_orbit_radius/m_orbit_dist,2.0);
        NS_LOG_INFO("Configuring Angular Velocity Helper with position: "<<position << ", Radius: "<< current_orbit_radius << ", center: " << center);
        m_helper.SetCenter(center);
        m_helper.SetPosition(position);
        if(odd_or_even> 0.0 && odd_or_even <=1.0)
        {
            m_helper.SetOmega(Vector2D(omega,1.0));
        }
        else 
        {
            m_helper.SetOmega(Vector2D(omega,-1.0));
        }
    }

    void HybridDroneModel::UpdatePosition()
    {
        if(under_surveillance)
        {
            // if(m_rotating_time >= m_time_rotate.GetSeconds())
            // {
            //     m_rotating_time = 0.0;
            //     m_helper.Pause();
            //     Simulator::ScheduleNow(&HybridDroneModel::DoOrbitSwitch,this);
            //     return;
            // }
            m_helper.Update();
            m_rotating_time += 0.1;
        }
        else
        {
            /*
            if(m_radial_time >= m_radial_total_time)
            {
                m_radial_total_time = 0.0;
                m_radial_time = 0.0;
                m_vel_helper.Pause();
                Simulator::ScheduleNow(&HybridDroneModel::DoSurveil,this);
                return;
            }
            */
            m_vel_helper.Update();
            m_radial_time += 0.1;
        }
        m_event = Simulator::Schedule(Seconds(0.1),&HybridDroneModel::UpdatePosition,this);
    }

    HybridDroneModel::~HybridDroneModel()
    {
    }
    void HybridDroneModel::DoSetPosition(const Vector &position)
    {
        m_helper.Update();
        m_helper.Pause();
        // set position of the drone in the entire map
        m_helper.SetPosition(position);
        m_vel_helper.SetPosition(position);
        m_event.Cancel();
        DoConfigureAngVelHelper(position);
        // change position here
        current_position = position;
        m_event = Simulator::ScheduleNow(&HybridDroneModel::DoSurveil,this);
        under_surveillance = true;
        //time_travelled += time_step;
    }
    void HybridDroneModel::DoSurveil()
    {
        m_vel_helper.Update();
        m_vel_helper.Pause();
        m_helper.Update();
        m_helper.Pause();
        // find out nearest exit from here on
        // if(time_travelled >= time_to_travel)
        // {
        //     // we travelled enough time on the orbit, time to switch orbits now
        //     // reset counters
        //     time_to_travel = 0;
        //     time_travelled = 0;
        //     Simulator::ScheduleNow(&HybridDroneModel::DoOrbitSwitch,this);
        //     NotifyCourseChange();
        //     return;
        // }
        NS_LOG_INFO("Starting circular motion from radius: "<<m_helper.GetRadius() << " At theta: " << m_helper.GetOmega()<< " With omega: " << m_helper.GetOmega());
        DoConfigureAngVelHelper(m_vel_helper.GetCurrentPosition());
        m_helper.Unpause();
        under_surveillance = true;
        m_event = Simulator::Schedule(Seconds(0.1),&HybridDroneModel::UpdatePosition,this);
        //m_helper.SetPosition(m_position);
    }
    void HybridDroneModel::DoOrbitSwitch()
    {
        // whether the node is currently conducting surveillance or orbit switching
        under_surveillance = false;
        m_helper.Update();
        m_helper.Pause();
        Vector cur_pos = m_helper.GetCurrentPosition();
        m_vel_helper.Pause();
        m_vel_helper.SetPosition(cur_pos);
        Vector travel_vec;
        // generate random number based on random walk or random flight mode
        double coin_flip = m_rw_rf_choice->GetValue();
        double travel_time = 0.0;
        double radial_component_vel = m_radial_vel;
        int orbit_number = std::round(m_helper.GetRadius()/m_orbit_dist);
        double radius = m_helper.GetRadius();
        // variable for staying in the same orbit without any switching
        bool no_orbit_switch = false;
        NS_LOG_INFO("Starting orbit switch");
        if(coin_flip >= m_epsilon)
        {
            NS_LOG_INFO("Random flight mode with epsilon: "<<coin_flip);
            double total_orbits = (m_max_orbit_rad)/(m_orbit_dist);
            double probability_step = 1.0/(total_orbits);
            double rand = m_rw_choice->GetValue();
            int fin_orbit_num = std::floor(rand/probability_step) +1;
            if(fin_orbit_num > total_orbits)
                fin_orbit_num-=1;
            if(fin_orbit_num < orbit_number)
            {
                // if the drone is in the last orbit, only move downwards, not upwards
                travel_vec = Vector(center.x - cur_pos.x, center.y - cur_pos.y,cur_pos.z);
                travel_time = ((radius)- (fin_orbit_num)*(m_orbit_dist))/(m_radial_vel);
                NS_LOG_INFO("Moving to orbit "<< fin_orbit_num << " from orbit "<< orbit_number);
                NS_LOG_INFO("Travelling from radius: "<<radius <<" to radius: "<< (fin_orbit_num)*(m_orbit_dist));
            }
            else if(fin_orbit_num > orbit_number)
            {
                // same with when the orbit is the first orbit, only move upwards not downwards
                travel_vec = Vector(cur_pos.x-center.x, cur_pos.y-center.y ,cur_pos.z);
                //travel_time = (fin_orbit_num-orbit_number)*(m_orbit_dist)/(m_radial_vel);
                travel_time = ((fin_orbit_num)*(m_orbit_dist)-(radius))/(m_radial_vel);
                NS_LOG_INFO("Moving to orbit "<< fin_orbit_num << " from orbit "<< orbit_number);
                NS_LOG_INFO("Travelling from radius: "<<radius <<" to radius: "<< (fin_orbit_num)*(m_orbit_dist));
            }
            else
            {
                //double step = 2.0/number_of_orbits;
                //int orbit_number = std::floor(rand/step);
                //travel_vec = Vector(cur_pos.x-center.x, cur_pos.y-center.y ,cur_pos.z);
                travel_time = m_orbit_dist/m_radial_vel;
                no_orbit_switch = true;
                NS_LOG_INFO("Staying in the same orbit");
            }
        }
        else
        {
            // probabilities of moving to upper orbit, lower orbit or stay in same orbit
            // undergoing random walk
            NS_LOG_INFO("Random walk mode with epsilon "<< coin_flip);
            double rand = m_rw_choice->GetValue();
            NS_LOG_INFO("Random walk mode with rand: "<< rand);
            if(m_helper.GetRadius() >= m_max_orbit_rad - m_orbit_dist)
            {
                // if the drone is in the last orbit, only move downwards, not upwards
                // transition to (n-2)th orbit with probability (n-2)/2n
                // transition to (n-1)th orbit with probability (n-1)/2n
                // stay in same orbit with probability 3/2n
                if(rand<= (0.5-(1.0)/number_of_orbits))
                {
                    travel_vec = Vector(center.x-cur_pos.x,center.y-cur_pos.y,cur_pos.z);
                    //travel_time = (m_orbit_dist)/(m_radial_vel);
                    radial_component_vel = 2*m_radial_vel;
                    travel_time = ((radius)-(orbit_number-2)*(m_orbit_dist))/(radial_component_vel);
                    NS_LOG_INFO("Moving to orbit "<< number_of_orbits -2 << " from orbit "<< orbit_number);
                    NS_LOG_INFO("Travelling from radius: "<<radius <<" to radius: "<< (orbit_number-2)*(m_orbit_dist));
                }
                else if(rand <= (1-((1.5)/number_of_orbits)) && rand >= (0.5-(1.0)/number_of_orbits))
                {
                    travel_vec = Vector(center.x-cur_pos.x,center.y-cur_pos.y,cur_pos.z);
                    travel_time = ((radius) - (orbit_number-1)*(m_orbit_dist))/(m_radial_vel);
                    //travel_time = (m_orbit_dist)/(m_radial_vel);
                    //radial_component_vel = 2*m_radial_vel;
                    NS_LOG_INFO("Moving to orbit "<< number_of_orbits -1 << " from orbit "<< orbit_number);
                    NS_LOG_INFO("Travelling from radius: "<<radius <<" to radius: "<< (orbit_number-1)*(m_orbit_dist));
                }
                else 
                {
                    // just stay in the orbit
                    travel_time = m_orbit_dist/m_radial_vel;
                    no_orbit_switch = true;
                    NS_LOG_INFO("Staying in the same orbit");
                }
            }
            else if(m_helper.GetRadius() < 1.5*m_orbit_dist)
            {
                // same with when the orbit is the first orbit, only move upwards not downwards
                travel_vec = Vector(cur_pos.x-center.x,cur_pos.y-center.y,cur_pos.z);
                // move to orbit 2 with probability 1/2 and stay in orbit with probability 1/2 
                if(rand<= (0.5))
                {
                    travel_vec = Vector(cur_pos.x-center.x,cur_pos.y-center.y,0);
                    travel_time = ((orbit_number+1)*(m_orbit_dist)-radius)/(m_radial_vel);
                    //travel_time = (m_orbit_dist)/(m_radial_vel);
                    radial_component_vel = m_radial_vel;
                    NS_LOG_INFO("Moving to orbit "<< 2 << " from orbit "<< orbit_number);
                    NS_LOG_INFO("Travelling from radius: "<<radius <<" to radius: "<< (2)*(m_orbit_dist));
                }
                else 
                {
                    //travel_vec = Vector(center.x-cur_pos.x,center.y-cur_pos.y,cur_pos.z);
                    travel_time = (m_orbit_dist)/(m_radial_vel);
                    no_orbit_switch = true;
                    NS_LOG_INFO("Staying in orbit 1");
                }
            }
            else
            {
                //double rand = m_rw_choice->GetValue();
                // distribution of the orbit
                // move to the upward orbit with probability 1/2
                // move to the lower orbit with probability (i-1)/(2i)
                // stay in the orbit with probability 1/(2i)
                if(rand <= 0.5)
                {
                    // travel outwards with probability 0.5
                    travel_vec = Vector(cur_pos.x-center.x,cur_pos.y-center.y,cur_pos.z);
                    travel_time = ((orbit_number+1)*(m_orbit_dist)-radius)/(m_radial_vel);
                    radial_component_vel = m_radial_vel;
                    NS_LOG_INFO("Moving to orbit "<< orbit_number+1 << " from orbit "<< orbit_number);
                    NS_LOG_INFO("Travelling from radius: "<<radius <<" to radius: "<< (orbit_number+1)*(m_orbit_dist));
                }
                else if(rand > 0.5 && rand <= 1-(0.5/orbit_number))
                {
                    // travel inwards with probability 0.5 - 0.5/i
                    travel_vec = Vector(center.x-cur_pos.x,center.y-cur_pos.y,cur_pos.z);
                    travel_time = (radius - (orbit_number-1)*(m_orbit_dist))/(m_radial_vel);
                    radial_component_vel = m_radial_vel;
                    NS_LOG_INFO("Moving to orbit "<< orbit_number -1 << " from orbit "<< orbit_number);
                    NS_LOG_INFO("Travelling from radius: "<<radius <<" to radius: "<< (orbit_number-1)*(m_orbit_dist));
                }
                else
                {
                    // stay in the orbit with the remaining probability
                    travel_time = (m_orbit_dist)/(m_radial_vel);
                    no_orbit_switch = true;
                    NS_LOG_INFO("Staying in the same orbit");
                }
            }
        }
        if(no_orbit_switch)
        {
            m_event = Simulator::ScheduleNow(&HybridDroneModel::DoSurveil,this);
        }
        else
        {
            m_radial_total_time = travel_time;
            double theta = m_helper.GetTheta(travel_vec);
            m_vel_helper.SetVelocity(Vector(radial_component_vel*std::cos(theta),radial_component_vel*std::sin(theta),0));
            m_vel_helper.Update();
            m_vel_helper.Unpause();
            m_event = Simulator::Schedule(Seconds(0.1),
                                &HybridDroneModel::UpdatePosition,this);
            NotifyCourseChange();
        }
    }

    Vector
    HybridDroneModel::DoGetVelocity (void) const
    {
        if(under_surveillance)
        {
            double theta = m_helper.GetTheta(m_helper.GetCurrentPosition());
            return Vector(-1*m_tangential_vel*std::sin(theta),m_tangential_vel*std::cos(theta),0.0);
        }
        else
        {
            return m_vel_helper.GetVelocity();
        }
    }
    Vector HybridDroneModel::DoGetPosition(void) const
    {
        return m_helper.GetCurrentPosition();
    }
    double HybridDroneModel::DoGetRadius()
    {
        if(under_surveillance)
            return m_helper.GetRadius();
        else
            return -1.0;
    }
    int64_t HybridDroneModel::DoAssignStreams (int64_t stream)
    {
        m_rw_choice->SetStream(stream);
        m_rf_choice->SetStream(stream+1);
        m_rw_rf_choice->SetStream(stream+2);
        return 3;
    } 

    void HybridDroneModel::SetMode(std::string m) {
        mode = m;
    }

    std::string HybridDroneModel::GetMode() {
        return mode;
    }

    void HybridDroneModel::Surveil() {
        m_event = Simulator::ScheduleNow(&HybridDroneModel::DoSurveil,this);
    }

    void HybridDroneModel::SetVelocity(Vector vel) {
        NS_LOG_INFO("Setting velocity to : "<< vel);
        m_vel_helper.SetVelocity(vel);
    }

    void HybridDroneModel::StopSurveil(Vector vel) {
        m_vel_helper.Update();
        m_vel_helper.Pause();
        m_helper.Update();
        m_helper.Pause();
        NS_LOG_INFO("Starting straightline motion from position: "<< m_helper.GetCurrentPosition());
        Vector top = m_helper.GetCurrentPosition();
        x_top = top.x;
        y_top = top.y;
        z_top = top.z;
        //DoConfigureAngVelHelper(m_vel_helper.GetCurrentPosition());
        NS_LOG_INFO("Setting velocity to : "<< vel);
        m_vel_helper.SetVelocity(vel);
        m_vel_helper.Unpause();
        //m_helper.Unpause();
        under_surveillance = false;
        m_event = Simulator::Schedule(Seconds(0.1),&HybridDroneModel::UpdatePosition,this);
    }

    Vector HybridDroneModel::GetTopPosition() {
        return Vector(x_top, y_top, z_top);
    }

    void HybridDroneModel::SanityCheck()
    {
        std::cout << "Debug Statement";
    }
}