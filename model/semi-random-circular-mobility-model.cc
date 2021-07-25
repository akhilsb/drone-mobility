#include "ns3/random-variable-stream.h"
#include <cmath>
#include "ns3/enum.h"
#include "ns3/string.h"
#include "ns3/simulator.h"
#include "ns3/pointer.h"
#include "semi-random-circular-mobility-model.h"
#include "ns3/double.h"
#include "ns3/vector.h"

namespace ns3{

// Circular mobility model with exchange points - 
// a node reaches an exchange point and only then it can switch orbits
// A model such as this is very difficult to analyze and predict, as there 
// arise phase differences between when drones are in transit and 
// when they are in circular motion 
NS_OBJECT_ENSURE_REGISTERED (SemiRandomCircularMobilityModel);
    TypeId
    SemiRandomCircularMobilityModel::GetTypeId (void)
    {
        static TypeId tid = TypeId ("ns3::SemiRandomCircularMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName ("Mobility")
            .AddConstructor<SemiRandomCircularMobilityModel> ()
            .AddAttribute("TangentialVelocity",
                        "Velocity of a node along the arcs/circumferences of the circular orbit",
                        DoubleValue (5.0),
                        MakeDoubleAccessor(&SemiRandomCircularMobilityModel::m_tangential_vel),
                        MakeDoubleChecker<double> ())
            .AddAttribute("RadialVelocity",
                        "Velocity of a node while moving on the radius of the circle",
                        DoubleValue(10.0),
                        MakeDoubleAccessor(&SemiRandomCircularMobilityModel::m_radial_vel),
                        MakeDoubleChecker<double> ())
            .AddAttribute("Center",
                        "Center of the circle to revolve around",
                        Vector2DValue(Vector2D(0.0,0.0)),
                        MakeVector2DAccessor(&SemiRandomCircularMobilityModel::center),
                        MakeVector2DChecker())
            .AddAttribute("Timestep",
                        "Time step for updating position along the orbit",
                        DoubleValue(1.0),
                        MakeDoubleAccessor(&SemiRandomCircularMobilityModel::time_step),
                        MakeDoubleChecker<double>())
            .AddAttribute("InterOrbitDistance",
                        "Distance between 2 successive orbits in the circle",
                        DoubleValue(75.0),
                        MakeDoubleAccessor(&SemiRandomCircularMobilityModel::m_orbit_dist),
                        MakeDoubleChecker<double> ())
            .AddAttribute("MaximumRadius",
                        "The maximum surveillance radius that the nodes should patrol",
                        DoubleValue(750.0),
                        MakeDoubleAccessor(&SemiRandomCircularMobilityModel::m_max_orbit_rad),
                        MakeDoubleChecker<double>())
            .AddAttribute("ExchangePoints",
                        "Number of exchange points on the entire orbit (for rate control)",
                        DoubleValue(4.0),
                        MakeDoubleAccessor(&SemiRandomCircularMobilityModel::m_exchange_points),
                        MakeDoubleChecker<double>())
            .AddAttribute("Mode",
                        "Random Flight mode or Random walk mode"
                        "Random Flight mode is when nodes can switch to any other orbit"
                        "Random Walk mode is when nodes can only switch to orbit above or below",
                        EnumValue(SemiRandomCircularMobilityModel::RANDOM_WALK_MODE),
                        MakeEnumAccessor(&SemiRandomCircularMobilityModel::m_mode),
                        MakeEnumChecker(SemiRandomCircularMobilityModel::RANDOM_FLIGHT_MODE,"Random Flights",
                        SemiRandomCircularMobilityModel::RANDOM_WALK_MODE,"Random walks"))
            .AddAttribute("WalkChoice",
                        "A random variable which decides whether we walk to the upper orbit or lower orbit",
                        StringValue("ns3::UniformRandomVariable[Min=0.0|Max=2.0]"),
                        MakePointerAccessor(&SemiRandomCircularMobilityModel::m_rw_choice),
                        MakePointerChecker<RandomVariableStream>())
            .AddAttribute("FlightChoice",
                        "A random variable which decides which orbit to move to next",
                        StringValue("ns3::UniformRandomVariable[Min[0.0|Max=2.0]]"),
                        MakePointerAccessor(&SemiRandomCircularMobilityModel::m_rf_choice),
                        MakePointerChecker<RandomVariableStream>());
        return tid;
    }
    SemiRandomCircularMobilityModel::SemiRandomCircularMobilityModel()
    {
        m_helper.SetCenter(center);
    }
    SemiRandomCircularMobilityModel::~SemiRandomCircularMobilityModel()
    {
    }
    void SemiRandomCircularMobilityModel::DoConfigureAngVelHelper(const Vector &position)
    {
        // set angular velocity in the circular mobility model
        Vector pos_vector = Vector(center.x-position.x,center.y-position.y,-1*position.z);
        double current_orbit_radius = std::sqrt(pos_vector.x*pos_vector.x + pos_vector.y*pos_vector.y);
        double omega = m_tangential_vel/current_orbit_radius;
        double odd_or_even = std::fmod(current_orbit_radius,2.0);
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
    void SemiRandomCircularMobilityModel::DoSetPosition(const Vector &position)
    {
        m_helper.Update();
        m_helper.Pause();
        m_helper.SetPosition(position);
        m_event.Cancel();
        DoConfigureAngVelHelper(position);
        // change position here
        current_position = position;
        // if angular distance covered is close to 2\Pi, initiate Orbit Switch and radial motion
        Vector2D omega = m_helper.GetOmega();
        // omega.x represents the angular velocity and omega.y represents clockwise and counterclockwise directions
        time_to_travel = (2*M_PI)/omega.x;
        // find the nearest exchange point to our particular position
        double theta = m_helper.GetTheta(position);
        double step = (2*M_PI)/m_exchange_points;
        if(omega.y >0)
        {
            int number = std::ceil(theta/step);
            time_to_travel += ((number*2*M_PI/m_exchange_points)-theta)/omega.x;
        }
        else
        {
            int number = std::floor(theta/step);
            time_to_travel += (theta-(number*2*M_PI/m_exchange_points))/omega.x;
        }
        m_event = Simulator::ScheduleNow(&SemiRandomCircularMobilityModel::DoSurveil,this);
        time_travelled += time_step;
    }
    void SemiRandomCircularMobilityModel::DoSurveil()
    {
        m_vel_helper.Update();
        m_vel_helper.Pause();
        m_helper.Update();
        m_helper.Pause();
        // find out nearest exit from here on
        if(time_travelled >= time_to_travel)
        {
            // we travelled enough time on the orbit, time to switch orbits now
            // reset counters
            time_to_travel = 0;
            time_travelled = 0;
            Simulator::ScheduleNow(&SemiRandomCircularMobilityModel::DoOrbitSwitch,this);
            NotifyCourseChange();
            return;
        }
        m_helper.Unpause();
        m_event = Simulator::Schedule(Seconds(time_step),&SemiRandomCircularMobilityModel::DoSurveil,this);
        //m_helper.SetPosition(m_position);
    }
    void SemiRandomCircularMobilityModel::DoOrbitSwitch()
    {
        m_helper.Update();
        Vector cur_pos = m_helper.GetCurrentPosition();
        m_vel_helper.Pause();
        m_vel_helper.SetPosition(cur_pos);
        Vector travel_vec;
        // generate random number based on random walk or random flight mode
        
        if(m_mode == SemiRandomCircularMobilityModel::Mode::RANDOM_FLIGHT_MODE)
        {
            // TODO: Complete this later
            //double rand = m_rf_choice->GetValue();
            if(m_helper.GetRadius() >= m_max_orbit_rad - m_orbit_dist)
            {
                // if the drone is in the last orbit, only move downwards, not upwards
                travel_vec = Vector(center.x - cur_pos.x, center.y - cur_pos.y,cur_pos.z);

            }
            else if(m_helper.GetRadius() < 1.5*m_orbit_dist)
            {
                // same with when the orbit is the first orbit, only move upwards not downwards
                travel_vec = Vector(cur_pos.x-center.x, cur_pos.y-center.y ,cur_pos.z);
            }
            else
            {
                //double step = 2.0/number_of_orbits;
                //int orbit_number = std::floor(rand/step);
                
                travel_vec = Vector(cur_pos.x-center.x, cur_pos.y-center.y ,cur_pos.z);
            }
        }
        else
        {
            if(m_helper.GetRadius() >= m_max_orbit_rad - m_orbit_dist)
            {
                // if the drone is in the last orbit, only move downwards, not upwards
                travel_vec = Vector(center.x-cur_pos.x,center.y-cur_pos.y,cur_pos.z);

            }
            else if(m_helper.GetRadius() < 1.5*m_orbit_dist)
            {
                // same with when the orbit is the first orbit, only move upwards not downwards
                travel_vec = Vector(cur_pos.x-center.x,cur_pos.y-center.y,cur_pos.z);
            }
            else
            {
                double rand = m_rw_choice->GetValue();
                // distribution of the orbit
                if(rand >= 1.0)
                {
                    travel_vec = Vector(cur_pos.x-center.x,cur_pos.y-center.y,cur_pos.z);
                }
                else
                {
                    travel_vec = Vector(center.x-cur_pos.x,center.y-cur_pos.y,cur_pos.z);
                }        
            }
            double theta = m_helper.GetTheta(travel_vec);
            m_vel_helper.SetVelocity(Vector(m_radial_vel*std::cos(theta),m_radial_vel*std::sin(theta),0));
            m_vel_helper.Update();
            m_vel_helper.Unpause();
            m_event = Simulator::Schedule(Seconds(m_orbit_dist/m_radial_vel),
                                    &SemiRandomCircularMobilityModel::DoSurveil,this);
            NotifyCourseChange();
        }
    }
    Vector
    SemiRandomCircularMobilityModel::DoGetVelocity (void) const
    {
        
        return Vector(0.0,0.0,0.0);
    }
    void SemiRandomCircularMobilityModel::DoInitialize()
    {
        // do nothing, let it spin slowly
    }
    double SemiRandomCircularMobilityModel::DoGetRadius()
    {
        return m_helper.GetRadius();
    }
    Vector SemiRandomCircularMobilityModel::DoGetPosition(void) const
    {
        return m_helper.GetCurrentPosition();
    }
    int64_t SemiRandomCircularMobilityModel::DoAssignStreams (int64_t stream)
    {
        m_rw_choice->SetStream(stream);
        m_rf_choice->SetStream(stream+1);
        return 2;
    }
}
