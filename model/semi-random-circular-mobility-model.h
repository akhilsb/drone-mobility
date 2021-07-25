#ifndef SEMI_RANDOM_CIRCULAR_MOBILITY_MODEL_H
#define SEMI_RANDOM_CIRCULAR_MOBILITY_MODEL_H

#include "constant-angular-velocity-helper.h"
#include "constant-velocity-helper.h"
#include "mobility-model.h"
#include "position-allocator.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"

namespace ns3{
    
    class SemiRandomCircularMobilityModel : public MobilityModel 
    {
        public : 
        /**
         * Register this type with the TypeId system.
         * \return the object TypeId
         */
        static TypeId GetTypeId (void);
        SemiRandomCircularMobilityModel ();
        virtual ~SemiRandomCircularMobilityModel();
        enum Mode  {
            RANDOM_FLIGHT_MODE,
            RANDOM_WALK_MODE
        };
        /**
         * This mobility model is a newer model that focuses on 
         * continuous time oriented surveillance where all the mobile nodes
         * rotate around a given center while conducting surveillance in orbits. 
         * 
         * Motions that nodes undergo: 
         * 1. Surveillance - nodes go around orbits like how earth rotates around the sun, 
         * but with a circular orbit
         * 2. Radial movement - There must be gossip of information between nodes because of which 
         * we enable switching orbits by nodes. 
         * */

        private:
            void DoSurveil();
            void DoOrbitSwitch();
            void DoConfigureAngVelHelper(const Vector &position);
            void DoInitializePrivate(void);
            virtual void DoInitialize(void);
            virtual Vector DoGetPosition(void) const;
            virtual void DoSetPosition(const Vector &position);
            virtual Vector DoGetVelocity(void) const;
            virtual double DoGetRadius();
            virtual int64_t DoAssignStreams(int64_t);

        ConstantAngularVelocityHelper m_helper;
        ConstantVelocityHelper m_vel_helper;
        Vector2D center;
        Vector current_position;
        double radius;
        int number_of_orbits;
        double m_tangential_vel;
        double m_radial_vel;
        double m_orbit_dist;
        double m_max_orbit_rad;
        enum Mode m_mode;
        double time_step;
        EventId m_event; //!< stored event ID 
        double m_exchange_points;
        double time_to_travel=0.0;
        double time_travelled = 0.0;
        std::vector<double> exchangePoints;
        Ptr<RandomVariableStream> m_rw_choice;
        Ptr<RandomVariableStream> m_rf_choice;
    };
}
#endif