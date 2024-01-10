#ifndef NS3_SCRATCH_HYBRID_DRONE_MODEL_H
#define NS3_SCRATCH_HYBRID_DRONE_MODEL_H

#include "drone-zoom-model.h"
#include "constant-angular-velocity-helper.h"
#include "ns3/constant-velocity-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/position-allocator.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"

namespace ns3{
    
    class HybridDroneModel : public MobilityModel
    {
        public : 
        /**
         * Register this type with the TypeId system.
         * \return the object TypeId
         */
        static TypeId GetTypeId (void);
        HybridDroneModel ();
        virtual ~HybridDroneModel();

        enum Mode {
            Ascending,
            Descending,
            Static
        };


        double x_top;
        double y_top;
        double z_top;

        std::string mode;

        /** \brief Sanity check for debugging
        */
        void SanityCheck ();

        void SetMode(std::string m);

        std::string GetMode();

        void Surveil();

        void StopSurveil(Vector vel);

        void SaveTopPosition();

        Vector GetTopPosition();

        void SetVelocity(Vector vel);
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
         * 
         * This model is free of the exchange point orbit switching, drones switch orbits whenever they 
         * complete x amount of time in the orbit 
         * */

        private:
            void DoSurveil();
            void DoOrbitSwitch();
            void DoConfigureAngVelHelper(const Vector &position);
            void DoInitializePrivate(void);
            void UpdatePosition(void);
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
        Vector m_initial_position;
        double radius;
        int number_of_orbits;
        double m_tangential_vel;
        double m_epsilon;
        Time m_time_rotate;
        double m_rotating_time = 0.0;
        double m_radial_time = 0.0;
        double m_radial_total_time = 0.0;
        double m_radial_vel;
        double m_orbit_dist;
        double m_max_orbit_rad;
        double time_step;
        EventId m_event; //!< stored event ID 
        double time_to_travel=0.0;
        double time_travelled = 0.0;
        bool under_surveillance = false;
        Ptr<RandomVariableStream> m_rw_rf_choice;
        Ptr<RandomVariableStream> m_rw_choice;
        Ptr<RandomVariableStream> m_rf_choice;
    };
}
#endif