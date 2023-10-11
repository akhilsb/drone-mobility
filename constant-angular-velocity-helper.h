/**
 * This is a steady state helper - being in a constant angular velocity position is nothing but a 
 * steady state of parameters. 
 * 
 * Angular velocity \Omega is a fundamental metric for circular motion with different
 * radii and angular motion constraints. 
 * 
 * Radius of rotation - the radius at which steady state has to be maintained
 * 
 * The center of the circle at which the rotation has to be carried out - A 2D location. 
 * */
#ifndef CONSTANT_ANGULAR_VELOCITY_HELPER_H
#define CONSTANT_ANGULAR_VELOCITY_HELPER_H

#include "ns3/nstime.h"
#include "ns3/vector.h"
#include "ns3/box.h"

namespace ns3{
    class ConstantAngularVelocityHelper
    {
    public:
        ConstantAngularVelocityHelper();
        /**
         * Create object and set position
         * of the radius of the circle to revolve around
         * */
        ConstantAngularVelocityHelper(const Vector2D &center);
        /**
         * Create object and set center, omega
         * of the object
         * */
        ConstantAngularVelocityHelper(const Vector2D &center,
                                        const Vector2D &omega, const Vector &position);
        /**
         * Variables to set and update different variables to 
         * set random motion in effect. 
         * */
        void SetCenter(const Vector2D &center);
        void SetOmega(const Vector2D &omega);
        void SetPosition(const Vector &position);

        Vector2D GetOmega(void) const;
        Vector2D GetCenter(void) const;

        double GetTheta(const Vector &position) const;

        double GetRadius(void) const;

        Vector GetCurrentPosition(void) const;
        
        /**
         * State related function changes
         * like pause is for when the node stops, unpause is for 
         * when we want the node to move again and update is for when 
         * we update the position of the node. 
         * */
        void Pause(void) const;

        void Unpause(void) const;

        void Update(void) const;

        void SetRadius(double);

    private:
        Vector2D center;
        double radius;
        Vector2D m_omega;
        mutable Time m_lastUpdate; //!< time of last update
        mutable Vector m_position; //!< state variable for current position
        mutable bool m_paused;  //!< state variable for paused
    };
}
#endif