/*
* Velcro Physics:
* Copyright (c) 2017 Ian Qvist
*
* Original source Box2D:
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using Genbox.VelcroPhysics.Dynamics.Joints;
using Genbox.VelcroPhysics.Dynamics.Joints.Misc;

namespace Genbox.VelcroPhysics.Dynamics
{
    /// <summary>
    /// Creates deep copies of a <see cref="World" />. Every object reachable from the source world (bodies, fixtures,
    /// shapes and joints) is recreated inside the destination world. No object that belongs to the source world is ever added
    /// to the destination, so the two worlds never share any mutable state. Contacts are not copied; they are rediscovered by
    /// the destination world on its first time step and then only reference the newly created fixtures.
    /// </summary>
    internal static class WorldCloner
    {
        /// <summary>Deep-copies every body, fixture and joint from <paramref name="source" /> into <paramref name="destination" />.</summary>
        public static void Clone(World source, World destination)
        {
            // Bodies first. The dictionary is the identity map used to rewire every internal reference
            // (fixture -> body, joint -> body, gear joint -> joint) to the destination world.
            Dictionary<Body, Body> bodyMap = new Dictionary<Body, Body>(source.BodyList.Count);
            foreach (Body sourceBody in source.BodyList)
            {
                Body destinationBody = sourceBody.CloneDeep();
                bodyMap.Add(sourceBody, destinationBody);
                destination.AddBodyWithProxies(destinationBody);
            }

            // Joints that reference other joints (gear joints) must be created last, once every joint
            // they point at already exists in the destination world.
            List<GearJoint> deferredGearJoints = new List<GearJoint>();
            Dictionary<Joint, Joint> jointMap = new Dictionary<Joint, Joint>(source.JointList.Count);

            foreach (Joint sourceJoint in source.JointList)
            {
                if (sourceJoint is GearJoint gearJoint)
                {
                    deferredGearJoints.Add(gearJoint);
                    continue;
                }

                Joint destinationJoint = CloneJoint(sourceJoint, bodyMap, null);
                jointMap.Add(sourceJoint, destinationJoint);
                destination.AddJoint(destinationJoint);
            }

            foreach (GearJoint sourceGearJoint in deferredGearJoints)
            {
                GearJoint destinationGearJoint = (GearJoint)CloneJoint(sourceGearJoint, bodyMap, jointMap);
                jointMap.Add(sourceGearJoint, destinationGearJoint);
                destination.AddJoint(destinationGearJoint);
            }
        }

        private static Joint CloneJoint(Joint source, Dictionary<Body, Body> bodyMap, Dictionary<Joint, Joint> jointMap)
        {
            Body bodyA = bodyMap[source.BodyA];
            Body bodyB = source.BodyB == null ? null : bodyMap[source.BodyB];

            Joint copy = CreateJoint(source, bodyA, bodyB, jointMap);

            copy.CollideConnected = source.CollideConnected;
            copy.Breakpoint = source.Breakpoint;
            copy.Enabled = source.Enabled;
            copy.UserData = source.UserData;

            return copy;
        }

        private static Joint CreateJoint(Joint source, Body bodyA, Body bodyB, Dictionary<Joint, Joint> jointMap)
        {
            switch (source.JointType)
            {
                case JointType.Distance:
                {
                    DistanceJoint j = (DistanceJoint)source;
                    DistanceJoint copy = new DistanceJoint(bodyA, bodyB, j.LocalAnchorA, j.LocalAnchorB, false);
                    copy.MinLength = j.MinLength;
                    copy.MaxLength = j.MaxLength;
                    copy.Length = j.Length;
                    copy.Stiffness = j.Stiffness;
                    copy.Damping = j.Damping;
                    return copy;
                }
                case JointType.Revolute:
                {
                    RevoluteJoint j = (RevoluteJoint)source;
                    RevoluteJoint copy = new RevoluteJoint(bodyA, bodyB, j.LocalAnchorA, j.LocalAnchorB, false)
                    {
                        ReferenceAngle = j.ReferenceAngle
                    };
                    copy.SetLimits(j.LowerLimit, j.UpperLimit);
                    copy.LimitEnabled = j.LimitEnabled;
                    copy.MotorSpeed = j.MotorSpeed;
                    copy.MaxMotorTorque = j.MaxMotorTorque;
                    copy.MotorEnabled = j.MotorEnabled;
                    return copy;
                }
                case JointType.Prismatic:
                {
                    PrismaticJoint j = (PrismaticJoint)source;
                    PrismaticJoint copy = new PrismaticJoint(bodyA, bodyB, j.LocalAnchorA, j.LocalAnchorB, j.LocalXAxisA, false)
                    {
                        ReferenceAngle = j.ReferenceAngle
                    };
                    copy.SetLimits(j.LowerLimit, j.UpperLimit);
                    copy.LimitEnabled = j.LimitEnabled;
                    copy.MotorSpeed = j.MotorSpeed;
                    copy.MaxMotorForce = j.MaxMotorForce;
                    copy.MotorEnabled = j.MotorEnabled;
                    return copy;
                }
                case JointType.Wheel:
                {
                    WheelJoint j = (WheelJoint)source;
                    WheelJoint copy = new WheelJoint(bodyA, bodyB, j.LocalAnchorB, j.LocalXAxisA, false);
                    copy.LocalAnchorA = j.LocalAnchorA;
                    copy.LocalAnchorB = j.LocalAnchorB;
                    copy.SetLimits(j.LowerLimit, j.UpperLimit);
                    copy.EnableLimit = j.EnableLimit;
                    copy.MotorSpeed = j.MotorSpeed;
                    copy.MaxMotorTorque = j.MaxMotorTorque;
                    copy.MotorEnabled = j.MotorEnabled;
                    copy.Stiffness = j.Stiffness;
                    copy.Damping = j.Damping;
                    return copy;
                }
                case JointType.Weld:
                {
                    WeldJoint j = (WeldJoint)source;
                    WeldJoint copy = new WeldJoint(bodyA, bodyB, j.LocalAnchorA, j.LocalAnchorB, false)
                    {
                        ReferenceAngle = j.ReferenceAngle,
                        Stiffness = j.Stiffness,
                        Damping = j.Damping
                    };
                    return copy;
                }
                case JointType.Friction:
                {
                    FrictionJoint j = (FrictionJoint)source;
                    FrictionJoint copy = new FrictionJoint(bodyA, bodyB, j.LocalAnchorA, false)
                    {
                        LocalAnchorB = j.LocalAnchorB,
                        MaxForce = j.MaxForce,
                        MaxTorque = j.MaxTorque
                    };
                    return copy;
                }
                case JointType.Motor:
                {
                    MotorJoint j = (MotorJoint)source;
                    MotorJoint copy = new MotorJoint(bodyA, bodyB, false)
                    {
                        LinearOffset = j.LinearOffset,
                        AngularOffset = j.AngularOffset,
                        MaxForce = j.MaxForce,
                        MaxTorque = j.MaxTorque,
                        CorrectionFactor = j.CorrectionFactor
                    };
                    return copy;
                }
                case JointType.Pulley:
                {
                    PulleyJoint j = (PulleyJoint)source;
                    PulleyJoint copy = new PulleyJoint(bodyA, bodyB, j.LocalAnchorA, j.LocalAnchorB, j.WorldAnchorA, j.WorldAnchorB, j.Ratio, false)
                    {
                        LengthA = j.LengthA,
                        LengthB = j.LengthB
                    };
                    return copy;
                }
                case JointType.FixedMouse:
                {
                    FixedMouseJoint j = (FixedMouseJoint)source;
                    FixedMouseJoint copy = new FixedMouseJoint(bodyA, j.WorldAnchorB)
                    {
                        LocalAnchorA = j.LocalAnchorA,
                        MaxForce = j.MaxForce,
                        Stiffness = j.Stiffness,
                        Damping = j.Damping
                    };
                    return copy;
                }
                case JointType.Angle:
                {
                    AngleJoint j = (AngleJoint)source;
                    AngleJoint copy = new AngleJoint(bodyA, bodyB)
                    {
                        TargetAngle = j.TargetAngle,
                        BiasFactor = j.BiasFactor,
                        MaxImpulse = j.MaxImpulse,
                        Softness = j.Softness
                    };
                    return copy;
                }
                case JointType.Gear:
                {
                    GearJoint j = (GearJoint)source;
                    Joint jointA = jointMap[j.JointA];
                    Joint jointB = jointMap[j.JointB];
                    return new GearJoint(bodyA, bodyB, jointA, jointB, j.Ratio);
                }
                default:
                    // Unknown joint types cannot be reconstructed without knowledge of their state.
                    // Throwing leaves both worlds untouched up to this point (the copy has not been
                    // added anywhere), so corrupted/unsupported data never damages an existing world.
                    throw new InvalidOperationException("Joint type '" + source.JointType + "' cannot be cloned.");
            }
        }
    }
}
