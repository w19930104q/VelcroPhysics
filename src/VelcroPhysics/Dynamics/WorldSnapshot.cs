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
using Genbox.VelcroPhysics.Collision.Filtering;
using Genbox.VelcroPhysics.Collision.Shapes;
using Genbox.VelcroPhysics.Dynamics.Joints;
using Genbox.VelcroPhysics.Dynamics.Joints.Misc;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Dynamics
{
    /// <summary>
    /// An immutable, serializable snapshot of a <see cref="World" />. A snapshot contains only plain data describing
    /// the world's settings, bodies, fixtures and joints. It holds no references to live engine objects, so it can be stored,
    /// passed around or written to disk without any risk of mutating the world it came from. It is rebuilt into live engine
    /// objects by <see cref="ApplyTo" /> (or <see cref="World.RestoreSnapshot" />).
    /// </summary>
    public sealed class WorldSnapshot
    {
        internal WorldSnapshot()
        {
            Bodies = new List<BodySnapshot>(0);
            Joints = new List<JointSnapshot>(0);
        }

        public Vector2 Gravity { get; internal set; }
        public bool Enabled { get; internal set; }
        public bool WarmStartingEnabled { get; internal set; }
        public bool SleepingAllowed { get; internal set; }
        public bool ContinuousPhysicsEnabled { get; internal set; }
        public bool EnableDiagnostics { get; internal set; }

        public List<BodySnapshot> Bodies { get; }

        public List<JointSnapshot> Joints { get; }

        internal static WorldSnapshot Capture(World world)
        {
            WorldSnapshot snapshot = new WorldSnapshot
            {
                Gravity = world.Gravity,
                Enabled = world.Enabled,
                WarmStartingEnabled = world.WarmStartingEnabled,
                SleepingAllowed = world.SleepingAllowed,
                ContinuousPhysicsEnabled = world.ContinuousPhysicsEnabled,
                EnableDiagnostics = world.EnableDiagnostics
            };

            // The index of each body in this list is its identity within the snapshot.
            // Joints refer to bodies through those indices, never through object references.
            Dictionary<Body, int> bodyIndices = new Dictionary<Body, int>(world.BodyList.Count);
            for (int i = 0; i < world.BodyList.Count; i++)
            {
                Body body = world.BodyList[i];
                bodyIndices.Add(body, i);
                snapshot.Bodies.Add(body.CreateSnapshot());
            }

            Dictionary<Joint, int> jointIndices = new Dictionary<Joint, int>(world.JointList.Count);
            for (int i = 0; i < world.JointList.Count; i++)
            {
                jointIndices.Add(world.JointList[i], i);
            }

            foreach (Joint joint in world.JointList)
            {
                snapshot.Joints.Add(JointSnapshot.Capture(joint, bodyIndices, jointIndices));
            }

            return snapshot;
        }

        /// <summary>
        /// Builds a set of live engine objects from this snapshot and adds them to <paramref name="world" />. The
        /// snapshot is read-only during this process, so it can be applied repeatedly and to more than one world; every
        /// application creates a fresh, independent object graph.
        /// </summary>
        internal void ApplyTo(World world)
        {
            world.Enabled = Enabled;
            world.WarmStartingEnabled = WarmStartingEnabled;
            world.SleepingAllowed = SleepingAllowed;
            world.ContinuousPhysicsEnabled = ContinuousPhysicsEnabled;
            world.EnableDiagnostics = EnableDiagnostics;

            Body[] bodies = new Body[Bodies.Count];
            for (int i = 0; i < Bodies.Count; i++)
            {
                bodies[i] = Bodies[i].CreateBody();
                world.AddBodyWithProxies(bodies[i]);
            }

            // Gear joints reference other joints and therefore have to be built last.
            List<int> gearJointIndices = new List<int>();
            Joint[] joints = new Joint[Joints.Count];

            for (int i = 0; i < Joints.Count; i++)
            {
                if (Joints[i].Type == JointType.Gear)
                {
                    gearJointIndices.Add(i);
                    continue;
                }

                joints[i] = Joints[i].CreateJoint(bodies, null);
                world.AddJoint(joints[i]);
            }

            foreach (int i in gearJointIndices)
            {
                joints[i] = Joints[i].CreateJoint(bodies, joints);
                world.AddJoint(joints[i]);
            }

            world._newContacts = true;
        }

        /// <summary>Creates a brand new world containing a deep copy of everything stored in this snapshot.</summary>
        public World CreateWorld()
        {
            World world = new World(Gravity);
            ApplyTo(world);
            return world;
        }
    }

    /// <summary>Serializable state of a single <see cref="Body" /> and all of its fixtures.</summary>
    public sealed class BodySnapshot
    {
        internal BodySnapshot()
        {
            Fixtures = new List<FixtureSnapshot>(0);
        }

        public BodyType BodyType { get; internal set; }
        public BodyFlags Flags { get; internal set; }
        public Vector2 Position { get; internal set; }
        public float Angle { get; internal set; }
        public Vector2 LocalCenter { get; internal set; }
        public Vector2 LinearVelocity { get; internal set; }
        public float AngularVelocity { get; internal set; }
        public float LinearDamping { get; internal set; }
        public float AngularDamping { get; internal set; }
        public float GravityScale { get; internal set; }
        public float Mass { get; internal set; }
        public float Inertia { get; internal set; }
        public float SleepTime { get; internal set; }
        public object UserData { get; internal set; }

        public List<FixtureSnapshot> Fixtures { get; }

        internal Body CreateBody()
        {
            return Body.CreateFromSnapshot(this);
        }
    }

    /// <summary>Serializable state of a single <see cref="Fixture" />.</summary>
    public sealed class FixtureSnapshot
    {
        internal FixtureSnapshot()
        {
        }

        public float Friction { get; internal set; }
        public float Restitution { get; internal set; }
        public float RestitutionThreshold { get; internal set; }
        public short CollisionGroup { get; internal set; }
        public Category CollisionCategories { get; internal set; }
        public Category CollidesWith { get; internal set; }
        public Category IgnoreCcdWith { get; internal set; }
        public bool IsSensor { get; internal set; }
        public Shape Shape { get; internal set; }
        public object UserData { get; internal set; }

        internal Fixture CreateFixture(Body body)
        {
            return Fixture.CreateFromSnapshot(this, body);
        }
    }

    /// <summary>
    /// Serializable state of a single <see cref="Joint" />. Bodies and (for gear joints) other joints are
    /// referenced by their index within the snapshot, never by object reference.
    /// </summary>
    public sealed class JointSnapshot
    {
        internal JointSnapshot()
        {
        }

        public JointType Type { get; internal set; }
        public int BodyAIndex { get; internal set; }
        public int BodyBIndex { get; internal set; }
        public bool CollideConnected { get; internal set; }
        public bool Enabled { get; internal set; }
        public float Breakpoint { get; internal set; }
        public object UserData { get; internal set; }

        // Geometry/limits shared by several joint types. Unused fields stay at their default.
        public Vector2 LocalAnchorA { get; internal set; }
        public Vector2 LocalAnchorB { get; internal set; }
        public Vector2 LocalAxisA { get; internal set; }
        public Vector2 WorldAnchorA { get; internal set; }
        public Vector2 WorldAnchorB { get; internal set; }
        public float ReferenceAngle { get; internal set; }
        public bool LimitEnabled { get; internal set; }
        public bool MotorEnabled { get; internal set; }
        public float LowerLimit { get; internal set; }
        public float UpperLimit { get; internal set; }
        public float MotorSpeed { get; internal set; }
        public float MaxMotorTorque { get; internal set; }
        public float MaxMotorForce { get; internal set; }
        public float Stiffness { get; internal set; }
        public float Damping { get; internal set; }
        public float MaxForce { get; internal set; }
        public float MaxTorque { get; internal set; }
        public float MaxImpulse { get; internal set; }
        public float Softness { get; internal set; }
        public float BiasFactor { get; internal set; }
        public float TargetAngle { get; internal set; }
        public Vector2 LinearOffset { get; internal set; }
        public float AngularOffset { get; internal set; }
        public float CorrectionFactor { get; internal set; }
        public float Length { get; internal set; }
        public float MinLength { get; internal set; }
        public float MaxLength { get; internal set; }
        public float Ratio { get; internal set; }
        public int JointAIndex { get; internal set; } = -1;
        public int JointBIndex { get; internal set; } = -1;

        internal static JointSnapshot Capture(Joint joint, Dictionary<Body, int> bodyIndices, Dictionary<Joint, int> jointIndices)
        {
            JointSnapshot snapshot = new JointSnapshot
            {
                Type = joint.JointType,
                BodyAIndex = bodyIndices[joint.BodyA],
                BodyBIndex = joint.BodyB == null ? -1 : bodyIndices[joint.BodyB],
                CollideConnected = joint.CollideConnected,
                Enabled = joint._enabled,
                Breakpoint = joint.Breakpoint,
                UserData = joint.UserData
            };

            switch (joint)
            {
                case DistanceJoint j:
                    snapshot.LocalAnchorA = j.LocalAnchorA;
                    snapshot.LocalAnchorB = j.LocalAnchorB;
                    snapshot.Length = j.Length;
                    snapshot.MinLength = j.MinLength;
                    snapshot.MaxLength = j.MaxLength;
                    snapshot.Stiffness = j.Stiffness;
                    snapshot.Damping = j.Damping;
                    break;
                case RevoluteJoint j:
                    snapshot.LocalAnchorA = j.LocalAnchorA;
                    snapshot.LocalAnchorB = j.LocalAnchorB;
                    snapshot.ReferenceAngle = j.ReferenceAngle;
                    snapshot.LimitEnabled = j.LimitEnabled;
                    snapshot.LowerLimit = j.LowerLimit;
                    snapshot.UpperLimit = j.UpperLimit;
                    snapshot.MotorEnabled = j.MotorEnabled;
                    snapshot.MotorSpeed = j.MotorSpeed;
                    snapshot.MaxMotorTorque = j.MaxMotorTorque;
                    break;
                case PrismaticJoint j:
                    snapshot.LocalAnchorA = j.LocalAnchorA;
                    snapshot.LocalAnchorB = j.LocalAnchorB;
                    snapshot.LocalAxisA = j.LocalXAxisA;
                    snapshot.ReferenceAngle = j.ReferenceAngle;
                    snapshot.LimitEnabled = j.LimitEnabled;
                    snapshot.LowerLimit = j.LowerLimit;
                    snapshot.UpperLimit = j.UpperLimit;
                    snapshot.MotorEnabled = j.MotorEnabled;
                    snapshot.MotorSpeed = j.MotorSpeed;
                    snapshot.MaxMotorForce = j.MaxMotorForce;
                    break;
                case WheelJoint j:
                    snapshot.LocalAnchorA = j.LocalAnchorA;
                    snapshot.LocalAnchorB = j.LocalAnchorB;
                    snapshot.LocalAxisA = j.LocalXAxisA;
                    snapshot.LimitEnabled = j.EnableLimit;
                    snapshot.LowerLimit = j.LowerLimit;
                    snapshot.UpperLimit = j.UpperLimit;
                    snapshot.MotorEnabled = j.MotorEnabled;
                    snapshot.MotorSpeed = j.MotorSpeed;
                    snapshot.MaxMotorTorque = j.MaxMotorTorque;
                    snapshot.Stiffness = j.Stiffness;
                    snapshot.Damping = j.Damping;
                    break;
                case WeldJoint j:
                    snapshot.LocalAnchorA = j.LocalAnchorA;
                    snapshot.LocalAnchorB = j.LocalAnchorB;
                    snapshot.ReferenceAngle = j.ReferenceAngle;
                    snapshot.Stiffness = j.Stiffness;
                    snapshot.Damping = j.Damping;
                    break;
                case FrictionJoint j:
                    snapshot.LocalAnchorA = j.LocalAnchorA;
                    snapshot.LocalAnchorB = j.LocalAnchorB;
                    snapshot.MaxForce = j.MaxForce;
                    snapshot.MaxTorque = j.MaxTorque;
                    break;
                case MotorJoint j:
                    snapshot.LinearOffset = j.LinearOffset;
                    snapshot.AngularOffset = j.AngularOffset;
                    snapshot.MaxForce = j.MaxForce;
                    snapshot.MaxTorque = j.MaxTorque;
                    snapshot.CorrectionFactor = j.CorrectionFactor;
                    break;
                case PulleyJoint j:
                    snapshot.LocalAnchorA = j.LocalAnchorA;
                    snapshot.LocalAnchorB = j.LocalAnchorB;
                    snapshot.WorldAnchorA = j.WorldAnchorA;
                    snapshot.WorldAnchorB = j.WorldAnchorB;
                    snapshot.Length = j.LengthA;
                    snapshot.MinLength = j.LengthB;
                    snapshot.Ratio = j.Ratio;
                    break;
                case FixedMouseJoint j:
                    snapshot.LocalAnchorA = j.LocalAnchorA;
                    snapshot.WorldAnchorB = j.WorldAnchorB;
                    snapshot.MaxForce = j.MaxForce;
                    snapshot.Stiffness = j.Stiffness;
                    snapshot.Damping = j.Damping;
                    break;
                case AngleJoint j:
                    snapshot.TargetAngle = j.TargetAngle;
                    snapshot.BiasFactor = j.BiasFactor;
                    snapshot.MaxImpulse = j.MaxImpulse;
                    snapshot.Softness = j.Softness;
                    break;
                case GearJoint j:
                    snapshot.JointAIndex = jointIndices[j.JointA];
                    snapshot.JointBIndex = jointIndices[j.JointB];
                    snapshot.Ratio = j.Ratio;
                    break;
                default:
                    throw new InvalidOperationException("Joint type '" + joint.JointType + "' cannot be serialized.");
            }

            return snapshot;
        }

        internal Joint CreateJoint(Body[] bodies, Joint[] joints)
        {
            Body bodyA = bodies[BodyAIndex];
            Body bodyB = BodyBIndex == -1 ? null : bodies[BodyBIndex];

            Joint joint;
            switch (Type)
            {
                case JointType.Distance:
                {
                    DistanceJoint j = new DistanceJoint(bodyA, bodyB, LocalAnchorA, LocalAnchorB, false);
                    j.MinLength = MinLength;
                    j.MaxLength = MaxLength;
                    j.Length = Length;
                    j.Stiffness = Stiffness;
                    j.Damping = Damping;
                    joint = j;
                    break;
                }
                case JointType.Revolute:
                {
                    RevoluteJoint j = new RevoluteJoint(bodyA, bodyB, LocalAnchorA, LocalAnchorB, false)
                    {
                        ReferenceAngle = ReferenceAngle
                    };
                    j.SetLimits(LowerLimit, UpperLimit);
                    j.LimitEnabled = LimitEnabled;
                    j.MotorSpeed = MotorSpeed;
                    j.MaxMotorTorque = MaxMotorTorque;
                    j.MotorEnabled = MotorEnabled;
                    joint = j;
                    break;
                }
                case JointType.Prismatic:
                {
                    PrismaticJoint j = new PrismaticJoint(bodyA, bodyB, LocalAnchorA, LocalAnchorB, LocalAxisA, false)
                    {
                        ReferenceAngle = ReferenceAngle
                    };
                    j.SetLimits(LowerLimit, UpperLimit);
                    j.LimitEnabled = LimitEnabled;
                    j.MotorSpeed = MotorSpeed;
                    j.MaxMotorForce = MaxMotorForce;
                    j.MotorEnabled = MotorEnabled;
                    joint = j;
                    break;
                }
                case JointType.Wheel:
                {
                    WheelJoint j = new WheelJoint(bodyA, bodyB, LocalAnchorB, LocalAxisA, false);
                    j.LocalAnchorA = LocalAnchorA;
                    j.LocalAnchorB = LocalAnchorB;
                    j.SetLimits(LowerLimit, UpperLimit);
                    j.EnableLimit = LimitEnabled;
                    j.MotorSpeed = MotorSpeed;
                    j.MaxMotorTorque = MaxMotorTorque;
                    j.MotorEnabled = MotorEnabled;
                    j.Stiffness = Stiffness;
                    j.Damping = Damping;
                    joint = j;
                    break;
                }
                case JointType.Weld:
                {
                    WeldJoint j = new WeldJoint(bodyA, bodyB, LocalAnchorA, LocalAnchorB, false)
                    {
                        ReferenceAngle = ReferenceAngle,
                        Stiffness = Stiffness,
                        Damping = Damping
                    };
                    joint = j;
                    break;
                }
                case JointType.Friction:
                {
                    FrictionJoint j = new FrictionJoint(bodyA, bodyB, LocalAnchorA, false)
                    {
                        LocalAnchorB = LocalAnchorB,
                        MaxForce = MaxForce,
                        MaxTorque = MaxTorque
                    };
                    joint = j;
                    break;
                }
                case JointType.Motor:
                {
                    MotorJoint j = new MotorJoint(bodyA, bodyB, false)
                    {
                        LinearOffset = LinearOffset,
                        AngularOffset = AngularOffset,
                        MaxForce = MaxForce,
                        MaxTorque = MaxTorque,
                        CorrectionFactor = CorrectionFactor
                    };
                    joint = j;
                    break;
                }
                case JointType.Pulley:
                {
                    PulleyJoint j = new PulleyJoint(bodyA, bodyB, LocalAnchorA, LocalAnchorB, WorldAnchorA, WorldAnchorB, Ratio, false)
                    {
                        LengthA = Length,
                        LengthB = MinLength
                    };
                    joint = j;
                    break;
                }
                case JointType.FixedMouse:
                {
                    FixedMouseJoint j = new FixedMouseJoint(bodyA, WorldAnchorB)
                    {
                        LocalAnchorA = LocalAnchorA,
                        MaxForce = MaxForce,
                        Stiffness = Stiffness,
                        Damping = Damping
                    };
                    joint = j;
                    break;
                }
                case JointType.Angle:
                {
                    AngleJoint j = new AngleJoint(bodyA, bodyB)
                    {
                        TargetAngle = TargetAngle,
                        BiasFactor = BiasFactor,
                        MaxImpulse = MaxImpulse,
                        Softness = Softness
                    };
                    joint = j;
                    break;
                }
                case JointType.Gear:
                {
                    if (joints == null || JointAIndex < 0 || JointBIndex < 0)
                        throw new InvalidOperationException("Gear joint snapshot is missing its referenced joints.");

                    joint = new GearJoint(bodyA, bodyB, joints[JointAIndex], joints[JointBIndex], Ratio);
                    break;
                }
                default:
                    throw new InvalidOperationException("Joint type '" + Type + "' cannot be deserialized.");
            }

            joint.CollideConnected = CollideConnected;
            joint.Enabled = Enabled;
            joint.Breakpoint = Breakpoint;
            joint.UserData = UserData;
            return joint;
        }
    }
}
