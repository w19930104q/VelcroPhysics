using System;
using System.Collections.Generic;
using Genbox.VelcroPhysics.Collision.ContactSystem;
using Genbox.VelcroPhysics.Collision.Shapes;
using Genbox.VelcroPhysics.Dynamics;
using Genbox.VelcroPhysics.Dynamics.Joints;
using Genbox.VelcroPhysics.Dynamics.Joints.Misc;
using Genbox.VelcroPhysics.Factories;
using Microsoft.Xna.Framework;
using Xunit;

namespace Genbox.VelcroPhysics.Tests.Tests
{
    /// <summary>
    /// Verifies that <see cref="World.Clone" /> and the snapshot APIs
    /// (<see cref="World.CreateSnapshot" />, <see cref="World.RestoreSnapshot" />, <see cref="WorldSnapshot.CreateWorld" />)
    /// produce fully independent worlds: every body, fixture, joint and contact reference points into the new world,
    /// no mutable state is shared between worlds, and corrupt input never damages the world being restored into.
    /// The scenario uses five rigid bodies and three joints, as in the original bug report.
    /// </summary>
    public class WorldCloneTest
    {
        private static World CreateSampleWorld()
        {
            World world = new World(new Vector2(0, -9.81f));
            world.WarmStartingEnabled = false;
            world.ContinuousPhysicsEnabled = false;

            Body ground = BodyFactory.CreateEdge(world, new Vector2(-40, 0), new Vector2(40, 0));

            Body b1 = BodyFactory.CreateRectangle(world, 1, 1, 1, new Vector2(-5, 2));
            b1.BodyType = BodyType.Dynamic;
            b1.Restitution = 0.4f;
            b1.Friction = 0.7f;

            Body b2 = BodyFactory.CreateCircle(world, 0.5f, 1, new Vector2(-3, 2));
            b2.BodyType = BodyType.Dynamic;
            b2.IsBullet = true;

            Body b3 = BodyFactory.CreateRectangle(world, 2, 1, 1, new Vector2(3, 2));
            b3.BodyType = BodyType.Dynamic;
            b3.SleepingAllowed = false;

            Body b4 = BodyFactory.CreateCircle(world, 0.7f, 1, new Vector2(5, 2));
            b4.BodyType = BodyType.Dynamic;

            Assert.Equal(5, world.BodyList.Count);

            RevoluteJoint revolute = JointFactory.CreateRevoluteJoint(world, ground, b1, new Vector2(-5, 2));
            revolute.MotorEnabled = true;
            revolute.MotorSpeed = 0.5f;
            revolute.MaxMotorTorque = 10;
            revolute.SetLimits(-1, 1);
            revolute.LimitEnabled = true;

            DistanceJoint distance = JointFactory.CreateDistanceJoint(world, b1, b2, b1.Position, b2.Position);
            distance.Stiffness = 5;
            distance.Damping = 0.2f;

            JointFactory.CreateWeldJoint(world, b3, b4, b3.Position, b4.Position);

            Assert.Equal(3, world.JointList.Count);

            // Step long enough for the falling bodies to hit the ground and create contacts.
            for (int i = 0; i < 60; i++)
                world.Step(1f / 60f);

            Assert.True(world.ContactManager.ContactCount > 0);
            Assert.True(AllContactsReferenceBodies(world, world.BodyList));

            return world;
        }

        [Fact]
        public void CloneRebuildsAllReferencesInsideTheClone()
        {
            World source = CreateSampleWorld();

            World clone = source.Clone();

            Assert.NotSame(source, clone);
            Assert.Equal(5, clone.BodyList.Count);
            Assert.Equal(3, clone.JointList.Count);

            Assert.False(SharesAnyBody(source, clone));
            Assert.False(SharesAnyFixture(source, clone));
            Assert.False(SharesAnyJoint(source, clone));
            Assert.False(SharesAnyShape(source, clone));

            // Joint body references must point at clone bodies.
            foreach (Joint joint in clone.JointList)
            {
                Assert.Contains(joint.BodyA, clone.BodyList);
                if (joint.BodyB != null)
                    Assert.Contains(joint.BodyB, clone.BodyList);
            }

            // Fixture back-references must point at clone bodies.
            foreach (Body body in clone.BodyList)
                foreach (Fixture fixture in body.FixtureList)
                    Assert.Same(body, fixture.Body);

            // Body/joint graph edges must stay inside the clone.
            Assert.True(AllBodyGraphLinksInternal(clone));

            // Joint state round-trips.
            RevoluteJoint revolute = FindJoint<RevoluteJoint>(clone);
            Assert.True(revolute.MotorEnabled);
            Assert.True(revolute.LimitEnabled);
            Assert.Equal(0.5f, revolute.MotorSpeed, 6);

            DistanceJoint distance = FindJoint<DistanceJoint>(clone);
            Assert.Equal(5f, distance.Stiffness, 6);
            Assert.Equal(0.2f, distance.Damping, 6);

            // Body/fixture state round-trips.
            Assert.True(clone.BodyList[2].IsBullet);
            Assert.False(clone.BodyList[3].SleepingAllowed);
            Assert.Equal(0.4f, clone.BodyList[1].FixtureList[0].Restitution, 6);

            // Broad-phase proxies are built, contacts are not copied.
            Assert.True(clone.ProxyCount > 0);
            Assert.Equal(0, clone.ContactManager.ContactCount);

            // Stepping rebuilds contacts, and every rebuilt contact references clone objects only.
            for (int i = 0; i < 30; i++)
                clone.Step(1f / 60f);

            Assert.True(clone.ContactManager.ContactCount > 0);
            Assert.True(AllContactsReferenceBodies(clone, clone.BodyList));
            Assert.True(NoContactReferencesBodies(clone, source.BodyList));
        }

        [Fact]
        public void CloneDoesNotShareMutableStateWithSource()
        {
            World source = CreateSampleWorld();
            World clone = source.Clone();

            int sourceBodyCount = source.BodyList.Count;
            int sourceJointCount = source.JointList.Count;
            Vector2 sourcePosition = source.BodyList[1].Position;

            clone.BodyList[1].ApplyForce(new Vector2(100, 100));
            clone.BodyList[1].Position = new Vector2(99, 99);
            clone.RemoveBody(clone.BodyList[4]);
            clone.RemoveJoint(clone.JointList[0]);

            for (int i = 0; i < 5; i++)
                clone.Step(1f / 60f);

            Assert.Equal(sourceBodyCount, source.BodyList.Count);
            Assert.Equal(sourceJointCount, source.JointList.Count);
            Assert.NotEqual(new Vector2(99, 99), source.BodyList[1].Position);
            Assert.True((source.BodyList[1].Position - sourcePosition).Length() < 1e-4f);
        }

        [Fact]
        public void SnapshotRoundTripProducesIndependentWorlds()
        {
            World source = CreateSampleWorld();
            WorldSnapshot snapshot = source.CreateSnapshot();

            Assert.Equal(5, snapshot.Bodies.Count);
            Assert.Equal(3, snapshot.Joints.Count);

            // Snapshot shapes are data copies, never the live engine shapes.
            List<Shape> sourceShapes = AllShapes(source);
            foreach (BodySnapshot bodySnapshot in snapshot.Bodies)
                foreach (FixtureSnapshot fixtureSnapshot in bodySnapshot.Fixtures)
                    Assert.DoesNotContain(fixtureSnapshot.Shape, sourceShapes);

            World restored1 = snapshot.CreateWorld();
            World restored2 = snapshot.CreateWorld();

            Assert.Equal(5, restored1.BodyList.Count);
            Assert.Equal(3, restored1.JointList.Count);
            Assert.False(SharesAnyBody(restored1, restored2));
            Assert.False(SharesAnyFixture(restored1, restored2));
            Assert.False(SharesAnyJoint(restored1, restored2));

            foreach (Joint joint in restored1.JointList)
            {
                Assert.Contains(joint.BodyA, restored1.BodyList);
                if (joint.BodyB != null)
                    Assert.Contains(joint.BodyB, restored1.BodyList);
            }

            for (int i = 0; i < 30; i++)
                restored1.Step(1f / 60f);

            Assert.True(restored1.ContactManager.ContactCount > 0);
            Assert.True(AllContactsReferenceBodies(restored1, restored1.BodyList));
        }

        [Fact]
        public void RestoreSnapshotReplacesAnExistingWorld()
        {
            World source = CreateSampleWorld();
            WorldSnapshot snapshot = source.CreateSnapshot();

            World world = snapshot.CreateWorld();
            int proxyCount = world.ProxyCount;

            world.RestoreSnapshot(snapshot);

            Assert.Equal(5, world.BodyList.Count);
            Assert.Equal(3, world.JointList.Count);
            Assert.Equal(0, world.ContactManager.ContactCount);
            Assert.Equal(proxyCount, world.ProxyCount);

            for (int i = 0; i < 30; i++)
                world.Step(1f / 60f);

            Assert.True(AllContactsReferenceBodies(world, world.BodyList));

            // The snapshot itself is not consumed: restoring it again yields another fresh world.
            World again = new World(Vector2.Zero);
            again.RestoreSnapshot(snapshot);

            Assert.Equal(5, again.BodyList.Count);
            Assert.Equal(3, again.JointList.Count);
            Assert.False(SharesAnyBody(again, world));
        }

        [Fact]
        public void RestoreSnapshotRejectsNullWithoutTouchingTheWorld()
        {
            World world = CreateSampleWorld();
            Body[] bodiesBefore = world.BodyList.ToArray();
            Joint[] jointsBefore = world.JointList.ToArray();

            Assert.Throws<ArgumentNullException>(() => world.RestoreSnapshot(null));

            Assert.Equal(5, world.BodyList.Count);
            Assert.Equal(3, world.JointList.Count);
            Assert.Same(bodiesBefore[0], world.BodyList[0]);
            Assert.Same(jointsBefore[0], world.JointList[0]);

            // The world still simulates correctly after the rejected restore.
            for (int i = 0; i < 10; i++)
                world.Step(1f / 60f);

            Assert.Equal(5, world.BodyList.Count);
        }

        [Fact]
        public void GearJointIsClonedAndRewired()
        {
            World world = new World(new Vector2(0, -9.81f));
            Body a = BodyFactory.CreateRectangle(world, 1, 1, 1, new Vector2(0, 0));
            Body b = BodyFactory.CreateRectangle(world, 1, 1, 1, new Vector2(2, 0));
            Body c = BodyFactory.CreateRectangle(world, 1, 1, 1, new Vector2(4, 0));
            a.BodyType = BodyType.Dynamic;
            b.BodyType = BodyType.Dynamic;
            c.BodyType = BodyType.Dynamic;

            RevoluteJoint r1 = JointFactory.CreateRevoluteJoint(world, a, b, a.Position);
            RevoluteJoint r2 = JointFactory.CreateRevoluteJoint(world, b, c, b.Position);
            GearJoint gear = JointFactory.CreateGearJoint(world, a, c, r1, r2, 2.5f);

            World clone = world.Clone();
            GearJoint clonedGear = FindJoint<GearJoint>(clone);

            Assert.NotNull(clonedGear);
            Assert.NotSame(gear, clonedGear);
            Assert.NotSame(r1, clonedGear.JointA);
            Assert.NotSame(r2, clonedGear.JointB);
            Assert.Contains(clonedGear.JointA, clone.JointList);
            Assert.Contains(clonedGear.JointB, clone.JointList);
            Assert.Equal(2.5f, clonedGear.Ratio, 6);

            // A gear joint must also survive the data-only snapshot path.
            World restored = world.CreateSnapshot().CreateWorld();
            Assert.NotNull(FindJoint<GearJoint>(restored));

            for (int i = 0; i < 10; i++)
            {
                clone.Step(1f / 60f);
                restored.Step(1f / 60f);
            }
        }

        [Fact]
        public void FixedJointIsClonedAndRewired()
        {
            World world = new World(new Vector2(0, -9.81f));
            Body body = BodyFactory.CreateCircle(world, 1, 1, new Vector2(0, 5));
            body.BodyType = BodyType.Dynamic;
            FixedMouseJoint mouse = JointFactory.CreateFixedMouseJoint(world, body, new Vector2(0, 5));

            World clone = world.Clone();
            FixedMouseJoint clonedMouse = FindJoint<FixedMouseJoint>(clone);

            Assert.NotNull(clonedMouse);
            Assert.NotSame(mouse, clonedMouse);
            Assert.Same(clone.BodyList[0], clonedMouse.BodyA);

            for (int i = 0; i < 10; i++)
                clone.Step(1f / 60f);
        }

        private static bool AllContactsReferenceBodies(World world, List<Body> allowed)
        {
            foreach (Body body in world.BodyList)
            {
                ContactEdge edge = body.ContactList;
                while (edge != null)
                {
                    if (!allowed.Contains(edge.Contact.FixtureA.Body) ||
                        !allowed.Contains(edge.Contact.FixtureB.Body))
                        return false;

                    edge = edge.Next;
                }
            }

            return true;
        }

        private static bool NoContactReferencesBodies(World world, List<Body> forbidden)
        {
            foreach (Body body in world.BodyList)
            {
                ContactEdge edge = body.ContactList;
                while (edge != null)
                {
                    if (forbidden.Contains(edge.Contact.FixtureA.Body) ||
                        forbidden.Contains(edge.Contact.FixtureB.Body))
                        return false;

                    edge = edge.Next;
                }
            }

            return true;
        }

        private static bool AllBodyGraphLinksInternal(World world)
        {
            foreach (Body body in world.BodyList)
            {
                JointEdge edge = body.JointList;
                while (edge != null)
                {
                    if (!world.JointList.Contains(edge.Joint))
                        return false;

                    if (edge.Other != null && !world.BodyList.Contains(edge.Other))
                        return false;

                    edge = edge.Next;
                }
            }

            return true;
        }

        private static bool SharesAnyBody(World a, World b)
        {
            foreach (Body body in a.BodyList)
                if (b.BodyList.Contains(body))
                    return true;

            return false;
        }

        private static bool SharesAnyFixture(World a, World b)
        {
            foreach (Body bodyA in a.BodyList)
                foreach (Fixture fixture in bodyA.FixtureList)
                    foreach (Body bodyB in b.BodyList)
                        if (bodyB.FixtureList.Contains(fixture))
                            return true;

            return false;
        }

        private static bool SharesAnyJoint(World a, World b)
        {
            foreach (Joint joint in a.JointList)
                if (b.JointList.Contains(joint))
                    return true;

            return false;
        }

        private static List<Shape> AllShapes(World world)
        {
            List<Shape> shapes = new List<Shape>();
            foreach (Body body in world.BodyList)
                foreach (Fixture fixture in body.FixtureList)
                    shapes.Add(fixture.Shape);

            return shapes;
        }

        private static bool SharesAnyShape(World a, World b)
        {
            List<Shape> otherShapes = AllShapes(b);
            foreach (Shape shape in AllShapes(a))
                if (otherShapes.Contains(shape))
                    return true;

            return false;
        }

        private static T FindJoint<T>(World world) where T : Joint
        {
            foreach (Joint joint in world.JointList)
                if (joint is T typed)
                    return typed;

            return null;
        }
    }
}
