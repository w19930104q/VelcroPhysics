using System.Collections.Generic;
using Genbox.VelcroPhysics.Collision.ContactSystem;
using Genbox.VelcroPhysics.Collision.Narrowphase;
using Genbox.VelcroPhysics.Collision.Shapes;
using Genbox.VelcroPhysics.Definitions;
using Genbox.VelcroPhysics.Dynamics;
using Genbox.VelcroPhysics.Factories;
using Microsoft.Xna.Framework;
using Xunit;

namespace Genbox.VelcroPhysics.Tests.Tests
{
    public class SensorContactTest
    {
        private struct Event
        {
            public string Kind;
            public Fixture A;
            public Fixture B;
        }

        private static Fixture CreateSensor(World world, Vector2 position, float radius)
        {
            CircleShape shape = new CircleShape(0.0f);
            shape.Radius = radius;
            shape.Position = position;

            FixtureDef fd = new FixtureDef();
            fd.Shape = shape;
            fd.IsSensor = true;
            return BodyFactory.CreateBody(world).AddFixture(fd);
        }

        private static Body CreateDynamicCircle(World world, Vector2 position, float radius = 0.5f)
        {
            Body body = BodyFactory.CreateBody(world, position, 0, BodyType.Dynamic);
            CircleShape shape = new CircleShape(radius, 1.0f);
            body.AddFixture(shape);
            return body;
        }

        private static int CountEdges(Body body)
        {
            int count = 0;
            for (ContactEdge edge = body.ContactList; edge != null; edge = edge.Next)
                count++;
            return count;
        }

        private static (Fixture, Fixture) OrderedPair(Contact c, Fixture visitor)
        {
            return c.FixtureA == visitor ? (visitor, c.FixtureB) : (c.FixtureA, visitor);
        }

        [Fact]
        public void TwoOverlappingSensors_BothReportBeginAndEndForSamePair()
        {
            World world = new World(Vector2.Zero);

            Fixture sensor1 = CreateSensor(world, new Vector2(0.0f, 0.0f), 2.0f);
            Fixture sensor2 = CreateSensor(world, new Vector2(1.0f, 0.0f), 2.0f);

            Body body = CreateDynamicCircle(world, new Vector2(20.0f, 20.0f));
            Fixture visitor = body.FixtureList[0];

            List<Event> events = new List<Event>();
            world.ContactManager.BeginContact += c => events.Add(new Event { Kind = "begin", A = c.FixtureA, B = c.FixtureB });
            world.ContactManager.EndContact += c => events.Add(new Event { Kind = "end", A = c.FixtureA, B = c.FixtureB });

            // Move the visitor into the overlap region of both sensors.
            body.Position = new Vector2(0.5f, 0.0f);
            world.Step(0.0f);

            Assert.Equal(2, events.Count);
            Assert.All(events, e => Assert.Equal("begin", e.Kind));
            Assert.Contains(events, e => e.A == sensor1 && e.B == visitor || e.A == visitor && e.B == sensor1);
            Assert.Contains(events, e => e.A == sensor2 && e.B == visitor || e.A == visitor && e.B == sensor2);

            events.Clear();

            // Move completely out of both sensors.
            body.Position = new Vector2(20.0f, 20.0f);
            world.Step(0.0f);

            Assert.Equal(2, events.Count);
            Assert.All(events, e => Assert.Equal("end", e.Kind));
            Assert.Contains(events, e => e.A == sensor1 && e.B == visitor || e.A == visitor && e.B == sensor1);
            Assert.Contains(events, e => e.A == sensor2 && e.B == visitor || e.A == visitor && e.B == sensor2);

            // After exit no stale contacts must remain queryable on the body.
            Assert.Equal(0, CountEdges(body));
            Assert.Equal(0, world.ContactManager.ContactCount);
        }

        [Fact]
        public void ReEntryRaisesOneNewBeginPerSensor()
        {
            World world = new World(Vector2.Zero);

            Fixture sensor1 = CreateSensor(world, new Vector2(0.0f, 0.0f), 2.0f);
            Fixture sensor2 = CreateSensor(world, new Vector2(1.0f, 0.0f), 2.0f);

            Body body = CreateDynamicCircle(world, new Vector2(20.0f, 20.0f));
            Fixture visitor = body.FixtureList[0];

            int beginCount = 0;
            int endCount = 0;
            world.ContactManager.BeginContact += c =>
            {
                Assert.True(c.FixtureA == visitor || c.FixtureB == visitor);
                Assert.True(c.FixtureA == sensor1 || c.FixtureB == sensor1 || c.FixtureA == sensor2 || c.FixtureB == sensor2);
                beginCount++;
            };
            world.ContactManager.EndContact += c => endCount++;

            body.Position = new Vector2(0.5f, 0.0f);
            world.Step(0.0f);
            Assert.Equal(2, beginCount);
            Assert.Equal(0, endCount);

            body.Position = new Vector2(20.0f, 20.0f);
            world.Step(0.0f);
            Assert.Equal(2, endCount);
            Assert.Equal(0, CountEdges(body));

            // Re-entering must produce a fresh begin for each sensor.
            body.Position = new Vector2(0.5f, 0.0f);
            world.Step(0.0f);
            Assert.Equal(4, beginCount);
            Assert.Equal(2, endCount);
            Assert.Equal(2, CountEdges(body));
        }

        [Fact]
        public void SensorDoesNotAffectDynamics()
        {
            World world = new World(Vector2.Zero);

            // A solid wall the dynamic body is allowed to pass through because it is a sensor.
            CreateSensor(world, new Vector2(0.0f, 0.0f), 5.0f);

            Body body = CreateDynamicCircle(world, new Vector2(0.0f, 0.0f));
            body.LinearVelocity = new Vector2(10.0f, 0.0f);

            Vector2 start = body.Position;
            for (int i = 0; i < 30; i++)
                world.Step(1.0f / 60.0f);

            // No collision response: the body should have travelled the full distance unobstructed.
            Assert.True(body.Position.X > start.X + 4.0f, $"Sensor affected dynamics, moved only {body.Position.X - start.X}");
        }

        [Fact]
        public void BeginAndEndEventsAlwaysReferenceTheSameObjectPair()
        {
            World world = new World(Vector2.Zero);

            Fixture sensor1 = CreateSensor(world, new Vector2(0.0f, 0.0f), 2.0f);
            Fixture sensor2 = CreateSensor(world, new Vector2(1.0f, 0.0f), 2.0f);

            Body body = CreateDynamicCircle(world, new Vector2(20.0f, 20.0f));
            Fixture visitor = body.FixtureList[0];

            Dictionary<(Fixture, Fixture), int> begins = new Dictionary<(Fixture, Fixture), int>();
            Dictionary<(Fixture, Fixture), int> ends = new Dictionary<(Fixture, Fixture), int>();

            world.ContactManager.BeginContact += c =>
            {
                (Fixture, Fixture) key = OrderedPair(c, visitor);
                begins.TryGetValue(key, out int n);
                begins[key] = n + 1;
            };
            world.ContactManager.EndContact += c =>
            {
                (Fixture, Fixture) key = OrderedPair(c, visitor);
                ends.TryGetValue(key, out int n);
                ends[key] = n + 1;
            };

            for (int cycle = 0; cycle < 3; cycle++)
            {
                body.Position = new Vector2(0.5f, 0.0f);
                world.Step(0.0f);
                body.Position = new Vector2(20.0f, 20.0f);
                world.Step(0.0f);
            }

            Assert.Equal(2, begins.Count);
            Assert.Equal(2, ends.Count);
            Assert.Equal(3, begins[(sensor1, visitor)]);
            Assert.Equal(3, begins[(sensor2, visitor)]);
            Assert.Equal(3, ends[(sensor1, visitor)]);
            Assert.Equal(3, ends[(sensor2, visitor)]);
        }

        [Fact]
        public void DisablingContactInBegin_FiresSingleEndAndLeavesNoStaleTouchingContact()
        {
            World world = new World(Vector2.Zero);

            Fixture sensor = CreateSensor(world, new Vector2(0.0f, 0.0f), 2.0f);
            Body body = CreateDynamicCircle(world, new Vector2(20.0f, 20.0f));

            int beginCount = 0;
            int endCount = 0;
            world.ContactManager.BeginContact += c =>
            {
                beginCount++;
                c.Enabled = false;
            };
            world.ContactManager.EndContact += c => endCount++;

            body.Position = new Vector2(0.0f, 0.0f);
            world.Step(0.0f);
            Assert.Equal(1, beginCount);

            // While the shapes still overlap, a disabled contact must not raise a second begin.
            for (int i = 0; i < 3; i++)
                world.Step(0.0f);
            Assert.Equal(1, beginCount);
            Assert.Equal(0, endCount);

            // Leaving the sensor must produce exactly one end event, never a duplicate from
            // both Contact.Update and ContactManager.Remove.
            body.Position = new Vector2(20.0f, 20.0f);
            world.Step(0.0f);

            Assert.Equal(1, endCount);
            Assert.Equal(0, world.ContactManager.ContactCount);
            Assert.Equal(0, CountEdges(body));

            // A later re-entry is a brand new begin for the same pair.
            body.Position = new Vector2(0.0f, 0.0f);
            world.Step(0.0f);
            Assert.Equal(2, beginCount);
            Assert.Equal(1, endCount);
        }

        [Fact]
        public void AsleepBodyLeavingSensor_FiresEndAndDropsStaleContact()
        {
            World world = new World(Vector2.Zero);

            Fixture sensor = CreateSensor(world, new Vector2(0.0f, 0.0f), 2.0f);
            Body body = CreateDynamicCircle(world, new Vector2(20.0f, 20.0f));

            int beginCount = 0;
            int endCount = 0;
            world.ContactManager.BeginContact += c => beginCount++;
            world.ContactManager.EndContact += c => endCount++;

            body.Position = new Vector2(0.0f, 0.0f);
            world.Step(0.0f);
            Assert.Equal(1, beginCount);
            Assert.Equal(1, CountEdges(body));

            // Put the body to sleep while it overlaps the static sensor, then move it out.
            body.Awake = false;
            body.Position = new Vector2(20.0f, 20.0f);
            world.Step(0.0f);

            // The end event must still settle and no stale contact may remain queryable.
            Assert.Equal(1, endCount);
            Assert.Equal(0, CountEdges(body));
            Assert.Equal(0, world.ContactManager.ContactCount);

            // Re-entering raises a fresh begin for the same object pair.
            body.Awake = false;
            body.Position = new Vector2(0.0f, 0.0f);
            world.Step(0.0f);
            Assert.Equal(2, beginCount);
            Assert.Equal(1, endCount);
            Assert.Equal(1, CountEdges(body));
        }

        [Fact]
        public void SensorRaisesUpdateEventsWhileTouchingForTheSamePair()
        {
            World world = new World(Vector2.Zero);

            Fixture sensor1 = CreateSensor(world, new Vector2(-1.0f, 0.0f), 5.0f);
            Fixture sensor2 = CreateSensor(world, new Vector2(1.0f, 0.0f), 5.0f);
            Body body = CreateDynamicCircle(world, new Vector2(20.0f, 20.0f));
            Fixture visitor = body.FixtureList[0];

            Dictionary<Fixture, int> updatesPerSensor = new Dictionary<Fixture, int>
            {
                { sensor1, 0 },
                { sensor2, 0 }
            };

            world.ContactManager.PreSolve += PreSolveHandler;
            void PreSolveHandler(Contact c, ref Manifold oldManifold)
            {
                // Every update event must settle against the same object pair as begin/end.
                Assert.True(c.FixtureA == visitor || c.FixtureB == visitor);

                if (c.FixtureA == sensor1 || c.FixtureB == sensor1)
                    updatesPerSensor[sensor1]++;
                else if (c.FixtureA == sensor2 || c.FixtureB == sensor2)
                    updatesPerSensor[sensor2]++;
                else
                    Assert.True(false, "Update event for an unknown object pair.");
            }

            body.Position = new Vector2(0.0f, 0.0f);

            // While the body stays inside both overlapping sensors, every step reports an
            // update for each sensor, so no callback is silently dropped on entry.
            for (int i = 0; i < 5; i++)
                world.Step(0.0f);

            Assert.True(updatesPerSensor[sensor1] >= 5, $"sensor1 updates={updatesPerSensor[sensor1]}");
            Assert.True(updatesPerSensor[sensor2] >= 5, $"sensor2 updates={updatesPerSensor[sensor2]}");

            // After leaving, the update stream stops together with the contacts.
            body.Position = new Vector2(20.0f, 20.0f);
            int before = updatesPerSensor[sensor1] + updatesPerSensor[sensor2];
            for (int i = 0; i < 3; i++)
                world.Step(0.0f);
            Assert.Equal(before, updatesPerSensor[sensor1] + updatesPerSensor[sensor2]);
            Assert.Equal(0, world.ContactManager.ContactCount);
        }
    }
}
