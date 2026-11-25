using System.Collections.Generic;
using Genbox.VelcroPhysics.Dynamics;
using Genbox.VelcroPhysics.Factories;
using Microsoft.Xna.Framework;
using Xunit;

namespace Genbox.VelcroPhysics.Tests.Tests
{
    /// <summary>
    /// Scenario from the thin-sensor/thick-wall sample: a bullet fast enough to tunnel in one
    /// discrete step crosses a thin sensor and then hits a thick solid wall.
    /// </summary>
    public class ContinuousCollisionTest
    {
        private const float BulletRadius = 0.1f;
        private const float BulletSpeed = 40.0f;
        private const float TimeStep = 1.0f / 60.0f;

        // The bullet covers 0.667 m per step. Both the thin sensor and the wall face are placed
        // inside that interval so a discrete-only engine would miss the time ordering.
        private const float SensorCenterX = 0.25f;
        private const float SensorThickness = 0.02f;
        private const float WallFaceX = 0.55f;
        private const float WallThickness = 0.4f;

        private static World CreateSensorThenWallWorld(out Fixture sensor, out Fixture wall)
        {
            // No gravity: the event order must come purely from the time of impact.
            World world = new World(Vector2.Zero);

            Body sensorBody = BodyFactory.CreateRectangle(world, SensorThickness, 2.0f, 0f, new Vector2(SensorCenterX, 0f));
            sensor = sensorBody.FixtureList[0];
            sensor.IsSensor = true;

            Body wallBody = BodyFactory.CreateRectangle(world, WallThickness, 2.0f, 1f,
                new Vector2(WallFaceX + WallThickness / 2f, 0f));
            wall = wallBody.FixtureList[0];

            return world;
        }

        private static Body CreateBullet(World world)
        {
            Body bullet = BodyFactory.CreateCircle(world, BulletRadius, 1f, new Vector2(0f, 0f), BodyType.Dynamic);
            bullet.IsBullet = true;
            bullet.LinearVelocity = new Vector2(BulletSpeed, 0f);
            return bullet;
        }

        private static string Label(Fixture fA, Fixture fB, Fixture sensor, Fixture wall)
        {
            if (fA == sensor || fB == sensor)
                return "sensor";

            if (fA == wall || fB == wall)
                return "wall";

            return "other";
        }

        [Fact]
        public void SensorBeginFiresBeforeSolidCollisionAndInTimeOrder()
        {
            World world = CreateSensorThenWallWorld(out Fixture sensor, out Fixture wall);
            Body bullet = CreateBullet(world);

            List<string> events = new List<string>();

            world.ContactManager.BeginContact += contact =>
            {
                events.Add("begin:" + Label(contact.FixtureA, contact.FixtureB, sensor, wall));
            };
            world.ContactManager.EndContact += contact =>
            {
                events.Add("end:" + Label(contact.FixtureA, contact.FixtureB, sensor, wall));
            };

            world.Step(TimeStep);

            // The first begin event of the step must be the thin sensor, not the thick wall.
            Assert.Contains("begin:sensor", events);
            Assert.Equal("begin:sensor", events[0]);

            // The solid wall contact is created as well during the step and comes later.
            Assert.Contains("begin:wall", events);
            Assert.True(events.IndexOf("begin:sensor") < events.IndexOf("begin:wall"));

            // The bullet must not tunnel through the wall.
            Assert.True(bullet.Position.X <= WallFaceX - BulletRadius + 0.05f);
        }

        [Fact]
        public void NoDuplicateBeginEventsAcrossSteps()
        {
            World world = CreateSensorThenWallWorld(out Fixture sensor, out Fixture wall);
            Body bullet = CreateBullet(world);

            List<string> events = new List<string>();
            world.ContactManager.BeginContact += contact =>
            {
                events.Add("begin:" + Label(contact.FixtureA, contact.FixtureB, sensor, wall));
            };

            // Run several steps while the bullet flies in and rests against the wall.
            for (int i = 0; i < 10; i++)
                world.Step(TimeStep);

            // Each fixture pair produces exactly one begin event for the whole contact lifetime.
            Assert.Equal(1, events.FindAll(e => e == "begin:sensor").Count);
            Assert.Equal(1, events.FindAll(e => e == "begin:wall").Count);
        }

        [Fact]
        public void SensorAppliesNoImpulse()
        {
            World world = CreateSensorThenWallWorld(out Fixture sensor, out Fixture wall);
            Body bullet = CreateBullet(world);

            // Same scene, but the wall is removed so only the thin sensor is in the path.
            wall.Body.RemoveFromWorld();

            world.Step(TimeStep);

            // A sensor must not change the bullet's velocity at all.
            Assert.Equal(BulletSpeed, bullet.LinearVelocity.X, 5);
            Assert.Equal(0f, bullet.LinearVelocity.Y, 5);

            // Free motion: position advanced by exactly v * dt.
            Assert.Equal(BulletSpeed * TimeStep, bullet.Position.X, 4);
        }

        [Fact]
        public void SolidContactNotPositionedByHistoricalCachedToi()
        {
            World world = CreateSensorThenWallWorld(out Fixture sensor, out Fixture wall);
            Body bullet = CreateBullet(world);

            List<Vector2> wallBeginPositions = new List<Vector2>();
            world.ContactManager.BeginContact += contact =>
            {
                if (contact.FixtureA == wall || contact.FixtureB == wall)
                    wallBeginPositions.Add(bullet.Position);
            };

            // Run two steps. Any stale TOI carried over from the first step (or from a pooled
            // contact's previous life) would park the wall contact away from the wall face.
            world.Step(TimeStep);
            world.Step(TimeStep);

            Assert.NotEmpty(wallBeginPositions);

            // The wall's left face is at WallFaceX. The bullet center must never be reported
            // past the face, and should come to rest against it.
            foreach (Vector2 position in wallBeginPositions)
            {
                Assert.True(position.X <= WallFaceX + 0.05f,
                    $"Wall contact reported at {position.X}, past the wall face {WallFaceX}; a historical TOI moved the body.");
            }

            Assert.True(bullet.Position.X >= WallFaceX - BulletRadius - 0.1f);
            Assert.True(bullet.Position.X <= WallFaceX - BulletRadius + 0.1f);
        }

        [Fact]
        public void SensorBeginAndEndBothFireWhenBulletCrossesThinSensor()
        {
            World world = CreateSensorThenWallWorld(out Fixture sensor, out Fixture wall);
            Body bullet = CreateBullet(world);

            // Remove the wall so the bullet can fully cross the thin sensor.
            wall.Body.RemoveFromWorld();

            List<string> events = new List<string>();
            world.ContactManager.BeginContact += c =>
            {
                if (c.FixtureA == sensor || c.FixtureB == sensor)
                    events.Add("begin");
            };
            world.ContactManager.EndContact += c =>
            {
                if (c.FixtureA == sensor || c.FixtureB == sensor)
                    events.Add("end");
            };

            // One step is enough to enter and leave the 0.02 m thick sensor at 40 m/s.
            world.Step(TimeStep);

            Assert.Equal(new[] { "begin", "end" }, events);
        }

        [Fact]
        public void EventsStillFireWithContinuousPhysicsDisabled()
        {
            // A dedicated slow-moving scene: without continuous physics there is no TOI sweep, so a
            // tunneling body cannot be tracked. Use a speed low enough that the bullet's discrete
            // end-of-step position is still inside the sensor; this guards that turning CCD off does
            // not silence ordinary sensor begin/end callbacks.
            World world = new World(Vector2.Zero);
            Body sensorBody = BodyFactory.CreateRectangle(world, 0.5f, 2.0f, 0f, new Vector2(0.15f, 0f));
            Fixture sensor = sensorBody.FixtureList[0];
            sensor.IsSensor = true;

            Body bullet = BodyFactory.CreateCircle(world, BulletRadius, 1f, new Vector2(0f, 0f), BodyType.Dynamic);
            bullet.LinearVelocity = new Vector2(6f, 0f);
            world.ContinuousPhysicsEnabled = false;

            List<string> events = new List<string>();
            world.ContactManager.BeginContact += contact =>
            {
                if (contact.FixtureA == sensor || contact.FixtureB == sensor)
                    events.Add("begin:sensor");
            };

            // A contact created during a step is reported by the narrow phase on the following step
            // (standard Box2D ordering), so run two steps.
            world.Step(TimeStep);
            world.Step(TimeStep);

            Assert.Contains("begin:sensor", events);
        }
    }
}
