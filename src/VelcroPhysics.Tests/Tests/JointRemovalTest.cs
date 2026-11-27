using Genbox.VelcroPhysics.Dynamics;
using Genbox.VelcroPhysics.Dynamics.Joints;
using Genbox.VelcroPhysics.Factories;
using Microsoft.Xna.Framework;
using Xunit;

namespace Genbox.VelcroPhysics.Tests.Tests
{
    public class JointRemovalTest
    {
        [Fact]
        public void RemoveJoint_WakesConnectedBodies()
        {
            World world = new World(new Vector2(0, -9.82f));

            Body body1 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(0.0f, 4.0f), 0.0f, BodyType.Dynamic);
            Body body2 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(0.0f, 2.0f), 0.0f, BodyType.Dynamic);
            Body body3 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(0.0f, 0.0f), 0.0f, BodyType.Dynamic);
            Body body4 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(2.0f, 4.0f), 0.0f, BodyType.Dynamic);

            DistanceJoint joint1 = JointFactory.CreateDistanceJoint(world, body1, body2, body1.Position, body2.Position);
            JointFactory.CreateDistanceJoint(world, body2, body3, body2.Position, body3.Position);

            // Let the simulation settle
            for (int i = 0; i < 60; i++)
                world.Step(1.0f / 60.0f);

            // Put all bodies to sleep
            body1.Awake = false;
            body2.Awake = false;
            body3.Awake = false;
            body4.Awake = false;

            // Step once — bodies should stay asleep (no forces)
            world.Step(1.0f / 60.0f);

            Assert.False(body1.Awake);
            Assert.False(body2.Awake);
            Assert.False(body3.Awake);
            Assert.False(body4.Awake);

            // Remove joint1 — this should wake body1 and body2
            world.RemoveJoint(joint1);

            Assert.True(body1.Awake, "Body1 should be awake after joint removal.");
            Assert.True(body2.Awake, "Body2 should be awake after joint removal.");
            Assert.False(body3.Awake, "Body3 should stay asleep — not connected to removed joint.");
            Assert.False(body4.Awake, "Body4 should stay asleep — not connected to any joint.");
        }

        [Fact]
        public void RemoveJoint_CleansUpJointList()
        {
            World world = new World(new Vector2(0, -9.82f));

            Body body1 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(0.0f, 4.0f), 0.0f, BodyType.Dynamic);
            Body body2 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(0.0f, 2.0f), 0.0f, BodyType.Dynamic);

            DistanceJoint joint = JointFactory.CreateDistanceJoint(world, body1, body2, body1.Position, body2.Position);

            Assert.Single(world.JointList);
            Assert.NotNull(body1.JointList);
            Assert.NotNull(body2.JointList);

            world.RemoveJoint(joint);

            // Joint removed from world list
            Assert.Empty(world.JointList);

            // Bodies no longer reference the joint through their joint lists
            Assert.Null(body1.JointList);
            Assert.Null(body2.JointList);
        }

        [Fact]
        public void RemoveJoint_DuringStep_DefersCorrectly()
        {
            World world = new World(new Vector2(0, -9.82f));

            Body body1 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(0.0f, 4.0f), 0.0f, BodyType.Dynamic);
            Body body2 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(0.0f, 2.0f), 0.0f, BodyType.Dynamic);

            DistanceJoint joint = JointFactory.CreateDistanceJoint(world, body1, body2, body1.Position, body2.Position);

            // Run one step to initialize
            world.Step(1.0f / 60.0f);

            // Register a controller that removes the joint during the step
            bool removedInCallback = false;
            world.AddController(new TestController(joint, world, () => removedInCallback = true));

            // Step — the controller runs while world is locked, RemoveJoint auto-defers
            world.Step(1.0f / 60.0f);

            // Joint still in list (deferred to next ProcessChanges)
            Assert.Single(world.JointList, "Joint still in list during the same step.");

            // Next step processes the deferred removal
            world.Step(1.0f / 60.0f);

            Assert.True(removedInCallback);
            Assert.Empty(world.JointList);
            Assert.True(body1.Awake, "Body1 should be awake after deferred joint removal.");
            Assert.True(body2.Awake, "Body2 should be awake after deferred joint removal.");
        }

        [Fact]
        public void RemoveJoint_PreservesBodyReferences()
        {
            World world = new World(new Vector2(0, -9.82f));

            Body body1 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(0.0f, 4.0f), 0.0f, BodyType.Dynamic);
            Body body2 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(0.0f, 2.0f), 0.0f, BodyType.Dynamic);

            DistanceJoint joint = JointFactory.CreateDistanceJoint(world, body1, body2, body1.Position, body2.Position);

            Assert.Equal(body1, joint.BodyA);
            Assert.Equal(body2, joint.BodyB);

            // Remove the joint
            world.RemoveJoint(joint);

            // Joint should still reference its bodies (user may still hold the reference)
            Assert.Equal(body1, joint.BodyA);
            Assert.Equal(body2, joint.BodyB);
        }

        [Fact]
        public void RemoveJoint_FromJointBreak_WakesBodiesAndClearsCache()
        {
            World world = new World(new Vector2(0, -9.82f));

            Body body1 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(0.0f, 4.0f), 0.0f, BodyType.Dynamic);
            Body body2 = BodyFactory.CreateRectangle(world, 1.0f, 1.0f, 1.0f, new Vector2(3.0f, 4.0f), 0.0f, BodyType.Dynamic);

            DistanceJoint joint = JointFactory.CreateDistanceJoint(world, body1, body2, body1.Position, body2.Position);
            joint.Breakpoint = 0.001f; // Very low — will break on any force

            // Apply opposing forces to stress the joint
            body1.ApplyForce(new Vector2(-100.0f, 0.0f), body1.Position);
            body2.ApplyForce(new Vector2(100.0f, 0.0f), body2.Position);

            // Run several steps — joint should break
            for (int i = 0; i < 10; i++)
                world.Step(1.0f / 60.0f);

            Assert.False(joint.Enabled, "Joint should be disabled after breaking.");

            // Bodies should be awake after joint breaks
            Assert.True(body1.Awake, "Body1 should be awake after joint break.");
            Assert.True(body2.Awake, "Body2 should be awake after joint break.");
        }

        private class TestController : Genbox.VelcroPhysics.Extensions.Controllers.ControllerBase.Controller
        {
            private Joint _joint;
            private World _world;
            private System.Action _onRemoved;

            public TestController(Joint joint, World world, System.Action onRemoved)
            {
                _joint = joint;
                _world = world;
                _onRemoved = onRemoved;
            }

            public override void Update(float dt)
            {
                _world.RemoveJoint(_joint);
                _onRemoved();
            }
        }
    }
}
