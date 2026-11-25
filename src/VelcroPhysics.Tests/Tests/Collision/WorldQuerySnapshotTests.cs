using System.Collections.Generic;
using Genbox.VelcroPhysics.Collision.Broadphase;
using Genbox.VelcroPhysics.Dynamics;
using Genbox.VelcroPhysics.Factories;
using Genbox.VelcroPhysics.Shared;
using Microsoft.Xna.Framework;
using Xunit;

namespace Genbox.VelcroPhysics.Tests.Tests.Collision
{
    /// <summary>
    /// End-to-end regression test for stale ray-cast / area query hits after rigid bodies teleport or are removed from a
    /// world. It uses 40 static bodies and a single batch update: 20 bodies teleport away and 20 are removed. The old query
    /// snapshot keeps reporting the world as it was, whereas the new world queries contain no residue.
    /// </summary>
    public class WorldQuerySnapshotTests
    {
        private const int BodyCount = 40;
        private const int RowSize = 20;
        private const float Spacing = 2.0f;

        private static World CreateWorldWithStaticBodies(out Body[] bodies)
        {
            World world = new World(Vector2.Zero);
            bodies = new Body[BodyCount];

            for (int i = 0; i < BodyCount; ++i)
            {
                int column = i % RowSize;
                int row = i / RowSize;
                bodies[i] = BodyFactory.CreateRectangle(
                    world,
                    1.0f,
                    1.0f,
                    1.0f,
                    new Vector2(column * Spacing + 0.5f, row * Spacing + 0.5f),
                    0.0f,
                    BodyType.Static);
            }

            // Let the broad-phase settle the proxies added by the factory.
            world.Step(0.0f);
            return world;
        }

        private static int CountWorldQuery(World world, ref AABB region)
        {
            return world.QueryAABB(ref region).Count;
        }

        private static int CountSnapshotQuery(BroadPhaseSnapshot snapshot, ref AABB region)
        {
            int count = 0;
            snapshot.Query(_ =>
            {
                ++count;
                return true;
            }, ref region);
            return count;
        }

        [Fact]
        public void TeleportAndRemoveInOneBatchUpdateLeavesNoResidueInNewWorld()
        {
            World world = CreateWorldWithStaticBodies(out Body[] bodies);

            AABB oldRegion = new AABB(new Vector2(-0.5f, -0.5f), new Vector2(40.0f, 3.5f));
            AABB row0Region = new AABB(new Vector2(-0.5f, -0.5f), new Vector2(40.0f, 1.5f));
            AABB row1Region = new AABB(new Vector2(-0.5f, 1.5f), new Vector2(40.0f, 3.5f));

            // Polygon shapes carry a small skin radius, so the teleported proxies span roughly y = 99.49..100.51.
            AABB teleportTargetRegion = new AABB(new Vector2(-0.5f, 99.0f), new Vector2(40.0f, 101.0f));

            Assert.Equal(BodyCount, CountWorldQuery(world, ref oldRegion));

            // Snapshot taken before the batch update.
            BroadPhaseSnapshot oldSnapshot = world.CreateQuerySnapshot();
            Assert.Equal(BodyCount, oldSnapshot.Count);

            // One batch update: row 0 teleports to y = 100, row 1 is removed.
            for (int i = 0; i < RowSize; ++i)
                bodies[i].SetTransform(new Vector2(bodies[i].Position.X, 100.0f), 0.0f);

            for (int i = RowSize; i < BodyCount; ++i)
                world.RemoveBody(bodies[i]);

            world.Step(0.0f);

            // New world: nothing remains at the old locations...
            Assert.Equal(0, CountWorldQuery(world, ref oldRegion));
            Assert.Equal(0, CountWorldQuery(world, ref row0Region));
            Assert.Equal(0, CountWorldQuery(world, ref row1Region));

            // ...the teleported bodies are queryable at their new location...
            Assert.Equal(RowSize, CountWorldQuery(world, ref teleportTargetRegion));

            // ...and ray-casts through the old world report no stale fixtures.
            List<Fixture> row0Hits = world.RayCast(new Vector2(-1.0f, 0.5f), new Vector2(40.0f, 0.5f));
            List<Fixture> row1Hits = world.RayCast(new Vector2(-1.0f, 2.5f), new Vector2(40.0f, 2.5f));
            Assert.Empty(row0Hits);
            Assert.Empty(row1Hits);

            List<Fixture> newHits = world.RayCast(new Vector2(-1.0f, 100.5f), new Vector2(40.0f, 100.5f));
            Assert.Equal(RowSize, newHits.Count);

            // The fixture-local proxy AABB was written back during the teleport, so it matches its broad-phase location.
            foreach (Body body in bodies[..RowSize])
            {
                foreach (Fixture fixture in body.FixtureList)
                {
                    fixture.GetAABB(out AABB proxyAabb, 0);
                    Assert.True(teleportTargetRegion.Contains(ref proxyAabb));
                    Assert.True(proxyAabb.LowerBound.Y >= 99.4f);
                }
            }

            // Removed bodies have been detached from the world and their broad-phase proxies are gone.
            for (int i = RowSize; i < BodyCount; ++i)
                Assert.Null(bodies[i].FixtureList);
            Assert.Equal(RowSize, world.ContactManager.BroadPhase.ProxyCount);

            // The old snapshot is unchanged by the batch update: removed/teleported objects only exist there.
            Assert.Equal(BodyCount, oldSnapshot.Count);
            Assert.Equal(RowSize, CountSnapshotQuery(oldSnapshot, ref row0Region));
            Assert.Equal(RowSize, CountSnapshotQuery(oldSnapshot, ref row1Region));
            Assert.Equal(0, CountSnapshotQuery(oldSnapshot, ref teleportTargetRegion));
        }

        [Fact]
        public void DisabledBodyWritesBackNullProxyIdsAndQueriesFindNoResidue()
        {
            World world = CreateWorldWithStaticBodies(out Body[] bodies);

            AABB oldRegion = new AABB(new Vector2(-0.5f, -0.5f), new Vector2(40.0f, 3.5f));
            Assert.Equal(BodyCount, CountWorldQuery(world, ref oldRegion));

            // Disabling a body destroys its broad-phase proxies but keeps the fixtures (and their proxy array) alive.
            Body disabled = bodies[0];
            disabled.Enabled = false;
            world.Step(0.0f);

            foreach (Fixture fixture in disabled.FixtureList)
            {
                Assert.Equal(0, fixture.ProxyCount);

                // The destroyed id must have been written back to the fixture-local proxy struct.
                Assert.Equal(DynamicTreeBroadPhase.NullProxy, fixture.Proxies[0].ProxyId);
            }

            // The broad-phase no longer contains the disabled body's proxy. The ray stops before the next body's
            // skinned AABB begins at x = 1.99.
            Assert.Equal(BodyCount - 1, world.ContactManager.BroadPhase.ProxyCount);
            AABB disabledBodyRegion = new AABB(new Vector2(-0.5f, -0.5f), new Vector2(1.5f, 1.5f));
            Assert.Equal(0, CountWorldQuery(world, ref disabledBodyRegion));
            Assert.Empty(world.RayCast(new Vector2(-1.0f, 0.5f), new Vector2(1.5f, 0.5f)));

            // Re-enabling the body creates fresh, non-null proxies that are queryable again.
            disabled.Enabled = true;
            world.Step(0.0f);

            Assert.Equal(BodyCount, world.ContactManager.BroadPhase.ProxyCount);
            Assert.Equal(1, CountWorldQuery(world, ref disabledBodyRegion));
            Assert.Single(world.RayCast(new Vector2(-1.0f, 0.5f), new Vector2(1.5f, 0.5f)));
        }
    }
}
