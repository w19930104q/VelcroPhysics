using System.Collections.Generic;
using Genbox.VelcroPhysics.Collision.Broadphase;
using Genbox.VelcroPhysics.Collision.RayCast;
using Genbox.VelcroPhysics.Dynamics;
using Genbox.VelcroPhysics.Shared;
using Microsoft.Xna.Framework;
using Xunit;

namespace Genbox.VelcroPhysics.Tests.Tests.Collision
{
    /// <summary>
    /// Regression tests for stale broad-phase data after rigid bodies teleport or get deleted. After a batch update the
    /// proxy AABB, the broad-phase proxy and any query snapshot must all agree: an old snapshot keeps seeing the old world
    /// (objects being removed only exist there), while ray-casts and area queries on the new world never return residue.
    /// The sample uses 40 static proxies and a single batch update.
    /// </summary>
    public class BroadphaseSnapshotTests
    {
        private const int ProxyCount = 40;
        private const float ProxySize = 1.0f;
        private const float GridSpacing = 2.0f;

        /// <summary>40 static proxies arranged in a 20 x 2 grid. Row 0 sits at y = 0..1, row 1 at y = 2..3.</summary>
        private static DynamicTreeBroadPhase CreateStaticBroadPhase(out int[] proxyIds, out FixtureProxy[] proxies)
        {
            DynamicTreeBroadPhase broadPhase = new DynamicTreeBroadPhase();
            proxyIds = new int[ProxyCount];
            proxies = new FixtureProxy[ProxyCount];

            for (int i = 0; i < ProxyCount; ++i)
            {
                int column = i % 20;
                int row = i / 20;

                FixtureProxy proxy = new FixtureProxy
                {
                    Fixture = null,
                    ChildIndex = 0,
                    ProxyId = DynamicTreeBroadPhase.NullProxy,
                    AABB = new AABB(
                        new Vector2(column * GridSpacing, row * GridSpacing),
                        new Vector2(column * GridSpacing + ProxySize, row * GridSpacing + ProxySize))
                };

                proxy.ProxyId = broadPhase.AddProxy(ref proxy);
                proxyIds[i] = proxy.ProxyId;
                proxies[i] = proxy;
            }

            // Pairs only become final after UpdatePairs, which also clears the move buffer produced by AddProxy.
            broadPhase.UpdatePairs((ref FixtureProxy a, ref FixtureProxy b) => { });

            Assert.Equal(ProxyCount, broadPhase.ProxyCount);
            return broadPhase;
        }

        private static AABB RegionAABB(int firstColumn, int lastColumn, float yMin, float yMax)
        {
            return new AABB(
                new Vector2(firstColumn * GridSpacing - 0.5f, yMin),
                new Vector2(lastColumn * GridSpacing + ProxySize + 0.5f, yMax));
        }

        private static int CountQuery(IBroadPhase broadPhase, ref AABB region)
        {
            int count = 0;
            broadPhase.Query(_ =>
            {
                ++count;
                return true;
            }, ref region);
            return count;
        }

        private static int CountQuery(BroadPhaseSnapshot snapshot, ref AABB region)
        {
            int count = 0;
            snapshot.Query(_ =>
            {
                ++count;
                return true;
            }, ref region);
            return count;
        }

        private static int CountRayHits(IBroadPhase broadPhase, Vector2 point1, Vector2 point2)
        {
            RayCastInput input = new RayCastInput
            {
                Point1 = point1,
                Point2 = point2,
                MaxFraction = 1.0f
            };

            int count = 0;
            broadPhase.RayCast((_, _) =>
            {
                ++count;
                return input.MaxFraction;
            }, ref input);
            return count;
        }

        private static int CountRayHits(BroadPhaseSnapshot snapshot, Vector2 point1, Vector2 point2)
        {
            RayCastInput input = new RayCastInput
            {
                Point1 = point1,
                Point2 = point2,
                MaxFraction = 1.0f
            };

            int count = 0;
            snapshot.RayCast((_, _) =>
            {
                ++count;
                return input.MaxFraction;
            }, ref input);
            return count;
        }

        [Fact]
        public void SnapshotIsFrozenWhenProxiesTeleportOrGetDeleted()
        {
            DynamicTreeBroadPhase broadPhase = CreateStaticBroadPhase(out int[] proxyIds, out FixtureProxy[] proxies);

            BroadPhaseSnapshot oldSnapshot = broadPhase.CreateSnapshot();
            Assert.Equal(ProxyCount, oldSnapshot.Count);

            // One batch update:
            //  - the 20 proxies of row 0 teleport far away (to y = 100),
            //  - the 20 proxies of row 1 are deleted.
            const int moved = 20;
            const int removed = 20;

            for (int i = 0; i < moved; ++i)
            {
                FixtureProxy proxy = proxies[i];
                Vector2 lower = new Vector2(proxy.AABB.LowerBound.X, 100.0f);
                proxy.AABB = new AABB(lower, lower + new Vector2(ProxySize));
                broadPhase.MoveProxy(proxy.ProxyId, ref proxy, ref proxy.AABB, new Vector2(0.0f, 100.0f));
                proxies[i] = proxy;
            }

            for (int i = moved; i < ProxyCount; ++i)
                broadPhase.RemoveProxy(proxyIds[i]);

            broadPhase.UpdatePairs((ref FixtureProxy a, ref FixtureProxy b) => { });

            // The old snapshot is an immutable copy and still shows the world as it was.
            Assert.Equal(ProxyCount, oldSnapshot.Count);

            AABB oldRow0 = RegionAABB(0, 19, -0.5f, 1.5f);
            AABB oldRow1 = RegionAABB(0, 19, 1.5f, 3.5f);
            Assert.Equal(moved, CountQuery(oldSnapshot, ref oldRow0));
            Assert.Equal(removed, CountQuery(oldSnapshot, ref oldRow1));

            // A ray along the old row 0 hits all 20 proxies in the snapshot...
            Assert.Equal(moved, CountRayHits(oldSnapshot, new Vector2(-1.0f, 0.5f), new Vector2(40.0f, 0.5f)));

            // ...and a ray through the new location hits nothing in the snapshot, because the snapshot predates the move.
            Assert.Equal(0, CountRayHits(oldSnapshot, new Vector2(-1.0f, 100.5f), new Vector2(40.0f, 100.5f)));
        }

        [Fact]
        public void LiveQueriesContainNoResidueAfterBatchUpdate()
        {
            DynamicTreeBroadPhase broadPhase = CreateStaticBroadPhase(out int[] proxyIds, out FixtureProxy[] proxies);

            const int moved = 20;
            const int removed = 20;

            // Single batch update: row 0 teleports, row 1 is deleted.
            for (int i = 0; i < moved; ++i)
            {
                FixtureProxy proxy = proxies[i];
                Vector2 lower = new Vector2(proxy.AABB.LowerBound.X, 100.0f);
                proxy.AABB = new AABB(lower, lower + new Vector2(ProxySize));
                broadPhase.MoveProxy(proxy.ProxyId, ref proxy, ref proxy.AABB, new Vector2(0.0f, 100.0f));
                proxies[i] = proxy;
            }

            for (int i = moved; i < ProxyCount; ++i)
                broadPhase.RemoveProxy(proxyIds[i]);

            broadPhase.UpdatePairs((ref FixtureProxy a, ref FixtureProxy b) => { });

            // Only the 20 teleported proxies remain in the new world.
            Assert.Equal(moved, broadPhase.ProxyCount);

            AABB oldRow0 = RegionAABB(0, 19, -0.5f, 1.5f);
            AABB oldRow1 = RegionAABB(0, 19, 1.5f, 3.5f);
            AABB newRegion = RegionAABB(0, 19, 99.5f, 101.5f);
            AABB wholeOldWorld = RegionAABB(0, 19, -0.5f, 3.5f);

            // No residue: neither the teleported-away proxies nor the deleted ones are found at the old locations.
            Assert.Equal(0, CountQuery(broadPhase, ref oldRow0));
            Assert.Equal(0, CountQuery(broadPhase, ref oldRow1));
            Assert.Equal(0, CountQuery(broadPhase, ref wholeOldWorld));

            // The teleported proxies are all reachable at their new AABB.
            Assert.Equal(moved, CountQuery(broadPhase, ref newRegion));

            // Ray-casts must not hit stale proxies either.
            Assert.Equal(0, CountRayHits(broadPhase, new Vector2(-1.0f, 0.5f), new Vector2(40.0f, 0.5f)));
            Assert.Equal(0, CountRayHits(broadPhase, new Vector2(-1.0f, 2.5f), new Vector2(40.0f, 2.5f)));
            Assert.Equal(moved, CountRayHits(broadPhase, new Vector2(-1.0f, 100.5f), new Vector2(40.0f, 100.5f)));

            // A fresh snapshot of the new world matches the live broad-phase and contains nothing from the old world.
            BroadPhaseSnapshot newSnapshot = broadPhase.CreateSnapshot();
            Assert.Equal(moved, newSnapshot.Count);
            Assert.Equal(0, CountQuery(newSnapshot, ref wholeOldWorld));
            Assert.Equal(moved, CountQuery(newSnapshot, ref newRegion));
        }

        [Fact]
        public void ProxyAABBAndBroadphaseAABBUpdateAtomicallyOnTeleport()
        {
            DynamicTreeBroadPhase broadPhase = CreateStaticBroadPhase(out _, out FixtureProxy[] proxies);

            // Batch-teleport every proxy far to the right in one update.
            for (int i = 0; i < ProxyCount; ++i)
            {
                FixtureProxy proxy = proxies[i];
                Vector2 lower = new Vector2(1000.0f + i * GridSpacing, proxy.AABB.LowerBound.Y);
                proxy.AABB = new AABB(lower, lower + new Vector2(ProxySize));
                broadPhase.MoveProxy(proxy.ProxyId, ref proxy, ref proxy.AABB, new Vector2(1000.0f, 0.0f));
                proxies[i] = proxy;
            }

            broadPhase.UpdatePairs((ref FixtureProxy a, ref FixtureProxy b) => { });

            AABB oldWorld = RegionAABB(0, 19, -0.5f, 3.5f);
            AABB newWorld = new AABB(new Vector2(999.5f, -0.5f), new Vector2(1000.0f + ProxyCount * GridSpacing, 3.5f));

            // The fat AABB stored in the broad-phase no longer overlaps the old world.
            Assert.Equal(0, CountQuery(broadPhase, ref oldWorld));
            Assert.Equal(ProxyCount, CountQuery(broadPhase, ref newWorld));

            // The user data served by the broad-phase carries the new (non-fat) AABB as well.
            broadPhase.Query(proxyId =>
            {
                FixtureProxy current = broadPhase.GetProxy(proxyId);
                Assert.True(current.AABB.LowerBound.X >= 1000.0f);
                Assert.True(newWorld.Contains(ref current.AABB));
                return true;
            }, ref newWorld);

            // A snapshot taken after the batch update captured the new AABBs only.
            BroadPhaseSnapshot snapshot = broadPhase.CreateSnapshot();
            Assert.Equal(0, CountQuery(snapshot, ref oldWorld));
            Assert.Equal(ProxyCount, CountQuery(snapshot, ref newWorld));
            for (int i = 0; i < snapshot.Count; ++i)
                Assert.True(snapshot.GetProxy(i).AABB.LowerBound.X >= 1000.0f);
        }

        [Fact]
        public void DeletedProxyIdsDoNotLeakIntoSnapshotsAfterRecycling()
        {
            DynamicTreeBroadPhase broadPhase = CreateStaticBroadPhase(out int[] proxyIds, out _);

            // Delete half of the proxies.
            for (int i = 0; i < 20; ++i)
                broadPhase.RemoveProxy(proxyIds[i]);

            broadPhase.UpdatePairs((ref FixtureProxy a, ref FixtureProxy b) => { });
            Assert.Equal(20, broadPhase.ProxyCount);

            // Re-allocate so the pooled node slots (and possibly their ids) get recycled with new data.
            List<int> recycledIds = new List<int>();
            for (int i = 0; i < 20; ++i)
            {
                FixtureProxy proxy = new FixtureProxy
                {
                    Fixture = null,
                    ChildIndex = 0,
                    ProxyId = DynamicTreeBroadPhase.NullProxy,
                    AABB = new AABB(new Vector2(500.0f + i, 500.0f), new Vector2(501.0f + i, 501.0f))
                };

                proxy.ProxyId = broadPhase.AddProxy(ref proxy);
                recycledIds.Add(proxy.ProxyId);
            }

            broadPhase.UpdatePairs((ref FixtureProxy a, ref FixtureProxy b) => { });

            BroadPhaseSnapshot snapshot = broadPhase.CreateSnapshot();

            // Exactly 40 live proxies are captured: the 20 untouched originals plus the 20 recycled-slot replacements.
            Assert.Equal(ProxyCount, snapshot.Count);
            Assert.Equal(ProxyCount, broadPhase.ProxyCount);

            // Every captured proxy must be one of the current live proxies - no destroyed proxy survives in the snapshot.
            AABB recycledRegion = new AABB(new Vector2(499.5f, 499.5f), new Vector2(522.0f, 501.5f));
            Assert.Equal(20, CountQuery(snapshot, ref recycledRegion));
        }
    }
}
