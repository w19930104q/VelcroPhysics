using System;
using Genbox.VelcroPhysics.Collision.Handlers;
using Genbox.VelcroPhysics.Collision.RayCast;
using Genbox.VelcroPhysics.Dynamics;
using Genbox.VelcroPhysics.Shared;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Collision.Broadphase
{
    public interface IBroadPhase
    {
        int ProxyCount { get; }

        void UpdatePairs(BroadphaseHandler callback);

        bool TestOverlap(int proxyIdA, int proxyIdB);

        int AddProxy(ref FixtureProxy proxy);

        void RemoveProxy(int proxyId);

        void MoveProxy(int proxyId, ref FixtureProxy proxy, ref AABB aabb, Vector2 displacement);

        FixtureProxy GetProxy(int proxyId);

        void TouchProxy(int proxyId);

        void GetFatAABB(int proxyId, out AABB aabb);

        void Query(Func<int, bool> callback, ref AABB aabb);

        void RayCast(Func<RayCastInput, int, float> callback, ref RayCastInput input);

        /// <summary>
        /// Capture an immutable, value-copied snapshot of every proxy currently in the broad-phase. Later moves and
        /// removals do not affect the snapshot, and queries against the live broad-phase never return proxies that have been
        /// removed.
        /// </summary>
        BroadPhaseSnapshot CreateSnapshot();

        void ShiftOrigin(ref Vector2 newOrigin);
    }
}