using System;
using System.Collections.Generic;
using System.Diagnostics;
using Genbox.VelcroPhysics.Collision.RayCast;
using Genbox.VelcroPhysics.Dynamics;
using Genbox.VelcroPhysics.Shared;
using Genbox.VelcroPhysics.Utilities;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Collision.Broadphase
{
    /// <summary>
    /// An immutable, value-copied snapshot of the proxies that live in a broad-phase at a single point in time.
    /// Because every entry is copied, teleports, removals and proxy id recycling in the live broad-phase can never mutate
    /// the snapshot. A query on the live broad-phase returns the new world without any residue, while a query on an old
    /// snapshot keeps returning the world as it was when the snapshot was taken.
    /// </summary>
    public sealed class BroadPhaseSnapshot
    {
        private readonly Entry[] _entries;

        internal BroadPhaseSnapshot(Entry[] entries)
        {
            _entries = entries;
        }

        /// <summary>The number of proxies captured by the snapshot.</summary>
        public int Count => _entries.Length;

        /// <summary>Get a captured proxy by snapshot-local index.</summary>
        /// <param name="index">An index in the range [0, <see cref="Count"/>).</param>
        public FixtureProxy GetProxy(int index)
        {
            return _entries[index].Proxy;
        }

        /// <summary>Get the fat AABB of a captured proxy by snapshot-local index.</summary>
        /// <param name="index">An index in the range [0, <see cref="Count"/>).</param>
        public AABB GetFatAABB(int index)
        {
            return _entries[index].FatAABB;
        }

        /// <summary>
        /// Query the snapshot for proxies that overlap the supplied AABB. The callback receives the snapshot-local
        /// index of every overlapping proxy. Return false from the callback to terminate the query.
        /// </summary>
        /// <param name="callback">The callback.</param>
        /// <param name="aabb">The query AABB.</param>
        public void Query(Func<int, bool> callback, ref AABB aabb)
        {
            for (int i = 0; i < _entries.Length; ++i)
            {
                if (AABB.TestOverlap(ref _entries[i].FatAABB, ref aabb) && !callback(i))
                    return;
            }
        }

        /// <summary>Query the snapshot for proxies that overlap the supplied AABB.</summary>
        /// <param name="aabb">The query AABB.</param>
        /// <returns>The snapshot-local indices of every overlapping proxy.</returns>
        public List<int> Query(ref AABB aabb)
        {
            List<int> result = new List<int>();
            Query((index =>
            {
                result.Add(index);
                return true;
            }), ref aabb);
            return result;
        }

        /// <summary>
        /// Ray-cast against the proxies captured by the snapshot. The callback performs the exact ray-cast and any
        /// collision filtering. It receives the ray-cast input and the snapshot-local index of the hit proxy. Return 0 to
        /// terminate the ray-cast, the hit fraction to clip the ray, or the input max fraction to continue.
        /// </summary>
        /// <param name="callback">A callback called for each proxy that is hit by the ray.</param>
        /// <param name="input">The ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).</param>
        public void RayCast(Func<RayCastInput, int, float> callback, ref RayCastInput input)
        {
            Vector2 p1 = input.Point1;
            Vector2 p2 = input.Point2;
            Vector2 r = p2 - p1;
            Debug.Assert(r.LengthSquared() > 0.0f);
            r.Normalize();

            // v is perpendicular to the segment.
            Vector2 absV = MathUtils.Abs(new Vector2(-r.Y, r.X));

            float maxFraction = input.MaxFraction;

            // Build a bounding box for the segment.
            AABB segmentAABB = new AABB();
            {
                Vector2 t = p1 + maxFraction * (p2 - p1);
                Vector2.Min(ref p1, ref t, out segmentAABB.LowerBound);
                Vector2.Max(ref p1, ref t, out segmentAABB.UpperBound);
            }

            for (int i = 0; i < _entries.Length; ++i)
            {
                AABB nodeAABB = _entries[i].FatAABB;

                if (!AABB.TestOverlap(ref nodeAABB, ref segmentAABB))
                    continue;

                // Separating axis for segment (Gino, p80).
                // |dot(v, p1 - c)| > dot(|v|, h)
                Vector2 c = nodeAABB.Center;
                Vector2 h = nodeAABB.Extents;
                float separation = Math.Abs(Vector2.Dot(new Vector2(-r.Y, r.X), p1 - c)) - Vector2.Dot(absV, h);
                if (separation > 0.0f)
                    continue;

                RayCastInput subInput;
                subInput.Point1 = input.Point1;
                subInput.Point2 = input.Point2;
                subInput.MaxFraction = maxFraction;

                float value = callback(subInput, i);

                if (value == 0.0f)
                {
                    // the client has terminated the raycast.
                    return;
                }

                if (value > 0.0f)
                {
                    // Update segment bounding box.
                    maxFraction = value;
                    Vector2 t = p1 + maxFraction * (p2 - p1);
                    segmentAABB.LowerBound = Vector2.Min(p1, t);
                    segmentAABB.UpperBound = Vector2.Max(p1, t);
                }
            }
        }

        /// <summary>A single value-copied broad-phase proxy as captured by a snapshot.</summary>
        internal struct Entry
        {
            public AABB FatAABB;
            public FixtureProxy Proxy;
        }
    }
}
