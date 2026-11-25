using System;

namespace Genbox.VelcroPhysics.Collision.ContactSystem
{
    [Flags]
    internal enum ContactFlags : int
    {
        Unknown = 0,

        /// <summary>Used when crawling contact graph when forming islands.</summary>
        IslandFlag = 1,

        /// <summary>Set when the shapes are touching.</summary>
        TouchingFlag = 2,

        /// <summary>This contact can be disabled (by user)</summary>
        EnabledFlag = 4,

        /// <summary>This contact needs filtering because a fixture filter was changed.</summary>
        FilterFlag = 8,

        /// <summary>This bullet contact had a TOI event</summary>
        BulletHitFlag = 16,

        /// <summary>This contact has a valid TOI in m_toi</summary>
        TOIFlag = 32,

        /// <summary>
        /// A begin contact transition was observed while callbacks were suppressed (the discrete
        /// Collide() pass of a continuous step or a silent TOI refresh) and still has to be reported.
        /// Stamped with the transition's time of impact so events can be emitted in time order.
        /// </summary>
        PendingBeginFlag = 64,

        /// <summary>An end contact transition was observed while callbacks were suppressed and is pending.</summary>
        PendingEndFlag = 128,

        /// <summary>
        /// The contact was created or refreshed during the current step's broad-phase passes and its
        /// touching state has already been evaluated for sensor event reporting.
        /// </summary>
        RefreshedFlag = 256
    }
}