    /// refreshes the attachment index and island links of the bodies whose attachments changed.
                        // Flagged as modified so the broad phase updates their AABBs with the raised margin.
    /// Refreshes every live cluster's proxy rigid body from the current particles: its pose is
            // Dead zone: the frame fit carries a little noise even at rest (the least-squares
        // so a one-refresh-stale proxy pose is fine.
    /// Refreshes every soft body's derived state at the end of a step: the surface orientation,
        // Every body reads its own particles and writes only itself: updated in parallel, then
                // Flagged as modified so the broad phase updates their AABBs with the new margin.
    /// Tears a soft body right away: removes the given edges and cells and everything spanning
    /// them, splits the particles the tear passes through (the velocity of the original kept, its

        if let Some(rb) = bodies.get_mut(sb.root_body) {
            rb.wake_up(true);
}
