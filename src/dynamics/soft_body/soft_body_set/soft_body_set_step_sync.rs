    /// refreshes the attachment index and island links of the bodies whose attachments changed.
        // so a one-refresh-stale proxy pose is fine.
    /// Refreshes every soft body's derived state at the end of a step: the surface orientation,
        // Every body reads its own particles and writes only itself: updated in parallel, then
    /// Tears a soft body right away: removes the given edges and cells and everything spanning
    /// them, splits the particles the tear passes through (the velocity of the original kept, its

        if let Some(rb) = bodies.get_mut(sb.root_body) {
            rb.wake_up(true);
}
