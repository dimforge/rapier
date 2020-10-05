//! Publish-subscribe mechanism for internal events.

use serde::export::PhantomData;
use std::collections::VecDeque;

/// The position of a subscriber on a pub-sub queue.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct PubSubCursor<T> {
    // Index of the next message to read.
    id: u32,
    next: u32,
    _phantom: PhantomData<T>,
}

impl<T> PubSubCursor<T> {
    fn id(&self, num_deleted: u32) -> usize {
        (self.id - num_deleted) as usize
    }

    fn next(&self, num_deleted: u32) -> usize {
        (self.next - num_deleted) as usize
    }
}

/// A pub-sub queue.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct PubSub<T> {
    deleted_messages: u32,
    deleted_offsets: u32,
    messages: VecDeque<T>,
    offsets: VecDeque<u32>,
}

impl<T> PubSub<T> {
    /// Create a new empty pub-sub queue.
    pub fn new() -> Self {
        Self {
            deleted_offsets: 0,
            deleted_messages: 0,
            messages: VecDeque::new(),
            offsets: VecDeque::new(),
        }
    }

    /// Publish a new message.
    pub fn publish(&mut self, message: T) {
        if self.offsets.is_empty() {
            // No subscribers, drop the message.
            return;
        }

        self.messages.push_back(message);
    }

    /// Subscribe to the queue.
    ///
    /// A subscription cannot be cancelled.
    pub fn subscribe(&mut self) -> PubSubCursor<T> {
        let cursor = PubSubCursor {
            next: self.messages.len() as u32 + self.deleted_messages,
            id: self.offsets.len() as u32 + self.deleted_offsets,
            _phantom: PhantomData,
        };

        self.offsets.push_back(cursor.next);
        cursor
    }

    /// Read the i-th message not yet read by the given subsciber.
    pub fn read_ith(&self, cursor: &PubSubCursor<T>, i: usize) -> Option<&T> {
        self.messages
            .get(cursor.next(self.deleted_messages) as usize + i)
    }

    /// Get the messages not yet read by the given subscriber.
    pub fn read(&self, cursor: &PubSubCursor<T>) -> impl Iterator<Item = &T> {
        let next = cursor.next(self.deleted_messages);

        // TODO: use self.queue.range(next..) once it is stabilised.
        MessageRange {
            queue: &self.messages,
            next,
        }
    }

    /// Makes the given subscribe acknowledge all the messages in the queue.
    ///
    /// A subscriber cannot read acknowledged messages any more.
    pub fn ack(&mut self, cursor: &mut PubSubCursor<T>) {
        // Update the cursor.
        cursor.next = self.messages.len() as u32 + self.deleted_messages;
        self.offsets[cursor.id(self.deleted_offsets)] = u32::MAX;
        cursor.id = self.offsets.len() as u32 + self.deleted_offsets;
        self.offsets.push_back(cursor.next);

        // Now clear the messages we don't need to
        // maintain in memory anymore.
        while self.offsets.front() == Some(&u32::MAX) {
            self.offsets.pop_front();
            self.deleted_offsets += 1;
        }

        // There must be at least one offset otherwise
        // that would mean we have no subscribers.
        let next = self.offsets.front().unwrap();
        let num_to_delete = *next - self.deleted_messages;

        for _ in 0..num_to_delete {
            self.messages.pop_front();
        }

        self.deleted_messages += num_to_delete;
    }
}

struct MessageRange<'a, T> {
    queue: &'a VecDeque<T>,
    next: usize,
}

impl<'a, T> Iterator for MessageRange<'a, T> {
    type Item = &'a T;

    #[inline(always)]
    fn next(&mut self) -> Option<&'a T> {
        let result = self.queue.get(self.next);
        self.next += 1;
        result
    }
}
