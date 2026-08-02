use crate::alloc_prelude::*;

/// A slot-indexed union-find (disjoint-set) over `0..len`, with path halving
/// and union by size.
///
/// Deterministic: no hashing, and the representative of a merged set only
/// depends on set sizes and union order (ties pick the first argument's root),
/// so identical edge sequences always produce identical partitions.
#[derive(Clone, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct UnionFind {
    /// `parent[i]` for set members; a root stores itself.
    parents: Vec<u32>,
    /// Set sizes, meaningful at roots only.
    sizes: Vec<u32>,
}

impl UnionFind {
    /// Resets to `len` singleton sets, reusing the allocations.
    pub fn reset(&mut self, len: usize) {
        self.parents.clear();
        self.parents.extend(0..len as u32);
        self.sizes.clear();
        self.sizes.resize(len, 1);
    }

    /// The representative of `i`'s set, halving the path along the way.
    pub fn find(&mut self, i: u32) -> u32 {
        let mut i = i;
        loop {
            let p = self.parents[i as usize];
            if p == i {
                return i;
            }
            let gp = self.parents[p as usize];
            self.parents[i as usize] = gp;
            i = gp;
        }
    }

    /// Merges the sets containing `a` and `b`.
    pub fn union(&mut self, a: u32, b: u32) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra == rb {
            return;
        }
        // Union by size; ties attach `rb` under `ra`.
        let (big, small) = if self.sizes[ra as usize] >= self.sizes[rb as usize] {
            (ra, rb)
        } else {
            (rb, ra)
        };
        self.parents[small as usize] = big;
        self.sizes[big as usize] += self.sizes[small as usize];
    }

    /// Compresses every node to point directly at its root, enabling [`Self::root`].
    pub fn flatten(&mut self) {
        for i in 0..self.parents.len() as u32 {
            let root = self.find(i);
            self.parents[i as usize] = root;
        }
    }

    /// The representative of `i`'s set as a single read. Only valid after
    /// [`Self::flatten`] with no `union` in between.
    pub fn root(&self, i: u32) -> u32 {
        let root = self.parents[i as usize];
        debug_assert_eq!(self.parents[root as usize], root, "not flattened");
        root
    }

    /// The number of elements in `root`'s set; meaningful only if `root` is a
    /// set representative.
    pub fn size(&self, root: u32) -> u32 {
        self.sizes[root as usize]
    }
}

#[cfg(test)]
mod test {
    use super::UnionFind;

    #[test]
    fn union_find_components() {
        let mut uf = UnionFind::default();
        uf.reset(6);
        uf.union(0, 1);
        uf.union(2, 3);
        uf.union(1, 2);
        assert_eq!(uf.find(0), uf.find(3));
        assert_ne!(uf.find(0), uf.find(4));
        assert_ne!(uf.find(4), uf.find(5));

        uf.flatten();
        assert_eq!(uf.root(0), uf.root(3));
        assert_ne!(uf.root(0), uf.root(4));
        assert_eq!(uf.size(uf.root(0)), 4);
        assert_eq!(uf.size(uf.root(4)), 1);

        // Reset reuses the buffers and clears the partition.
        uf.reset(3);
        assert_ne!(uf.find(0), uf.find(1));
    }
}
