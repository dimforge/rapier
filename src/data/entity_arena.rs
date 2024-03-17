use crate::data::arena::Index;
use crate::data::Arena;
use crate::data::Coarena;
use bevy::prelude::Entity;
use std::ops::IndexMut;

impl From<Entity> for Index {
    #[inline]
    fn from(value: Entity) -> Self {
        Index::from_raw_parts(value.index(), value.generation())
    }
}

impl From<Index> for Entity {
    #[inline]
    fn from(value: Index) -> Self {
        let (id, gen) = value.into_raw_parts();
        Entity::from_bits(id as u64 | ((gen as u64) << u32::BITS))
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, Default)]
struct EntityMap {
    entity_to_index: Coarena<Index>,
    index_to_entity: Coarena<Entity>, // Is this really needed?
}

impl EntityMap {
    pub fn get_index(&self, entity: Entity) -> Option<Index> {
        self.entity_to_index.get(entity.into()).copied()
    }

    pub fn get_entity(&self, index: Index) -> Option<Entity> {
        self.index_to_entity.get(index).copied()
    }

    pub fn insert(&mut self, entity: Entity, index: Index) {
        self.entity_to_index.insert(entity.into(), index);
        self.index_to_entity
            .insert_with_default(index, entity, Entity::PLACEHOLDER);
    }

    pub fn remove(&mut self, entity: Entity) {
        if let Some(index) = self.entity_to_index.remove(
            entity.into(),
            Index::from_raw_parts(crate::INVALID_U32, crate::INVALID_U32),
        ) {
            self.index_to_entity.remove(index, Entity::PLACEHOLDER);
        }
    }
}

impl std::ops::Index<Index> for EntityMap {
    type Output = Entity;

    fn index(&self, index: Index) -> &Self::Output {
        self.index_to_entity.get(index).expect("Index not found.")
    }
}

impl std::ops::Index<Entity> for EntityMap {
    type Output = Index;

    fn index(&self, entity: Entity) -> &Self::Output {
        self.entity_to_index
            .get(entity.into())
            .expect("Index not found.")
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct EntityArena<T> {
    entity_map: EntityMap,
    elements: Arena<T>,
}

impl<T> Default for EntityArena<T> {
    fn default() -> Self {
        Self {
            entity_map: EntityMap::default(),
            elements: Arena::default(),
        }
    }
}

impl<T> EntityArena<T> {
    pub fn insert(&mut self, entity: Entity, value: T) {
        let index = self.elements.insert(value);
        self.entity_map.insert(entity, index);
    }

    pub fn remove(&mut self, entity: Entity) -> Option<T> {
        let elt = self.elements.remove(self.entity_map.get_index(entity)?)?;
        self.entity_map.remove(entity);
        Some(elt)
    }

    pub fn contains(&self, entity: Entity) -> bool {
        self.entity_map.get_index(entity).is_some()
    }

    pub fn len(&self) -> usize {
        self.elements.len()
    }

    pub fn is_empty(&self) -> bool {
        self.elements.is_empty()
    }

    pub fn get(&self, entity: Entity) -> Option<&T> {
        self.elements.get(self.entity_map.get_index(entity)?)
    }

    pub fn get_mut(&mut self, entity: Entity) -> Option<&mut T> {
        self.elements.get_mut(self.entity_map.get_index(entity)?)
    }

    pub fn iter(&self) -> impl ExactSizeIterator<Item = (Entity, &T)> {
        self.elements
            .iter()
            .map(|(h, data)| (self.entity_map[h], data))
    }

    pub fn iter_mut(&mut self) -> impl ExactSizeIterator<Item = (Entity, &mut T)> {
        self.elements
            .iter_mut()
            .map(|(h, data)| (self.entity_map[h], data))
    }
}

impl<T> std::ops::Index<Entity> for EntityArena<T> {
    type Output = T;

    fn index(&self, index: Entity) -> &Self::Output {
        &self.elements[self.entity_map[index]]
    }
}

impl<T> IndexMut<Entity> for EntityArena<T> {
    fn index_mut(&mut self, index: Entity) -> &mut Self::Output {
        &mut self.elements[self.entity_map[index]]
    }
}
