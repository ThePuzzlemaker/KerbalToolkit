use std::{
    collections::HashMap,
    hash::Hash,
    ops::{Index, IndexMut},
};

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct Arena<Id: IdLike + Hash + Copy + Eq + PartialOrd, T> {
    inner: HashMap<Id, T>,
    next_id: Id,
}

impl<Id: IdLike + Hash + Copy + Eq + PartialOrd, T> Arena<Id, T> {
    pub fn new() -> Self {
        Self {
            inner: HashMap::new(),
            next_id: Id::from_raw(0),
        }
    }

    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    pub fn push(&mut self, x: T) -> Id {
        let id = self.next_id;
        self.next_id = Id::from_raw(self.next_id.into_raw() + 1);
        self.inner.insert(id, x);
        id
    }

    pub fn insert(&mut self, id: Id, x: T) {
        self.inner.insert(id, x);
        if id >= self.next_id {
            self.next_id = Id::from_raw(self.next_id.into_raw() + 1);
        }
    }

    pub fn get(&self, id: Id) -> Option<&T> {
        self.inner.get(&id)
    }

    pub fn iter(&self) -> impl Iterator<Item = (Id, &T)> {
        self.inner.iter().map(|(i, v)| (*i, v))
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (Id, &mut T)> {
        self.inner.iter_mut().map(|(i, v)| (*i, v))
    }

    pub fn retain(&mut self, mut f: impl FnMut(Id, &T) -> bool) {
        self.inner.retain(|k, v| f(*k, v));
    }
}

impl<Id: IdLike + Hash + Copy + Eq + PartialOrd, T> Default for Arena<Id, T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<Id: IdLike + Hash + Copy + Eq + PartialOrd, T> Index<Id> for Arena<Id, T> {
    type Output = T;

    fn index(&self, index: Id) -> &Self::Output {
        &self.inner[&index]
    }
}

impl<Id: IdLike + Hash + Copy + Eq + PartialOrd, T> IndexMut<Id> for Arena<Id, T> {
    fn index_mut(&mut self, index: Id) -> &mut Self::Output {
        self.inner.get_mut(&index).unwrap()
    }
}

pub trait IdLike {
    fn from_raw(index: usize) -> Self;
    fn into_raw(self) -> usize;
}
