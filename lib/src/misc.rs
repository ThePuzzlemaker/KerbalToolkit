//! Simple sorted list collection like the one found in the .NET collections library.

use std::iter::FromIterator;

use serde::{Deserialize, Serialize};

/// `SortedList` stores multiple `(K, V)` tuples ordered by K, then in the order of insertion for `V`.
/// Implmented using two `Vec` this should be fast for in-order inserts and quite bad in the
/// worst-case of reverse insertion order.
///
/// # Example
///
/// ```
/// use sorted_list::SortedList;
///
/// let mut list: SortedList<u32, u8> = SortedList::new();
/// list.insert(0, 0);
/// list.insert(1, 1);
/// list.insert(0, 2);
///
/// assert_eq!(
///     list.iter().collect::<Vec<_>>(),
///     vec![(&0, &0), (&0, &2), (&1, &1)]);
/// ```
#[derive(Debug, PartialEq, Deserialize, Serialize)]
pub struct SortedList<K: Ord, V: PartialEq> {
    pub keys: Vec<K>,
    pub values: Vec<V>,
}

impl<K: Ord, V: PartialEq> Default for SortedList<K, V> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K: Ord, V: PartialEq> SortedList<K, V> {
    /// Creates a new as small as possible `SortedList`
    pub fn new() -> Self {
        SortedList {
            keys: Vec::new(),
            values: Vec::new(),
        }
    }

    /// Returns the number of tuples
    pub fn len(&self) -> usize {
        self.keys.len()
    }

    /// Returns true if the collection is empty.
    pub fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }

    /// Returns `true` if the `(key, value)` did not exist in the sorted list before and it exists now,
    /// `false` otherwise.
    pub fn insert(&mut self, key: K, value: V) -> bool {
        match self.keys.binary_search(&key) {
            Ok(found_at) => {
                let insertion_position = self.find_insertion_positition(found_at, &key, &value);

                if let Some(insertion_position) = insertion_position {
                    insertion_position.insert(key, value, &mut self.keys, &mut self.values);
                    true
                } else {
                    false
                }
            }
            Err(insert_at) => {
                self.keys.insert(insert_at, key);
                self.values.insert(insert_at, value);

                true
            }
        }
    }

    fn find_insertion_positition(
        &self,
        from: usize,
        key: &K,
        value: &V,
    ) -> Option<InsertionPosition> {
        let mut keys = self.keys.iter().skip(from);
        let mut values = self.values.iter().skip(from);

        let mut index: usize = from;

        loop {
            index += 1;

            match (keys.next(), values.next()) {
                (Some(other_key), Some(other_value)) => {
                    if key == other_key {
                        if value == other_value {
                            // found it already
                            return None;
                        }
                    } else {
                        // we ran past the matching keys, insert before
                        return Some(InsertionPosition::Before(index));
                    }
                }
                (None, None) => {
                    return Some(InsertionPosition::Last);
                }
                (_, _) => unreachable!(),
            };
        }
    }

    pub fn first_value_of_mut(&mut self, key: &K) -> Option<&mut V> {
        self.find_first_position(key)
            .ok()
            .map(|idx| &mut self.values[idx])
    }

    pub fn find_first_position(&self, key: &K) -> Result<usize, usize> {
        match self.keys.binary_search(key) {
            Ok(mut pos) => {
                while pos > 0 && key == &self.keys[pos] {
                    pos -= 1;
                }

                if pos == 0 {
                    if key == &self.keys[0] {
                        Ok(0)
                    } else {
                        Ok(1)
                    }
                } else {
                    Ok(pos + 1)
                }
            }
            Err(pos) => Err(pos),
        }
    }
}

impl<K: Ord + Clone, V: PartialEq + Clone> Clone for SortedList<K, V> {
    fn clone(&self) -> Self {
        SortedList {
            keys: self.keys.clone(),
            values: self.values.clone(),
        }
    }
}

impl<K: Ord, V: PartialEq> FromIterator<(K, V)> for SortedList<K, V> {
    fn from_iter<T: IntoIterator<Item = (K, V)>>(iter: T) -> Self {
        let mut this = Self::new();

        for (k, v) in iter {
            this.insert(k, v);
        }

        this
    }
}

/// Helper value for knowning where to insert the value
enum InsertionPosition {
    Before(usize),
    Last,
}

impl InsertionPosition {
    fn insert<K, V>(self, key: K, value: V, keys: &mut Vec<K>, values: &mut Vec<V>) {
        match self {
            InsertionPosition::Before(index) => {
                keys.insert(index - 1, key);
                values.insert(index - 1, value);

                assert_eq!(keys.len(), values.len());
            }
            InsertionPosition::Last => {
                keys.push(key);
                values.push(value);

                assert_eq!(keys.len(), values.len());
            }
        }
    }
}
