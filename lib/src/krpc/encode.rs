use std::collections::HashMap;
use std::hash::{BuildHasher, Hash};
use std::io::{Cursor, Write};

use color_eyre::eyre;
use prost::bytes::{Buf, BufMut};
use prost::Message;
use varint_rs::{VarintReader, VarintWriter};

use super::krpc::schema::{Dictionary, DictionaryEntry, List, Status, Tuple};
use super::{
    CelestialBody, Decoupler, Editor, EditorParts, EditorShip, LaunchClamp, Module, Orbit,
    PFDecoupler, Part, Parts, RODecoupler, ReferenceFrame, Resource, Resources, Vessel,
};

pub trait DecodeValue: Sized {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self>;
}

pub trait EncodeValue: Sized {
    fn encode_value(self) -> eyre::Result<Vec<u8>>;
}

impl<K: DecodeValue + Hash + Eq, V: DecodeValue, S: BuildHasher + Default> DecodeValue
    for HashMap<K, V, S>
{
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        let mut map = HashMap::default();
        let dict = Dictionary::decode(&mut &*buf)?;
        for entry in dict.entries {
            let key = K::decode_value(&entry.key)?;
            let value = V::decode_value(&entry.value)?;
            map.insert(key, value);
        }
        Ok(map)
    }
}

impl<K: EncodeValue + Hash + Eq, V: EncodeValue, S: BuildHasher + Default> EncodeValue
    for HashMap<K, V, S>
{
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut entries = vec![];
        for (key, value) in self {
            let key = key.encode_value()?;
            let value = value.encode_value()?;
            entries.push(DictionaryEntry { key, value });
        }
        Ok(Dictionary { entries }.encode_to_vec())
    }
}

impl<T: DecodeValue> DecodeValue for Vec<T> {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        let mut v = vec![];
        let list = List::decode(&mut &*buf)?;
        for item in list.items {
            v.push(T::decode_value(&item)?);
        }
        Ok(v)
    }
}

impl<T: EncodeValue> EncodeValue for Vec<T> {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        for item in self {
            v.push(item.encode_value()?);
        }
        Ok(List { items: v }.encode_to_vec())
    }
}

macro_rules! encode_tuples {
    ($($($x:ident),* => $($ix:literal: $i:ident),*);*) => {
	$(
	    impl<$($x: EncodeValue),*> EncodeValue for ($($x),*,) {
		fn encode_value(self) -> eyre::Result<Vec<u8>> {
		    let mut v = vec![];
		    let ($($i,)*) = self;
		    $(v.push($i.encode_value()?);)*
		    Ok(Tuple { items: v }.encode_to_vec())
		}
	    }

	    impl<$($x: DecodeValue),*> DecodeValue for ($($x),*,) {
		fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
		    let tuple = Tuple::decode(&mut &*buf)?;
		    let tuple = ($(
			$x::decode_value(&tuple.items[$ix])?,
		    )*);
		    Ok(tuple)
		}
	    }
	)*
    }
}

encode_tuples![
    T => 0: a;
    T, U => 0: a, 1: b;
    T, U, V => 0: a, 1: b, 2: c;
    T, U, V, W => 0: a, 1: b, 2: c, 3: d;
    T, U, V, W, X => 0: a, 1: b, 2: c, 3: d, 4: e
];

impl DecodeValue for String {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        let mut cur = Cursor::new(buf);
        let _ = cur.read_u64_varint();
        let pos = cur.position() as usize;
        Ok(String::from_utf8(buf[pos..].to_vec())?)
    }
}

impl EncodeValue for String {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.len() as u64)?;
        v.write_all(self.as_bytes())?;
        Ok(v)
    }
}

impl<T: DecodeValue> DecodeValue for Option<T> {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        if buf == [0u8] {
            Ok(None)
        } else {
            Ok(Some(T::decode_value(buf)?))
        }
    }
}

impl<T: EncodeValue> EncodeValue for Option<T> {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        match self {
            Some(v) => v.encode_value(),
            None => Ok(vec![0u8]),
        }
    }
}

impl DecodeValue for f32 {
    fn decode_value(mut buf: &[u8]) -> eyre::Result<Self> {
        Ok(buf.get_f32_le())
    }
}

impl EncodeValue for f32 {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.put_f32_le(self);
        Ok(v)
    }
}

impl DecodeValue for f64 {
    fn decode_value(mut buf: &[u8]) -> eyre::Result<Self> {
        Ok(buf.get_f64_le())
    }
}

impl EncodeValue for f64 {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.put_f64_le(self);
        Ok(v)
    }
}

impl DecodeValue for i32 {
    fn decode_value(mut buf: &[u8]) -> eyre::Result<Self> {
        Ok(buf.read_i32_varint()?)
    }
}

impl EncodeValue for i32 {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_i32_varint(self)?;
        Ok(v)
    }
}

impl DecodeValue for u32 {
    fn decode_value(mut buf: &[u8]) -> eyre::Result<Self> {
        Ok(buf.read_u32_varint()?)
    }
}

impl EncodeValue for u32 {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u32_varint(self)?;
        Ok(v)
    }
}

impl DecodeValue for u64 {
    fn decode_value(mut buf: &[u8]) -> eyre::Result<Self> {
        Ok(buf.read_u64_varint()?)
    }
}

impl EncodeValue for u64 {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self)?;
        Ok(v)
    }
}

impl DecodeValue for bool {
    fn decode_value(mut buf: &[u8]) -> eyre::Result<Self> {
        Ok(buf.read_i32_varint()? != 0)
    }
}

impl EncodeValue for bool {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_i32_varint(self as i32)?;
        Ok(v)
    }
}

impl DecodeValue for Vec<u8> {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Vec::<u8>::decode(buf)?)
    }
}

impl EncodeValue for Vec<u8> {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        Ok(self.encode_to_vec())
    }
}

impl DecodeValue for Orbit {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Orbit {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for Orbit {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for CelestialBody {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(CelestialBody {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for CelestialBody {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for Status {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Status::decode(buf)?)
    }
}

impl DecodeValue for Vessel {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for Vessel {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for EditorShip {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for EditorShip {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for EditorParts {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for EditorParts {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for Parts {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for Parts {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for Part {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for Part {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for Editor {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for Editor {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for Decoupler {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for Decoupler {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for RODecoupler {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for RODecoupler {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for ReferenceFrame {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for ReferenceFrame {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for Resources {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for Resources {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for Resource {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for Resource {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for LaunchClamp {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for LaunchClamp {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for Module {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for Module {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}

impl DecodeValue for PFDecoupler {
    fn decode_value(buf: &[u8]) -> eyre::Result<Self> {
        Ok(Self {
            id: u64::decode_value(buf)?,
        })
    }
}

impl EncodeValue for PFDecoupler {
    fn encode_value(self) -> eyre::Result<Vec<u8>> {
        let mut v = vec![];
        v.write_u64_varint(self.id)?;
        Ok(v)
    }
}
