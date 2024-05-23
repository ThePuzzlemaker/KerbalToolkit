use std::collections::{HashMap, HashSet};

use color_eyre::eyre;
use serde::{Deserialize, Serialize};

use crate::{
    arena::{Arena, IdLike},
    krpc::Client,
};

pub struct Vessel {}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct VesselClass {
    pub name: String,
    pub description: String,
    pub shortcode: String,
    pub parts: Arena<PartId, Part>,
    pub root: Option<PartId>,
}

impl VesselClass {
    pub fn load_parts_from_editor(
        client: &mut Client,
    ) -> eyre::Result<(Arena<PartId, Part>, Option<PartId>)> {
        let mut map = HashMap::new();
        let mut parts = Arena::new();
        let mut root = None;

        let mut sc = client.space_center();
        for part in sc
            .get_editor()?
            .get_current_ship(&mut sc)?
            .unwrap()
            .get_parts(&mut sc)?
            .get_all(&mut sc)?
        {
            let name = part.get_name(&mut sc)?;
            let title = part.get_title(&mut sc)?;
            let tag = part.get_tag(&mut sc)?;

            let rodecoupler = part.get_ro_decoupler(&mut sc)?;
            let decoupler = part.get_decoupler(&mut sc)?;
            let decouplers = if let Some(rodecoupler) = rodecoupler {
                // TODO: make these non-nullable
                let top = rodecoupler.get_top_decoupler(&mut sc)?.unwrap();
                let bot = rodecoupler.get_bottom_decoupler(&mut sc)?.unwrap();
                Some(Decouplers::RODecoupler {
                    top: Decoupler {
                        is_omni_decoupler: top.get_is_omni_decoupler(&mut sc)?,
                        attached_part: top.get_attached_part(&mut sc)?.map(|part| {
                            *map.entry(part)
                                .or_insert_with(|| parts.push(Part::default()))
                        }),
                    },
                    bot: Decoupler {
                        is_omni_decoupler: bot.get_is_omni_decoupler(&mut sc)?,
                        attached_part: bot.get_attached_part(&mut sc)?.map(|part| {
                            *map.entry(part)
                                .or_insert_with(|| parts.push(Part::default()))
                        }),
                    },
                })
            } else {
                decoupler
                    .map(|decoupler| -> eyre::Result<_> {
                        Ok(Decouplers::Single(Decoupler {
                            is_omni_decoupler: decoupler.get_is_omni_decoupler(&mut sc)?,
                            attached_part: decoupler.get_attached_part(&mut sc)?.map(|part| {
                                *map.entry(part)
                                    .or_insert_with(|| parts.push(Part::default()))
                            }),
                        }))
                    })
                    .transpose()?
            };

            let children = part
                .get_children(&mut sc)?
                .into_iter()
                .map(|child| {
                    *map.entry(child)
                        .or_insert_with(|| parts.push(Part::default()))
                })
                .collect();
            let parent = part.get_parent(&mut sc)?.map(|parent| {
                *map.entry(parent)
                    .or_insert_with(|| parts.push(Part::default()))
            });

            let attachment = if part.get_radially_attached(&mut sc)? {
                Attachment::Radial
            } else if part.get_axially_attached(&mut sc)? {
                Attachment::Axial
            } else {
                Attachment::None
            };

            let part1 = Part {
                parent,
                children,
                name,
                title,
                tag,
                decouplers,
                attachment,
            };

            if let Some(id) = map.get(&part) {
                if parent.is_none() {
                    root = Some(*id)
                }
                parts[*id] = part1;
            } else {
                let id = parts.push(part1);
                if parent.is_none() {
                    root = Some(id)
                }
                map.insert(part, id);
            }
        }

        Ok((parts, root))
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Part {
    pub parent: Option<PartId>,
    pub children: Vec<PartId>,
    pub name: String,
    pub title: String,
    pub tag: String,
    pub decouplers: Option<Decouplers>,
    pub attachment: Attachment,
}
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Attachment {
    Radial,
    Axial,
    None,
}

impl Default for Attachment {
    fn default() -> Self {
        Self::None
    }
}

#[derive(
    Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Deserialize, Serialize,
)]
pub struct PartId(u32);

impl IdLike for PartId {
    fn from_raw(index: usize) -> Self {
        Self(index as u32)
    }

    fn into_raw(self) -> usize {
        self.0 as usize
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub enum Decouplers {
    Single(Decoupler),
    RODecoupler { top: Decoupler, bot: Decoupler },
    ProceduralFairing(ProceduralFairingDecoupler),
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub struct ProceduralFairingDecoupler;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Decoupler {
    pub is_omni_decoupler: bool,
    pub attached_part: Option<PartId>,
}

pub fn decoupled_vessels(
    vessel: &VesselClass,
    fired_decouplers: &[PartId],
    ro_decouplers: &HashMap<PartId, (bool, bool)>,
) -> Vec<HashSet<PartId>> {
    let parts: HashSet<PartId> = vessel.parts.iter().map(|(x, _)| x).collect();
    let mut worklist = vec![parts];
    let mut vessels = vec![];
    let mut visited_decouplers = HashSet::new();
    let fired_decouplers = fired_decouplers
        .iter()
        .copied()
        .chain(ro_decouplers.keys().copied())
        .collect::<Vec<_>>();

    while let Some(subvessel) = worklist.pop() {
        let mut found_decoupler = false;
        for &decoupler in &fired_decouplers {
            if !visited_decouplers.contains(&decoupler) && subvessel.contains(&decoupler) {
                found_decoupler = true;
                visited_decouplers.insert(decoupler);

                match vessel.parts[decoupler].decouplers.as_ref().expect("oops") {
                    Decouplers::Single(d) => {
                        let mut explosive_node = HashSet::new();
                        let mut secondary_node = HashSet::new();

                        if let Some(attached_part) = d.attached_part {
                            traverse_until(
                                vessel,
                                attached_part,
                                &visited_decouplers,
                                &mut explosive_node,
                            );
                        }
                        if let Some(parent) = vessel.parts[decoupler].parent {
                            if Some(parent) != d.attached_part {
                                traverse_until(
                                    vessel,
                                    parent,
                                    &visited_decouplers,
                                    &mut secondary_node,
                                );
                            }
                        }

                        if d.is_omni_decoupler {
                            for &child in &vessel.parts[decoupler].children {
                                if Some(child) == d.attached_part {
                                    continue;
                                }

                                if vessel.parts[child].attachment == Attachment::Radial {
                                    let mut set = HashSet::new();
                                    traverse_until(vessel, child, &visited_decouplers, &mut set);
                                    worklist.push(set);
                                } else {
                                    traverse_until(
                                        vessel,
                                        child,
                                        &visited_decouplers,
                                        &mut secondary_node,
                                    );
                                }
                            }
                            let mut set = HashSet::new();
                            set.insert(decoupler);
                            worklist.push(set);
                        } else {
                            for &child in &vessel.parts[decoupler].children {
                                if Some(child) == d.attached_part {
                                    continue;
                                }

                                traverse_until(
                                    vessel,
                                    child,
                                    &visited_decouplers,
                                    &mut secondary_node,
                                );
                            }
                            secondary_node.insert(decoupler);
                        }

                        worklist.extend([explosive_node, secondary_node]);
                    }
                    Decouplers::RODecoupler { top, bot } => {
                        let (top_en, bot_en) = ro_decouplers
                            .get(&decoupler)
                            .copied()
                            .unwrap_or((false, false));

                        let mut explosive_node = HashSet::new();
                        let mut secondary_node = HashSet::new();

                        let mut omni = false;
                        let attached_part = match (top_en, bot_en) {
                            (true, true) => {
                                omni = true;
                                top.attached_part
                            }
                            (true, false) => top.attached_part,
                            (false, true) => bot.attached_part,
                            (false, false) => None,
                        };

                        if let Some(attached_part) = attached_part {
                            traverse_until(
                                vessel,
                                attached_part,
                                &visited_decouplers,
                                &mut explosive_node,
                            );
                        }
                        if let Some(parent) = vessel.parts[decoupler].parent {
                            if Some(parent) != attached_part {
                                traverse_until(
                                    vessel,
                                    parent,
                                    &visited_decouplers,
                                    &mut secondary_node,
                                );
                            }
                        }

                        if omni {
                            for &child in &vessel.parts[decoupler].children {
                                if Some(child) == attached_part {
                                    continue;
                                }

                                if vessel.parts[child].attachment == Attachment::Radial {
                                    let mut set = HashSet::new();
                                    traverse_until(vessel, child, &visited_decouplers, &mut set);
                                    worklist.push(set);
                                } else {
                                    traverse_until(
                                        vessel,
                                        child,
                                        &visited_decouplers,
                                        &mut secondary_node,
                                    );
                                }
                            }
                            let mut set = HashSet::new();
                            set.insert(decoupler);
                            worklist.push(set);
                        } else {
                            for &child in &vessel.parts[decoupler].children {
                                if Some(child) == attached_part {
                                    continue;
                                }

                                traverse_until(
                                    vessel,
                                    child,
                                    &visited_decouplers,
                                    &mut secondary_node,
                                );
                            }
                            secondary_node.insert(decoupler);
                        }

                        worklist.extend([explosive_node, secondary_node]);
                    }
                    Decouplers::ProceduralFairing(_) => todo!(),
                }

                break;
            }
        }

        if !found_decoupler && !subvessel.is_empty() {
            vessels.push(subvessel);
        }
    }

    vessels
}

fn traverse_until(
    vessel: &VesselClass,
    target: PartId,
    visited: &HashSet<PartId>,
    set: &mut HashSet<PartId>,
) {
    let mut worklist = vec![target];
    while let Some(target) = worklist.pop() {
        if set.contains(&target) {
            continue;
        }

        if !visited.contains(&target) {
            set.insert(target);
        }

        for &child in &vessel.parts[target].children {
            if !visited.contains(&child) {
                worklist.push(child);
            }
        }

        if let Some(parent) = vessel.parts[target].parent {
            if !visited.contains(&parent) {
                worklist.push(parent);
            }
        }
    }
}
