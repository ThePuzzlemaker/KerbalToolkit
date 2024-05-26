use std::{
    collections::HashMap,
    sync::{mpsc::Receiver, Arc},
};

use color_eyre::eyre::{self, OptionExt};
use kerbtk::{
    arena::Arena,
    bodies::{Body, SolarSystem},
    kepler::orbits::{Orbit, StateVector},
    krpc::{self, Client},
    time::UT,
    vessel::{Part, PartId, VesselClass},
};
use parking_lot::RwLock;

use crate::mission::Mission;

pub enum HReq {
    LoadVesselPartsFromEditor,
    LoadVesselClassFromFlight,
    LoadVesselsList,
    LoadSystem,
    LoadStateVector(krpc::Vessel),
    RPCConnect(String, String, String),
    RPCDisconnect,
}

pub enum HRes {
    LoadedVesselClass(Arena<PartId, Part>, Option<PartId>),
    LoadedVesselsList(HashMap<String, krpc::Vessel>),
    LoadedSystem(SolarSystem),
    LoadedStateVector(StateVector),
    Connected(String),
    Disconnected,
    ConnectionFailure(eyre::Report),
}

pub fn handler_thread(
    rx: Receiver<(usize, HReq)>,
    tx: std::sync::mpsc::Sender<(usize, eyre::Result<HRes>)>,
    mission: Arc<RwLock<Mission>>,
) {
    use HReq::*;
    use HRes::*;
    let mut client = None;
    while let Ok((txi, req)) = rx.recv() {
        let res = (|| match req {
            LoadVesselPartsFromEditor => {
                let parts = VesselClass::load_parts_from_editor(
                    client.as_mut().ok_or_eyre("kRPC not connected.")?,
                )?;
                Ok(LoadedVesselClass(parts.0, parts.1))
            }
            LoadStateVector(vessel) => {
                let mut sc = client
                    .as_mut()
                    .ok_or_eyre("kRPC not connected.")?
                    .space_center();
                let rf = vessel
                    .get_orbit(&mut sc)?
                    .get_body(&mut sc)?
                    .get_non_rotating_reference_frame(&mut sc)?;
                let sv = vessel.get_state_vector(&mut sc, rf)?;
                let sv = StateVector {
                    body: mission
                        .read()
                        .system
                        .bodies
                        .get(&*sv.0)
                        .ok_or_eyre("StateVector body was not loaded")?
                        .clone(),
                    frame: kerbtk::kepler::orbits::ReferenceFrame::BodyCenteredInertial,
                    position: sv.1,
                    velocity: sv.2,
                    time: sv.3,
                };
                Ok(LoadedStateVector(sv))
            }
            LoadVesselClassFromFlight => {
                println!("== STATE VECTOR TEST ==");
                let mut sc = client
                    .as_mut()
                    .ok_or_eyre("kRPC not connected.")?
                    .space_center();
                let vessel = sc.get_active_vessel()?.expect("active vessel");
                let rf = vessel
                    .get_orbit(&mut sc)?
                    .get_body(&mut sc)?
                    .get_non_rotating_reference_frame(&mut sc)?;

                let sv = vessel.get_state_vector(&mut sc, rf)?;
                let sv = StateVector {
                    body: mission
                        .read()
                        .system
                        .bodies
                        .get(&*sv.0)
                        .expect("oops")
                        .clone(),
                    frame: kerbtk::kepler::orbits::ReferenceFrame::BodyCenteredInertial,
                    position: sv.1,
                    velocity: sv.2,
                    time: sv.3,
                };

                println!("SV0 {:#?}", sv);
                let sv1 = sv.next_soi(&mission.read().system, 1e-7, 30000).unwrap();
                println!("SV1 {sv1:#?}");
                let sv2 = sv1.next_soi(&mission.read().system, 1e-7, 30000).unwrap();
                println!("SV2 {sv2:#?}");
                println!(
                    "SV3 {:#?}",
                    sv2.next_soi(&mission.read().system, 1e-7, 30000).unwrap()
                );

                Ok(Connected("debug".into()))
            }
            LoadVesselsList => {
                let mut sc = client
                    .as_mut()
                    .ok_or_eyre("kRPC not connected.")?
                    .space_center();
                let vessels = sc
                    .get_vessels()?
                    .into_iter()
                    .map(|x| Ok((x.get_name(&mut sc)?, x)))
                    .collect::<Result<HashMap<_, _>, eyre::Report>>()?;
                Ok(LoadedVesselsList(vessels))
            }
            RPCConnect(host, rpc, stream) => {
                let res = Client::new(
                    "KerbalToolkit",
                    &host,
                    rpc.parse().unwrap_or(50000),
                    stream.parse().unwrap_or(50001),
                );
                if let Err(e) = res {
                    return Ok(ConnectionFailure(e));
                }
                client = Some(res?);

                let version = client
                    .as_mut()
                    .ok_or_eyre("kRPC not connected.")?
                    .krpc()
                    .get_status()?;
                Ok(Connected(version.version))
            }
            RPCDisconnect => {
                client = None;
                Ok(Disconnected)
            }
            LoadSystem => {
                let mut system = SolarSystem::default();
                let bodies = client
                    .as_mut()
                    .ok_or_eyre("kRPC is not connected.")?
                    .space_center()
                    .get_bodies()?;
                for (name, body) in bodies {
                    let mut sc = client
                        .as_mut()
                        .ok_or_eyre("kRPC is not connected.")?
                        .space_center();
                    let mu = body.get_gravitational_parameter(&mut sc)? / (1000.0f64.powi(3));
                    let radius = body.get_equatorial_radius(&mut sc)? / 1000.0;
                    let rotperiod = body.get_rotational_period(&mut sc)?;
                    let rotini = body.get_initial_rotation(&mut sc)?;
                    let satellites = body
                        .get_satellites(&mut sc)?
                        .into_iter()
                        .map(|x| x.get_name(&mut sc))
                        .collect::<eyre::Result<Vec<_>>>()?;

                    let (ephem, parent) = if let Some(orbit) = body.get_orbit(&mut sc)? {
                        let ephem = orbit.get_ephemerides(&mut sc, UT::new_seconds(0.0))?;

                        let parent = Some(orbit.get_body(&mut sc)?.get_name(&mut sc)?);

                        (ephem, parent)
                    } else {
                        (
                            Orbit {
                                p: 0.0,
                                e: 0.0,
                                i: 0.0,
                                lan: 0.0,
                                argpe: 0.0,
                                epoch: UT::new_seconds(0.0),
                                ta: 0.0,
                            },
                            None,
                        )
                    };

                    let name: Arc<str> = name.into();
                    let body = Body {
                        mu,
                        radius,
                        ephem,
                        rotperiod,
                        rotini,
                        satellites: satellites.into_iter().map(|x| x.into()).collect(),
                        name: name.clone(),
                        parent: parent.map(|x| x.into()),
                        is_star: body.get_is_star(&mut sc)?,
                        soi: body.get_sphere_of_influence(&mut sc)? / 1000.0,
                    };
                    system.bodies.insert(name, body);
                }
                Ok(LoadedSystem(system))
            }
        })();
        let _ = tx.send((txi, res));
    }
}
