use std::{
    collections::HashMap,
    sync::{mpsc::Receiver, Arc},
};

use color_eyre::eyre::{self, OptionExt};
use kerbtk::{
    arena::Arena,
    bodies::{Body, SolarSystem},
    ffs::{Resource, ResourceId},
    kepler::orbits::{Orbit, StateVector},
    krpc::{self, Client},
    maneuver::{self, Maneuver, ManeuverKind},
    time::{GET, UT},
    translunar::{TLIConstraintSet, TLISolver},
    vessel::{Part, PartId, VesselClass, VesselClassRef},
};
use parking_lot::RwLock;

use crate::{i18n, mission::Mission};

pub enum HReq {
    LoadVesselPartsFromEditor,
    LoadVesselClassFromFlight,
    LoadVesselsList,
    LoadSystem,
    LoadStateVector(krpc::Vessel),
    RPCConnect(String, String, String),
    RPCDisconnect,
    LoadVesselGETBase(krpc::Vessel),
    LoadVesselResources(krpc::Vessel, VesselClassRef),
    CalculateTLI(Box<TLIInputs>),
}

pub struct TLIInputs {
    pub cs: TLIConstraintSet,
    pub central: Body,
    pub moon: Body,
    pub get_base: UT,
    pub maxiter: u64,
    pub temp: f64,
    pub opt_periapse: bool,
    pub allow_retrograde: bool,
}

pub enum HRes {
    LoadedVesselClass(Arena<PartId, Part>, Option<PartId>, HashMap<u32, PartId>),
    LoadedVesselsList(HashMap<String, krpc::Vessel>),
    LoadedSystem(SolarSystem),
    LoadedStateVector(StateVector),
    LoadedVesselResources(HashMap<(PartId, ResourceId), Resource>),
    LoadedGETBase(UT),
    Connected(String),
    CalculatedManeuver(Maneuver),
    Disconnected,
    ConnectionFailure(eyre::Report),
    MPTTransfer(Maneuver, String),
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
                    client.as_mut().ok_or_eyre(i18n!("error-krpc-noconn"))?,
                )?;
                Ok(LoadedVesselClass(parts.0, parts.1, parts.2))
            }
            LoadStateVector(vessel) => {
                let mut sc = client
                    .as_mut()
                    .ok_or_eyre(i18n!("error-krpc-noconn"))?
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
                        .ok_or_eyre(i18n!("error-krpc-svbody"))?
                        .clone(),
                    frame: kerbtk::kepler::orbits::ReferenceFrame::BodyCenteredInertial,
                    position: sv.1,
                    velocity: sv.2,
                    time: sv.3,
                };
                Ok(LoadedStateVector(sv))
            }
            LoadVesselResources(vessel, class) => {
                let mut sc = client
                    .as_mut()
                    .ok_or_eyre(i18n!("error-krpc-noconn"))?
                    .space_center();

                let parts = vessel.get_parts(&mut sc)?.get_all(&mut sc)?;
                let class = class.0.read();

                let mut part_resources = HashMap::new();
                for part in parts {
                    let pid = part.get_persistent_id(&mut sc)?;
                    // TODO: reduce reliance on this lock to prevent UI slowdowns
                    // TODO: mass modifiers
                    if let Some(&partid) = class.persistent_id_map.get(&pid) {
                        let resources = part.get_resources(&mut sc)?;
                        let resources = resources.get_all(&mut sc)?;
                        for resource in resources {
                            let id = ResourceId(resource.get_id(&mut sc)?);
                            let density = resource.get_density(&mut sc)?;
                            let amount = resource.get_amount(&mut sc)?;
                            part_resources.insert(
                                (partid, id),
                                Resource {
                                    free: density <= f32::EPSILON,
                                    max_amount: resource.get_max_amount(&mut sc)? as f64,
                                    amount: amount as f64,
                                    density: density as f64,
                                    residual: 0.0,
                                    enabled: resource.get_enabled(&mut sc)?,
                                    name: resource.get_name(&mut sc)?.into(),
                                },
                            );
                        }
                    }
                }

                Ok(LoadedVesselResources(part_resources))
            }
            LoadVesselGETBase(vessel) => {
                let mut sc = client
                    .as_mut()
                    .ok_or_eyre(i18n!("error-krpc-noconn"))?
                    .space_center();
                let ut = vessel.get_met_base(&mut sc)?;
                Ok(LoadedGETBase(ut))
            }
            LoadVesselClassFromFlight => {
                let parts = VesselClass::load_parts_from_flight(
                    client.as_mut().ok_or_eyre(i18n!("error-krpc-noconn"))?,
                )?;
                Ok(LoadedVesselClass(parts.0, parts.1, parts.2))
            }
            CalculateTLI(inputs) => {
                let TLIInputs {
                    cs,
                    central,
                    moon,
                    get_base,
                    maxiter,
                    temp,
                    opt_periapse,
                    allow_retrograde,
                } = *inputs;
                let t0 = cs.central_sv.time;
                let mut tli_solver =
                    TLISolver::new(cs, central, moon, t0, opt_periapse, allow_retrograde);
                let sol = tli_solver
                    .run(maxiter, temp)
                    .ok_or_eyre(i18n!("error-tli-nosoln"))?;
                let tig_vector = sol.sv_init;
                let frenet_inv = maneuver::frenet(&tig_vector)
                    .try_inverse()
                    .ok_or_eyre(i18n!("error-tli-general"))?;
                let deltav = frenet_inv * sol.deltav;
                let man = Maneuver {
                    geti: GET::from_duration(tig_vector.time - get_base),
                    deltav,
                    tig_vector,
                    kind: ManeuverKind::TranslunarInjection,
                };
                Ok(CalculatedManeuver(man))
            }

            LoadVesselsList => {
                let mut sc = client
                    .as_mut()
                    .ok_or_eyre(i18n!("error-krpc-noconn"))?
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
                    .ok_or_eyre(i18n!("error-krpc-noconn"))?
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
                    .ok_or_eyre(i18n!("error-krpc-noconn"))?
                    .space_center()
                    .get_bodies()?;
                for (name, body) in bodies {
                    let mut sc = client
                        .as_mut()
                        .ok_or_eyre(i18n!("error-krpc-noconn"))?
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
