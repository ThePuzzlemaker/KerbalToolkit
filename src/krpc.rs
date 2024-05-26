use core::fmt;
use std::{collections::HashMap, hash::Hash, io::Cursor, net::TcpStream, time::Instant};

use base64::{engine::general_purpose::STANDARD, Engine as _};
use color_eyre::eyre;
use nalgebra::Vector3;
use prost::Message as _;
use time::Duration;
use tracing::trace;
use tungstenite::{stream::MaybeTlsStream, Message, WebSocket};
use varint_rs::VarintReader;

use crate::time::UT;

use self::encode::{DecodeValue, EncodeValue};

#[allow(clippy::module_inception)]
pub mod krpc {
    pub mod schema {
        include!(concat!(env!("OUT_DIR"), "/krpc.schema.rs"));
    }
}
pub mod encode;

pub struct Client {
    rpc: WebSocket<MaybeTlsStream<TcpStream>>,
    _stream: WebSocket<MaybeTlsStream<TcpStream>>,
}

impl Client {
    pub fn new(name: &str, addr: &str, rpc_port: u16, stream_port: u16) -> eyre::Result<Self> {
        // TODO: something better than this
        let rpc_url = format!("ws://{addr}:{rpc_port}/?name={name}");
        let (mut rpc_stream, _) = tungstenite::client::connect(rpc_url)?;

        let call = krpc::schema::ProcedureCall {
            service: "KRPC".into(),
            procedure: "GetClientID".into(),
            service_id: 1,
            procedure_id: 1,
            arguments: vec![],
        };
        let request = krpc::schema::Request { calls: vec![call] };
        rpc_stream.send(Message::Binary(request.encode_to_vec()))?;

        let response = rpc_stream.read()?;
        let response = krpc::schema::Response::decode(&*response.into_data())?;

        let mut cur = Cursor::new(&response.results[0].value);
        let _ = cur.read_u32_varint();
        let pos = cur.position() as usize;
        let id = STANDARD.encode(&response.results[0].value[pos..]);

        let stream_url = format!("ws://{addr}:{stream_port}/?id={id}");
        let (stream_stream, _) = tungstenite::client::connect(stream_url)?;

        Ok(Self {
            rpc: rpc_stream,
            _stream: stream_stream,
        })
    }

    #[inline]
    pub fn krpc(&mut self) -> KRPC<'_> {
        KRPC(self)
    }

    #[inline]
    pub fn space_center(&mut self) -> SpaceCenter {
        SpaceCenter(self)
    }

    pub fn raw_call(
        &mut self,
        request: krpc::schema::Request,
    ) -> eyre::Result<krpc::schema::Response> {
        let then = Instant::now();
        self.rpc.send(Message::Binary(request.encode_to_vec()))?;

        let response = self.rpc.read()?;
        let response = krpc::schema::Response::decode(&*response.into_data())?;
        trace!("{}", Duration::try_from(Instant::now() - then).unwrap());

        if let Some(e) = response.error {
            return Err(e.into());
        }

        for res in &response.results {
            if let Some(e) = res.error.clone() {
                return Err(e.into());
            }
        }

        Ok(response)
    }

    pub fn procedure_call<V: DecodeValue>(
        &mut self,
        service: String,
        procedure: String,
        arguments: Vec<krpc::schema::Argument>,
    ) -> eyre::Result<V> {
        let call = krpc::schema::ProcedureCall {
            service,
            procedure,
            service_id: 0,
            procedure_id: 0,
            arguments,
        };
        let request = krpc::schema::Request { calls: vec![call] };
        let mut response = self.raw_call(request)?;

        if response.results.len() > 1 {
            panic!();
        }

        let result = std::mem::take(&mut response.results[0]);
        V::decode_value(&result.value)
    }
}

#[repr(transparent)]
pub struct KRPC<'a>(&'a mut Client);

#[repr(transparent)]
pub struct SpaceCenter<'a>(&'a mut Client);

impl<'a> KRPC<'a> {
    pub fn get_status(&'a mut self) -> eyre::Result<krpc::schema::Status> {
        self.0
            .procedure_call("KRPC".into(), "GetStatus".into(), vec![])
    }
}

impl<'a> SpaceCenter<'a> {
    pub fn get_bodies(&mut self) -> eyre::Result<HashMap<String, CelestialBody>> {
        self.0
            .procedure_call("SpaceCenter".into(), "get_Bodies".into(), vec![])
    }

    pub fn get_editor(&mut self) -> eyre::Result<Editor> {
        self.0
            .procedure_call("SpaceCenter".into(), "get_Editor".into(), vec![])
    }

    pub fn get_active_vessel(&mut self) -> eyre::Result<Option<Vessel>> {
        self.0
            .procedure_call("SpaceCenter".into(), "get_ActiveVessel".into(), vec![])
    }

    pub fn get_ut(&mut self) -> eyre::Result<UT> {
        let ut: f64 = self
            .0
            .procedure_call("SpaceCenter".into(), "get_UT".into(), vec![])?;
        Ok(UT::new_seconds(ut))
    }

    pub fn get_vessels(&mut self) -> eyre::Result<Vec<Vessel>> {
        self.0
            .procedure_call("SpaceCenter".into(), "get_Vessels".into(), vec![])
    }
}

#[derive(Copy, Clone, Debug)]
pub struct CelestialBody {
    pub id: u64,
}

impl CelestialBody {
    pub fn get_name(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<String> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "CelestialBody_get_Name".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_satellites(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Vec<CelestialBody>> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "CelestialBody_get_Satellites".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_gravitational_parameter(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<f64> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "CelestialBody_get_GravitationalParameter".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_equatorial_radius(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<f64> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "CelestialBody_get_EquatorialRadius".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_rotational_period(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<f64> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "CelestialBody_get_RotationalPeriod".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_initial_rotation(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<f64> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "CelestialBody_get_InitialRotation".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_orbit(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Option<Orbit>> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "CelestialBody_get_Orbit".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_is_star(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<bool> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "CelestialBody_get_IsStar".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_sphere_of_influence(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<f64> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "CelestialBody_get_SphereOfInfluence".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_non_rotating_reference_frame(
        self,
        sc: &mut SpaceCenter<'_>,
    ) -> eyre::Result<ReferenceFrame> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "CelestialBody_get_NonRotatingReferenceFrame".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct ReferenceFrame {
    id: u64,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct Orbit {
    id: u64,
}

impl Orbit {
    pub fn get_body(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<CelestialBody> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Orbit_get_Body".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_ephemerides(
        self,
        sc: &mut SpaceCenter<'_>,
        epoch: UT,
    ) -> eyre::Result<crate::kepler::orbits::Orbit> {
        let args = vec![krpc::schema::Argument {
            position: 0,
            value: self.encode_value()?,
        }];
        let sma = krpc::schema::ProcedureCall {
            service: "SpaceCenter".into(),
            procedure: "Orbit_get_SemiMajorAxis".into(),
            arguments: args.clone(),
            ..Default::default()
        };
        let ecc = krpc::schema::ProcedureCall {
            service: "SpaceCenter".into(),
            procedure: "Orbit_get_Eccentricity".into(),
            arguments: args.clone(),
            ..Default::default()
        };
        let inc = krpc::schema::ProcedureCall {
            service: "SpaceCenter".into(),
            procedure: "Orbit_get_Inclination".into(),
            arguments: args.clone(),
            ..Default::default()
        };
        let lan = krpc::schema::ProcedureCall {
            service: "SpaceCenter".into(),
            procedure: "Orbit_get_LongitudeOfAscendingNode".into(),
            arguments: args.clone(),
            ..Default::default()
        };
        let argpe = krpc::schema::ProcedureCall {
            service: "SpaceCenter".into(),
            procedure: "Orbit_get_ArgumentOfPeriapsis".into(),
            arguments: args.clone(),
            ..Default::default()
        };
        let ta = krpc::schema::ProcedureCall {
            service: "SpaceCenter".into(),
            procedure: "Orbit_TrueAnomalyAtUT".into(),
            arguments: vec![
                args[0].clone(),
                krpc::schema::Argument {
                    position: 1,
                    value: epoch.into_duration().as_seconds_f64().encode_value()?,
                },
            ],
            ..Default::default()
        };
        let req = krpc::schema::Request {
            calls: vec![sma, ecc, inc, lan, argpe, ta],
        };
        let response = sc.0.raw_call(req)?;
        let sma = f64::decode_value(&response.results[0].value)? / 1000.0;
        let ecc = f64::decode_value(&response.results[1].value)?;
        let inc = f64::decode_value(&response.results[2].value)?;
        let lan = f64::decode_value(&response.results[3].value)?;
        let argpe = f64::decode_value(&response.results[4].value)?;
        let ta = f64::decode_value(&response.results[5].value)?;
        Ok(crate::kepler::orbits::Orbit {
            p: sma * (1.0 - ecc.powi(2)),
            e: ecc,
            i: inc,
            lan,
            argpe,
            epoch,
            ta,
        })
    }

    pub fn get_semimajor_axis(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<f64> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Orbit_get_SemiMajorAxis".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_eccentricity(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<f64> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Orbit_get_Eccentricity".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_inclination(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<f64> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Orbit_get_Inclination".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_lan(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<f64> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Orbit_get_LongitudeOfAscendingNode".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn get_argpe(self, sc: &mut SpaceCenter<'_>) -> eyre::Result<f64> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Orbit_get_ArgumentOfPeriapsis".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
        )
    }

    pub fn true_anomaly_at_ut(self, sc: &mut SpaceCenter<'_>, ut: UT) -> eyre::Result<f64> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Orbit_TrueAnomalyAtUT".into(),
            vec![
                krpc::schema::Argument {
                    position: 0,
                    value: self.encode_value()?,
                },
                krpc::schema::Argument {
                    position: 1,
                    value: ut.into_duration().as_seconds_f64().encode_value()?,
                },
            ],
        )
    }

    pub fn position_at(
        self,
        sc: &mut SpaceCenter<'_>,
        ut: UT,
        rf: ReferenceFrame,
    ) -> eyre::Result<Vector3<f64>> {
        sc.0.procedure_call::<(f64, f64, f64)>(
            "SpaceCenter".into(),
            "Orbit_PositionAt".into(),
            vec![
                krpc::schema::Argument {
                    position: 0,
                    value: self.encode_value()?,
                },
                krpc::schema::Argument {
                    position: 1,
                    value: ut.into_duration().as_seconds_f64().encode_value()?,
                },
                krpc::schema::Argument {
                    position: 2,
                    value: rf.encode_value()?,
                },
            ],
        )
        .map(|x| Vector3::new(x.0 / 1000.0, x.1 / 1000.0, x.2 / 1000.0))
    }

    pub fn orbital_velocity_at(
        self,
        sc: &mut SpaceCenter<'_>,
        ut: UT,
        rf: ReferenceFrame,
    ) -> eyre::Result<Vector3<f64>> {
        sc.0.procedure_call::<(f64, f64, f64)>(
            "SpaceCenter".into(),
            "Orbit_OrbitalVelocityAt".into(),
            vec![
                krpc::schema::Argument {
                    position: 0,
                    value: self.encode_value()?,
                },
                krpc::schema::Argument {
                    position: 1,
                    value: ut.into_duration().as_seconds_f64().encode_value()?,
                },
                krpc::schema::Argument {
                    position: 2,
                    value: rf.encode_value()?,
                },
            ],
        )
        .map(|x| Vector3::new(x.0 / 1000.0, x.1 / 1000.0, x.2 / 1000.0))
    }

    pub fn reference_plane_direction(
        sc: &mut SpaceCenter<'_>,
        rf: ReferenceFrame,
    ) -> eyre::Result<Vector3<f64>> {
        sc.0.procedure_call::<(f64, f64, f64)>(
            "SpaceCenter".into(),
            "Orbit_static_ReferencePlaneDirection".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: rf.encode_value()?,
            }],
        )
        .map(|x| Vector3::new(x.0, x.1, x.2))
    }

    pub fn reference_plane_normal(
        sc: &mut SpaceCenter<'_>,
        rf: ReferenceFrame,
    ) -> eyre::Result<Vector3<f64>> {
        sc.0.procedure_call::<(f64, f64, f64)>(
            "SpaceCenter".into(),
            "Orbit_static_ReferencePlaneNormal".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: rf.encode_value()?,
            }],
        )
        .map(|x| Vector3::new(x.0, x.1, x.2))
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct Editor {
    id: u64,
}

impl Editor {
    pub fn get_current_ship(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Option<EditorShip>> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Editor_get_CurrentShip".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct Vessel {
    pub id: u64,
}

impl Vessel {
    pub fn get_parts(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Parts> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Vessel_get_Parts".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_orbit(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Orbit> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Vessel_get_Orbit".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_name(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<String> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Vessel_get_Name".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_state_vector(
        &self,
        sc: &mut SpaceCenter<'_>,
        rf: ReferenceFrame,
    ) -> eyre::Result<(String, Vector3<f64>, Vector3<f64>, UT)> {
        let args = vec![
            krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            },
            krpc::schema::Argument {
                position: 1,
                value: rf.encode_value()?,
            },
        ];
        let soi = krpc::schema::ProcedureCall {
            service: "KerbTk".into(),
            procedure: "VesselSOIBodyName".into(),
            arguments: vec![krpc::schema::Argument {
                position: 0,
                value: self.encode_value()?,
            }],
            ..Default::default()
        };
        let pos = krpc::schema::ProcedureCall {
            service: "SpaceCenter".into(),
            procedure: "Vessel_Position".into(),
            arguments: args.clone(),
            ..Default::default()
        };
        let vel = krpc::schema::ProcedureCall {
            service: "SpaceCenter".into(),
            procedure: "Vessel_Velocity".into(),
            arguments: args.clone(),
            ..Default::default()
        };
        let ut = krpc::schema::ProcedureCall {
            service: "SpaceCenter".into(),
            procedure: "get_UT".into(),
            arguments: vec![],
            ..Default::default()
        };
        let req = krpc::schema::Request {
            calls: vec![soi, pos, vel, ut],
        };
        let response = sc.0.raw_call(req)?;
        let soi = String::decode_value(&response.results[0].value)?;
        let pos = <(f64, f64, f64)>::decode_value(&response.results[1].value)?;
        let vel = <(f64, f64, f64)>::decode_value(&response.results[2].value)?;
        let ut = UT::new_seconds(f64::decode_value(&response.results[3].value)?);
        Ok((
            soi,
            Vector3::new(pos.0 / 1000.0, pos.2 / 1000.0, pos.1 / 1000.0),
            Vector3::new(vel.0 / 1000.0, vel.2 / 1000.0, vel.1 / 1000.0),
            ut,
        ))
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct EditorShip {
    id: u64,
}

impl EditorShip {
    pub fn get_parts(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<EditorParts> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "EditorShip_get_Parts".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_ship_name(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<String> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "EditorShip_get_ShipName".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_ship_description(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<String> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "EditorShip_get_ShipDescription".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct EditorParts {
    id: u64,
}

impl EditorParts {
    pub fn get_all(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Vec<Part>> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "EditorParts_get_All".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct Parts {
    id: u64,
}

impl Parts {
    pub fn get_all(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Vec<Part>> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Parts_get_All".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct Part {
    id: u64,
}

impl Part {
    pub fn get_name(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<String> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Part_get_Name".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_title(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<String> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Part_get_Title".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_tag(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<String> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Part_get_Tag".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_parent(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Option<Part>> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Part_get_Parent".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_children(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Vec<Part>> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Part_get_Children".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_decoupler(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Option<Decoupler>> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Part_get_Decoupler".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_ro_decoupler(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Option<RODecoupler>> {
        sc.0.procedure_call(
            "KerbTk".into(),
            "RODecouplerOnPart".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_axially_attached(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<bool> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Part_get_AxiallyAttached".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_radially_attached(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<bool> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Part_get_RadiallyAttached".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct Decoupler {
    id: u64,
}

impl Decoupler {
    pub fn get_is_omni_decoupler(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<bool> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Decoupler_get_IsOmniDecoupler".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_attached_part(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Option<Part>> {
        sc.0.procedure_call(
            "SpaceCenter".into(),
            "Decoupler_get_AttachedPart".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct RODecoupler {
    id: u64,
}

impl RODecoupler {
    pub fn get_top_decoupler(&self, sc: &mut SpaceCenter<'_>) -> eyre::Result<Option<Decoupler>> {
        sc.0.procedure_call(
            "KerbTk".into(),
            "RODecoupler_get_TopDecoupler".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }

    pub fn get_bottom_decoupler(
        &self,
        sc: &mut SpaceCenter<'_>,
    ) -> eyre::Result<Option<Decoupler>> {
        sc.0.procedure_call(
            "KerbTk".into(),
            "RODecoupler_get_BottomDecoupler".into(),
            vec![krpc::schema::Argument {
                position: 0,
                value: self.id.encode_value()?,
            }],
        )
    }
}

impl fmt::Display for krpc::schema::Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}::{}: {}\n{}",
            self.service, self.name, self.description, self.stack_trace
        )
    }
}

impl std::error::Error for krpc::schema::Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        None
    }

    fn description(&self) -> &str {
        "description() is deprecated; use Display"
    }

    fn cause(&self) -> Option<&dyn std::error::Error> {
        self.source()
    }
}
