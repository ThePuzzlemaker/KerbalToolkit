use std::{cmp, collections::HashMap, mem};

use nalgebra::Vector3;
use ordered_float::OrderedFloat;

use crate::{
    arena::{Arena, IdLike},
    math::H1,
};

/// A propellant used in an engine or RCS thruster
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Propellant {
    /// Is this propellant ignored for ISP calculations?
    ignore_for_isp: bool,
    /// Consumption rate of this resource, in units per second
    ratio: f64,
    /// Where is this propellant allowed to come from?
    flow_mode: FlowMode,
    /// Density of the propellant, in tons per unit.
    density: f64,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum FlowMode {
    NoFlow,
    AllVessel,
    StagePriorityFlow,
    StackPrioritySearch,
    AllVesselBalance,
    StagePriorityFlowBalance,
    StageStackFlow,
    StageStackFlowBalance,
    Null,
}

/// A generic resource consumed by a part.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Resource {
    /// Does consumption of this resource affect part mass?
    pub free: bool,
    /// The maximum amount of this resource that this part can hold.
    pub max_amount: f64,
    /// How much of this resource is stored?
    pub amount: f64,
    /// Density of the resource, in tons per unit.
    pub density: f64,
    /// Predicted maximum or actualized residuals of this resource, as
    /// a multiplier of [`Self::max_amount`].
    pub residual: f64,
}

impl Resource {
    pub fn residual_threshold(&self) -> f64 {
        self.residual * self.max_amount
    }

    pub fn drain(&mut self, resource_drain: f64) {
        self.amount -= resource_drain;
        if self.amount < 0.0 {
            self.amount = 0.0;
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct FuelStats {
    /// Mass before engines burn (tons)
    pub start_mass: f64,
    start_time: f64,
    /// Mass after all engines burn to depletion (tons)
    pub end_mass: f64,

    /// Thrust generated from all active engines (N)
    pub thrust: f64,
    /// Specific impulse from all active engines (s)
    pub isp: f64,
    /// Maximum spool-up time over all active engines (s)
    pub spool_up_time: f64,
    /// Cumulative burn time of all active engines (s)
    pub delta_time: f64,

    /// Maximum RCS delta-v (m/s)
    pub max_rcs_deltav: f64,
    /// Minimum RCS delta-v (m/s)
    pub min_rcs_deltav: f64,
    /// Delta-v from all active engines (m/s)
    pub deltav: f64,

    /// Thrust generated from RCS (N)
    pub rcs_thrust: f64,
    /// Maximimum RCS burn time (s)
    pub rcs_delta_time: f64,
    /// Mass burned if RCS burns to depletion (tons)
    pub rcs_mass: f64,
    /// Specific impulse of RCS (s)
    pub rcs_isp: f64,

    /// Minimum RCS acceleration (m/s^2)
    pub rcs_start_tmr: f64,
    /// Maximum RCS acceleration (m/s^2)
    pub rcs_end_tmr: f64,
}

impl FuelStats {
    /// Maximum acceleration from all active engines (m/s^2)
    pub fn max_accel(&self) -> f64 {
        if self.end_mass > 0.0 {
            self.thrust / self.end_mass
        } else {
            0.0
        }
    }

    /// Mass burned when all active engines burn to depletion (tons)
    pub fn resource_mass(&self) -> f64 {
        self.start_mass - self.end_mass
    }

    /// Minimum RCS thrust-to-weight ratio
    pub fn rcs_start_twr(&self, gee_asl: f64) -> f64 {
        self.rcs_start_tmr / (G0 * gee_asl)
    }

    /// Maximum RCS thrust-to-weight ratio
    pub fn rcs_max_twr(&self, gee_asl: f64) -> f64 {
        self.rcs_end_tmr / (G0 * gee_asl)
    }

    /// Minimum engine thrust-to-weight ratio
    pub fn start_twr(&self, gee_asl: f64) -> f64 {
        if self.start_mass > 0.0 {
            self.thrust / (G0 * gee_asl * self.start_mass)
        } else {
            0.0
        }
    }

    /// Maximum engine thrust-to-weight ratio
    pub fn max_twr(&self, gee_asl: f64) -> f64 {
        self.max_accel() / (G0 * gee_asl)
    }
}

const G0: f64 = 9.80665;

pub struct SimVessel {
    pub parts: Arena<SimPartId, SimPart>,
    pub active_engines: Vec<Engine>,
    // pub active_rcs: Vec<RCS>,
    mass: f64,
    thrust_current: Vector3<f64>,
    //rcs_thrust: f64,
    thrust_magnitude: f64,
    thrust_no_cos_loss: f64,
    spoolup_current: f64,
    pub conditions: Conditions,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Conditions {
    pub atm_pressure: f64,
    pub atm_density: f64,
    pub mach_number: f64,
    pub main_throttle: f64,
}

impl SimVessel {
    fn update_mass(&mut self) {
        self.mass = 0.0;

        for (_, part) in self.parts.iter_mut() {
            part.update_mass();
            self.mass += part.mass;
        }
    }

    fn update_active_engines(&mut self) {
        for mut engine in mem::take(&mut self.active_engines) {
            if engine.mass_flow_rate <= 0.0 || engine.is_unrestartable_dead_engine {
                continue;
            }
            engine.update_engine_status(self);
            if !engine.is_operational {
                continue;
            }
            self.active_engines.push(engine);
        }

        self.compute_thrust_and_spoolup();
    }

    fn compute_thrust_and_spoolup(&mut self) {
        self.thrust_current = Vector3::zeros();
        self.thrust_magnitude = 0.0;
        self.thrust_no_cos_loss = 0.0;
        self.spoolup_current = 0.0;

        for engine in &mut self.active_engines {
            if !engine.is_operational {
                continue;
            }

            self.spoolup_current += engine.thrust_current.norm() * engine.module_spoolup_time;

            engine.update(self.conditions);
            self.thrust_current += engine.thrust_current;
            self.thrust_no_cos_loss += engine.thrust_current.norm();
        }

        self.thrust_magnitude = self.thrust_current.norm();
        self.spoolup_current /= self.thrust_current.norm();
    }

    fn update_engine_stats(&mut self) {
        for engine in &mut self.active_engines {
            engine.update(self.conditions);
        }
    }
}

#[derive(Clone, Debug)]
pub struct SimPart {
    crossfeed_part_set: Vec<SimPartId>,
    // links: Vec<SimPartId>,
    pub resources: HashMap<ResourceId, Resource>,
    resource_drains: HashMap<ResourceId, f64>,

    // activates_even_if_disconnected: bool,
    // is_throttle_locked: bool,
    resource_priority: i32,
    resource_request_remaining_threshold: f64,

    mass: f64,
    dry_mass: f64,
    crew_mass: f64,
    /// For all modules with variable mass, this represents the
    /// current mass of that module, depending on whether it is staged
    /// or unstaged.
    modules_current_mass: f64,
    disabled_resource_mass: f64,

    is_launch_clamp: bool,
    // is_engine: bool,
}

impl SimPart {
    fn update_mass(&mut self) {
        if self.is_launch_clamp {
            self.mass = 0.0;
            return;
        }

        self.mass = self.dry_mass
            + self.crew_mass
            + self.disabled_resource_mass
            + self.modules_current_mass;
        for (_, resource) in self.resources.iter() {
            self.mass += resource.amount * resource.density;
        }
    }

    fn apply_drains(&mut self, dt: f64) {
        for (id, drain) in &self.resource_drains {
            if let Some(resource) = self.resources.get_mut(id) {
                resource.drain(*drain * dt);
            }
        }
    }

    fn update_residuals(&mut self, residual: f64, res: ResourceId) {
        if let Some(resource) = self.resources.get_mut(&res) {
            resource.residual = cmp::max(OrderedFloat(resource.residual), OrderedFloat(residual)).0;
        }
    }

    fn clear_residuals(&mut self) {
        for (_, resource) in self.resources.iter_mut() {
            resource.residual = 0.0;
        }
    }

    fn add_drain(&mut self, res: ResourceId, consumption: f64) {
        self.resource_drains
            .entry(res)
            .and_modify(|x| *x += consumption)
            .or_insert(consumption);
    }

    fn max_time(&self) -> f64 {
        let mut max_time = f64::MAX;

        for (res, resource) in self.resources.iter() {
            if resource.free || resource.amount <= self.resource_request_remaining_threshold {
                continue;
            }

            if let Some(resource_drain) = self.resource_drains.get(res) {
                println!(
                    "amt - res = {}",
                    resource.amount - resource.residual_threshold()
                );
                println!("resource_drain = {}", resource_drain);
                let dt = (resource.amount - resource.residual_threshold()) / resource_drain;
                println!("dt = {}", dt);

                max_time = cmp::min(OrderedFloat(max_time), OrderedFloat(dt)).0;
            }
        }

        max_time
    }

    fn clear_resource_drains(&mut self) {
        self.resource_drains.clear();
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SimPartId(u32);

impl IdLike for SimPartId {
    fn from_raw(index: usize) -> Self {
        Self(index as u32)
    }

    fn into_raw(self) -> usize {
        self.0 as usize
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ResourceId(i32);

impl IdLike for ResourceId {
    fn from_raw(index: usize) -> Self {
        Self(index as i32)
    }

    fn into_raw(self) -> usize {
        self.0 as usize
    }
}

fn lerp(x: f64, y: f64, t: f64) -> f64 {
    x + t * (y - x)
}

// TODO: maybe use Rc<RefCell<Engine>> on this to prevent it being
// cloned unnecessarily
#[derive(Clone)]
pub struct Engine {
    propellants: HashMap<ResourceId, Propellant>,
    propellant_flow_modes: HashMap<ResourceId, FlowMode>,
    resource_consumptions: HashMap<ResourceId, f64>,
    thrust_transform_multipliers: Vec<f64>,
    thrust_direction_vectors: Vec<Vector3<f64>>,

    is_operational: bool,
    flow_multiplier: f64,
    thrust_current: Vector3<f64>,
    thrust_max: Vector3<f64>,
    thrust_min: Vector3<f64>,
    mass_flow_rate: f64,
    isp: f64,
    g: f64,
    max_fuel_flow: f64,
    max_thrust: f64,
    min_fuel_flow: f64,
    min_thrust: f64,
    mult_isp: f64,
    clamp: f64,
    flow_mult_cap: f64,
    flow_mult_cap_sharpness: f64,
    throttle_locked: bool,
    throttle_limiter: f64,
    atm_change_flow: bool,
    use_atm_curve: bool,
    use_atm_curve_isp: bool,
    use_throttle_isp_curve: bool,
    use_vel_curve: bool,
    use_vel_curve_isp: bool,
    module_residuals: f64,
    module_spoolup_time: f64,
    no_propellants: bool,
    is_unrestartable_dead_engine: bool,

    throttle_isp_curve: H1,
    throttle_isp_curve_atm_strength: H1,
    vel_curve: H1,
    vel_curve_isp: H1,
    atm_curve: H1,
    atm_curve_isp: H1,
    atmosphere_curve: H1,

    is_sepratron: bool,
    part: SimPartId,
}

impl Engine {
    fn update_engine_status(&mut self, vessel: &SimVessel) {
        self.is_operational = self.can_draw_resources(vessel);
    }

    fn can_draw_resources(&self, vessel: &SimVessel) -> bool {
        use FlowMode::*;
        if self.no_propellants {
            return false;
        }

        for &prop in self.resource_consumptions.keys() {
            match self.propellant_flow_modes[&prop] {
                NoFlow => {
                    if !self.part_has_resource(vessel, self.part, prop) {
                        return false;
                    }
                }
                AllVessel | AllVesselBalance | StagePriorityFlow | StagePriorityFlowBalance => {
                    if !self.parts_have_resource(vessel, vessel.parts.iter().map(|x| x.0), prop) {
                        return false;
                    }
                }
                StackPrioritySearch | StageStackFlow | StageStackFlowBalance => {
                    if !self.parts_have_resource(
                        vessel,
                        vessel.parts[self.part].crossfeed_part_set.iter().copied(),
                        prop,
                    ) {
                        return false;
                    }
                }
                Null => return false,
            }
        }

        true
    }

    fn part_has_resource(&self, vessel: &SimVessel, part: SimPartId, res: ResourceId) -> bool {
        if let Some(resource) = vessel.parts[part].resources.get(&res) {
            resource.amount
                > vessel.parts[part].resources[&res].max_amount * self.module_residuals
                    + vessel.parts[part].resource_request_remaining_threshold
        } else {
            false
        }
    }

    fn parts_have_resource(
        &self,
        vessel: &SimVessel,
        parts: impl Iterator<Item = SimPartId>,
        res: ResourceId,
    ) -> bool {
        for part in parts {
            if self.part_has_resource(vessel, part, res) {
                return true;
            }
        }
        false
    }

    fn update(&mut self, conditions: Conditions) {
        self.isp = self.isp_at_conditions(conditions);
        self.flow_multiplier = self.flow_multiplier_at_conditions(conditions);
        self.mass_flow_rate = self.flow_rate_at_conditions(conditions);
        self.refresh_thrust();
        self.set_consumption_rates();
    }

    fn flow_rate_at_conditions(&mut self, conditions: Conditions) -> f64 {
        let mut min_fuel_flow = self.min_fuel_flow;
        let mut max_fuel_flow = self.max_fuel_flow;

        println!("FR1: {min_fuel_flow} {max_fuel_flow}");

        if min_fuel_flow == 0.0 && self.min_thrust > 0.0 {
            min_fuel_flow = self.min_thrust / (self.atmosphere_curve.evaluate(0.0) * self.g);
        }
        if max_fuel_flow == 0.0 && self.max_thrust > 0.0 {
            max_fuel_flow = self.max_thrust / (self.atmosphere_curve.evaluate(0.0) * self.g);
        }

        println!(
            "FR. {min_fuel_flow} {max_fuel_flow} {} {} {}",
            conditions.main_throttle * 0.01 * self.throttle_limiter,
            self.flow_multiplier,
            lerp(
                min_fuel_flow,
                max_fuel_flow,
                conditions.main_throttle * 0.01 * self.throttle_limiter
            ) * self.flow_multiplier,
        );

        lerp(
            min_fuel_flow,
            max_fuel_flow,
            conditions.main_throttle * 0.01 * self.throttle_limiter,
        ) * self.flow_multiplier
    }

    fn refresh_thrust(&mut self) {
        self.thrust_current = Vector3::zeros();
        self.thrust_max = Vector3::zeros();
        self.thrust_min = Vector3::zeros();

        let thrust_limiter = self.throttle_limiter / 100.0;
        let max_thrust =
            self.max_fuel_flow * self.flow_multiplier * self.isp * self.g * self.mult_isp;
        let min_thrust =
            self.min_fuel_flow * self.flow_multiplier * self.isp * self.g * self.mult_isp;

        let e_max_thrust = min_thrust + (max_thrust - min_thrust) * thrust_limiter;
        let e_min_thrust = if self.throttle_locked {
            e_max_thrust
        } else {
            self.min_thrust
        };
        let e_current_thrust = self.mass_flow_rate * self.isp * self.g * self.mult_isp;

        for (i, thrust_direction_vector) in self.thrust_direction_vectors.iter().enumerate() {
            let thrust_transform_multiplier = self.thrust_transform_multipliers[i];
            let t_current_thrust = e_current_thrust * thrust_transform_multiplier;

            self.thrust_current += t_current_thrust * thrust_direction_vector;
            self.thrust_max += e_max_thrust * thrust_direction_vector * thrust_transform_multiplier;
            self.thrust_min += e_min_thrust * thrust_direction_vector * thrust_transform_multiplier;
        }
    }

    fn flow_multiplier_at_conditions(&mut self, conditions: Conditions) -> f64 {
        let mut flow_multiplier = 1.0;

        if self.atm_change_flow {
            if self.use_atm_curve {
                flow_multiplier = self
                    .atm_curve
                    .evaluate(conditions.atm_density * 40.0 / 49.0);
            } else {
                flow_multiplier = conditions.atm_density * 40.0 / 49.0;
            }
        }

        if self.use_vel_curve {
            flow_multiplier *= self.vel_curve.evaluate(conditions.mach_number);
        }

        if flow_multiplier > self.flow_mult_cap {
            let excess = flow_multiplier - self.flow_mult_cap;
            flow_multiplier = self.flow_mult_cap
                + excess / (self.flow_mult_cap_sharpness + excess / self.flow_mult_cap);
        }

        if flow_multiplier < self.clamp && self.clamp < 1.0 {
            flow_multiplier = self.clamp;
        }

        flow_multiplier
    }

    fn isp_at_conditions(&mut self, conditions: Conditions) -> f64 {
        let mut isp = self.atmosphere_curve.evaluate(conditions.atm_pressure);
        println!("ISP = {isp}");
        if self.use_throttle_isp_curve {
            isp *= lerp(
                1.0,
                self.throttle_isp_curve.evaluate(conditions.main_throttle),
                self.throttle_isp_curve_atm_strength
                    .evaluate(conditions.atm_pressure),
            );
        }
        if self.use_atm_curve_isp {
            isp *= self
                .atm_curve_isp
                .evaluate(conditions.atm_density * 40.0 / 49.0);
        }
        if self.use_vel_curve_isp {
            isp *= self.vel_curve_isp.evaluate(conditions.mach_number);
        }
        isp
    }

    fn set_consumption_rates(&mut self) {
        self.resource_consumptions.clear();
        self.propellant_flow_modes.clear();

        let mut total_density = 0.0;

        for (&id, propellant) in &self.propellants {
            let density = propellant.density;

            if density <= 0.0 {
                continue;
            }

            self.propellant_flow_modes.insert(id, propellant.flow_mode);

            if propellant.ignore_for_isp {
                continue;
            }

            total_density += propellant.ratio * density;
        }
        println!("dens {:#?}", total_density);

        let volume_flow_rate = self.mass_flow_rate / total_density;
        println!("VFR {volume_flow_rate}");

        for (&id, propellant) in &self.propellants {
            let density = propellant.density;

            let prop_volume_rate = propellant.ratio * volume_flow_rate;

            if density <= 0.0 {
                continue;
            }

            println!("PVR {prop_volume_rate}");

            self.resource_consumptions
                .entry(id)
                .and_modify(|x| *x += prop_volume_rate)
                .or_insert(prop_volume_rate);
        }
    }
}

#[derive(Debug)]
pub struct FuelFlowSimulation {
    pub segments: Vec<FuelStats>,
    current_segment: FuelStats,
    time: f64,
    dv_linear_thrust: bool,
    parts_with_resource_drains: Vec<SimPartId>,
    sources: Vec<SimPartId>,
}

impl FuelFlowSimulation {
    pub fn run(&mut self, vessel: &mut SimVessel) {
        self.time = 0.0;
        self.segments.clear();

        vessel.conditions.main_throttle = 1.0;

        'sim: {
            vessel.update_mass();
            vessel.update_engine_stats();
            vessel.update_active_engines();

            self.get_next_segment(vessel);
            //self.compute_rcs_min_values(vessel);

            self.update_resource_drains_and_residuals(vessel);
            let mut current_thrust = vessel.thrust_magnitude;

            for _ in 0..100 {
                if vessel.active_engines.is_empty()
                    || vessel.active_engines.iter().all(|x| x.is_sepratron)
                {
                    break 'sim;
                }

                let dt = self.minimum_time_step(vessel);

                if (vessel.thrust_magnitude - current_thrust).abs() > 1e-12 {
                    self.clear_residuals(vessel);
                    // self.compute_rcs_max_values(vessel);
                    self.finish_segment(vessel);
                    self.get_next_segment(vessel);
                    current_thrust = vessel.thrust_magnitude;
                }

                self.time += dt;
                self.apply_resource_drains(vessel, dt);

                vessel.update_mass();
                vessel.update_engine_stats();
                vessel.update_active_engines();
                self.update_resource_drains_and_residuals(vessel);
            }

            panic!("oops");
        }

        self.clear_residuals(vessel);
        //self.compute_rcs_max_values(vessel);
        self.finish_segment(vessel);
        self.parts_with_resource_drains.clear();
    }

    fn clear_residuals(&mut self, vessel: &mut SimVessel) {
        for part in &self.parts_with_resource_drains {
            vessel.parts[*part].clear_residuals();
        }
    }

    fn apply_resource_drains(&mut self, vessel: &mut SimVessel, dt: f64) {
        for part in &self.parts_with_resource_drains {
            vessel.parts[*part].apply_drains(dt);
        }
    }

    fn update_resource_drains_and_residuals(&mut self, vessel: &mut SimVessel) {
        use FlowMode::*;
        for part in &self.parts_with_resource_drains {
            vessel.parts[*part].clear_resource_drains();
            vessel.parts[*part].clear_residuals();
        }

        self.parts_with_resource_drains.clear();

        for e in vessel.active_engines.clone() {
            for (res, mode) in &e.propellant_flow_modes {
                match mode {
                    NoFlow => self.update_resource_drains_and_residuals_in_part(
                        vessel,
                        e.part,
                        e.resource_consumptions[res],
                        *res,
                        e.module_residuals,
                    ),
                    AllVessel | AllVesselBalance => self
                        .update_resource_drains_and_residuals_in_parts(
                            vessel,
                            vessel
                                .parts
                                .iter()
                                .map(|x| x.0)
                                .collect::<Vec<_>>()
                                .into_iter(),
                            e.resource_consumptions[res],
                            *res,
                            false,
                            e.module_residuals,
                        ),
                    StagePriorityFlow | StagePriorityFlowBalance => self
                        .update_resource_drains_and_residuals_in_parts(
                            vessel,
                            vessel
                                .parts
                                .iter()
                                .map(|x| x.0)
                                .collect::<Vec<_>>()
                                .into_iter(),
                            e.resource_consumptions[res],
                            *res,
                            true,
                            e.module_residuals,
                        ),
                    StageStackFlow | StageStackFlowBalance | StackPrioritySearch => self
                        .update_resource_drains_and_residuals_in_parts(
                            vessel,
                            vessel.parts[e.part].crossfeed_part_set.clone().into_iter(),
                            e.resource_consumptions[res],
                            *res,
                            true,
                            e.module_residuals,
                        ),
                    Null => {}
                }
            }
        }
    }

    fn update_resource_drains_and_residuals_in_parts(
        &mut self,
        vessel: &mut SimVessel,
        parts: impl Iterator<Item = SimPartId>,
        resource_consumption: f64,
        res: ResourceId,
        use_priority: bool,
        residual: f64,
    ) {
        let mut max_priority = i32::MIN;

        self.sources.clear();

        for p in parts {
            if let Some(resource) = vessel.parts[p].resources.get(&res) {
                if resource.free
                    || resource.amount
                        <= residual * resource.max_amount
                            + vessel.parts[p].resource_request_remaining_threshold
                {
                    continue;
                }

                if use_priority {
                    if vessel.parts[p].resource_priority < max_priority {
                        continue;
                    }

                    if vessel.parts[p].resource_priority > max_priority {
                        self.sources.clear();
                        max_priority = vessel.parts[p].resource_priority;
                    }
                }
                self.sources.push(p);
            }
        }

        for source in self.sources.clone() {
            println!("res={:#?} source={:#?}", res, &vessel.parts[source]);
            self.update_resource_drains_and_residuals_in_part(
                vessel,
                source,
                resource_consumption / self.sources.len() as f64,
                res,
                residual,
            );
        }
    }

    fn update_resource_drains_and_residuals_in_part(
        &mut self,
        vessel: &mut SimVessel,
        p: SimPartId,
        resource_consumption: f64,
        res: ResourceId,
        residual: f64,
    ) {
        self.parts_with_resource_drains.push(p);
        vessel.parts[p].add_drain(res, resource_consumption);
        vessel.parts[p].update_residuals(residual, res);
    }

    fn minimum_time_step(&self, vessel: &SimVessel) -> f64 {
        let max_time = self.resource_max_time(vessel);

        if (0.0..f64::MAX).contains(&max_time) {
            max_time
        } else {
            0.0
        }
    }

    fn resource_max_time(&self, vessel: &SimVessel) -> f64 {
        let mut max_time = f64::MAX;

        for part in &self.parts_with_resource_drains {
            max_time = cmp::min(
                OrderedFloat(vessel.parts[*part].max_time()),
                OrderedFloat(max_time),
            )
            .0;
        }

        max_time
    }

    fn get_next_segment(&mut self, vessel: &mut SimVessel) {
        self.current_segment = FuelStats {
            delta_time: 0.0,
            start_time: 0.0,
            deltav: 0.0,
            end_mass: 0.0,
            isp: 0.0,
            start_mass: vessel.mass,
            thrust: if self.dv_linear_thrust {
                vessel.thrust_magnitude
            } else {
                vessel.thrust_no_cos_loss
            },
            spool_up_time: vessel.spoolup_current,
            max_rcs_deltav: 0.0,
            min_rcs_deltav: 0.0,
            rcs_isp: 0.0,
            rcs_delta_time: 0.0,
            rcs_start_tmr: 0.0,
            rcs_end_tmr: 0.0,
            rcs_mass: 0.0,
            rcs_thrust: 0.0,
        };
    }

    fn finish_segment(&mut self, vessel: &mut SimVessel) {
        let start_mass = self.current_segment.start_mass;
        let thrust = self.current_segment.thrust;
        let end_mass = vessel.mass;
        let delta_time = self.time - self.current_segment.start_time;
        let delta_v = if start_mass > end_mass {
            thrust * delta_time / (start_mass - end_mass) * libm::log(start_mass / end_mass)
        } else {
            0.0
        };
        let isp = if start_mass > end_mass {
            delta_v / (G0 * libm::log(start_mass / end_mass))
        } else {
            0.0
        };

        self.current_segment.delta_time = delta_time;
        self.current_segment.end_mass = end_mass;
        self.current_segment.deltav = delta_v;
        self.current_segment.isp = isp;
    }
}

#[test]
fn foo() {
    let mut ffs = FuelFlowSimulation {
        segments: vec![],
        current_segment: FuelStats::default(),
        time: 0.0,
        dv_linear_thrust: true,
        parts_with_resource_drains: vec![],
        sources: vec![],
    };

    let mut vessel = SimVessel {
        parts: Arena::default(),
        active_engines: vec![],
        mass: 0.0,
        thrust_current: Vector3::zeros(),
        thrust_magnitude: 0.0,
        thrust_no_cos_loss: 0.0,
        spoolup_current: 0.0,
        conditions: Conditions {
            atm_pressure: 0.0,
            atm_density: 0.0,
            mach_number: 0.0,
            main_throttle: 1.0,
        },
    };

    let mk1_pod = vessel.parts.push(SimPart {
        crossfeed_part_set: vec![],
        resources: HashMap::new(),
        resource_drains: HashMap::new(),
        resource_priority: -10,
        resource_request_remaining_threshold: 0.0,
        mass: 0.706,
        dry_mass: 0.706,
        crew_mass: 0.0,
        modules_current_mass: 0.0,
        disabled_resource_mass: 0.0,
        is_launch_clamp: false,
    });

    vessel.parts[mk1_pod].resources.insert(
        ResourceId(1566956177),
        Resource {
            free: true,
            max_amount: 50.0,
            amount: 50.0,
            density: 0.0,
            residual: 0.0,
        },
    );
    vessel.parts[mk1_pod].resources.insert(
        ResourceId(-929029996),
        Resource {
            free: false,
            max_amount: 10.0,
            amount: 10.0,
            density: 0.004,
            residual: 0.0,
        },
    );

    let fuel_tank = vessel.parts.push(SimPart {
        crossfeed_part_set: vec![],
        resources: HashMap::new(),
        resource_drains: HashMap::new(),
        resource_priority: -10,
        resource_request_remaining_threshold: 0.0,
        mass: 0.125,
        dry_mass: 0.125,
        crew_mass: 0.0,
        modules_current_mass: 0.0,
        disabled_resource_mass: 0.0,
        is_launch_clamp: false,
    });

    let ox = ResourceId(-1154601244);
    vessel.parts[fuel_tank].resources.insert(
        ox,
        Resource {
            free: false,
            max_amount: 110.0,
            amount: 110.0,
            density: 0.005,
            residual: 0.0,
        },
    );
    let lf = ResourceId(-1483389306);
    vessel.parts[fuel_tank].resources.insert(
        lf, // LF
        Resource {
            free: false,
            max_amount: 90.0,
            amount: 90.0,
            density: 0.005,
            residual: 0.0,
        },
    );

    let engine = vessel.parts.push(SimPart {
        crossfeed_part_set: vec![],
        resources: HashMap::new(),
        resource_drains: HashMap::new(),
        resource_priority: 0,
        resource_request_remaining_threshold: 0.0,
        mass: 1.5,
        dry_mass: 1.5,
        crew_mass: 0.0,
        modules_current_mass: 0.0,
        disabled_resource_mass: 0.0,
        is_launch_clamp: false,
    });

    vessel.parts[mk1_pod].crossfeed_part_set.push(mk1_pod);
    vessel.parts[mk1_pod].crossfeed_part_set.push(fuel_tank);
    vessel.parts[mk1_pod].crossfeed_part_set.push(engine);

    vessel.parts[fuel_tank].crossfeed_part_set.push(mk1_pod);
    vessel.parts[fuel_tank].crossfeed_part_set.push(fuel_tank);
    vessel.parts[fuel_tank].crossfeed_part_set.push(engine);

    vessel.parts[engine].crossfeed_part_set.push(mk1_pod);
    vessel.parts[engine].crossfeed_part_set.push(fuel_tank);
    vessel.parts[engine].crossfeed_part_set.push(engine);

    vessel.active_engines.push(Engine {
        propellants: HashMap::new(),
        propellant_flow_modes: HashMap::new(),
        resource_consumptions: HashMap::new(),
        thrust_transform_multipliers: vec![1.0],
        thrust_direction_vectors: vec![Vector3::new(0.0, 0.0, 1.0)],
        is_operational: true,
        flow_multiplier: 1.0,
        thrust_current: Vector3::zeros(),
        thrust_max: Vector3::zeros(),
        thrust_min: Vector3::zeros(),
        mass_flow_rate: 0.0,
        isp: 0.0,
        g: 9.8067,
        max_fuel_flow: 0.0685,
        max_thrust: 215.0,
        min_fuel_flow: 0.0,
        min_thrust: 0.0,
        mult_isp: 1.0,
        clamp: 0.0,
        flow_mult_cap: 340282300000000000000000000000000000000.0,
        flow_mult_cap_sharpness: 2.0,
        throttle_locked: false,
        throttle_limiter: 100.0,
        atm_change_flow: false,
        use_atm_curve: false,
        use_atm_curve_isp: false,
        use_throttle_isp_curve: false,
        use_vel_curve: false,
        use_vel_curve_isp: false,
        module_residuals: 0.0,
        module_spoolup_time: 0.0,
        no_propellants: false,
        is_unrestartable_dead_engine: false,
        throttle_isp_curve: H1::default(),
        throttle_isp_curve_atm_strength: H1::default(),
        vel_curve: H1::default(),
        vel_curve_isp: H1::default(),
        atm_curve: H1::default(),
        atm_curve_isp: H1::default(),
        atmosphere_curve: H1::default(),
        is_sepratron: false,
        part: engine,
    });

    vessel.active_engines[0]
        .atmosphere_curve
        .add_with_tangents(0.0, 320.0, -70.0, -70.0);
    vessel.active_engines[0]
        .atmosphere_curve
        .add_with_tangents(1.0, 250.0, -59.9999, -59.9999);
    vessel.active_engines[0]
        .atmosphere_curve
        .add_with_tangents(6.0, 0.001, -49.9998, -49.9998);

    vessel.active_engines[0].propellants.insert(
        lf,
        Propellant {
            ignore_for_isp: false,
            ratio: 0.9,
            flow_mode: FlowMode::StackPrioritySearch,
            density: 0.005,
        },
    );

    vessel.active_engines[0].propellants.insert(
        ox,
        Propellant {
            ignore_for_isp: false,
            ratio: 1.1,
            flow_mode: FlowMode::StackPrioritySearch,
            density: 0.005,
        },
    );

    ffs.run(&mut vessel);

    println!("{:#?}", ffs);
    panic!();
}
