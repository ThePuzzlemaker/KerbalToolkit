//! This code is mercilessly ~~stolen~~ ported from MechJeb's
//! FuelFlowSimulation. MechJeb's FuelFlowSimulation is licensed under
//! MIT, individually from the rest of the project.
//!
//! TODO: links
//! TODO: use pools/arenas?
//! TODO: do this in a less leaky way

use std::{
    cell::RefCell,
    cmp,
    collections::HashMap,
    fmt,
    rc::{Rc, Weak},
};

use nalgebra::Vector3;
use ordered_float::OrderedFloat;

use crate::misc::SortedList;

#[derive(Clone)]
pub enum SimPartModule {
    SimModuleEngines(Rc<RefCell<SimModuleEngines>>),
    SimModuleRCS(Rc<RefCell<SimModuleRCS>>),
    SimLaunchClamp(Rc<RefCell<SimLaunchClamp>>),
    SimModuleDecouple(Rc<RefCell<SimModuleDecouple>>),
    SimModuleDockingNode(Rc<RefCell<SimModuleDockingNode>>),
    SimProceduralFairingDecoupler(Rc<RefCell<SimProceduralFairingDecoupler>>),
}

pub struct SimPart {
    pub modules: Vec<SimPartModule>,
    pub crossfeed_part_set: Vec<Weak<RefCell<SimPart>>>,
    pub links: Vec<Weak<RefCell<SimPart>>>,
    pub resources: HashMap<i32, SimResource>,
    resource_drains: HashMap<i32, f64>,
    rcs_drains: HashMap<i32, f64>,

    pub decoupled_in_stage: i32,
    pub staging_on: bool,
    pub inverse_stage: i32,
    pub vessel: Weak<RefCell<SimVessel>>,
    pub name: String,

    pub activates_even_if_disconnected: bool,
    pub is_throttle_locked: bool,
    pub resource_priority: i32,
    pub resource_request_remaining_threshold: f64,
    pub is_enabled: bool,

    pub mass: f64,
    pub dry_mass: f64,
    pub crew_mass: f64,
    pub modules_staged_mass: f64,
    pub modules_unstaged_mass: f64,
    pub disabled_resource_mass: f64,
    pub engine_residuals: f64,

    pub is_root: bool,
    pub is_launch_clamp: bool,
    pub is_engine: bool,

    resource_keys: Vec<i32>,
}

impl SimPart {
    pub fn is_sepratron(&self) -> bool {
        self.is_engine
            && self.is_throttle_locked
            && self.activates_even_if_disconnected
            && self.inverse_stage == self.decoupled_in_stage
    }

    pub fn update_mass(&mut self) {
        if self.is_launch_clamp {
            self.mass = 0.0;
            return;
        }

        self.mass = self.dry_mass + self.crew_mass + self.disabled_resource_mass;
        self.mass += if self.vessel.upgrade().unwrap().borrow().current_stage <= self.inverse_stage
        {
            self.modules_staged_mass
        } else {
            self.modules_unstaged_mass
        };
        for resource in self.resources.values() {
            self.mass += resource.amount * resource.density;
        }
    }

    pub fn try_get_resource(&self, resource_id: i32) -> Option<&SimResource> {
        self.resources.get(&resource_id)
    }

    pub fn apply_resource_drain(&mut self, dt: f64) {
        for (id, resource_drain) in &self.resource_drains {
            self.resources
                .get_mut(id)
                .unwrap()
                .drain(dt * *resource_drain);
        }
    }

    pub fn apply_rcs_drain(&mut self, dt: f64) {
        for (id, rcs_drain) in &self.rcs_drains {
            self.resources
                .get_mut(id)
                .unwrap()
                .rcs_drain(dt * *rcs_drain);
        }
    }

    pub fn unapply_rcs_drains(&mut self) {
        self.resource_keys.clear();
        for &id in self.resources.keys() {
            self.resource_keys.push(id);
        }

        for id in &self.resource_keys {
            self.resources.get_mut(id).unwrap().reset_rcs();
        }
    }

    pub fn update_resource_residual(&mut self, residual: f64, resource_id: i32) {
        if !self.resources.contains_key(&resource_id) {
            return;
        }

        let resource = self.resources.get_mut(&resource_id).unwrap();
        resource.residual = cmp::max(OrderedFloat(resource.residual), OrderedFloat(residual)).0;
    }

    pub fn clear_residuals(&mut self) {
        self.resource_keys.clear();
        for &id in self.resources.keys() {
            self.resource_keys.push(id);
        }

        for id in &self.resource_keys {
            let resource = self.resources.get_mut(id).unwrap();
            resource.residual = 0.0;
        }
    }

    pub fn residual_threshold(&self, resource_id: i32) -> f64 {
        self.resources[&resource_id].residual_threshold()
            + self.resource_request_remaining_threshold
    }

    pub fn clear_resource_drains(&mut self) {
        self.resource_drains.clear();
    }

    pub fn clear_rcs_drains(&mut self) {
        self.rcs_drains.clear();
    }

    pub fn add_resource_drain(&mut self, resource_id: i32, resource_consumption: f64) {
        self.resource_drains
            .entry(resource_id)
            .and_modify(|x| *x += resource_consumption)
            .or_insert(resource_consumption);
    }

    pub fn add_rcs_drain(&mut self, resource_id: i32, resource_consumption: f64) {
        self.rcs_drains
            .entry(resource_id)
            .and_modify(|x| *x += resource_consumption)
            .or_insert(resource_consumption);
    }

    pub fn resource_max_time(&self) -> f64 {
        let mut max_time = f64::MAX;

        for resource in self.resources.values() {
            if resource.free {
                continue;
            }

            if resource.amount <= self.resource_request_remaining_threshold {
                continue;
            }

            if let Some(resource_drain) = self.resource_drains.get(&resource.id) {
                let dt = (resource.amount - resource.residual_threshold()) / resource_drain;
                max_time = cmp::min(OrderedFloat(max_time), OrderedFloat(dt)).0;
            }
        }

        max_time
    }

    pub fn rcs_max_time(&self) -> f64 {
        let mut max_time = f64::MAX;

        for resource in self.resources.values() {
            if resource.free {
                continue;
            }

            if resource.amount <= self.resource_request_remaining_threshold {
                continue;
            }

            if let Some(resource_drain) = self.rcs_drains.get(&resource.id) {
                let dt = (resource.amount - resource.residual_threshold()) / resource_drain;
                max_time = cmp::min(OrderedFloat(max_time), OrderedFloat(dt)).0;
            }
        }

        max_time
    }
}

impl fmt::Display for SimPart {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "{}: ", self.name)?;
        write!(f, "  Neighbors:")?;
        for link in &self.links {
            write!(f, " {}", link.upgrade().unwrap().borrow().name)?;
        }
        writeln!(f)?;
        write!(f, "  Resources:")?;
        for resource in self.resources.values() {
            write!(
                f,
                " {}={}*{}",
                resource.id, resource.amount, resource.density
            )?;
        }
        writeln!(f)?;
        write!(
            f,
            "  DecoupledInStage: {} InverseStage: {}",
            self.decoupled_in_stage, self.inverse_stage
        )?;
        writeln!(f)
    }
}

#[derive(Default)]
pub struct SimResource {
    pub free: bool,
    pub max_amount: f64,
    amount: f64,
    rcs_amount: f64,
    pub id: i32,
    pub density: f64,
    pub residual: f64,
}

impl SimResource {
    pub fn get_amount(&self) -> f64 {
        self.amount + self.rcs_amount
    }

    pub fn set_amount(&mut self, amount: f64) {
        self.amount = amount;
    }

    pub fn residual_threshold(&self) -> f64 {
        self.residual * self.max_amount
    }

    pub fn drain(&mut self, resource_drain: f64) -> &mut Self {
        self.amount -= resource_drain;
        if self.amount < 0.0 {
            self.amount = 0.0;
        }
        self
    }

    pub fn rcs_drain(&mut self, rcs_drain: f64) -> &mut Self {
        self.rcs_amount -= rcs_drain;
        if self.get_amount() < 0.0 {
            self.rcs_amount = -self.amount;
        }
        self
    }

    pub fn reset_rcs(&mut self) -> &mut Self {
        self.rcs_amount = 0.0;
        self
    }
}

pub struct SimPropellant {
    pub id: i32,
    pub ignore_for_isp: bool,
    pub ratio: f64,
    pub flow_mode: SimFlowMode,
    pub density: f64,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SimFlowMode {
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

pub struct SimVessel {
    pub parts: Vec<Rc<RefCell<SimPart>>>,
    pub parts_remaining_in_stage: HashMap<i32, Vec<Rc<RefCell<SimPart>>>>,
    pub engines_dropped_in_stage: HashMap<i32, Vec<Rc<RefCell<SimModuleEngines>>>>,
    pub engines_activated_in_stage: HashMap<i32, Vec<Rc<RefCell<SimModuleEngines>>>>,
    pub rcs_activated_in_stage: HashMap<i32, Vec<Rc<RefCell<SimModuleRCS>>>>,
    pub rcs_dropped_in_stage: HashMap<i32, Vec<Rc<RefCell<SimModuleRCS>>>>,
    pub active_engines: Vec<Rc<RefCell<SimModuleEngines>>>,
    pub active_rcs: Vec<Rc<RefCell<SimModuleRCS>>>,

    pub has_launch_clamp: bool,
    pub current_stage: i32,
    // TODO: default=1.0
    pub main_throttle: f64,
    pub mass: f64,
    pub thrust_current: Vector3<f64>,
    pub rcs_thrust: f64,
    pub thrust_magnitude: f64,
    pub thrust_no_cos_loss: f64,
    pub spoolup_current: f64,
    pub atm_pressure: f64,
    pub atm_density: f64,
    pub mach_number: f64,
    pub t: f64,
    pub r: Vector3<f64>,
    pub v: Vector3<f64>,
    pub u: Vector3<f64>,
}

impl SimVessel {
    pub fn set_conditions(&mut self, atm_density: f64, atm_pressure: f64, mach_number: f64) {
        self.atm_density = atm_density;
        self.atm_pressure = atm_pressure;
        self.mach_number = mach_number;
    }

    pub fn set_initial(&mut self, t: f64, r: Vector3<f64>, v: Vector3<f64>, u: Vector3<f64>) {
        self.t = t;
        self.r = r;
        self.v = v;
        self.u = u;
    }

    pub fn update_mass(&mut self) {
        self.mass = 0.0;

        for part in &self.parts_remaining_in_stage[&self.current_stage] {
            let mut part = part.borrow_mut();
            part.update_mass();
            self.mass += part.mass;
        }
    }

    pub fn stage(&mut self) {
        if self.current_stage < 0 {
            return;
        }

        self.current_stage -= 1;

        self.activate_engines_and_rcs();
        self.update_mass();
    }

    pub fn activate_engines_and_rcs(&mut self) {
        for engine in &self.engines_activated_in_stage[&self.current_stage] {
            let mut engine = engine.borrow_mut();
            if engine.is_enabled {
                engine.activate();
            }
        }

        for rcs in &self.rcs_activated_in_stage[&self.current_stage] {
            let mut rcs = rcs.borrow_mut();
            if rcs.is_enabled {
                rcs.activate();
            }
        }
    }

    pub fn update_active_engines(&mut self) {
        self.active_engines.clear();

        // FIXME: why am i not iterating over the last ActiveEngines?
        for i in -1..self.current_stage {
            for engine_rc in &self.engines_dropped_in_stage[&i] {
                let mut engine = engine_rc.borrow_mut();
                if engine.mass_flow_rate <= 0.0 {
                    continue;
                }
                if engine.is_unrestartable_dead_engine {
                    continue;
                }
                engine.update_engine_status();
                if !engine.is_operational {
                    continue;
                }
                self.active_engines.push(engine_rc.clone());
            }
        }

        self.compute_thrust_and_spoolup();
    }

    pub fn update_active_rcs(&mut self) {
        self.active_rcs.clear();

        // FIXME: why am i not iterating over the last ActiveRcs?
        for i in -1..self.current_stage {
            for rcs_rc in &self.rcs_dropped_in_stage[&i] {
                let mut rcs = rcs_rc.borrow_mut();
                if rcs.mass_flow_rate <= 0.0 {
                    continue;
                }
                rcs.update_rcs_status();
                if !rcs.rcs_enabled {
                    continue;
                }
                self.active_rcs.push(rcs_rc.clone());
            }
        }

        self.compute_rcs_thrust();
    }

    pub fn compute_rcs_thrust(&mut self) {
        self.rcs_thrust = 0.0;

        for rcs in &self.active_rcs {
            let mut rcs = rcs.borrow_mut();
            rcs.update();
            self.rcs_thrust += rcs.thrust;
        }
    }

    pub fn compute_thrust_and_spoolup(&mut self) {
        self.thrust_current = Vector3::zeros();
        self.thrust_magnitude = 0.0;
        self.thrust_no_cos_loss = 0.0;
        self.spoolup_current = 0.0;

        for engine in &self.active_engines {
            let mut engine = engine.borrow_mut();
            if !engine.is_operational {
                continue;
            }

            self.spoolup_current += engine.thrust_current.norm() * engine.module_spoolup_time;

            engine.update();
            self.thrust_current += engine.thrust_current;
            self.thrust_no_cos_loss += engine.thrust_current.norm();
        }

        self.thrust_magnitude = self.thrust_current.norm();
        self.spoolup_current /= self.thrust_current.norm();
    }

    pub fn update_rcs_stats(&mut self) {
        for i in -1..self.current_stage {
            for rcs in &self.rcs_dropped_in_stage[&i] {
                rcs.borrow_mut().update();
            }
        }
    }

    pub fn update_engine_stats(&mut self) {
        for i in -1..self.current_stage {
            for engine in &self.engines_dropped_in_stage[&i] {
                engine.borrow_mut().update();
            }
        }
    }

    pub fn save_rcs_status(&mut self) {
        for i in -1..self.current_stage {
            for rcs in &self.rcs_dropped_in_stage[&i] {
                rcs.borrow_mut().save_status();
            }
        }
    }

    pub fn reset_rcs_status(&mut self) {
        for i in -1..self.current_stage {
            for rcs in &self.rcs_dropped_in_stage[&i] {
                rcs.borrow_mut().reset_status();
            }
        }
    }
}

impl fmt::Display for SimVessel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for i in 0..=self.current_stage {
            writeln!(f, "stage {i}:")?;
            for part in &self.parts_remaining_in_stage[&i] {
                writeln!(f, "{}", part.borrow())?;
            }
            writeln!(f, "------------------------")?;
        }
        Ok(())
    }
}

pub struct SimModuleEngines {
    pub propellants: Vec<SimPropellant>,
    pub propellant_flow_modes: HashMap<i32, SimFlowMode>,
    pub resource_consumptions: HashMap<i32, f64>,
    pub thrust_transform_multipliers: Vec<f64>,
    pub thrust_direction_vectors: Vec<Vector3<f64>>,

    pub is_operational: bool,
    pub flow_multiplier: f64,
    pub thrust_current: Vector3<f64>,
    pub thrust_max: Vector3<f64>,
    pub thrust_min: Vector3<f64>,
    pub mass_flow_rate: f64,
    pub isp: f64,
    pub g: f64,
    pub max_fuel_flow: f64,
    pub max_thrust: f64,
    pub min_fuel_flow: f64,
    pub min_thrust: f64,
    pub mult_isp: f64,
    pub clamp: f64,
    pub flow_mult_cap: f64,
    pub flow_mult_cap_sharpness: f64,
    pub throttle_locked: bool,
    pub throttle_limiter: f64,
    pub atm_change_flow: bool,
    pub use_atm_curve: bool,
    pub use_atm_curve_isp: bool,
    pub use_throttle_isp_curve: bool,
    pub use_thrust_curve: bool,
    pub use_vel_curve: bool,
    pub use_vel_curve_isp: bool,
    pub module_residuals: f64,
    pub module_spoolup_time: f64,
    pub no_propellants: bool,
    pub is_module_engines_rf: bool,
    pub is_unrestartable_dead_engine: bool,

    pub thrust_curve: H1,
    pub throttle_isp_curve: H1,
    pub throttle_isp_curve_atm_strength: H1,
    pub vel_curve: H1,
    pub vel_curve_isp: H1,
    pub atm_curve: H1,
    pub atm_curve_isp: H1,
    pub atmosphere_curve: H1,

    pub is_enabled: bool,
    pub part: Weak<RefCell<SimPart>>,
    pub module_is_enabled: bool,
    pub staging_enabled: bool,
}

impl SimModuleEngines {
    fn part(&self) -> Rc<RefCell<SimPart>> {
        self.part.upgrade().unwrap()
    }

    fn throttle(&self) -> f64 {
        self.part()
            .borrow_mut()
            .vessel
            .upgrade()
            .unwrap()
            .borrow()
            .main_throttle
    }
    fn atm_pressure(&self) -> f64 {
        self.part()
            .borrow_mut()
            .vessel
            .upgrade()
            .unwrap()
            .borrow()
            .atm_pressure
    }
    fn atm_density(&self) -> f64 {
        self.part()
            .borrow_mut()
            .vessel
            .upgrade()
            .unwrap()
            .borrow()
            .atm_density
    }
    fn mach_number(&self) -> f64 {
        self.part()
            .borrow_mut()
            .vessel
            .upgrade()
            .unwrap()
            .borrow()
            .mach_number
    }
    pub fn activate(&mut self) {
        self.is_operational = true;
    }
    pub fn update_engine_status(&mut self) {
        if self.can_draw_resources() {
            return;
        }
        self.is_operational = false;
    }
    pub fn can_draw_resources(&self) -> bool {
        if self.no_propellants {
            return false;
        }

        for resource_id in self.resource_consumptions.keys() {
            match self.propellant_flow_modes[resource_id] {
                SimFlowMode::NoFlow => {
                    if !self.part_has_resource(&self.part().borrow(), *resource_id) {
                        return false;
                    }
                }
                SimFlowMode::AllVessel
                | SimFlowMode::AllVesselBalance
                | SimFlowMode::StagePriorityFlow
                | SimFlowMode::StagePriorityFlowBalance => {
                    if !self.parts_have_resource(
                        self.part()
                            .borrow()
                            .vessel
                            .upgrade()
                            .unwrap()
                            .borrow()
                            .parts
                            .iter()
                            .cloned(),
                        *resource_id,
                    ) {
                        return false;
                    }
                }
                SimFlowMode::StackPrioritySearch
                | SimFlowMode::StageStackFlow
                | SimFlowMode::StageStackFlowBalance => {
                    if !self.parts_have_resource(
                        self.part()
                            .borrow()
                            .crossfeed_part_set
                            .iter()
                            .map(|x| x.upgrade().unwrap()),
                        *resource_id,
                    ) {
                        return false;
                    }
                }
                SimFlowMode::Null => return false,
            }
        }

        true
    }

    fn part_has_resource(&self, part: &SimPart, resource_id: i32) -> bool {
        if let Some(resource) = part.try_get_resource(resource_id) {
            resource.amount
                > part.resources[&resource_id].max_amount * self.module_residuals
                    + part.resource_request_remaining_threshold
        } else {
            false
        }
    }

    fn parts_have_resource(
        &self,
        parts: impl Iterator<Item = Rc<RefCell<SimPart>>>,
        resource_id: i32,
    ) -> bool {
        for part in parts {
            if self.part_has_resource(&part.borrow(), resource_id) {
                return true;
            }
        }
        false
    }

    fn drawing_fuel_from_part_dropped_in_stage(
        &self,
        part: &SimPart,
        resource_id: i32,
        stage_num: i32,
    ) -> bool {
        self.part_has_resource(part, resource_id) && part.decoupled_in_stage == stage_num
    }

    fn drawing_fuel_from_parts_dropped_in_stage(
        &self,
        parts: impl Iterator<Item = Rc<RefCell<SimPart>>>,
        resource_id: i32,
        stage_num: i32,
    ) -> bool {
        for part in parts {
            let part = part.borrow();
            if self.part_has_resource(&part, resource_id) && part.decoupled_in_stage == stage_num {
                return true;
            }
        }
        false
    }

    pub fn would_drop_accessible_fuel_tank(&self, stage_num: i32) -> bool {
        for resource_id in self.resource_consumptions.keys() {
            match self.propellant_flow_modes[resource_id] {
                SimFlowMode::NoFlow => {
                    if self.drawing_fuel_from_part_dropped_in_stage(
                        &self.part().borrow(),
                        *resource_id,
                        stage_num,
                    ) {
                        return true;
                    }
                }
                SimFlowMode::AllVessel
                | SimFlowMode::AllVesselBalance
                | SimFlowMode::StagePriorityFlow
                | SimFlowMode::StagePriorityFlowBalance => {
                    if self.drawing_fuel_from_parts_dropped_in_stage(
                        self.part()
                            .borrow()
                            .vessel
                            .upgrade()
                            .unwrap()
                            .borrow()
                            .parts
                            .iter()
                            .cloned(),
                        *resource_id,
                        stage_num,
                    ) {
                        return true;
                    }
                }
                SimFlowMode::StageStackFlow
                | SimFlowMode::StageStackFlowBalance
                | SimFlowMode::StackPrioritySearch => {
                    if self.drawing_fuel_from_parts_dropped_in_stage(
                        self.part()
                            .borrow()
                            .crossfeed_part_set
                            .iter()
                            .map(|x| x.upgrade().unwrap()),
                        *resource_id,
                        stage_num,
                    ) {
                        return true;
                    }
                }
                _ => return false,
            }
        }

        false
    }

    pub fn update(&mut self) {
        self.isp = self.isp_at_conditions();
        self.flow_multiplier = self.flow_multiplier_at_conditions();
        self.mass_flow_rate = self.flow_rate_at_conditions();
        self.refresh_thrust();
        self.set_consumption_rates();
    }

    fn flow_rate_at_conditions(&mut self) -> f64 {
        let mut min_fuel_flow = self.min_fuel_flow;
        let mut max_fuel_flow = self.max_fuel_flow;

        if min_fuel_flow == 0.0 && self.min_thrust > 0.0 {
            min_fuel_flow = self.min_thrust / (self.atmosphere_curve.evaluate(0.0) * self.g);
        }
        if max_fuel_flow == 0.0 && self.max_thrust > 0.0 {
            max_fuel_flow = self.max_thrust / (self.atmosphere_curve.evaluate(0.0) * self.g);
        }

        lerp(
            min_fuel_flow,
            max_fuel_flow,
            self.throttle() * 0.01 * self.throttle_limiter,
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

    fn flow_multiplier_at_conditions(&mut self) -> f64 {
        let mut flow_multiplier = 1.0;

        if self.atm_change_flow {
            if self.use_atm_curve {
                flow_multiplier = self.atm_curve.evaluate(self.atm_density() * 40.0 / 49.0);
            } else {
                flow_multiplier = self.atm_density() * 40.0 / 49.0;
            }
        }

        if self.use_vel_curve {
            flow_multiplier *= self.vel_curve.evaluate(self.mach_number());
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

    fn isp_at_conditions(&mut self) -> f64 {
        let mut isp = self.atmosphere_curve.evaluate(self.atm_pressure());
        if self.use_throttle_isp_curve {
            isp *= lerp(
                1.0,
                self.throttle_isp_curve.evaluate(self.throttle()),
                self.throttle_isp_curve_atm_strength
                    .evaluate(self.atm_pressure()),
            );
        }
        if self.use_atm_curve_isp {
            isp *= self
                .atm_curve_isp
                .evaluate(self.atm_density() * 40.0 / 49.0);
        }
        if self.use_vel_curve_isp {
            isp *= self.vel_curve_isp.evaluate(self.mach_number());
        }
        isp
    }

    fn set_consumption_rates(&mut self) {
        self.resource_consumptions.clear();
        self.propellant_flow_modes.clear();

        let mut total_density = 0.0;

        for propellant in &self.propellants {
            let density = propellant.density;

            if density <= 0.0 {
                continue;
            }

            self.propellant_flow_modes
                .insert(propellant.id, propellant.flow_mode);

            if propellant.ignore_for_isp {
                continue;
            }

            total_density += propellant.ratio * density;
        }

        let volume_flow_rate = self.mass_flow_rate / total_density;

        for propellant in &self.propellants {
            let density = propellant.density;

            let prop_volume_rate = propellant.ratio * volume_flow_rate;

            if density <= 0.0 {
                continue;
            }

            self.resource_consumptions
                .entry(propellant.id)
                .and_modify(|x| *x += prop_volume_rate)
                .or_insert(prop_volume_rate);
        }
    }
}

fn lerp(x: f64, y: f64, t: f64) -> f64 {
    x + t * (y - x)
}

pub struct H1 {
    min_time: f64,
    max_time: f64,
    last_lo: i32,
    list: SortedList<OrderedFloat<f64>, HFrame<f64>>,
}

impl Default for H1 {
    fn default() -> Self {
        Self::new()
    }
}

impl H1 {
    pub fn new() -> Self {
        Self {
            min_time: f64::MAX,
            max_time: f64::MIN,
            last_lo: -1,
            list: SortedList::new(),
        }
    }
}

impl HBase<f64> for H1 {
    #[inline]
    fn min_time(&self) -> f64 {
        self.min_time
    }

    #[inline]
    fn max_time(&self) -> f64 {
        self.max_time
    }

    #[inline]
    fn set_min_time(&mut self, x: f64) {
        self.min_time = x;
    }

    #[inline]
    fn set_max_time(&mut self, x: f64) {
        self.max_time = x;
    }

    #[inline]
    fn last_lo(&self) -> i32 {
        self.last_lo
    }

    #[inline]
    fn set_last_lo(&mut self, x: i32) {
        self.last_lo = x
    }

    #[inline]
    fn list(&self) -> &SortedList<OrderedFloat<f64>, HFrame<f64>> {
        &self.list
    }

    #[inline]
    fn list_mut(&mut self) -> &mut SortedList<OrderedFloat<f64>, HFrame<f64>> {
        &mut self.list
    }

    #[inline]
    fn allocate_zero() -> f64 {
        0.0
    }

    #[inline]
    fn alloc_f64(x: f64) -> f64 {
        x
    }

    #[inline]
    fn subtract(&mut self, a: f64, b: f64) -> f64 {
        a - b
    }

    #[inline]
    fn divide(&mut self, a: f64, b: f64) -> f64 {
        a / b
    }

    #[inline]
    fn multiply(&mut self, a: f64, b: f64) -> f64 {
        a * b
    }

    #[inline]
    fn addition(&mut self, a: f64, b: f64) -> f64 {
        a + b
    }

    fn interpolant(
        &mut self,
        x1: f64,
        y1: f64,
        yp1: f64,
        x2: f64,
        y2: f64,
        yp2: f64,
        x: f64,
    ) -> f64 {
        let t = (x - x1) / (x2 - x1);
        let t2 = t * t;
        let t3 = t2 * t;
        let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        let h10 = t3 - 2.0 * t2 + t;
        let h01 = -2.0 * t3 + 3.0 * t2;
        let h11 = t3 - t2;
        h00 * y1 + h10 * (x2 - x1) * yp1 + h01 * y2 + h11 * (x2 - x1) * yp2
    }
}

pub trait HBase<T: Sized + PartialEq + Copy + Clone> {
    fn min_time(&self) -> f64;
    fn max_time(&self) -> f64;

    fn set_min_time(&mut self, x: f64);
    fn set_max_time(&mut self, x: f64);

    fn last_lo(&self) -> i32;
    fn set_last_lo(&mut self, x: i32);

    fn list(&self) -> &SortedList<OrderedFloat<f64>, HFrame<T>>;
    fn list_mut(&mut self) -> &mut SortedList<OrderedFloat<f64>, HFrame<T>>;

    fn allocate_zero() -> T;
    fn alloc_f64(x: f64) -> T;

    fn subtract(&mut self, a: T, b: T) -> T;
    fn divide(&mut self, a: T, b: T) -> T;
    fn multiply(&mut self, a: T, b: T) -> T;
    fn addition(&mut self, a: T, b: T) -> T;

    #[allow(clippy::too_many_arguments)]
    fn interpolant(&mut self, x1: f64, y1: T, yp1: T, x2: f64, y2: T, yp2: T, x: f64) -> T;

    fn add(&mut self, time: f64, value: T) {
        self.list_mut().insert(
            OrderedFloat(time),
            HFrame {
                in_tangent: Self::allocate_zero(),
                out_tangent: Self::allocate_zero(),
                time,
                value,
                auto_tangent: true,
            },
        );
        self.set_min_time(cmp::min(OrderedFloat(self.min_time()), OrderedFloat(time)).0);
        self.set_max_time(cmp::max(OrderedFloat(self.max_time()), OrderedFloat(time)).0);
        self.recompute_tangents(
            self.list()
                .find_first_position(&OrderedFloat(time))
                .unwrap() as i32,
        );
        self.set_last_lo(-1);
    }

    fn add_with_tangents(&mut self, time: f64, value: T, in_tangent: T, out_tangent: T) {
        self.list_mut().insert(
            OrderedFloat(time),
            HFrame {
                in_tangent,
                out_tangent,
                time,
                value,
                auto_tangent: true,
            },
        );
        self.set_min_time(cmp::min(OrderedFloat(self.min_time()), OrderedFloat(time)).0);
        self.set_max_time(cmp::max(OrderedFloat(self.max_time()), OrderedFloat(time)).0);
        self.recompute_tangents(
            self.list()
                .find_first_position(&OrderedFloat(time))
                .unwrap() as i32,
        );
        self.set_last_lo(-1);
    }

    fn add_with_tangent(&mut self, time: f64, value: T, tangent: T) {
        if let Some(frame) = self.list_mut().first_value_of_mut(&OrderedFloat(time)) {
            frame.value = value;
            frame.out_tangent = tangent;
        } else {
            self.add_with_tangents(time, value, tangent, tangent)
        }
    }

    fn find_index(&mut self, value: f64) -> i32 {
        assert!(self.list().len() > 1);
        assert!(value > self.min_time());
        assert!(value < self.max_time());

        if self.last_lo() > 0 && value > *self.list().keys[self.last_lo() as usize] {
            if value < *self.list().keys[(self.last_lo() + 1) as usize] {
                return !(self.last_lo() + 1);
            }

            if value == *self.list().keys[(self.last_lo() + 1) as usize] {
                self.set_last_lo(self.last_lo() + 1);
                return self.last_lo();
            }

            if value > *self.list().keys[(self.last_lo() + 1) as usize]
                && value < *self.list().keys[(self.last_lo() + 2) as usize]
            {
                self.set_last_lo(self.last_lo() + 1);
                return !(self.last_lo() + 1);
            }
        }

        let mut lo = 0;
        let mut hi = self.list().len() - 1;

        while lo <= hi {
            let i = lo + ((hi - lo) >> 1);
            let order = value.partial_cmp(&self.list().keys[i].0).unwrap();
            match order {
                cmp::Ordering::Less => {
                    hi = i - 1;
                }
                cmp::Ordering::Equal => {
                    self.set_last_lo(i as i32);
                    return i as i32;
                }
                cmp::Ordering::Greater => {
                    lo = i + 1;
                }
            }
        }

        self.set_last_lo(lo as i32 - 1);

        !(lo as i32)
    }

    fn recompute_tangents(&mut self, i: i32) {
        if self.list().len() == 1 {
            let temp = &mut self.list_mut().values[0];
            if temp.auto_tangent {
                temp.in_tangent = Self::allocate_zero();
                temp.out_tangent = Self::allocate_zero();
            }
            return;
        }

        self.fix_tangent(i);

        if i != 0 {
            self.fix_tangent(i - 1);
        }

        if i != (self.list().len() - 1) as i32 {
            self.fix_tangent(i + 1);
        }
    }

    fn fix_tangent(&mut self, i: i32) {
        let mut current = self.list().values[i as usize];

        if !current.auto_tangent {
            return;
        }

        let mut slope1 = Self::allocate_zero();

        if i < (self.list().len() - 1) as i32 {
            let right = self.list().values[(i + 1) as usize];

            slope1 = self.subtract(right.value, current.value);
            slope1 = self.divide(slope1, Self::alloc_f64(right.time - current.time));

            if i == 0 {
                current.in_tangent = slope1;
                current.out_tangent = slope1;
                self.list_mut().values[i as usize] = current;
            }
        }

        let mut slope2 = Self::allocate_zero();

        if i > 0 {
            let left = self.list().values[(i - 1) as usize];

            slope2 = self.subtract(current.value, left.value);
            slope2 = self.divide(slope2, Self::alloc_f64(current.time - left.time));

            if i == (self.list().len() - 1) as i32 {
                current.in_tangent = slope2;
                current.out_tangent = slope2;
                self.list_mut().values[i as usize] = current;
            }
        }

        slope1 = self.addition(slope1, slope2);
        slope1 = self.divide(slope1, Self::alloc_f64(2.0));
        current.in_tangent = slope1;
        current.out_tangent = slope1;
        self.list_mut().values[i as usize] = current;
    }

    fn evaluate(&mut self, t: f64) -> T {
        if self.list().is_empty() {
            return Self::allocate_zero();
        }

        if t <= self.min_time() {
            return self.list().values[0].value;
        }

        if t >= self.max_time() {
            return self.list().values[self.list().len() - 1].value;
        }

        let hi = self.find_index(t);

        if hi >= 0 {
            return self.list().values[hi as usize].value;
        }

        let hi = !hi;

        let test_keyframe = self.list().values[(hi - 1) as usize];
        let test_keyframe2 = self.list().values[hi as usize];

        self.interpolant(
            test_keyframe.time,
            test_keyframe.value,
            test_keyframe.out_tangent,
            test_keyframe2.time,
            test_keyframe2.value,
            test_keyframe2.in_tangent,
            t,
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct HFrame<T: PartialEq> {
    pub in_tangent: T,
    pub out_tangent: T,
    pub time: f64,
    pub value: T,
    pub auto_tangent: bool,
}

pub struct SimModuleRCS {
    pub propellants: Vec<Rc<RefCell<SimPropellant>>>,
    pub propellant_flow_modes: HashMap<i32, SimFlowMode>,
    pub resource_consumptions: HashMap<i32, f64>,

    pub atmosphere_curve: H1,

    pub g: f64,
    pub isp: f64,
    pub thrust: f64,
    pub rcs_enabled: bool,
    saved_rcs_enabled: bool,
    pub isp_mult: f64,
    pub thrust_percentage: f64,
    pub max_fuel_flow: f64,
    pub mass_flow_rate: f64,

    pub is_enabled: bool,
    pub part: Weak<RefCell<SimPart>>,
    pub module_is_enabled: bool,
    pub staging_enabled: bool,
}

impl SimModuleRCS {
    fn part(&self) -> Rc<RefCell<SimPart>> {
        self.part.upgrade().unwrap()
    }

    fn atm_pressure(&self) -> f64 {
        self.part()
            .borrow()
            .vessel
            .upgrade()
            .unwrap()
            .borrow()
            .atm_pressure
    }

    pub fn update_rcs_status(&mut self) {
        if self.can_draw_resources() {
            return;
        }

        self.rcs_enabled = false;
    }

    pub fn save_status(&mut self) {
        self.saved_rcs_enabled = self.rcs_enabled;
    }

    pub fn reset_status(&mut self) {
        self.rcs_enabled = self.saved_rcs_enabled;
    }

    pub fn can_draw_resources(&self) -> bool {
        for resource_id in self.resource_consumptions.keys() {
            match self.propellant_flow_modes[resource_id] {
                SimFlowMode::NoFlow => {
                    if !self.part_has_resource(&self.part().borrow(), *resource_id) {
                        return false;
                    }
                }
                SimFlowMode::AllVessel
                | SimFlowMode::AllVesselBalance
                | SimFlowMode::StagePriorityFlow
                | SimFlowMode::StagePriorityFlowBalance => {
                    if !self.parts_have_resource(
                        self.part()
                            .borrow()
                            .vessel
                            .upgrade()
                            .unwrap()
                            .borrow()
                            .parts
                            .iter()
                            .cloned(),
                        *resource_id,
                    ) {
                        return false;
                    }
                }
                SimFlowMode::StackPrioritySearch
                | SimFlowMode::StageStackFlow
                | SimFlowMode::StageStackFlowBalance => {
                    if !self.parts_have_resource(
                        self.part()
                            .borrow()
                            .crossfeed_part_set
                            .iter()
                            .map(|x| x.upgrade().unwrap()),
                        *resource_id,
                    ) {
                        return false;
                    }
                }
                SimFlowMode::Null => return false,
            }
        }

        true
    }

    fn part_has_resource(&self, part: &SimPart, resource_id: i32) -> bool {
        if let Some(resource) = part.try_get_resource(resource_id) {
            resource.amount > part.resource_request_remaining_threshold
        } else {
            false
        }
    }

    fn parts_have_resource(
        &self,
        parts: impl Iterator<Item = Rc<RefCell<SimPart>>>,
        resource_id: i32,
    ) -> bool {
        for part in parts {
            if self.part_has_resource(&part.borrow(), resource_id) {
                return true;
            }
        }
        false
    }

    pub fn update(&mut self) {
        self.isp = self.atmosphere_curve.evaluate(self.atm_pressure());
        let exhaust_vel = self.isp * self.g * self.isp_mult;
        self.mass_flow_rate = self.max_fuel_flow * (self.thrust_percentage * 0.01);
        self.thrust = exhaust_vel * self.mass_flow_rate;
        self.set_consumption_rates();
    }

    pub fn activate(&mut self) {
        self.rcs_enabled = true;
    }
    fn set_consumption_rates(&mut self) {
        self.resource_consumptions.clear();
        self.propellant_flow_modes.clear();

        let mut total_density = 0.0;

        for propellant in &self.propellants {
            let propellant = propellant.borrow();
            let density = propellant.density;

            if density <= 0.0 {
                continue;
            }

            self.propellant_flow_modes
                .insert(propellant.id, propellant.flow_mode);

            if propellant.ignore_for_isp {
                continue;
            }

            total_density += propellant.ratio * density;
        }

        let volume_flow_rate = self.mass_flow_rate / total_density;

        for propellant in &self.propellants {
            let propellant = propellant.borrow();
            let density = propellant.density;

            let prop_volume_rate = propellant.ratio * volume_flow_rate;

            if density <= 0.0 {
                continue;
            }

            self.resource_consumptions
                .entry(propellant.id)
                .and_modify(|x| *x += prop_volume_rate)
                .or_insert(prop_volume_rate);
        }
    }
}

#[derive(Clone)]
pub struct SimLaunchClamp {
    pub is_enabled: bool,
    pub part: Weak<RefCell<SimPart>>,
    pub module_is_enabled: bool,
    pub staging_enabled: bool,
}

#[derive(Clone)]
pub struct SimModuleDecouple {
    pub is_decoupled: bool,
    pub is_omni_decoupler: bool,
    pub staged: bool,
    pub attached_part: Option<Rc<RefCell<SimPart>>>,

    pub is_enabled: bool,
    pub part: Weak<RefCell<SimPart>>,
    pub module_is_enabled: bool,
    pub staging_enabled: bool,
}

#[derive(Clone)]
pub struct SimModuleDockingNode {
    pub staged: bool,
    pub attached_part: Option<Rc<RefCell<SimPart>>>,

    pub is_enabled: bool,
    pub part: Weak<RefCell<SimPart>>,
    pub module_is_enabled: bool,
    pub staging_enabled: bool,
}

#[derive(Clone)]
pub struct SimProceduralFairingDecoupler {
    pub is_decoupled: bool,

    pub is_enabled: bool,
    pub part: Weak<RefCell<SimPart>>,
    pub module_is_enabled: bool,
    pub staging_enabled: bool,
}

#[derive(Clone)]
pub struct FuelStats {
    pub delta_time: f64,
    pub deltav: f64,
    pub end_mass: f64,
    pub isp: f64,
    pub ksp_stage: i32,
    pub staged_mass: f64,
    pub start_mass: f64,
    pub start_time: f64,
    pub thrust: f64,
    pub spool_up_time: f64,
    pub max_rcs_deltav: f64,
    pub min_rcs_deltav: f64,
    pub rcs_isp: f64,
    pub rcs_delta_time: f64,
    pub rcs_thrust: f64,
    pub rcs_mass: f64,
    pub rcs_start_tmr: f64,
    pub rcs_end_tmr: f64,
}

impl FuelStats {
    pub fn max_accel(&self) -> f64 {
        if self.end_mass > 0.0 {
            self.thrust / self.end_mass
        } else {
            0.0
        }
    }
    pub fn resource_mass(&self) -> f64 {
        self.start_mass - self.end_mass
    }
    pub fn rcs_start_twr(&self, gee_asl: f64) -> f64 {
        self.rcs_start_tmr / (9.80665 * gee_asl)
    }
    pub fn rcs_max_twr(&self, gee_asl: f64) -> f64 {
        self.rcs_end_tmr / (9.80665 * gee_asl)
    }
    pub fn start_twr(&self, gee_asl: f64) -> f64 {
        if self.start_mass > 0.0 {
            self.thrust / (9.80665 * gee_asl * self.start_mass)
        } else {
            0.0
        }
    }
    pub fn max_twr(&self, gee_asl: f64) -> f64 {
        self.max_accel() / (9.80665 * gee_asl)
    }
}

impl fmt::Display for FuelStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "KSP Stage: {} Thrust: {} Time: {} StartMass: {} EndMass: {} DeltaV: {} ISP: {}",
            self.ksp_stage,
            self.thrust,
            self.delta_time,
            self.start_mass,
            self.end_mass,
            self.deltav,
            self.isp
        )
    }
}

pub mod decoupling_analyzer {
    use std::{cell::RefCell, rc::Rc};

    use super::{SimPart, SimPartModule, SimVessel};

    pub fn analyze(vessel: Rc<RefCell<SimVessel>>) {
        let root_part = find_root_part(&vessel.borrow().parts);
        calculate_decoupled_in_stage_recursively(vessel, root_part, None, -1);
    }

    fn find_root_part(parts: &[Rc<RefCell<SimPart>>]) -> Rc<RefCell<SimPart>> {
        for part in parts {
            if part.borrow().is_root {
                return part.clone();
            }
        }
        parts[0].clone()
    }

    fn calculate_decoupled_in_stage_recursively(
        vessel: Rc<RefCell<SimVessel>>,
        part: Rc<RefCell<SimPart>>,
        parent: Option<Rc<RefCell<SimPart>>>,
        inherited_decoupled_in_stage: i32,
    ) {
        let child_decoupled_in_stage = calculate_decoupled_in_stage(
            vessel.clone(),
            part.clone(),
            parent.clone(),
            inherited_decoupled_in_stage,
        );

        for link in &part.borrow().links {
            if parent
                .as_ref()
                .map(|x| Rc::ptr_eq(&link.upgrade().unwrap(), x))
                .unwrap_or_default()
            {
                continue;
            }

            calculate_decoupled_in_stage_recursively(
                vessel.clone(),
                part.clone(),
                parent.clone(),
                child_decoupled_in_stage,
            )
        }
    }

    fn calculate_decoupled_in_stage(
        vessel_rc: Rc<RefCell<SimVessel>>,
        part_rc: Rc<RefCell<SimPart>>,
        parent: Option<Rc<RefCell<SimPart>>>,
        parent_decoupled_in_stage: i32,
    ) -> i32 {
        let mut part = part_rc.borrow_mut();

        if part.decoupled_in_stage != i32::MIN {
            return part.decoupled_in_stage;
        }

        if part.inverse_stage >= parent_decoupled_in_stage {
            for module in part.modules.iter().cloned() {
                match module {
                    SimPartModule::SimModuleDecouple(decouple) => {
                        let decouple = decouple.borrow();
                        if !decouple.is_decoupled && decouple.staging_enabled && part.staging_on {
                            if decouple.is_omni_decoupler {
                                part_rc.borrow_mut().decoupled_in_stage = part.inverse_stage;
                                track_part_decoupled_in_stage(
                                    &vessel_rc,
                                    &part_rc,
                                    part.decoupled_in_stage,
                                );
                                return part.decoupled_in_stage;
                            }

                            if let Some(attached_part) = &decouple.attached_part {
                                if parent
                                    .as_ref()
                                    .map(|x| Rc::ptr_eq(x, attached_part))
                                    .unwrap_or_default()
                                    && decouple.staged
                                {
                                    part.decoupled_in_stage = part.inverse_stage;
                                    track_part_decoupled_in_stage(
                                        &vessel_rc,
                                        &part_rc,
                                        part.decoupled_in_stage,
                                    );
                                    return part.decoupled_in_stage;
                                }

                                part.decoupled_in_stage = parent_decoupled_in_stage;
                                track_part_decoupled_in_stage(
                                    &vessel_rc,
                                    &part_rc,
                                    part.decoupled_in_stage,
                                );
                                calculate_decoupled_in_stage_recursively(
                                    vessel_rc.clone(),
                                    attached_part.clone(),
                                    Some(part_rc.clone()),
                                    part.inverse_stage,
                                );
                                return part.decoupled_in_stage;
                            }
                        }
                    }
                    SimPartModule::SimLaunchClamp(_) => {
                        part.decoupled_in_stage = if part.inverse_stage > parent_decoupled_in_stage
                        {
                            part.inverse_stage
                        } else {
                            parent_decoupled_in_stage
                        };
                        track_part_decoupled_in_stage(
                            &vessel_rc,
                            &part_rc,
                            part.decoupled_in_stage,
                        );
                        return part.decoupled_in_stage;
                    }
                    SimPartModule::SimModuleDockingNode(node) => {
                        let node = node.borrow();

                        if node.staging_enabled && part.staging_on {
                            if let Some(attached_part) = &node.attached_part {
                                if parent
                                    .as_ref()
                                    .map(|x| Rc::ptr_eq(x, attached_part))
                                    .unwrap_or_default()
                                    && node.staged
                                {
                                    part.decoupled_in_stage = part.inverse_stage;
                                    track_part_decoupled_in_stage(
                                        &vessel_rc,
                                        &part_rc,
                                        part.decoupled_in_stage,
                                    );
                                    return part.decoupled_in_stage;
                                }

                                part.decoupled_in_stage = parent_decoupled_in_stage;
                                track_part_decoupled_in_stage(
                                    &vessel_rc,
                                    &part_rc,
                                    part.decoupled_in_stage,
                                );
                                calculate_decoupled_in_stage_recursively(
                                    vessel_rc.clone(),
                                    attached_part.clone(),
                                    Some(part_rc.clone()),
                                    part.inverse_stage,
                                );
                                return part.decoupled_in_stage;
                            }
                        }
                    }
                    SimPartModule::SimProceduralFairingDecoupler(decoupler) => {
                        let decoupler = decoupler.borrow();
                        if !decoupler.is_decoupled && decoupler.staging_enabled && part.staging_on {
                            part.decoupled_in_stage = part.inverse_stage;
                            track_part_decoupled_in_stage(
                                &vessel_rc,
                                &part_rc,
                                part.decoupled_in_stage,
                            );
                            return part.decoupled_in_stage;
                        }
                    }
                    _ => {}
                }
            }
        }

        part.decoupled_in_stage = parent_decoupled_in_stage;
        track_part_decoupled_in_stage(&vessel_rc, &part_rc, part.decoupled_in_stage);
        part.decoupled_in_stage
    }

    fn track_part_decoupled_in_stage(
        vessel: &Rc<RefCell<SimVessel>>,
        part: &Rc<RefCell<SimPart>>,
        stage: i32,
    ) {
        let mut vessel = vessel.borrow_mut();
        for i in (stage + 1)..=vessel.current_stage {
            vessel
                .parts_remaining_in_stage
                .entry(i)
                .or_default()
                .push(part.clone());
        }
    }
}
