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

use crate::math::H1;

pub struct FuelFlowSimulation {
    pub segments: Vec<FuelStats>,
    current_segment: FuelStats,
    time: f64,
    dv_linear_thrust: bool,
    parts_with_resource_drains: Vec<Rc<RefCell<SimPart>>>,
    parts_with_rcs_drains: Vec<Rc<RefCell<SimPart>>>,
    parts_with_rcs_drains2: Vec<Rc<RefCell<SimPart>>>,
    allocated_first_segment: bool,
    sources_rcs: Vec<Rc<RefCell<SimPart>>>,
    sources: Vec<Rc<RefCell<SimPart>>>,
}

impl FuelFlowSimulation {
    pub fn run(&mut self, vessel: &Rc<RefCell<SimVessel>>) {
        self.allocated_first_segment = false;
        self.time = 0.0;
        self.segments.clear();
        vessel.borrow_mut().main_throttle = 1.0;
        vessel.borrow_mut().activate_engines_and_rcs();

        while vessel.borrow_mut().current_stage >= 0 {
            self.simulate_stage(vessel);
            self.clear_residuals();
            self.compute_rcs_max_values(vessel);
            self.finish_segment(vessel);
            vessel.borrow_mut().stage();
        }

        self.segments.reverse();

        self.parts_with_resource_drains.clear();
    }

    fn simulate_rcs(&mut self, vessel: &Rc<RefCell<SimVessel>>, max: bool) {
        vessel.borrow_mut().save_rcs_status();
        self.parts_with_rcs_drains2.clear();

        let mut lastmass = vessel.borrow().mass;

        let mut steps = 100;

        loop {
            if steps == 0 {
                panic!("oops");
            }

            vessel.borrow_mut().update_rcs_stats();
            vessel.borrow_mut().update_active_rcs();
            if vessel.borrow().active_rcs.is_empty() {
                break;
            }

            self.update_rcs_drains(vessel);
            let dt = self.minimum_rcs_time_step();

            self.apply_rcs_drains(dt);
            vessel.borrow_mut().update_mass();
            self.finish_rcs_segment(
                max,
                dt,
                lastmass,
                vessel.borrow().mass,
                vessel.borrow().rcs_thrust,
            );
            lastmass = vessel.borrow().mass;

            steps -= 1;
        }
        self.unapply_rcs_drains();
        vessel.borrow_mut().reset_rcs_status();
        vessel.borrow_mut().update_mass();
    }

    fn unapply_rcs_drains(&mut self) {
        for p in &self.parts_with_rcs_drains2 {
            p.borrow_mut().unapply_rcs_drains();
        }
    }

    fn compute_rcs_min_values(&mut self, vessel: &Rc<RefCell<SimVessel>>) {
        self.simulate_rcs(vessel, false);
    }

    fn compute_rcs_max_values(&mut self, vessel: &Rc<RefCell<SimVessel>>) {
        self.simulate_rcs(vessel, true);
    }

    fn simulate_stage(&mut self, vessel: &Rc<RefCell<SimVessel>>) {
        vessel.borrow_mut().update_mass();
        vessel.borrow_mut().update_engine_stats();
        vessel.borrow_mut().update_active_engines();

        self.get_next_segment(vessel);
        self.compute_rcs_min_values(vessel);

        self.update_resource_drains_and_residuals(vessel);
        let mut current_thrust = vessel.borrow().thrust_magnitude;

        for _ in 0..100 {
            if self.allowed_to_stage(vessel) {
                return;
            }

            let dt = self.minimum_time_step();

            if (vessel.borrow().thrust_magnitude - current_thrust).abs() > 1e-12 {
                self.clear_residuals();
                self.compute_rcs_max_values(vessel);
                self.finish_segment(vessel);
                self.get_next_segment(vessel);
                current_thrust = vessel.borrow().thrust_magnitude;
            }

            self.time += dt;
            self.apply_resource_drains(dt);

            vessel.borrow_mut().update_mass();
            vessel.borrow_mut().update_engine_stats();
            vessel.borrow_mut().update_active_engines();
            self.update_resource_drains_and_residuals(vessel);
        }

        panic!("oops");
    }

    fn apply_rcs_drains(&mut self, dt: f64) {
        for part in &self.parts_with_rcs_drains {
            part.borrow_mut().apply_rcs_drain(dt);
        }
    }

    fn apply_resource_drains(&mut self, dt: f64) {
        for part in &self.parts_with_resource_drains {
            part.borrow_mut().apply_resource_drain(dt);
        }
    }

    fn update_rcs_drains(&mut self, vessel: &Rc<RefCell<SimVessel>>) {
        for part in &self.parts_with_rcs_drains {
            part.borrow_mut().clear_rcs_drains();
        }

        self.parts_with_rcs_drains.clear();

        for e in &vessel.borrow().active_rcs {
            let e = e.borrow();
            for (resource_id, mode) in &e.propellant_flow_modes {
                match mode {
                    SimFlowMode::NoFlow => self.update_rcs_drains_in_part(
                        &e.part.upgrade().unwrap(),
                        e.resource_consumptions[resource_id],
                        *resource_id,
                    ),
                    SimFlowMode::AllVessel | SimFlowMode::AllVesselBalance => self
                        .update_rcs_drains_in_parts(
                            &vessel.borrow().parts_remaining_in_stage
                                [&vessel.borrow().current_stage],
                            e.resource_consumptions[resource_id],
                            *resource_id,
                            false,
                        ),
                    SimFlowMode::StagePriorityFlow | SimFlowMode::StagePriorityFlowBalance => self
                        .update_rcs_drains_in_parts(
                            &vessel.borrow().parts_remaining_in_stage
                                [&vessel.borrow().current_stage],
                            e.resource_consumptions[resource_id],
                            *resource_id,
                            true,
                        ),
                    SimFlowMode::StageStackFlow
                    | SimFlowMode::StageStackFlowBalance
                    | SimFlowMode::StackPrioritySearch => self.update_rcs_drains_in_parts(
                        &e.part
                            .upgrade()
                            .unwrap()
                            .borrow()
                            .crossfeed_part_set
                            .iter()
                            .map(|x| x.upgrade().unwrap())
                            .collect::<Vec<_>>(),
                        e.resource_consumptions[resource_id],
                        *resource_id,
                        true,
                    ),
                    SimFlowMode::Null => {}
                }
            }
        }
    }

    fn update_rcs_drains_in_parts(
        &mut self,
        parts: &[Rc<RefCell<SimPart>>],
        resource_consumption: f64,
        resource_id: i32,
        use_priority: bool,
    ) {
        let mut max_priority = i32::MIN;

        self.sources_rcs.clear();

        for p in parts {
            if let Some(resource) = p.borrow().try_get_resource(resource_id) {
                if resource.free
                    || resource.amount <= p.borrow().resource_request_remaining_threshold
                {
                    continue;
                }

                if use_priority {
                    if p.borrow().resource_priority < max_priority {
                        continue;
                    }

                    if p.borrow().resource_priority > max_priority {
                        self.sources_rcs.clear();
                        max_priority = p.borrow().resource_priority;
                    }
                }
            }

            self.sources_rcs.push(p.clone());
        }

        for source in self.sources_rcs.clone() {
            self.update_rcs_drains_in_part(
                &source,
                resource_consumption / self.sources_rcs.len() as f64,
                resource_id,
            );
        }
    }

    fn update_rcs_drains_in_part(
        &mut self,
        p: &Rc<RefCell<SimPart>>,
        resource_consumption: f64,
        resource_id: i32,
    ) {
        self.parts_with_rcs_drains.push(p.clone());
        self.parts_with_rcs_drains2.push(p.clone());
        p.borrow_mut()
            .add_rcs_drain(resource_id, resource_consumption);
    }

    fn clear_residuals(&mut self) {
        for part in &self.parts_with_resource_drains {
            part.borrow_mut().clear_residuals();
        }
    }

    fn update_resource_drains_and_residuals(&mut self, vessel: &Rc<RefCell<SimVessel>>) {
        for part in &self.parts_with_resource_drains {
            part.borrow_mut().clear_resource_drains();
            part.borrow_mut().clear_residuals();
        }

        self.parts_with_resource_drains.clear();

        for e in &vessel.borrow().active_engines {
            for (resource_id, mode) in &e.borrow().propellant_flow_modes {
                match mode {
                    SimFlowMode::NoFlow => self.update_resource_drains_and_residuals_in_part(
                        &e.borrow().part.upgrade().unwrap(),
                        e.borrow().resource_consumptions[resource_id],
                        *resource_id,
                        e.borrow().module_residuals,
                    ),
                    SimFlowMode::AllVessel | SimFlowMode::AllVesselBalance => self
                        .update_resource_drains_and_residuals_in_parts(
                            &vessel.borrow().parts_remaining_in_stage
                                [&vessel.borrow().current_stage],
                            e.borrow().resource_consumptions[resource_id],
                            *resource_id,
                            false,
                            e.borrow().module_residuals,
                        ),
                    SimFlowMode::StagePriorityFlow | SimFlowMode::StagePriorityFlowBalance => self
                        .update_resource_drains_and_residuals_in_parts(
                            &vessel.borrow().parts_remaining_in_stage
                                [&vessel.borrow().current_stage],
                            e.borrow().resource_consumptions[resource_id],
                            *resource_id,
                            true,
                            e.borrow().module_residuals,
                        ),
                    SimFlowMode::StageStackFlow
                    | SimFlowMode::StageStackFlowBalance
                    | SimFlowMode::StackPrioritySearch => self
                        .update_resource_drains_and_residuals_in_parts(
                            &e.borrow()
                                .part
                                .upgrade()
                                .unwrap()
                                .borrow()
                                .crossfeed_part_set
                                .iter()
                                .map(|x| x.upgrade().unwrap())
                                .collect::<Vec<_>>(),
                            e.borrow().resource_consumptions[resource_id],
                            *resource_id,
                            true,
                            e.borrow().module_residuals,
                        ),
                    SimFlowMode::Null => {}
                }
            }
        }
    }

    fn update_resource_drains_and_residuals_in_parts(
        &mut self,
        parts: &[Rc<RefCell<SimPart>>],
        resource_consumption: f64,
        resource_id: i32,
        use_priority: bool,
        residual: f64,
    ) {
        let mut max_priority = i32::MIN;

        self.sources.clear();

        for p in parts {
            if let Some(resource) = p.borrow().try_get_resource(resource_id).cloned() {
                if resource.free
                    || resource.amount
                        <= residual * resource.max_amount
                            + p.borrow().resource_request_remaining_threshold
                {
                    continue;
                }

                if use_priority {
                    if p.borrow().resource_priority < max_priority {
                        continue;
                    }

                    if p.borrow().resource_priority > max_priority {
                        self.sources.clear();
                        max_priority = p.borrow().resource_priority;
                    }
                }
            }

            self.sources.push(p.clone());
        }

        for source in self.sources.clone() {
            self.update_resource_drains_and_residuals_in_part(
                &source,
                resource_consumption / self.sources.len() as f64,
                resource_id,
                residual,
            );
        }
    }

    fn update_resource_drains_and_residuals_in_part(
        &mut self,
        p: &Rc<RefCell<SimPart>>,
        resource_consumption: f64,
        resource_id: i32,
        residual: f64,
    ) {
        self.parts_with_resource_drains.push(p.clone());
        p.borrow_mut()
            .add_resource_drain(resource_id, resource_consumption);
        p.borrow_mut()
            .update_resource_residual(residual, resource_id);
    }

    fn minimum_rcs_time_step(&self) -> f64 {
        let max_time = self.rcs_max_time();

        if (0.0..f64::MAX).contains(&max_time) {
            max_time
        } else {
            0.0
        }
    }

    fn rcs_max_time(&self) -> f64 {
        let mut max_time = f64::MAX;

        for part in &self.parts_with_rcs_drains {
            max_time = cmp::min(
                OrderedFloat(part.borrow().rcs_max_time()),
                OrderedFloat(max_time),
            )
            .0;
        }

        max_time
    }

    fn minimum_time_step(&self) -> f64 {
        let max_time = self.resource_max_time();

        if (0.0..f64::MAX).contains(&max_time) {
            max_time
        } else {
            0.0
        }
    }

    fn resource_max_time(&self) -> f64 {
        let mut max_time = f64::MAX;

        for part in &self.parts_with_resource_drains {
            max_time = cmp::min(
                OrderedFloat(part.borrow().resource_max_time()),
                OrderedFloat(max_time),
            )
            .0;
        }

        max_time
    }

    fn finish_rcs_segment(
        &mut self,
        max: bool,
        delta_time: f64,
        start_mass: f64,
        end_mass: f64,
        rcs_thrust: f64,
    ) {
        let rcs_delta_v =
            rcs_thrust * delta_time / (start_mass - end_mass) * libm::log(start_mass / end_mass);
        let rcs_isp = rcs_delta_v / (G0 * libm::log(start_mass / end_mass));

        if self.current_segment.rcs_isp == 0.0 {
            self.current_segment.rcs_isp = rcs_isp;
        }
        if self.current_segment.rcs_thrust == 0.0 {
            self.current_segment.rcs_thrust = rcs_thrust;
        }

        if max {
            self.current_segment.max_rcs_deltav += rcs_delta_v;
            if self.current_segment.rcs_start_tmr == 0.0 {
                self.current_segment.rcs_start_tmr = rcs_thrust / start_mass;
            }
        } else {
            self.current_segment.min_rcs_deltav += rcs_delta_v;
            self.current_segment.rcs_end_tmr = self.current_segment.rcs_thrust / end_mass;
            self.current_segment.rcs_mass += start_mass - end_mass;
            self.current_segment.rcs_delta_time += delta_time;
        }
    }

    fn finish_segment(&mut self, vessel: &Rc<RefCell<SimVessel>>) {
        if !self.allocated_first_segment {
            return;
        }

        let start_mass = self.current_segment.start_mass;
        let thrust = self.current_segment.thrust;
        let end_mass = vessel.borrow().mass;
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

        self.segments.push(self.current_segment.clone());
    }

    fn get_next_segment(&mut self, vessel: &Rc<RefCell<SimVessel>>) {
        let mut staged_mass = 0.0;

        if self.allocated_first_segment {
            staged_mass = self.current_segment.end_mass - vessel.borrow().mass;
        } else {
            self.allocated_first_segment = true;
        }

        self.current_segment = FuelStats {
            delta_time: 0.0,
            deltav: 0.0,
            end_mass: 0.0,
            isp: 0.0,
            ksp_stage: vessel.borrow().current_stage,
            staged_mass,
            start_mass: vessel.borrow().mass,
            start_time: self.time,
            thrust: if self.dv_linear_thrust {
                vessel.borrow().thrust_magnitude
            } else {
                vessel.borrow().thrust_no_cos_loss
            },
            spool_up_time: vessel.borrow().spoolup_current,
            max_rcs_deltav: 0.0,
            min_rcs_deltav: 0.0,
            rcs_isp: 0.0,
            rcs_delta_time: 0.0,
            rcs_thrust: 0.0,
            rcs_mass: 0.0,
            rcs_start_tmr: 0.0,
            rcs_end_tmr: 0.0,
        };
    }

    fn allowed_to_stage(&self, vessel: &Rc<RefCell<SimVessel>>) -> bool {
        if vessel.borrow().active_engines.is_empty() {
            return true;
        }

        for e in &vessel.borrow().active_engines {
            if e.borrow().part.upgrade().unwrap().borrow().is_sepratron() {
                continue;
            }

            if e.borrow()
                .part
                .upgrade()
                .unwrap()
                .borrow()
                .decoupled_in_stage
                >= vessel.borrow().current_stage - 1
                || e.borrow()
                    .would_drop_accessible_fuel_tank(vessel.borrow().current_stage - 1)
            {
                return false;
            }
        }

        if vessel.borrow().parts_remaining_in_stage[&(vessel.borrow().current_stage - 1)].len()
            == vessel.borrow().parts_remaining_in_stage[&vessel.borrow().current_stage].len()
        {
            return false;
        }

        vessel.borrow().current_stage > 0
    }
}

const G0: f64 = 9.80665;

#[derive(Clone)]
pub enum SimPartModule {
    SimModuleEngines(Rc<RefCell<SimModuleEngines>>),
    SimModuleRCS(Rc<RefCell<SimModuleRCS>>),
    // SimLaunchClamp(Rc<RefCell<SimLaunchClamp>>),
    // SimModuleDecouple(Rc<RefCell<SimModuleDecouple>>),
    // SimModuleDockingNode(Rc<RefCell<SimModuleDockingNode>>),
    // SimProceduralFairingDecoupler(Rc<RefCell<SimProceduralFairingDecoupler>>),
}

pub struct SimPart {
    crossfeed_part_set: Vec<Weak<RefCell<SimPart>>>,
    links: Vec<Weak<RefCell<SimPart>>>,
    resources: HashMap<i32, SimResource>,
    resource_drains: HashMap<i32, f64>,
    rcs_drains: HashMap<i32, f64>,

    decoupled_in_stage: i32,
    inverse_stage: i32,
    vessel: Weak<RefCell<SimVessel>>,
    name: String,

    activates_even_if_disconnected: bool,
    is_throttle_locked: bool,
    resource_priority: i32,
    resource_request_remaining_threshold: f64,

    mass: f64,
    dry_mass: f64,
    crew_mass: f64,
    modules_staged_mass: f64,
    modules_unstaged_mass: f64,
    disabled_resource_mass: f64,

    is_launch_clamp: bool,
    is_engine: bool,

    resource_keys: Vec<i32>,
}

impl SimPart {
    fn is_sepratron(&self) -> bool {
        self.is_engine
            && self.is_throttle_locked
            && self.activates_even_if_disconnected
            && self.inverse_stage == self.decoupled_in_stage
    }

    fn update_mass(&mut self) {
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

    fn try_get_resource(&self, resource_id: i32) -> Option<&SimResource> {
        self.resources.get(&resource_id)
    }

    fn apply_resource_drain(&mut self, dt: f64) {
        for (id, resource_drain) in &self.resource_drains {
            self.resources
                .get_mut(id)
                .unwrap()
                .drain(dt * *resource_drain);
        }
    }

    fn apply_rcs_drain(&mut self, dt: f64) {
        for (id, rcs_drain) in &self.rcs_drains {
            self.resources
                .get_mut(id)
                .unwrap()
                .rcs_drain(dt * *rcs_drain);
        }
    }

    fn unapply_rcs_drains(&mut self) {
        self.resource_keys.clear();
        for &id in self.resources.keys() {
            self.resource_keys.push(id);
        }

        for id in &self.resource_keys {
            self.resources.get_mut(id).unwrap().reset_rcs();
        }
    }

    fn update_resource_residual(&mut self, residual: f64, resource_id: i32) {
        if !self.resources.contains_key(&resource_id) {
            return;
        }

        let resource = self.resources.get_mut(&resource_id).unwrap();
        resource.residual = cmp::max(OrderedFloat(resource.residual), OrderedFloat(residual)).0;
    }

    fn clear_residuals(&mut self) {
        self.resource_keys.clear();
        for &id in self.resources.keys() {
            self.resource_keys.push(id);
        }

        for id in &self.resource_keys {
            let resource = self.resources.get_mut(id).unwrap();
            resource.residual = 0.0;
        }
    }

    fn clear_resource_drains(&mut self) {
        self.resource_drains.clear();
    }

    fn clear_rcs_drains(&mut self) {
        self.rcs_drains.clear();
    }

    fn add_resource_drain(&mut self, resource_id: i32, resource_consumption: f64) {
        self.resource_drains
            .entry(resource_id)
            .and_modify(|x| *x += resource_consumption)
            .or_insert(resource_consumption);
    }

    fn add_rcs_drain(&mut self, resource_id: i32, resource_consumption: f64) {
        self.rcs_drains
            .entry(resource_id)
            .and_modify(|x| *x += resource_consumption)
            .or_insert(resource_consumption);
    }

    fn resource_max_time(&self) -> f64 {
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

    fn rcs_max_time(&self) -> f64 {
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

#[derive(Copy, Clone, Default)]
pub struct SimResource {
    free: bool,
    max_amount: f64,
    amount: f64,
    rcs_amount: f64,
    id: i32,
    density: f64,
    residual: f64,
}

impl SimResource {
    fn get_amount(&self) -> f64 {
        self.amount + self.rcs_amount
    }

    fn residual_threshold(&self) -> f64 {
        self.residual * self.max_amount
    }

    fn drain(&mut self, resource_drain: f64) -> &mut Self {
        self.amount -= resource_drain;
        if self.amount < 0.0 {
            self.amount = 0.0;
        }
        self
    }

    fn rcs_drain(&mut self, rcs_drain: f64) -> &mut Self {
        self.rcs_amount -= rcs_drain;
        if self.get_amount() < 0.0 {
            self.rcs_amount = -self.amount;
        }
        self
    }

    fn reset_rcs(&mut self) -> &mut Self {
        self.rcs_amount = 0.0;
        self
    }
}

pub struct SimPropellant {
    id: i32,
    ignore_for_isp: bool,
    ratio: f64,
    flow_mode: SimFlowMode,
    density: f64,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
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
    parts: Vec<Rc<RefCell<SimPart>>>,
    parts_remaining_in_stage: HashMap<i32, Vec<Rc<RefCell<SimPart>>>>,
    engines_dropped_in_stage: HashMap<i32, Vec<Rc<RefCell<SimModuleEngines>>>>,
    engines_activated_in_stage: HashMap<i32, Vec<Rc<RefCell<SimModuleEngines>>>>,
    rcs_activated_in_stage: HashMap<i32, Vec<Rc<RefCell<SimModuleRCS>>>>,
    rcs_dropped_in_stage: HashMap<i32, Vec<Rc<RefCell<SimModuleRCS>>>>,
    active_engines: Vec<Rc<RefCell<SimModuleEngines>>>,
    active_rcs: Vec<Rc<RefCell<SimModuleRCS>>>,

    current_stage: i32,
    // TODO: default=1.0
    main_throttle: f64,
    mass: f64,
    thrust_current: Vector3<f64>,
    rcs_thrust: f64,
    thrust_magnitude: f64,
    thrust_no_cos_loss: f64,
    spoolup_current: f64,
    atm_pressure: f64,
    atm_density: f64,
    mach_number: f64,
}

impl SimVessel {
    pub fn set_conditions(&mut self, atm_density: f64, atm_pressure: f64, mach_number: f64) {
        self.atm_density = atm_density;
        self.atm_pressure = atm_pressure;
        self.mach_number = mach_number;
    }

    fn update_mass(&mut self) {
        self.mass = 0.0;

        for part in &self.parts_remaining_in_stage[&self.current_stage] {
            let mut part = part.borrow_mut();
            part.update_mass();
            self.mass += part.mass;
        }
    }

    fn stage(&mut self) {
        if self.current_stage < 0 {
            return;
        }

        self.current_stage -= 1;

        self.activate_engines_and_rcs();
        self.update_mass();
    }

    fn activate_engines_and_rcs(&mut self) {
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

    fn update_active_engines(&mut self) {
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

    fn update_active_rcs(&mut self) {
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

    fn compute_rcs_thrust(&mut self) {
        self.rcs_thrust = 0.0;

        for rcs in &self.active_rcs {
            let mut rcs = rcs.borrow_mut();
            rcs.update();
            self.rcs_thrust += rcs.thrust;
        }
    }

    fn compute_thrust_and_spoolup(&mut self) {
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

    fn update_rcs_stats(&mut self) {
        for i in -1..self.current_stage {
            for rcs in &self.rcs_dropped_in_stage[&i] {
                rcs.borrow_mut().update();
            }
        }
    }

    fn update_engine_stats(&mut self) {
        for i in -1..self.current_stage {
            for engine in &self.engines_dropped_in_stage[&i] {
                engine.borrow_mut().update();
            }
        }
    }

    fn save_rcs_status(&mut self) {
        for i in -1..self.current_stage {
            for rcs in &self.rcs_dropped_in_stage[&i] {
                rcs.borrow_mut().save_status();
            }
        }
    }

    fn reset_rcs_status(&mut self) {
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
    propellants: Vec<SimPropellant>,
    propellant_flow_modes: HashMap<i32, SimFlowMode>,
    resource_consumptions: HashMap<i32, f64>,
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

    is_enabled: bool,
    part: Weak<RefCell<SimPart>>,
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
    fn activate(&mut self) {
        self.is_operational = true;
    }
    fn update_engine_status(&mut self) {
        if self.can_draw_resources() {
            return;
        }
        self.is_operational = false;
    }
    fn can_draw_resources(&self) -> bool {
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

    fn would_drop_accessible_fuel_tank(&self, stage_num: i32) -> bool {
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

    fn update(&mut self) {
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

pub struct SimModuleRCS {
    propellants: Vec<Rc<RefCell<SimPropellant>>>,
    propellant_flow_modes: HashMap<i32, SimFlowMode>,
    resource_consumptions: HashMap<i32, f64>,

    atmosphere_curve: H1,

    g: f64,
    isp: f64,
    thrust: f64,
    rcs_enabled: bool,
    saved_rcs_enabled: bool,
    isp_mult: f64,
    thrust_percentage: f64,
    max_fuel_flow: f64,
    mass_flow_rate: f64,

    is_enabled: bool,
    part: Weak<RefCell<SimPart>>,
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

    fn update_rcs_status(&mut self) {
        if self.can_draw_resources() {
            return;
        }

        self.rcs_enabled = false;
    }

    fn save_status(&mut self) {
        self.saved_rcs_enabled = self.rcs_enabled;
    }

    fn reset_status(&mut self) {
        self.rcs_enabled = self.saved_rcs_enabled;
    }

    fn can_draw_resources(&self) -> bool {
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

    fn update(&mut self) {
        self.isp = self.atmosphere_curve.evaluate(self.atm_pressure());
        let exhaust_vel = self.isp * self.g * self.isp_mult;
        self.mass_flow_rate = self.max_fuel_flow * (self.thrust_percentage * 0.01);
        self.thrust = exhaust_vel * self.mass_flow_rate;
        self.set_consumption_rates();
    }

    fn activate(&mut self) {
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

// #[derive(Clone)]
// pub struct SimLaunchClamp {
//     // is_enabled: bool,
//     // part: Weak<RefCell<SimPart>>,
//     // module_is_enabled: bool,
//     // staging_enabled: bool,
// }

// #[derive(Clone)]
// pub struct SimModuleDecouple {
//     is_decoupled: bool,
//     is_omni_decoupler: bool,
//     staged: bool,
//     attached_part: Option<Rc<RefCell<SimPart>>>,

//     // is_enabled: bool,
//     // part: Weak<RefCell<SimPart>>,
//     // module_is_enabled: bool,
//     staging_enabled: bool,
// }

// #[derive(Clone)]
// pub struct SimModuleDockingNode {
//     staged: bool,
//     attached_part: Option<Rc<RefCell<SimPart>>>,

//     // is_enabled: bool,
//     // part: Weak<RefCell<SimPart>>,
//     // module_is_enabled: bool,
//     staging_enabled: bool,
// }

// #[derive(Clone)]
// pub struct SimProceduralFairingDecoupler {
//     is_decoupled: bool,

//     // is_enabled: bool,
//     // part: Weak<RefCell<SimPart>>,
//     // module_is_enabled: bool,
//     staging_enabled: bool,
// }

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

// pub mod decoupling_analyzer {
//     use std::{cell::RefCell, rc::Rc};

//     use super::{SimPart, SimPartModule, SimVessel};

//     pub fn analyze(vessel: Rc<RefCell<SimVessel>>) {
//         let root_part = find_root_part(&vessel.borrow().parts);
//         calculate_decoupled_in_stage_recursively(vessel, root_part, None, -1);
//     }

//     fn find_root_part(parts: &[Rc<RefCell<SimPart>>]) -> Rc<RefCell<SimPart>> {
//         for part in parts {
//             if part.borrow().is_root {
//                 return part.clone();
//             }
//         }
//         parts[0].clone()
//     }

//     fn calculate_decoupled_in_stage_recursively(
//         vessel: Rc<RefCell<SimVessel>>,
//         part: Rc<RefCell<SimPart>>,
//         parent: Option<Rc<RefCell<SimPart>>>,
//         inherited_decoupled_in_stage: i32,
//     ) {
//         let child_decoupled_in_stage = calculate_decoupled_in_stage(
//             vessel.clone(),
//             part.clone(),
//             parent.clone(),
//             inherited_decoupled_in_stage,
//         );

//         for link in &part.borrow().links {
//             if parent
//                 .as_ref()
//                 .map(|x| Rc::ptr_eq(&link.upgrade().unwrap(), x))
//                 .unwrap_or_default()
//             {
//                 continue;
//             }

//             calculate_decoupled_in_stage_recursively(
//                 vessel.clone(),
//                 part.clone(),
//                 parent.clone(),
//                 child_decoupled_in_stage,
//             )
//         }
//     }

//     fn calculate_decoupled_in_stage(
//         vessel_rc: Rc<RefCell<SimVessel>>,
//         part_rc: Rc<RefCell<SimPart>>,
//         parent: Option<Rc<RefCell<SimPart>>>,
//         parent_decoupled_in_stage: i32,
//     ) -> i32 {
//         let mut part = part_rc.borrow_mut();

//         if part.decoupled_in_stage != i32::MIN {
//             return part.decoupled_in_stage;
//         }

//         if part.inverse_stage >= parent_decoupled_in_stage {
//             for module in part.modules.iter().cloned() {
//                 match module {
//                     SimPartModule::SimModuleDecouple(decouple) => {
//                         let decouple = decouple.borrow();
//                         if !decouple.is_decoupled && decouple.staging_enabled && part.staging_on {
//                             if decouple.is_omni_decoupler {
//                                 part_rc.borrow_mut().decoupled_in_stage = part.inverse_stage;
//                                 track_part_decoupled_in_stage(
//                                     &vessel_rc,
//                                     &part_rc,
//                                     part.decoupled_in_stage,
//                                 );
//                                 return part.decoupled_in_stage;
//                             }

//                             if let Some(attached_part) = &decouple.attached_part {
//                                 if parent
//                                     .as_ref()
//                                     .map(|x| Rc::ptr_eq(x, attached_part))
//                                     .unwrap_or_default()
//                                     && decouple.staged
//                                 {
//                                     part.decoupled_in_stage = part.inverse_stage;
//                                     track_part_decoupled_in_stage(
//                                         &vessel_rc,
//                                         &part_rc,
//                                         part.decoupled_in_stage,
//                                     );
//                                     return part.decoupled_in_stage;
//                                 }

//                                 part.decoupled_in_stage = parent_decoupled_in_stage;
//                                 track_part_decoupled_in_stage(
//                                     &vessel_rc,
//                                     &part_rc,
//                                     part.decoupled_in_stage,
//                                 );
//                                 calculate_decoupled_in_stage_recursively(
//                                     vessel_rc.clone(),
//                                     attached_part.clone(),
//                                     Some(part_rc.clone()),
//                                     part.inverse_stage,
//                                 );
//                                 return part.decoupled_in_stage;
//                             }
//                         }
//                     }
//                     SimPartModule::SimLaunchClamp(_) => {
//                         part.decoupled_in_stage = if part.inverse_stage > parent_decoupled_in_stage
//                         {
//                             part.inverse_stage
//                         } else {
//                             parent_decoupled_in_stage
//                         };
//                         track_part_decoupled_in_stage(
//                             &vessel_rc,
//                             &part_rc,
//                             part.decoupled_in_stage,
//                         );
//                         return part.decoupled_in_stage;
//                     }
//                     SimPartModule::SimModuleDockingNode(node) => {
//                         let node = node.borrow();

//                         if node.staging_enabled && part.staging_on {
//                             if let Some(attached_part) = &node.attached_part {
//                                 if parent
//                                     .as_ref()
//                                     .map(|x| Rc::ptr_eq(x, attached_part))
//                                     .unwrap_or_default()
//                                     && node.staged
//                                 {
//                                     part.decoupled_in_stage = part.inverse_stage;
//                                     track_part_decoupled_in_stage(
//                                         &vessel_rc,
//                                         &part_rc,
//                                         part.decoupled_in_stage,
//                                     );
//                                     return part.decoupled_in_stage;
//                                 }

//                                 part.decoupled_in_stage = parent_decoupled_in_stage;
//                                 track_part_decoupled_in_stage(
//                                     &vessel_rc,
//                                     &part_rc,
//                                     part.decoupled_in_stage,
//                                 );
//                                 calculate_decoupled_in_stage_recursively(
//                                     vessel_rc.clone(),
//                                     attached_part.clone(),
//                                     Some(part_rc.clone()),
//                                     part.inverse_stage,
//                                 );
//                                 return part.decoupled_in_stage;
//                             }
//                         }
//                     }
//                     SimPartModule::SimProceduralFairingDecoupler(decoupler) => {
//                         let decoupler = decoupler.borrow();
//                         if !decoupler.is_decoupled && decoupler.staging_enabled && part.staging_on {
//                             part.decoupled_in_stage = part.inverse_stage;
//                             track_part_decoupled_in_stage(
//                                 &vessel_rc,
//                                 &part_rc,
//                                 part.decoupled_in_stage,
//                             );
//                             return part.decoupled_in_stage;
//                         }
//                     }
//                     _ => {}
//                 }
//             }
//         }

//         part.decoupled_in_stage = parent_decoupled_in_stage;
//         track_part_decoupled_in_stage(&vessel_rc, &part_rc, part.decoupled_in_stage);
//         part.decoupled_in_stage
//     }

//     fn track_part_decoupled_in_stage(
//         vessel: &Rc<RefCell<SimVessel>>,
//         part: &Rc<RefCell<SimPart>>,
//         stage: i32,
//     ) {
//         let mut vessel = vessel.borrow_mut();
//         for i in (stage + 1)..=vessel.current_stage {
//             vessel
//                 .parts_remaining_in_stage
//                 .entry(i)
//                 .or_default()
//                 .push(part.clone());
//         }
//     }
// }
