#![allow(clippy::all)]
#![allow(warnings)]
use std::fmt;

use cxx::{SharedPtr, UniquePtr};
use ffi::{CxxAlgorithmWrapper, CxxProblemWrapper};

pub trait GenericProblem {
    fn fitness(&self, x: &[f64]) -> Vec<f64>;
    fn get_bounds(&self) -> (Vec<f64>, Vec<f64>);
    fn get_nobj(&self) -> usize;

    fn get_nec(&self) -> usize {
        0
    }

    fn get_nic(&self) -> usize {
        0
    }

    fn get_nix(&self) -> usize {
        0
    }

    fn get_name(&self) -> Option<String> {
        None
    }

    fn get_extra_info(&self) -> Option<String> {
        None
    }

    fn has_hessians_sparsity(&self) -> bool {
        false
    }

    fn has_gradient_sparsity(&self) -> bool {
        false
    }

    fn has_hessians(&self) -> bool {
        false
    }

    fn has_gradient(&self) -> bool {
        false
    }

    fn has_batch_fitness(&self) -> bool {
        false
    }

    // TODO
    //fn get_thread_safety(&self )-> Option<ThreadSafety> { None }
}

struct ProblemV0;

impl GenericProblem for ProblemV0 {
    fn fitness(&self, x: &[f64]) -> Vec<f64> {
        vec![
            x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2],
            x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3] - 40., // equality con.
            25. - x[0] * x[1] * x[2] * x[3],                             // inequality con.
        ]
    }

    fn get_bounds(&self) -> (Vec<f64>, Vec<f64>) {
        (vec![1., 1., 1., 1.], vec![5., 5., 5., 5.])
    }

    fn get_nobj(&self) -> usize {
        1
    }

    fn get_nic(&self) -> usize {
        1
    }

    fn get_nec(&self) -> usize {
        1
    }
}

#[test]
fn foo() {
    let p = PagmoProblem::new(Box::new(ProblemV0));
    println!("{}", p);
    let gaco = PagmoAlgorithm::from(GacoBuilder::new().build());
    println!("{:?}", p.inner.fitness(&[2.5, 2.5, 2.5, 2.5]));
    panic!();
}

pub struct PagmoAlgorithm {
    inner: UniquePtr<CxxAlgorithmWrapper>,
}

impl From<Gaco> for PagmoAlgorithm {
    fn from(value: Gaco) -> Self {
        PagmoAlgorithm {
            inner: ffi::of_gaco(value.0),
        }
    }
}

pub struct PagmoProblem {
    inner: UniquePtr<CxxProblemWrapper>,
}

impl PagmoProblem {
    pub fn new(problem: Box<dyn GenericProblem>) -> Self {
        Self {
            inner: ffi::new_problem(Box::new(ProblemWrapper { inner: problem })),
        }
    }
}

impl GenericProblem for PagmoProblem {
    fn fitness(&self, x: &[f64]) -> Vec<f64> {
        todo!()
    }

    fn get_bounds(&self) -> (Vec<f64>, Vec<f64>) {
        todo!()
    }

    fn get_nobj(&self) -> usize {
        todo!()
    }
}

impl fmt::Display for PagmoProblem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.inner)
    }
}

struct ProblemWrapper {
    inner: Box<dyn GenericProblem>,
}

impl ProblemWrapper {
    fn fitness(&self, x: &cxx::Vector<f64>) -> Vec<f64> {
        self.inner.fitness(x.as_slice())
    }

    fn get_bounds(&self, lb: &mut Vec<f64>, ub: &mut Vec<f64>) {
        let (lb_new, ub_new) = self.inner.get_bounds();
        *lb = lb_new;
        *ub = ub_new;
    }

    fn get_nobj(&self) -> usize {
        self.inner.get_nobj()
    }

    fn get_nic(&self) -> usize {
        self.inner.get_nic()
    }

    fn get_nec(&self) -> usize {
        self.inner.get_nec()
    }

    fn get_nix(&self) -> usize {
        self.inner.get_nix()
    }
}

impl fmt::Display for CxxProblemWrapper {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.debug())
    }
}

pub struct Gaco(SharedPtr<ffi::Gaco>);

pub struct GacoBuilder {
    gen: u32,
    ker: u32,
    q: f64,
    oracle: f64,
    acc: f64,
    threshold: u32,
    n_gen_mark: u32,
    impstop: u32,
    evalstop: u32,
    focus: f64,
    memory: bool,
    seed: u32,
}

impl GacoBuilder {
    pub fn new() -> Self {
        Self {
            gen: 1,
            ker: 63,
            q: 1.0,
            oracle: 0.0,
            acc: 0.01,
            threshold: 1,
            n_gen_mark: 7,
            impstop: 100_000,
            evalstop: 100_000,
            focus: 0.0,
            memory: false,
            seed: rand::random(),
        }
    }

    pub fn gen(mut self, gen: u32) -> Self {
        self.gen = gen;
        self
    }

    pub fn ker(mut self, ker: u32) -> Self {
        self.ker = ker;
        self
    }

    pub fn q(mut self, q: f64) -> Self {
        self.q = q;
        self
    }

    pub fn oracle(mut self, oracle: f64) -> Self {
        self.oracle = oracle;
        self
    }

    pub fn acc(mut self, acc: f64) -> Self {
        self.acc = acc;
        self
    }

    pub fn threshold(mut self, threshold: u32) -> Self {
        self.threshold = threshold;
        self
    }

    pub fn n_gen_mark(mut self, n_gen_mark: u32) -> Self {
        self.n_gen_mark = n_gen_mark;
        self
    }

    pub fn impstop(mut self, impstop: u32) -> Self {
        self.impstop = impstop;
        self
    }

    pub fn evalstop(mut self, evalstop: u32) -> Self {
        self.evalstop = evalstop;
        self
    }

    pub fn focus(mut self, focus: f64) -> Self {
        self.focus = focus;
        self
    }

    pub fn memory(mut self, memory: bool) -> Self {
        self.memory = memory;
        self
    }

    pub fn seed(mut self, seed: u32) -> Self {
        self.seed = seed;
        self
    }

    pub fn build(self) -> Gaco {
        Gaco(ffi::new_gaco(
            self.gen,
            self.ker,
            self.q,
            self.oracle,
            self.acc,
            self.threshold,
            self.n_gen_mark,
            self.impstop,
            self.evalstop,
            self.focus,
            self.memory,
            self.seed,
        ))
    }
}

#[cxx::bridge]
mod ffi {
    extern "Rust" {
        type ProblemWrapper;
        fn fitness(&self, x: &CxxVector<f64>) -> Vec<f64>;
        fn get_bounds(&self, lb: &mut Vec<f64>, ub: &mut Vec<f64>);
        fn get_nobj(&self) -> usize;
        fn get_nec(&self) -> usize;
        fn get_nic(&self) -> usize;
        fn get_nix(&self) -> usize;
    }

    unsafe extern "C++" {
        include!("pagmo2/src/common.hpp");
        include!("pagmo2/src/bindings.hpp");

        // type Problem;
        type CxxProblemWrapper;
        fn new_problem(inner: Box<ProblemWrapper>) -> UniquePtr<CxxProblemWrapper>;
        fn debug(&self) -> UniquePtr<CxxString>;
        fn fitness(&self, x: &[f64]) -> Vec<f64>;
        fn get_bounds(&self, lb: &mut Vec<f64>, ub: &mut Vec<f64>);
        fn get_nobj(&self) -> usize;
        fn get_nec(&self) -> usize;
        fn get_nic(&self) -> usize;
        fn get_nix(&self) -> usize;
    }

    unsafe extern "C++" {
        include!("pagmo2/src/common.hpp");
        include!("pagmo2/src/bindings.hpp");

        type CxxAlgorithmWrapper;
        fn debug(&self) -> UniquePtr<CxxString>;
    }

    unsafe extern "C++" {
        include!("pagmo2/src/common.hpp");
        include!("pagmo2/src/bindings.hpp");

        type Gaco;
        fn new_gaco(
            gen: u32,
            ker: u32,
            q: f64,
            oracle: f64,
            acc: f64,
            threshold: u32,
            n_gen_mark: u32,
            impstop: u32,
            evalstop: u32,
            focus: f64,
            memory: bool,
            seed: u32,
        ) -> SharedPtr<Gaco>;
        fn of_gaco(gaco: SharedPtr<Gaco>) -> UniquePtr<CxxAlgorithmWrapper>;
    }
}
