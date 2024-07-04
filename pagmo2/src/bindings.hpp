#pragma once

#include <optional>
#include <pagmo/pagmo.hpp>

#include "pagmo2/src/lib.rs.h"
// #include "rust/cxx.h"

#define f64 double
#define u32 unsigned
#define usize vector_double::size_type


using namespace pagmo;

vector_double vec_from_rust(rust::Vec<f64> res);
vector_double vec_from_slice(rust::Slice<const f64> val);
rust::Vec<f64> vec_to_rust(rust::Vec<f64> val);

struct CxxProblemWrapperInner {
  std::optional<std::shared_ptr<rust::Box<ProblemWrapper>>> m_inner;

public:
  CxxProblemWrapperInner() {}
  CxxProblemWrapperInner(rust::Box<ProblemWrapper> inner)
      : m_inner(
            std::make_shared<rust::Box<ProblemWrapper>>(std::move(inner))){};

  rust::Box<ProblemWrapper> &get() const;

  vector_double fitness(const vector_double &x) const;

  std::pair<vector_double, vector_double> get_bounds() const;

  usize get_nobj() const;

  usize get_nec() const;

  usize get_nic() const;

  usize get_nix() const;
};

struct CxxProblemWrapper {
private:
  problem m_problem;

public:
  CxxProblemWrapper(rust::Box<ProblemWrapper> inner)
      : m_problem(problem(CxxProblemWrapperInner(std::move(inner)))) {}
  std::unique_ptr<std::string> debug() const;
  rust::Vec<f64> fitness(rust::Slice<const f64> x) const;
  void get_bounds(rust::Vec<f64> &lb, rust::Vec<f64> &ub) const;
  usize get_nobj() const;
  usize get_nec() const;
  usize get_nic() const;
  usize get_nix() const;
};

std::unique_ptr<CxxProblemWrapper> new_problem(rust::Box<ProblemWrapper> inner);

struct Gaco {
private:
  gaco m_inner;
public:
  Gaco() : m_inner(gaco()) {}
  Gaco(gaco inner) : m_inner(inner) {}
  population evolve(const population& pop) const {
    return m_inner.evolve(pop);
  }
};

struct CxxAlgorithmWrapper {
private:
  algorithm m_algorithm;

public:
  template <typename T> CxxAlgorithmWrapper(std::shared_ptr<T> alg) : m_algorithm(algorithm(*alg.get())) {}
  std::unique_ptr<std::string> debug() const;
  
};

std::shared_ptr<Gaco> new_gaco(u32 gen, u32 ker, f64 q, f64 oracle, f64 acc,
                               u32 threshold, u32 n_gen_mark, u32 impstop,
                               u32 evalstop, f64 focus, bool memory, u32 seed);

std::unique_ptr<CxxAlgorithmWrapper> of_gaco(std::shared_ptr<Gaco> inner);
