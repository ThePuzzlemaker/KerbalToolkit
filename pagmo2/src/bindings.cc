#include "pagmo2/src/bindings.hpp"

vector_double vec_from_rust(rust::Vec<f64> res) {
  vector_double out;
  out.reserve(res.size());
  out.insert(out.end(), res.size(), res.front());
  return out;
}

std::unique_ptr<CxxProblemWrapper>
new_problem(rust::Box<ProblemWrapper> inner) {
  return std::make_unique<CxxProblemWrapper>(std::move(inner));
};

std::pair<vector_double, vector_double>
CxxProblemWrapperInner::get_bounds() const {
  assert(m_inner.has_value());
  rust::Vec<f64> lb_rs;
  rust::Vec<f64> ub_rs;
  get()->get_bounds(lb_rs, ub_rs);
  return std::make_pair(vec_from_rust(lb_rs), vec_from_rust(ub_rs));
}

pagmo::vector_double
CxxProblemWrapperInner::fitness(const vector_double &x) const {
  assert(m_inner.has_value());
  return vec_from_rust(get()->fitness(x));
}

vector_double::size_type CxxProblemWrapperInner::get_nobj() const {
  assert(m_inner.has_value());
  return get()->get_nobj();
}

vector_double::size_type CxxProblemWrapperInner::get_nec() const {
  assert(m_inner.has_value());
  return get()->get_nec();
}

vector_double::size_type CxxProblemWrapperInner::get_nic() const {
  assert(m_inner.has_value());
  return get()->get_nic();
}

vector_double::size_type CxxProblemWrapperInner::get_nix() const {
  assert(m_inner.has_value());
  return get()->get_nix();
};

rust::Box<ProblemWrapper> &CxxProblemWrapperInner::get() const {
  return *m_inner.value().get();
}

std::unique_ptr<std::string> CxxProblemWrapper::debug() const {
  std::stringstream ss;
  ss << m_problem;
  return std::make_unique<std::string>(ss.str());
}

std::shared_ptr<Gaco> new_gaco(u32 gen, u32 ker, f64 q, f64 oracle, f64 acc,
                               u32 threshold, u32 n_gen_mark, u32 impstop,
                               u32 evalstop, f64 focus, bool memory, u32 seed) {
  return std::make_shared<Gaco>(gaco(gen, ker, q, oracle, acc, threshold,
                                     n_gen_mark, impstop, evalstop, focus,
                                     memory, seed));
}

std::unique_ptr<CxxAlgorithmWrapper> of_gaco(std::shared_ptr<Gaco> inner) {
  return std::make_unique<CxxAlgorithmWrapper>(std::move(inner));
};

pagmo::vector_double vec_from_slice(rust::Slice<const f64> val) {
  vector_double out;
  out.reserve(val.size());
  out.insert(out.end(), val.size(), val.front());
  return out;
};

rust::Vec<f64> vec_to_rust(vector_double val) {
  rust::Vec<f64> v;
  v.reserve(val.size());
  for (auto x : val) {
    v.push_back(x);
  }
  return v;
}

rust::Vec<f64> CxxProblemWrapper::fitness(rust::Slice<const f64> x) const {
  const vector_double x1 = vec_from_slice(x);
  const vector_double fitness = m_problem.fitness(x1);
  return vec_to_rust(fitness);
}

void CxxProblemWrapper::get_bounds(rust::Vec<f64> &lb, rust::Vec<f64> &ub) const {
  const auto lb_ub = m_problem.get_bounds();
  lb = vec_to_rust(lb_ub.first);
  ub = vec_to_rust(lb_ub.second);
}

usize CxxProblemWrapper::get_nobj() const { return m_problem.get_nobj(); }
usize CxxProblemWrapper::get_nic() const { return m_problem.get_nic(); }
usize CxxProblemWrapper::get_nec() const { return m_problem.get_nec(); }
usize CxxProblemWrapper::get_nix() const { return m_problem.get_nix(); }


std::unique_ptr<std::string> CxxAlgorithmWrapper::debug() const {
  std::stringstream ss;
  ss << m_algorithm;
  return std::make_unique<std::string>(ss.str());
}
