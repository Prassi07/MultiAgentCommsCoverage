#ifndef _ECBS_PLANNER_H_
#define _ECBS_PLANNER_H_

#include "planner_commons.h"
#include "mapf_lib/ecbs.hpp"

class ECBS_Environment {
 public:
  ECBS_Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
              std::vector<Location> goals, bool disappearAtGoal = false)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
        m_disappearAtGoal(disappearAtGoal)
  {
  }

  ECBS_Environment(const ECBS_Environment&) = delete;
  ECBS_Environment& operator=(const ECBS_Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  int admissibleHeuristic(const State& s) {
    return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);
  }

  // low-level
  int focalStateHeuristic(
      const State& s, int /*gScore*/,
      const std::vector<mapf_lib::PlanResult<State, Action, int> >& solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State state2 = getState(i, solution, s.time);
        if (s.equalExceptTime(state2)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // low-level
  int focalTransitionHeuristic(
      const State& s1a, const State& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
      const std::vector<mapf_lib::PlanResult<State, Action, int> >& solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State s2a = getState(i, solution, s1a.time);
        State s2b = getState(i, solution, s1b.time);
        if (s1a.equalExceptTime(s2b) && s1b.equalExceptTime(s2a)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // Count all conflicts
  int focalHeuristic(
      const std::vector<mapf_lib::PlanResult<State, Action, int> >& solution) {
    int numConflicts = 0;

    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            ++numConflicts;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            ++numConflicts;
          }
        }
      }
    }
    return numConflicts;
  }

  bool isSolution(const State& s) {
    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    std::vector<mapf_lib::Neighbor<State, Action, int> >& neighbors) {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
    neighbors.clear();
    {
      State n(s.time + 1, s.x, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            mapf_lib::Neighbor<State, Action, int>(n, Action::Wait, 1));
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            mapf_lib::Neighbor<State, Action, int>(n, Action::Left, 1));
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            mapf_lib::Neighbor<State, Action, int>(n, Action::Right, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(mapf_lib::Neighbor<State, Action, int>(n, Action::Up, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            mapf_lib::Neighbor<State, Action, int>(n, Action::Down, 1));
      }
    }
  }

  bool getFirstConflict(
      const std::vector<mapf_lib::PlanResult<State, Action, int> >& solution,
      Conflict& result) {
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t <= max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
            // std::endl;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }

  void createConstraintsFromConflict(
      const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

 private:
  State getState(size_t agentIdx,
                 const std::vector<mapf_lib::PlanResult<State, Action, int> >& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    if (m_disappearAtGoal) {
      // This is a trick to avoid changing the rest of the code significantly
      // After an agent disappeared, put it at a unique but invalid position
      // This will cause all calls to equalExceptTime(.) to return false.
      return State(-1, -1 * (agentIdx+1), -1);
    }
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool transitionValid(const State& s1, const State& s2) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }

 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<Location> m_obstacles;
  std::vector<Location> m_goals;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  bool m_disappearAtGoal;
};

#endif