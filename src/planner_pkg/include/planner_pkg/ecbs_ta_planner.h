#ifndef _ECBS_TA_PLANNER_H_
#define _ECBS_TA_PLANNER_H_

#include <iostream>
#include "planner_commons.h"
#include "mapf_lib/ecbs_ta.hpp"
#include "mapf_lib/next_best_assignment.hpp"
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/graphviz.hpp>

using mapf_lib::ECBSTA;
using mapf_lib::Neighbor;
using mapf_lib::PlanResult;
using mapf_lib::NextBestAssignment;

class ShortestPathHeuristic{
 public:
  ShortestPathHeuristic(size_t dimx, size_t dimy,
                        const std::unordered_set<Location>& obstacles)
      : m_shortestDistance(nullptr), m_dimx(dimx), m_dimy(dimy) {
    searchGraph_t searchGraph;

    // add vertices
    for (size_t x = 0; x < dimx; ++x) {
      for (size_t y = 0; y < dimy; ++y) {
        boost::add_vertex(searchGraph);
      }
    }

    // add edges
    for (size_t x = 0; x < dimx; ++x) {
      for (size_t y = 0; y < dimy; ++y) {
        Location l(x, y);
        if (obstacles.find(l) == obstacles.end()) {
          Location right(x + 1, y);
          if (x < dimx - 1 && obstacles.find(right) == obstacles.end()) {
            auto e =
                boost::add_edge(locToVert(l), locToVert(right), searchGraph);
            searchGraph[e.first].weight = 1;
          }
          Location below(x, y + 1);
          if (y < dimy - 1 && obstacles.find(below) == obstacles.end()) {
            auto e =
                boost::add_edge(locToVert(l), locToVert(below), searchGraph);
            searchGraph[e.first].weight = 1;
          }
        }
      }
    }

    writeDotFile(searchGraph, "searchGraph.dot");

    m_shortestDistance = new distanceMatrix_t(boost::num_vertices(searchGraph));
    distanceMatrixMap_t distanceMap(*m_shortestDistance, searchGraph);
    // The following generates a clang-tidy error, see
    // https://svn.boost.org/trac10/ticket/10830
    boost::floyd_warshall_all_pairs_shortest_paths(
        searchGraph, distanceMap,
        boost::weight_map(boost::get(&Edge::weight, searchGraph)));
  }

  ~ShortestPathHeuristic() { delete m_shortestDistance; }

  int getValue(const Location& a, const Location& b) {
    vertex_t idx1 = locToVert(a);
    vertex_t idx2 = locToVert(b);
    return (*m_shortestDistance)[idx1][idx2];
  }

  private:
    size_t locToVert(const Location& l) const { return l.x + m_dimx * l.y; }

    Location idxToLoc(size_t idx) {
      int x = idx % m_dimx;
      int y = idx / m_dimx;
      return Location(x, y);
    }

  private:
  typedef boost::adjacency_list_traits<boost::vecS, boost::vecS,
                                       boost::undirectedS>
      searchGraphTraits_t;
  typedef searchGraphTraits_t::vertex_descriptor vertex_t;
  typedef searchGraphTraits_t::edge_descriptor edge_t;

  struct Vertex {};

  struct Edge {
    int weight;
  };

  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                Vertex, Edge>
      searchGraph_t;
  typedef boost::exterior_vertex_property<searchGraph_t, int>
      distanceProperty_t;
  typedef distanceProperty_t::matrix_type distanceMatrix_t;
  typedef distanceProperty_t::matrix_map_type distanceMatrixMap_t;

  class VertexDotWriter {
   public:
    explicit VertexDotWriter(const searchGraph_t& graph, size_t dimx)
        : m_graph(graph), m_dimx(dimx) {}

    void operator()(std::ostream& out, const vertex_t& v) const {
      static const float DX = 100;
      static const float DY = 100;
      out << "[label=\"";
      int x = v % m_dimx;
      int y = v / m_dimx;
      out << "\" pos=\"" << x * DX << "," << y * DY << "!\"]";
    }

    private:
      const searchGraph_t& m_graph;
      size_t m_dimx;
  };

  class EdgeDotWriter {
   public:
    explicit EdgeDotWriter(const searchGraph_t& graph) : m_graph(graph) {}

    void operator()(std::ostream& out, const edge_t& e) const {
      out << "[label=\"" << m_graph[e].weight << "\"]";
    }

   private:
    const searchGraph_t& m_graph;
  };

  private:
    void writeDotFile(const searchGraph_t& graph, const std::string& fileName) {
      VertexDotWriter vw(graph, m_dimx);
      EdgeDotWriter ew(graph);
      std::ofstream dotFile(fileName);
      boost::write_graphviz(dotFile, graph, vw, ew);
    }

  private:
    distanceMatrix_t* m_shortestDistance;
    size_t m_dimx;
    size_t m_dimy;
};

class ECBSTA_Environment {
 public:
  ECBSTA_Environment(size_t dimx, size_t dimy,
              const std::unordered_set<Location>& obstacles,
              const std::vector<State>& startStates,
              const std::vector<std::unordered_set<Location> >& goals,
              size_t maxTaskAssignments)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(obstacles),
        m_agentIdx(0),
        m_goal(nullptr),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_maxTaskAssignments(maxTaskAssignments),
        m_numTaskAssignments(0),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
        m_heuristic(dimx, dimy, obstacles) {
    m_numAgents = startStates.size();
    for (size_t i = 0; i < startStates.size(); ++i) {
      for (const auto& goal : goals[i]) {
        m_assignment.setCost(
            i, goal, m_heuristic.getValue(
                         Location(startStates[i].x, startStates[i].y), goal));
        m_goals.insert(goal);
      }
    }
    m_assignment.solve();
  }

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints,
                          const Location* task) {
    assert(constraints);
    m_agentIdx = agentIdx;
    m_goal = task;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    if (m_goal != nullptr) {
      for (const auto& vc : constraints->vertexConstraints) {
        if (vc.x == m_goal->x && vc.y == m_goal->y) {
          m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
        }
      }
    } else {
      for (const auto& vc : constraints->vertexConstraints) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
    // std::cout << "setLLCtx: " << agentIdx << " " << m_lastGoalConstraint <<
    // std::endl;
  }

  int admissibleHeuristic(const State& s) {
    if (m_goal != nullptr) {
      return m_heuristic.getValue(Location(s.x, s.y), *m_goal);
    } else {
      return 0;
    }
  }

  // low-level
  int focalStateHeuristic(
      const State& s, int /*gScore*/,
      const std::vector<PlanResult<State, Action, int> >& solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && solution[i].states.size() > 0) {
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
      const std::vector<PlanResult<State, Action, int> >& solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && solution[i].states.size() > 0) {
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
      const std::vector<PlanResult<State, Action, int> >& solution) {
    int numConflicts = 0;

    int max_t = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      max_t = std::max<int>(max_t, solution[i].states.size() - 1);
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
    bool atGoal = true;
    if (m_goal != nullptr) {
      atGoal = s.x == m_goal->x && s.y == m_goal->y;
    }
    return atGoal && s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
    neighbors.clear();
    {
      State n(s.time + 1, s.x, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        bool atGoal = true;
        if (m_goal != nullptr) {
          atGoal = s.x == m_goal->x && s.y == m_goal->y;
        }
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, atGoal ? 0 : 1));
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Left, 1));
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Right, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Down, 1));
      }
    }
  }

  bool getFirstConflict(
      const std::vector<PlanResult<State, Action, int> >& solution,
      Conflict& result) {
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size());
    }

    for (int t = 0; t < max_t; ++t) {
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

  void nextTaskAssignment(std::map<size_t, Location>& tasks) {
    if (m_numTaskAssignments > m_maxTaskAssignments) {
      return;
    }

    int64_t cost = m_assignment.nextSolution(tasks);
    if (!tasks.empty()) {
      std::cout << "nextTaskAssignment: cost: " << cost << std::endl;
      for (const auto& s : tasks) {
        std::cout << s.first << "->" << s.second << std::endl;
      }

      ++m_numTaskAssignments;
    }
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

  size_t numTaskAssignments() const { return m_numTaskAssignments; }

 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int> >& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
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
  size_t m_agentIdx;
  const Location* m_goal;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  NextBestAssignment<size_t, Location> m_assignment;
  size_t m_maxTaskAssignments;
  size_t m_numTaskAssignments;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  ShortestPathHeuristic m_heuristic;
  size_t m_numAgents;
  std::unordered_set<Location> m_goals;
};


#endif