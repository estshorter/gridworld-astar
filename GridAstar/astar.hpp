#pragma once
#include <algorithm>
#include <functional>
#include <vector>
#include <queue>
#include "misc.hpp"

#include "GridGraph.hpp"

namespace AStars {
	template<typename T, typename priority_t>
	struct PriorityQueue {
		using PQElement = std::pair<priority_t, T>;
		std::priority_queue<PQElement, std::vector<PQElement>,
			std::greater<PQElement>> elements;

		inline bool empty() const {
			return elements.empty();
		}

		inline void put(T item, priority_t priority) {
			elements.emplace(priority, item);
		}

		T get() {
			T best_item = elements.top().second;
			elements.pop();
			return best_item;
		}
	};

	// return cost of the shortest path
	template <class Cost, class Hash, class Graph>
	Cost AStar(const Graph& graph,
		const typename Graph::Location& start,
		const typename Graph::Location& goal,
		std::function<Cost(const typename Graph::Location& a,
			const typename Graph::Location& b)> cost,
		std::function<Cost(const typename Graph::Location& a,
			const typename Graph::Location& b)> heuristic,
		std::unordered_map<typename Graph::Location, typename Graph::Location, Hash>& came_from,
		std::unordered_map<typename Graph::Location, Cost, Hash>& cost_so_far
	) {
		using Location = typename Graph::Location;

		PriorityQueue<Location, Cost> frontier;
		frontier.put(start, Cost(0));

		came_from[start] = start;
		cost_so_far[start] = Cost(0);

		while (!frontier.empty()) {
			Location current = frontier.get();
			if (current == goal) {
				break;
			}
			for (Location next : graph.GetNeighbors(current)) {
				Cost new_cost = cost_so_far[current] + cost(current, next);
				if (cost_so_far.find(next) == cost_so_far.end()
					|| new_cost < cost_so_far[next]) {
					cost_so_far[next] = new_cost;
					Cost priority = new_cost + heuristic(next, goal);
					frontier.put(next, priority);
					came_from[next] = current;
				}
			}
		}
		return cost_so_far[goal];
	}

 	// return shortest path and its cost
	template <class Cost, class Hash, class Graph>
	auto AStar(const Graph& graph,
		const typename Graph::Location& start,
		const typename Graph::Location& goal,
		std::function<Cost(const typename Graph::Location& a,
			const typename Graph::Location& b)> cost,
		std::function<Cost(const typename Graph::Location& a,
			const typename Graph::Location& b)> heuristic
		) {
		using Location = typename Graph::Location;
		std::unordered_map<Location, Location, Hash> came_from;
		std::unordered_map<Location, Cost, Hash> cost_so_far;
		
		Cost cost_shortest_path = AStar(graph, start, goal, cost, heuristic, came_from, cost_so_far);

		std::vector<Location> path = { goal };
		for (auto prev = came_from[goal]; prev != start; prev = came_from.at(prev)) {
			path.push_back(prev);
		}
		path.push_back(start);
		std::reverse(path.begin(), path.end());

		return std::make_pair(path, cost_shortest_path);
	}
}