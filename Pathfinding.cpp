#include "stdafx.h"
#include "Pathfinding.h"
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>
#include <functional>   // std::greater
#include <cassert> //assertions

using std::unordered_map;
using std::unordered_set;
using std::array;
using std::vector;
using std::queue;
using std::priority_queue;
using std::pair;
using std::tuple;
using std::tie;
using std::string;

namespace std {
	template <>
	struct hash<tuple<int, int> > {
		inline size_t operator()(const tuple<int, int>& location) const {
			int x, y;
			tie(x, y) = location;
			return x * 1812433253 + y;
		}
	};
}

template<typename L>
struct Graph {
	typedef L Location;
	typedef typename vector<Location>::iterator iterator;
	unordered_map<Location, vector<Location> > edges;

	inline const vector<Location>  neighbors(Location id) {
		return edges[id];
	}
};

template<typename T, typename Number = int>
struct PriorityQueue {
	typedef pair<Number, T> PQElement;
	priority_queue<PQElement, vector<PQElement>, std::greater<PQElement> > elements;
	inline bool empty() { return elements.empty(); }

	inline void put(T item, Number priority) {
		elements.emplace(priority, item);
	}

	inline T get() {
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	}
};

struct twoDimensionalGrid {
	typedef tuple<int, int> Location;

	unordered_set<Location> forests;
	int cost(Location a, Location b) {
		return forests.count(b) ? 5 : 1;
	}

	int width, height;
	unordered_set<Location> walls;

	twoDimensionalGrid(int width_, int height_)
		: width(width_), height(height_) {}

	inline bool in_bounds(Location id) {
		int x, y;
		tie(x, y) = id;
		return 0 <= x && x < width && 0 <= y && y < height;
	}

	inline bool passable(Location id) {
		return !walls.count(id);
	}
	
	vector<Location> neighbors(Location id) {
		int x, y, dx, dy;
		tie(x, y) = id;
		vector<Location> results;
		static array<Location, 4> DIRECTIONS { Location{ 1, 0 }, Location{ 0, -1 }, Location{ -1, 0 }, Location{ 0, 1 } };
		for (auto dir : DIRECTIONS) {
			tie(dx, dy) = dir;
			Location next(x + dx, y + dy);
			if (in_bounds(next) && passable(next)) {
				results.push_back(next);
			}
		}

		if ((x + y) % 2 == 0) {
			std::reverse(results.begin(), results.end());
		}

		return results;
	}
};

inline int heuristic(twoDimensionalGrid::Location a, twoDimensionalGrid::Location b) {
	int x1, y1, x2, y2;
	tie(x1, y1) = a;
	tie(x2, y2) = b;
	return abs(x1 - x2) + abs(y1 - y2);
}

template<typename Graph>
void a_star_search (Graph graph, typename Graph::Location start, typename Graph::Location goal,
					unordered_map<typename Graph::Location, typename Graph::Location>& came_from,
					unordered_map<typename Graph::Location, int>& cost_so_far)
{
	typedef typename Graph::Location Location;
	PriorityQueue<Location> frontier;
	frontier.put(start, 0);
	came_from[start] = start;
	cost_so_far[start] = 0;
	while (!frontier.empty()) {
		auto current = frontier.get();
		
		if (current == goal) {
			break;
		}

		for (auto next : graph.neighbors(current)) {
			int new_cost = cost_so_far[current] + graph.cost(current, next);
			if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
				cost_so_far[next] = new_cost;
				int priority = new_cost + heuristic(next, goal);
				frontier.put(next, priority);
				came_from[next] = current;
			}
		}
	}
}

template<typename Location>
vector<Location> reconstruct_path(
	Location start,
	Location goal,
	unordered_map<Location, Location>& came_from
	) {
		vector<Location> path;
		Location current = goal;
		path.push_back(current);
		while (current != start) {
			current = came_from[current];
			path.push_back(current);
	}
	std::reverse(path.begin(), path.end());
	return path;
}

int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY,
				const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
				int* pOutBuffer, const int nOutBufferSize) 
{
	twoDimensionalGrid grid(nMapWidth, nMapHeight);

	for (int y = 0; y < nMapHeight; ++y) {
		for (int x = 0; x < nMapWidth; ++x) {
			if (pMap[y * nMapWidth + x] == 0)
			{
				grid.walls.insert(twoDimensionalGrid::Location{ x, y });
			}
		}
	}

	twoDimensionalGrid::Location start{ nStartX, nStartY };
	twoDimensionalGrid::Location goal{ nTargetX, nTargetY };
	unordered_map<twoDimensionalGrid::Location, twoDimensionalGrid::Location> came_from;
	unordered_map<twoDimensionalGrid::Location, int> cost_so_far;
	
	a_star_search(grid, start, goal, came_from, cost_so_far);
	unordered_map<twoDimensionalGrid::Location, twoDimensionalGrid::Location>::const_iterator got = came_from.find(goal);
	
	if (got == came_from.end())
		return -1;
	
	vector<twoDimensionalGrid::Location> path = reconstruct_path(start, goal, came_from);

	for (int index = 1; index <= nOutBufferSize; index++)
	{
		if (index < path.size())
		{
			twoDimensionalGrid::Location currentLocation = path[index];
			int xLocation = std::get<0>(currentLocation);
			int yLocation = std::get<1>(currentLocation);
			int bufferValue = yLocation * nMapWidth + xLocation;
			pOutBuffer[index - 1] = bufferValue;
		}
		else
		{
			pOutBuffer[index - 1] = 0;
		}
	}

	int shortestLength = path.size() - 1;
	return shortestLength;
}