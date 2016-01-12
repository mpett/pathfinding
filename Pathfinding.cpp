#include "stdafx.h";
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
	// I know, this is technically not allowed
	template <>
	struct hash<tuple<int, int> > {
		inline size_t operator()(const tuple<int, int>& location) const {
			int x, y;
			tie(x, y) = location;
			return x * 1812433253 + y;
		}
	};
}

std::basic_iostream<char>::basic_ostream& operator<<(std::basic_iostream<char>::basic_ostream& out, tuple<int, int> loc) {
	int x, y;
	tie(x, y) = loc;
	out << '(' << x << ',' << y << ')';
	return out;
}

template<class Graph>
void draw_grid(const Graph& graph, int field_width,
	unordered_map<typename Graph::Location, int>* distances = nullptr,
	unordered_map<typename Graph::Location, typename Graph::Location>* point_to = nullptr,
	vector<typename Graph::Location>* path = nullptr) {
	for (int y = 0; y != graph.height; ++y) {
		for (int x = 0; x != graph.width; ++x) {
			typename Graph::Location id{ x, y };
			std::cout << std::left << std::setw(field_width);
			if (graph.walls.count(id)) {
				std::cout << '#';
			}
			else if (point_to != nullptr && point_to->count(id)) {
				int x2, y2;
				tie(x2, y2) = (*point_to)[id];
				// TODO: how do I get setw to work with utf8?
				if (x2 == x + 1) { std::cout << ">"; }
				else if (x2 == x - 1) { std::cout << "<"; }
				else if (y2 == y + 1) { std::cout << "^"; }
				else if (y2 == y - 1) { std::cout << "v"; }
				else { std::cout << "* "; }
			}
			else if (distances != nullptr && distances->count(id)) {
				std::cout << (*distances)[id];
			}
			else if (path != nullptr && find(path->begin(), path->end(), id) != path->end()) {
				std::cout << '@';
			}
			else {
				std::cout << '.';
			}
		}
		std::cout << std::endl;
	}
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


template<typename Graph>
unordered_map<typename Graph::Location, typename Graph::Location> breadth_first_search(Graph graph,
	typename Graph::Location start,
	typename Graph::Location goal) {
	typedef typename Graph::Location Location;
	queue<Location> frontier;
	frontier.push(start);

	unordered_map<Location, Location> came_from;
	came_from[start] = start;

	while (!frontier.empty()) {
		auto current = frontier.front();
		frontier.pop();
		
		if (current == goal) {
			break;
		}

		for (auto next : graph.neighbors(current)) {
		
			if (!came_from.count(next)) {
				
				frontier.push(next);
				came_from[next] = current;
			}
		}
	}
	return came_from;
}

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

template<typename Graph>
void dijkstra_search
(Graph graph,
	typename Graph::Location start,
	typename Graph::Location goal,
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
				came_from[next] = current;
				frontier.put(next, new_cost);
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
		static array<Location, 4> DIRS { Location{ 1, 0 }, Location{ 0, -1 }, Location{ -1, 0 }, Location{ 0, 1 } };
		for (auto dir : DIRS) {
			
			tie(dx, dy) = dir;
			Location next(x + dx, y + dy);
			if (in_bounds(next) && passable(next)) {
				results.push_back(next);
			}
		}

		if ((x + y) % 2 == 0) {
			// aesthetic improvement on square grids
			std::reverse(results.begin(), results.end());
		}

		return results;
	}
};

void add_rect(twoDimensionalGrid& grid, int x1, int y1, int x2, int y2) {
	for (int x = x1; x <= x2; ++x) {
		for (int y = y1; y <= y2; ++y) {
			grid.walls.insert(twoDimensionalGrid::Location{ x, y });
		}
	}
}

struct GridWithWeights : twoDimensionalGrid {
	unordered_set<Location> forests;
	GridWithWeights(int w, int h) : twoDimensionalGrid(w, h) {}
	int cost(Location a, Location b) {
		return forests.count(b) ? 5 : 1;
	}
};

twoDimensionalGrid make_diagram1() {
	twoDimensionalGrid grid(4, 3);
	add_rect(grid, 0, 1, 0, 2);
	add_rect(grid, 2, 1, 2, 1);
	return grid;
}

GridWithWeights make_diagram4() {
	GridWithWeights grid(10, 10);
	add_rect(grid, 1, 7, 4, 9);
	typedef twoDimensionalGrid::Location L;
	grid.forests = unordered_set<twoDimensionalGrid::Location>{
		L{ 3, 4 }, L{ 3, 5 }, L{ 4, 1 }, L{ 4, 2 },
		L{ 4, 3 }, L{ 4, 4 }, L{ 4, 5 }, L{ 4, 6 },
		L{ 4, 7 }, L{ 4, 8 }, L{ 5, 1 }, L{ 5, 2 },
		L{ 5, 3 }, L{ 5, 4 }, L{ 5, 5 }, L{ 5, 6 },
		L{ 5, 7 }, L{ 5, 8 }, L{ 6, 2 }, L{ 6, 3 },
		L{ 6, 4 }, L{ 6, 5 }, L{ 6, 6 }, L{ 6, 7 },
		L{ 7, 3 }, L{ 7, 4 }, L{ 7, 5 }
	};
	return grid;

}

inline int heuristic(twoDimensionalGrid::Location a, twoDimensionalGrid::Location b) {
	int x1, y1, x2, y2;
	tie(x1, y1) = a;
	tie(x2, y2) = b;
	return abs(x1 - x2) + abs(y1 - y2);
}

template<typename Graph>
void a_star_search
(Graph graph,
	typename Graph::Location start,
	typename Graph::Location goal,
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

int FindPath(const int nStartX, const int nStartY,
	const int nTargetX, const int nTargetY,
	const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
	int* pOutBuffer, const int nOutBufferSize) 
{
	twoDimensionalGrid grid(nMapWidth, nMapHeight);
	add_rect(grid, 0, 1, 0, 2);
	add_rect(grid, 2, 1, 2, 1);


	twoDimensionalGrid::Location start{ nStartX, nStartY };
	twoDimensionalGrid::Location goal{ nTargetX, nTargetY };
	unordered_map<twoDimensionalGrid::Location, twoDimensionalGrid::Location> came_from;
	unordered_map<twoDimensionalGrid::Location, int> cost_so_far;
	a_star_search(grid, start, goal, came_from, cost_so_far);
	
	draw_grid(grid, 2, nullptr, &came_from);
	std::cout << std::endl;
	draw_grid(grid, 3, &cost_so_far, nullptr);
	std::cout << std::endl;
	
	vector<twoDimensionalGrid::Location> path = reconstruct_path(start, goal, came_from);

	draw_grid(grid, 3, nullptr, nullptr, &path);
	std::cout << "-------------------" << std::endl;



	int shortestLength = path.size() - 1;
	return shortestLength;
}

int main() {	
	unsigned char pMap[] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
	int pOutBuffer[12];
	int result = FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);
	std::cout << result << std::endl;
	std::getchar();
	return 0;
}