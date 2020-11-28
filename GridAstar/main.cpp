#include <fstream>
#include <string>
#include <cstdio>
#include <vector>
//#include <iomanip>

#include <nlohmann/json.hpp>

#include "GridGraph.hpp"
#include "astar.hpp"

using json = nlohmann::json;

using namespace GridGraph;
using namespace AStars;

template <typename ... Args>
std::string format(const std::string& fmt, Args ... args)
{
	size_t len = std::snprintf(nullptr, 0, fmt.c_str(), args ...);
	std::vector<char> buf(len + 1);
	std::snprintf(&buf[0], len + 1, fmt.c_str(), args ...);
	return std::string(&buf[0], &buf[0] + len);
}

void Grid2D() {
	constexpr int width = 20;
	constexpr int height = 10;
	auto graph = GridGraph2d(width, height);
	for (int i = 2; i < height - 2; i++) {
		graph.AddWall(GridGraph2d::Location(10, i));
	}
	for (int i = 3; i < 10; i++) {
		graph.AddWall(GridGraph2d::Location(i, 2));
		graph.AddWall(GridGraph2d::Location(i, 7));
	}

	auto start = GridGraph2d::Location(5, 5);
	auto goal = GridGraph2d::Location(18, 5);

	using Location = GridGraph2d::Location;
	std::unordered_map<Location, Location, pair_hash> came_from;
	std::unordered_map<Location, int, pair_hash> cost_so_far;
	AStar<int>(graph, start, goal,
		L1NormInt<Location>, L1NormInt<Location>
		, came_from, cost_so_far);
	auto [path, cost] = AStar<int, pair_hash>(graph, start, goal, L1NormInt<Location>, L1NormInt<Location>);

	graph.Draw(start, goal, path);
	std::cout << cost << std::endl;
}

void GridPseudo3D() {
	std::ifstream i("config.json");
	json config;
	i >> config;
	int cellSize = config["cellSize"];
	std::cout << config.dump() << std::endl;

	std::string filename = "map.bin";
	std::ifstream fin(filename, std::ios::in | std::ios::binary);
	if (!fin) {
		throw std::runtime_error("cannot open a map file");
	}
	std::vector<unsigned char> height_map(cellSize * cellSize);
	fin.read((char*)height_map.data(), height_map.size());
	fin.close();  //ファイルを閉じる

	int cnt = 1;
	for (auto& data : height_map) {
		std::cout << format("%02d ", +data);
		cnt++;
		if (cnt % cellSize == 0) {
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;

	auto graph = GridGraph2d(cellSize, cellSize);

	auto start = GridGraph2d::Location(1, 1);
	auto goal = GridGraph2d::Location(50, 50);

	using Location = GridGraph2d::Location;
	std::unordered_map<Location, Location, pair_hash> came_from;
	std::unordered_map<Location, int, pair_hash> cost_so_far;
	for (int idx = 0; idx < cellSize * cellSize; idx++) {
		if (height_map[idx] > 12) {
			graph.AddWall(idx);
		}
	}

	auto [path, cost] = AStar<int, pair_hash>(graph, start, goal, L1NormInt<Location>, L1NormInt<Location>);

	graph.Draw(start, goal, path);
}

void Grid3D() {
	std::ifstream i("config.json");
	json config;
	i >> config;
	int cellSize = config["cellSize"];
	std::cout << config.dump() << std::endl;

	std::string filename = "map.bin";
	std::ifstream fin(filename, std::ios::in | std::ios::binary);
	if (!fin) {
		throw std::runtime_error("cannot open a map file");
	}
	std::vector<unsigned char> height_map(cellSize * cellSize);
	fin.read((char*)height_map.data(), height_map.size());
	fin.close();  //ファイルを閉じる

	int cnt = 1;
	for (auto& data : height_map) {
		std::cout << format("%02d ", +data);
		cnt++;
		if (cnt % cellSize == 0) {
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;

	int max_height = *std::max_element(height_map.begin(), height_map.end());

	auto graph = GridGraph3d(cellSize, cellSize, max_height + 1);

	auto start = GridGraph3d::Location(1, 1, 9);
	auto goal = GridGraph3d::Location(50, 50, 1);
	for (int idx = 0; idx < cellSize * cellSize; idx++) {
		graph.AddHeight(idx, height_map[idx]);
	}

	if (graph.InObstacle(start)) {
		throw std::runtime_error("In obstacle at start");
	}
	if (graph.InObstacle(goal)) {
		throw std::runtime_error("In obstacle at goal");
	}
	using Location = GridGraph3d::Location;
	auto [path, cost] = AStar<int, triple_tuple_hash>(graph, start, goal, L1NormInt3D<Location>, L1NormInt3D<Location>);

	json path_json;
	for (auto& pos : path) {
		auto [x, y, z] = pos;
		path_json.push_back(std::array<int,3> { x, y, z });
		//std::cout << format("(%02d, %02d, %02d) ", +x,+y,+z);
	}
	//std::cout << std::endl;

	std::cout << "cost: " << cost << std::endl;
	std::cout << path_json << std::endl;
	std::ofstream o("path.json");
	o << path_json << std::endl;

}

int main(void) {
	//GridPseudo3D();
	Grid3D();
}