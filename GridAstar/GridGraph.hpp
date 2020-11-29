#pragma once

#include <algorithm>
#include <utility>
#include <unordered_set>
#include <array>
#include <vector>
#include <iterator>
#include <iostream>
#include <queue>

#include "misc.hpp"

namespace GridGraph {
	template <class Location>
	int L1NormInt(const Location& u, const Location& v) {
		return abs(std::get<0>(u) - std::get<0>(v)) + abs(std::get<1>(u) - std::get<1>(v));
	}
	template <class Location>
	int L1NormInt3D(const Location& u, const Location& v) {
		return abs(std::get<0>(u) - std::get<0>(v)) + abs(std::get<1>(u) - std::get<1>(v))
			+ abs(std::get<2>(u) - std::get<2>(v));
	}
	class GridGraph2d {
	public:
		using Location = std::pair<int, int>;
		static constexpr std::array<Location, 4> DIRS = {
			/* East, West, North, South */
			Location{1, 0}, Location{-1, 0},
			Location{0, -1}, Location{0, 1}
		};

		GridGraph2d(const int sizeX, const int sizeY) : mSizeX(sizeX), mSizeY(sizeY) {
		}

		std::vector<Location> GetNeighbors(const Location& u) const {
			std::vector<Location> results;

			for (Location dir : DIRS) {
				Location next{ u.first + dir.first, u.second + dir.second };
				if (InBounds(next) && Passable(next)) {
					results.push_back(next);
				}
			}

			if ((u.first + u.second) % 2 == 0) {
				// see "Ugly paths" section for an explanation:
				std::reverse(results.begin(), results.end());
			}
			return results;
		};

		std::vector<Location> GetNthNeighbors(const Location& u, const int max_depth) const {
			std::unordered_set<Location, pair_hash> visited;
			std::queue<std::pair<Location, int>> queue; // location and depth
			std::vector<Location> nNeighbors;

			queue.push({ u, 0 });
			while (!queue.empty()) {
				auto [current, depth] = queue.front();
				queue.pop();

				visited.insert(current);
				nNeighbors.push_back(current);
				auto depth_next = depth + 1;
				for (Location next : GetNeighbors(current)) {
					if (visited.find(next) == visited.end() && depth_next <= max_depth) {
						queue.push({ next, depth_next });
					}
				}
			}
			return nNeighbors;
		};

		void AddWall(const int idx) {
			int posX = idx % mSizeX;
			int posY = (idx - posX) / mSizeX;
			mWalls.insert({ posX, posY });
		}
		void AddWall(const Location&& u) {
			mWalls.insert(u);
		}

		void Draw(const Location& start, const Location& goal) const {
			constexpr int field_width = 3;
			std::cout << std::string(field_width * mSizeX, '_') << '\n';
			for (int y = 0; y != mSizeY; ++y) {
				for (int x = 0; x != mSizeX; ++x) {
					Location u{ x, y };
					if (mWalls.find(u) != mWalls.end()) {
						std::cout << std::string(field_width, '#');
					}
					else if (u == start) {
						std::cout << " A ";
					}
					else if (u == goal) {
						std::cout << " Z ";
					}
					else {
						std::cout << " . ";
					}
				}
				std::cout << '\n';
			}
			std::cout << std::string(field_width * mSizeX, '~') << '\n';
		}

		void Draw(const Location& start, const Location& goal, const std::vector<Location> path) const {
			constexpr int field_width = 3;
			std::cout << std::string(field_width * mSizeX, '_') << '\n';
			for (int y = 0; y != mSizeY; ++y) {
				for (int x = 0; x != mSizeX; ++x) {
					Location u{ x, y };
					if (mWalls.find(u) != mWalls.end()) {
						std::cout << std::string(field_width, '#');
					}
					else if (u == start) {
						std::cout << " A ";
					}
					else if (u == goal) {
						std::cout << " Z ";
					}
					else if (find(path.begin(), path.end(), u) != path.end()) {
						std::cout << " @ ";
					}
					else {
						std::cout << " . ";
					}
				}
				std::cout << '\n';
			}
			std::cout << std::string(field_width * mSizeX, '~') << '\n';
		}

		int GetSizeX()const {
			return mSizeX;
		}
		int GetSizeY() const {
			return mSizeY;
		}

		const std::unordered_set<Location, pair_hash>& GetWalls() const {
			return mWalls;
		}

	private:
		std::unordered_set<Location, pair_hash> mWalls;
		int mSizeX; //width
		int mSizeY; //height

		bool InBounds(const Location& u) const {
			return 0 <= u.first && u.first < mSizeX
				&& 0 <= u.second && u.second < mSizeY;
		}

		bool Passable(const Location& u) const {
			return mWalls.find(u) == mWalls.end();
		}
	};

	class GridGraph3d {
	public:
		using Location = std::tuple<int, int, int>;
		using Location2D = std::pair<int, int>;
		static constexpr std::array<Location, 6> DIRS = {
			Location{1, 0,0}, Location{-1, 0,0},
			Location{0, -1,0}, Location{0, 1,0},
			Location{0, 0,1},Location{0, 0,-1}
		};

		GridGraph3d(const int sizeX, const int sizeY, const int sizeZ) : mSizeX(sizeX), mSizeY(sizeY), mSizeZ(sizeZ) {
		}

		std::vector<Location> GetNeighbors(const Location u) const {
			std::vector<Location> results;

			for (Location dir : DIRS) {
				Location next{ std::get<0>(u) + std::get<0>(dir), std::get<1>(u) + std::get<1>(dir), std::get<2>(u) + std::get<2>(dir) };
				if (InBounds(next) && Passable(next)) {
					results.push_back(next);
				}
			}

			if ((std::get<0>(u) + std::get<1>(u) + std::get<2>(u)) % 2 == 0) {
				// see "Ugly paths" section for an explanation:
				std::reverse(results.begin(), results.end());
			}
			return results;
		};

		void AddHeight(const int idx, const uint8_t height) {
			int posX = idx % mSizeX;
			int posY = (idx - posX) / mSizeX;
			mHeightMap[{posX, posY}] = height;
		}
		void AddHeight(const Location2D u, const uint8_t height) {
			mHeightMap[u] = height;
		}

		// いしのなかにいる
		bool InObstacle(const Location& u) const {
			Location2D posXY = { std::get<0>(u), std::get<1>(u) };
			if (mHeightMap.find(posXY) == mHeightMap.end())
				return false;

			if (mHeightMap.at(posXY) >= std::get<2>(u)) {
				return true;
			}
			return false;
		}
		const std::unordered_map<Location2D, uint8_t, pair_hash>& GetHeight() const {
			return mHeightMap;
		}

		std::vector<Location> GetNthNeighbors(const Location& u, const int max_depth) const {
			std::unordered_set<Location, triple_tuple_hash> visited;
			std::queue<std::pair<Location, int>> queue; // location and depth
			std::vector<Location> nNeighbors;

			queue.push({ u, 0 });
			while (!queue.empty()) {
				auto [current, depth] = queue.front();
				queue.pop();

				visited.insert(current);
				nNeighbors.push_back(current);
				auto depth_next = depth + 1;
				for (Location next : GetNeighbors(current)) {
					if (visited.find(next) == visited.end() && depth_next <= max_depth) {
						queue.push({ next, depth_next });
					}
				}
			}
			return nNeighbors;
		};


	private:
		// height0は地面（impassable）として仮定する。
		std::unordered_map<Location2D, uint8_t, pair_hash> mHeightMap;
		int mSizeX; //width
		int mSizeY; //height
		int mSizeZ; //depth

		bool InBounds(const Location& u) const {
			return 0 <= std::get<0>(u) && std::get<0>(u) < mSizeX
				&& 0 <= std::get<1>(u) && std::get<1>(u) < mSizeY
				&& 0 <= std::get<2>(u) && std::get<2>(u) < mSizeZ;
		}

		bool Passable(const Location& v) const {
			Location2D posXY = { std::get<0>(v), std::get<1>(v) };
			int posZ = std::get<2>(v);
			if (mHeightMap.find(posXY) != mHeightMap.end() && mHeightMap.at(posXY) >= posZ) {
				return false;
			}
			return true;
		}
	};
}