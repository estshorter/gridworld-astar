#pragma once

#include <utility>

struct pair_hash {
	template <class T1, class T2>
	unsigned int operator() (const std::pair<T1, T2>& p) const {
		unsigned int lhs = std::hash<T1>()(p.first), rhs = std::hash<T2>()(p.second);
		return lhs ^ (rhs + 0x9e3779b9 + (lhs << 6) + (lhs >> 2));
	}
};

struct triple_tuple_hash {
	template <class T1, class T2, class T3>
	unsigned int operator() (const std::tuple<T1, T2, T3>& t) const {
		std::pair<T1, T2> t12 = std::make_pair(std::get<0>(t), std::get<1>(t));
		pair_hash phash;
		unsigned int phashed = phash(t12);
		unsigned int lhs = std::hash<unsigned int>()(phashed), rhs = std::hash<T2>()(std::get<2>(t));
		return lhs ^ (rhs + 0x9e3779b9 + (lhs << 6) + (lhs >> 2));
	}
};
