#pragma once

#include <utility>
#include <random>
#include <iostream>
#include "typefile.h"
#include <numeric>
#include <discreture.hpp>

// check if two ranges intersect
bool isIntersect(const doublepair &, const doublepair &);
doublepair getIntersect(const doublepair &, const doublepair &);
bool isInTol(double, double);

std::string makeReplicate(const stringvec &, const std::string &, unsigned &);
bool fileInDirectory(const std::string &, const std::string &);

template<class T>
std::vector<std::vector<T>> vectorCombinations(const std::vector<std::vector<T>> &);

template<class T>
inline std::vector<std::vector<T>> vectorCombinations(const std::vector<std::vector<T>> &_vector)
{
	unsigned vec_size = 1;
	intvec index_vec(_vector.size());
	auto &ibeg = index_vec.begin();
	for (auto &vvbeg = _vector.begin(); vvbeg != _vector.end(); vvbeg++, ibeg++)
	{
		*ibeg = static_cast<int>(vvbeg->size() - 1);
		vec_size *= static_cast<unsigned>(vvbeg->size());
	}
	std::vector<std::vector<T>> vector(vec_size);

	discreture::multisets X(index_vec);
	auto &vbeg = vector.begin();
	for (auto &&x : X)
	{
		auto &vvbeg = _vector.begin();
		for (auto &xbeg = x.begin(); xbeg != x.end(); xbeg++, vvbeg++)
		{
			vbeg->push_back((*vvbeg)[*xbeg]);
		}
		vbeg++;
	}
	
	return vector;
}

template<class T>
inline std::vector<T> getSubVector(const std::vector<T> &vec, 
	const size_t &start, const size_t &end)
{
	return vector<T>(vec.begin() + start, vec.begin() + end);
}

template<class T>
inline size_t getPosInVec(const T &element, const std::vector<T> &vec)
{
	if (std::find(vec.begin(), vec.end(), element) != vec.end())
	{
		return std::find(vec.begin(), vec.end(), element) - vec.begin();
	}
	else
	{
		return -1;
	}
}

template <class T>
std::vector<T> getSubVector(const std::vector<T> &, const size_t &, const size_t &);

doublepairs makePairs(const doublevec &, const doublevec &);

doublevec sample(const double &, const double &, const unsigned & = unsigned(100));
std::string removeComponentPrePostfix(std::string);
std::string removeComponentPostfix(std::string);

std::string switchPath(std::string &, const std::string & = "Connections", const std::string & = "Specifications");

double roundByPrecision(const double &, const double & = 1e-4);

template <class T> 
size_t getPosInVec(const T &, const std::vector<T> &);
unsigned getFileNumInDirectory(const std::string &);
doublepairs generateIntersections(const unsignedvec2d &, const doublepairs &);

// miniclique cover related functions
bool removable(intvec &neighbor, intvec &cover);
int max_removable(intvec &neighbors, intvec &cover);
intvec procedure_1(intvec2d &neighbors, intvec &cover);
intvec procedure_2(intvec2d &neighbors, intvec &cover, int k);
int cover_size(intvec &cover);
intvec minCliqueCover(intvec2d &graph, int n);
cliquetype minCliqueCover(doublepairs &);
bool emptyGraph(intvec2d &graph);
void removeNode(intvec2d &graph, intvec &nodes);
std::string createConnectionName(const std::string &, const std::string &);

bool isBattery(const std::string &);
bool isVoltageRegulator(const std::string &);
bool isHbridge(const std::string &);
bool isMicroController(const std::string &);
bool isDCMotor(const std::string &);
bool isServo(const std::string &);
bool isBluetooth(const std::string &);
bool isForceSensor(const std::string &);
bool isCamera(const std::string &);
bool isEncoder(const std::string &);
bool isFileInFolder(const std::string &, const std::string &);