#include "utility_functions.h"

using std::string;
using std::vector;
using std::filesystem::directory_iterator;

// function is used to determine whether two voltage rages have intersection,
// first pair is input, second pair is output
bool isIntersect(const doublepair &range1, const doublepair &range2)
{
	double min = std::max(range1.first, range2.first),
		max = std::min(range1.second, range2.second);

	return isInTol(min, max);
}

// function is used to compute intersected range of two voltage ranges, 
// first is input, second is output
doublepair getIntersect(const doublepair &range1, const doublepair &range2)
{
	double min = std::max(range1.first, range2.first), 
		max = std::min(range1.second, range2.second);
	if (isInTol(min, max))
	{
		return std::make_pair(min, max);
	}
	else
	{
		throw INTERSECT_ERROR;
	}
}

string makeReplicate(const stringvec &reference, const string &name, unsigned &index)
{
	std::string _name;
	if (index != 0)
	{
		_name = name + std::to_string(index);
	}
	else
	{
		_name = name;
	}

	auto &iter = std::find(reference.begin(), reference.end(), _name);
	if (iter != reference.end())
	{
		return makeReplicate(reference, name, ++index);
	}
	else
	{
		if (index == 0)
		{
			return name;
		}
		else
		{
			return name + std::to_string(index);
		}
	}
}

bool fileInDirectory(const std::string &file, const std::string &directory)
{
	for (const auto &entry : std::filesystem::directory_iterator(directory))
	{
		if (entry.path().string() == file)
		{
			return true;
		}
	}
	return false;
}
doublepairs makePairs(const doublevec &vec1, const doublevec &vec2)
{
	doublepairs pairs(vec1.size());
	auto &v1beg = vec1.begin(), &v2beg = vec2.begin();
	for (auto &pbeg = pairs.begin(); pbeg != pairs.end(); pbeg++, v1beg++, v2beg++)
	{
		pbeg->first = *v1beg;
		pbeg->second = *v2beg;
	}
	return pairs;
}

doublevec sample(const double &lb, const double &ub, const unsigned &num)
{
	doublevec samples(num);

	std::random_device rdev{};
	std::default_random_engine generator{ rdev() };
	std::uniform_real_distribution<double> distribution(lb, ub);

	// assign every element in the vector with a randome value
	for (auto &beg = samples.begin(); beg != samples.end(); beg++)
	{
		*beg = distribution(generator);
	}

	return samples;
}

std::string removeComponentPrePostfix(std::string component)
{
	auto beg_pos = component.rfind("\\") + 1, end_pos = component.find(".");
	return component.substr(beg_pos, end_pos - beg_pos);
}

string removeComponentPostfix(std::string component)
{
	auto end_pos = component.rfind("t");
	return component.substr(0, end_pos+1);
}

std::string switchPath(std::string &path, const std::string &str1, const std::string &str2)
{
	auto idx = path.find(str1);
	path.erase(idx, str1.size());
	path.insert(path.begin() + idx, str2.begin(), str2.end());
	return path;
}

double roundByPrecision(const double &val, const double &precesion)
{
	int temp_int = static_cast<int>(val / precesion);
	double rounded_val = temp_int * precesion;
	return rounded_val;
}

// function is used to accomodate numberic error happened in comparison
bool isInTol(double min, double max)
{
	bool status = false;
	double tol = 1e-3;
	if (max < min && max + tol >= min)
	{
		max = min;
		status = true;
	}
	else if (min <= max)
	{
		status = true;
	}
	return status;
}

unsigned getFileNumInDirectory(const std::string &path)
{
	unsigned cnt = 0;
	std::filesystem::directory_iterator end;
	for (auto &entry = std::filesystem::directory_iterator(path); entry != end; entry++)
	{
		cnt++;
	}
	return cnt;
}

// this function returns intersection of torque and velocity based on given indices
doublepairs generateIntersections(const unsignedvec2d &indices, const doublepairs &ranges)
{
	doublepairs intersected_ranges(indices.size());
	auto irbeg = intersected_ranges.begin();
	for (auto &ivbeg = indices.begin(); ivbeg != indices.end(); ivbeg++)
	{
		doublepair intersect_range;
		if (ivbeg->size() != 1)
		{
			for (auto &ibeg = ivbeg->begin() + 1; ibeg != ivbeg->end(); ibeg++)
			{
				if (ibeg == ivbeg->begin() + 1)
				{
					intersect_range = getIntersect(ranges[*ibeg], ranges[*ivbeg->begin()]);
				}
				else
				{
					intersect_range = getIntersect(intersect_range, ranges[*ibeg]);
				}
			}
		}
		else
		{
			intersect_range = ranges[*ivbeg->begin()];
		}
		*irbeg++ = intersect_range;
	}
	return intersected_ranges;
}

bool emptyGraph(intvec2d &graph)
{
	bool empty = true;
	for (int i = 0; i < graph.size(); ++i)
	{
		if (std::find(graph[i].begin(), graph[i].end(), 0) != graph[i].end())
		{
			empty = false;
		}
	}
	return empty;
}

void removeNode(intvec2d &graph, intvec &nodes)
{
	for (int k = 0; k < nodes.size(); ++k)
	{
		for (int i = 0; i < graph.size(); ++i)
		{
			for (int j = 0; j < graph.size(); ++j)
			{
				if (i == nodes[k] || j == nodes[k])
				{
					graph[i][j] = 1;
				}
			}
		}
	}
}

string createConnectionName(const std::string &component_name, const std::string &pin_name)
{
	return component_name + "." + pin_name;
}

cliquetype minCliqueCover(doublepairs &ranges)
{
	intvec all_index;
	cliquetype results;
	doublepairs output_ranges;
	intvec2d graph(ranges.size()), indices;
	auto gbeg = graph.begin();
	for (int i = 0; i < ranges.size(); ++i)
	{
		intvec row(ranges.size());
		auto rbeg = row.begin();
		for (int j = 0; j < ranges.size(); ++j)
		{
			if (i == j)
			{
				*rbeg++ = 1;
			}
			else
			{
				if (isIntersect(ranges[i], ranges[j]))
				{
					*rbeg++ = 0;
				}
				else
				{
					*rbeg++ = 1;
				}
			}
		}
		*gbeg++ = row;
	}

	intvec index;
	while (!emptyGraph(graph))
	{
		index = minCliqueCover(graph, graph.size() - index.size());
		all_index.insert(all_index.end(), index.begin(), index.end());
		indices.push_back(index);

		doublevec minvec, maxvec;
		for (int i = 0; i < index.size(); ++i)
		{
			minvec.push_back(ranges[index[i]].first);
			maxvec.push_back(ranges[index[i]].second);
		}
		output_ranges.push_back(std::make_pair(*std::max_element(minvec.begin(), minvec.end()),
			*std::min_element(maxvec.begin(), maxvec.end())));

		removeNode(graph, index);
	}
	for (int i = 0; i < ranges.size(); ++i)
	{
		if (std::find(all_index.begin(), all_index.end(), i) == all_index.end())
		{
			indices.insert(indices.end(), intvec{i});
			output_ranges.insert(output_ranges.end(), ranges[i]);
		}
	}
	results = std::make_tuple(output_ranges, indices);
	return results;
}

intvec minCliqueCover(intvec2d &graph, int _n)
{
	int n = graph.size(), K, i, j, k, p, q, r, s, min, edge;

	intvec2d neighbors;
	for (int i = 0; i < graph.size(); i++)
	{
		intvec neighbor;
		for (int j = 0; j < graph[i].size(); j++)
			if (graph[i][j] == 1)
				neighbor.push_back(j);
		neighbors.push_back(neighbor);
	}

	intvec clique_index;
	for (size_t K = _n; K != 1; K--)
	{
//		std::cout << "K: " << K << std::endl;
		int counter = 0;

		k = n - K;
		//Find Cliques
		bool found = false;

		min = n + 1;
		intvec2d covers;
		intvec allcover(graph.size());
		for (i = 0; i < graph.size(); i++)
			allcover[i] = 1;
		for (i = 0; i < allcover.size(); i++)
		{
			if (found)
				break;
			counter++;
			intvec cover = allcover;
			cover[i] = 0;
			cover = procedure_1(neighbors, cover);
			s = cover_size(cover);
			if (s < min)
				min = s;
			if (s <= k)
			{
				if (n - s == K)
				{
					clique_index.resize(K);
					auto cibeg = clique_index.begin();
					for (j = 0; j < cover.size(); j++)
						if (cover[j] == 0)
							*cibeg++ = j;
				}
				covers.insert(covers.end(), cover);
				found = true;
				break;
			}
			for (j = 0; j < n - k; j++)
				cover = procedure_2(neighbors, cover, j);
			s = cover_size(cover);
			if (s < min)
				min = s;
			if (n - s == K)
			{
				clique_index.resize(K);
				auto cibeg = clique_index.begin();
				for (j = 0; j < cover.size(); j++)
					if (cover[j] == 0)
						*cibeg++ = j;
			}
			covers.insert(covers.end(), cover);
			if (s <= k)
			{
				found = true;
				break;
			}
		}
		//Pairwise Intersections
		for (p = 0; p < covers.size(); p++)
		{
			if (found)
				break;
			for (q = p + 1; q < covers.size(); q++)
			{
				if (found)
					break;
				counter++;

				intvec cover = allcover;
				for (r = 0; r < cover.size(); r++)
					if (covers[p][r] == 0 && covers[q][r] == 0)
						cover[r] = 0;
				cover = procedure_1(neighbors, cover);
				s = cover_size(cover);
				if (s < min)
					min = s;
				if (s <= k)
				{
					if (n - s == K)
					{
						clique_index.resize(K);
						auto cibeg = clique_index.begin();
						for (j = 0; j < cover.size(); j++)
							if (cover[j] == 0)
								*cibeg++ = j;
					}
					found = true;
					break;
				}
				for (j = 0; j < k; j++)
					cover = procedure_2(neighbors, cover, j);
				s = cover_size(cover);
				if (s < min)
					min = s;
				if (n - s == K)
				{
					clique_index.resize(K);
					auto cibeg = clique_index.begin();
					for (j = 0; j < cover.size(); j++)
						if (cover[j] == 0)
							*cibeg++ = j;
				}
				if (s <= k)
				{
					found = true;
					break;
				}
			}
		}
		if (found)
			break;
	}
	return clique_index;
}

bool removable(intvec &neighbor, intvec &cover)
{
	bool check = true;
	for (int i = 0; i < neighbor.size(); i++)
		if (cover[neighbor[i]] == 0)
		{
			check = false;
			break;
		}
	return check;
}

int max_removable(intvec2d &neighbors, intvec &cover)
{
	int r = -1, max = -1;
	for (int i = 0; i < cover.size(); i++)
	{
		if (cover[i] == 1 && removable(neighbors[i], cover) == true)
		{
			intvec temp_cover = cover;
			temp_cover[i] = 0;
			int sum = 0;
			for (int j = 0; j < temp_cover.size(); j++)
				if (temp_cover[j] == 1 && removable(neighbors[j], temp_cover)
					== true)
					sum++;
			if (sum > max)
			{
				max = sum;
				r = i;
			}
		}
	}
	return r;
}

intvec procedure_1(intvec2d &neighbors, intvec &cover)
{
	intvec temp_cover = cover;
	int r = 0;
	while (r != -1)
	{
		r = max_removable(neighbors, temp_cover);
		if (r != -1)
			temp_cover[r] = 0;
	}
	return temp_cover;
}

intvec procedure_2(intvec2d &neighbors, intvec &cover, int k)
{
	int count = 0;
	intvec temp_cover = cover;
	int i = 0;
	for (int i = 0; i < temp_cover.size(); i++)
	{
		if (temp_cover[i] == 1)
		{
			int sum = 0, index;
			for (int j = 0; j < neighbors[i].size(); j++)
				if (temp_cover[neighbors[i][j]] == 0)
				{
					index = j;
					sum++;
				}
			if (sum == 1 && cover[neighbors[i][index]] == 0)
			{
				temp_cover[neighbors[i][index]] = 1;
				temp_cover[i] = 0;
				temp_cover = procedure_1(neighbors, temp_cover);
				count++;
			}
			if (count > k)
				break;
		}
	}
	return temp_cover;
}

int cover_size(intvec &cover)
{
	int count = 0;
	for (int i = 0; i < cover.size(); i++)
		if (cover[i] == 1)
			count++;
	return count;
}

bool isBattery(const string &file)
{
	return isFileInFolder(file, battery_path);
}

bool isVoltageRegulator(const string &file)
{
	return isFileInFolder(file, voltage_regulator_path);
}

bool isHbridge(const string &file)
{
	return isFileInFolder(file, h_bridge_path);
}

bool isMicroController(const string &file)
{
	return isFileInFolder(file, micro_controller_path);
}

bool isDCMotor(const string &file)
{
	return isFileInFolder(file, dc_motor_path);
}

bool isServo(const string &file)
{
	return isFileInFolder(file, servo_path);
}

bool isBluetooth(const string &file)
{
	return isFileInFolder(file, bluetooth_path);
}

bool isForceSensor(const string &file)
{
	return isFileInFolder(file, forcesensor_path);
}

bool isCamera(const string &file)
{
	return isFileInFolder(file, camera_path);
}

bool isEncoder(const string &file)
{
	return isFileInFolder(file, encoder_path);
}

bool isFileInFolder(const string &file, const string &path)
{
	bool indicator = false;
	for (auto &entry : directory_iterator(path))
	{
		if (entry.path().string() == removeComponentPostfix(file))
		{
			indicator = true;
			break;
		}
	}
	return indicator;
}