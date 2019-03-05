#ifndef __JENKS_BREAKS_HPP__
#define __JENKS_BREAKS_HPP__

#include <valarray>
#include <valarray>
#include <set>
#include <algorithm>
#include <iostream>

double inline variance(const std::valarray<double> &data_list);

double inline max_variance(const std::valarray<double> &data_list, const std::valarray<double> &kclass, std::vector<std::valarray<double>> &separated_data);

std::valarray<double> get_jenks_breaks(std::valarray<double> &data_list, int number_class);

std::vector<std::valarray<double>> auto_cluster(std::vector<double> &data_list, double max_var);

#endif // __MAPS_MLS_GRID_HPP__
