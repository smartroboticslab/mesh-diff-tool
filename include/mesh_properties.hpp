/*
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Dimos Tzoumanikas
 * SPDX-FileCopyrightText: 2022-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MESH_PROPERTIES_HPP
#define MESH_PROPERTIES_HPP
#include <cstdint>
#include <map>
#include <numeric>
#include <string>
#include <vector>

std::map<int8_t, int> extract_mesh_scales(const std::string& filename);

std::vector<float> extract_mesh_distances(const std::string& filename);

float percentage_at_scale(const std::map<int8_t, int>& mesh_scales, int8_t desired_scale);

float percentage_at_distance(const std::vector<float>& mesh_distances, float desired_distance);

template<typename T>
T mean(const std::vector<T>& v)
{
    return std::reduce(v.begin(), v.end()) / v.size();
}
#endif // MESH_PROPERTIES_HPP
