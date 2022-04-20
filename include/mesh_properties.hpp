#ifndef MESH_PROPERTIES_HPP
#define MESH_PROPERTIES_HPP
#include <cstdint>
#include <map>
#include <string>

std::map<int8_t, int> extract_mesh_scales(const std::string& filename);

float percentage_at_scale(const std::map<int8_t, int>& mesh_scales, int8_t desired_scale);
#endif // MESH_PROPERTIES_HPP
