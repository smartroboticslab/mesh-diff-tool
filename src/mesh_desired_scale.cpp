#include <fstream>
#include <mesh_desired_scale.hpp>
#include <sstream>
#include <vector>

bool begins_with(const std::string& s, const std::string& prefix)
{
    if (s.size() >= prefix.size()) {
        return (s.compare(0, prefix.length(), prefix) == 0);
    }
    else {
        return false;
    }
}



std::string column(const std::string& s, size_t col)
{
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ' ')) {
        // Empty items result from consecutive occurences of the delimiter.
        if (!item.empty()) {
            elems.push_back(item);
        }
    }
    return col < elems.size() ? elems[col] : "";
}



std::map<int8_t, int> extract_mesh_scales(const std::string& filename)
{
    std::map<int8_t, int> scales;
    std::ifstream f(filename);
    // Parse the PLY header.
    bool element_face_found = false;
    bool property_scale_found = false;
    int property_scale_idx = 0;
    size_t num_faces = 0;
    size_t num_non_faces = 0;
    for (std::string line; std::getline(f, line) && line != "end_header";) {
        if (begins_with(line, "format binary ")) {
            // Can only handle ASCII PLY files.
            return scales;
        }
        if (begins_with(line, "element face ")) {
            element_face_found = true;
            num_faces = std::stoll(column(line, 2));
        }
        else if (element_face_found && !property_scale_found && begins_with(line, "property ")) {
            if (begins_with(line, "property char scale")) {
                property_scale_found = true;
            } else {
                property_scale_idx++;
            }
        }
        else if (!element_face_found && begins_with(line, "element ")) {
            num_non_faces += std::stoll(column(line, 2));
        }
    }
    // Skip the non-face lines.
    for (std::string line; num_non_faces > 0 && std::getline(f, line); num_non_faces--) {
    }
    // Iterate over the face lines.
    for (std::string line; num_faces > 0 && std::getline(f, line); num_faces--) {
        const int scale_col = std::stoi(column(line, 0)) + property_scale_idx;
        const int8_t scale = std::stoi(column(line, scale_col));
        try {
            scales.at(scale)++;
        }
        catch (std::out_of_range) {
            scales[scale] = 1;
        }
    }
    return scales;
}

float percentage_at_scale(const std::map<int8_t, int>& mesh_scales, int8_t desired_scale)
{
    long int total_data = 0;
    int desired_data = 0;
    for (const auto& e : mesh_scales) {
        total_data += e.second;
        if (e.first == desired_scale) {
            desired_data = e.second;
        }
    }
    return 100.0 * desired_data / total_data;
}
