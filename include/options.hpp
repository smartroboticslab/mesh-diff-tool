/*
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Dimos Tzoumanikas
 * SPDX-FileCopyrightText: 2022-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <filesystem>
#include <iostream>
#include <string>

struct Options {
    std::filesystem::path source_mesh_path;
    std::filesystem::path target_mesh_path;
    std::filesystem::path heatmap_dir;
    std::filesystem::path tsv_file;
    double inlier_threshold = 0.05;
    int verbose = 0;
};

std::ostream& operator<<(std::ostream& os, const Options& options);

Options parse_options(int argc, char** argv);
