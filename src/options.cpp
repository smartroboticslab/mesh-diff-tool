/*
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Dimos Tzoumanikas
 * SPDX-FileCopyrightText: 2022-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <argp.h>
#include <options.hpp>

static char doc[] =
    "Try to match all PLY meshes in SOURCE_PATH to those in TARGET_PATH. "
    "An error heatmap mesh will be created for each match and the per-mesh accuracy and "
    "completness will be saved to a TSV file.";
static char args_doc[] = "SOURCE_PATH TARGET_PATH";

static struct argp_option program_options[] = {
    {"heatmap-dir",
     'h',
     "PATH",
     0,
     "The directory where error heatmaps are saved. No heatmaps are generated by default."},
    {"tsv",
     't',
     "FILE",
     0,
     "The file where the TSV output will be saved. Defaults to SOURCE_PATH/mesh_comparison.tsv"},
    {"verbose",
     'v',
     0,
     0,
     "Produce verbose output. Multiple occurences increase the verbosity level"},
    {0, 0, 0, 0, 0}};

static error_t parse_opt(int key, char* arg, struct argp_state* state)
{
    struct Options* options = static_cast<Options*>(state->input);

    switch (key) {
    case 'h':
        options->heatmap_dir = arg;
        break;

    case 't':
        options->tsv_file = arg;
        break;

    case 'v':
        options->verbose++;
        break;

    case ARGP_KEY_ARG:
        switch (state->arg_num) {
        case 0:
            options->source_mesh_path = arg;
            break;
        case 1:
            options->target_mesh_path = arg;
            break;
        default:
            // Too many options.
            argp_usage(state);
        }
        break;

    case ARGP_KEY_END:
        if (state->arg_num < 2) {
            // Too few options.
            argp_usage(state);
        }
        break;

    default:
        return ARGP_ERR_UNKNOWN;
    }

    return 0;
}



std::ostream& operator<<(std::ostream& os, const Options& options)
{
    os << "Source mesh path:  " << options.source_mesh_path << "\n";
    os << "Target mesh path:  " << options.target_mesh_path << "\n";
    os << "Heatmap directory: " << options.heatmap_dir << "\n";
    os << "TSV file:          " << options.tsv_file << "\n";
    os << "Verbose:           " << options.verbose << "\n";
    return os;
}



Options parse_options(int argc, char** argv)
{
    Options options;
    struct argp argp_options = {program_options, parse_opt, args_doc, doc};
    argp_parse(&argp_options, argc, argv, 0, 0, &options);
    // Set the default option values.
    if (options.tsv_file.empty()) {
        options.tsv_file = options.source_mesh_path / "mesh_comparison.tsv";
    }
    if (options.verbose) {
        std::cout << options;
    }
    return options;
}
