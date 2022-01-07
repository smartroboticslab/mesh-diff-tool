#include <argp.h>
#include <compare_multiple_options.hpp>

static char doc[] =
    "Try to match all PLY meshes in SOURCE_DIR to those in TARGET_DIR. "
    "An error heatmap mesh will be created for each match and the per-mesh accuracy and "
    "completness will be saved to a TSV file.";
static char args_doc[] = "SOURCE_DIR TARGET_DIR";

static struct argp_option program_options[] = {
    {"heatmap-dir",
     'h',
     "DIR",
     0,
     "The directory where error heatmaps are saved. Defaults to SOURCE_DIR/heatmaps"},
    {"tsv",
     't',
     "FILE",
     0,
     "The file where the TSV output will be saved. Defaults to SOURCE_DIR/mesh_comparison.tsv"},
    {"verbose",
     'v',
     0,
     0,
     "Produce verbose output. Multiple occurences increase the verbosity level"},
    {0, 0, 0, 0, 0}};

static error_t parse_opt(int key, char* arg, struct argp_state* state)
{
    struct Options* options = static_cast<Options*>(state->input);
    bool heatmap_dir_supplied = false;
    bool tsv_file_supplied = false;

    switch (key) {
    case 'h':
        options->heatmap_dir = arg;
        heatmap_dir_supplied = true;
        break;

    case 't':
        options->tsv_file = arg;
        tsv_file_supplied = true;
        break;

    case 'v':
        options->verbose++;
        break;

    case ARGP_KEY_ARG:
        switch (state->arg_num) {
        case 0:
            options->source_mesh_dir = arg;
            break;
        case 1:
            options->target_mesh_dir = arg;
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

    // Set the default option values.
    if (!heatmap_dir_supplied) {
        options->heatmap_dir = options->source_mesh_dir / "heatmaps/";
    }
    if (!tsv_file_supplied) {
        options->tsv_file = options->source_mesh_dir / "mesh_comparison.tsv";
    }

    return 0;
}



std::ostream& operator<<(std::ostream& os, const Options& options)
{
    os << "Source mesh directory: " << options.source_mesh_dir << "\n";
    os << "Target mesh directory: " << options.target_mesh_dir << "\n";
    os << "Heatmap directory:     " << options.heatmap_dir << "\n";
    os << "TSV file:              " << options.tsv_file << "\n";
    os << "Verbose:               " << options.verbose << "\n";
    return os;
}



Options parse_options(int argc, char** argv)
{
    Options options;
    struct argp argp_options = {program_options, parse_opt, args_doc, doc};
    argp_parse(&argp_options, argc, argv, 0, 0, &options);
    if (options.verbose) {
        std::cout << options;
    }
    return options;
}
