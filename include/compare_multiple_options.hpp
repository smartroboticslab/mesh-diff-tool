#include <filesystem>
#include <iostream>
#include <string>

struct Options {
    std::filesystem::path source_mesh_dir;
    std::filesystem::path target_mesh_dir;
    std::filesystem::path heatmap_dir;
    std::filesystem::path tsv_file;
    int verbose = 0;
};

std::ostream& operator<<(std::ostream& os, const Options& options);

Options parse_options(int argc, char** argv);
