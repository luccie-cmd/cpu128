#include <clopts.h>
#include <context.h>
#include <filesystem>

using namespace command_line_opts;
using namespace assembler;
std::string inputFile;
std::string outputFile;

void setOutput(std::string path) {
    if (!outputFile.empty()) {
        fprintf(stderr, "Cannot have multiple output files\n");
        exit(1);
    }
    outputFile = path;
}
int unknownArg(std::string path) {
    if (std::filesystem::exists(path)) {
        if (!inputFile.empty()) {
            fprintf(stderr, "Cannot have multiple input files\n");
            return 1;
        }
        inputFile = path;
        return 0;
    }
    return 1;
}

clopts_opt_t clopts = {{{"-o", setOutput, true}}, unknownArg};

int main(int argc, char** argv) {
    clopts.parse(argc, argv);
    std::string inFileContent = clopts.handleFile(inputFile);
    Context*    context       = new Context(inFileContent);
    context->start();
    delete context;
    return 0;
}