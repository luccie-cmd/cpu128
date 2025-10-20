#include <context.h>

namespace assembler {
Context::Context(std::string contents) {
    this->lexer = new Lexer(contents);
}
Context::~Context() {
    delete this->lexer;
}
void Context::start() {
    this->lexer->start();
    for (Token* tok : this->lexer->getTokens()) {
        std::printf("Tok: `%s`: %u\n", tok->getValue().c_str(), (uint32_t)tok->getType());
        delete tok;
    }
}
}; // namespace assembler