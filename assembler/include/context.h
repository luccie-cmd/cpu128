#if !defined(_ASSEMBLER_CONTEXT_H_)
#define _ASSEMBLER_CONTEXT_H_
#include "lexer.h"
#include <string>

namespace assembler{
    class Context {
        public:
            Context(std::string contents);
            ~Context();
            void start();
        private:
            Lexer* lexer;
    };
};

#endif // _ASSEMBLER_CONTEXT_H_
