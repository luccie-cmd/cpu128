#if !defined(_ASSEMBLER_LEXER_H_)
#define _ASSEMBLER_LEXER_H_
#include <string_view>
#include <token.h>
#include <vector>

namespace assembler {
class Lexer {
  public:
    Lexer(std::string inContent);
    ~Lexer();
    void                start();
    std::vector<Token*> getTokens();

  private:
    void                stripWhitespace();
    Token*              lexToken();
    Token*              lexIdentifier();
    Token*              lexNumber();
    Token*              lexString();
    void                nextChar();
    char                peek(size_t offset);
    size_t              column, line, i;
    std::vector<Token*> tokenBuffer;
    std::string_view    data;
    char                c;
};
}; // namespace assembler

#endif // _ASSEMBLER_LEXER_H_
