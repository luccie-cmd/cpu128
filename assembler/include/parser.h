#if !defined(_ASSEMBLER_PARSER_H_)
#define _ASSEMBLER_PARSER_H_
#include "lexer.h"
#include "ast.h"

namespace assembler {
class Parser {
  public:
    Parser(Lexer* lexer);
    ~Parser();
    Ast* getAst();

  private:
    AstNode* parseInstruction();
    AstNode* parseDeclaration();
    AstNode* parseNode();

    Token*              consume();
    Token*              expect(bool consume, TokenType expect);
    Token*              expect(bool consume, std::vector<TokenType> expecteds);
    Token*              peek(size_t offset);
    std::vector<Token*> tokenBuffer;
    size_t              tokenIdx;
    Token*              current;
    std::string         currentLabelDef;
};
}; // namespace assembler

#endif // _ASSEMBLER_PARSER_H_
