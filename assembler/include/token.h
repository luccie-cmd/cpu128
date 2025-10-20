#if !defined(_ASSEMBLER_TOKEN_H_)
#define _ASSEMBLER_TOKEN_H_
#include <cstdint>
#include <string>

namespace assembler {
enum struct TokenType : uint16_t {
    Invalid          = 0,
    Eof              = 1,
    Openparen        = '(',
    Closeparen       = ')',
    Openbracket      = '[',
    Closebracket     = ']',
    Openbrace        = '{',
    Closebrace       = '}',
    Colon            = ':',
    At               = '@',
    Comma            = ',',
    Less             = '<',
    Greater          = '>',
    Equal            = '=',
    Star             = '*',
    Plus             = '+',
    Minus            = '-',
    Slash            = '/',
    Hashtag          = '#',
    __MultibyteStart = 128,
    Identifier,
    LitString,
    LitNumber,
    __KeywordsStart,
    Global,
    Extern,
    Section,
    Qword,
    Dword,
    Word,
    Byte,
    Rel,
    DirectByte,
    DirectWord,
    DirectDword,
    DirectQword,
};
class Token {
  private:
    std::string value;
    TokenType   type;

  public:
    Token(std::string _value, TokenType _type) {
        this->value = _value;
        this->type  = _type;
    }
    std::string& getValue() {
        return this->value;
    };
    TokenType getType() {
        return this->type;
    };
    void setValue(std::string _value) {
        this->value = _value;
    }
    void setType(TokenType _type) {
        this->type = _type;
    }
};
}; // namespace assembler

#endif // _ASSEMBLER_TOKEN_H_
