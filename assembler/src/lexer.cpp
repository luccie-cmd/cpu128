#include <algorithm>
#include <climits>
#include <cstring>
#include <lexer.h>
#include <unordered_map>
#define COMMENT_TOKEN ';'

static bool isContinueIdentifier(char c) {
    return ('a' <= c && c <= 'z') or ('A' <= c && c <= 'Z') or ('0' <= c && c <= '9') or (c == '_');
}

namespace assembler {
void Lexer::nextChar() {
    if (__glibc_unlikely(this->i >= this->data.size())) {
        this->c = 0;
    } else {
        this->c = this->data.at(this->i++);
    }
}
char Lexer::peek(size_t offset) {
    if (__glibc_unlikely(this->i + offset >= this->data.size())) {
        return 0;
    } else {
        return this->data.at(this->i + offset);
    }
}
void Lexer::stripWhitespace() {
    while (true) {
        if (this->c == COMMENT_TOKEN) {
            while (this->c != '\n' and this->c != 0) {
                this->nextChar();
            }
        }
        if (!isspace(this->c)) {
            return;
        }
        this->nextChar();
    }
}
std::unordered_map<std::string, TokenType> identifierLookup = {
    {"global", TokenType::Global},  {"extern", TokenType::Extern},  {"section", TokenType::Section},
    {"qword", TokenType::Qword},    {"dword", TokenType::Dword},    {"word", TokenType::Word},
    {"byte", TokenType::Byte},      {"db", TokenType::DirectByte},  {"dw", TokenType::DirectWord},
    {"dd", TokenType::DirectDword}, {"dq", TokenType::DirectQword}, {"rel", TokenType::Rel}};
Token* Lexer::lexIdentifier() {
    std::string tokenData(1, this->c);
    this->nextChar();
    while (isContinueIdentifier(this->c)) {
        tokenData.push_back(this->c);
        this->nextChar();
    }
    return new Token(tokenData, identifierLookup.contains(tokenData) ? identifierLookup.at(tokenData)
                                                                     : TokenType::Identifier);
}
Token* Lexer::lexNumber() {
    uint8_t base = 0;
    static_assert(sizeof(unsigned long) == sizeof(uint64_t),
                  "sizeof unsigned long is not equal to size of uint64_t");
    std::string_view view(this->data.data() + this->i - 1, this->data.size() - this->i);
    std::string      temp(view);
    char*            end   = nullptr;
    uint64_t         digit = strtoul(temp.c_str(), &end, base);
    uint64_t         oldI  = this->i;
    this->i += end - temp.c_str();
    if (this->i - oldI != 0) {
        this->i -= 1;
    }
    if (digit == ULONG_MAX) {
        std::printf("ERROR: Number out of range\n");
        std::exit(1);
    }
    this->nextChar();
    return new Token(std::to_string(digit), TokenType::LitNumber);
}
Token* Lexer::lexString() {
    this->nextChar();
    std::string strData;
    while (this->c != '"' and this->c != 0) {
        strData.push_back(this->c);
        this->nextChar();
    }
    this->nextChar();
    return new Token(strData, TokenType::LitString);
}
Token* Lexer::lexToken() {
    this->stripWhitespace();
    switch (this->c) {
    case 'a' ... 'z':
    case 'A' ... 'Z':
    case '_':
    case '.': {
        return this->lexIdentifier();
    } break;
    case '0' ... '9': {
        return this->lexNumber();
    } break;
    case '"': {
        return this->lexString();
    } break;
    case '[':
    case ']':
    case ',':
    case ':': {
        char ch = this->c;
        this->nextChar();
        return new Token(std::string(1, ch), (TokenType)ch);
    } break;
    default: {
        std::printf("Invalid character `%c` found\n", this->c);
        std::exit(1);
    } break;
    }
}
Lexer::Lexer(std::string contents) {
    char* newData = new char[contents.length()];
    std::memcpy((void*)newData, (void*)contents.c_str(), contents.length());
    this->data        = std::string_view(newData, contents.length());
    this->column      = 0;
    this->line        = 0;
    this->i           = 0;
    this->tokenBuffer = {};
    this->nextChar();
}
Lexer::~Lexer() {
    delete[] this->data.data();
}
void Lexer::start() {
    while (this->c != 0) {
        Token* newToken = this->lexToken();
        this->tokenBuffer.push_back(newToken);
    }
    Token* endToken = new Token(std::string("\\0"), TokenType::Eof);
    this->tokenBuffer.push_back(endToken);
}
std::vector<Token*> Lexer::getTokens() {
    return this->tokenBuffer;
}
}; // namespace assembler