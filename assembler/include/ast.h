#if !defined(_ASSEMBLER_AST_H_)
#define _ASSEMBLER_AST_H_
#include <cstdint>
#include <vector>

namespace assembler {
enum struct AstNodeType {
    Direct,
    Expression,
    Instruction,
    Declaration,
};
class AstNode {
  public:
    AstNode(AstNodeType astType);
    virtual ~AstNode() = default;
    AstNodeType getAstNodeType();

  protected:
  private:
    AstNodeType astType;
};
enum struct ExpressionType {};
enum struct InstructionList : uint32_t {
    Mov   = (0b0000 << 16) | 1,
    Lad   = (0b0000 << 16) | 2,
    Zext  = (0b0000 << 16) | 3,
    Sext  = (0b0001 << 16) | 3,
    Xor   = (0b0000 << 16) | 4,
    Sub   = (0b0000 << 16) | 5,
    Add   = (0b0001 << 16) | 5,
    Asr   = (0b0000 << 16) | 6,
    Shr   = (0b0010 << 16) | 6,
    Shl   = (0b0011 << 16) | 6,
    And   = (0b0000 << 16) | 7,
    Or    = (0b0001 << 16) | 7,
    Mul   = (0b0000 << 16) | 8,
    Smul  = (0b0001 << 16) | 8,
    Cmp   = (0b0000 << 16) | 9,
    Call  = (0b0000 << 16) | 10,
    Ret   = (0b0001 << 16) | 10,
    Push  = (0b0000 << 16) | 11,
    Pop   = (0b0001 << 16) | 11,
    Jmp   = (0b0000 << 16) | 12,
    Jz    = (0b0001 << 16) | 12,
    Jnz   = (0b1001 << 16) | 12,
    Jc    = (0b0010 << 16) | 12,
    Jnc   = (0b1010 << 16) | 12,
    Je    = (0b0011 << 16) | 12,
    Jne   = (0b1011 << 16) | 12,
    Jl    = (0b0100 << 16) | 12,
    Jle   = (0b1100 << 16) | 12,
    Jg    = (0b0101 << 16) | 12,
    Jge   = (0b1101 << 16) | 12,
    Scall = (0b0000 << 16) | 13,
    Sret  = (0b0000 << 16) | 14,
    Wrptp = (0b0001 << 16) | 14,
    Rdptp = (0b0010 << 16) | 14,
    Wrclr = (0b0011 << 16) | 14,
    Rdclr = (0b0100 << 16) | 14,
    Wrcpr = (0b0101 << 16) | 14,
    Rdcpr = (0b0110 << 16) | 14,
};
struct Ast {
    std::vector<AstNode*> nodes;
};
}; // namespace assembler

#endif // _ASSEMBLER_AST_H_
