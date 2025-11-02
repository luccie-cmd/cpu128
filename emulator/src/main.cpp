#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <queue>
#include <unordered_map>
#define LOW_WATER 0.3
#define HIGH_WATER 0.9
#define MAX_MICROCODE_QUEUE 64
#define MAX_INSTRUCTION_BUFFER 128

const uint8_t RAM[16777216] = {
    128, 130, 12,  0,   4,                                                // xor r0, r0 // 5 bytes
    128, 134, 12,  33,  4,                                                // xor r1, r1 // 5 bytes
    144, 137, 223, 128, 2,   0,   0,   0,  13,                            // lad r2, [rel hello] // 9 bytes. rel hello = +13
    160, 142, 0,   0,   1,   7,                                           // mov r3, byte 7      // 6 bytes
    0,   0,   12,  249, 255, 255, 255,                                    // jmp .loop // 7 bytes
    72,  101, 108, 108, 111, 44,  32,  87, 111, 114, 108, 100, 33, 10, 0, // db "Hello, World!", 0x0a, 0
    // 48,  0,   13, // scall // 3 bytes
};

constexpr uint64_t ramSize = sizeof(RAM);

enum struct MicroCodeOperationType : uint16_t {
    Size = 0, // Not an actual microop, but used for the size of the decoded instruction as microops.
    Offset,   // Not an actual microop, but used for the offset of the list of microops in the array.
    LoadI,
    LoadR,
    GenerateAddress, // BaseIn : 8,IndexIn : 8, scaleIn : 2, dispIn : 32, zero : 14 in data field as AGU
                     // paramters
    ReadMemory,
    WriteMemory,
    AddI,
    AddR,
    SubI,
    SubR,
    MulI,
    MulR,
    SmulI,
    SmulR,
    ShrI,
    ShrR,
    ShrSI,
    ShrSR,
    ShlI,
    ShlR,
    AndI,
    AndR,
    OrI,
    OrR,
    XorI,
    XorR,
    NotI,
    NotR,
    ReadFlag,
    BranchIf,
    BranchR,
    BranchI,
    Scall, // TODO: Remove this
};

enum struct Registers : uint16_t {
    R0  = 0,
    R29 = 29,
    RSP = 30,
    RIP = 31,
    RFLAGS,
    RCYCLE,
    RFETCHRIP,
    RFAULTRIP,
    RDECODERIP,
    RTEMP,
    READDR0,
    READDR1,
    READDR2,
    READDR3,
    READDR4,
    READDR5,
    READDR6,
    READDR7,
    Size,
};

enum struct Flags : uint8_t {
    Zero,
    Sign,
    Carry,
    Overflow,
    ExceptionEnable,
};

struct __attribute__((packed, aligned(128))) MicroCodeOperation {
    MicroCodeOperationType type;
    Registers              registerN; // 548 bits
    uint64_t               tag;
    uint64_t               dependsOn[3];
    uint64_t               data;
    uint64_t               mask;
    uint64_t               result;
    uint64_t               instructionIP;
    uint8_t                operandSize : 2;
    bool                   ready : 1;
    bool                   executed : 1;
};

struct RegisterStatusEntry {
    bool     busy;
    uint64_t producerTag;
};

RegisterStatusEntry registerStatus[(uint8_t)Registers::Size];

#define MOV_R64_I64_IDX 0
#define MOV_R64_R64_IDX 1
#define MOV_R64_M64_IDX 2
#define MOV_M64_R64_IDX 3
#define LAD_R64_M64_IDX 4
#define XOR_R64_I64_IDX 5
#define XOR_R64_R64_IDX 6
#define SUB_R64_I64_IDX 7
#define SUB_R64_R64_IDX 8
#define ADD_R64_I64_IDX 9
#define ADD_R64_R64_IDX 10
#define SHR_R64_I64_IDX 11
#define SHR_R64_R64_IDX 12
#define SHL_R64_I64_IDX 13
#define SHL_R64_R64_IDX 14
#define ASR_R64_I64_IDX 15
#define ASR_R64_R64_IDX 16
#define AND_R64_I64_IDX 17
#define AND_R64_R64_IDX 18
#define OR_R64_I64_IDX 19
#define OR_R64_R64_IDX 20
#define MUL_R64_R64_I64_IDX 21
#define MUL_R64_R64_R64_IDX 22
#define SMUL_R64_R64_I64_IDX 23
#define SMUL_R64_R64_R64_IDX 24
#define CMP_R64_R64_IDX 25
#define CALL_I64_IDX 26
#define RET_IDX 27
#define PUSH_R64_IDX 28
#define POP_IDX 29
#define JMP_I64_IDX 30
#define JZ_I64_IDX 31
#define JNZ_I64_IDX 32
#define JC_I64_IDX 33
#define JNC_I64_IDX 34
#define JE_I64_IDX 35
#define JNE_I64_IDX 36
#define JL_I64_IDX 37
#define JLE_I64_IDX 38
#define JG_I64_IDX 39
#define JGE_I64_IDX 40
#define SCALL_IDX 41
#define MAX_MICROCODE_SIZE (42 * 2)

MicroCodeOperation microcodeROM[] = {
    // Size and offset for `mov r64, i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE},
    // Size and offset for `mov r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 1},
    // Size and offset for `mov r64, m64`
    {.type = MicroCodeOperationType::Size, .data = 2},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 2},
    // Size and offset for `mov m64, r64`
    {.type = MicroCodeOperationType::Size, .data = 2},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 4},
    // Size and offset for `lad r64, m64`
    {.type = MicroCodeOperationType::Size, .data = 2},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 6},
    // Size and offset for `xor r64, i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 8},
    // Size and offset for `xor r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 9},
    // Size and offset for `sub r64, i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 10},
    // Size and offset for `sub r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 11},
    // Size and offset for `add r64, i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 12},
    // Size and offset for `add r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 13},
    // Size and offset for `shr r64, i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 14},
    // Size and offset for `shr r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 15},
    // Size and offset for `shl r64, i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 16},
    // Size and offset for `shl r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 17},
    // Size and offset for `asr r64, i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 18},
    // Size and offset for `asr r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 19},
    // Size and offset for `and r64, i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 20},
    // Size and offset for `and r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 21},
    // Size and offset for `or r64, i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 22},
    // Size and offset for `or r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 23},
    // Size and offset for `mul r64, r64, i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 24},
    // Size and offset for `mul r64, r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 25},
    // Size and offset for `smul r64, r64, i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 26},
    // Size and offset for `smul r64, r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 27},
    // Size and offset for `cmp r64, r64`
    {.type = MicroCodeOperationType::Size, .data = 2},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 28},
    // Size and offset for `call i64`
    {.type = MicroCodeOperationType::Size, .data = 4},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 30},
    // Size and offset for `ret`
    {.type = MicroCodeOperationType::Size, .data = 4},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 34},
    // Size and offset for `push r64`
    {.type = MicroCodeOperationType::Size, .data = 3},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 38},
    // Size and offset for `pop r64`
    {.type = MicroCodeOperationType::Size, .data = 3},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 41},
    // Size and offset for `jmp i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 44},
    // Size and offset for `jz i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 45},
    // Size and offset for `jnz i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 46},
    // Size and offset for `jc i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 47},
    // Size and offset for `jnc i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 48},
    // Size and offset for `je i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 49},
    // Size and offset for `jne i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 50},
    // Size and offset for `jl i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 51},
    // Size and offset for `jle i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 52},
    // Size and offset for `jg i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 53},
    // Size and offset for `jge i64`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 54},
    // Size and offset for `scall`
    {.type = MicroCodeOperationType::Size, .data = 1},
    {.type = MicroCodeOperationType::Offset, .data = MAX_MICROCODE_SIZE + 55},
    // mov r64, i64
    {.type = MicroCodeOperationType::LoadI},
    // mov r64, r64
    {.type = MicroCodeOperationType::LoadR},
    // mov r64, m64
    {.type = MicroCodeOperationType::GenerateAddress},
    {.type = MicroCodeOperationType::ReadMemory},
    // mov m64, r64
    {.type = MicroCodeOperationType::GenerateAddress},
    {.type = MicroCodeOperationType::WriteMemory},
    // lad r64, m64
    {.type = MicroCodeOperationType::GenerateAddress},
    {.type = MicroCodeOperationType::LoadR},
    // xor r64, i64
    {.type = MicroCodeOperationType::XorI},
    // xor r64, r64
    {.type = MicroCodeOperationType::XorR},
    // sub r64, i64
    {.type = MicroCodeOperationType::SubI},
    // sub r64, r64
    {.type = MicroCodeOperationType::SubR},
    // add r64, i64
    {.type = MicroCodeOperationType::AddI},
    // add r64, r64
    {.type = MicroCodeOperationType::AddR},
    // shr r64, i64
    {.type = MicroCodeOperationType::ShrI},
    // shr r64, r64
    {.type = MicroCodeOperationType::ShrR},
    // shl r64, i64
    {.type = MicroCodeOperationType::ShlI},
    // shl r64, r64
    {.type = MicroCodeOperationType::ShlR},
    // asr r64, i64
    {.type = MicroCodeOperationType::ShrSI},
    // asr r64, r64
    {.type = MicroCodeOperationType::ShrSR},
    // and r64, i64
    {.type = MicroCodeOperationType::AndI},
    // and r64, r64
    {.type = MicroCodeOperationType::AndR},
    // or r64, i64
    {.type = MicroCodeOperationType::OrI},
    // or r64, r64
    {.type = MicroCodeOperationType::OrR},
    // mul r64, r64, i64
    {.type = MicroCodeOperationType::MulI},
    // mul r64, r64, r64
    {.type = MicroCodeOperationType::MulR},
    // smul r64, r64, i64
    {.type = MicroCodeOperationType::SmulI},
    // smul r64, r64, r64
    {.type = MicroCodeOperationType::SmulR},
    // cmp r64, r64
    {.type = MicroCodeOperationType::LoadR, .registerN = Registers::RTEMP},
    {.type = MicroCodeOperationType::SubR, .registerN = Registers::RTEMP},
    // call i64
    {.type = MicroCodeOperationType::SubR, .registerN = Registers::RSP, .data = 8},
    {.type = MicroCodeOperationType::GenerateAddress, .data = (uint16_t)Registers::RSP},
    {.type = MicroCodeOperationType::WriteMemory, .registerN = Registers::RIP},
    {.type = MicroCodeOperationType::BranchI},
    // ret
    {.type = MicroCodeOperationType::GenerateAddress, .data = (uint16_t)Registers::RSP},
    {.type = MicroCodeOperationType::ReadMemory, .registerN = Registers::RTEMP},
    {.type = MicroCodeOperationType::AddR, .registerN = Registers::RSP, .data = 8},
    {.type = MicroCodeOperationType::BranchR, .registerN = Registers::RTEMP},
    // push r64
    {.type = MicroCodeOperationType::SubR, .registerN = Registers::RSP, .data = 8},
    {.type = MicroCodeOperationType::GenerateAddress, .data = (uint16_t)Registers::RSP},
    {.type = MicroCodeOperationType::WriteMemory},
    // pop r64
    {.type = MicroCodeOperationType::GenerateAddress, .data = (uint16_t)Registers::RSP},
    {.type = MicroCodeOperationType::ReadMemory},
    {.type = MicroCodeOperationType::AddR, .registerN = Registers::RSP, .data = 8},
    // jmp i64
    {.type = MicroCodeOperationType::BranchI},
    // jz i64
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::RTEMP, .data = (uint8_t)Flags::Zero},
    {.type = MicroCodeOperationType::BranchIf, .registerN = (Registers)JZ_I64_IDX},
    // jnz i64
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::RTEMP, .data = (uint8_t)Flags::Zero},
    {.type = MicroCodeOperationType::BranchIf, .registerN = (Registers)JNZ_I64_IDX},
    // jc i64
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::RTEMP, .data = (uint8_t)Flags::Carry},
    {.type = MicroCodeOperationType::BranchIf, .registerN = (Registers)JC_I64_IDX},
    // jnc i64
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::RTEMP, .data = (uint8_t)Flags::Carry},
    {.type = MicroCodeOperationType::BranchIf, .registerN = (Registers)JNC_I64_IDX},
    // je i64
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::RTEMP, .data = (uint8_t)Flags::Zero},
    {.type = MicroCodeOperationType::BranchIf, .registerN = (Registers)JE_I64_IDX},
    // jne i64
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::RTEMP, .data = (uint8_t)Flags::Zero},
    {.type = MicroCodeOperationType::BranchIf, .registerN = (Registers)JNE_I64_IDX},
    // jl i64
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::RTEMP, .data = (uint8_t)Flags::Sign},
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::READDR, .data = (uint8_t)Flags::Overflow},
    // {.type = MicroCodeOperationType::XorR, .registerN = Registers::RTEMP, .data = (uint16_t)Registers::READDR},
    {.type = MicroCodeOperationType::BranchIf, .registerN = (Registers)JL_I64_IDX},
    // jle i64
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::RTEMP, .data = (uint8_t)Flags::Sign},
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::READDR, .data = (uint8_t)Flags::Overflow},
    // {.type = MicroCodeOperationType::XorR, .registerN = Registers::RTEMP, .data = (uint16_t)Registers::READDR},
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::READDR, .data = (uint8_t)Flags::Zero},
    // {.type = MicroCodeOperationType::OrR, .registerN = Registers::RTEMP, .data = (uint16_t)Registers::READDR},
    {.type = MicroCodeOperationType::BranchIf, .registerN = (Registers)JLE_I64_IDX},
    // jg i64
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::RTEMP, .data = (uint8_t)Flags::Sign},
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::READDR, .data = (uint8_t)Flags::Overflow},
    // {.type = MicroCodeOperationType::XorR, .registerN = Registers::RTEMP, .data = (uint16_t)Registers::READDR},
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::READDR, .data = (uint8_t)Flags::Zero},
    // {.type = MicroCodeOperationType::NotR, .registerN = Registers::READDR},
    // {.type = MicroCodeOperationType::AndR, .registerN = Registers::RTEMP, .data = (uint16_t)Registers::READDR},
    {.type = MicroCodeOperationType::BranchIf, .registerN = (Registers)JG_I64_IDX},
    // jge i64
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::RTEMP, .data = (uint8_t)Flags::Sign},
    // {.type = MicroCodeOperationType::ReadFlag, .registerN = Registers::READDR, .data = (uint8_t)Flags::Overflow},
    // {.type = MicroCodeOperationType::XorR, .registerN = Registers::RTEMP, .data = (uint16_t)Registers::READDR},
    {.type = MicroCodeOperationType::BranchIf, .registerN = (Registers)JGE_I64_IDX},
    // scall
    {.type = MicroCodeOperationType::Scall},
};

struct __attribute__((packed)) OpcodeSpecificDecode {
    uint8_t opcodeId : 4;
    uint8_t opType : 4;
};

struct __attribute__((packed)) MRPrefix {
    uint8_t hasMemoryPrefix : 1;
    uint8_t hasRegisterPrefix : 1;
    uint8_t destRegister : 5;
    uint8_t hasDestReg : 1; // If this is 0, destination register should also be 0
};

struct __attribute__((packed)) RegisterPrefix {
    uint16_t registerSrc2 : 5;
    uint16_t registerSrc1 : 5;
    uint16_t operandSize : 2;
    uint16_t zero : 4;
};

struct __attribute__((packed)) MemoryPrefix {
    uint16_t scaleBase2 : 2;
    uint16_t indexRegister : 5;
    uint16_t baseRegister : 5;
    uint16_t memoryMode : 2;
    uint16_t operandSize : 2;
};

union Prefixes {
    uint16_t       bits;
    RegisterPrefix registerPrefix;
    MemoryPrefix   memoryPrefix;
};

struct __attribute__((packed)) RegisterFile {
    uint64_t GP[30];
    uint64_t SP;
    uint64_t IP;
    uint64_t flags;
    uint64_t cycle;
    uint64_t fetchRIP;
    uint64_t faultRIP;
    uint64_t decodeRIP;
    uint64_t temp;
    uint64_t eaddr[8];
};

struct CPUState {
    // Register file
    RegisterFile registerFile;
};

struct CacheLine {
    std::array<uint8_t, 64> data;
    bool                    dirty      = false;
    uint64_t                lastAccess = 0; // For LRU
};

// Key = address, Value = L1 address
std::unordered_map<uint64_t, CacheLine*> L1Cache;
// Key = address, Value = L2 address
std::unordered_map<uint64_t, CacheLine*> L2Cache;
// Key = address, Value = L3 address
std::unordered_map<uint64_t, CacheLine*> L3Cache;
// std::array<uint8_t, 16 * 1024 * 1024> RAMArray;
std::queue<uint8_t>             instructionBuffer;
std::vector<MicroCodeOperation> microcodeBuffer;
bool                            canFetch   = true;
bool                            canDecode  = true;
uint64_t                        decoderTag = 1;
CPUState                        state;

template <typename T>
T getQueueNext(std::queue<T>& q) {
    T elem = q.front();
    q.pop();
    return elem;
}

CacheLine* CPUMemoryFetchLine(uint64_t address) {
    if ((address & 0x3F) != 0) {
        printf("ERROR: Unaligned loads\n");
        exit(1);
    }
    if (L1Cache.contains(address)) {
        L1Cache.at(address)->lastAccess = state.registerFile.cycle;
        return L1Cache.at(address);
    }
    if (L2Cache.contains(address)) {
        L1Cache.insert_or_assign(address, L2Cache.at(address));
        L2Cache.at(address)->lastAccess = state.registerFile.cycle;
        return L2Cache.at(address);
    }
    if (L3Cache.contains(address)) {
        L2Cache.insert_or_assign(address, L3Cache.at(address));
        L3Cache.at(address)->lastAccess = state.registerFile.cycle;
        return L3Cache.at(address);
    }
    CacheLine* line = new CacheLine;
    uint64_t   base = address & ~(63ULL); // 64-byte line
    for (int i = 0; i < 64; ++i) {
        uint64_t addr = base + i;
        if (addr < ramSize) {
            line->data[i] = RAM[addr];
        } else {
            line->data[i] = 0; // fill missing bytes
        }
    }
    L3Cache.insert_or_assign(address, line);
    L3Cache.at(address)->lastAccess = state.registerFile.cycle;
    return line;
}

class LoadStoreUnit {
  public:
    // === LOAD OPERATIONS - BLOCKING BY NATURE ===

    uint8_t loadByte(uint64_t address) {
        CacheLine* line   = CPUMemoryFetchLine(address & ~0x3FULL);
        uint8_t    offset = address & 0x3F;
        return line->data[offset];
    }

    uint16_t loadHalfWord(uint64_t address) {
        uint64_t line_base = address & ~0x3FULL;
        uint8_t  offset    = address & 0x3F;

        if ((address & 0x1) == 0 && (offset + 1) < 64) {
            // Aligned access
            CacheLine* line = CPUMemoryFetchLine(line_base);
            return *reinterpret_cast<uint16_t*>(&line->data[offset]);
        } else {
            // Unaligned
            return loadUnaligned16(address);
        }
    }

    uint32_t loadWord(uint64_t address) {
        uint64_t line_base = address & ~0x3FULL;
        uint8_t  offset    = address & 0x3F;

        if ((address & 0x3) == 0 && (offset + 3) < 64) {
            CacheLine* line = CPUMemoryFetchLine(line_base);
            return *reinterpret_cast<uint32_t*>(&line->data[offset]);
        } else {
            return loadUnaligned32(address);
        }
    }

    uint64_t loadDoubleWord(uint64_t address) {
        uint64_t line_base = address & ~0x3FULL;
        uint8_t  offset    = address & 0x3F;

        if ((address & 0x7) == 0 && (offset + 7) < 64) {
            CacheLine* line = CPUMemoryFetchLine(line_base);
            return *reinterpret_cast<uint64_t*>(&line->data[offset]);
        } else if (offset <= 56) {
            // Unaligned but same cache line
            CacheLine* line = CPUMemoryFetchLine(line_base);
            return loadUnaligned64SameLine(line, offset);
        } else {
            // Crosses cache line boundary
            return loadUnaligned64CrossLine(address);
        }
    }

    // === STORE OPERATIONS - IMMEDIATE ===

    void storeByte(uint64_t address, uint8_t value) {
        CacheLine* line    = CPUMemoryFetchLine(address & ~0x3FULL);
        uint8_t    offset  = address & 0x3F;
        line->data[offset] = value;
        line->dirty        = true;
    }

    void storeHalfWord(uint64_t address, uint16_t value) {
        uint64_t line_base = address & ~0x3FULL;
        uint8_t  offset    = address & 0x3F;

        if ((address & 0x1) == 0 && (offset + 1) < 64) {
            CacheLine* line                                   = CPUMemoryFetchLine(line_base);
            *reinterpret_cast<uint16_t*>(&line->data[offset]) = value;
            line->dirty                                       = true;
        } else {
            storeUnaligned16(address, value);
        }
    }

    void storeWord(uint64_t address, uint32_t value) {
        uint64_t line_base = address & ~0x3FULL;
        uint8_t  offset    = address & 0x3F;

        if ((address & 0x3) == 0 && (offset + 3) < 64) {
            CacheLine* line                                   = CPUMemoryFetchLine(line_base);
            *reinterpret_cast<uint32_t*>(&line->data[offset]) = value;
            line->dirty                                       = true;
        } else {
            storeUnaligned32(address, value);
        }
    }

    void storeDoubleWord(uint64_t address, uint64_t value) {
        uint64_t line_base = address & ~0x3FULL;
        uint8_t  offset    = address & 0x3F;

        if ((address & 0x7) == 0 && (offset + 7) < 64) {
            CacheLine* line                                   = CPUMemoryFetchLine(line_base);
            *reinterpret_cast<uint64_t*>(&line->data[offset]) = value;
            line->dirty                                       = true;
        } else if (offset <= 56) {
            storeUnaligned64SameLine(address, value);
        } else {
            storeUnaligned64CrossLine(address, value);
        }
    }

  private:
    // === LOAD HELPERS ===

    uint16_t loadUnaligned16(uint64_t address) {
        uint8_t b1 = loadByte(address);
        uint8_t b2 = loadByte(address + 1);
        return (b2 << 8) | b1;
    }

    uint32_t loadUnaligned32(uint64_t address) {
        uint16_t w1 = loadUnaligned16(address);
        uint16_t w2 = loadUnaligned16(address + 2);
        return (w2 << 16) | w1;
    }

    uint64_t loadUnaligned64SameLine(CacheLine* line, uint8_t offset) {
        uint64_t result = 0;
        std::memcpy(&result, &(line->data[offset]), sizeof(result));
        return result;
    }

    uint64_t loadUnaligned64CrossLine(uint64_t address) {
        uint64_t line1_base = address & ~0x3FULL;
        uint64_t line2_base = (address + 7) & ~0x3FULL;

        CacheLine* line1 = CPUMemoryFetchLine(line1_base);
        CacheLine* line2 = CPUMemoryFetchLine(line2_base);

        uint8_t offset1        = address & 0x3F;
        uint8_t bytes_in_line1 = 64 - offset1;

        uint64_t result = 0;
        for (int i = 0; i < bytes_in_line1; i++) {
            result |= static_cast<uint64_t>(line1->data[offset1 + i]) << (i * 8);
        }
        for (int i = 0; i < (8 - bytes_in_line1); i++) {
            result |= static_cast<uint64_t>(line2->data[i]) << ((bytes_in_line1 + i) * 8);
        }
        return result;
    }

    // === STORE HELPERS ===

    void storeUnaligned16(uint64_t address, uint16_t value) {
        storeByte(address, value & 0xFF);
        storeByte(address + 1, (value >> 8) & 0xFF);
    }

    void storeUnaligned32(uint64_t address, uint32_t value) {
        storeUnaligned16(address, value & 0xFFFF);
        storeUnaligned16(address + 2, (value >> 16) & 0xFFFF);
    }

    void storeUnaligned64SameLine(uint64_t address, uint64_t value) {
        CacheLine* line   = CPUMemoryFetchLine(address & ~0x3FULL);
        uint8_t    offset = address & 0x3F;

        *reinterpret_cast<uint32_t*>(&line->data[offset])     = value & 0xFFFFFFFF;
        *reinterpret_cast<uint32_t*>(&line->data[offset + 4]) = (value >> 32) & 0xFFFFFFFF;
        line->dirty                                           = true;
    }

    void storeUnaligned64CrossLine(uint64_t address, uint64_t value) {
        uint64_t line1_base = address & ~0x3FULL;
        uint64_t line2_base = (address + 7) & ~0x3FULL;

        CacheLine* line1 = CPUMemoryFetchLine(line1_base);
        CacheLine* line2 = CPUMemoryFetchLine(line2_base);

        uint8_t offset1        = address & 0x3F;
        uint8_t bytes_in_line1 = 64 - offset1;

        for (int i = 0; i < bytes_in_line1; i++) {
            line1->data[offset1 + i] = (value >> (i * 8)) & 0xFF;
        }
        line1->dirty = true;

        for (int i = 0; i < (8 - bytes_in_line1); i++) {
            line2->data[i] = (value >> ((bytes_in_line1 + i) * 8)) & 0xFF;
        }
        line2->dirty = true;
    }
};

LoadStoreUnit* LSU = new LoadStoreUnit();

void fetch() {
    if (!canFetch) {
        return;
    }
    if ((instructionBuffer.size() + 8) / MAX_INSTRUCTION_BUFFER >= 1.0) {
        return;
    }
    if (state.registerFile.fetchRIP % 8 == 0) {
        auto qWord = LSU->loadDoubleWord(state.registerFile.fetchRIP);
        for (size_t i = 0; i < sizeof(qWord); ++i) {
            instructionBuffer.push((qWord >> (i * 8)) & 0xFF);
        }
        state.registerFile.fetchRIP += sizeof(qWord);
    } else {
        while (state.registerFile.fetchRIP % 8 != 0) {
            uint8_t byte = LSU->loadByte(state.registerFile.fetchRIP);
            instructionBuffer.push(byte);
            state.registerFile.fetchRIP++;
        }
    }
}

uint16_t getMicroOpIndex(uint8_t opcode, OpcodeSpecificDecode opSpecific) {
    switch (opcode) {
    // Mov
    case 0: {
        // mov r64, i64
        if (opSpecific.opType == 0b1010) {
            return MOV_R64_I64_IDX;
        }
        // mov r64, r64
        if (opSpecific.opType == 0b1000) {
            return MOV_R64_R64_IDX;
        }
        // mov r64, m64
        if (opSpecific.opType == 0b1001) {
            return MOV_R64_M64_IDX;
        }
        // mov m64, r64
        if (opSpecific.opType == 0b1011) {
            return MOV_M64_R64_IDX;
        }
    } break;
    // Lad
    case 1: {
        // Lad r64, m64
        if (opSpecific.opType == 0b1001) {
            return LAD_R64_M64_IDX;
        }
    } break;
    // Xor
    case 3: {
        // xor r64, i64
        if (opSpecific.opType == 0b1000) {
            return XOR_R64_R64_IDX;
        }
        // xor r64, r64
        if (opSpecific.opType == 0b1010) {
            return XOR_R64_I64_IDX;
        }
    } break;
    // And/Or
    case 6: {
        if (opSpecific.opType == 0b1000) {
            // And r64, r64
            if (opSpecific.opcodeId == 0) {
                return AND_R64_R64_IDX;
            }
            // Or r64, r64
            if (opSpecific.opcodeId == 1) {
                return OR_R64_R64_IDX;
            }
        }
        if (opSpecific.opType == 0b1010) {
            // And r64, r64
            if (opSpecific.opcodeId == 0) {
                return AND_R64_I64_IDX;
            }
            // Or r64, r64
            if (opSpecific.opcodeId == 1) {
                return OR_R64_I64_IDX;
            }
        }
    } break;
    case 11: {
        if (opSpecific.opType == 0) {
            return JMP_I64_IDX;
        }
    } break;
    case 12: {
        return SCALL_IDX;
    } break;
    }
    printf("TODO: Correctly decode opcode %u\n", opcode + 1);
    exit(1);
}

uint64_t packAGUParams(MemoryPrefix mem, uint32_t disp) {
    // BaseIn : 8,IndexIn : 8, scaleIn : 2, dispIn : 32, zero : 14
    uint8_t base  = mem.baseRegister;
    uint8_t index = 0xFF;
    if (mem.memoryMode >= 0b10) {
        index = mem.indexRegister;
    }
    uint8_t scale = mem.scaleBase2;
    return (uint64_t)base | ((uint64_t)index << 8) | ((uint64_t)scale << 16) | ((uint64_t)disp << 18);
}

Registers allocateEaddrReg() {
    static uint8_t    i         = 0;
    constexpr uint8_t maxEaddrs = 8;
    for (uint8_t attempts = 0; attempts < maxEaddrs; ++attempts) {
        uint8_t   idx      = (i + attempts) % maxEaddrs;
        Registers eaddrReg = (Registers)((uint16_t)Registers::READDR0 + idx);
        if (!registerStatus[(uint16_t)eaddrReg].busy) { // track busy/free
            i = idx + 1;
            return eaddrReg;
        }
    }
    std::printf("We need more EADDRs\n");
    std::exit(1);
}

void decode() {
    if (!canDecode) {
        return;
    }
    if (state.registerFile.faultRIP != (uint64_t)-1) {
        canFetch  = false;
        canDecode = false;
        return;
    }
    if (instructionBuffer.size() / MAX_INSTRUCTION_BUFFER <= LOW_WATER) {
        // printf("Low water: continueing fetcher\n");
        canFetch = true;
    }
    if (instructionBuffer.size() / MAX_INSTRUCTION_BUFFER >= HIGH_WATER) {
        // printf("High water: stopping fetcher\n");
        canFetch = false;
    }
    if (instructionBuffer.size() < 12) {
        // printf("Too little bytes, stalling and forcing fetch enable\n");
        canFetch = true;
        return;
    }
    uint64_t             oldInstBufferSize = instructionBuffer.size();
    uint8_t              byte              = getQueueNext(instructionBuffer);
    OpcodeSpecificDecode opSpecifcByte     = *(OpcodeSpecificDecode*)&byte;
    byte                                   = getQueueNext(instructionBuffer);
    MRPrefix MRprefixByte                  = *(MRPrefix*)&byte;
    uint8_t  opcodeByte                    = 0;
    Prefixes prefix;
    prefix.bits        = 0;
    uint64_t immediate = 0;
    if (MRprefixByte.hasMemoryPrefix or MRprefixByte.hasRegisterPrefix) {
        byte           = getQueueNext(instructionBuffer);
        uint8_t  byte2 = getQueueNext(instructionBuffer);
        uint16_t word  = (byte << 8) | byte2;
        prefix.bits    = word;
        opcodeByte     = getQueueNext(instructionBuffer);
        if (MRprefixByte.hasMemoryPrefix) {
            byte          = getQueueNext(instructionBuffer);
            byte2         = getQueueNext(instructionBuffer);
            uint8_t byte3 = getQueueNext(instructionBuffer);
            uint8_t byte4 = getQueueNext(instructionBuffer);
            immediate     = (byte << 24) | (byte2 << 16) | (byte3 << 8) | byte4;
        }
    } else {
        opcodeByte = getQueueNext(instructionBuffer);
    }
    if (opcodeByte == 0 or opcodeByte > 15) {
        state.registerFile.faultRIP = state.registerFile.decodeRIP;
        canDecode                   = false;
        canFetch                    = false;
        return;
    }
    if (opSpecifcByte.opType == 0 or opSpecifcByte.opType == 0b1010) {
        uint8_t opSize = MRprefixByte.hasMemoryPrefix ? prefix.memoryPrefix.operandSize
                                                      : (MRprefixByte.hasRegisterPrefix ? prefix.registerPrefix.operandSize : 0b10);
        byte           = getQueueNext(instructionBuffer);
        uint8_t byte2  = 0;
        uint8_t byte3  = 0;
        uint8_t byte4  = 0;
        uint8_t byte5  = 0;
        uint8_t byte6  = 0;
        uint8_t byte7  = 0;
        uint8_t byte8  = 0;
        if (opSize >= 1) {
            byte2 = getQueueNext(instructionBuffer);
        }
        if (opSize >= 2) {
            byte3 = getQueueNext(instructionBuffer);
            byte4 = getQueueNext(instructionBuffer);
        }
        if (opSize == 3) {
            byte5 = getQueueNext(instructionBuffer);
            byte6 = getQueueNext(instructionBuffer);
            byte7 = getQueueNext(instructionBuffer);
            byte8 = getQueueNext(instructionBuffer);
        }
        if (byte & 0x80) {
            if (opSize < 3) {
                byte5 = byte6 = byte7 = byte8 = 0xff;
            }
            if (opSize < 2) {
                byte3 = byte4 = 0xff;
            }
            if (opSize < 1) {
                byte2 = 0xff;
            }
        }
        immediate = ((uint64_t)byte8 << 56) | ((uint64_t)byte7 << 48) | ((uint64_t)byte6 << 40) | ((uint64_t)byte5 << 32) | ((uint64_t)byte4 << 24) |
                    ((uint64_t)byte3 << 16) | ((uint64_t)byte2 << 8) | (uint64_t)byte;
    }
    uint16_t            microcodeIdx = getMicroOpIndex(opcodeByte - 1, opSpecifcByte) * 2;
    uint8_t             size         = microcodeROM[microcodeIdx].data & 0xFF;
    uint8_t             offset       = microcodeROM[microcodeIdx + 1].data;
    MicroCodeOperation* sequence     = new MicroCodeOperation[size + 1];
    memcpy(sequence + 1, &microcodeROM[offset], size * sizeof(MicroCodeOperation));
    sequence[0] = {
        .type          = MicroCodeOperationType::AddI,
        .registerN     = Registers::RIP,
        .tag           = decoderTag,
        .data          = oldInstBufferSize - instructionBuffer.size(),
        .instructionIP = state.registerFile.decodeRIP,
        .operandSize   = 3,
        .executed      = false,
    };
    if (registerStatus[(uint16_t)Registers::RIP].busy) {
        sequence[0].dependsOn[0] = registerStatus[(uint16_t)Registers::RIP].producerTag;
    }
    registerStatus[(uint16_t)Registers::RIP].producerTag = decoderTag;
    sequence[0].ready                                    = sequence[0].dependsOn[0] == 0;
    if (sequence[0].operandSize != 3) {
        sequence[0].mask = (1ULL << (8 << sequence[0].operandSize)) - 1;
    }
    decoderTag++;
    microcodeBuffer.push_back(sequence[0]);
    Registers lastAGUReg = Registers::R0;
    bool      aguSet     = false;
    for (uint8_t i = 1; i < size + 1; i++) {
        if (aguSet) {
            aguSet     = false;
            lastAGUReg = Registers::R0;
        }
        if (lastAGUReg != Registers::R0) {
            aguSet = true;
        }
        switch (sequence[i].type) {
        case MicroCodeOperationType::LoadI: {
            if (sequence[i].registerN == Registers::R0) {
                sequence[i].registerN = (Registers)MRprefixByte.destRegister;
            }
            sequence[i].operandSize = MRprefixByte.hasMemoryPrefix ? prefix.memoryPrefix.operandSize
                                                                   : (MRprefixByte.hasRegisterPrefix ? prefix.registerPrefix.operandSize : 0b11);
            if (aguSet) {
                sequence[i].data = (uint64_t)lastAGUReg;
            }
            if (sequence[i].data == 0) {
                sequence[i].data = immediate;
            }
            sequence[i].ready = true;
        } break;
        case MicroCodeOperationType::LoadR: {
            if (sequence[i].registerN == Registers::R0) {
                sequence[i].registerN = (Registers)MRprefixByte.destRegister;
            }
            sequence[i].operandSize = MRprefixByte.hasMemoryPrefix ? prefix.memoryPrefix.operandSize
                                                                   : (MRprefixByte.hasRegisterPrefix ? prefix.registerPrefix.operandSize : 0b11);
            if (aguSet) {
                sequence[i].data = (uint64_t)lastAGUReg;
            }
            if (sequence[i].data == 0) {
                sequence[i].data = prefix.registerPrefix.registerSrc1;
            }
            if (registerStatus[(uint16_t)sequence[i].registerN].busy) {
                sequence[i].dependsOn[0] = registerStatus[(uint16_t)sequence[i].registerN].producerTag;
            }
        } break;
        case MicroCodeOperationType::GenerateAddress: {
            if (sequence[i].registerN == Registers::R0) {
                lastAGUReg            = allocateEaddrReg();
                sequence[i].registerN = lastAGUReg;
            }
            sequence[i].operandSize = 0b11;
            if (sequence[i].data == 0) {
                sequence[i].data = packAGUParams(prefix.memoryPrefix, immediate & 0xFFFFFFFF);
            }
            if (registerStatus[prefix.memoryPrefix.baseRegister].busy) {
                sequence[i].dependsOn[0] = registerStatus[prefix.memoryPrefix.baseRegister].producerTag;
            }
            if (prefix.memoryPrefix.memoryMode >= 0b10) {
                if (registerStatus[prefix.memoryPrefix.indexRegister].busy) {
                    sequence[i].dependsOn[1] = registerStatus[prefix.memoryPrefix.indexRegister].producerTag;
                }
            }
        } break;
        case MicroCodeOperationType::ReadMemory: {
            if (!aguSet) {
                std::printf("ICE: ReadMemory doesn't have a GenerateAddress pred\n");
                std::exit(1);
            }
            if (sequence[i].registerN == Registers::R0) {
                sequence[i].registerN = (Registers)MRprefixByte.destRegister;
            }
            sequence[i].operandSize = MRprefixByte.hasMemoryPrefix ? prefix.memoryPrefix.operandSize
                                                                   : (MRprefixByte.hasRegisterPrefix ? prefix.registerPrefix.operandSize : 0b11);
            if (registerStatus[(uint16_t)lastAGUReg].busy) {
                sequence[i].dependsOn[0] = registerStatus[(uint16_t)lastAGUReg].producerTag;
            }
        } break;
        case MicroCodeOperationType::WriteMemory: {
            if (!aguSet) {
                std::printf("ICE: WriteMemory doesn't have a GenerateAddress pred\n");
                std::exit(1);
            }
            if (sequence[i].registerN == Registers::R0) {
                sequence[i].registerN = (Registers)MRprefixByte.destRegister;
            }
            sequence[i].operandSize = MRprefixByte.hasMemoryPrefix ? prefix.memoryPrefix.operandSize
                                                                   : (MRprefixByte.hasRegisterPrefix ? prefix.registerPrefix.operandSize : 0b11);
            if (registerStatus[(uint16_t)lastAGUReg].busy) {
                sequence[i].dependsOn[0] = registerStatus[(uint16_t)lastAGUReg].producerTag;
            }
            if (registerStatus[MRprefixByte.destRegister].busy) {
                sequence[i].dependsOn[1] = registerStatus[MRprefixByte.destRegister].producerTag;
            }
        } break;
        case MicroCodeOperationType::XorR: {
            if (aguSet) {
                sequence[i].registerN = lastAGUReg;
            }
            if (sequence[i].registerN == Registers::R0) {
                sequence[i].registerN = (Registers)MRprefixByte.destRegister;
            }
            sequence[i].operandSize = prefix.registerPrefix.operandSize;
            if (sequence[i].data == 0) {
                sequence[i].data = prefix.registerPrefix.registerSrc1;
            }
            if (registerStatus[MRprefixByte.destRegister].busy) {
                sequence[i].dependsOn[0] = registerStatus[MRprefixByte.destRegister].producerTag;
            }
            if (registerStatus[prefix.registerPrefix.registerSrc1].busy) {
                sequence[i].dependsOn[1] = registerStatus[prefix.registerPrefix.registerSrc1].producerTag;
            }
        } break;
        case MicroCodeOperationType::AndR: {
            if (aguSet) {
                sequence[i].registerN = lastAGUReg;
            }
            if (sequence[i].registerN == Registers::R0) {
                sequence[i].registerN = (Registers)MRprefixByte.destRegister;
            }
            sequence[i].operandSize = prefix.registerPrefix.operandSize;
            if (sequence[i].data == 0) {
                sequence[i].data = prefix.registerPrefix.registerSrc1;
            }
            if (registerStatus[MRprefixByte.destRegister].busy) {
                sequence[i].dependsOn[0] = registerStatus[MRprefixByte.destRegister].producerTag;
            }
            if (registerStatus[prefix.registerPrefix.registerSrc1].busy) {
                sequence[i].dependsOn[1] = registerStatus[prefix.registerPrefix.registerSrc1].producerTag;
            }
        } break;
        case MicroCodeOperationType::BranchI: {
            sequence[i].registerN = Registers::RIP;
            if (sequence[i].data == 0) {
                sequence[i].data = immediate;
            }
        } break;
        case MicroCodeOperationType::Scall: {
            sequence[i].registerN = Registers::RIP;
        } break;
        default: {
            printf("TODO: Support %u microcode decoding\n", (uint16_t)sequence[i].type);
            exit(1);
        } break;
        }
        if (sequence[i].operandSize != 3) {
            sequence[i].mask = (1ULL << (8 << sequence[i].operandSize)) - 1;
        }
        registerStatus[(uint16_t)sequence[i].registerN] = {.busy = true, .producerTag = decoderTag};
        uint64_t zeroes[3]                              = {0, 0, 0};
        sequence[i].ready                               = std::memcmp(sequence[i].dependsOn, zeroes, sizeof(zeroes)) == 0;
        sequence[i].tag                                 = decoderTag++;
        sequence[i].instructionIP                       = state.registerFile.decodeRIP;
        microcodeBuffer.push_back(sequence[i]);
    }
    state.registerFile.decodeRIP += oldInstBufferSize - instructionBuffer.size();
}

uint64_t AGUCalculate(uint64_t base, uint64_t index, uint8_t scale, int32_t disp) {
    return (uint64_t)(base + (index * scale) + disp);
}

enum struct ALUOperation {
    Add,
    Xor,
    And,
};

uint64_t ALUCalculate(uint64_t A, uint64_t B, ALUOperation operation) {
    switch (operation) {
    case ALUOperation::Add: {
        return A + B;
    }
    case ALUOperation::Xor: {
        return A ^ B;
    }
    case ALUOperation::And: {
        return A & B;
    }
    }
    printf("Unreachable %s:%d\n", __FILE_NAME__, __LINE__);
    exit(1);
}

void pipelineFlush() {
    while (!microcodeBuffer.empty()) microcodeBuffer.pop_back();
    while (!instructionBuffer.empty()) instructionBuffer.pop();
    state.registerFile.faultRIP = (uint64_t)-1;
    state.registerFile.fetchRIP = state.registerFile.decodeRIP = state.registerFile.IP;
    canFetch                                                   = true;
    canDecode                                                  = true;
    // TODO: Flush TLB
}

MicroCodeOperation pickMicrocodeFix = {.type = (MicroCodeOperationType)0};

MicroCodeOperation& pickMicrocodeOp() {
    for (MicroCodeOperation& microOp : microcodeBuffer) {
        if (microOp.ready && !microOp.executed) {
            return microOp;
        }
    }
    return pickMicrocodeFix;
}

void execute() {
    if (microcodeBuffer.size() / MAX_MICROCODE_QUEUE <= LOW_WATER) {
        // printf("Low water: continueing decoder\n");
        canDecode = true;
    }
    if (microcodeBuffer.size() / MAX_MICROCODE_QUEUE >= HIGH_WATER) {
        // printf("High water: stopping decoder\n");
        canDecode = false;
    }
    if (microcodeBuffer.size() == 0) {
        if (state.registerFile.faultRIP == state.registerFile.IP) {
            printf("Faulty address found 0x%lx\n", state.registerFile.IP);
            exit(1);
        }
        // printf("No microcode, stalling and forcing decode enable\n");
        canDecode = true;
        return;
    }
    // Read
    MicroCodeOperation& microOp = pickMicrocodeOp();
    if ((uint16_t)microOp.type == 0) {
        // printf("No more executable microOps, stalling and forcing decode enable\n");
        canDecode = true;
        return;
    }
    if (state.registerFile.faultRIP == microOp.instructionIP) {
        printf("Faulty address found 0x%lx\n", microOp.instructionIP);
        exit(1);
    }
    // Send to EU
    uint64_t result = 0;
    switch (microOp.type) {
    case MicroCodeOperationType::LoadR: {
        uint16_t srcN = (uint16_t)microOp.data;
        result        = ((uint64_t*)(&state.registerFile))[srcN];
    } break;
    case MicroCodeOperationType::LoadI: {
        result = microOp.data;
    } break;
    case MicroCodeOperationType::GenerateAddress: {
        uint8_t  rBase  = microOp.data & 0xFF;
        uint8_t  rIndex = (microOp.data >> 8) & 0xFF;
        uint8_t  scale  = (uint8_t)pow(2, (double)((microOp.data >> 16) & 0b11));
        int32_t  disp   = (microOp.data >> 18) & 0xFFFFFFFF;
        uint64_t base   = ((uint64_t*)(&state.registerFile))[rBase];
        uint64_t index  = 0;
        if (rIndex != 0xFF) {
            index = ((uint64_t*)(&state.registerFile))[rIndex];
        }
        result = AGUCalculate(base, index, scale, disp);
    } break;
    case MicroCodeOperationType::ReadMemory: {
        uint64_t addr = ((uint64_t*)(&state.registerFile))[(uint8_t)microOp.data];
        switch (microOp.operandSize) {
        case 0:
            result = LSU->loadByte(addr);
            break;
        case 1:
            result = LSU->loadHalfWord(addr);
            break;
        case 2:
            result = LSU->loadWord(addr);
            break;
        case 3:
            result = LSU->loadDoubleWord(addr);
            break;
        }
    } break;
    case MicroCodeOperationType::AddI: {
        uint16_t registerN    = (uint16_t)microOp.registerN;
        int64_t  registerData = ((uint64_t*)(&state.registerFile))[registerN];
        result                = ALUCalculate(registerData, microOp.data, ALUOperation::Add);
    } break;
    case MicroCodeOperationType::AndR: {
        uint16_t registerN     = (uint16_t)microOp.registerN;
        uint64_t registerData  = ((uint64_t*)(&state.registerFile))[registerN];
        uint16_t registerN1    = (uint16_t)microOp.data;
        uint64_t register2Data = ((uint64_t*)(&state.registerFile))[registerN1];
        result                 = ALUCalculate(registerData, register2Data, ALUOperation::And);
    } break;
    case MicroCodeOperationType::XorR: {
        uint16_t registerN     = (uint16_t)microOp.registerN;
        uint64_t registerData  = ((uint64_t*)(&state.registerFile))[registerN];
        uint16_t registerN1    = (uint16_t)microOp.data;
        uint64_t register2Data = ((uint64_t*)(&state.registerFile))[registerN1];
        result                 = ALUCalculate(registerData, register2Data, ALUOperation::Xor);
    } break;
    case MicroCodeOperationType::BranchI: {
        uint16_t registerN    = (uint16_t)Registers::RIP;
        int64_t  registerData = ((uint64_t*)(&state.registerFile))[registerN];
        result                = ALUCalculate(registerData, microOp.data, ALUOperation::Add);
    } break;
    default:
        printf("TODO: Execute instruction %u writing to %u using %lx data\n", (uint16_t)microOp.type, (uint16_t)microOp.registerN, microOp.data);
        exit(1);
    }
    printf("Executed instruction %u writing to %u using %lx data\n", (uint16_t)microOp.type, (uint16_t)microOp.registerN, microOp.data);
    if (microOp.operandSize != 3) {
        result &= microOp.mask;
    }
    microOp.executed = true;
    microOp.result   = result;
    for (MicroCodeOperation& microOpLoop : microcodeBuffer) {
        if (microOpLoop.dependsOn[0] == microOp.tag) {
            microOpLoop.dependsOn[0] = 0;
        }
        if (microOpLoop.dependsOn[1] == microOp.tag) {
            microOpLoop.dependsOn[1] = 0;
        }
        if (microOpLoop.dependsOn[2] == microOp.tag) {
            microOpLoop.dependsOn[2] = 0;
        }
        uint64_t zeroes[3] = {0, 0, 0};
        microOpLoop.ready  = std::memcmp(microOpLoop.dependsOn, zeroes, sizeof(zeroes)) == 0;
    }
}

void writeback() {
    if (microcodeBuffer.size() == 0) {
        // printf("No microcode, stalling and forcing decode enable\n");
        canDecode = true;
        return;
    }
    std::vector<size_t> toRemove;
    size_t              i             = 0;
    bool                flushPipeline = false;
    for (MicroCodeOperation& microOp : microcodeBuffer) {
        if (microOp.executed) {
            // commit to register file or memory
            if (microOp.type != MicroCodeOperationType::WriteMemory) {
                ((uint64_t*)(&state.registerFile))[(uint16_t)microOp.registerN] = microOp.result;
                if (registerStatus[(uint16_t)microOp.registerN].producerTag == microOp.tag) {
                    registerStatus[(uint16_t)microOp.registerN].busy = false;
                }
            }
            if (microOp.type == MicroCodeOperationType::BranchI) {
                flushPipeline = true;
            }
            toRemove.push_back(i);
            break;
        }
        i++;
    }
    auto begin = microcodeBuffer.begin();
    for (size_t j : toRemove) {
        microcodeBuffer.erase(begin + j);
    }
    if (flushPipeline) {
        pipelineFlush();
    }
}

void cycle() {
    fetch();
    decode();
    execute();
    writeback();
    state.registerFile.cycle++;
}

int main() {
    // TODO: Load BIOS from file
    pipelineFlush();
    for (size_t i = 0; i < 300; ++i) {
        cycle();
    }
    for (size_t i = 0; i < sizeof(RegisterFile) / 8; ++i) {
        printf("R%lu = 0x%lx\n", i, ((uint64_t*)(&state.registerFile))[i]);
    }
}