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
    144, 9, 48, 143, 1, 0x00, 0x00, 0x00, 0x20, // mov r2, [r3 + r1 * 8 + 0x20]
};

constexpr uint64_t ramSize = sizeof(RAM);

enum struct MicroCodeOperationType : uint16_t {
    Size = 0,        // Not an actual microop, but used for the size of the decoded instruction as microops.
    Offset,          // Not an actual microop, but used for the offset of the list of microops in the array.
    LoadI,           // Load 64 bit data field into the registerN.
    LoadR,           // Use lower 8 bits of the data field as register and load into the registerN.
    GenerateAddress, // BaseIn : 8,IndexIn : 8, scaleIn : 2, dispIn : 32, zero : 14 in data field as AGU
                     // paramters
    ReadMemory,      // Read memory from address in EADDR field and load into the registerN.
    WriteMemory,     // Store value in registerN to the memory address in EADDR.
    AddI,            // Adds the immediate in data to the registerN
};

enum struct Registers : uint8_t {
    R0  = 0,
    R29 = 29,
    RSP = 30,
    RIP = 31,
    READDR,
    RTEMP,
};

struct __attribute__((packed, aligned(128))) MicroCodeOperation {
    MicroCodeOperationType type;
    uint8_t                registerN;
    uint64_t               data;
    uint8_t                operandSize : 2;
};

#define MOV_R64_I64_IDX 0
#define MOV_R64_R64_IDX 1
#define MOV_R64_M64_IDX 2
#define MOV_M64_R64_IDX 3

MicroCodeOperation microcodeROM[] = {
    // Size and offset for `mov r64, i64`
    {.type = MicroCodeOperationType::Size, .registerN = 0, .data = 1, .operandSize = 0},
    {.type = MicroCodeOperationType::Offset, .registerN = 0, .data = 8, .operandSize = 0},
    // Size and offset for `mov r64, r64`
    {.type = MicroCodeOperationType::Size, .registerN = 0, .data = 1, .operandSize = 0},
    {.type = MicroCodeOperationType::Offset, .registerN = 0, .data = 9, .operandSize = 0},
    // Size and offset for `mov r64, m64`
    {.type = MicroCodeOperationType::Size, .registerN = 0, .data = 2, .operandSize = 0},
    {.type = MicroCodeOperationType::Offset, .registerN = 0, .data = 10, .operandSize = 0},
    // Size and offset for `mov m64, r64`
    {.type = MicroCodeOperationType::Size, .registerN = 0, .data = 2, .operandSize = 0},
    {.type = MicroCodeOperationType::Offset, .registerN = 0, .data = 12, .operandSize = 0},
    // mov r64, i64
    {.type = MicroCodeOperationType::LoadI, .registerN = 0, .data = 0, .operandSize = 0},
    // mov r64, r64
    {.type = MicroCodeOperationType::LoadR, .registerN = 0, .data = 0, .operandSize = 0},
    // mov r64, m64
    {.type = MicroCodeOperationType::GenerateAddress, .registerN = 0, .data = 0, .operandSize = 0},
    {.type = MicroCodeOperationType::ReadMemory, .registerN = 0, .data = 0, .operandSize = 0},
    // mov m64, r64
    {.type = MicroCodeOperationType::GenerateAddress, .registerN = 0, .data = 0, .operandSize = 0},
    {.type = MicroCodeOperationType::WriteMemory, .registerN = 0, .data = 0, .operandSize = 0},
};

struct __attribute__((packed)) OpcodeSpecificDecode {
    uint8_t opcodeId : 4;
    uint8_t opType : 4;
};

struct __attribute__((packed)) MRPrefix {
    uint8_t hasMemoryPrefix : 1;
    uint8_t hasRegisterPrefix : 1;
    uint8_t destRegister : 5;
    uint8_t hasDestReg : 1; // If this is 1, destination register should be 0
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
    uint64_t eaddr;
    uint64_t cycle;
    uint64_t fetchRIP;
    uint64_t faultRIP;
    uint64_t decodeRIP;
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

// Key = address, Value = L1I address
std::unordered_map<uint64_t, CacheLine*> L1Cache;
// Key = address, Value = L1I address
std::unordered_map<uint64_t, CacheLine*> L2Cache;
// Key = address, Value = L1I address
std::unordered_map<uint64_t, CacheLine*> L3Cache;
// std::array<uint8_t, 16 * 1024 * 1024> RAMArray;
std::queue<uint8_t>            instructionBuffer;
std::queue<MicroCodeOperation> microcodeBuffer;
bool                           canFetch  = true;
bool                           canDecode = true;
CPUState                       state;

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
        uint32_t low  = *reinterpret_cast<uint32_t*>(&line->data[offset]);
        uint32_t high = *reinterpret_cast<uint32_t*>(&line->data[offset + 4]);
        return (static_cast<uint64_t>(high) << 32) | low;
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
    uint64_t qWord = LSU->loadDoubleWord(state.registerFile.fetchRIP);
    for (size_t i = 0; i < 8; ++i) {
        instructionBuffer.push((qWord >> (i * 8)) & 0xFF);
    }
    state.registerFile.fetchRIP += 8;
}

uint16_t getMicroOpIndex(uint8_t opcode, MRPrefix mrPrefix, OpcodeSpecificDecode opSpecific) {
    switch (opcode) {
    // Mov
    case 0: {
        // mov r64, i64
        if (mrPrefix.hasMemoryPrefix == 0 and mrPrefix.hasRegisterPrefix == 0) {
            return MOV_R64_I64_IDX;
        }
        // mov r64, r64
        if (mrPrefix.hasMemoryPrefix == 0 and mrPrefix.hasRegisterPrefix == 1) {
            return MOV_R64_R64_IDX;
        }
        // mov r64, m64
        if (mrPrefix.hasMemoryPrefix == 1 and mrPrefix.hasRegisterPrefix == 0) {
            if (opSpecific.opType == 0b1001) {
                return MOV_R64_M64_IDX;
            }
            // mov m64, r64
            if (opSpecific.opType == 0b1011) {
                return MOV_M64_R64_IDX;
            }
        }
    }
    }
    printf("TODO: Correctly decode opcode %d\n", opcode);
    exit(1);
}

uint64_t packAGUParams(MemoryPrefix mem, uint32_t disp) {
    // BaseIn : 8,IndexIn : 8, scaleIn : 2, dispIn : 32, zero : 14
    uint8_t base  = mem.baseRegister;
    uint8_t index = mem.indexRegister;
    uint8_t scale = mem.scaleBase2;
    return base | (index << 8) | (scale << 16) | (disp << 18);
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
    byte                                   = getQueueNext(instructionBuffer);
    uint8_t  byte2                         = getQueueNext(instructionBuffer);
    uint16_t word                          = (byte << 8) | byte2;
    Prefixes prefix;
    prefix.bits         = word;
    uint64_t immediate  = 0;
    uint8_t  opcodeByte = getQueueNext(instructionBuffer);
    if (MRprefixByte.hasMemoryPrefix) {
        byte          = getQueueNext(instructionBuffer);
        byte2         = getQueueNext(instructionBuffer);
        uint8_t byte3 = getQueueNext(instructionBuffer);
        uint8_t byte4 = getQueueNext(instructionBuffer);
        immediate     = (byte << 24) | (byte2 << 16) | (byte3 << 8) | byte4;
    }
    if (opSpecifcByte.opType == 0 or opSpecifcByte.opType == 0b1010) {
        byte          = getQueueNext(instructionBuffer);
        byte2         = getQueueNext(instructionBuffer);
        uint8_t byte3 = getQueueNext(instructionBuffer);
        uint8_t byte4 = getQueueNext(instructionBuffer);
        uint8_t byte5 = getQueueNext(instructionBuffer);
        uint8_t byte6 = getQueueNext(instructionBuffer);
        uint8_t byte7 = getQueueNext(instructionBuffer);
        uint8_t byte8 = getQueueNext(instructionBuffer);
        immediate     = ((uint64_t)byte << 56) | ((uint64_t)byte2 << 48) | ((uint64_t)byte3 << 40) |
                    ((uint64_t)byte4 << 32) | (byte5 << 24) | (byte6 << 16) | (byte7 << 8) | byte8;
    }
    if (opcodeByte == 0) {
        state.registerFile.faultRIP = state.registerFile.decodeRIP;
        canDecode                   = false;
        canFetch                    = false;
        return;
    }
    state.registerFile.decodeRIP += oldInstBufferSize - instructionBuffer.size();
    uint16_t            microcodeIdx = getMicroOpIndex(opcodeByte - 1, MRprefixByte, opSpecifcByte) * 2;
    uint8_t             size         = microcodeROM[microcodeIdx].data & 0xFF;
    uint8_t             offset       = microcodeROM[microcodeIdx + 1].data & 0xFF;
    MicroCodeOperation* sequence     = new MicroCodeOperation[size + 1];
    memcpy(sequence, &microcodeROM[offset], size * sizeof(MicroCodeOperation));
    for (uint8_t i = 0; i < size; i++) {
        switch (sequence[i].type) {
        case MicroCodeOperationType::LoadI: {
            if (sequence[i].registerN == 0) {
                sequence[i].registerN = MRprefixByte.destRegister;
            }
            sequence[i].operandSize =
                MRprefixByte.hasMemoryPrefix
                    ? prefix.memoryPrefix.operandSize
                    : (MRprefixByte.hasMemoryPrefix ? prefix.registerPrefix.operandSize : 0b11);
            sequence[i].data = immediate;
        } break;
        case MicroCodeOperationType::LoadR: {
            if (sequence[i].registerN == 0) {
                sequence[i].registerN = MRprefixByte.destRegister;
            }
            sequence[i].operandSize =
                MRprefixByte.hasMemoryPrefix
                    ? prefix.memoryPrefix.operandSize
                    : (MRprefixByte.hasMemoryPrefix ? prefix.registerPrefix.operandSize : 0b11);
            sequence[i].data = prefix.registerPrefix.registerSrc1;
        } break;
        case MicroCodeOperationType::GenerateAddress: {
            if (sequence[i].registerN == 0) {
                sequence[i].registerN = (uint8_t)Registers::READDR;
            }
            sequence[i].operandSize = 0b11;
            sequence[i].data        = packAGUParams(prefix.memoryPrefix, immediate);
        } break;
        case MicroCodeOperationType::ReadMemory: {
            if (sequence[i].registerN == 0) {
                sequence[i].registerN = MRprefixByte.destRegister;
            }
            sequence[i].operandSize =
                MRprefixByte.hasMemoryPrefix
                    ? prefix.memoryPrefix.operandSize
                    : (MRprefixByte.hasMemoryPrefix ? prefix.registerPrefix.operandSize : 0b11);
        } break;
        case MicroCodeOperationType::WriteMemory: {
            if (sequence[i].registerN == 0) {
                sequence[i].registerN = MRprefixByte.destRegister;
            }
            sequence[i].operandSize =
                MRprefixByte.hasMemoryPrefix
                    ? prefix.memoryPrefix.operandSize
                    : (MRprefixByte.hasMemoryPrefix ? prefix.registerPrefix.operandSize : 0b11);
        } break;
        default: {
            printf("TODO: Support %u microcode operation\n", (uint16_t)sequence[i].type);
            exit(1);
        } break;
        }
        microcodeBuffer.push(sequence[i]);
    }
    sequence[size] = {.type        = MicroCodeOperationType::AddI,
                      .registerN   = (uint8_t)Registers::RIP,
                      .data        = oldInstBufferSize - instructionBuffer.size(),
                      .operandSize = 3};
    microcodeBuffer.push(sequence[size]);
}

uint64_t AGUCalculate(uint64_t base, uint64_t index, uint8_t scale, uint32_t disp) {
    return base + index * scale + disp;
}

enum struct ALUOperation {
    Add,
};

uint64_t ALUCalculate(uint64_t A, uint64_t B, ALUOperation operation) {
    switch (operation) {
    case ALUOperation::Add: {
        return A + B;
    }
    }
    printf("Unreachable %s:%d\n", __FILE_NAME__, __LINE__);
    exit(1);
}

void pipelineFlush() {
    while (!microcodeBuffer.empty()) microcodeBuffer.pop();
    while (!instructionBuffer.empty()) instructionBuffer.pop();
    state.registerFile.faultRIP = (uint64_t)-1;
    state.registerFile.fetchRIP = state.registerFile.decodeRIP = state.registerFile.IP;
    canFetch                                                   = true;
    canDecode                                                  = true;
    // TODO: Flush TLB
}

void execute() {
    if (state.registerFile.faultRIP == state.registerFile.IP) {
        printf("Faulty address found 0x%lx\n", state.registerFile.IP);
        exit(1);
    }
    if (microcodeBuffer.size() / MAX_MICROCODE_QUEUE <= LOW_WATER) {
        // printf("Low water: continueing decoder\n");
        canDecode = true;
    }
    if (microcodeBuffer.size() / MAX_MICROCODE_QUEUE >= HIGH_WATER) {
        // printf("High water: stopping decoder\n");
        canDecode = false;
    }
    if (microcodeBuffer.size() == 0) {
        // printf("No microcode, stalling and forcing decode enable\n");
        canDecode = true;
        return;
    }
    // Read
    MicroCodeOperation microOp = getQueueNext(microcodeBuffer);
    // Send to EU
    uint64_t result = 0;
    switch (microOp.type) {
    case MicroCodeOperationType::GenerateAddress: {
        uint8_t  rBase  = microOp.data & 0xFF;
        uint8_t  rIndex = (microOp.data >> 8) & 0xFF;
        uint8_t  scale  = (uint8_t)pow(2, (double)((microOp.data >> 16) & 0b11));
        uint32_t disp   = (microOp.data >> 18) & 0xFFFFFFFF;
        uint64_t base   = ((uint64_t*)(&state.registerFile))[rBase];
        uint64_t index  = ((uint64_t*)(&state.registerFile))[rIndex];
        result          = AGUCalculate(base, index, scale, disp);
    } break;
    case MicroCodeOperationType::ReadMemory: {
        uint64_t addr = ((uint64_t*)(&state.registerFile))[(uint8_t)Registers::READDR];
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
        uint8_t  registerN    = microOp.registerN;
        uint64_t registerData = ((uint64_t*)(&state.registerFile))[registerN];
        result                = ALUCalculate(registerData, microOp.data, ALUOperation::Add);
    } break;
    default:
        printf("TODO: Execute %u > %u\n", (uint16_t)microOp.type, microOp.registerN);
        exit(1);
    }
    if (microOp.operandSize != 3) {
        uint64_t mask = (1ULL << (8 << microOp.operandSize)) - 1;
        result &= mask;
    }
    // Write back
    if (microOp.type != MicroCodeOperationType::WriteMemory) {
        ((uint64_t*)(&state.registerFile))[microOp.registerN] = result;
    }
}

void cycle() {
    fetch();
    decode();
    execute();
    state.registerFile.cycle++;
}

int main() {
    // TODO: Load BIOS from file
    state.registerFile.IP = 0x10000;
    pipelineFlush();
    for (size_t i = 0; i < 100; ++i) {
        cycle();
    }
    for (size_t i = 0; i < sizeof(RegisterFile) / 8; ++i) {
        printf("R%lu = 0x%lx\n", i, ((uint64_t*)(&state.registerFile))[i]);
    }
}