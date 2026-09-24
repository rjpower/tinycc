#ifdef TARGET_DEFS_ONLY

/* tcc's ELF objects for wasm32 use a private machine number */
#define EM_TCC_TARGET 0x4157

/* relocation types (private to tcc's wasm ELF objects) */
#define R_WASM_NONE      0
#define R_WASM_32        1 /* 32-bit absolute value in data (address or table index) */
#define R_WASM_SLEB      2 /* padded sleb128 in code: address or table index, addend in place */
#define R_WASM_FUNC_LEB  3 /* padded uleb128 in code: function index */
#define R_WASM_TYPE_LEB  4 /* padded uleb128 in code: type index of a signature symbol */
#define R_WASM_SIG       5 /* zero size marker: signature of the preceding call */
#define R_WASM_JMP_SLOT  6
#define R_WASM_GLOB_DAT  7
#define R_WASM_COPY      8
#define R_WASM_RELATIVE  9
#define R_WASM_TAG_LEB  10 /* padded uleb128 in code: tag index */

#define R_DATA_32   R_WASM_32
#define R_DATA_PTR  R_WASM_32
#define R_JMP_SLOT  R_WASM_JMP_SLOT
#define R_GLOB_DAT  R_WASM_GLOB_DAT
#define R_COPY      R_WASM_COPY
#define R_RELATIVE  R_WASM_RELATIVE

#define R_NUM       11

#define ELF_START_ADDR 0x1000
#define ELF_PAGE_SIZE  0x1000

#define PCRELATIVE_DLLPLT 0
#define RELOCATE_DLLPLT 0

#else /* !TARGET_DEFS_ONLY */

#include "tcc.h"

/* The generic ELF linking machinery is not used for wasm output; these
   are stubs to satisfy tccelf.c. */

ST_FUNC int code_reloc (int reloc_type)
{
    switch (reloc_type) {
    case R_WASM_32:
        return 0;
    case R_WASM_SLEB:
    case R_WASM_FUNC_LEB:
    case R_WASM_TYPE_LEB:
    case R_WASM_SIG:
    case R_WASM_TAG_LEB:
        return 1;
    }
    return -1;
}

ST_FUNC int gotplt_entry_type (int reloc_type)
{
    return NO_GOTPLT_ENTRY;
}

ST_FUNC unsigned create_plt_entry(TCCState *s1, unsigned got_offset, struct sym_attr *attr)
{
    return 0;
}

ST_FUNC void relocate_plt(TCCState *s1)
{
}

ST_FUNC void relocate(TCCState *s1, ElfW_Rel *rel, int type, unsigned char *ptr, addr_t addr, addr_t val)
{
    tcc_error_noabort("wasm relocations are resolved by tccwasm.c");
}

#endif /* !TARGET_DEFS_ONLY */
