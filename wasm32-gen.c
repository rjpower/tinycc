/*
 *  WebAssembly (wasm32) code generator for TCC
 *
 *  Copyright (c) 2026 Russell Power
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
   Overview
   --------
   tccgen expects a register machine: values live in numbered registers,
   jumps go to byte offsets that are patched later.  WebAssembly is a
   typed stack machine with structured control flow.  This backend bridges
   the two:

   - "registers" are wasm locals: 8 x i64 (RC_INT) and 8 x f64 (RC_FLOAT).
     Every C integer value is held in an i64 local, 32-bit values with
     undefined upper bits (they are wrapped before every 32-bit operation).
     Every C float value is held in an f64 local, 'float' values being
     rounded to single precision after each operation.

   - all C locals/params live in linear memory in a frame addressed from
     the 'fp' local, below a global stack pointer (wasm global 0).

   - within a function we first emit an intermediate byte stream into the
     text section: plain wasm instructions, plus a few pseudo instructions
     (prefixed by 0xff) for jumps, symbol references and frame-relative
     accesses.  When the function is finished (gfunc_epilog), the stream
     is split into basic blocks and rewritten into structured wasm:
     forward jumps become 'br' out of nested blocks, backward jumps set a
     label local and 'br' to an enclosing loop whose head dispatches with
     'br_table'.

   - the output ELF object holds, per function, "<signature>\0" followed
     by the wasm function body (locals + code).  Relocations are padded
     5-byte LEBs.  tccwasm.c turns the linked sections into a module.
*/

#ifdef TARGET_DEFS_ONLY

#define NB_REGS 16

#define RC_INT     0x0001 /* generic integer register */
#define RC_FLOAT   0x0002 /* generic float register */
#define RC_R(x)    (1 << (2 + (x)))  /* x = 0..7 */
#define RC_F(x)    (1 << (10 + (x))) /* x = 0..7 */
#define RC_IRET    RC_R(0) /* function return: integer register */
#define RC_FRET    RC_F(0) /* function return: float register */

#define TREG_R(x)  (x)
#define TREG_F(x)  (8 + (x))

#define REG_IRET   TREG_R(0)
#define REG_FRET   TREG_F(0)

#define PTR_SIZE 4

/* callers truncate/extend small return values too (calls through
   mis-typed function pointers then behave like on other targets) */
#define PROMOTE_RET

/* long double has the wasm C ABI layout (IEEE quad), see gen_ldouble_* */
#define LDOUBLE_SIZE  16
#define LDOUBLE_ALIGN 16
#define MAX_ALIGN     16

/******************************************************/
#else /* ! TARGET_DEFS_ONLY */
/******************************************************/
#define USING_GLOBALS
#include "tcc.h"

ST_DATA const char * const target_machine_defs =
    "__wasm__\0"
    "__wasm32__\0"
    "__wasi__\0"
    ;

ST_DATA const int reg_classes[NB_REGS] = {
    RC_INT | RC_R(0), RC_INT | RC_R(1), RC_INT | RC_R(2), RC_INT | RC_R(3),
    RC_INT | RC_R(4), RC_INT | RC_R(5), RC_INT | RC_R(6), RC_INT | RC_R(7),
    RC_FLOAT | RC_F(0), RC_FLOAT | RC_F(1), RC_FLOAT | RC_F(2), RC_FLOAT | RC_F(3),
    RC_FLOAT | RC_F(4), RC_FLOAT | RC_F(5), RC_FLOAT | RC_F(6), RC_FLOAT | RC_F(7),
};

/* ---------------------------------------------------------------- */
/* wasm opcodes */

enum {
    W_UNREACHABLE = 0x00, W_NOP = 0x01, W_BLOCK = 0x02, W_LOOP = 0x03,
    W_IF = 0x04, W_ELSE = 0x05, W_END = 0x0b, W_BR = 0x0c, W_BR_IF = 0x0d,
    W_BR_TABLE = 0x0e, W_RETURN = 0x0f, W_CALL = 0x10, W_CALL_INDIRECT = 0x11,
    W_DROP = 0x1a, W_SELECT = 0x1b,
    W_LOCAL_GET = 0x20, W_LOCAL_SET = 0x21, W_LOCAL_TEE = 0x22,
    W_GLOBAL_GET = 0x23, W_GLOBAL_SET = 0x24,
    W_I32_LOAD = 0x28, W_I64_LOAD = 0x29, W_F32_LOAD = 0x2a, W_F64_LOAD = 0x2b,
    W_I32_LOAD8_S = 0x2c, W_I32_LOAD8_U = 0x2d, W_I32_LOAD16_S = 0x2e, W_I32_LOAD16_U = 0x2f,
    W_I64_LOAD8_S = 0x30, W_I64_LOAD8_U = 0x31, W_I64_LOAD16_S = 0x32, W_I64_LOAD16_U = 0x33,
    W_I64_LOAD32_S = 0x34, W_I64_LOAD32_U = 0x35,
    W_I32_STORE = 0x36, W_I64_STORE = 0x37, W_F32_STORE = 0x38, W_F64_STORE = 0x39,
    W_I32_STORE8 = 0x3a, W_I32_STORE16 = 0x3b,
    W_I64_STORE8 = 0x3c, W_I64_STORE16 = 0x3d, W_I64_STORE32 = 0x3e,
    W_I32_CONST = 0x41, W_I64_CONST = 0x42, W_F32_CONST = 0x43, W_F64_CONST = 0x44,
    W_I32_EQZ = 0x45, W_I32_EQ = 0x46, W_I32_NE = 0x47,
    W_I32_LT_S = 0x48, W_I32_LT_U = 0x49, W_I32_GT_S = 0x4a, W_I32_GT_U = 0x4b,
    W_I32_LE_S = 0x4c, W_I32_LE_U = 0x4d, W_I32_GE_S = 0x4e, W_I32_GE_U = 0x4f,
    W_I64_EQZ = 0x50, W_I64_EQ = 0x51, W_I64_NE = 0x52,
    W_I64_LT_S = 0x53, W_I64_LT_U = 0x54, W_I64_GT_S = 0x55, W_I64_GT_U = 0x56,
    W_I64_LE_S = 0x57, W_I64_LE_U = 0x58, W_I64_GE_S = 0x59, W_I64_GE_U = 0x5a,
    W_F64_EQ = 0x61, W_F64_NE = 0x62, W_F64_LT = 0x63, W_F64_GT = 0x64,
    W_F64_LE = 0x65, W_F64_GE = 0x66,
    W_I32_ADD = 0x6a, W_I32_SUB = 0x6b, W_I32_MUL = 0x6c,
    W_I32_DIV_S = 0x6d, W_I32_DIV_U = 0x6e, W_I32_REM_S = 0x6f, W_I32_REM_U = 0x70,
    W_I32_AND = 0x71, W_I32_OR = 0x72, W_I32_XOR = 0x73,
    W_I32_SHL = 0x74, W_I32_SHR_S = 0x75, W_I32_SHR_U = 0x76,
    W_I64_ADD = 0x7c, W_I64_SUB = 0x7d, W_I64_MUL = 0x7e,
    W_I64_DIV_S = 0x7f, W_I64_DIV_U = 0x80, W_I64_REM_S = 0x81, W_I64_REM_U = 0x82,
    W_I64_AND = 0x83, W_I64_OR = 0x84, W_I64_XOR = 0x85,
    W_I64_SHL = 0x86, W_I64_SHR_S = 0x87, W_I64_SHR_U = 0x88,
    W_F64_ABS = 0x99, W_F64_NEG = 0x9a, W_F64_SQRT = 0x9f,
    W_F64_ADD = 0xa0, W_F64_SUB = 0xa1, W_F64_MUL = 0xa2, W_F64_DIV = 0xa3,
    W_I32_WRAP_I64 = 0xa7,
    W_I64_EXTEND_I32_S = 0xac, W_I64_EXTEND_I32_U = 0xad,
    W_F32_DEMOTE_F64 = 0xb6,
    W_F64_CONVERT_I32_S = 0xb7, W_F64_CONVERT_I32_U = 0xb8,
    W_F64_CONVERT_I64_S = 0xb9, W_F64_CONVERT_I64_U = 0xba,
    W_F64_PROMOTE_F32 = 0xbb,
    W_I64_EXTEND8_S = 0xc2, W_I64_EXTEND16_S = 0xc3, W_I64_EXTEND32_S = 0xc4,
    W_PREFIX_FC = 0xfc,
    /* 0xfc sub-opcodes */
    W_I32_TRUNC_SAT_F64_S = 2, W_I32_TRUNC_SAT_F64_U = 3,
    W_I64_TRUNC_SAT_F64_S = 6, W_I64_TRUNC_SAT_F64_U = 7,
};

#define W_BLOCKTYPE_VOID 0x40

/* pseudo instructions in the intermediate stream (prefixed by 0xff) */
enum {
    P_ESC = 0,      /* a literal 0xff byte */
    P_JMP,          /* [u32 target]                   unconditional jump */
    P_JMPIF,        /* [u32 target]                   jump if i32 on stack != 0 */
    P_SYM,          /* [u8 reloc][u32 sym][i32 addend] padded leb128 with relocation */
    P_SIG,          /* [u32 sym]                      signature marker relocation */
    P_PROLOG,       /* frame setup, expanded when the frame size is known */
    P_LOCADDR,      /* [i32 loc]                      push address fp+frame+loc */
    P_MEM,          /* [u8 opcode][u8 align][i32 loc] memory op at fp+frame+loc */
    P_JMPIND,       /* computed goto: L_LBL holds the target block */
};

#define P_JMP_SIZE 6
#define P_JMPIF_SIZE 6
#define P_SYM_SIZE 11
#define P_SIG_SIZE 6
#define P_PROLOG_SIZE 2
#define P_LOCADDR_SIZE 6
#define P_MEM_SIZE 8
#define P_JMPIND_SIZE 2

/* jump targets are stored with this bit set; without it the field is
   a link in a not yet resolved jump chain (0 = end of chain) */
#define JMP_RESOLVED 0x80000000u

/* wasm locals: parameters come first, then these, then the registers */
#define NB_FIXED_LOCALS 5
#define L_FP   (wasm_nparams + 0)  /* i32: frame pointer */
#define L_LBL  (wasm_nparams + 1)  /* i32: block dispatch label */
#define L_COND (wasm_nparams + 2)  /* i32: result of last comparison */
#define L_T0   (wasm_nparams + 3)  /* i32: scratch */
#define L_T1   (wasm_nparams + 4)  /* i64: scratch */
#define L_REG(r) (wasm_nparams + NB_FIXED_LOCALS + (r))

#define WASM_STACK_ALIGN 16

/* ---------------------------------------------------------------- */
/* per function state */

struct wasm_param {
    int slot;    /* frame offset (negative) */
    int idx;     /* first wasm local */
    char kind;   /* 'i', 'I', 'f', 'F', 'L' (long double: two i64 locals) */
};

static int wasm_nparams;     /* wasm locals used by the parameters */
static int wasm_nb_params;   /* entries in wasm_params */
static struct wasm_param wasm_params[128];
static char wasm_cur_sig[256];
static int wasm_va_slot;
static int wasm_main_void;   /* 'void main': return 0 */
static int wasm_ldret_slot;  /* slot of the hidden long double result pointer */
static int wasm_cond_op;   /* comparison op that produced L_COND */

/* ---------------------------------------------------------------- */
/* raw emission into the intermediate stream */

static void g_raw(int c)
{
    int ind1;
    if (nocode_wanted)
        return;
    ind1 = ind + 1;
    if (ind1 > cur_text_section->data_allocated)
        section_realloc(cur_text_section, ind1);
    cur_text_section->data[ind] = c;
    ind = ind1;
}

/* a wasm instruction byte; 0xff is escaped */
static void wb(int c)
{
    c &= 0xff;
    g_raw(c);
    if (c == 0xff)
        g_raw(P_ESC);
}

static void w_uleb(unsigned long long v)
{
    do {
        int b = v & 0x7f;
        v >>= 7;
        wb(v ? b | 0x80 : b);
    } while (v);
}

static void w_sleb(long long v)
{
    for (;;) {
        int b = v & 0x7f;
        v >>= 7;
        if ((v == 0 && !(b & 0x40)) || (v == -1 && (b & 0x40))) {
            wb(b);
            return;
        }
        wb(b | 0x80);
    }
}

static void g_le32_raw(unsigned v)
{
    g_raw(v); g_raw(v >> 8); g_raw(v >> 16); g_raw(v >> 24);
}

static void w_op(int opc) { wb(opc); }
static void w_op_idx(int opc, unsigned idx) { wb(opc); w_uleb(idx); }
static void w_local_get(int i) { w_op_idx(W_LOCAL_GET, i); }
static void w_local_set(int i) { w_op_idx(W_LOCAL_SET, i); }
static void w_i32_const(int v) { wb(W_I32_CONST); w_sleb(v); }
static void w_i64_const(long long v) { wb(W_I64_CONST); w_sleb(v); }
static void w_f64_const(double d)
{
    unsigned char b[8];
    int i;
    memcpy(b, &d, 8);
    wb(W_F64_CONST);
    for (i = 0; i < 8; i++)
        wb(b[i]);
}
/* memory op with plain (non-negative) immediate offset */
static void w_mem(int opc, int align, unsigned off)
{
    wb(opc);
    w_uleb(align);
    w_uleb(off);
}

/* ---------------------------------------------------------------- */
/* pseudo instructions */

static void p_jmp_op(int tag, unsigned target)
{
    g_raw(0xff);
    g_raw(tag);
    g_le32_raw(target);
}

static int elf_sym_index(Sym *sym)
{
    if (0 == sym->c) {
        put_extern_sym(sym, NULL, 0, 0);
        if (sym->sym_scope
            && (sym->type.t & (VT_STATIC|VT_EXTERN)) == (VT_STATIC|VT_EXTERN)) {
            Sym *s = sym;
            while (s->prev_tok)
                s = s->prev_tok;
            s->c = sym->c;
        }
    }
    return sym->c;
}

/* padded leb128 immediate relocated against symbol 'sym' */
static void p_sym(int reloc, Sym *sym, int addend)
{
    int c;
    if (nocode_wanted)
        return;
    c = elf_sym_index(sym);
    g_raw(0xff);
    g_raw(P_SYM);
    g_raw(reloc);
    g_le32_raw(c);
    g_le32_raw(addend);
}

#define sig_sym_index(sig) wasm_sig_symbol(tcc_state, sig)

static void p_type(const char *sig)
{
    int c;
    if (nocode_wanted)
        return;
    c = sig_sym_index(sig);
    g_raw(0xff);
    g_raw(P_SYM);
    g_raw(R_WASM_TYPE_LEB);
    g_le32_raw(c);
    g_le32_raw(0);
}

static void p_sig(const char *sig)
{
    int c;
    if (nocode_wanted)
        return;
    c = sig_sym_index(sig);
    g_raw(0xff);
    g_raw(P_SIG);
    g_le32_raw(c);
}

static void p_locaddr(int loc)
{
    g_raw(0xff);
    g_raw(P_LOCADDR);
    g_le32_raw(loc);
}

static void p_mem(int opc, int align, int loc)
{
    g_raw(0xff);
    g_raw(P_MEM);
    g_raw(opc);
    g_raw(align);
    g_le32_raw(loc);
}

/* ---------------------------------------------------------------- */
/* jumps */

ST_FUNC void gsym_addr(int t, int a)
{
    while (t) {
        unsigned char *ptr = cur_text_section->data + t;
        uint32_t n = read32le(ptr); /* next value */
        write32le(ptr, a | JMP_RESOLVED);
        t = n;
    }
}

ST_FUNC int gjmp(int t)
{
    int r;
    if (nocode_wanted)
        return t;
    g_raw(0xff);
    g_raw(P_JMP);
    r = ind;
    g_le32_raw(t);
    return r;
}

ST_FUNC void gjmp_addr(int a)
{
    p_jmp_op(P_JMP, a | JMP_RESOLVED);
}

ST_FUNC int gjmp_append(int n, int t)
{
    void *p;
    /* insert vtop->c jump list in t */
    if (n) {
        uint32_t n1 = n, n2;
        while ((n2 = read32le(p = cur_text_section->data + n1)))
            n1 = n2;
        write32le(p, t);
        t = n;
    }
    return t;
}

/* jump if condition 'op' (relative to the comparison that set L_COND) */
ST_FUNC int gjmp_cond(int op, int t)
{
    int r;
    if (nocode_wanted)
        return t;
    w_local_get(L_COND);
    if (op != wasm_cond_op)
        w_op(W_I32_EQZ);
    g_raw(0xff);
    g_raw(P_JMPIF);
    r = ind;
    g_le32_raw(t);
    return r;
}

/* computed goto: label addresses are block indices (see
   wasm_finish_function), so dispatch through the loop */
ST_FUNC void ggoto(void)
{
    int r = gv(RC_INT);
    w_local_get(L_REG(r));
    w_op(W_I32_WRAP_I64);
    w_local_set(L_LBL);
    g_raw(0xff);
    g_raw(P_JMPIND);
    vtop--;
}

ST_FUNC void gen_fill_nops(int bytes)
{
}

/* ---------------------------------------------------------------- */
/* signatures */

/* 'L' is long double: an IEEE quad passed as two i64 (low, high) and
   returned through a hidden pointer, as in the wasm C ABI */
static char kind_of_type(CType *type)
{
    int bt = type->t & VT_BTYPE;
    if (bt == VT_FLOAT)
        return 'f';
    if (bt == VT_DOUBLE)
        return 'F';
    if (bt == VT_LDOUBLE)
        return 'L';
    if (bt == VT_LLONG)
        return 'I';
    return 'i';
}

static char *sig_add(char *p, char k)
{
    if (k == 'L') {
        *p++ = 'I';
        *p++ = 'I';
    } else {
        *p++ = k;
    }
    return p;
}

static int returns_indirect(CType *type)
{
    int bt = type->t & VT_BTYPE;
    return bt == VT_STRUCT || bt == VT_LDOUBLE;
}

/* Build the wasm signature of a function type. For unprototyped
   functions, 'args' (bottom to top) gives the actual argument values. */
static void func_sig(char *buf, Sym *f, SValue *args, int nb_args)
{
    char *p = buf;
    Sym *s;
    int i;

    if (returns_indirect(&f->type))
        *p++ = 'i'; /* pointer to the returned struct / long double */
    if (f->f.func_type == FUNC_OLD && args) {
        for (i = 0; i < nb_args; i++)
            p = sig_add(p, kind_of_type(&args[i].type));
    } else {
        for (s = f->next; s; s = s->next)
            p = sig_add(p, kind_of_type(&s->type));
        if (f->f.func_type == FUNC_ELLIPSIS)
            *p++ = 'i'; /* pointer to variadic arguments */
    }
    *p++ = ':';
    if ((f->type.t & VT_BTYPE) != VT_VOID && !returns_indirect(&f->type))
        *p++ = kind_of_type(&f->type);
    *p = 0;
}

/* wasi crt convention: 'main' is renamed according to its prototype */
ST_FUNC const char *wasm_symbol_name(Sym *sym, const char *name)
{
    if (sym->type.ref && !strcmp(name, "main"))
        return sym->type.ref->next ? "__main_argc_argv" : "__main_void";
    return name;
}

/* ---------------------------------------------------------------- */
/* load/store */

/* push the base address of lvalue 'sv'. Returns 1 if the following
   memory op must be frame relative (P_MEM with *poff as 'loc'),
   0 for a plain memory op with immediate offset *poff. */
static int gen_base(SValue *sv, int *poff)
{
    int v = sv->r & VT_VALMASK;
    int fc = sv->c.i;

    if (v == VT_LOCAL) {
        w_local_get(L_FP);
        *poff = fc;
        return 1;
    }
    if (v == VT_LLOCAL) {
        /* pointer stored in a frame slot */
        w_local_get(L_FP);
        p_mem(W_I32_LOAD, 2, fc);
        *poff = 0;
        return 0;
    }
    if (v == VT_CONST) {
        if (sv->r & VT_SYM) {
            wb(W_I32_CONST);
            p_sym(R_WASM_SLEB, sv->sym, fc);
        } else {
            w_i32_const(fc);
        }
        *poff = 0;
        return 0;
    }
    if (v < VT_CONST) {
        /* pointer in a register; offsets are already folded into it */
        w_local_get(L_REG(v));
        w_op(W_I32_WRAP_I64);
        *poff = 0;
        return 0;
    }
    tcc_error("internal: bad lvalue kind %x", v);
    return 0;
}

/* push the effective address of lvalue 'sv' */
static void gen_addr(SValue *sv)
{
    int v = sv->r & VT_VALMASK;
    int fc = sv->c.i;
    if (v == VT_LOCAL) {
        p_locaddr(fc);
    } else if (v == VT_LLOCAL) {
        w_local_get(L_FP);
        p_mem(W_I32_LOAD, 2, fc);
    } else if (v == VT_CONST) {
        if (sv->r & VT_SYM) {
            wb(W_I32_CONST);
            p_sym(R_WASM_SLEB, sv->sym, fc);
        } else {
            w_i32_const(fc);
        }
    } else if (v < VT_CONST) {
        w_local_get(L_REG(v));
        w_op(W_I32_WRAP_I64);
    } else {
        tcc_error("internal: bad lvalue kind %x", v);
    }
}

/* call a runtime helper: its arguments are on the wasm stack */
static void gen_helper_call(const char *name, const char *sig)
{
    Sym *sym = external_helper_sym(tok_alloc_const(name));
    w_op(W_CALL);
    p_sym(R_WASM_FUNC_LEB, sym, 0);
    p_sig(sig);
}

static void gen_memop(int opc, int align, int is_loc, int off)
{
    if (is_loc)
        p_mem(opc, align, off);
    else
        w_mem(opc, align, off);
}

/* load 'r' from value 'sv' */
ST_FUNC void load(int r, SValue *sv)
{
    int v, ft, fc, fr, bt, is_loc, off;

    fr = sv->r;
    ft = sv->type.t & ~(VT_DEFSIGN | VT_VOLATILE | VT_CONSTANT);
    fc = sv->c.i;
    v = fr & VT_VALMASK;
    bt = ft & VT_BTYPE;

    if (fr & VT_LVAL) {
        if (bt == VT_LDOUBLE) {
            /* quad in memory -> double */
            gen_addr(sv);
            gen_helper_call("__tcc_ld_load", "i:F");
            w_local_set(L_REG(r));
            return;
        }
        is_loc = gen_base(sv, &off);
        if (bt == VT_FLOAT) {
            gen_memop(W_F32_LOAD, 2, is_loc, off);
            w_op(W_F64_PROMOTE_F32);
        } else if (bt == VT_DOUBLE) {
            gen_memop(W_F64_LOAD, 3, is_loc, off);
        } else if (bt == VT_BOOL) {
            gen_memop(W_I64_LOAD8_U, 0, is_loc, off);
        } else if (bt == VT_BYTE) {
            gen_memop((ft & VT_UNSIGNED) ? W_I64_LOAD8_U : W_I64_LOAD8_S, 0, is_loc, off);
        } else if (bt == VT_SHORT) {
            gen_memop((ft & VT_UNSIGNED) ? W_I64_LOAD16_U : W_I64_LOAD16_S, 1, is_loc, off);
        } else if (bt == VT_LLONG) {
            gen_memop(W_I64_LOAD, 3, is_loc, off);
        } else {
            gen_memop(W_I64_LOAD32_S, 2, is_loc, off);
        }
        w_local_set(L_REG(r));
    } else {
        if (v == VT_CONST) {
            if (fr & VT_SYM) {
                wb(W_I32_CONST);
                p_sym(R_WASM_SLEB, sv->sym, fc);
                w_op(W_I64_EXTEND_I32_S);
            } else if (bt == VT_FLOAT) {
                w_f64_const(sv->c.f);
            } else if (bt == VT_DOUBLE) {
                w_f64_const(sv->c.d);
            } else if (bt == VT_LDOUBLE) {
                w_f64_const((double)sv->c.ld);
            } else {
                w_i64_const(sv->c.i);
            }
            w_local_set(L_REG(r));
        } else if (v == VT_LOCAL) {
            p_locaddr(fc);
            w_op(W_I64_EXTEND_I32_S);
            w_local_set(L_REG(r));
        } else if (v == VT_LLOCAL) {
            w_local_get(L_FP);
            p_mem(W_I32_LOAD, 2, fc);
            w_op(W_I64_EXTEND_I32_S);
            w_local_set(L_REG(r));
        } else if (v == VT_CMP) {
            w_local_get(L_COND);
            if (fc != wasm_cond_op)
                w_op(W_I32_EQZ);
            w_op(W_I64_EXTEND_I32_U);
            w_local_set(L_REG(r));
        } else if (v == VT_JMP || v == VT_JMPI) {
            int t = v & 1, a;
            w_i64_const(t);
            w_local_set(L_REG(r));
            a = gjmp(0);
            gsym(fc);
            w_i64_const(t ^ 1);
            w_local_set(L_REG(r));
            gsym(a);
        } else if (v < VT_CONST) {
            if (v != r) {
                w_local_get(L_REG(v));
                w_local_set(L_REG(r));
            }
        } else {
            tcc_error("internal: load from kind %x", v);
        }
    }
}

/* store register 'r' in lvalue 'v' */
ST_FUNC void store(int r, SValue *sv)
{
    int fr, bt, is_loc, off;

    bt = sv->type.t & VT_BTYPE;
    fr = sv->r & VT_VALMASK;

    if (fr == VT_CONST || fr == VT_LOCAL || fr == VT_LLOCAL || (sv->r & VT_LVAL)) {
        if (bt == VT_LDOUBLE) {
            /* double -> quad in memory */
            gen_addr(sv);
            w_local_get(L_REG(r));
            gen_helper_call("__tcc_ld_store", "iF:");
            return;
        }
        is_loc = gen_base(sv, &off);
        w_local_get(L_REG(r));
        if (bt == VT_FLOAT) {
            w_op(W_F32_DEMOTE_F64);
            gen_memop(W_F32_STORE, 2, is_loc, off);
        } else if (bt == VT_DOUBLE) {
            gen_memop(W_F64_STORE, 3, is_loc, off);
        } else if (bt == VT_BYTE || bt == VT_BOOL) {
            gen_memop(W_I64_STORE8, 0, is_loc, off);
        } else if (bt == VT_SHORT) {
            gen_memop(W_I64_STORE16, 1, is_loc, off);
        } else if (bt == VT_LLONG) {
            gen_memop(W_I64_STORE, 3, is_loc, off);
        } else {
            gen_memop(W_I64_STORE32, 2, is_loc, off);
        }
    } else if (fr != r) {
        w_local_get(L_REG(r));
        w_local_set(L_REG(fr));
    }
}

/* ---------------------------------------------------------------- */
/* function calls */

/* Return the number of registers needed to return the struct, or 0 if
   returning via struct pointer. */
ST_FUNC int gfunc_sret(CType *vt, int variadic, CType *ret, int *ret_align, int *regsize)
{
    *ret_align = 1;
    *regsize = 4;
    return 0;
}

/* copy the struct value *e to a fresh temporary in the frame and
   turn *e into the (int) address of that temporary */
static void struct_arg_to_ptr(SValue *e)
{
    int size, align, addr;
    size = type_size(&e->type, &align);
    if (align < 4)
        align = 4;
    loc = (loc - size) & -align;
    addr = loc; /* 'loc' may move during the copy (register spills) */
    vset(&e->type, VT_LOCAL | VT_LVAL, addr);
    vpushv(e);
    vstore();
    vpop();
    e->type.t = VT_INT;
    e->type.ref = NULL;
    e->r = VT_LOCAL;
    e->r2 = VT_CONST;
    e->c.i = addr;
}

/* push the value of an evaluated argument (in a register, or spilled
   to the frame) onto the wasm stack as wasm type 'kind' */
static void push_arg_value(SValue *e, char kind)
{
    int v = e->r & VT_VALMASK;
    if (v < VT_CONST && !(e->r & VT_LVAL)) {
        w_local_get(L_REG(v));
        if (kind == 'i') {
            w_op(W_I32_WRAP_I64);
        } else if (kind == 'f') {
            w_op(W_F32_DEMOTE_F64);
        } else if (kind == 'L') {
            gen_helper_call("__tcc_ld_lo", "F:I");
            w_local_get(L_REG(v));
            gen_helper_call("__tcc_ld_hi", "F:I");
        }
    } else if (v == VT_LOCAL && (e->r & VT_LVAL)) {
        int bt = e->type.t & VT_BTYPE, opc, align;
        if (kind == 'L') {
            /* a spilled quad: its two words */
            w_local_get(L_FP);
            p_mem(W_I64_LOAD, 3, e->c.i);
            w_local_get(L_FP);
            p_mem(W_I64_LOAD, 3, e->c.i + 8);
            return;
        }
        w_local_get(L_FP);
        switch (kind) {
        case 'I': opc = W_I64_LOAD; align = 3; break;
        case 'f': opc = W_F32_LOAD; align = 2; break;
        case 'F': opc = W_F64_LOAD; align = 3; break;
        default:
            if (bt == VT_BYTE || bt == VT_BOOL)
                opc = (e->type.t & VT_UNSIGNED) || bt == VT_BOOL ? W_I32_LOAD8_U : W_I32_LOAD8_S, align = 0;
            else if (bt == VT_SHORT)
                opc = (e->type.t & VT_UNSIGNED) ? W_I32_LOAD16_U : W_I32_LOAD16_S, align = 1;
            else
                opc = W_I32_LOAD, align = 2;
            break;
        }
        p_mem(opc, align, e->c.i);
    } else {
        tcc_error("internal: unexpected argument location %x", e->r);
    }
}

/* Generate function call. The function address is pushed first, then
   all the parameters in call order. This functions pops all the
   parameters and the function address. */
ST_FUNC void gfunc_call(int nb_args)
{
    int i, r, nfixed, ndecl, nvar, func_type, is_direct, fr = 0;
    int ld_ret = 0, ld_sret = 0;
    Sym *f;
    SValue *fn, *e;
    char sig[256];
    int regs[128];
    char kinds[128];
    const char *res;

    if (nb_args > 120)
        tcc_error("too many arguments to function");

    fn = vtop - nb_args;
    f = fn->type.ref;
    func_type = f->f.func_type;
    is_direct = (fn->r & (VT_VALMASK | VT_LVAL | VT_SYM)) == (VT_CONST | VT_SYM);

    /* alloca is an intrinsic: allocate below the stack pointer, the
       memory is released when the function returns */
    if (is_direct && nb_args == 1 && fn->sym->v >= TOK_IDENT
        && !strcmp(get_tok_str(fn->sym->v, NULL), "alloca")) {
        r = gv(RC_INT);
        w_op_idx(W_GLOBAL_GET, 0);
        w_local_get(L_REG(r));
        w_op(W_I32_WRAP_I64);
        w_op(W_I32_SUB);
        w_i32_const(-WASM_STACK_ALIGN);
        w_op(W_I32_AND);
        w_op_idx(W_GLOBAL_SET, 0);
        save_reg_upstack(REG_IRET, 2);
        w_op_idx(W_GLOBAL_GET, 0);
        w_op(W_I64_EXTEND_I32_S);
        w_local_set(L_REG(REG_IRET));
        vtop -= 2;
        return;
    }

    /* a long double result comes back through a hidden pointer to a
       temporary in our frame */
    if ((f->type.t & VT_BTYPE) == VT_LDOUBLE) {
        ld_ret = 1;
        loc = (loc - 16) & -16;
        ld_sret = loc;
    }

    /* number of declared parameters (plus the hidden struct return pointer) */
    ndecl = 0;
    if ((f->type.t & VT_BTYPE) == VT_STRUCT)
        ndecl++;
    if (func_type != FUNC_OLD)
        for (e = (SValue*)f->next; e; e = (SValue*)((Sym*)e)->next)
            ndecl++;
    if (func_type == FUNC_ELLIPSIS && nb_args > ndecl) {
        nfixed = ndecl;
        nvar = nb_args - ndecl;
    } else {
        nfixed = nb_args;
        nvar = 0;
    }

    /* the signature */
    if (func_type == FUNC_OLD)
        func_sig(sig, f, vtop - nb_args + 1, nb_args);
    else
        func_sig(sig, f, NULL, 0);

    /* pass 1: structs are passed by pointer to a copy, variadic
       arguments go to a buffer in the caller's frame */
    if (nvar) {
        int total = 0, size, align, off[128], va_loc;
        for (i = 0; i < nvar; i++) {
            e = vtop - nvar + 1 + i;
            if (e->type.t & VT_ARRAY) {
                /* arrays are passed as pointers */
                e->type.t = (e->type.t & ~(VT_ARRAY | VT_VLA)) | VT_PTR;
                size = align = 4;
            } else {
                size = type_size(&e->type, &align);
            }
            if (align < 4)
                align = 4;
            total = (total + align - 1) & -align;
            off[i] = total;
            total += (size + 3) & -4;
        }
        loc = (loc - total) & -WASM_STACK_ALIGN;
        va_loc = loc; /* 'loc' may move below (register spills) */
        for (i = 0; i < nvar; i++) {
            e = vtop - nvar + 1 + i;
            vpushv(e);
            if (!is_float(vtop->type.t) && (vtop->type.t & VT_BTYPE) != VT_STRUCT
                && type_size(&vtop->type, &align) < 4) {
                /* default argument promotion to int */
                gv(RC_INT);
                vtop->type.t = VT_INT;
                vtop->type.ref = NULL;
            }
            vset(&vtop->type, VT_LOCAL | VT_LVAL, va_loc + off[i]);
            vswap();
            vstore();
            vpop();
        }
        vtop -= nvar;
        vseti(VT_LOCAL, va_loc); /* address of the buffer, last argument */
        nb_args = nfixed + 1;
    } else if (func_type == FUNC_ELLIPSIS) {
        /* variadic function called without variadic arguments */
        vpushi(0);
        nb_args++;
    }
    for (i = 0; i < nb_args; i++) {
        e = vtop - nb_args + 1 + i;
        if ((e->type.t & VT_BTYPE) == VT_STRUCT)
            struct_arg_to_ptr(e);
    }

    /* the return registers are about to be clobbered. This must happen
       before the arguments are moved to registers: while they still
       reference temporaries, those cannot be reused for the spills. */
    save_reg_upstack(REG_IRET, nb_args + 1);
    save_reg_upstack(REG_FRET, nb_args + 1);

    /* pass 2: evaluate everything into registers, function pointer first.
       With many arguments, earlier ones get spilled to the frame again. */
    if (!is_direct) {
        vrotb(nb_args + 1); /* function to the top */
        gv(RC_INT);
        vrott(nb_args + 1); /* and back to the bottom */
    }
    for (i = 0; i < nb_args; i++) {
        vrotb(nb_args); /* first argument to the top */
        gv(is_float(vtop->type.t) ? RC_FLOAT : RC_INT);
    }

    /* pass 3: emit the call */
    if (ld_ret)
        p_locaddr(ld_sret);
    for (i = 0; i < nb_args; i++)
        push_arg_value(vtop - nb_args + 1 + i, kind_of_type(&vtop[-nb_args + 1 + i].type));
    if (is_direct) {
        w_op(W_CALL);
        p_sym(R_WASM_FUNC_LEB, fn->sym, 0);
        p_sig(sig);
    } else {
        push_arg_value(fn, 'i');
        w_op(W_CALL_INDIRECT);
        p_type(sig);
        w_uleb(0); /* table index */
    }
    res = strchr(sig, ':') + 1;
    if (ld_ret) {
        p_locaddr(ld_sret);
        gen_helper_call("__tcc_ld_load", "i:F");
        w_local_set(L_REG(REG_FRET));
    }
    switch (*res) {
    case 'i':
        w_op(W_I64_EXTEND_I32_S);
        w_local_set(L_REG(REG_IRET));
        break;
    case 'I':
        w_local_set(L_REG(REG_IRET));
        break;
    case 'f':
        w_op(W_F64_PROMOTE_F32);
        w_local_set(L_REG(REG_FRET));
        break;
    case 'F':
        w_local_set(L_REG(REG_FRET));
        break;
    }
    (void)r; (void)fr; (void)regs; (void)kinds;
    vtop -= nb_args + 1;
}

/* ---------------------------------------------------------------- */
/* prolog / epilog and the structured rewrite */

/* generate function prolog of type 't' */
ST_FUNC void gfunc_prolog(Sym *func_sym)
{
    CType *func_type = &func_sym->type;
    int size, align, np = 0, nl = 0;
    Sym *sym;
    CType *type;

    sym = func_type->ref;
    loc = 0;
    func_vc = 0;
    wasm_cond_op = -1;
    wasm_va_slot = 0;
    wasm_ldret_slot = 0;

    func_sig(wasm_cur_sig, sym, NULL, 0);
    /* the crt expects main to return int */
    wasm_main_void = 0;
    if (!strcmp(funcname, "main") && (func_vt.t & VT_BTYPE) == VT_VOID) {
        strcat(wasm_cur_sig, "i");
        wasm_main_void = 1;
    }

    /* hidden pointer to the returned struct or long double */
    if (returns_indirect(&func_vt)) {
        loc -= 4;
        if ((func_vt.t & VT_BTYPE) == VT_STRUCT)
            func_vc = loc;
        else
            wasm_ldret_slot = loc;
        wasm_params[np].slot = loc;
        wasm_params[np].kind = 'i';
        wasm_params[np].idx = nl++;
        np++;
    }
    /* define parameters */
    while ((sym = sym->next) != NULL) {
        if (np >= 120)
            tcc_error("too many parameters");
        type = &sym->type;
        size = type_size(type, &align);
        wasm_params[np].idx = nl;
        if ((type->t & VT_BTYPE) == VT_STRUCT) {
            /* structs are passed by pointer to a copy */
            loc -= 4;
            wasm_params[np].kind = 'i';
            gfunc_set_param(sym, loc, 1);
            nl++;
        } else {
            char k = kind_of_type(type);
            size = (k == 'I' || k == 'F') ? 8 : k == 'L' ? 16 : 4;
            loc = (loc - size) & -size;
            wasm_params[np].kind = k;
            gfunc_set_param(sym, loc, 0);
            nl += k == 'L' ? 2 : 1;
        }
        wasm_params[np].slot = loc;
        np++;
    }
    if (func_var) {
        loc -= 4;
        wasm_va_slot = loc;
        wasm_params[np].slot = loc;
        wasm_params[np].kind = 'i';
        wasm_params[np].idx = nl++;
        np++;
    }
    wasm_nparams = nl;
    wasm_nb_params = np;

    g_raw(0xff);
    g_raw(P_PROLOG);
}

ST_FUNC void gen_va_start(void)
{
    vtop--;
    vset(&char_pointer_type, VT_LOCAL | VT_LVAL, wasm_va_slot);
}

/* output buffer for the rewritten function */
static CString wasm_out;

static void ob(int c)
{
    cstr_ccat(&wasm_out, c);
}

static void ouleb(unsigned long long v)
{
    do {
        int b = v & 0x7f;
        v >>= 7;
        ob(v ? b | 0x80 : b);
    } while (v);
}

static void osleb(long long v)
{
    for (;;) {
        int b = v & 0x7f;
        v >>= 7;
        if ((v == 0 && !(b & 0x40)) || (v == -1 && (b & 0x40))) {
            ob(b);
            return;
        }
        ob(b | 0x80);
    }
}

static void osleb_padded(int v)
{
    int i;
    for (i = 0; i < 4; i++) {
        ob((v & 0x7f) | 0x80);
        v >>= 7;
    }
    ob(v & 0x7f);
}

static void ouleb_padded(unsigned v)
{
    int i;
    for (i = 0; i < 4; i++) {
        ob((v & 0x7f) | 0x80);
        v >>= 7;
    }
    ob(v & 0x7f);
}

static int frame_off(int frame, int loc)
{
    int off = frame + loc;
    if (off < 0)
        tcc_error("internal: frame offset %d out of frame %d", loc, frame);
    return off;
}

static void emit_prolog(int frame)
{
    int i;
    ob(W_GLOBAL_GET); ouleb(0);
    ob(W_I32_CONST); osleb(frame);
    ob(W_I32_SUB);
    ob(W_LOCAL_SET); ouleb(L_FP);
    /* frame chain for __builtin_frame_address(n): the word at the
       bottom of the frame is the address of this frame as tccgen sees
       it (VT_LOCAL 0 = fp + frame), which is the caller's fp */
    ob(W_LOCAL_GET); ouleb(L_FP);
    ob(W_GLOBAL_GET); ouleb(0);
    ob(W_I32_STORE); ouleb(2); ouleb(0);
    for (i = 0; i < wasm_nb_params; i++) {
        int opc, align, idx = wasm_params[i].idx;
        switch (wasm_params[i].kind) {
        case 'I': opc = W_I64_STORE; align = 3; break;
        case 'f': opc = W_F32_STORE; align = 2; break;
        case 'F': opc = W_F64_STORE; align = 3; break;
        case 'L':
            /* the two words of a quad */
            ob(W_LOCAL_GET); ouleb(L_FP);
            ob(W_LOCAL_GET); ouleb(idx + 1);
            ob(W_I64_STORE); ouleb(3); ouleb(frame_off(frame, wasm_params[i].slot + 8));
            opc = W_I64_STORE; align = 3;
            break;
        default:  opc = W_I32_STORE; align = 2; break;
        }
        ob(W_LOCAL_GET); ouleb(L_FP);
        ob(W_LOCAL_GET); ouleb(idx);
        ob(opc); ouleb(align); ouleb(frame_off(frame, wasm_params[i].slot));
    }
    ob(W_LOCAL_GET); ouleb(L_FP);
    ob(W_GLOBAL_SET); ouleb(0);
}

static int pseudo_size(int tag)
{
    switch (tag) {
    case P_ESC: return 2;
    case P_JMP: return P_JMP_SIZE;
    case P_JMPIF: return P_JMPIF_SIZE;
    case P_SYM: return P_SYM_SIZE;
    case P_SIG: return P_SIG_SIZE;
    case P_PROLOG: return P_PROLOG_SIZE;
    case P_LOCADDR: return P_LOCADDR_SIZE;
    case P_MEM: return P_MEM_SIZE;
    case P_JMPIND: return P_JMPIND_SIZE;
    }
    tcc_error("internal: bad pseudo op %d", tag);
    return 0;
}

static int jmp_target(unsigned char *code, int p, int start, int end)
{
    unsigned t = read32le(code + p + 2);
    if (!(t & JMP_RESOLVED))
        return end - start; /* unresolved: treat as jump to function end */
    t &= ~JMP_RESOLVED;
    if ((int)t < start || (int)t > end)
        tcc_error("internal: jump target out of function");
    return t - start;
}

/* Convert the intermediate stream of the current function into
   structured wasm code.  See the overview at the top of the file. */
static void wasm_finish_function(int frame)
{
    unsigned char *code;
    unsigned char *lead;
    int *blk;
    int start = func_ind, end = ind, len = end - start;
    int p, tag, n, i, j, cur, has_back, nblocks;
    Section *s = cur_text_section;

    if (nocode_wanted)
        return;

    /* the generic code must not have added relocations of its own */
    if (s->reloc) {
        ElfW_Rel *rel;
        for_each_elem(s->reloc, 0, rel, ElfW_Rel)
            if (rel->r_offset >= (unsigned)start && rel->r_offset < (unsigned)end)
                tcc_error("internal: unexpected relocation in wasm function");
    }

    code = s->data;
    lead = tcc_mallocz(len + 2);
    blk = tcc_malloc((len + 2) * sizeof(int));

    /* pass 1: find basic block leaders */
    lead[0] = 1;
    has_back = 0;
    for (p = start; p < end; ) {
        if (code[p] != 0xff) {
            p++;
            continue;
        }
        tag = code[p + 1];
        if (tag == P_JMP || tag == P_JMPIF) {
            lead[jmp_target(code, p, start, end)] = 1;
            p += pseudo_size(tag);
            if (tag == P_JMP)
                lead[p - start] = 1;
        } else if (tag == P_JMPIND) {
            has_back = 1;
            p += pseudo_size(tag);
            lead[p - start] = 1;
        } else {
            p += pseudo_size(tag);
        }
    }
    /* labels are leaders too: their addresses become block indices */
    for (i = 0; i < 2; i++) {
        Sym *ls;
        for (ls = i ? local_label_stack : global_label_stack; ls; ls = ls->prev)
            if (ls->r == LABEL_DEFINED && ls->jind >= start && ls->jind < end)
                lead[ls->jind - start] = 1;
    }
    n = 0;
    for (i = 0; i <= len; i++)
        blk[i] = lead[i] ? n++ : -1;
    nblocks = n;
    if (lead[len]) /* jumps to the end of the function */
        nblocks--;
    for (i = 0; i < 2; i++) {
        Sym *ls;
        for (ls = i ? local_label_stack : global_label_stack; ls; ls = ls->prev)
            if (ls->r == LABEL_DEFINED && ls->jind >= start && ls->jind < end)
                ls->jind = blk[ls->jind - start];
    }

    /* is any jump backwards (or to the same block)? */
    cur = -1;
    for (p = start; p < end; ) {
        if (lead[p - start])
            cur = blk[p - start];
        if (code[p] != 0xff) {
            p++;
            continue;
        }
        tag = code[p + 1];
        if (tag == P_JMP || tag == P_JMPIF) {
            j = blk[jmp_target(code, p, start, end)];
            if (j <= cur)
                has_back = 1;
        }
        p += pseudo_size(tag);
    }

    /* pass 2: emit */
    cstr_new(&wasm_out);
    cstr_cat(&wasm_out, wasm_cur_sig, strlen(wasm_cur_sig) + 1);
    /* locals: 4 x i32, 9 x i64, 8 x f64 */
    ouleb(3);
    ouleb(4); ob(0x7f);
    ouleb(9); ob(0x7e);
    ouleb(8); ob(0x7c);

    n = nblocks;
    if (has_back) {
        ob(W_LOOP); ob(W_BLOCKTYPE_VOID);
    }
    for (i = 0; i < n; i++) {
        ob(W_BLOCK); ob(W_BLOCKTYPE_VOID);
    }
    if (has_back) {
        ob(W_LOCAL_GET); ouleb(L_LBL);
        ob(W_BR_TABLE); ouleb(n);
        for (i = 0; i < n; i++)
            ouleb(i);
        ouleb(0);
    }
    cur = -1;
    for (p = start; p < end; ) {
        if (lead[p - start]) {
            cur = blk[p - start];
            ob(W_END);
        }
        if (code[p] != 0xff) {
            ob(code[p]);
            p++;
            continue;
        }
        tag = code[p + 1];
        switch (tag) {
        case P_ESC:
            ob(0xff);
            break;
        case P_JMP:
            j = blk[jmp_target(code, p, start, end)];
            if (j >= nblocks) {
                /* jump to function end: return */
                ob(W_UNREACHABLE);
            } else if (j > cur) {
                if (j != cur + 1) {
                    ob(W_BR); ouleb(j - cur - 1);
                }
            } else {
                ob(W_I32_CONST); osleb(j);
                ob(W_LOCAL_SET); ouleb(L_LBL);
                ob(W_BR); ouleb(n - cur - 1);
            }
            break;
        case P_JMPIND:
            ob(W_BR); ouleb(n - cur - 1);
            break;
        case P_JMPIF:
            j = blk[jmp_target(code, p, start, end)];
            if (j >= nblocks) {
                ob(W_IF); ob(W_BLOCKTYPE_VOID);
                ob(W_UNREACHABLE);
                ob(W_END);
            } else if (j > cur) {
                ob(W_BR_IF); ouleb(j - cur - 1);
            } else {
                ob(W_IF); ob(W_BLOCKTYPE_VOID);
                ob(W_I32_CONST); osleb(j);
                ob(W_LOCAL_SET); ouleb(L_LBL);
                ob(W_BR); ouleb(n - cur);
                ob(W_END);
            }
            break;
        case P_SYM: {
            int type = code[p + 2];
            int symidx = read32le(code + p + 3);
            int addend = (int)read32le(code + p + 7);
            put_elf_reloca(symtab_section, s, start + wasm_out.size, type, symidx, 0);
            if (type == R_WASM_SLEB)
                osleb_padded(addend);
            else
                ouleb_padded(0);
            break;
        }
        case P_SIG: {
            int symidx = read32le(code + p + 2);
            put_elf_reloca(symtab_section, s, start + wasm_out.size, R_WASM_SIG, symidx, 0);
            break;
        }
        case P_PROLOG:
            emit_prolog(frame);
            break;
        case P_LOCADDR: {
            int off = frame_off(frame, (int)read32le(code + p + 2));
            ob(W_LOCAL_GET); ouleb(L_FP);
            if (off) {
                ob(W_I32_CONST); osleb(off);
                ob(W_I32_ADD);
            }
            break;
        }
        case P_MEM: {
            int opc = code[p + 2], align = code[p + 3];
            int off = frame_off(frame, (int)read32le(code + p + 4));
            ob(opc); ouleb(align); ouleb(off);
            break;
        }
        }
        p += pseudo_size(tag);
    }
    if (has_back)
        ob(W_END);
    ob(W_UNREACHABLE);
    ob(W_END);

    /* replace the intermediate stream */
    if (start + wasm_out.size > s->data_allocated)
        section_realloc(s, start + wasm_out.size);
    memcpy(s->data + start, wasm_out.data, wasm_out.size);
    ind = start + wasm_out.size;
    cstr_free(&wasm_out);
    tcc_free(lead);
    tcc_free(blk);
}

/* generate function epilog */
ST_FUNC void gfunc_epilog(void)
{
    /* locals plus the frame chain word at the bottom */
    int frame = (-loc + 4 + WASM_STACK_ALIGN - 1) & -WASM_STACK_ALIGN;
    const char *res = strchr(wasm_cur_sig, ':') + 1;

    /* restore the stack pointer */
    w_local_get(L_FP);
    w_i32_const(frame);
    w_op(W_I32_ADD);
    w_op_idx(W_GLOBAL_SET, 0);
    if ((func_vt.t & VT_BTYPE) == VT_LDOUBLE) {
        w_local_get(L_FP);
        p_mem(W_I32_LOAD, 2, wasm_ldret_slot);
        w_local_get(L_REG(REG_FRET));
        gen_helper_call("__tcc_ld_store", "iF:");
    }
    switch (*res) {
    case 'i': {
        /* small integers are extended by the callee */
        int bt = func_vt.t & VT_BTYPE;
        if (wasm_main_void) {
            w_i32_const(0);
            break;
        }
        w_local_get(L_REG(REG_IRET));
        if (bt == VT_BYTE || bt == VT_BOOL) {
            if (func_vt.t & VT_UNSIGNED) {
                w_i64_const(0xff);
                w_op(W_I64_AND);
            } else {
                w_op(W_I64_EXTEND8_S);
            }
        } else if (bt == VT_SHORT) {
            if (func_vt.t & VT_UNSIGNED) {
                w_i64_const(0xffff);
                w_op(W_I64_AND);
            } else {
                w_op(W_I64_EXTEND16_S);
            }
        }
        w_op(W_I32_WRAP_I64);
        break;
    }
    case 'I':
        w_local_get(L_REG(REG_IRET));
        break;
    case 'f':
        w_local_get(L_REG(REG_FRET));
        w_op(W_F32_DEMOTE_F64);
        break;
    case 'F':
        w_local_get(L_REG(REG_FRET));
        break;
    }
    w_op(W_RETURN);

    wasm_finish_function(frame);
}

/* ---------------------------------------------------------------- */
/* arithmetic */

/* the second operand: a register or an inline constant */
static int gen_op2_const(void)
{
    return (vtop->r & (VT_VALMASK | VT_LVAL | VT_SYM)) == VT_CONST;
}

static void gen_int_binop(int op, int is64)
{
    int r, fr = 0, opc, cmp = 0, use_const;
    long long c = 0;

    use_const = gen_op2_const();
    if (use_const) {
        vswap();
        r = gv(RC_INT);
        vswap();
        c = vtop->c.i;
    } else {
        gv2(RC_INT, RC_INT);
        r = vtop[-1].r;
        fr = vtop[0].r;
    }

    w_local_get(L_REG(r));
    if (!is64)
        w_op(W_I32_WRAP_I64);
    if (use_const) {
        if (is64)
            w_i64_const(c);
        else
            w_i32_const((int)c);
    } else {
        w_local_get(L_REG(fr));
        if (!is64)
            w_op(W_I32_WRAP_I64);
    }

    switch (op) {
    case '+': opc = W_I32_ADD; break;
    case '-': opc = W_I32_SUB; break;
    case '&': opc = W_I32_AND; break;
    case '^': opc = W_I32_XOR; break;
    case '|': opc = W_I32_OR; break;
    case '*': opc = W_I32_MUL; break;
    case TOK_SHL: opc = W_I32_SHL; break;
    case TOK_SHR: opc = W_I32_SHR_U; break;
    case TOK_SAR: opc = W_I32_SHR_S; break;
    case '/':
    case TOK_PDIV: opc = W_I32_DIV_S; break;
    case TOK_UDIV: opc = W_I32_DIV_U; break;
    case '%': opc = W_I32_REM_S; break;
    case TOK_UMOD: opc = W_I32_REM_U; break;
    case TOK_EQ: opc = W_I32_EQ; cmp = 1; break;
    case TOK_NE: opc = W_I32_NE; cmp = 1; break;
    case TOK_LT: opc = W_I32_LT_S; cmp = 1; break;
    case TOK_GE: opc = W_I32_GE_S; cmp = 1; break;
    case TOK_LE: opc = W_I32_LE_S; cmp = 1; break;
    case TOK_GT: opc = W_I32_GT_S; cmp = 1; break;
    case TOK_ULT: opc = W_I32_LT_U; cmp = 1; break;
    case TOK_UGE: opc = W_I32_GE_U; cmp = 1; break;
    case TOK_ULE: opc = W_I32_LE_U; cmp = 1; break;
    case TOK_UGT: opc = W_I32_GT_U; cmp = 1; break;
    default:
        tcc_error("internal: unsupported integer op %d", op);
        return;
    }
    if (is64)
        opc += cmp ? (W_I64_EQ - W_I32_EQ) : (W_I64_ADD - W_I32_ADD);
    w_op(opc);
    vtop--;
    if (cmp) {
        w_local_set(L_COND);
        vset_VT_CMP(op);
        wasm_cond_op = op;
    } else {
        if (!is64)
            w_op(W_I64_EXTEND_I32_S);
        w_local_set(L_REG(r));
        vtop->r = r;
    }
}

ST_FUNC void gen_opi(int op)
{
    gen_int_binop(op, 0);
}

ST_FUNC void gen_opl(int op)
{
    gen_int_binop(op, 1);
}

/* generate a floating point operation 'v = t1 op t2' instruction. The
   two operands are guaranteed to have the same floating point type */
ST_FUNC void gen_opf(int op)
{
    int r, fr, opc, cmp = 0, bt;

    if (op == TOK_NEG) {
        r = gv(RC_FLOAT);
        w_local_get(L_REG(r));
        w_op(W_F64_NEG);
        w_local_set(L_REG(r));
        return;
    }

    gv2(RC_FLOAT, RC_FLOAT);
    r = vtop[-1].r;
    fr = vtop[0].r;
    bt = vtop->type.t & VT_BTYPE;

    w_local_get(L_REG(r));
    w_local_get(L_REG(fr));
    switch (op) {
    case '+': opc = W_F64_ADD; break;
    case '-': opc = W_F64_SUB; break;
    case '*': opc = W_F64_MUL; break;
    case '/': opc = W_F64_DIV; break;
    case TOK_EQ: opc = W_F64_EQ; cmp = 1; break;
    case TOK_NE: opc = W_F64_NE; cmp = 1; break;
    case TOK_LT: case TOK_ULT: opc = W_F64_LT; cmp = 1; break;
    case TOK_GE: case TOK_UGE: opc = W_F64_GE; cmp = 1; break;
    case TOK_LE: case TOK_ULE: opc = W_F64_LE; cmp = 1; break;
    case TOK_GT: case TOK_UGT: opc = W_F64_GT; cmp = 1; break;
    default:
        tcc_error("internal: unsupported float op %d", op);
        return;
    }
    w_op(opc);
    vtop--;
    if (cmp) {
        w_local_set(L_COND);
        vset_VT_CMP(op);
        wasm_cond_op = op;
    } else {
        if (bt == VT_FLOAT) {
            w_op(W_F32_DEMOTE_F64);
            w_op(W_F64_PROMOTE_F32);
        }
        w_local_set(L_REG(r));
        vtop->r = r;
    }
}

/* ---------------------------------------------------------------- */
/* conversions */

/* convert integers to fp 't' type. Must handle 'int', 'unsigned int'
   and 'long long' cases. */
ST_FUNC void gen_cvt_itof(int t)
{
    int r, fr, st = vtop->type.t;
    int is64 = (st & VT_BTYPE) == VT_LLONG;
    int uns = (st & VT_UNSIGNED) != 0;

    r = gv(RC_INT);
    fr = get_reg(RC_FLOAT);
    w_local_get(L_REG(r));
    if (is64) {
        w_op(uns ? W_F64_CONVERT_I64_U : W_F64_CONVERT_I64_S);
    } else {
        w_op(W_I32_WRAP_I64);
        w_op(uns ? W_F64_CONVERT_I32_U : W_F64_CONVERT_I32_S);
    }
    if ((t & VT_BTYPE) == VT_FLOAT) {
        w_op(W_F32_DEMOTE_F64);
        w_op(W_F64_PROMOTE_F32);
    }
    w_local_set(L_REG(fr));
    vtop->r = fr;
}

/* convert fp to int 't' type */
ST_FUNC void gen_cvt_ftoi(int t)
{
    int r, fr;
    int is64 = (t & VT_BTYPE) == VT_LLONG;
    int uns = (t & VT_UNSIGNED) != 0;

    r = gv(RC_FLOAT);
    fr = get_reg(RC_INT);
    w_local_get(L_REG(r));
    w_op(W_PREFIX_FC);
    if (is64) {
        w_uleb(uns ? W_I64_TRUNC_SAT_F64_U : W_I64_TRUNC_SAT_F64_S);
    } else {
        w_uleb(uns ? W_I32_TRUNC_SAT_F64_U : W_I32_TRUNC_SAT_F64_S);
        w_op(W_I64_EXTEND_I32_S);
    }
    w_local_set(L_REG(fr));
    vtop->r = fr;
}

/* convert from one floating point type to another */
ST_FUNC void gen_cvt_ftof(int t)
{
    /* all floats are held as f64 in registers; 'float' values are
       rounded to single precision */
    int r = gv(RC_FLOAT);
    if ((t & VT_BTYPE) == VT_FLOAT) {
        w_local_get(L_REG(r));
        w_op(W_F32_DEMOTE_F64);
        w_op(W_F64_PROMOTE_F32);
        w_local_set(L_REG(r));
    }
}

/* char/short to int conversion */
ST_FUNC void gen_cvt_csti(int t)
{
    int r, bt = t & VT_BTYPE;
    r = gv(RC_INT);
    w_local_get(L_REG(r));
    if (bt == VT_BYTE || bt == VT_BOOL) {
        if (t & VT_UNSIGNED) {
            w_i64_const(0xff);
            w_op(W_I64_AND);
        } else {
            w_op(W_I64_EXTEND8_S);
        }
    } else if (bt == VT_SHORT) {
        if (t & VT_UNSIGNED) {
            w_i64_const(0xffff);
            w_op(W_I64_AND);
        } else {
            w_op(W_I64_EXTEND16_S);
        }
    }
    w_local_set(L_REG(r));
}

/* sign extend int to long long */
ST_FUNC void gen_cvt_sxtw(void)
{
    int r = gv(RC_INT);
    w_local_get(L_REG(r));
    w_op(W_I64_EXTEND32_S);
    w_local_set(L_REG(r));
}

/* ---------------------------------------------------------------- */
/* variable length arrays */

/* Save the stack pointer onto the stack and return the location of its address */
ST_FUNC void gen_vla_sp_save(int addr)
{
    w_local_get(L_FP);
    w_op_idx(W_GLOBAL_GET, 0);
    p_mem(W_I32_STORE, 2, addr);
}

/* Restore the SP from a location on the stack */
ST_FUNC void gen_vla_sp_restore(int addr)
{
    w_local_get(L_FP);
    p_mem(W_I32_LOAD, 2, addr);
    w_op_idx(W_GLOBAL_SET, 0);
}

/* Subtract from the stack pointer, and push the resulting value onto the stack */
ST_FUNC void gen_vla_alloc(CType *type, int align)
{
    int r = gv(RC_INT);
    w_op_idx(W_GLOBAL_GET, 0);
    w_local_get(L_REG(r));
    w_op(W_I32_WRAP_I64);
    w_op(W_I32_SUB);
    w_i32_const(-WASM_STACK_ALIGN);
    w_op(W_I32_AND);
    w_op_idx(W_GLOBAL_SET, 0);
    vpop();
}

/* end of wasm32 code generator */
/*************************************************************/
#endif
/*************************************************************/
