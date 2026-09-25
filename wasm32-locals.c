/*
 *  Promotion of frame slots to wasm locals for the wasm32 code generator
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

/* Included by wasm32-gen.c after the output helpers.

   Every C local lives in the linear memory frame, so the engine cannot
   keep it in a register.  This pass finds the frame slots that the
   function only ever touches through P_MEM at one offset, with one
   access width and one value class, and that no other access or
   address-taken object overlaps.  Those become wasm locals: an i64 for
   integers of any width (loads re-extend from the low bits, as they do
   from memory), an f32 or f64 for floats.

   The extent of an address-taken object comes with P_LOCADDR (see
   gen_locaddr).  If it is unknown for any object, nothing in the
   function is promoted.  The emitter keeps the 'local.get fp' that
   precedes every frame access and drops it, which costs nothing after
   the engine's own optimisation. */

enum { CLS_INT = 1, CLS_F32, CLS_F64, CLS_ADDR };

#define MAX_PROMOTED 512

typedef struct WasmAccess {
    int loc, size;
    unsigned char cls;
} WasmAccess;

typedef struct WasmPromo {
    int frame;
    int *local;          /* (loc + frame) -> wasm local index, or -1 */
    unsigned char *cls;  /* (loc + frame) -> class of the promoted slot */
    int nb_i64, nb_f32, nb_f64;
} WasmPromo;

static WasmPromo wasm_promo;

/* width and class of a memory access opcode, or 0 for other opcodes */
static int mem_op_info(int opc, int *cls)
{
    switch (opc) {
    case W_I32_LOAD: case W_I32_STORE:
    case W_I64_LOAD32_S: case W_I64_LOAD32_U: case W_I64_STORE32:
        *cls = CLS_INT; return 4;
    case W_I64_LOAD: case W_I64_STORE:
        *cls = CLS_INT; return 8;
    case W_I32_LOAD8_S: case W_I32_LOAD8_U: case W_I32_STORE8:
    case W_I64_LOAD8_S: case W_I64_LOAD8_U: case W_I64_STORE8:
        *cls = CLS_INT; return 1;
    case W_I32_LOAD16_S: case W_I32_LOAD16_U: case W_I32_STORE16:
    case W_I64_LOAD16_S: case W_I64_LOAD16_U: case W_I64_STORE16:
        *cls = CLS_INT; return 2;
    case W_F32_LOAD: case W_F32_STORE:
        *cls = CLS_F32; return 4;
    case W_F64_LOAD: case W_F64_STORE:
        *cls = CLS_F64; return 8;
    }
    return 0;
}

static int access_cmp(const void *a, const void *b)
{
    const WasmAccess *x = a, *y = b;
    return x->loc - y->loc;
}

/* the promoted local for frame offset loc, or -1 */
static int promo_local(int loc)
{
    if (!wasm_promo.local)
        return -1;
    return wasm_promo.local[loc + wasm_promo.frame];
}

static void promo_free(void)
{
    tcc_free(wasm_promo.local);
    tcc_free(wasm_promo.cls);
    memset(&wasm_promo, 0, sizeof wasm_promo);
}

/* Decide which slots of the current function to promote.  first_local
   is the index of the first free wasm local. */
static void promo_analyze(unsigned char *code, int start, int end, int frame, int first_local)
{
    WasmAccess *acc = NULL;
    int nb = 0, cap = 0, p, i, j, cls, size, max_end;
    int nb_slots = 0, nb_i64 = 0, nb_f32 = 0, nb_f64 = 0;

    memset(&wasm_promo, 0, sizeof wasm_promo);
    wasm_promo.frame = frame;
    for (p = start; p < end; ) {
        if (code[p] != 0xff) {
            p++;
            continue;
        }
        cls = 0;
        if (code[p + 1] == P_MEM) {
            size = mem_op_info(code[p + 2], &cls);
            if (!size)
                tcc_error("internal: bad memory op");
            p += 4;
        } else if (code[p + 1] == P_LOCADDR) {
            cls = CLS_ADDR;
            size = (int)read32le(code + p + 10);
            if (size < 0)
                goto done; /* an object of unknown extent */
            p += 6; /* the base */
        }
        if (cls) {
            if (nb == cap) {
                cap = cap ? 2 * cap : 64;
                acc = tcc_realloc(acc, cap * sizeof(WasmAccess));
            }
            acc[nb].loc = (int)read32le(code + p);
            acc[nb].size = size;
            acc[nb].cls = cls;
            nb++;
            p += cls == CLS_ADDR ? 8 : 4;
        } else {
            p += pseudo_size(code[p + 1]);
        }
    }
    if (!nb)
        goto done;
    qsort(acc, nb, sizeof(WasmAccess), access_cmp);

    wasm_promo.local = tcc_malloc((frame + 1) * sizeof(int));
    wasm_promo.cls = tcc_mallocz(frame + 1);
    for (i = 0; i <= frame; i++)
        wasm_promo.local[i] = -1;

    /* sweep the groups of accesses at each offset; max_end is the end
       of everything at lower offsets */
    max_end = -frame;
    for (i = 0; i < nb; i = j) {
        int loc = acc[i].loc, ok = 1, ext = 0;
        for (j = i; j < nb && acc[j].loc == loc; j++) {
            if (acc[j].cls == CLS_ADDR || acc[j].cls != acc[i].cls || acc[j].size != acc[i].size)
                ok = 0;
            if (acc[j].size > ext)
                ext = acc[j].size;
        }
        if (loc < -frame || loc + ext > 0)
            tcc_error("internal: frame access outside the frame");
        if (max_end > loc || (j < nb && acc[j].loc < loc + ext))
            ok = 0;
        if (loc + ext > max_end)
            max_end = loc + ext;
        if (!ok || nb_slots == MAX_PROMOTED)
            continue;
        /* the index is assigned below, once the group counts are known */
        wasm_promo.cls[loc + frame] = acc[i].cls;
        nb_slots++;
        if (acc[i].cls == CLS_INT)
            nb_i64++;
        else if (acc[i].cls == CLS_F32)
            nb_f32++;
        else
            nb_f64++;
    }
    /* locals are declared by type: the i64s, then the f32s, then the f64s */
    {
        int next_i64 = first_local, next_f32 = next_i64 + nb_i64, next_f64 = next_f32 + nb_f32;
        for (i = 0; i <= frame; i++) {
            switch (wasm_promo.cls[i]) {
            case CLS_INT: wasm_promo.local[i] = next_i64++; break;
            case CLS_F32: wasm_promo.local[i] = next_f32++; break;
            case CLS_F64: wasm_promo.local[i] = next_f64++; break;
            }
        }
    }
    wasm_promo.nb_i64 = nb_i64;
    wasm_promo.nb_f32 = nb_f32;
    wasm_promo.nb_f64 = nb_f64;
done:
    tcc_free(acc);
}

/* Emit the replacement of a frame access whose slot was promoted.  For
   a load the stack holds fp; for a store fp and the value. */
static void promo_emit_mem(int opc, int local)
{
    switch (opc) {
    case W_I32_STORE: case W_I32_STORE8: case W_I32_STORE16:
        ob(W_I64_EXTEND_I32_U);
        /* fallthrough */
    case W_I64_STORE: case W_I64_STORE8: case W_I64_STORE16: case W_I64_STORE32:
    case W_F32_STORE: case W_F64_STORE:
        ob(W_LOCAL_SET); ouleb(local);
        ob(W_DROP);
        return;
    }
    ob(W_DROP);
    ob(W_LOCAL_GET); ouleb(local);
    switch (opc) {
    case W_I32_LOAD8_S: case W_I64_LOAD8_S:
        ob(W_I64_EXTEND8_S);
        break;
    case W_I32_LOAD16_S: case W_I64_LOAD16_S:
        ob(W_I64_EXTEND16_S);
        break;
    case W_I64_LOAD32_S:
        ob(W_I64_EXTEND32_S);
        break;
    case W_I32_LOAD8_U: case W_I64_LOAD8_U:
        ob(W_I64_CONST); osleb(0xff);
        ob(W_I64_AND);
        break;
    case W_I32_LOAD16_U: case W_I64_LOAD16_U:
        ob(W_I64_CONST); osleb(0xffff);
        ob(W_I64_AND);
        break;
    case W_I64_LOAD32_U:
        ob(W_I64_CONST); osleb(0xffffffffLL);
        ob(W_I64_AND);
        break;
    }
    switch (opc) {
    case W_I32_LOAD: case W_I32_LOAD8_S: case W_I32_LOAD8_U:
    case W_I32_LOAD16_S: case W_I32_LOAD16_U:
        ob(W_I32_WRAP_I64);
        break;
    }
}
