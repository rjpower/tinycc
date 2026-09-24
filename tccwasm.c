/*
 *  WebAssembly module output for TCC
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
   Links tcc's (ELF flavoured) sections into a single wasm module.

   Conventions of the ELF objects produced by wasm32-gen.c:
   - each function in .text is "<signature>\0" followed by its wasm body
     (locals declaration + code); function symbols cover exactly that.
   - signature strings: one char per parameter, ':', result char;
     chars are i (i32), I (i64), f (f32), F (f64).
   - R_WASM_FUNC_LEB relocs (calls) are followed by a R_WASM_SIG marker
     against a ".wasm.sig:<signature>" symbol.
   - R_WASM_TYPE_LEB relocs (call_indirect) are against such symbols.
   - R_WASM_SLEB / R_WASM_32 relocs against function symbols resolve to
     table indices, against data symbols to linear memory addresses.
   - undefined functions whose name is "module.name" are imported from
     that module.
*/

#include "tcc.h"

#define WASM_PAGE_SIZE 65536
#define WASM_DATA_START 1024
#define WASM_STACK_SIZE (1 << 20)

typedef struct WasmFunc {
    int sym;            /* first ELF symbol seen for this function */
    unsigned offset;    /* body offset in .text (defined functions) */
    unsigned size;
    const char *sig;
    char *module;       /* import module (NULL if defined) */
    const char *name;
    int index;          /* function index in the module */
    int table;          /* table slot, 0 if not address-taken */
    int type;           /* type index */
    int weak_stub;      /* trap stub for an undefined weak function: address is 0 */
} WasmFunc;

typedef struct WasmLink {
    TCCState *s1;
    WasmFunc *funcs;
    int nb_funcs;
    int *sym_func;      /* elf symbol index -> function, or -1 */
    char **sym_sig;     /* elf symbol index -> signature from call sites */
    int nb_sym_sig;
    int *weak_stubs;    /* elf symbol indices of synthesized weak stubs */
    int nb_weak_stubs;
    int *off_func;      /* text offset -> function, or -1 */
    char **types;
    int nb_types;
    int nb_imports;
    int nb_table;
    unsigned data_end, stack_top, heap_base, heap_end;
} WasmLink;

static void synth_call_ctors(TCCState *s1);
static void synth_trap_function(TCCState *s1, const char *name, const char *sig);

/* ---------------------------------------------------------------- */
/* encoding helpers */

static void wo_byte(CString *cs, int c)
{
    cstr_ccat(cs, c);
}

static void wo_uleb(CString *cs, unsigned long long v)
{
    do {
        int b = v & 0x7f;
        v >>= 7;
        wo_byte(cs, v ? b | 0x80 : b);
    } while (v);
}

static void wo_sleb(CString *cs, long long v)
{
    for (;;) {
        int b = v & 0x7f;
        v >>= 7;
        if ((v == 0 && !(b & 0x40)) || (v == -1 && (b & 0x40))) {
            wo_byte(cs, b);
            return;
        }
        wo_byte(cs, b | 0x80);
    }
}

static void wo_name(CString *cs, const char *s)
{
    int len = strlen(s);
    wo_uleb(cs, len);
    cstr_cat(cs, s, len);
}

static void wo_section(CString *mod, int id, CString *payload)
{
    wo_byte(mod, id);
    wo_uleb(mod, payload->size);
    cstr_cat(mod, payload->data, payload->size);
    cstr_free(payload);
    cstr_new(payload);
}

static int wasm_valtype(TCCState *s1, int c)
{
    switch (c) {
    case 'i': return 0x7f;
    case 'I': return 0x7e;
    case 'f': return 0x7d;
    case 'F': return 0x7c;
    }
    tcc_error_noabort("internal: bad signature char '%c'", c);
    return 0x7f;
}

static void patch_uleb(unsigned char *p, unsigned v)
{
    int i;
    for (i = 0; i < 4; i++) {
        p[i] = (v & 0x7f) | 0x80;
        v >>= 7;
    }
    p[4] = v & 0x7f;
}

static void patch_sleb(unsigned char *p, int v)
{
    int i;
    for (i = 0; i < 4; i++) {
        p[i] = (v & 0x7f) | 0x80;
        v >>= 7;
    }
    p[4] = v & 0x7f;
}

static int read_sleb_padded(unsigned char *p)
{
    unsigned v = 0;
    int i;
    for (i = 0; i < 5; i++)
        v |= (unsigned)(p[i] & 0x7f) << (7 * i);
    /* sign extend from bit 34 -> 32 bit value */
    return (int)v;
}

/* ---------------------------------------------------------------- */
/* symbols and functions */

/* (macros: they need 's1' in the caller's scope) */
#define elf_sym(index) ((ElfW(Sym) *)symtab_section->data + (index))
#define elf_sym_name(index) ((char *)symtab_section->link->data + elf_sym(index)->st_name)

static int type_index(WasmLink *wl, const char *sig)
{
    int i;
    for (i = 0; i < wl->nb_types; i++)
        if (!strcmp(wl->types[i], sig))
            return i;
    dynarray_add(&wl->types, &wl->nb_types, tcc_strdup(sig));
    return wl->nb_types - 1;
}

static WasmFunc *add_func(WasmLink *wl)
{
    WasmFunc *f;
    wl->funcs = tcc_realloc(wl->funcs, (wl->nb_funcs + 1) * sizeof(WasmFunc));
    f = &wl->funcs[wl->nb_funcs++];
    memset(f, 0, sizeof *f);
    return f;
}

static int sym_is_func(ElfW(Sym) *esym)
{
    return ELFW(ST_TYPE)(esym->st_info) == STT_FUNC;
}

/* Find every function: defined ones in .text, undefined ones become
   imports. */
static void collect_functions(WasmLink *wl)
{
    TCCState *s1 = wl->s1;
    int i, nb_syms = symtab_section->data_offset / sizeof(ElfW(Sym));
    ElfW(Sym) *esym;
    WasmFunc *f;

    wl->sym_func = tcc_malloc(nb_syms * sizeof(int));
    for (i = 0; i < nb_syms; i++)
        wl->sym_func[i] = -1;
    wl->off_func = tcc_malloc((text_section->data_offset + 1) * sizeof(int));
    for (i = 0; i <= (int)text_section->data_offset; i++)
        wl->off_func[i] = -1;

    for (i = 1; i < nb_syms; i++) {
        esym = elf_sym(i);
        if (esym->st_shndx == text_section->sh_num) {
            if (!sym_is_func(esym))
                continue;
            if (esym->st_value > text_section->data_offset)
                tcc_error_noabort("internal: function symbol outside .text");
            if (wl->off_func[esym->st_value] >= 0) {
                wl->sym_func[i] = wl->off_func[esym->st_value];
                continue;
            }
            f = add_func(wl);
            f->sym = i;
            f->offset = esym->st_value;
            f->size = esym->st_size;
            f->sig = (char *)text_section->data + esym->st_value;
            f->name = elf_sym_name(i);
            wl->off_func[esym->st_value] = wl->sym_func[i] = wl->nb_funcs - 1;
            {
                int k;
                for (k = 0; k < wl->nb_weak_stubs; k++)
                    if ((int)(size_t)wl->weak_stubs[k] == i)
                        f->weak_stub = 1;
            }
        } else if (esym->st_shndx == SHN_UNDEF && sym_is_func(esym)) {
            const char *name = elf_sym_name(i);
            const char *dot = strchr(name, '.');
            if (!dot) {
                if (ELFW(ST_BIND)(esym->st_info) == STB_WEAK)
                    continue; /* resolves to a null function pointer */
                tcc_error_noabort("undefined symbol '%s'", name);
                continue;
            }
            f = add_func(wl);
            f->sym = i;
            f->module = tcc_strdup(name);
            f->module[dot - name] = 0;
            f->name = dot + 1;
            if (i < wl->nb_sym_sig && wl->sym_sig[i])
                f->sig = tcc_strdup(wl->sym_sig[i]);
            wl->sym_func[i] = wl->nb_funcs - 1;
        }
    }
    (void)s1;
}

/* learn the signatures of called functions from the call site markers */
static void scan_sigs(WasmLink *wl)
{
    TCCState *s1 = wl->s1;
    Section *s;
    ElfW_Rel *rel;
    int i, type, sym, last_call = -1;

    wl->nb_sym_sig = symtab_section->data_offset / sizeof(ElfW(Sym));
    wl->sym_sig = tcc_mallocz(wl->nb_sym_sig * sizeof(char *));
    for (i = 1; i < s1->nb_sections; i++) {
        s = s1->sections[i];
        if (!s->reloc || !(s->sh_flags & SHF_ALLOC))
            continue;
        for_each_elem(s->reloc, 0, rel, ElfW_Rel) {
            type = ELFW(R_TYPE)(rel->r_info);
            sym = ELFW(R_SYM)(rel->r_info);
            if (type == R_WASM_FUNC_LEB) {
                last_call = sym;
            } else if (type == R_WASM_SIG) {
                if (last_call > 0 && last_call < wl->nb_sym_sig && !wl->sym_sig[last_call])
                    wl->sym_sig[last_call] = elf_sym_name(sym) + strlen(".wasm.sig:");
                last_call = -1;
            }
        }
    }
}

/* calls to undefined weak functions go to a trapping stub */
static void synth_weak_stubs(WasmLink *wl)
{
    TCCState *s1 = wl->s1;
    int i;
    ElfW(Sym) *esym;
    for (i = 1; i < wl->nb_sym_sig; i++) {
        esym = elf_sym(i);
        if (esym->st_shndx == SHN_UNDEF && sym_is_func(esym)
            && ELFW(ST_BIND)(esym->st_info) == STB_WEAK && wl->sym_sig[i]
            && !strchr(elf_sym_name(i), '.')) {
            synth_trap_function(s1, elf_sym_name(i), wl->sym_sig[i]);
            dynarray_add(&wl->weak_stubs, &wl->nb_weak_stubs, (void *)(size_t)i);
        }
    }
}

/* find address-taken functions */
static void scan_relocs(WasmLink *wl)
{
    TCCState *s1 = wl->s1;
    Section *s;
    ElfW_Rel *rel;
    int i, type, sym, fi;
    WasmFunc *f;

    for (i = 1; i < s1->nb_sections; i++) {
        s = s1->sections[i];
        if (!s->reloc || !(s->sh_flags & SHF_ALLOC))
            continue;
        for_each_elem(s->reloc, 0, rel, ElfW_Rel) {
            type = ELFW(R_TYPE)(rel->r_info);
            sym = ELFW(R_SYM)(rel->r_info);
            switch (type) {
            case R_WASM_SLEB:
            case R_WASM_32:
                if ((fi = wl->sym_func[sym]) >= 0) {
                    f = &wl->funcs[fi];
                    if (!f->table && !f->weak_stub)
                        f->table = ++wl->nb_table;
                }
                break;
            }
        }
    }
}

static void assign_indices(WasmLink *wl)
{
    TCCState *s1 = wl->s1;
    int i, n = 0;
    WasmFunc *f;

    /* imports first */
    for (i = 0; i < wl->nb_funcs; i++) {
        f = &wl->funcs[i];
        if (!f->module)
            continue;
        if (!f->sig)
            tcc_error_noabort("cannot determine the signature of imported function '%s.%s' (it is never called directly)", f->module, f->name);
        else
            f->type = type_index(wl, f->sig);
        f->index = n++;
    }
    wl->nb_imports = n;
    for (i = 0; i < wl->nb_funcs; i++) {
        f = &wl->funcs[i];
        if (f->module)
            continue;
        f->type = type_index(wl, f->sig);
        f->index = n++;
    }
}

/* ---------------------------------------------------------------- */
/* memory layout */

static void set_abs_sym(TCCState *s1, const char *name, unsigned value)
{
    set_elf_sym(symtab_section, value, 0, ELFW(ST_INFO)(STB_GLOBAL, STT_NOTYPE),
                0, SHN_ABS, name);
}

static void layout_memory(WasmLink *wl)
{
    TCCState *s1 = wl->s1;
    Section *s;
    unsigned addr = WASM_DATA_START;
    int i, pass;

    for (pass = 0; pass < 2; pass++) {
        for (i = 1; i < s1->nb_sections; i++) {
            s = s1->sections[i];
            if (!(s->sh_flags & SHF_ALLOC) || s == text_section)
                continue;
            if ((s->sh_type == SHT_NOBITS) != pass)
                continue;
            if (s->sh_addralign > 1)
                addr = (addr + s->sh_addralign - 1) & -s->sh_addralign;
            s->sh_addr = addr;
            s->sh_size = s->data_offset;
            addr += s->sh_size;
        }
    }
    wl->data_end = addr;
    addr = (addr + 15) & -16;
    wl->stack_top = addr + WASM_STACK_SIZE;
    wl->heap_base = wl->stack_top;
    wl->heap_end = (wl->heap_base + WASM_PAGE_SIZE - 1) & -WASM_PAGE_SIZE;

    /* the usual wasm-ld synthesized symbols */
    set_abs_sym(s1, "__data_end", wl->data_end);
    set_abs_sym(s1, "__heap_base", wl->heap_base);
    set_abs_sym(s1, "__heap_end", wl->heap_end);
    set_abs_sym(s1, "__stack_high", wl->stack_top);
    set_abs_sym(s1, "__stack_low", wl->stack_top - WASM_STACK_SIZE);
    set_abs_sym(s1, "__global_base", WASM_DATA_START);
    set_abs_sym(s1, "__memory_base", 0);
    set_abs_sym(s1, "__table_base", 1);
    set_abs_sym(s1, "__wasm_first_page_end", WASM_PAGE_SIZE);
}

static int sym_defined(TCCState *s1, const char *name)
{
    int i = find_elf_sym(symtab_section, name);
    return i && elf_sym(i)->st_shndx != SHN_UNDEF;
}

static void define_array_syms(TCCState *s1, const char *secname, const char *start, const char *end)
{
    Section *s = find_section(s1, secname);
    if (!sym_defined(s1, start))
        set_global_sym(s1, start, s, 0);
    if (!sym_defined(s1, end))
        set_global_sym(s1, end, s, s->data_offset);
}

/* ---------------------------------------------------------------- */
/* relocations */

/* value of a symbol as seen from data (address or table index) */
static unsigned sym_value(WasmLink *wl, int sym, int *is_func)
{
    TCCState *s1 = wl->s1;
    ElfW(Sym) *esym = elf_sym(sym);
    int fi = wl->sym_func[sym];

    *is_func = 0;
    if (fi >= 0) {
        *is_func = 1;
        return wl->funcs[fi].table;
    }
    if (esym->st_shndx == SHN_ABS)
        return esym->st_value;
    if (esym->st_shndx == SHN_UNDEF) {
        if (ELFW(ST_BIND)(esym->st_info) != STB_WEAK)
            tcc_error_noabort("undefined symbol '%s'", elf_sym_name(sym));
        return 0;
    }
    if (esym->st_shndx == SHN_COMMON)
        tcc_error_noabort("internal: unresolved common symbol");
    return s1->sections[esym->st_shndx]->sh_addr + esym->st_value;
}

static void apply_relocs(WasmLink *wl)
{
    TCCState *s1 = wl->s1;
    Section *s;
    ElfW_Rel *rel;
    int i, type, sym, fi, is_func;
    unsigned char *p;
    unsigned val;

    for (i = 1; i < s1->nb_sections; i++) {
        s = s1->sections[i];
        if (!s->reloc || !(s->sh_flags & SHF_ALLOC))
            continue;
        for_each_elem(s->reloc, 0, rel, ElfW_Rel) {
            type = ELFW(R_TYPE)(rel->r_info);
            sym = ELFW(R_SYM)(rel->r_info);
            p = s->data + rel->r_offset;
            switch (type) {
            case R_WASM_FUNC_LEB:
                fi = wl->sym_func[sym];
                if (fi < 0) {
                    tcc_error_noabort("undefined function '%s'", elf_sym_name(sym));
                    break;
                }
                patch_uleb(p, wl->funcs[fi].index);
                break;
            case R_WASM_TYPE_LEB:
                patch_uleb(p, type_index(wl, elf_sym_name(sym) + strlen(".wasm.sig:")));
                break;
            case R_WASM_SLEB:
                val = sym_value(wl, sym, &is_func);
                if (is_func) {
                    if (read_sleb_padded(p))
                        tcc_error_noabort("offset from function symbol '%s'", elf_sym_name(sym));
                    patch_sleb(p, val);
                } else {
                    patch_sleb(p, val + read_sleb_padded(p));
                }
                break;
            case R_WASM_32:
                val = sym_value(wl, sym, &is_func);
                if (is_func)
                    write32le(p, val);
                else
                    add32le(p, val);
                break;
            case R_WASM_SIG:
                break;
            default:
                tcc_error_noabort("internal: unknown relocation type %d", type);
                break;
            }
        }
    }
}

static void check_undefined(WasmLink *wl)
{
    TCCState *s1 = wl->s1;
    int i, nb_syms = symtab_section->data_offset / sizeof(ElfW(Sym));
    ElfW(Sym) *esym;
    for (i = 1; i < nb_syms; i++) {
        esym = elf_sym(i);
        if (esym->st_shndx != SHN_UNDEF || wl->sym_func[i] >= 0)
            continue;
        if (ELFW(ST_BIND)(esym->st_info) == STB_WEAK)
            continue;
        if (sym_is_func(esym))
            continue; /* already reported */
        tcc_error_noabort("undefined symbol '%s'", elf_sym_name(i));
    }
}

/* ---------------------------------------------------------------- */
/* module emission */

static void emit_module(WasmLink *wl, CString *mod)
{
    TCCState *s1 = wl->s1;
    CString sec;
    int i, n;
    WasmFunc *f;
    Section *s;
    const char *entry = s1->elf_entryname ? s1->elf_entryname : "_start";
    int entry_sym = find_elf_sym(symtab_section, entry);

    cstr_new(&sec);
    cstr_cat(mod, "\0asm\1\0\0\0", 8);

    /* type section */
    wo_uleb(&sec, wl->nb_types);
    for (i = 0; i < wl->nb_types; i++) {
        const char *sig = wl->types[i], *colon = strchr(sig, ':');
        const char *p;
        wo_byte(&sec, 0x60);
        wo_uleb(&sec, colon - sig);
        for (p = sig; p < colon; p++)
            wo_byte(&sec, wasm_valtype(s1, *p));
        wo_uleb(&sec, strlen(colon + 1));
        for (p = colon + 1; *p; p++)
            wo_byte(&sec, wasm_valtype(s1, *p));
    }
    wo_section(mod, 1, &sec);

    /* import section */
    if (wl->nb_imports) {
        wo_uleb(&sec, wl->nb_imports);
        for (i = 0; i < wl->nb_funcs; i++) {
            f = &wl->funcs[i];
            if (!f->module)
                continue;
            wo_name(&sec, f->module);
            wo_name(&sec, f->name);
            wo_byte(&sec, 0x00);
            wo_uleb(&sec, f->type);
        }
        wo_section(mod, 2, &sec);
    }

    /* function section */
    wo_uleb(&sec, wl->nb_funcs - wl->nb_imports);
    for (i = 0; i < wl->nb_funcs; i++) {
        f = &wl->funcs[i];
        if (!f->module)
            wo_uleb(&sec, f->type);
    }
    wo_section(mod, 3, &sec);

    /* table section */
    wo_uleb(&sec, 1);
    wo_byte(&sec, 0x70);
    wo_byte(&sec, 0x01);
    wo_uleb(&sec, wl->nb_table + 1);
    wo_uleb(&sec, wl->nb_table + 1);
    wo_section(mod, 4, &sec);

    /* memory section */
    wo_uleb(&sec, 1);
    wo_byte(&sec, 0x00);
    wo_uleb(&sec, wl->heap_end / WASM_PAGE_SIZE);
    wo_section(mod, 5, &sec);

    /* global section: __stack_pointer */
    wo_uleb(&sec, 1);
    wo_byte(&sec, 0x7f);
    wo_byte(&sec, 0x01);
    wo_byte(&sec, 0x41);
    wo_sleb(&sec, wl->stack_top);
    wo_byte(&sec, 0x0b);
    wo_section(mod, 6, &sec);

    /* export section */
    n = 1 + (entry_sym && wl->sym_func[entry_sym] >= 0);
    wo_uleb(&sec, n);
    wo_name(&sec, "memory");
    wo_byte(&sec, 0x02);
    wo_uleb(&sec, 0);
    if (n > 1) {
        wo_name(&sec, entry);
        wo_byte(&sec, 0x00);
        wo_uleb(&sec, wl->funcs[wl->sym_func[entry_sym]].index);
    }
    wo_section(mod, 7, &sec);

    /* element section: the function table */
    if (wl->nb_table) {
        int *tab = tcc_mallocz((wl->nb_table + 1) * sizeof(int));
        for (i = 0; i < wl->nb_funcs; i++)
            if (wl->funcs[i].table)
                tab[wl->funcs[i].table] = wl->funcs[i].index;
        wo_uleb(&sec, 1);
        wo_byte(&sec, 0x00);
        wo_byte(&sec, 0x41);
        wo_sleb(&sec, 1);
        wo_byte(&sec, 0x0b);
        wo_uleb(&sec, wl->nb_table);
        for (i = 1; i <= wl->nb_table; i++)
            wo_uleb(&sec, tab[i]);
        wo_section(mod, 9, &sec);
        tcc_free(tab);
    }

    /* code section */
    wo_uleb(&sec, wl->nb_funcs - wl->nb_imports);
    for (i = 0; i < wl->nb_funcs; i++) {
        int hdr, len;
        f = &wl->funcs[i];
        if (f->module)
            continue;
        hdr = strlen(f->sig) + 1;
        len = f->size - hdr;
        if (len <= 0)
            tcc_error_noabort("internal: empty function body for '%s'", f->name);
        wo_uleb(&sec, len);
        cstr_cat(&sec, (char *)text_section->data + f->offset + hdr, len);
    }
    wo_section(mod, 10, &sec);

    /* data section */
    n = 0;
    for (i = 1; i < s1->nb_sections; i++) {
        s = s1->sections[i];
        if ((s->sh_flags & SHF_ALLOC) && s != text_section
            && s->sh_type != SHT_NOBITS && s->data_offset)
            n++;
    }
    if (n) {
        wo_uleb(&sec, n);
        for (i = 1; i < s1->nb_sections; i++) {
            s = s1->sections[i];
            if (!((s->sh_flags & SHF_ALLOC) && s != text_section
                  && s->sh_type != SHT_NOBITS && s->data_offset))
                continue;
            wo_byte(&sec, 0x00);
            wo_byte(&sec, 0x41);
            wo_sleb(&sec, s->sh_addr);
            wo_byte(&sec, 0x0b);
            wo_uleb(&sec, s->data_offset);
            cstr_cat(&sec, (char *)s->data, s->data_offset);
        }
        wo_section(mod, 11, &sec);
    }

    /* custom "name" section with function names */
    {
        CString names;
        cstr_new(&names);
        int pass;
        wo_uleb(&names, wl->nb_funcs);
        for (pass = 0; pass < 2; pass++) { /* by index: imports first */
            for (i = 0; i < wl->nb_funcs; i++) {
                f = &wl->funcs[i];
                if ((f->module != NULL) == pass)
                    continue;
                wo_uleb(&names, f->index);
                wo_name(&names, f->name);
            }
        }
        wo_name(&sec, "name");
        wo_byte(&sec, 1);
        wo_uleb(&sec, names.size);
        cstr_cat(&sec, names.data, names.size);
        cstr_free(&names);
        wo_section(mod, 0, &sec);
    }
    cstr_free(&sec);
}

/* ---------------------------------------------------------------- */

ST_FUNC int wasm_output_file(TCCState *s1, const char *filename)
{
    WasmLink wl;
    CString mod;
    FILE *f;
    int i, ret = -1;

    memset(&wl, 0, sizeof wl);
    wl.s1 = s1;

    /* reference the entry point so that it is pulled from the runtime;
       destructors are registered with atexit (see synth_call_ctors) */
    {
        const char *entry = s1->elf_entryname ? s1->elf_entryname : "_start";
        Section *fa = find_section(s1, ".fini_array");
        if (!find_elf_sym(symtab_section, entry))
            set_global_sym(s1, entry, NULL, 0);
        if (fa->data_offset && !find_elf_sym(symtab_section, "atexit"))
            set_global_sym(s1, "atexit", NULL, 0);
    }
    tcc_add_runtime(s1);
    resolve_common_syms(s1);
    define_array_syms(s1, ".init_array", "__init_array_start", "__init_array_end");
    define_array_syms(s1, ".fini_array", "__fini_array_start", "__fini_array_end");
    i = find_elf_sym(symtab_section, "__wasm_call_ctors");
    if (!i || elf_sym(i)->st_shndx == SHN_UNDEF)
        synth_call_ctors(s1);
    scan_sigs(&wl);
    synth_weak_stubs(&wl);

    layout_memory(&wl);
    collect_functions(&wl);
    scan_relocs(&wl);
    assign_indices(&wl);
    check_undefined(&wl);
    if (s1->nb_errors)
        goto the_end;
    apply_relocs(&wl);
    if (s1->nb_errors)
        goto the_end;

    cstr_new(&mod);
    emit_module(&wl, &mod);
    f = fopen(filename, "wb");
    if (!f) {
        tcc_error_noabort("could not write '%s: %s'", filename, strerror(errno));
    } else {
        if (fwrite(mod.data, 1, mod.size, f) != mod.size)
            tcc_error_noabort("could not write '%s'", filename);
        else
            ret = 0;
        fclose(f);
    }
    cstr_free(&mod);

 the_end:
    for (i = 0; i < wl.nb_funcs; i++) {
        if (wl.funcs[i].module) {
            tcc_free(wl.funcs[i].module);
            tcc_free((char *)wl.funcs[i].sig);
        }
    }
    tcc_free(wl.funcs);
    tcc_free(wl.sym_func);
    tcc_free(wl.sym_sig);
    tcc_free(wl.weak_stubs);
    tcc_free(wl.off_func);
    dynarray_reset(&wl.types, &wl.nb_types);
    return ret;
}

/* ---------------------------------------------------------------- */
/* signature symbols (shared with wasm32-gen.c) */

ST_FUNC int wasm_sig_symbol(TCCState *s1, const char *sig)
{
    char name[300];
    int c;
    snprintf(name, sizeof name, ".wasm.sig:%s", sig);
    c = find_elf_sym(symtab_section, name);
    if (!c)
        c = put_elf_sym(symtab_section, 0, 0,
                        ELFW(ST_INFO)(STB_WEAK, STT_NOTYPE), 0, SHN_ABS, name);
    return c;
}

/* ---------------------------------------------------------------- */
/* Reading wasm object files (as produced by clang/llvm, e.g. wasi-libc)
   into tcc's sections, following the same conventions as wasm32-gen.c */

/* wasm object relocation types */
enum {
    WR_FUNCTION_INDEX_LEB = 0,
    WR_TABLE_INDEX_SLEB = 1,
    WR_TABLE_INDEX_I32 = 2,
    WR_MEMORY_ADDR_LEB = 3,
    WR_MEMORY_ADDR_SLEB = 4,
    WR_MEMORY_ADDR_I32 = 5,
    WR_TYPE_INDEX_LEB = 6,
    WR_GLOBAL_INDEX_LEB = 7,
    WR_FUNCTION_OFFSET_I32 = 8,
    WR_SECTION_OFFSET_I32 = 9,
    WR_TAG_INDEX_LEB = 10,
    WR_MEMORY_ADDR_REL_SLEB = 11,
    WR_TABLE_INDEX_REL_SLEB = 12,
    WR_GLOBAL_INDEX_I32 = 13,
    WR_TABLE_NUMBER_LEB = 20,
    WR_MEMORY_ADDR_TLS_SLEB = 21,
    WR_FUNCTION_INDEX_I32 = 26,
};

/* symbol kinds / flags in the "linking" section */
enum {
    WSYM_FUNCTION = 0, WSYM_DATA = 1, WSYM_GLOBAL = 2, WSYM_SECTION = 3,
    WSYM_TAG = 4, WSYM_TABLE = 5,
};
#define WSYM_BINDING_WEAK 1
#define WSYM_BINDING_LOCAL 2
#define WSYM_VISIBILITY_HIDDEN 4
#define WSYM_UNDEFINED 0x10
#define WSYM_EXPORTED 0x20
#define WSYM_EXPLICIT_NAME 0x40
#define WSYM_NO_STRIP 0x80

typedef struct WRd {
    const unsigned char *p, *end;
    TCCState *s1;
    int err;
} WRd;

typedef struct WImport {
    char *module, *name;
    int kind, type;
} WImport;

typedef struct WSeg {
    unsigned payload;    /* offset of the bytes in the data section contents */
    unsigned size;
    char *name;
    int p2align;
    Section *sec;        /* target tcc section */
    unsigned sec_off;
} WSeg;

typedef struct WSym {
    int kind, flags, index;
    unsigned seg, off, size;
    char *name;
    int elfsym;
} WSym;

typedef struct WObj {
    TCCState *s1;
    unsigned char *buf;
    unsigned len;
    char **sigs; int nb_sigs;                 /* type section */
    WImport **imports; int nb_imports;
    int nb_func_imports;
    int *func_types; int nb_funcs;            /* defined functions */
    unsigned *body_start, *body_len;          /* in code contents */
    unsigned *text_off;                       /* in text_section */
    unsigned code_contents, code_len;         /* code section contents */
    unsigned data_contents, data_len;
    WSeg **segs; int nb_segs;
    WSym **syms; int nb_syms;
    int code_secidx, data_secidx;
    /* reloc sections, processed after linking info */
    unsigned reloc_code, reloc_code_len, reloc_data, reloc_data_len;
    unsigned linking, linking_len;
} WObj;

static unsigned rd_uleb(WRd *r)
{
    unsigned v = 0, shift = 0;
    int b;
    do {
        if (r->p >= r->end) { r->err = 1; return 0; }
        b = *r->p++;
        v |= (unsigned)(b & 0x7f) << shift;
        shift += 7;
    } while (b & 0x80);
    return v;
}

static int rd_sleb(WRd *r)
{
    unsigned v = 0, shift = 0;
    int b;
    do {
        if (r->p >= r->end) { r->err = 1; return 0; }
        b = *r->p++;
        v |= (unsigned)(b & 0x7f) << shift;
        shift += 7;
    } while (b & 0x80);
    if (shift < 32 && (b & 0x40))
        v |= -(1u << shift);
    return (int)v;
}

static int rd_byte(WRd *r)
{
    if (r->p >= r->end) { r->err = 1; return 0; }
    return *r->p++;
}

static char *rd_name(WRd *r)
{
    unsigned len = rd_uleb(r);
    char *s;
    if (r->end - r->p < len) { r->err = 1; return tcc_strdup(""); }
    s = tcc_malloc(len + 1);
    memcpy(s, r->p, len);
    s[len] = 0;
    r->p += len;
    return s;
}

static void rd_skip(WRd *r, unsigned n)
{
    if (r->end - r->p < n) { r->err = 1; return; }
    r->p += n;
}

static char sig_char(WRd *r, int vt)
{
    switch (vt) {
    case 0x7f: return 'i';
    case 0x7e: return 'I';
    case 0x7d: return 'f';
    case 0x7c: return 'F';
    }
    r->err = 2;
    return 'i';
}

static void wobj_error(WObj *o, const char *msg)
{
    TCCState *s1 = o->s1;
    tcc_error_noabort("wasm object: %s", msg);
}

/* the kind/type of the n-th function in the object's index space */
static const char *func_sig_of(WObj *o, int idx)
{
    int t;
    if (idx < o->nb_func_imports) {
        int i, n = 0;
        for (i = 0; i < o->nb_imports; i++)
            if (o->imports[i]->kind == 0 && n++ == idx)
                return o->sigs[o->imports[i]->type];
        return ":";
    }
    t = o->func_types[idx - o->nb_func_imports];
    return o->sigs[t];
}

static void parse_type_section(WObj *o, WRd *r)
{
    unsigned n = rd_uleb(r), i, np, nr, j;
    char buf[256];
    for (i = 0; i < n && !r->err; i++) {
        int k = 0;
        if (rd_byte(r) != 0x60) { r->err = 1; return; }
        np = rd_uleb(r);
        for (j = 0; j < np && k < 250; j++)
            buf[k++] = sig_char(r, rd_byte(r));
        buf[k++] = ':';
        nr = rd_uleb(r);
        if (nr > 1) { r->err = 3; return; }
        for (j = 0; j < nr; j++)
            buf[k++] = sig_char(r, rd_byte(r));
        buf[k] = 0;
        dynarray_add(&o->sigs, &o->nb_sigs, tcc_strdup(buf));
    }
}

static void parse_import_section(WObj *o, WRd *r)
{
    unsigned n = rd_uleb(r), i;
    for (i = 0; i < n && !r->err; i++) {
        WImport *im = tcc_mallocz(sizeof *im);
        im->module = rd_name(r);
        im->name = rd_name(r);
        im->kind = rd_byte(r);
        switch (im->kind) {
        case 0: /* function */
            im->type = rd_uleb(r);
            o->nb_func_imports++;
            break;
        case 1: /* table */
            rd_byte(r);
            goto limits;
        case 2: /* memory */
        limits:
            {
                int flags = rd_byte(r);
                rd_uleb(r);
                if (flags & 1)
                    rd_uleb(r);
            }
            break;
        case 3: /* global */
            rd_byte(r);
            rd_byte(r);
            break;
        default:
            r->err = 1;
            break;
        }
        dynarray_add(&o->imports, &o->nb_imports, im);
    }
}

static void parse_function_section(WObj *o, WRd *r)
{
    unsigned n = rd_uleb(r), i;
    o->nb_funcs = n;
    o->func_types = tcc_mallocz((n + 1) * sizeof(int));
    for (i = 0; i < n && !r->err; i++)
        o->func_types[i] = rd_uleb(r);
}

static void parse_code_section(WObj *o, WRd *r, unsigned contents)
{
    unsigned n = rd_uleb(r), i, size;
    if (n != (unsigned)o->nb_funcs) { r->err = 1; return; }
    o->body_start = tcc_mallocz((n + 1) * sizeof(unsigned));
    o->body_len = tcc_mallocz((n + 1) * sizeof(unsigned));
    o->text_off = tcc_mallocz((n + 1) * sizeof(unsigned));
    for (i = 0; i < n && !r->err; i++) {
        size = rd_uleb(r);
        o->body_start[i] = (r->p - o->buf) - contents;
        o->body_len[i] = size;
        rd_skip(r, size);
    }
}

static void parse_data_section(WObj *o, WRd *r, unsigned contents)
{
    unsigned n = rd_uleb(r), i, flags, size;
    for (i = 0; i < n && !r->err; i++) {
        WSeg *seg = tcc_mallocz(sizeof *seg);
        flags = rd_uleb(r);
        if (flags & 2)
            rd_uleb(r); /* memory index */
        if (!(flags & 1)) {
            /* offset expression: i32.const N end */
            if (rd_byte(r) != 0x41) { r->err = 4; return; }
            rd_sleb(r);
            if (rd_byte(r) != 0x0b) { r->err = 4; return; }
        }
        size = rd_uleb(r);
        seg->payload = (r->p - o->buf) - contents;
        seg->size = size;
        rd_skip(r, size);
        dynarray_add(&o->segs, &o->nb_segs, seg);
    }
}

static void parse_linking_section(WObj *o, WRd *r)
{
    unsigned version = rd_uleb(r), n, i, id, len;
    const unsigned char *sub_end;
    if (version != 2) { r->err = 5; return; }
    while (r->p < r->end && !r->err) {
        id = rd_byte(r);
        len = rd_uleb(r);
        sub_end = r->p + len;
        switch (id) {
        case 5: /* WASM_SEGMENT_INFO */
            n = rd_uleb(r);
            for (i = 0; i < n && i < (unsigned)o->nb_segs && !r->err; i++) {
                o->segs[i]->name = rd_name(r);
                o->segs[i]->p2align = rd_uleb(r);
                rd_uleb(r); /* flags */
            }
            break;
        case 6: /* WASM_INIT_FUNCS */
            n = rd_uleb(r);
            for (i = 0; i < n && !r->err; i++) {
                int prio = rd_uleb(r), sym = rd_uleb(r);
                /* symbols are not created yet: remember the wasm symbol index,
                   translated later (flag with a negative index) */
                TCCState *s1 = o->s1;
                s1->wasm_ctors = tcc_realloc(s1->wasm_ctors, (s1->nb_wasm_ctors + 1) * sizeof(*s1->wasm_ctors));
                s1->wasm_ctors[s1->nb_wasm_ctors].sym = -1 - sym;
                s1->wasm_ctors[s1->nb_wasm_ctors].priority = prio;
                s1->nb_wasm_ctors++;
            }
            break;
        case 8: /* WASM_SYMBOL_TABLE */
            n = rd_uleb(r);
            for (i = 0; i < n && !r->err; i++) {
                WSym *sym = tcc_mallocz(sizeof *sym);
                sym->kind = rd_byte(r);
                sym->flags = rd_uleb(r);
                switch (sym->kind) {
                case WSYM_FUNCTION:
                case WSYM_GLOBAL:
                case WSYM_TAG:
                case WSYM_TABLE:
                    sym->index = rd_uleb(r);
                    if (!(sym->flags & WSYM_UNDEFINED) || (sym->flags & WSYM_EXPLICIT_NAME))
                        sym->name = rd_name(r);
                    break;
                case WSYM_DATA:
                    sym->name = rd_name(r);
                    if (!(sym->flags & WSYM_UNDEFINED)) {
                        sym->seg = rd_uleb(r);
                        sym->off = rd_uleb(r);
                        sym->size = rd_uleb(r);
                    }
                    break;
                case WSYM_SECTION:
                    sym->index = rd_uleb(r);
                    break;
                default:
                    r->err = 6;
                    break;
                }
                dynarray_add(&o->syms, &o->nb_syms, sym);
            }
            break;
        default: /* comdat info etc. */
            break;
        }
        r->p = sub_end;
    }
}

/* find the import record of an imported index of the given kind */
static WImport *find_import(WObj *o, int kind, int idx)
{
    int i, n = 0;
    for (i = 0; i < o->nb_imports; i++)
        if (o->imports[i]->kind == kind && n++ == idx)
            return o->imports[i];
    return NULL;
}
#define func_import(o, idx) find_import(o, 0, idx)

static Section *segment_section(TCCState *s1, const char *name)
{
    if (!name)
        return data_section;
    if (!strncmp(name, ".bss", 4))
        return bss_section;
    if (!strncmp(name, ".rodata", 7))
        return rodata_section;
    return data_section;
}

/* create the ELF symbols */
static void define_symbols(WObj *o)
{
    TCCState *s1 = o->s1;
    int i, bind, other, info;
    WSym *sym;
    char buf[512];
    const char *name;

    for (i = 0; i < o->nb_syms; i++) {
        sym = o->syms[i];
        bind = (sym->flags & WSYM_BINDING_WEAK) ? STB_WEAK
             : (sym->flags & WSYM_BINDING_LOCAL) ? STB_LOCAL : STB_GLOBAL;
        other = (sym->flags & WSYM_VISIBILITY_HIDDEN) ? STV_HIDDEN : STV_DEFAULT;
        name = sym->name;
        switch (sym->kind) {
        case WSYM_FUNCTION:
            info = ELFW(ST_INFO)(bind, STT_FUNC);
            if (sym->flags & WSYM_UNDEFINED) {
                WImport *im = func_import(o, sym->index);
                if (!im) { wobj_error(o, "bad function import"); break; }
                if (strcmp(im->module, "env")) {
                    snprintf(buf, sizeof buf, "%s.%s", im->module, im->name);
                    name = buf;
                } else if (!name) {
                    name = im->name;
                }
                if (bind == STB_LOCAL)
                    bind = STB_GLOBAL;
                sym->elfsym = set_elf_sym(symtab_section, 0, 0, ELFW(ST_INFO)(bind, STT_FUNC), other, SHN_UNDEF, name);
            } else {
                int fi = sym->index - o->nb_func_imports;
                if (fi < 0 || fi >= o->nb_funcs) { wobj_error(o, "bad function index"); break; }
                sym->elfsym = set_elf_sym(symtab_section, o->text_off[fi],
                    strlen(o->sigs[o->func_types[fi]]) + 1 + o->body_len[fi],
                    info, other, text_section->sh_num, name ? name : "");
            }
            break;
        case WSYM_DATA:
            if (sym->flags & WSYM_UNDEFINED) {
                sym->elfsym = set_elf_sym(symtab_section, 0, 0, ELFW(ST_INFO)(bind, STT_OBJECT), other, SHN_UNDEF, name);
            } else {
                WSeg *seg;
                if (sym->seg >= (unsigned)o->nb_segs) { wobj_error(o, "bad segment index"); break; }
                seg = o->segs[sym->seg];
                sym->elfsym = set_elf_sym(symtab_section, seg->sec_off + sym->off, sym->size,
                    ELFW(ST_INFO)(bind, STT_OBJECT), other, seg->sec->sh_num, name);
            }
            break;
        case WSYM_GLOBAL:
            if (sym->flags & WSYM_UNDEFINED) {
                WImport *im = find_import(o, 3, sym->index);
                if (!name && im)
                    name = im->name;
            }
            if (!(sym->flags & WSYM_UNDEFINED) || !name || strcmp(name, "__stack_pointer")) {
                snprintf(buf, sizeof buf, "unsupported global '%s' (only __stack_pointer is supported)", name ? name : "?");
                wobj_error(o, buf);
            }
            break;
        case WSYM_TABLE:
            if (!(sym->flags & WSYM_UNDEFINED))
                wobj_error(o, "unsupported table definition");
            break;
        default:
            break;
        }
    }
    /* resolve the init function symbols recorded from this object */
    for (i = 0; i < s1->nb_wasm_ctors; i++) {
        int ws = s1->wasm_ctors[i].sym;
        if (ws < 0) {
            ws = -1 - ws;
            if (ws < o->nb_syms)
                s1->wasm_ctors[i].sym = o->syms[ws]->elfsym;
            else
                s1->wasm_ctors[i].sym = 0;
        }
    }
}

static int find_body(WObj *o, unsigned off)
{
    int i;
    for (i = 0; i < o->nb_funcs; i++)
        if (off >= o->body_start[i] && off < o->body_start[i] + o->body_len[i])
            return i;
    return -1;
}

static int find_seg(WObj *o, unsigned off)
{
    int i;
    for (i = 0; i < o->nb_segs; i++)
        if (off >= o->segs[i]->payload && off < o->segs[i]->payload + o->segs[i]->size)
            return i;
    return -1;
}

static void parse_reloc_section(WObj *o, WRd *r)
{
    TCCState *s1 = o->s1;
    unsigned secidx = rd_uleb(r), n = rd_uleb(r), i;
    int is_code = secidx == (unsigned)o->code_secidx;
    int is_data = secidx == (unsigned)o->data_secidx;

    if (!is_code && !is_data)
        return;
    for (i = 0; i < n && !r->err; i++) {
        int type = rd_byte(r);
        unsigned off = rd_uleb(r);
        unsigned idx = rd_uleb(r);
        int addend = 0, k, esym;
        Section *sec;
        unsigned toff;
        unsigned char *p;
        WSym *sym = NULL;

        if (type == WR_MEMORY_ADDR_LEB || type == WR_MEMORY_ADDR_SLEB
            || type == WR_MEMORY_ADDR_I32 || type == WR_FUNCTION_OFFSET_I32
            || type == WR_SECTION_OFFSET_I32 || type == WR_MEMORY_ADDR_REL_SLEB)
            addend = rd_sleb(r);
        if (r->err)
            break;
        if (type != WR_TYPE_INDEX_LEB) {
            if (idx >= (unsigned)o->nb_syms) { wobj_error(o, "bad relocation symbol"); continue; }
            sym = o->syms[idx];
        }
        if (is_code) {
            k = find_body(o, off);
            if (k < 0) { wobj_error(o, "code relocation outside function"); continue; }
            sec = text_section;
            toff = o->text_off[k] + strlen(o->sigs[o->func_types[k]]) + 1 + (off - o->body_start[k]);
        } else {
            k = find_seg(o, off);
            if (k < 0) { wobj_error(o, "data relocation outside segment"); continue; }
            sec = o->segs[k]->sec;
            if (sec->sh_type == SHT_NOBITS) { wobj_error(o, "relocation in bss"); continue; }
            toff = o->segs[k]->sec_off + (off - o->segs[k]->payload);
        }
        p = sec->data + toff;
        esym = sym ? sym->elfsym : 0;
        switch (type) {
        case WR_FUNCTION_INDEX_LEB:
            put_elf_reloca(symtab_section, sec, toff, R_WASM_FUNC_LEB, esym, 0);
            put_elf_reloca(symtab_section, sec, toff + 5, R_WASM_SIG,
                           wasm_sig_symbol(s1, func_sig_of(o, sym->index)), 0);
            break;
        case WR_TABLE_INDEX_SLEB:
            patch_sleb(p, 0);
            put_elf_reloca(symtab_section, sec, toff, R_WASM_SLEB, esym, 0);
            break;
        case WR_TABLE_INDEX_I32:
            write32le(p, 0);
            put_elf_reloca(symtab_section, sec, toff, R_WASM_32, esym, 0);
            break;
        case WR_MEMORY_ADDR_LEB:
        case WR_MEMORY_ADDR_SLEB:
            patch_sleb(p, addend);
            put_elf_reloca(symtab_section, sec, toff, R_WASM_SLEB, esym, 0);
            break;
        case WR_MEMORY_ADDR_I32:
            write32le(p, addend);
            put_elf_reloca(symtab_section, sec, toff, R_WASM_32, esym, 0);
            break;
        case WR_TYPE_INDEX_LEB:
            if (idx >= (unsigned)o->nb_sigs) { wobj_error(o, "bad type index"); break; }
            put_elf_reloca(symtab_section, sec, toff, R_WASM_TYPE_LEB, wasm_sig_symbol(s1, o->sigs[idx]), 0);
            break;
        case WR_GLOBAL_INDEX_LEB:
            patch_uleb(p, 0); /* __stack_pointer is global 0 */
            break;
        case WR_TABLE_NUMBER_LEB:
            patch_uleb(p, 0);
            break;
        case WR_GLOBAL_INDEX_I32:
            write32le(p, 0);
            break;
        default:
            tcc_error_noabort("wasm object: unsupported relocation type %d", type);
            break;
        }
    }
}

static void wobj_free(WObj *o)
{
    int i;
    for (i = 0; i < o->nb_imports; i++) {
        tcc_free(o->imports[i]->module);
        tcc_free(o->imports[i]->name);
        tcc_free(o->imports[i]);
    }
    tcc_free(o->imports);
    for (i = 0; i < o->nb_segs; i++) {
        tcc_free(o->segs[i]->name);
        tcc_free(o->segs[i]);
    }
    tcc_free(o->segs);
    for (i = 0; i < o->nb_syms; i++) {
        tcc_free(o->syms[i]->name);
        tcc_free(o->syms[i]);
    }
    tcc_free(o->syms);
    dynarray_reset(&o->sigs, &o->nb_sigs);
    tcc_free(o->func_types);
    tcc_free(o->body_start);
    tcc_free(o->body_len);
    tcc_free(o->text_off);
    tcc_free(o->buf);
}

/* size of the object at 'file_offset' (a file or an archive member) */
static unsigned wasm_object_size(int fd, unsigned long file_offset)
{
    char hdr[60];
    if (file_offset == 0)
        return lseek(fd, 0, SEEK_END);
    lseek(fd, file_offset - 60, SEEK_SET);
    if (full_read(fd, hdr, 60) != 60)
        return 0;
    return strtoul(hdr + 48, NULL, 10);
}

ST_FUNC int tcc_load_wasm_object(TCCState *s1, int fd, unsigned long file_offset)
{
    WObj o;
    WRd r;
    unsigned size, id, len, contents, secidx = 0;
    int i, ret = -1;

    memset(&o, 0, sizeof o);
    o.s1 = s1;
    o.code_secidx = o.data_secidx = -1;
    size = wasm_object_size(fd, file_offset);
    if (size < 8)
        return tcc_error_noabort("invalid wasm object");
    o.buf = tcc_malloc(size);
    o.len = size;
    lseek(fd, file_offset, SEEK_SET);
    if (full_read(fd, o.buf, size) != size || memcmp(o.buf, "\0asm\1\0\0\0", 8)) {
        tcc_error_noabort("invalid wasm object");
        goto the_end;
    }
    r.s1 = s1;
    r.err = 0;
    r.p = o.buf + 8;
    r.end = o.buf + size;

    /* first pass: everything but linking/reloc info */
    while (r.p < r.end && !r.err) {
        id = rd_byte(&r);
        len = rd_uleb(&r);
        contents = r.p - o.buf;
        if (r.end - r.p < len) { r.err = 1; break; }
        r.end = r.p + len;
        switch (id) {
        case 1: parse_type_section(&o, &r); break;
        case 2: parse_import_section(&o, &r); break;
        case 3: parse_function_section(&o, &r); break;
        case 10:
            o.code_secidx = secidx;
            o.code_contents = contents;
            o.code_len = len;
            parse_code_section(&o, &r, contents);
            break;
        case 11:
            o.data_secidx = secidx;
            o.data_contents = contents;
            o.data_len = len;
            parse_data_section(&o, &r, contents);
            break;
        case 0: {
            char *name = rd_name(&r);
            if (!strcmp(name, "linking")) {
                o.linking = r.p - o.buf;
                o.linking_len = r.end - r.p;
            } else if (!strcmp(name, "reloc.CODE")) {
                o.reloc_code = r.p - o.buf;
                o.reloc_code_len = r.end - r.p;
            } else if (!strcmp(name, "reloc.DATA")) {
                o.reloc_data = r.p - o.buf;
                o.reloc_data_len = r.end - r.p;
            }
            tcc_free(name);
            break;
        }
        default:
            break;
        }
        r.p = r.end;
        r.end = o.buf + size;
        secidx++;
    }
    if (r.err) {
        tcc_error_noabort("wasm object: parse error %d", r.err);
        goto the_end;
    }
    if (!o.linking) {
        tcc_error_noabort("wasm object: not a relocatable object (no linking section)");
        goto the_end;
    }

    /* copy function bodies to .text, each preceded by its signature */
    for (i = 0; i < o.nb_funcs; i++) {
        const char *sig = o.sigs[o.func_types[i]];
        int hdr = strlen(sig) + 1;
        o.text_off[i] = section_add(text_section, hdr + o.body_len[i], 1);
        memcpy(text_section->data + o.text_off[i], sig, hdr);
        memcpy(text_section->data + o.text_off[i] + hdr,
               o.buf + o.code_contents + o.body_start[i], o.body_len[i]);
    }

    /* linking section: segment names, symbols, init functions */
    r.p = o.buf + o.linking;
    r.end = r.p + o.linking_len;
    parse_linking_section(&o, &r);
    if (r.err) {
        tcc_error_noabort("wasm object: bad linking section (%d)", r.err);
        goto the_end;
    }

    /* copy data segments */
    for (i = 0; i < o.nb_segs; i++) {
        WSeg *seg = o.segs[i];
        seg->sec = segment_section(s1, seg->name);
        seg->sec_off = section_add(seg->sec, seg->size, 1 << seg->p2align);
        if (seg->sec->sh_type != SHT_NOBITS)
            memcpy(seg->sec->data + seg->sec_off, o.buf + o.data_contents + seg->payload, seg->size);
    }

    define_symbols(&o);

    if (o.reloc_code) {
        r.p = o.buf + o.reloc_code;
        r.end = r.p + o.reloc_code_len;
        parse_reloc_section(&o, &r);
    }
    if (o.reloc_data && !r.err) {
        r.p = o.buf + o.reloc_data;
        r.end = r.p + o.reloc_data_len;
        parse_reloc_section(&o, &r);
    }
    if (r.err)
        tcc_error_noabort("wasm object: bad relocation section");
    ret = s1->nb_errors ? -1 : 0;

 the_end:
    wobj_free(&o);
    return ret;
}

/* ---------------------------------------------------------------- */
/* synthesized functions */

/* append a function "sig" with wasm body 'code' to .text and define
   its symbol; returns the text offset of the body's code */
static unsigned add_synth_function(TCCState *s1, const char *name, const char *sig, CString *code)
{
    int hdr = strlen(sig) + 1;
    unsigned off = section_add(text_section, hdr + code->size, 1);
    memcpy(text_section->data + off, sig, hdr);
    memcpy(text_section->data + off + hdr, code->data, code->size);
    set_elf_sym(symtab_section, off, hdr + code->size,
                ELFW(ST_INFO)(STB_GLOBAL, STT_FUNC), STV_HIDDEN, text_section->sh_num, name);
    return off + hdr;
}

static int ctor_cmp(const void *a, const void *b)
{
    const struct { int sym, priority; } *x = a, *y = b;
    return x->priority - y->priority;
}

/* a 'call' with a padded function index and its signature marker */
static void synth_call(TCCState *s1, CString *code, unsigned base, int sym, const char *sig)
{
    unsigned off = base + code->size;
    wo_byte(code, 0x10);
    wo_byte(code, 0x80); wo_byte(code, 0x80); wo_byte(code, 0x80); wo_byte(code, 0x80); wo_byte(code, 0);
    put_elf_reloca(symtab_section, text_section, off + 1, R_WASM_FUNC_LEB, sym, 0);
    put_elf_reloca(symtab_section, text_section, off + 6, R_WASM_SIG, wasm_sig_symbol(s1, sig), 0);
}

/* __wasm_call_ctors: calls the wasm init functions in priority order,
   then the .init_array entries, and registers the .fini_array entries
   with atexit (like clang does for destructors) */
static void synth_call_ctors(TCCState *s1)
{
    CString code;
    Section *ia, *fa;
    unsigned base, hdr = strlen(":") + 1;
    int i, atexit_sym;
    ElfW_Rel *rel;

    /* the function is placed at the current end of .text */
    base = text_section->data_offset + hdr;
    cstr_new(&code);
    wo_uleb(&code, 0); /* no locals */
    if (s1->nb_wasm_ctors > 1)
        qsort(s1->wasm_ctors, s1->nb_wasm_ctors, sizeof(*s1->wasm_ctors), ctor_cmp);
    for (i = 0; i < s1->nb_wasm_ctors; i++)
        if (s1->wasm_ctors[i].sym > 0)
            synth_call(s1, &code, base, s1->wasm_ctors[i].sym, ":");
    ia = find_section(s1, ".init_array");
    if (ia->reloc)
        for_each_elem(ia->reloc, 0, rel, ElfW_Rel)
            synth_call(s1, &code, base, ELFW(R_SYM)(rel->r_info), ":");
    fa = find_section(s1, ".fini_array");
    atexit_sym = find_elf_sym(symtab_section, "atexit");
    if (fa->reloc && atexit_sym) {
        for_each_elem(fa->reloc, 0, rel, ElfW_Rel) {
            /* i32.const <table index>; call atexit; drop */
            wo_byte(&code, 0x41);
            put_elf_reloca(symtab_section, text_section, base + code.size, R_WASM_SLEB, ELFW(R_SYM)(rel->r_info), 0);
            wo_byte(&code, 0x80); wo_byte(&code, 0x80); wo_byte(&code, 0x80); wo_byte(&code, 0x80); wo_byte(&code, 0);
            synth_call(s1, &code, base, atexit_sym, "i:i");
            wo_byte(&code, 0x1a);
        }
    }
    wo_byte(&code, 0x0b);
    if (add_synth_function(s1, "__wasm_call_ctors", ":", &code) != base)
        tcc_error_noabort("internal: ctor function misplaced");
    cstr_free(&code);
}

/* a function that traps, for calls to undefined weak functions */
static void synth_trap_function(TCCState *s1, const char *name, const char *sig)
{
    CString code;
    cstr_new(&code);
    wo_uleb(&code, 0);
    wo_byte(&code, 0x00); /* unreachable */
    wo_byte(&code, 0x0b);
    add_synth_function(s1, name, sig, &code);
    cstr_free(&code);
}
