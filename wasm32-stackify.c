/*
 *  Structured control flow for the wasm32 code generator
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

   The intermediate stream of a function (see the overview in
   wasm32-gen.c) is cut into basic blocks and turned into a control flow
   graph.  Two emitters turn the graph into wasm:

   - emit_stackified: the algorithm of Ramsey, "Beyond Relooper" (ICFP
     2022), which is also what LLVM's CFGStackify pass does.  Blocks are
     emitted in dominator tree order.  A loop header gets a 'loop'.  A
     merge node (two or more forward edges in) gets a 'block' that ends
     right before its code; the block is opened around the code of the
     node's immediate dominator, so every branch to the merge node is
     inside it.  A node with a single forward edge in is emitted inline
     where that edge is taken, so the arms of a conditional jump become
     'if ... else ... end'.  This needs a reducible graph.

   - emit_dispatch: the fallback for computed goto, setjmp landing pads
     and irreducible graphs.  Every block is a 'block' in one flat
     ladder inside a 'loop'; backward jumps set L_LBL and go round the
     loop to a 'br_table' at its head.
*/

/* block terminators */
enum {
    T_FALL,     /* falls into the next block */
    T_JMP,      /* jumps to 'target' */
    T_JMPIF,    /* jumps to 'target' if the i32 on the stack is nonzero */
    T_JMPIND,   /* computed goto: L_LBL holds the target block */
    T_END       /* the end of the stream (the epilog's 'return') */
};

typedef struct WasmBB {
    int start, end;     /* stream offsets of the code, terminator excluded */
    int term;
    int target;         /* for T_JMP/T_JMPIF: block, or -1 if never resolved */
    /* analysis, valid if rpo >= 0 */
    int rpo;            /* reverse postorder number, -1 if unreachable */
    int idom;
    int nb_fwd_preds;   /* forward edges in: 2 or more make a merge node */
    int loop_header;    /* target of a back edge */
    int child, sibling; /* dominator tree, children in rpo order */
} WasmBB;

typedef struct WasmCFG {
    unsigned char *code;
    int start, end;
    int frame;          /* frame size, for P_PROLOG and frame accesses */
    WasmBB *bb;
    int nb;
    int *blk;           /* stream offset (relative) -> block, or -1 */
    int *rpo_order;     /* blocks in reverse postorder */
    int nb_reached;
    int sjlj;           /* has a setjmp landing pad */
    int jmpind;         /* has a computed goto */
    int irreducible;
} WasmCFG;

/* ---------------------------------------------------------------- */
/* building the graph */

static int cfg_jmp_target(WasmCFG *c, int p)
{
    unsigned t = read32le(c->code + p + 2);
    if (!(t & JMP_RESOLVED))
        return -1; /* never resolved: the jump is not reachable */
    t &= ~JMP_RESOLVED;
    if ((int)t < c->start || (int)t > c->end)
        tcc_error("internal: jump target out of function");
    return t - c->start;
}

static void cfg_mark_leaders(WasmCFG *c, unsigned char *lead)
{
    int p, tag, t, i;

    lead[0] = 1;
    for (p = c->start; p < c->end; ) {
        if (c->code[p] != 0xff) {
            p++;
            continue;
        }
        tag = c->code[p + 1];
        switch (tag) {
        case P_JMP:
        case P_JMPIF:
        case P_JMPIND:
            if (tag != P_JMPIND && (t = cfg_jmp_target(c, p)) >= 0)
                lead[t] = 1;
            p += pseudo_size(tag);
            lead[p - c->start] = 1;
            if (tag == P_JMPIND)
                c->jmpind = 1;
            break;
        case P_SJLABEL:
            /* longjmps come back through the dispatch loop */
            if ((t = cfg_jmp_target(c, p)) >= 0)
                lead[t] = 1;
            c->sjlj = 1;
            p += pseudo_size(tag);
            break;
        default:
            p += pseudo_size(tag);
            break;
        }
    }
    /* labels are leaders too: their addresses become block indices */
    for (i = 0; i < 2; i++) {
        Sym *ls;
        for (ls = i ? local_label_stack : global_label_stack; ls; ls = ls->prev)
            if (ls->r == LABEL_DEFINED && ls->jind >= c->start && ls->jind < c->end)
                lead[ls->jind - c->start] = 1;
    }
}

static void cfg_build(WasmCFG *c, unsigned char *code, int start, int end, int frame)
{
    int len = end - start, p, tag, i, n;
    unsigned char *lead;
    WasmBB *b;

    memset(c, 0, sizeof *c);
    c->code = code;
    c->start = start;
    c->end = end;
    c->frame = frame;
    lead = tcc_mallocz(len + 1);
    cfg_mark_leaders(c, lead);

    c->blk = tcc_malloc((len + 1) * sizeof(int));
    n = 0;
    for (i = 0; i < len; i++)
        c->blk[i] = lead[i] ? n++ : -1;
    c->blk[len] = -1;
    c->nb = n;
    c->bb = tcc_mallocz(n * sizeof(WasmBB));

    b = NULL;
    for (p = start; p < end; ) {
        if (lead[p - start]) {
            b = &c->bb[c->blk[p - start]];
            b->start = p;
            b->end = end;
            b->term = T_END;
            b->target = -1;
        }
        if (code[p] != 0xff) {
            p++;
            continue;
        }
        tag = code[p + 1];
        if (tag == P_JMP || tag == P_JMPIF || tag == P_JMPIND) {
            b->end = p;
            b->term = tag == P_JMP ? T_JMP : tag == P_JMPIF ? T_JMPIF : T_JMPIND;
            if (tag != P_JMPIND) {
                int t = cfg_jmp_target(c, p);
                b->target = t < 0 ? -1 : c->blk[t];
            }
            p += pseudo_size(tag);
            /* what follows is a leader, so the fallthrough is the next block */
            continue;
        }
        p += pseudo_size(tag);
    }
    /* blocks cut by a leader fall into the next one */
    for (i = 0; i < n; i++) {
        b = &c->bb[i];
        if (b->term == T_END && i + 1 < n) {
            b->end = c->bb[i + 1].start;
            b->term = T_FALL;
        }
    }
    tcc_free(lead);
}

static void cfg_free(WasmCFG *c)
{
    tcc_free(c->bb);
    tcc_free(c->blk);
    tcc_free(c->rpo_order);
}

/* the successors of block i: at most the fallthrough and the target */
static int cfg_succs(WasmCFG *c, int i, int *out)
{
    WasmBB *b = &c->bb[i];
    int n = 0;
    if (b->term == T_FALL || b->term == T_JMPIF)
        out[n++] = i + 1;
    if ((b->term == T_JMP || b->term == T_JMPIF) && b->target >= 0)
        out[n++] = b->target;
    return n;
}

/* ---------------------------------------------------------------- */
/* reverse postorder, dominators, loops */

static void cfg_rpo(WasmCFG *c)
{
    int *stack, *pos, *post, sp, n, i;
    unsigned char *seen;

    for (i = 0; i < c->nb; i++)
        c->bb[i].rpo = -1;
    stack = tcc_malloc(c->nb * sizeof(int));
    pos = tcc_malloc(c->nb * sizeof(int));
    post = tcc_malloc(c->nb * sizeof(int));
    seen = tcc_mallocz(c->nb);
    n = 0;
    sp = 0;
    stack[sp] = 0;
    pos[sp] = 0;
    seen[0] = 1;
    sp = 1;
    while (sp) {
        int succ[2], ns;
        i = stack[sp - 1];
        ns = cfg_succs(c, i, succ);
        if (pos[sp - 1] < ns) {
            int s = succ[pos[sp - 1]++];
            if (!seen[s]) {
                seen[s] = 1;
                stack[sp] = s;
                pos[sp] = 0;
                sp++;
            }
        } else {
            post[n++] = i;
            sp--;
        }
    }
    c->nb_reached = n;
    c->rpo_order = tcc_malloc(n * sizeof(int));
    for (i = 0; i < n; i++) {
        c->rpo_order[i] = post[n - 1 - i];
        c->bb[post[n - 1 - i]].rpo = i;
    }
    tcc_free(stack);
    tcc_free(pos);
    tcc_free(post);
    tcc_free(seen);
}

static int cfg_dom_intersect(WasmCFG *c, int a, int b)
{
    while (a != b) {
        while (c->bb[a].rpo > c->bb[b].rpo)
            a = c->bb[a].idom;
        while (c->bb[b].rpo > c->bb[a].rpo)
            b = c->bb[b].idom;
    }
    return a;
}

/* Cooper, Harvey, Kennedy: "A Simple, Fast Dominance Algorithm" */
static void cfg_dominators(WasmCFG *c)
{
    int *npreds, *pstart, *plist, i, k, changed;

    npreds = tcc_mallocz((c->nb + 1) * sizeof(int));
    for (k = 0; k < c->nb_reached; k++) {
        int succ[2], ns, j;
        i = c->rpo_order[k];
        ns = cfg_succs(c, i, succ);
        for (j = 0; j < ns; j++)
            npreds[succ[j]]++;
    }
    pstart = tcc_malloc((c->nb + 1) * sizeof(int));
    pstart[0] = 0;
    for (i = 0; i < c->nb; i++)
        pstart[i + 1] = pstart[i] + npreds[i];
    plist = tcc_malloc((pstart[c->nb] + 1) * sizeof(int));
    memset(npreds, 0, (c->nb + 1) * sizeof(int));
    for (k = 0; k < c->nb_reached; k++) {
        int succ[2], ns, j;
        i = c->rpo_order[k];
        ns = cfg_succs(c, i, succ);
        for (j = 0; j < ns; j++)
            plist[pstart[succ[j]] + npreds[succ[j]]++] = i;
    }

    for (i = 0; i < c->nb; i++)
        c->bb[i].idom = -1;
    c->bb[0].idom = 0;
    do {
        changed = 0;
        for (k = 1; k < c->nb_reached; k++) {
            int idom = -1, j;
            i = c->rpo_order[k];
            for (j = pstart[i]; j < pstart[i + 1]; j++) {
                int p = plist[j];
                if (c->bb[p].idom < 0)
                    continue;
                idom = idom < 0 ? p : cfg_dom_intersect(c, p, idom);
            }
            if (c->bb[i].idom != idom) {
                c->bb[i].idom = idom;
                changed = 1;
            }
        }
    } while (changed);

    tcc_free(npreds);
    tcc_free(pstart);
    tcc_free(plist);
}

static int cfg_dominates(WasmCFG *c, int a, int b)
{
    while (c->bb[b].rpo > c->bb[a].rpo)
        b = c->bb[b].idom;
    return a == b;
}

/* classify edges and build the dominator tree */
static void cfg_analyze(WasmCFG *c)
{
    int i, k;

    cfg_rpo(c);
    cfg_dominators(c);
    for (k = 0; k < c->nb_reached; k++) {
        int succ[2], ns, j;
        i = c->rpo_order[k];
        ns = cfg_succs(c, i, succ);
        for (j = 0; j < ns; j++) {
            int s = succ[j];
            if (c->bb[s].rpo > c->bb[i].rpo)
                c->bb[s].nb_fwd_preds++;
            else if (cfg_dominates(c, s, i))
                c->bb[s].loop_header = 1;
            else
                c->irreducible = 1;
        }
    }
    for (i = 0; i < c->nb; i++)
        c->bb[i].child = c->bb[i].sibling = -1;
    /* prepending in reverse order leaves each child list in rpo order */
    for (k = c->nb_reached - 1; k > 0; k--) {
        WasmBB *d;
        i = c->rpo_order[k];
        d = &c->bb[c->bb[i].idom];
        c->bb[i].sibling = d->child;
        d->child = i;
    }
}

/* ---------------------------------------------------------------- */
/* the stackifying emitter */

enum { C_BLOCK, C_LOOP, C_IF };            /* open constructs */
enum { K_TREE, K_CODE, K_END, K_ELSE };    /* pending work */

typedef struct Stackifier {
    WasmCFG *c;
    int *ctx_kind, *ctx_bb, nctx;           /* open constructs, innermost last */
    int *work_kind, *work_bb, nwork;        /* pending work, next last */
} Stackifier;

static void st_push_work(Stackifier *st, int kind, int bb)
{
    st->work_kind[st->nwork] = kind;
    st->work_bb[st->nwork] = bb;
    st->nwork++;
}

static void st_open(Stackifier *st, int kind, int bb)
{
    ob(kind == C_LOOP ? W_LOOP : kind == C_BLOCK ? W_BLOCK : W_IF);
    ob(W_BLOCKTYPE_VOID);
    st->ctx_kind[st->nctx] = kind;
    st->ctx_bb[st->nctx] = bb;
    st->nctx++;
}

static void st_close(Stackifier *st, int kind)
{
    if (st->nctx == 0 || st->ctx_kind[st->nctx - 1] != kind)
        tcc_error("internal: wasm construct nesting");
    st->nctx--;
    ob(W_END);
}

/* the branch depth of the construct that a jump to block t leaves or repeats */
static int st_depth(Stackifier *st, int t)
{
    int i;
    for (i = st->nctx - 1; i >= 0; i--)
        if (st->ctx_bb[i] == t && st->ctx_kind[i] != C_IF)
            return st->nctx - 1 - i;
    tcc_error("internal: no wasm label for jump target");
    return 0;
}

/* Block t can be emitted where the edge from 'from' is taken if that is
   its only forward edge in.  Then 'from' is its immediate dominator and
   this is the only place that needs to reach it. */
static int st_inline(Stackifier *st, int from, int t)
{
    WasmBB *bb = st->c->bb;
    return t >= 0 && bb[t].nb_fwd_preds < 2 && bb[t].rpo > bb[from].rpo;
}

static void st_br(Stackifier *st, int t)
{
    if (t < 0) {
        ob(W_UNREACHABLE);
    } else {
        ob(W_BR); ouleb(st_depth(st, t));
    }
}

static void st_br_if(Stackifier *st, int t)
{
    if (t < 0) {
        ob(W_IF); ob(W_BLOCKTYPE_VOID);
        ob(W_UNREACHABLE);
        ob(W_END);
    } else {
        ob(W_BR_IF); ouleb(st_depth(st, t));
    }
}

/* control leaves block 'from' for block t */
static void st_goto(Stackifier *st, int from, int t)
{
    if (st_inline(st, from, t))
        st_push_work(st, K_TREE, t);
    else
        st_br(st, t);
}

static void st_tree(Stackifier *st, int x)
{
    WasmBB *bb = st->c->bb, *b = &bb[x];
    int merge[64], *m = merge, nm = 0, i;

    if (b->loop_header) {
        st_open(st, C_LOOP, x);
        st_push_work(st, K_END, C_LOOP);
    }
    /* the merge nodes among the children: blocks that end before
       each one's code, the one first in rpo order innermost */
    for (i = b->child; i >= 0; i = bb[i].sibling)
        nm += bb[i].nb_fwd_preds >= 2;
    if (nm > 64)
        m = tcc_malloc(nm * sizeof(int));
    nm = 0;
    for (i = b->child; i >= 0; i = bb[i].sibling)
        if (bb[i].nb_fwd_preds >= 2)
            m[nm++] = i;
    for (i = nm - 1; i >= 0; i--)
        st_open(st, C_BLOCK, m[i]);
    for (i = nm - 1; i >= 0; i--) {
        st_push_work(st, K_TREE, m[i]);
        st_push_work(st, K_END, C_BLOCK);
    }
    st_push_work(st, K_CODE, x);
    if (m != merge)
        tcc_free(m);
}

static void st_code(Stackifier *st, int x)
{
    WasmBB *b = &st->c->bb[x];
    int t = b->target, f = x + 1;

    emit_range(st->c, x, -1);
    switch (b->term) {
    case T_JMP:
        st_goto(st, x, t);
        break;
    case T_FALL:
        st_goto(st, x, f);
        break;
    case T_END:
        ob(W_UNREACHABLE);
        break;
    case T_JMPIF:
        if (!st_inline(st, x, t)) {
            st_br_if(st, t);
            st_goto(st, x, f);
        } else if (!st_inline(st, x, f)) {
            ob(W_I32_EQZ);
            st_br_if(st, f);
            st_push_work(st, K_TREE, t);
        } else {
            st_open(st, C_IF, -1);
            st_push_work(st, K_END, C_IF);
            st_push_work(st, K_TREE, f);
            st_push_work(st, K_ELSE, 0);
            st_push_work(st, K_TREE, t);
        }
        break;
    default:
        tcc_error("internal: computed goto in stackified function");
    }
}

static void emit_stackified(WasmCFG *c)
{
    Stackifier st;
    int n = c->nb;

    st.c = c;
    st.nctx = st.nwork = 0;
    /* each block opens at most a loop, a block and an if */
    st.ctx_kind = tcc_malloc((3 * n + 1) * sizeof(int));
    st.ctx_bb = tcc_malloc((3 * n + 1) * sizeof(int));
    /* each block contributes at most a tree, its code, an else and
       three ends */
    st.work_kind = tcc_malloc((6 * n + 1) * sizeof(int));
    st.work_bb = tcc_malloc((6 * n + 1) * sizeof(int));

    st_push_work(&st, K_TREE, 0);
    while (st.nwork) {
        int kind, bb;
        st.nwork--;
        kind = st.work_kind[st.nwork];
        bb = st.work_bb[st.nwork];
        switch (kind) {
        case K_TREE: st_tree(&st, bb); break;
        case K_CODE: st_code(&st, bb); break;
        case K_END:  st_close(&st, bb); break;
        case K_ELSE: ob(W_ELSE); break;
        }
    }
    if (st.nctx)
        tcc_error("internal: wasm construct left open");
    tcc_free(st.ctx_kind);
    tcc_free(st.ctx_bb);
    tcc_free(st.work_kind);
    tcc_free(st.work_bb);
}

/* ---------------------------------------------------------------- */
/* the dispatching emitter */

static void emit_dispatch(WasmCFG *c)
{
    int n = c->nb, i, j, has_back, loopd;

    has_back = c->sjlj || c->jmpind;
    for (i = 0; i < n && !has_back; i++) {
        WasmBB *b = &c->bb[i];
        if ((b->term == T_JMP || b->term == T_JMPIF) && b->target >= 0 && b->target <= i)
            has_back = 1;
    }
    if (has_back) {
        ob(W_LOOP); ob(W_BLOCKTYPE_VOID);
    }
    /* the landing block of longjmp: it encloses all blocks, so the
       loop is one level further out */
    loopd = c->sjlj;
    if (c->sjlj) {
        ob(W_BLOCK); ob(W_BLOCKTYPE_I32);
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
    for (i = 0; i < n; i++) {
        WasmBB *b = &c->bb[i];
        ob(W_END); /* the block whose end precedes this one's code */
        emit_range(c, i, c->sjlj ? n - i - 1 : -1);
        j = b->target;
        switch (b->term) {
        case T_JMP:
            if (j < 0) {
                ob(W_UNREACHABLE);
            } else if (j > i) {
                if (j != i + 1) {
                    ob(W_BR); ouleb(j - i - 1);
                }
            } else {
                ob(W_I32_CONST); osleb(j);
                ob(W_LOCAL_SET); ouleb(L_LBL);
                ob(W_BR); ouleb(n - i - 1 + loopd);
            }
            break;
        case T_JMPIF:
            if (j < 0) {
                ob(W_IF); ob(W_BLOCKTYPE_VOID);
                ob(W_UNREACHABLE);
                ob(W_END);
            } else if (j > i) {
                ob(W_BR_IF); ouleb(j - i - 1);
            } else {
                ob(W_IF); ob(W_BLOCKTYPE_VOID);
                ob(W_I32_CONST); osleb(j);
                ob(W_LOCAL_SET); ouleb(L_LBL);
                ob(W_BR); ouleb(n - i + loopd);
                ob(W_END);
            }
            break;
        case T_JMPIND:
            ob(W_BR); ouleb(n - i - 1 + loopd);
            break;
        }
    }
    if (c->sjlj)
        emit_landing();
    if (has_back)
        ob(W_END);
}
