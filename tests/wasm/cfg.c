/* control flow shapes for the structured rewrite */
#include <stdio.h>

/* Duff's device: a goto into a loop body, irreducible */
static void duff(char *to, const char *from, int count)
{
    int n = (count + 7) / 8;
    switch (count % 8) {
    case 0: do { *to++ = *from++;
    case 7:      *to++ = *from++;
    case 6:      *to++ = *from++;
    case 5:      *to++ = *from++;
    case 4:      *to++ = *from++;
    case 3:      *to++ = *from++;
    case 2:      *to++ = *from++;
    case 1:      *to++ = *from++;
            } while (--n > 0);
    }
}

/* goto into the middle of a loop */
static int goto_into_loop(int n)
{
    int i = 0, s = 0;
    goto middle;
    for (i = 0; i < n; i++) {
        s += i;
middle:
        s += 100;
    }
    return s;
}

/* a loop header reached by falling through from a later block */
static int fall_back_edge(int n)
{
    int s = 0;
    goto b;
a:
    s += n--;
b:
    if (n > 0)
        goto a;
    return s;
}

/* nested loops with break and continue at both levels */
static int nested(int n)
{
    int i, j, s = 0;
    for (i = 0; i < n; i++) {
        if (i == 3)
            continue;
        for (j = 0; j < n; j++) {
            if (j == i)
                continue;
            if (j > 5)
                break;
            s += i * j;
        }
        if (s > 200)
            break;
    }
    return s;
}

/* switch inside a loop with fallthrough, continue and break */
static int switch_loop(const char *s)
{
    int v = 0;
    while (*s) {
        switch (*s++) {
        case '+': v++; break;
        case '-': v--; break;
        case 'x': v *= 2; /* fallthrough */
        case 'y': v++; break;
        case ' ': continue;
        default: return -v;
        }
        v += 10;
    }
    return v;
}

/* computed goto: the dispatch fallback */
static int computed(int n)
{
    static const void *ops[] = { &&add, &&dbl, &&done };
    int v = 1, i = 0;
    goto *ops[i];
add:
    v += 3;
    i = 1;
    goto *ops[i];
dbl:
    v *= 2;
    i = 2;
    goto *ops[i];
done:
    return v + n;
}

/* do-while with continue, and a while (0) trick */
static int dowhile(int n)
{
    int s = 0;
    do {
        n--;
        if (n & 1)
            continue;
        s += n;
    } while (n > 0);
    do {
        s += 1000;
        if (n == 0)
            break;
        s += 1;
    } while (0);
    return s;
}

/* many sequential ifs: the join of each is a merge node */
static int sequence(int x)
{
    int s = 0;
    if (x & 1) s += 1;
    if (x & 2) s += 2; else s -= 1;
    if (x & 4) { if (x & 8) s += 4; else s += 8; }
    if (x & 16) s += 16;
    while (s > 40) s -= 7;
    if (x & 32) return s * 2;
    return s;
}

/* a loop whose only exit is a return from the middle */
static int find(const int *a, int n, int v)
{
    int i = 0;
    for (;;) {
        if (i == n)
            return -1;
        if (a[i] == v)
            return i;
        i++;
    }
}

/* short circuit and conditional expressions inside loop conditions */
static int shortcircuit(int n)
{
    int i = 0, c = 0;
    while ((i < n && i % 3) || i == 0) {
        c += (i > 5) ? i : -i;
        i++;
    }
    return c;
}

/* dead code after return and an unreachable label */
static int dead(int x)
{
    return x + 1;
    x = 5;
never:
    x++;
    goto never;
}

int main(void)
{
    char buf[20] = "";
    const int a[] = { 5, 7, 9, 11 };
    int i;
    duff(buf, "hello, world!", 14);
    printf("duff: %s\n", buf);
    printf("goto_into_loop: %d %d\n", goto_into_loop(0), goto_into_loop(4));
    printf("fall_back_edge: %d %d\n", fall_back_edge(0), fall_back_edge(5));
    printf("nested: %d %d\n", nested(4), nested(9));
    printf("switch_loop: %d %d %d\n", switch_loop("++ - xy"), switch_loop("x?+"), switch_loop(""));
    printf("computed: %d\n", computed(10));
    printf("dowhile: %d %d\n", dowhile(6), dowhile(0));
    for (i = 0; i < 64; i += 13)
        printf("sequence(%d): %d\n", i, sequence(i));
    printf("find: %d %d\n", find(a, 4, 9), find(a, 4, 4));
    printf("shortcircuit: %d\n", shortcircuit(10));
    printf("dead: %d\n", dead(1));
    return 0;
}
