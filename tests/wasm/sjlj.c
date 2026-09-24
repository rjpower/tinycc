/* setjmp/longjmp on wasm32: exception based, see wasm32-gen.c */
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <alloca.h>

static jmp_buf outer, inner;
static int depth;

static void deep(int n)
{
    char pad[64];
    memset(pad, n, sizeof pad);
    depth = n;
    if (n == 5)
        longjmp(outer, pad[3] * 10);
    deep(n + 1);
}

/* longjmp through a function that itself uses setjmp for other jumps */
static int middle(int n)
{
    if (setjmp(inner) == 0) {
        printf("middle: first\n");
        if (n)
            deep(0);
        longjmp(inner, 7);
    }
    printf("middle: after inner\n");
    return 1;
}

struct big { int a[6]; };
static struct big mk(int v) { struct big b; int i; for (i = 0; i < 6; i++) b.a[i] = v + i; return b; }
static double half(double d) { return d / 2; }
static long double quad(long double x) { return x * 3; }
static int (*fp)(int) = 0;
static int twice(int x) { return 2 * x; }

int main(void)
{
    int r, i, count = 0;
    volatile int vol = 1;
    struct big b;
    double d;
    jmp_buf loop;

    fp = twice;
    /* jump over several frames, value carried */
    r = setjmp(outer);
    printf("outer: %d depth=%d\n", r, depth);
    if (r == 0)
        middle(1);

    /* zero is turned into 1 */
    if ((r = setjmp(outer)) == 0)
        longjmp(outer, 0);
    printf("zero -> %d\n", r);

    /* several setjmps in one function, in a loop */
    while (count < 3) {
        if (setjmp(loop) == 0) {
            count++;
            longjmp(loop, 1);
        }
    }
    printf("count %d\n", count);

    /* other kinds of calls inside a setjmp function */
    b = mk(10);
    d = half(5.0);
    printf("%d %d %.1f %d %.1Lf\n", b.a[0], b.a[5], d, fp(21), quad(2.5L));

    /* the stack pointer is restored: deep frames are gone afterwards */
    {
        char *before = alloca(16), *after;
        if (setjmp(outer) == 0)
            deep(0);
        after = alloca(16);
        printf("alloca %s\n", after < before && before - after <= 64 ? "ok" : "bad");
    }

    /* inner setjmp not confused with the outer one */
    r = middle(0);
    printf("middle returned %d\n", r);

    /* longjmp within the same function */
    i = 0;
    if (setjmp(outer) == 0 && vol) {
        i = 42;
        longjmp(outer, 2);
    }
    printf("same function: i=%d\n", i);
    return 0;
}
