/* frame slots promoted to wasm locals: the aliasing cases that must
   keep a local in memory */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <setjmp.h>

/* an interior pointer into an array, used to reach other elements */
static int interior(void)
{
    int a[8], i, *mid;
    for (i = 0; i < 8; i++)
        a[i] = i * 10;
    mid = &a[4];
    mid[-4] = 99;       /* a[0] through the pointer */
    a[7] = mid[1] + 1;  /* a[5] through the pointer, a[7] directly */
    return a[0] + a[7] + *(mid - 2);
}

/* a struct: members direct, whole struct by address */
struct pt { char tag; int x; double y; };
static void fill(struct pt *p) { p->x = 5; p->y = 2.5; p->tag = 'z'; }
static double members(void)
{
    struct pt p, q;
    p.x = 1;
    p.y = 1.5;
    p.tag = 'a';
    fill(&p);
    q = p;
    q.x += 10;
    return p.x + p.y + q.x + q.y + (p.tag == q.tag);
}

/* type punning through a union and through a cast */
static unsigned punning(float f)
{
    union { float f; unsigned u; } u;
    double d = 1.0;
    unsigned long long bits;
    u.f = f;
    memcpy(&bits, &d, sizeof bits);
    return u.u + (unsigned)(bits >> 52);
}

/* address of a scalar escaping to a callee */
static void bump(int *p, char *c, short *s) { (*p)++; (*c)++; (*s)--; }
static int escape(void)
{
    int n = 1;
    char c = 'a';
    short s = 3;
    bump(&n, &c, &s);
    bump(&n, &c, &s);
    return n * 1000 + c * 10 + s;
}

/* small integer widths: sign and zero extension from promoted slots */
static long long widths(signed char sc, unsigned char uc, short sh, unsigned short ush, unsigned ui)
{
    long long r = 0;
    sc -= 10; uc -= 10; sh -= 10; ush -= 10; ui -= 10;
    r += sc; r += uc; r += sh; r += ush; r += (long long)ui;
    return r + (sc < 0) + (uc > 200) + (sh < 0) + (ush > 60000) + (ui > 4000000000u);
}

/* float and double locals and parameters */
static double floats(float f, double d)
{
    float g = f * 2;
    double e = d / 2;
    int i;
    for (i = 0; i < 3; i++) { g += 0.25f; e -= 0.125; }
    return g + e + f + d;
}

/* varargs: the buffer lives in the caller's frame */
static int vsum(int n, ...)
{
    va_list ap;
    int s = 0;
    va_start(ap, n);
    while (n--)
        s += va_arg(ap, int);
    va_end(ap);
    return s;
}
static double vmix(int n, ...)
{
    va_list ap;
    double s = 0;
    va_start(ap, n);
    while (n--)
        s += va_arg(ap, double);
    va_end(ap);
    return s;
}

/* struct passed and returned by value */
static struct pt mk(int x) { struct pt p; p.tag = 't'; p.x = x; p.y = x / 4.0; return p; }
static int take(struct pt p) { return p.x + (int)p.y; }
static int byvalue(void)
{
    struct pt p = mk(8);
    return take(p) + take(mk(20)) + mk(3).x;
}

/* the same frame offsets reused by sibling scopes with different types */
static double scopes(int k)
{
    double r = 0;
    { int a = k, b = k * 2; r += a + b; }
    { double c = k / 2.0; r += c; }
    { char s[4]; s[0] = 'x'; s[1] = 0; r += s[0]; }
    { long long l = k; l <<= 33; r += (double)(l >> 33); }
    return r;
}

/* compound literals, whose objects have no symbol */
static int literal(void)
{
    int *p = (int[]){ 1, 2, 3 };
    int q = *(int *)&(struct { int v; }){ 7 };
    return p[0] + p[2] + q;
}

/* a VLA and alloca beside promoted scalars */
static int vla(int n)
{
    int a[n], i, s = 0;
    for (i = 0; i < n; i++)
        a[i] = i;
    for (i = 0; i < n; i++)
        s += a[i];
    return s + n;
}

/* locals kept across longjmp */
static jmp_buf jb;
static void thrower(int v) { longjmp(jb, v); }
static int sjlj(void)
{
    volatile int count = 0;
    int last = -1;
    int r = setjmp(jb);
    if (r)
        last = r;
    count = count + 1;
    if (count < 4)
        thrower(count * 7);
    return count * 100 + last;
}

int main(void)
{
    printf("interior: %d\n", interior());
    printf("members: %g\n", members());
    printf("punning: %u\n", punning(1.0f));
    printf("escape: %d\n", escape());
    printf("widths: %lld\n", widths(-5, 5, -5, 5, 5));
    printf("floats: %g\n", floats(1.5f, 3.0));
    printf("varargs: %d %g\n", vsum(4, 1, 2, 3, 4), vmix(3, 0.5, 1.5, 2.25));
    printf("byvalue: %d\n", byvalue());
    printf("scopes: %g\n", scopes(6));
    printf("literal: %d\n", literal());
    printf("vla: %d\n", vla(5));
    printf("sjlj: %d\n", sjlj());
    return 0;
}
