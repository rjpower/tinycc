/* undefined weak functions: address compares as null, calls are never made */
#include <stdio.h>

__attribute__((weak)) int weak_a(int x);
__attribute__((weak)) int weak_b(int x);
__attribute__((weak)) int weak_c(int x);

static int probe(const char *name, int (*fn)(int))
{
    if (fn) {
        printf("%s: defined -> %d\n", name, fn(1));
        return 1;
    }
    printf("%s: null\n", name);
    return 0;
}

int main(void)
{
    /* direct calls guarded by address tests; each symbol gets a trap stub
       and a table slot, which must not make the address non-null */
    if (weak_a) weak_a(1);
    if (weak_b) weak_b(2);
    if (weak_c) weak_c(3);
    probe("weak_a", weak_a);
    probe("weak_b", weak_b);
    probe("weak_c", weak_c);
    return 0;
}
