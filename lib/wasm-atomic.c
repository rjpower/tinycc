/* atomic builtins for tcc's wasm32 target (single threaded).
   tcc calls the __atomic_*_N helpers as unprototyped functions, so they
   all return a value; the others are declared in stdatomic.h */

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;
typedef _Bool bool;

#define ATOMIC_OPS(T, N) \
T __atomic_load_##N(const volatile void *p, int mo) { return *(volatile T *)p; } \
int __atomic_store_##N(volatile void *p, T v, int mo) { *(volatile T *)p = v; return 0; } \
T __atomic_exchange_##N(volatile void *p, T v, int mo) { T o = *(volatile T *)p; *(volatile T *)p = v; return o; } \
bool __atomic_compare_exchange_##N(volatile void *p, void *expected, T desired, bool weak, int smo, int fmo) \
{ T o = *(volatile T *)p; if (o == *(T *)expected) { *(volatile T *)p = desired; return 1; } *(T *)expected = o; return 0; } \
T __atomic_fetch_add_##N(volatile void *p, T v, int mo) { T o = *(volatile T *)p; *(volatile T *)p = o + v; return o; } \
T __atomic_fetch_sub_##N(volatile void *p, T v, int mo) { T o = *(volatile T *)p; *(volatile T *)p = o - v; return o; } \
T __atomic_fetch_and_##N(volatile void *p, T v, int mo) { T o = *(volatile T *)p; *(volatile T *)p = o & v; return o; } \
T __atomic_fetch_or_##N(volatile void *p, T v, int mo) { T o = *(volatile T *)p; *(volatile T *)p = o | v; return o; } \
T __atomic_fetch_xor_##N(volatile void *p, T v, int mo) { T o = *(volatile T *)p; *(volatile T *)p = o ^ v; return o; } \
T __atomic_fetch_nand_##N(volatile void *p, T v, int mo) { T o = *(volatile T *)p; *(volatile T *)p = ~(o & v); return o; } \
T __atomic_add_fetch_##N(volatile void *p, T v, int mo) { return *(volatile T *)p = *(volatile T *)p + v; } \
T __atomic_sub_fetch_##N(volatile void *p, T v, int mo) { return *(volatile T *)p = *(volatile T *)p - v; } \
T __atomic_and_fetch_##N(volatile void *p, T v, int mo) { return *(volatile T *)p = *(volatile T *)p & v; } \
T __atomic_or_fetch_##N(volatile void *p, T v, int mo) { return *(volatile T *)p = *(volatile T *)p | v; } \
T __atomic_xor_fetch_##N(volatile void *p, T v, int mo) { return *(volatile T *)p = *(volatile T *)p ^ v; } \
T __atomic_nand_fetch_##N(volatile void *p, T v, int mo) { return *(volatile T *)p = ~(*(volatile T *)p & v); }

ATOMIC_OPS(u8, 1)
ATOMIC_OPS(u16, 2)
ATOMIC_OPS(u32, 4)
ATOMIC_OPS(u64, 8)

bool __atomic_is_lock_free(unsigned long size, const volatile void *ptr)
{
    return size == 1 || size == 2 || size == 4 || size == 8;
}

bool atomic_flag_test_and_set(volatile void *p)
{
    bool o = *(volatile bool *)p;
    *(volatile bool *)p = 1;
    return o;
}

bool atomic_flag_test_and_set_explicit(volatile void *p, int mo)
{
    return atomic_flag_test_and_set(p);
}

void atomic_flag_clear(volatile void *p)
{
    *(volatile bool *)p = 0;
}

void atomic_flag_clear_explicit(volatile void *p, int mo)
{
    *(volatile bool *)p = 0;
}

void atomic_thread_fence(int mo) {}
void atomic_signal_fence(int mo) {}
