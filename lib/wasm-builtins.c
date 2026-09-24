/* compiler-rt style builtins needed by wasi-libc, for tcc's wasm32 target.
   128-bit integers follow the wasm C ABI: two i64 per value (low, high),
   results returned through a pointer. */

typedef unsigned long long u64;
typedef long long s64;
typedef unsigned int u32;

/* 128x128 -> 256 bit unsigned multiply using 32-bit limbs */
static void mul128(const u32 *a, const u32 *b, u32 *r)
{
    int i, j;
    for (i = 0; i < 8; i++)
        r[i] = 0;
    for (i = 0; i < 4; i++) {
        u64 carry = 0;
        for (j = 0; j < 4; j++) {
            u64 t = (u64)a[i] * b[j] + r[i + j] + carry;
            r[i + j] = (u32)t;
            carry = t >> 32;
        }
        r[i + 4] = (u32)carry;
    }
}

static void neg128(u32 *x)
{
    u64 carry = 1;
    int i;
    for (i = 0; i < 4; i++) {
        u64 t = (u64)(u32)~x[i] + carry;
        x[i] = (u32)t;
        carry = t >> 32;
    }
}

/* signed 128-bit multiply with overflow detection */
void __muloti4(u64 *res, u64 alo, s64 ahi, u64 blo, s64 bhi, int *overflow)
{
    u32 a[4], b[4], r[8];
    int sa = ahi < 0, sb = bhi < 0, i, ovf = 0;

    a[0] = (u32)alo; a[1] = (u32)(alo >> 32); a[2] = (u32)ahi; a[3] = (u32)((u64)ahi >> 32);
    b[0] = (u32)blo; b[1] = (u32)(blo >> 32); b[2] = (u32)bhi; b[3] = (u32)((u64)bhi >> 32);
    if (sa)
        neg128(a);
    if (sb)
        neg128(b);
    mul128(a, b, r);
    for (i = 4; i < 8; i++)
        if (r[i])
            ovf = 1;
    if (sa != sb) {
        /* negative result: magnitude may be up to 2^127 */
        if (r[3] & 0x80000000u) {
            if ((r[3] & 0x7fffffffu) || r[2] || r[1] || r[0])
                ovf = 1;
        }
        neg128(r);
    } else if (r[3] & 0x80000000u) {
        ovf = 1;
    }
    res[0] = (u64)r[0] | (u64)r[1] << 32;
    res[1] = (u64)r[2] | (u64)r[3] << 32;
    *overflow = ovf;
}

/* unsigned 128-bit multiply */
void __multi3(u64 *res, u64 alo, s64 ahi, u64 blo, s64 bhi)
{
    u32 a[4], b[4], r[8];
    a[0] = (u32)alo; a[1] = (u32)(alo >> 32); a[2] = (u32)ahi; a[3] = (u32)((u64)ahi >> 32);
    b[0] = (u32)blo; b[1] = (u32)(blo >> 32); b[2] = (u32)bhi; b[3] = (u32)((u64)bhi >> 32);
    mul128(a, b, r);
    res[0] = (u64)r[0] | (u64)r[1] << 32;
    res[1] = (u64)r[2] | (u64)r[3] << 32;
}
