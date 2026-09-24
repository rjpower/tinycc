/* long double support for tcc's wasm32 target.

   'long double' has the wasm C ABI layout: IEEE binary128, 16 bytes,
   passed as two i64 (low, high) and returned through a pointer. tcc
   itself computes long double values in double precision and converts
   when loading/storing; but code from wasi-libc (printf, strtod, libm)
   does real quad arithmetic through the compiler-rt style builtins
   below, which are implemented here as IEEE-754 conforming soft-float
   (round to nearest even). */

typedef unsigned long long u64;
typedef long long s64;
typedef unsigned int u32;
typedef union { double d; u64 u; } dbits;
typedef union { float f; u32 u; } fbits;

/* 128-bit unsigned integer as two limbs */
typedef struct { u64 lo, hi; } u128;

/* unpacked quad: value = (-1)^sign * sig * 2^(exp - 115) where the
   leading bit of a normalized 'sig' is bit 115 (113 significant bits
   plus 3 extra bits for rounding) */
typedef struct {
    int sign;
    int exp;      /* unbiased exponent of bit 115 */
    u128 sig;
    int cls;      /* 0 finite, 1 zero, 2 infinity, 3 nan */
} tf;

#define TF_ZERO 1
#define TF_INF 2
#define TF_NAN 3
#define EXP_BIAS 16383
#define EXP_MIN (-16382)
#define EXP_MAX 16383

/* ---------------------------------------------------------------- */
/* 128-bit helpers */

static u128 u128_shl(u128 a, int n)
{
    u128 r;
    if (n == 0) return a;
    if (n >= 128) { r.lo = r.hi = 0; return r; }
    if (n >= 64) { r.hi = a.lo << (n - 64); r.lo = 0; return r; }
    r.hi = a.hi << n | a.lo >> (64 - n);
    r.lo = a.lo << n;
    return r;
}

/* shift right, or-ing the shifted out bits into bit 0 (sticky) */
static u128 u128_shr_sticky(u128 a, int n)
{
    u128 r;
    int sticky;
    if (n == 0) return a;
    if (n >= 128) {
        r.lo = (a.lo | a.hi) != 0;
        r.hi = 0;
        return r;
    }
    if (n >= 64) {
        sticky = (a.lo | (a.hi & ((1ULL << (n - 64)) - 1))) != 0;
        r.lo = a.hi >> (n - 64) | sticky;
        r.hi = 0;
        return r;
    }
    sticky = (a.lo & ((1ULL << n) - 1)) != 0;
    r.lo = a.lo >> n | a.hi << (64 - n) | sticky;
    r.hi = a.hi >> n;
    return r;
}

static u128 u128_shr(u128 a, int n)
{
    u128 r;
    if (n == 0) return a;
    if (n >= 128) { r.lo = r.hi = 0; return r; }
    if (n >= 64) { r.lo = a.hi >> (n - 64); r.hi = 0; return r; }
    r.lo = a.lo >> n | a.hi << (64 - n);
    r.hi = a.hi >> n;
    return r;
}

static u128 u128_add(u128 a, u128 b)
{
    u128 r;
    r.lo = a.lo + b.lo;
    r.hi = a.hi + b.hi + (r.lo < a.lo);
    return r;
}

static u128 u128_sub(u128 a, u128 b)
{
    u128 r;
    r.lo = a.lo - b.lo;
    r.hi = a.hi - b.hi - (a.lo < b.lo);
    return r;
}

static int u128_cmp(u128 a, u128 b)
{
    if (a.hi != b.hi) return a.hi < b.hi ? -1 : 1;
    if (a.lo != b.lo) return a.lo < b.lo ? -1 : 1;
    return 0;
}

static int u128_is_zero(u128 a)
{
    return (a.lo | a.hi) == 0;
}

/* index of the highest set bit, -1 if zero */
static int u128_msb(u128 a)
{
    u64 v;
    int base, i;
    if (a.hi) { v = a.hi; base = 64; } else if (a.lo) { v = a.lo; base = 0; } else return -1;
    for (i = 63; i >= 0; i--)
        if (v >> i & 1)
            return base + i;
    return -1;
}

/* 64x64 -> 128 */
static u128 mul64(u64 a, u64 b)
{
    u64 a0 = (u32)a, a1 = a >> 32, b0 = (u32)b, b1 = b >> 32;
    u64 p00 = a0 * b0, p01 = a0 * b1, p10 = a1 * b0, p11 = a1 * b1;
    u64 mid = (p00 >> 32) + (u32)p01 + (u32)p10;
    u128 r;
    r.lo = (mid << 32) | (u32)p00;
    r.hi = p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32);
    return r;
}

/* ---------------------------------------------------------------- */
/* pack / unpack */

static tf tf_unpack(u64 lo, u64 hi)
{
    tf x;
    int e = (hi >> 48) & 0x7fff;
    u128 m;
    m.lo = lo;
    m.hi = hi & 0xffffffffffffULL;
    x.sign = hi >> 63;
    x.cls = 0;
    if (e == 0x7fff) {
        x.cls = u128_is_zero(m) ? TF_INF : TF_NAN;
        x.exp = 0;
        x.sig = m;
    } else if (e == 0) {
        if (u128_is_zero(m)) {
            x.cls = TF_ZERO;
            x.exp = 0;
            x.sig = m;
        } else {
            /* subnormal: normalize */
            int msb = u128_msb(m);
            x.sig = u128_shl(m, 115 - msb);
            x.exp = EXP_MIN - (112 - msb);
        }
    } else {
        m.hi |= 1ULL << 48;
        x.sig = u128_shl(m, 3);
        x.exp = e - EXP_BIAS;
    }
    return x;
}

static void tf_store(u64 *r, u64 lo, u64 hi)
{
    r[0] = lo;
    r[1] = hi;
}

static void tf_pack_special(u64 *r, int sign, int cls)
{
    u64 hi = (u64)sign << 63;
    if (cls == TF_INF)
        hi |= 0x7fffULL << 48;
    else if (cls == TF_NAN)
        hi |= 0x7fffULL << 48 | 1ULL << 47;
    tf_store(r, 0, hi);
}

/* round 'sig' (leading bit anywhere, exponent of bit 115 is 'exp')
   to 113 bits and pack */
static void tf_pack(u64 *r, int sign, int exp, u128 sig)
{
    int msb, shift;
    u64 lo, hi;

    if (u128_is_zero(sig)) {
        tf_pack_special(r, sign, TF_ZERO);
        return;
    }
    /* normalize so that the leading bit is bit 115 */
    msb = u128_msb(sig);
    if (msb > 115) {
        sig = u128_shr_sticky(sig, msb - 115);
        exp += msb - 115;
    } else if (msb < 115) {
        sig = u128_shl(sig, 115 - msb);
        exp -= 115 - msb;
    }
    if (exp < EXP_MIN) {
        /* subnormal result: shift right keeping the sticky bit */
        shift = EXP_MIN - exp;
        sig = u128_shr_sticky(sig, shift);
        exp = EXP_MIN;
    }
    /* round to nearest even on the 3 low bits */
    {
        int low = sig.lo & 7;
        u128 one;
        one.lo = 8; one.hi = 0;
        if (low > 4 || (low == 4 && (sig.lo & 8))) {
            sig = u128_add(sig, one);
            if (sig.hi >> 52) { /* carry into bit 116 */
                sig = u128_shr_sticky(sig, 1);
                exp++;
            }
        }
        sig.lo &= ~7ULL;
    }
    if (exp > EXP_MAX) {
        tf_pack_special(r, sign, TF_INF);
        return;
    }
    /* mantissa = bits [114:3]; bit 115 is the implicit bit */
    sig = u128_shr_sticky(sig, 3);
    lo = sig.lo;
    hi = sig.hi & 0xffffffffffffULL;
    if (sig.hi >> 48 & 1)
        hi |= (u64)(exp + EXP_BIAS) << 48; /* normal */
    /* else subnormal: exponent field 0 */
    hi |= (u64)sign << 63;
    tf_store(r, lo, hi);
}

/* ---------------------------------------------------------------- */
/* arithmetic */

static void tf_add_core(u64 *r, tf a, tf b, int sub)
{
    int sign, exp;
    u128 sig;

    if (sub)
        b.sign ^= 1;
    if (a.cls == TF_NAN || b.cls == TF_NAN) {
        tf_pack_special(r, 0, TF_NAN);
        return;
    }
    if (a.cls == TF_INF || b.cls == TF_INF) {
        if (a.cls == TF_INF && b.cls == TF_INF && a.sign != b.sign)
            tf_pack_special(r, 0, TF_NAN);
        else
            tf_pack_special(r, a.cls == TF_INF ? a.sign : b.sign, TF_INF);
        return;
    }
    if (a.cls == TF_ZERO && b.cls == TF_ZERO) {
        tf_pack_special(r, a.sign & b.sign, TF_ZERO);
        return;
    }
    if (a.cls == TF_ZERO) {
        tf_pack(r, b.sign, b.exp, b.sig);
        return;
    }
    if (b.cls == TF_ZERO) {
        tf_pack(r, a.sign, a.exp, a.sig);
        return;
    }
    /* make 'a' the larger exponent */
    if (a.exp < b.exp || (a.exp == b.exp && u128_cmp(a.sig, b.sig) < 0)) {
        tf t = a; a = b; b = t;
    }
    exp = a.exp;
    if (a.sign == b.sign) {
        /* room for a carry: work with the leading bit at 114 */
        a.sig = u128_shr_sticky(a.sig, 1);
        b.sig = u128_shr_sticky(b.sig, 1 + (a.exp - b.exp));
        exp += 1;
        sig = u128_add(a.sig, b.sig);
        sign = a.sign;
    } else {
        /* no carry possible; keeping the leading bit at 115 leaves the
           sticky bit below the round bit after a one bit normalization */
        b.sig = u128_shr_sticky(b.sig, a.exp - b.exp);
        sig = u128_sub(a.sig, b.sig);
        sign = a.sign;
        if (u128_is_zero(sig)) {
            tf_pack_special(r, 0, TF_ZERO);
            return;
        }
    }
    tf_pack(r, sign, exp, sig);
}

void __addtf3(u64 *r, u64 alo, u64 ahi, u64 blo, u64 bhi)
{
    tf_add_core(r, tf_unpack(alo, ahi), tf_unpack(blo, bhi), 0);
}

void __subtf3(u64 *r, u64 alo, u64 ahi, u64 blo, u64 bhi)
{
    tf_add_core(r, tf_unpack(alo, ahi), tf_unpack(blo, bhi), 1);
}

void __negtf2(u64 *r, u64 alo, u64 ahi)
{
    tf_store(r, alo, ahi ^ (1ULL << 63));
}

void __multf3(u64 *r, u64 alo, u64 ahi, u64 blo, u64 bhi)
{
    tf a = tf_unpack(alo, ahi), b = tf_unpack(blo, bhi);
    int sign = a.sign ^ b.sign;
    u128 p0, p1, p2, p3, lo, hi, sig;
    u64 carry;

    if (a.cls == TF_NAN || b.cls == TF_NAN) {
        tf_pack_special(r, 0, TF_NAN);
        return;
    }
    if (a.cls == TF_INF || b.cls == TF_INF) {
        if (a.cls == TF_ZERO || b.cls == TF_ZERO)
            tf_pack_special(r, 0, TF_NAN);
        else
            tf_pack_special(r, sign, TF_INF);
        return;
    }
    if (a.cls == TF_ZERO || b.cls == TF_ZERO) {
        tf_pack_special(r, sign, TF_ZERO);
        return;
    }
    /* significands have their leading bit at 115: use them at bit 112
       (shift right 3, exact) and multiply: 113x113 -> 225/226 bits,
       as four 64-bit limbs p[0..3] */
    {
        u64 p[4], c;
        a.sig = u128_shr(a.sig, 3);
        b.sig = u128_shr(b.sig, 3);
        p0 = mul64(a.sig.lo, b.sig.lo);
        p1 = mul64(a.sig.lo, b.sig.hi);
        p2 = mul64(a.sig.hi, b.sig.lo);
        p3 = mul64(a.sig.hi, b.sig.hi);
        p[0] = p0.lo;
        p[1] = p0.hi;
        c = 0;
        p[1] += p1.lo; c += p[1] < p1.lo;
        p[1] += p2.lo; c += p[1] < p2.lo;
        p[2] = p3.lo + c; c = p[2] < c;
        p[2] += p1.hi; c += p[2] < p1.hi;
        p[2] += p2.hi; c += p[2] < p2.hi;
        p[3] = p3.hi + c;
        /* the leading bit is at 224 or 225: shift right by 109 so that
           it lands on 115 or 116, folding the rest into a sticky bit */
        sig.lo = p[1] >> 45 | p[2] << 19;
        sig.hi = p[2] >> 45 | p[3] << 19;
        if (p[0] || (p[1] & ((1ULL << 45) - 1)))
            sig.lo |= 1;
        lo = hi = sig; (void)lo; (void)hi; (void)carry;
    }
    /* bit 224 of the product has exponent a.exp + b.exp, so does bit 115 of sig */
    tf_pack(r, sign, a.exp + b.exp, sig);
}

void __divtf3(u64 *r, u64 alo, u64 ahi, u64 blo, u64 bhi)
{
    tf a = tf_unpack(alo, ahi), b = tf_unpack(blo, bhi);
    int sign = a.sign ^ b.sign, i, exp;
    u128 rem, q, bit;

    if (a.cls == TF_NAN || b.cls == TF_NAN) {
        tf_pack_special(r, 0, TF_NAN);
        return;
    }
    if (a.cls == TF_INF) {
        tf_pack_special(r, sign, b.cls == TF_INF ? TF_NAN : TF_INF);
        return;
    }
    if (b.cls == TF_INF) {
        tf_pack_special(r, sign, TF_ZERO);
        return;
    }
    if (b.cls == TF_ZERO) {
        tf_pack_special(r, sign, a.cls == TF_ZERO ? TF_NAN : TF_INF);
        return;
    }
    if (a.cls == TF_ZERO) {
        tf_pack_special(r, sign, TF_ZERO);
        return;
    }
    /* long division: both significands have the leading bit at 115.
       Produce a 117-bit quotient (leading bit at 116 or 115) with a
       sticky bit from the remainder. */
    rem = a.sig;
    exp = a.exp - b.exp;
    q.lo = q.hi = 0;
    /* rem/b in [0.5, 2): generate bits for 2^0 down to 2^-116 */
    bit.lo = 1;
    bit.hi = 0;
    for (i = 116; i >= 0; i--) {
        if (u128_cmp(rem, b.sig) >= 0) {
            rem = u128_sub(rem, b.sig);
            q = u128_add(q, u128_shl(bit, i));
        }
        rem = u128_shl(rem, 1);
    }
    if (!u128_is_zero(rem))
        q.lo |= 1;
    /* q = (a/b) * 2^116: bit 116 has exponent 'exp', bit 115 exp - 1 */
    tf_pack(r, sign, exp - 1, q);
}

/* ---------------------------------------------------------------- */
/* comparisons: <0, 0, >0. Unordered gives 'nan_result' */

static int tf_cmp(u64 alo, u64 ahi, u64 blo, u64 bhi, int nan_result)
{
    tf a = tf_unpack(alo, ahi), b = tf_unpack(blo, bhi);
    int c;
    if (a.cls == TF_NAN || b.cls == TF_NAN)
        return nan_result;
    if (a.cls == TF_ZERO && b.cls == TF_ZERO)
        return 0;
    if (a.sign != b.sign)
        return a.sign ? -1 : 1;
    /* same sign: compare magnitudes */
    if (a.cls == TF_ZERO)
        c = -1;
    else if (b.cls == TF_ZERO)
        c = 1;
    else if (a.cls == TF_INF || b.cls == TF_INF)
        c = (a.cls == TF_INF) - (b.cls == TF_INF);
    else if (a.exp != b.exp)
        c = a.exp < b.exp ? -1 : 1;
    else
        c = u128_cmp(a.sig, b.sig);
    return a.sign ? -c : c;
}

int __eqtf2(u64 alo, u64 ahi, u64 blo, u64 bhi) { return tf_cmp(alo, ahi, blo, bhi, 1); }
int __netf2(u64 alo, u64 ahi, u64 blo, u64 bhi) { return tf_cmp(alo, ahi, blo, bhi, 1); }
int __lttf2(u64 alo, u64 ahi, u64 blo, u64 bhi) { return tf_cmp(alo, ahi, blo, bhi, 1); }
int __letf2(u64 alo, u64 ahi, u64 blo, u64 bhi) { return tf_cmp(alo, ahi, blo, bhi, 1); }
int __gttf2(u64 alo, u64 ahi, u64 blo, u64 bhi) { return tf_cmp(alo, ahi, blo, bhi, -1); }
int __getf2(u64 alo, u64 ahi, u64 blo, u64 bhi) { return tf_cmp(alo, ahi, blo, bhi, -1); }
int __unordtf2(u64 alo, u64 ahi, u64 blo, u64 bhi)
{
    return tf_unpack(alo, ahi).cls == TF_NAN || tf_unpack(blo, bhi).cls == TF_NAN;
}

/* ---------------------------------------------------------------- */
/* conversions */

/* double -> quad, exact */
void __extenddftf2(u64 *r, double d)
{
    dbits x;
    int sign, exp;
    u64 m;
    u128 sig;

    x.d = d;
    sign = x.u >> 63;
    exp = (x.u >> 52) & 0x7ff;
    m = x.u & ((1ULL << 52) - 1);
    if (exp == 0x7ff) {
        tf_pack_special(r, sign, m ? TF_NAN : TF_INF);
        return;
    }
    if (exp == 0) {
        if (m == 0) {
            tf_pack_special(r, sign, TF_ZERO);
            return;
        }
        /* subnormal: value = m * 2^-1074; tf_pack normalizes */
        sig.lo = m;
        sig.hi = 0;
        tf_pack(r, sign, -1074 + 115, sig);
        return;
    }
    /* value = (2^52 | m) * 2^(exp - 1075): put the leading bit at 115 */
    sig.lo = (1ULL << 52) | m;
    sig.hi = 0;
    sig = u128_shl(sig, 63);
    tf_pack(r, sign, exp - 1023, sig);
}

void __extendsftf2(u64 *r, float f)
{
    __extenddftf2(r, (double)f);
}

/* quad -> double, rounded */
double __trunctfdf2(u64 lo, u64 hi)
{
    tf x = tf_unpack(lo, hi);
    dbits d;
    int exp;
    u128 sig;
    u64 m;

    if (x.cls == TF_NAN) {
        d.u = 0x7ff8000000000000ULL | (u64)x.sign << 63;
        return d.d;
    }
    if (x.cls == TF_INF) {
        d.u = 0x7ff0000000000000ULL | (u64)x.sign << 63;
        return d.d;
    }
    if (x.cls == TF_ZERO) {
        d.u = (u64)x.sign << 63;
        return d.d;
    }
    /* leading bit at 115 -> want 53 bits: bit 115 down to 63; round
       on the 63 low bits */
    exp = x.exp;
    sig = x.sig;
    if (exp < -1022) {
        /* double subnormal */
        int shift = -1022 - exp;
        if (shift > 116) {
            d.u = (u64)x.sign << 63;
            return d.d;
        }
        sig = u128_shr_sticky(sig, shift);
        exp = -1022;
    }
    {
        u64 low = sig.lo; /* 64 bits below the mantissa... bit 63 is the guard */
        u64 half = 1ULL << 62;
        u64 frac = low & ((1ULL << 63) - 1);
        m = sig.hi << 1 | low >> 63; /* bits 115..63: 53 bits */
        (void)half;
        if (frac > (1ULL << 62) || (frac == (1ULL << 62) && (m & 1))) {
            m++;
            if (m >> 53) {
                m >>= 1;
                exp++;
            }
        }
    }
    if (exp > 1023) {
        d.u = 0x7ff0000000000000ULL | (u64)x.sign << 63;
        return d.d;
    }
    if (!(m >> 52)) {
        /* subnormal */
        d.u = (u64)x.sign << 63 | m;
        return d.d;
    }
    d.u = (u64)x.sign << 63 | (u64)(exp + 1023) << 52 | (m & ((1ULL << 52) - 1));
    return d.d;
}

float __trunctfsf2(u64 lo, u64 hi)
{
    /* double rounding is harmless here for all but pathological cases */
    return (float)__trunctfdf2(lo, hi);
}

/* quad -> integers, truncating */
static u64 tf_to_u64_mag(tf x, int *overflow)
{
    u128 sig;
    int shift;
    *overflow = 0;
    if (x.cls == TF_ZERO)
        return 0;
    if (x.cls != 0 || x.exp >= 64) {
        *overflow = 1;
        return 0;
    }
    if (x.exp < 0)
        return 0;
    /* value = sig * 2^(exp - 115), truncated */
    shift = 115 - x.exp;
    sig = u128_shr(x.sig, shift);
    return sig.lo;
}

s64 __fixtfdi(u64 lo, u64 hi)
{
    tf x = tf_unpack(lo, hi);
    int ovf;
    u64 m;
    if (x.cls == TF_NAN)
        return (s64)0x8000000000000000ULL;
    m = tf_to_u64_mag(x, &ovf);
    if (x.sign) {
        if (ovf || m > 0x8000000000000000ULL)
            return (s64)0x8000000000000000ULL;
        return -(s64)m;
    }
    if (ovf || m > 0x7fffffffffffffffULL)
        return 0x7fffffffffffffffLL;
    return (s64)m;
}

int __fixtfsi(u64 lo, u64 hi)
{
    s64 v = __fixtfdi(lo, hi);
    if (v > 0x7fffffffLL) return 0x7fffffff;
    if (v < -0x80000000LL) return (int)0x80000000;
    return (int)v;
}

u64 __fixunstfdi(u64 lo, u64 hi)
{
    tf x = tf_unpack(lo, hi);
    int ovf;
    u64 m;
    if (x.cls == TF_NAN || x.sign)
        return 0;
    m = tf_to_u64_mag(x, &ovf);
    return ovf ? ~0ULL : m;
}

u32 __fixunstfsi(u64 lo, u64 hi)
{
    u64 v = __fixunstfdi(lo, hi);
    return v > 0xffffffffULL ? 0xffffffffu : (u32)v;
}

/* integers -> quad, exact */
void __floatunditf(u64 *r, u64 i)
{
    u128 sig;
    if (i == 0) {
        tf_pack_special(r, 0, TF_ZERO);
        return;
    }
    sig.lo = i;
    sig.hi = 0;
    tf_pack(r, 0, 115, sig); /* bit 115 of sig would be 2^115; tf_pack normalizes */
}

void __floatditf(u64 *r, s64 i)
{
    u128 sig;
    if (i == 0) {
        tf_pack_special(r, 0, TF_ZERO);
        return;
    }
    sig.lo = i < 0 ? -(u64)i : (u64)i;
    sig.hi = 0;
    tf_pack(r, i < 0, 115, sig);
}

void __floatsitf(u64 *r, int i) { __floatditf(r, i); }
void __floatunsitf(u64 *r, u32 i) { __floatunditf(r, i); }

/* ---------------------------------------------------------------- */
/* tcc's own helpers */

double __tcc_ld_load(const void *p)
{
    const u64 *q = p;
    return __trunctfdf2(q[0], q[1]);
}

void __tcc_ld_store(void *p, double d)
{
    __extenddftf2(p, d);
}

/* the two halves of a quad, for passing long double arguments */
u64 __tcc_ld_lo(double d) { u64 q[2]; __extenddftf2(q, d); return q[0]; }
u64 __tcc_ld_hi(double d) { u64 q[2]; __extenddftf2(q, d); return q[1]; }
