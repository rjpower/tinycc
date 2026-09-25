#!/bin/sh
# Assemble the wasm toolchain tarball: tcc compiled to wasm, its runtime
# library and headers, the shellsim bridge library, and the subset of the
# wasi-libc sysroot that C programs link against.  The tarball extracts
# flat into the directory that the compiler sees as /tcc (see README.md).
#
# usage: wasm/package.sh <wasi-sysroot> <out.tar.gz>
# Run from the top directory after: make tcc.wasm tcc-shellsim.wasm libshellsim.a
set -e

sysroot=$1
out=$2
if [ -z "$sysroot" ] || [ -z "$out" ]; then
    echo "usage: $0 <wasi-sysroot> <out.tar.gz>" >&2
    exit 1
fi
top=$(cd "$(dirname "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
pkg=$tmp/pkg

mkdir -p "$pkg/include" "$pkg/sysroot/include" "$pkg/sysroot/lib/wasm32-wasip1" "$pkg/licenses"
cp "$top/tcc.wasm" "$top/tcc-shellsim.wasm" "$top/wasm32-libtcc1.a" "$top/libshellsim.a" "$pkg/"
cp "$top"/include/*.h "$pkg/include/"
cp "$top/wasm/README.md" "$pkg/README.md"

# the sysroot: C headers and the libraries a C program can link with
cp -R "$sysroot/include/wasm32-wasip1" "$pkg/sysroot/include/"
rm -rf "$pkg/sysroot/include/wasm32-wasip1/eh" "$pkg/sysroot/include/wasm32-wasip1/noeh" # the C++ trees
for lib in libc libm libsetjmp libc-printscan-long-double \
           libwasi-emulated-getpid libwasi-emulated-mman \
           libwasi-emulated-process-clocks libwasi-emulated-signal; do
    cp "$sysroot/lib/wasm32-wasip1/$lib.a" "$pkg/sysroot/lib/wasm32-wasip1/"
done

cp "$top/COPYING" "$pkg/licenses/TCC-COPYING"
cp "$top/SHELLSIM-LIBC-LICENSE" "$pkg/licenses/SHELLSIM-LIBC-LICENSE"
for f in "$top"/wasm/wasi-libc-licenses/*; do
    cp "$f" "$pkg/licenses/WASI-LIBC-$(basename "$f")"
done

# provenance
{
    echo "tcc $(cat "$top/VERSION") $(git -C "$top" rev-parse HEAD 2>/dev/null || echo unknown)"
    echo "wasi-sysroot $(head -n 1 "$sysroot/VERSION" 2>/dev/null || echo unknown)"
    sed -n '2,$p' "$sysroot/VERSION" 2>/dev/null | sed 's/^/  /'
    echo
    (cd "$pkg" && find . -type f ! -name MANIFEST | LC_ALL=C sort | xargs sha256sum)
} > "$pkg/MANIFEST"

# a reproducible archive where the tar supports it (GNU)
if tar --version 2>/dev/null | grep -q GNU; then
    tar --sort=name --mtime=@0 --owner=0 --group=0 --numeric-owner \
        -czf "$out" -C "$pkg" .
else
    tar -czf "$out" -C "$pkg" .
fi
ls -l "$out"
