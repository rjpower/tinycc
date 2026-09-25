# tcc for WebAssembly

This archive is TinyCC compiled to WebAssembly (WASI Preview 1), together
with everything it needs to compile and link C programs for wasm32. It
is self-contained: no host compiler, no other sysroot.

Extract it into one directory and make that directory visible to the
compiler as `/tcc`. Every path below is relative to it.

    tcc.wasm            the compiler for an ordinary WASI host
    tcc-shellsim.wasm   the same compiler for shellsim (see below)
    wasm32-libtcc1.a    tcc's runtime library
    libshellsim.a       the shellsim bridge, linked by tcc-shellsim.wasm
    include/            tcc's own headers (stdarg.h, stddef.h, ...)
    sysroot/            the wasi-libc subset: headers and libraries
      include/wasm32-wasip1/
      lib/wasm32-wasip1/  libc.a libm.a libsetjmp.a
                          libc-printscan-long-double.a
                          libwasi-emulated-*.a
    licenses/           TinyCC (LGPL-2.1), wasi-libc, the bridge (MIT)
    MANIFEST            versions, git revision and file hashes
    README.md           this file

The compiler needs no arguments to find its files: `/tcc` and
`/tcc/sysroot` are compiled in. Programs it produces use the wasm
exception handling proposal for setjmp/longjmp, so the host must
support it (wasmtime 49 does).

## Under wasmtime

    wasmtime run --dir . --dir /path/to/extracted::/tcc /path/to/extracted/tcc.wasm -o hello.wasm hello.c
    wasmtime hello.wasm

## Under shellsim

`tcc-shellsim.wasm` imports `shellsim.path_chmod` so the executables it
links get virtual execute permission, and C programs calling `chmod()`
are routed to the same virtual call through `libshellsim.a`. Extract
the archive into `/tcc` in the virtual filesystem, mark the compiler
executable, and optionally link it as `/usr/bin/cc`.

## Provenance

See `MANIFEST` for the tcc revision and the wasi-sdk release the
sysroot subset was taken from. The archive is produced by
`wasm/package.sh` in the TinyCC repository, from a CI build of the
tagged revision.
