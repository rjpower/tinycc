/* WASI command entry point for tcc's wasm32 target.
   Mirrors wasi-libc's crt1-command.c (which is built position independent
   in some distributions and cannot be linked by tcc). */

extern void __wasm_call_ctors(void);
extern void __wasm_call_dtors(void);
extern int __main_void(void);
extern _Noreturn void __wasi_proc_exit(int code);

void _start(void)
{
    int r;
    __wasm_call_ctors();
    r = __main_void();
    __wasm_call_dtors();
    if (r != 0)
        __wasi_proc_exit(r);
}
