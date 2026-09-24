#!/usr/bin/env python3
"""Run tests/tests2 and tests/wasm/*.c with the wasm32 cross compiler under wasmtime.

usage: tests/wasm/run-tests2.py [name-substring ...]

Needs wasm32-tcc and wasm32-libtcc1.a in the top directory
(make cross-wasm32) and wasmtime in PATH. Tests needing inline asm,
bounds checking, setjmp, threads, tcc -run or ELF specifics are skipped.
"""
import os, sys, subprocess, glob, re
TOP=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC=TOP+'/tests/tests2'
OUT=os.path.join(TOP,'tests','wasm','out')
os.makedirs(OUT, exist_ok=True)
TCC=[TOP+'/wasm32-tcc','-B'+TOP]
ARGS={'31_args':['arg1','arg2','arg3','arg4','arg5'],
      '46_grep':['[^* ]*[:a:d: ]+\\:\\*-/: $', SRC+'/46_grep.c']}
FLAGS={'76_dollars_in_identifiers':['-fdollars-in-identifiers'],
       '60_errors_and_warnings':['-dt'],'96_nodata_wanted':['-dt'],
       '104_inline':[SRC+'/104+_inline.c'],'120_alias':[SRC+'/120+_alias.c']}
only=sys.argv[1:]
SKIP=set("""34_array_assignment 85_asm-outside-function 95_bitfields 95_bitfields_ms
98_al_ax_extend 99_fastcall 113_btdll 114_bound_signal 115_bound_setjmp 116_bound_setjmp2
117_builtins 119_random_stuff 124_atomic_counter 126_bound_global 127_asm_goto 138_arm64_encoding
140_arm64_extasm 141_riscv_asm 144_tls 145_winarm64_interlocked 146_tls_extern 148_linker_symbols""".split())
tests=sorted(glob.glob(SRC+'/[0-9][0-9]_*.c')+glob.glob(SRC+'/[0-9][0-9][0-9]_*.c'))
tests+=sorted(glob.glob(TOP+'/tests/wasm/*.c'))  # wasm specific tests, e.g. setjmp
npass=nfail=0; fails=[]
for t in tests:
    name=os.path.basename(t)[:-2]
    if only and not any(o in name for o in only): continue
    if not only and name in SKIP: continue
    exp=t[:-2]+'.expect'
    if not os.path.exists(exp): continue
    if '[test_' in open(exp,errors='replace').read(): continue  # -dt snippet tests need -run
    wasm=OUT+'/'+name+'.wasm'
    cmd=TCC+FLAGS.get(name,[])+['-o',wasm,t]
    try:
        c=subprocess.run(cmd,capture_output=True,text=True,errors='replace',timeout=60,cwd=SRC)
    except subprocess.TimeoutExpired:
        nfail+=1; fails.append((name,'COMPILE TIMEOUT')); continue
    out=c.stdout+c.stderr
    if c.returncode==0 and os.path.exists(wasm):
        try:
            r=subprocess.run(['wasmtime','run','--dir',SRC,'--dir','.',wasm]+ARGS.get(name,[]),capture_output=True,text=True,errors='replace',timeout=30,cwd=SRC)
            out+=r.stdout+r.stderr
        except subprocess.TimeoutExpired:
            nfail+=1; fails.append((name,'RUN TIMEOUT')); continue
    want=open(exp,errors='replace').read()
    # normalize paths like the Makefile does
    out=out.replace(SRC+'/','').replace(TOP+'/','')
    norm=lambda t: '\n'.join(l.rstrip() for l in t.strip().split('\n'))
    if norm(out)==norm(want):
        npass+=1
    else:
        nfail+=1; fails.append((name,out[-600:] if len(out)>600 else out))
        open(OUT+'/'+name+'.out','w').write(out)
for n,m in fails:
    print("=== FAIL",n); print(m[:800])
print("PASS %d FAIL %d"%(npass,nfail))
sys.exit(1 if nfail else 0)
