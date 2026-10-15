"""
Compara o decodificador Python (lpc_codec.py) com lpc.h compilado no PC.
Uso: python teste_bitexato.py [arquivos.wav ...]
Precisa de gcc no PATH. Também faz um teste com quadros aleatórios.
"""
import os, random, subprocess, sys, tempfile
import numpy as np
AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(AQUI))
import lpc_codec as L

def compila():
    exe = os.path.join(tempfile.gettempdir(), "decodifica_pc" + (".exe" if os.name == "nt" else ""))
    subprocess.check_call(["gcc", "-O2", "-o", exe, os.path.join(AQUI, "decodifica_pc.c")])
    return exe

def roda_c(exe, dados):
    with tempfile.TemporaryDirectory() as t:
        b, r = os.path.join(t, "f.bin"), os.path.join(t, "f.raw")
        open(b, "wb").write(dados)
        subprocess.check_call([exe, b, r], stdout=subprocess.DEVNULL)
        return np.fromfile(r, dtype="<i2")

def compara(nome, dados, exe):
    c = roda_c(exe, dados)
    p = L.decodifica(dados)
    ok = len(c) == len(p) and np.array_equal(c, p)
    print(f"{'OK  ' if ok else 'ERRO'} {nome}: {len(dados)} bytes, {len(p)} amostras")
    if not ok:
        n = min(len(c), len(p))
        dif = np.nonzero(c[:n] != p[:n])[0]
        print("   tamanhos", len(c), len(p), "primeira diferença", dif[:5])
    return ok

def quadros_aleatorios(seed, n=300):
    rnd = random.Random(seed)
    qs = []
    for _ in range(n):
        t = rnd.random()
        if t < 0.1:
            qs.append(L.Quadro(energia=0)); continue
        q = L.Quadro(energia=rnd.randint(1, 14), periodo=0 if t < 0.35 else rnd.randint(1, 63))
        q.repete = rnd.random() < 0.2 and len(qs) > 0
        q.k = [rnd.randrange(1 << b) for b in L.BITS_K]
        qs.append(q)
    return qs

if __name__ == "__main__":
    exe = compila()
    tudo = True
    for w in sys.argv[1:]:
        x, fs = L.ler_wav(w)
        r = L.codifica(x, fs, log=lambda *a: None)
        tudo &= compara(os.path.basename(w), r.dados, exe)
    for s in range(20):
        tudo &= compara(f"aleatorio {s}", L.empacota(quadros_aleatorios(s)), exe)
    # bitstream truncado no meio de um quadro
    d = L.empacota(quadros_aleatorios(99))
    for corte in (1, 5, 17, len(d) // 2):
        tudo &= compara(f"truncado em {corte}", d[:corte], exe)
    print("TUDO IGUAL" if tudo else "HÁ DIFERENÇAS")
    sys.exit(0 if tudo else 1)
