#!/usr/bin/env python3
"""
previa_numero.py - Gera o WAV de como o Paçoca fala um número (voz.falarNumero).

Lê as falas do ../falasNumeros.h (o mesmo arquivo que vai para o AVR), monta
as palavras na mesma ordem e com a mesma pausa do voz.h, e decodifica igual
ao AVR (amostra por amostra).

Uso:
    python previa_numero.py 123 -45 1500
    python previa_numero.py 180 --pausa 30          (pausa entre palavras, ms)

Saída: previas/numero_<valor>.wav e previas/numero_<valor>_buzzer.wav
"""
from __future__ import annotations

import argparse
import os
import re
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import lpc_codec as L  # noqa: E402

UNIDADES = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13",
            "14", "15", "16", "17", "18", "19"]


def le_falas(caminho: str) -> dict[str, bytes]:
    texto = open(caminho, encoding="utf-8").read()
    falas = {}
    for m in re.finditer(r"FALA_(\w+)\[\]\s*PROGMEM\s*=\s*\{([^}]*)\}", texto):
        falas[m.group(1)] = bytes(int(x, 16) for x in re.findall(r"0x[0-9A-Fa-f]{2}", m.group(2)))
    return falas


def ate_mil(n: int) -> list[str]:
    """Mesma regra do numerosPorExtenso.h (1..999)."""
    if n == 100:
        return ["NUM_100"]
    p = []
    c, r = divmod(n, 100)
    if c:
        p.append("NUM_CENTO" if c == 1 else f"NUM_{c}00")
        if r:
            p.append("NUM_E")
    if r:
        if r < 20:
            p.append("NUM_" + UNIDADES[r])
        else:
            p.append(f"NUM_{r // 10}0")
            if r % 10:
                p += ["NUM_E", "NUM_" + UNIDADES[r % 10]]
    return p


def por_extenso(v: int) -> list[str]:
    p = []
    if v < 0:
        p.append("NUM_MENOS")
        v = -v
    if v == 0:
        return p + ["NUM_0"]
    mil, resto = divmod(v, 1000)
    if mil:
        if mil > 1:
            p += ate_mil(mil)
        p.append("NUM_MIL")
        if resto and (resto < 100 or resto % 100 == 0):
            p.append("NUM_E")
    if resto:
        p += ate_mil(resto)
    return p


def main():
    ap = argparse.ArgumentParser(description="Prévia de voz.falarNumero()")
    ap.add_argument("numeros", nargs="+", type=int)
    ap.add_argument("--falas", default=os.path.join(AQUI, "..", "falasNumeros.h"))
    ap.add_argument("--pausa", type=float, default=50.0, help="ms entre palavras (voz.pausaEntreFalas)")
    ap.add_argument("--previas", default=os.path.join(AQUI, "previas"))
    ap.add_argument("--ressonancia", type=float, default=2700.0)
    args = ap.parse_args()

    falas = le_falas(args.falas)
    textos = {}                                   # NUM_20 -> "vinte" (só para mostrar)
    arq = os.path.join(AQUI, "numeros.txt")
    if os.path.exists(arq):
        for linha in open(arq, encoding="utf-8"):
            if ":" in linha and not linha.startswith("#"):
                nome, texto = (t.strip() for t in linha.split(":", 1))
                textos[nome] = "e" if nome == "NUM_E" else texto.rstrip(".")
    os.makedirs(args.previas, exist_ok=True)
    pausa = int(args.pausa * 7843 / 1000)
    for v in args.numeros:
        if not -32768 <= v <= 32767:
            print(f"{v}: fora da faixa (-32768 a 32767)")
            continue
        nomes = por_extenso(v)
        niveis = []
        for i, nome in enumerate(nomes):
            a = L.decodifica(falas[nome])
            niveis += list(L.para_nivel(a, 128)) + [128]
            if i < len(nomes) - 1:
                niveis += [128] * (pausa + 1)
        # (no AVR ainda há ~16 ms de rampa no começo e no fim, sem som)
        x = (np.array(niveis, dtype=np.float64) - 128) / 128.0
        base = os.path.join(args.previas, f"numero_{v}")
        L.grava_wav(base + ".wav", x, L.TAXA)
        L.grava_wav(base + "_buzzer.wav", L.simula_buzzer(x, L.TAXA, args.ressonancia), L.TAXA)
        palavras = " ".join(textos.get(n, n) for n in nomes)
        print(f"{v}: {palavras}  ({(len(niveis) + 129) / L.TAXA:.2f} s) -> {base}.wav")


if __name__ == "__main__":
    main()
