#!/usr/bin/env python3
"""
gerar_falas.py - Converte frases em texto em falas LPC para o brick Paçoca.

    texto -> Piper (TTS neural) -> WAV -> codificador LPC -> falas.h

Uso:
    python gerar_falas.py frases.txt
    python gerar_falas.py frases.txt --voz pt_BR-cadu-medium --velocidade 1.3
    python gerar_falas.py frases.txt --motor espeak          (sem Piper)

Formato do frases.txt (UTF-8), uma fala por linha:
    VITIMA: Vítima encontrada.
    SALA_TRES: Sala três.
    MINHA_VOZ: @gravacoes/oi.wav        (usa uma gravação em vez da TTS)
    # comentário

Saídas:
    ../falas.h                  vetores PROGMEM FALA_<NOME> para o sketch
                                (outro nome com -o, ex.: -o ../falasAulas.h)
    previas/<NOME>.wav          o que o AVR vai gerar, amostra por amostra
    previas/<NOME>_buzzer.wav   o mesmo passando por um modelo de buzzer piezo
    previas/<NOME>_tts.wav      a saída da TTS, antes do LPC
    previas/<NOME>.bin          o bitstream (para enviar_fala.py)

Dependências:  pip install numpy scipy piper-tts
"""
from __future__ import annotations

import argparse
import datetime
import io
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unicodedata
import wave

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import lpc_codec as L  # noqa: E402


# ------------------------------------------------------------------ TTS
class MotorTTS:
    def fala(self, texto: str) -> tuple[np.ndarray, int]:
        raise NotImplementedError


class MotorPiper(MotorTTS):
    def __init__(self, voz: str, pasta: str, velocidade: float, locutor: int | None):
        try:
            from piper import PiperVoice, SynthesisConfig
        except ImportError:
            sys.exit("Piper não instalado. Rode:  pip install piper-tts\n"
                     "(ou use --motor espeak)")
        modelo = voz if voz.endswith(".onnx") else os.path.join(pasta, voz + ".onnx")
        if not os.path.exists(modelo):
            print(f"Baixando a voz {voz} para {pasta} ...")
            os.makedirs(pasta, exist_ok=True)
            from piper.download_voices import download_voice
            from pathlib import Path
            download_voice(voz, Path(pasta))
        self.voz = PiperVoice.load(modelo)
        self.cfg = SynthesisConfig(length_scale=velocidade, speaker_id=locutor)

    def fala(self, texto):
        buf = io.BytesIO()
        with wave.open(buf, "wb") as w:
            self.voz.synthesize_wav(texto, w, syn_config=self.cfg)
        buf.seek(0)
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as t:
            t.write(buf.read())
        try:
            return L.ler_wav(t.name)
        finally:
            os.unlink(t.name)


class MotorEspeak(MotorTTS):
    def __init__(self, voz: str, velocidade: float):
        self.exe = shutil.which("espeak-ng") or shutil.which("espeak")
        if not self.exe:
            sys.exit("espeak-ng não encontrado no PATH.")
        self.voz = voz
        self.vel = int(round(160 / velocidade))

    def fala(self, texto):
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as t:
            nome = t.name
        try:
            subprocess.check_call([self.exe, "-v", self.voz, "-s", str(self.vel), "-w", nome, texto])
            return L.ler_wav(nome)
        finally:
            os.unlink(nome)


# ------------------------------------------------------------------ utilidades
def nome_c(nome: str) -> str:
    s = unicodedata.normalize("NFKD", nome).encode("ascii", "ignore").decode()
    s = re.sub(r"[^A-Za-z0-9]+", "_", s).strip("_").upper()
    if not s or s[0].isdigit():
        s = "F_" + s
    return s


def le_frases(caminho: str) -> list[tuple[str, str]]:
    frases = []
    with open(caminho, encoding="utf-8-sig") as f:
        for n, linha in enumerate(f, 1):
            linha = linha.strip()
            if not linha or linha.startswith("#"):
                continue
            if ":" not in linha:
                sys.exit(f"{caminho}:{n}: use o formato  NOME: texto")
            nome, texto = linha.split(":", 1)
            frases.append((nome_c(nome), texto.strip()))
    nomes = [f[0] for f in frases]
    rep = {x for x in nomes if nomes.count(x) > 1}
    if rep:
        sys.exit(f"Nomes repetidos: {', '.join(sorted(rep))}")
    return frases


def vetor_c(nome: str, dados: bytes, comentario: str) -> str:
    linhas = []
    for i in range(0, len(dados), 16):
        linhas.append("    " + ", ".join(f"0x{b:02X}" for b in dados[i:i + 16]))
    corpo = ",\n".join(linhas)
    return f"/* {comentario} */\nconst uint8_t FALA_{nome}[] PROGMEM = {{\n{corpo}\n}};\n"


# ------------------------------------------------------------------ principal
def main():
    ap = argparse.ArgumentParser(description="Gera falas LPC (falas.h) para o brick Paçoca.")
    ap.add_argument("frases", help="arquivo de frases (NOME: texto)")
    ap.add_argument("-o", "--saida", default=os.path.join(AQUI, "..", "falas.h"),
                    help="arquivo .h gerado (padrão: ../falas.h)")
    ap.add_argument("--previas", default=os.path.join(AQUI, "previas"), help="pasta das prévias")
    ap.add_argument("--sem-previas", action="store_true")
    ap.add_argument("--motor", choices=["piper", "espeak"], default="piper")
    ap.add_argument("--voz", default=None,
                    help="Piper: pt_BR-faber-medium (padrão), pt_BR-cadu-medium, pt_BR-jeff-medium "
                         "ou caminho de um .onnx. espeak: pt-br, mb-br1 ...")
    ap.add_argument("--pasta-vozes", default=os.path.join(AQUI, "vozes"))
    ap.add_argument("--locutor", type=int, default=None, help="Piper: id do locutor (vozes com vários)")
    ap.add_argument("--velocidade", type=float, default=1.25,
                    help="fator de duração: >1 fala mais devagar (ajuda o LPC). Padrão 1.25")
    ap.add_argument("--pitch-min", type=float, default=70.0, help="Hz (voz masculina ~70, feminina ~140)")
    ap.add_argument("--pitch-max", type=float, default=320.0, help="Hz")
    ap.add_argument("--limiar-sonoro", type=float, default=0.38,
                    help="0..1: mais alto = mais quadros viram ruído")
    ap.add_argument("--sem-repeticao", action="store_true",
                    help="não usa quadros de repetição (fica maior, às vezes mais nítido)")
    ap.add_argument("--ganho-db", type=float, default=0.0, help="ganho extra na codificação")
    ap.add_argument("--ressonancia", type=float, default=2700.0,
                    help="Hz da ressonância do buzzer na prévia _buzzer")
    args = ap.parse_args()

    frases = le_frases(args.frases)
    if not frases:
        sys.exit("Nenhuma frase no arquivo.")
    base = os.path.dirname(os.path.abspath(args.frases))

    motor = None
    if any(not t.startswith("@") for _, t in frases):
        if args.motor == "piper":
            motor = MotorPiper(args.voz or "pt_BR-faber-medium", args.pasta_vozes, args.velocidade, args.locutor)
        else:
            motor = MotorEspeak(args.voz or "pt-br", args.velocidade)

    p = L.Parametros(pitch_min_hz=args.pitch_min, pitch_max_hz=args.pitch_max,
                     limiar_sonoro=args.limiar_sonoro, usar_repeticao=not args.sem_repeticao,
                     ganho_db=args.ganho_db)
    if not args.sem_previas:
        os.makedirs(args.previas, exist_ok=True)

    blocos = []
    total = 0
    for nome, texto in frases:
        if texto.startswith("@"):
            arq = texto[1:].strip()
            if not os.path.isabs(arq):
                arq = os.path.join(base, arq)
            x, fs = L.ler_wav(arq)
            origem = os.path.basename(arq)
        else:
            x, fs = motor.fala(texto)
            origem = texto
        r = L.codifica(x, fs, p)
        total += len(r.dados)
        dur = len(r.amostras) / L.TAXA
        print(f"FALA_{nome:<20} {L.estatisticas(r)}")
        comentario = f'"{origem}" - {dur:.2f} s, {len(r.dados)} bytes'
        blocos.append(vetor_c(nome, r.dados, comentario.replace("*/", "* /")))
        if not args.sem_previas:
            pv = os.path.join(args.previas, nome)
            sinal = r.amostras.astype(np.float64) / 2048.0
            L.grava_wav(pv + ".wav", sinal, L.TAXA)
            # o que sai no pino: valor do PWM com o volume padrão (200%)
            nivel = (L.para_nivel(r.amostras, 128).astype(np.float64) - 128) / 128.0
            L.grava_wav(pv + "_buzzer.wav", L.simula_buzzer(nivel, L.TAXA, args.ressonancia), L.TAXA)
            L.grava_wav(pv + "_tts.wav", r.entrada, L.TAXA)
            with open(pv + ".bin", "wb") as f:
                f.write(r.dados)

    agora = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    voz = args.voz or ("pt_BR-faber-medium" if args.motor == "piper" else "pt-br")
    desc = (f"motor {args.motor}, voz {os.path.basename(voz)}, "
            f"velocidade {args.velocidade}")
    arquivo = os.path.basename(args.saida)
    guarda = nome_c(os.path.splitext(arquivo)[0]) + "_H"      # falas.h -> FALAS_H
    cab = (f"/*\n * {arquivo} - gerado por ferramentas/gerar_falas.py em {agora}. Não edite à mão.\n"
           f" * {desc}\n * {len(frases)} falas, {total} bytes de flash no total.\n"
           f" * Taxa de amostragem no AVR: {L.TAXA:.2f} Hz (Timer2, 16 MHz).\n */\n"
           f"#ifndef {guarda}\n#define {guarda}\n\n#include <avr/pgmspace.h>\n\n")
    with open(args.saida, "w", encoding="utf-8", newline="\n") as f:
        f.write(cab + "\n".join(blocos) + f"\n#endif /* {guarda} */\n")
    print(f"\n{args.saida}: {len(frases)} falas, {total} bytes")
    if not args.sem_previas:
        print(f"prévias em {args.previas}")


if __name__ == "__main__":
    main()
