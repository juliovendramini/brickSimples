#!/usr/bin/env python3
"""
enviar_fala.py - Manda uma fala para o Paçoca pela serial, em tempo real.

O brick precisa estar rodando um sketch com SUPORTE_VOZ_SERIAL e chamando
voz.atualizaSerial(Serial) no loop (veja exemplos/vozMusica.ino).

Uso:
    python enviar_fala.py --porta COM5 "Vítima encontrada na sala três"
    python enviar_fala.py --porta /dev/ttyUSB0 --bin previas/OLA.bin
    python enviar_fala.py --porta COM5 --wav gravacao.wav
    python enviar_fala.py --porta COM5 --listar          (mostra as portas)

Protocolo (igual a Voz::atualizaSerial):
    PC -> brick : 0xA5, tipo, tamanho, dados..., soma(tipo+tamanho+dados) & 0xFF
                  tipo 'D' dados, 'F' fim, 'P' parar, 'S' status
    brick -> PC : 0x5A, código, bytes livres na FIFO, estado (bit0 = falando),
                  (código + livres + estado) XOR 0xA5

Dependências: pip install pyserial numpy scipy  (+ piper-tts para texto)
"""
from __future__ import annotations

import argparse
import os
import sys
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)

MAX_DADOS = 56          # cabe no buffer de 64 bytes da serial do AVR com folga


class Brick:
    def __init__(self, porta: str, baud: int, reset: bool, espera_boot: float):
        try:
            import serial
        except ImportError:
            sys.exit("Instale o pyserial:  pip install pyserial")
        self.s = serial.Serial()
        self.s.port = porta
        self.s.baudrate = baud
        self.s.timeout = 0.05
        if not reset:
            self.s.dtr = False      # não reinicia o brick ao abrir a porta
            self.s.rts = False
        self.s.open()
        if reset:
            time.sleep(espera_boot)  # o bootloader + brick.inicializa() levam um tempo
        self.s.reset_input_buffer()

    def pacote(self, tipo: str, dados: bytes = b"", tentativas: int = 5) -> tuple[int, int, int]:
        t = ord(tipo)
        soma = (t + len(dados) + sum(dados)) & 0xFF
        quadro = bytes([0xA5, t, len(dados)]) + dados + bytes([soma])
        for _ in range(tentativas):
            self.s.write(quadro)
            resp = self._resposta(0.5)
            if resp is None:
                continue
            codigo, livre, estado = resp
            if codigo == 0:
                return codigo, livre, estado
            if codigo == 2:          # sem espaço: espera e tenta de novo
                time.sleep(0.05)
                continue
            if codigo == 3:
                raise RuntimeError("o brick não reconheceu o pacote")
        raise TimeoutError("o brick não respondeu. Ele está rodando voz.atualizaSerial(Serial)?")

    def _resposta(self, limite: float):
        fim = time.time() + limite
        buf = bytearray()
        while time.time() < fim:
            b = self.s.read(1)
            if not b:
                continue
            if not buf and b[0] != 0x5A:
                # texto que o brick imprimiu (Serial.print): mostra e ignora
                continue
            buf += b
            if len(buf) == 5:
                if ((buf[1] + buf[2] + buf[3]) & 0xFF) ^ 0xA5 == buf[4]:
                    return buf[1], buf[2], buf[3]
                # não era resposta (texto impresso pelo brick): procura outro 0x5A
                i = buf.find(0x5A, 1)
                buf = buf[i:] if i > 0 else bytearray()
        return None

    def sincroniza(self, limite: float = 6.0):
        """Espera o brick responder (ele pode estar reiniciando)."""
        fim = time.time() + limite
        while time.time() < fim:
            try:
                return self.pacote("S", tentativas=1)
            except TimeoutError:
                pass
        raise TimeoutError("o brick não respondeu. Ele está rodando voz.atualizaSerial(Serial)?")

    def envia_fala(self, dados: bytes, esperar: bool = True):
        _, livre, _ = self.sincroniza()
        pos = 0
        t0 = time.time()
        while pos < len(dados):
            n = min(MAX_DADOS, livre, len(dados) - pos)
            if n <= 0:
                time.sleep(0.02)
                _, livre, _ = self.pacote("S")
                continue
            _, livre, _ = self.pacote("D", dados[pos:pos + n])
            pos += n
        self.pacote("F")
        print(f"enviado: {len(dados)} bytes em {time.time() - t0:.2f} s")
        if esperar:
            while True:
                time.sleep(0.1)
                _, _, estado = self.pacote("S")
                if not (estado & 1):
                    break
            print("o brick terminou de falar")

    def parar(self):
        self.pacote("P")


def main():
    ap = argparse.ArgumentParser(description="Envia fala LPC para o Paçoca pela serial.")
    ap.add_argument("texto", nargs="?", help="texto a falar (usa a TTS)")
    ap.add_argument("--porta", help="COM5, /dev/ttyUSB0 ...")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--bin", help="bitstream LPC pronto (previas/NOME.bin)")
    ap.add_argument("--wav", help="gravação WAV para codificar")
    ap.add_argument("--motor", choices=["piper", "espeak"], default="piper")
    ap.add_argument("--voz", default=None)
    ap.add_argument("--pasta-vozes", default=os.path.join(AQUI, "vozes"))
    ap.add_argument("--velocidade", type=float, default=1.25)
    ap.add_argument("--reset", action="store_true",
                    help="deixa o brick reiniciar ao abrir a porta (padrão: não reinicia)")
    ap.add_argument("--espera-boot", type=float, default=2.5)
    ap.add_argument("--nao-esperar", action="store_true", help="não espera o brick terminar de falar")
    ap.add_argument("--parar", action="store_true", help="só manda o brick parar de falar")
    ap.add_argument("--listar", action="store_true", help="lista as portas seriais")
    args = ap.parse_args()

    if args.listar:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"{p.device:15} {p.description}")
        return
    if not args.porta:
        sys.exit("informe --porta (use --listar para ver as portas)")

    dados = None
    if not args.parar:
        if args.bin:
            dados = open(args.bin, "rb").read()
        else:
            import lpc_codec as L
            if args.wav:
                x, fs = L.ler_wav(args.wav)
            elif args.texto:
                from gerar_falas import MotorEspeak, MotorPiper
                if args.motor == "piper":
                    motor = MotorPiper(args.voz or "pt_BR-faber-medium", args.pasta_vozes, args.velocidade, None)
                else:
                    motor = MotorEspeak(args.voz or "pt-br", args.velocidade)
                t0 = time.time()
                x, fs = motor.fala(args.texto)
                print(f"TTS: {time.time() - t0:.2f} s")
            else:
                sys.exit("informe um texto, --bin ou --wav")
            t0 = time.time()
            r = L.codifica(x, fs)
            print(f"LPC: {time.time() - t0:.2f} s - {L.estatisticas(r)}")
            dados = r.dados

    b = Brick(args.porta, args.baud, args.reset, args.espera_boot)
    if args.parar:
        b.sincroniza()
        b.parar()
        return
    b.envia_fala(dados, esperar=not args.nao_esperar)


if __name__ == "__main__":
    main()
