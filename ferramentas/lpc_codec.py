"""
lpc_codec.py - Codificador e decodificador LPC (formato TMS5220/Talkie) para o
brick Paçoca.

  * Decodificador: tradução linha a linha de ../lpc.h, em aritmética inteira.
    Gera exatamente as mesmas amostras que o AVR (verificado por
    teste_bitexato.py). É ele que produz as prévias em WAV.
  * Codificador: áudio -> quadros LPC (pitch, sonoro/surdo, energia, K1..K10)
    -> bitstream, com ajuste de ganho por análise-por-síntese usando o
    próprio decodificador.

Só depende de numpy e scipy.
"""
from __future__ import annotations

import math
import wave
from dataclasses import dataclass, field
from fractions import Fraction

import numpy as np
from scipy import signal

# ---------------------------------------------------------------- constantes
F_CPU = 16_000_000
TAXA = F_CPU / 510 / 4          # 7843,14 Hz: Timer2 phase-correct, 1 amostra a cada 4 ciclos
AMOSTRAS_POR_PASSO = 25
PASSOS_POR_QUADRO = 8
QUADRO = AMOSTRAS_POR_PASSO * PASSOS_POR_QUADRO   # 200 amostras
SAIDA_MAX = 2047

# ---------------------------------------------------------------- tabelas (iguais a lpc.h)
TAB_ENERGIA = [0, 2, 3, 4, 5, 7, 10, 15, 20, 32, 41, 57, 81, 114, 161, 0]
TAB_PERIODO = [
    0, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 45, 47, 49,
    51, 53, 54, 57, 59, 61, 63, 66, 69, 71, 73, 77, 79, 81, 85, 87,
    92, 95, 99, 102, 106, 110, 115, 119, 123, 128, 133, 138, 143, 149, 154, 160]
TAB_K = [
    [-32064, -31872, -31808, -31680, -31552, -31424, -31232, -30848,
     -30592, -30336, -30016, -29696, -29376, -28928, -28480, -27968,
     -26368, -24320, -21696, -18432, -14528, -10112, -5184, -64,
     5120, 10048, 14464, 18368, 21568, 24256, 26304, 27904],
    [-20992, -19392, -17536, -15616, -13504, -11200, -8832, -6336,
     -3776, -1152, 1536, 4096, 6720, 9152, 11520, 13760,
     15872, 17792, 19584, 21184, 22656, 23936, 25088, 26112,
     27008, 27840, 28480, 29120, 29632, 30080, 30464, 32384],
    [-110, -97, -83, -70, -56, -43, -29, -16, -2, 11, 25, 38, 52, 65, 79, 92],
    [-82, -68, -54, -40, -26, -12, 1, 15, 29, 43, 57, 71, 85, 99, 113, 126],
    [-82, -70, -59, -47, -35, -24, -12, -1, 11, 22, 34, 46, 57, 69, 81, 92],
    [-64, -53, -42, -31, -20, -9, 3, 14, 25, 36, 47, 58, 69, 80, 91, 102],
    [-77, -65, -53, -41, -29, -17, -5, 7, 19, 31, 43, 55, 67, 79, 90, 102],
    [-64, -40, -16, 7, 31, 55, 79, 102],
    [-64, -44, -24, -4, 16, 37, 57, 77],
    [-51, -33, -15, 4, 22, 40, 59, 77],
]
BITS_K = [5, 5, 4, 4, 4, 4, 4, 3, 3, 3]
ESCALA_K = [32768.0, 32768.0] + [128.0] * 8      # Q15 para K1,K2; Q7 para o resto
CHIRP = [0, 42, -44, 50, -78, 18, 37, 20, 2, -31, -59, 2, 95, 90,
         5, 15, 38, -4, -91, -91, -42, -35, -36, -4, 37, 43, 34, 33,
         15, -1, -8, -18, -19, -17, -9, -10, -6, 0, 3, 2, 1]
DESLOC_INTERP = [3, 3, 3, 2, 2, 1, 1, 0]


def _i16(v: int) -> int:
    """Simula o estouro (wrap) de int16_t."""
    return ((v + 32768) & 0xFFFF) - 32768


# ================================================================ quadros
@dataclass
class Quadro:
    energia: int = 0           # índice 0..14 (0 = silêncio)
    repete: bool = False
    periodo: int = 0           # índice 0..63 (0 = surdo)
    k: list = field(default_factory=lambda: [0] * 10)   # índices

    def bits(self) -> int:
        if self.energia == 0:
            return 4
        if self.repete:
            return 11
        return 29 if self.periodo == 0 else 50


class EscritorBits:
    """Empacota no mesmo formato que o Talkie: cada campo MSB primeiro,
    bits ocupando cada byte a partir do LSB."""

    def __init__(self):
        self.dados = bytearray()
        self.n = 0

    def escreve(self, valor: int, nbits: int):
        for i in range(nbits - 1, -1, -1):
            b = (valor >> i) & 1
            if self.n % 8 == 0:
                self.dados.append(0)
            self.dados[-1] |= b << (self.n % 8)
            self.n += 1


def empacota(quadros: list[Quadro]) -> bytes:
    w = EscritorBits()
    for q in quadros:
        w.escreve(q.energia, 4)
        if q.energia == 0:
            continue
        w.escreve(1 if q.repete else 0, 1)
        w.escreve(q.periodo, 6)
        if q.repete:
            continue
        n = 10 if q.periodo else 4
        for i in range(n):
            w.escreve(q.k[i], BITS_K[i])
    w.escreve(15, 4)     # fim
    return bytes(w.dados)


# ================================================================ decodificador (espelho de lpc.h)
class Decodificador:
    def __init__(self, dados: bytes):
        self.dados = dados
        self.ptr = 0
        self.fim = len(dados)
        self.bit = 0
        self.energia = self.alvo_energia = 0
        self.periodo = self.alvo_periodo = 0
        self.k = [0] * 10
        self.alvo_k = [0] * 10
        self.x = [0] * 10
        self.ruido = 1
        self.cont_periodo = 0
        self.cont_amostra = 0
        self.passo = 0
        self.ultimo_quadro = False
        self.ativo = True

    # --- bits
    def _bits_disponiveis(self) -> int:
        nbytes = self.fim - self.ptr
        if nbytes > 31:
            return 255
        return nbytes * 8 - self.bit

    def _ler_bits(self, n: int) -> int:
        v = 0
        for _ in range(n):
            b = (self.dados[self.ptr] >> self.bit) & 1
            v = ((v << 1) | b) & 0xFF
            self.bit += 1
            if self.bit == 8:
                self.bit = 0
                self.ptr += 1
        return v

    # --- quadro
    def _carrega_quadro(self):
        disp = self._bits_disponiveis()
        e = 15 if disp < 4 else self._ler_bits(4)
        if e == 0:
            self.alvo_energia = 0
            return
        if e == 15 or disp < 11:
            self.alvo_energia = 0
            self.ultimo_quadro = True
            return
        self.alvo_energia = TAB_ENERGIA[e]
        rep = self._ler_bits(1)
        ip = self._ler_bits(6)
        self.alvo_periodo = TAB_PERIODO[ip]
        if not rep:
            if disp < 29 or (ip and disp < 50):
                self.alvo_energia = 0
                self.ultimo_quadro = True
                return
            n = 10 if ip else 4
            for i in range(n):
                self.alvo_k[i] = TAB_K[i][self._ler_bits(BITS_K[i])]
            if not ip:
                for i in range(4, 10):
                    self.alvo_k[i] = 0
        if self.energia == 0 or ((self.periodo == 0) != (self.alvo_periodo == 0)):
            self.periodo = self.alvo_periodo
            self.k = list(self.alvo_k)

    def _passo(self):
        if self.passo == 0:
            if self.ultimo_quadro:
                self.ativo = False
                return
            self._carrega_quadro()
        s = DESLOC_INTERP[self.passo]
        self.energia = _i16(self.energia + (_i16(self.alvo_energia - self.energia) >> s))
        self.periodo = _i16(self.periodo + (_i16(self.alvo_periodo - self.periodo) >> s))
        for i in range(10):
            self.k[i] = _i16(self.k[i] + (_i16(self.alvo_k[i] - self.k[i]) >> s))
        self.passo = (self.passo + 1) & 7

    def proxima_amostra(self) -> int:
        if not self.ativo:
            return 0
        if self.cont_amostra == 0:
            self._passo()
            if not self.ativo:
                return 0
            self.cont_amostra = AMOSTRAS_POR_PASSO
        self.cont_amostra -= 1

        if self.periodo:
            self.cont_periodo = (self.cont_periodo + 1) & 0xFF
            if self.cont_periodo >= (self.periodo & 0xFF):
                self.cont_periodo = 0
            if self.cont_periodo < len(CHIRP):
                u = _i16((CHIRP[self.cont_periodo] * self.energia) >> 4)
            else:
                u = 0
        else:
            r = self.ruido
            self.ruido = ((r >> 1) ^ (0xB800 if (r & 1) else 0)) & 0xFFFF
            u = _i16(self.energia << 2) if (self.ruido & 1) else _i16(-(self.energia << 2))

        k, x = self.k, self.x
        q7 = lambda kk, v: _i16((kk * v) >> 7)
        q15 = lambda kk, v: _i16((kk * v) >> 15)
        u = _i16(u - q7(k[9], x[9]))
        u8 = _i16(u - q7(k[8], x[8]))
        u7 = _i16(u8 - q7(k[7], x[7]))
        u6 = _i16(u7 - q7(k[6], x[6]))
        u5 = _i16(u6 - q7(k[5], x[5]))
        u4 = _i16(u5 - q7(k[4], x[4]))
        u3 = _i16(u4 - q7(k[3], x[3]))
        u2 = _i16(u3 - q7(k[2], x[2]))
        u1 = _i16(u2 - q15(k[1], x[1]))
        u0 = _i16(u1 - q15(k[0], x[0]))
        if u0 > SAIDA_MAX:
            u0 = SAIDA_MAX
        if u0 < -SAIDA_MAX:
            u0 = -SAIDA_MAX
        x[9] = _i16(x[8] + q7(k[8], u8))
        x[8] = _i16(x[7] + q7(k[7], u7))
        x[7] = _i16(x[6] + q7(k[6], u6))
        x[6] = _i16(x[5] + q7(k[5], u5))
        x[5] = _i16(x[4] + q7(k[4], u4))
        x[4] = _i16(x[3] + q7(k[3], u3))
        x[3] = _i16(x[2] + q7(k[2], u2))
        x[2] = _i16(x[1] + q15(k[1], u1))
        x[1] = _i16(x[0] + q15(k[0], u0))
        x[0] = u0
        return u0


def decodifica(dados: bytes) -> np.ndarray:
    """Bitstream -> amostras int16 (-2047..2047), exatamente como no AVR."""
    d = Decodificador(dados)
    saida = []
    while True:
        a = d.proxima_amostra()
        if not d.ativo:
            break
        saida.append(a)
    return np.array(saida, dtype=np.int16)


def para_nivel(a: np.ndarray, ganho: int = 128) -> np.ndarray:
    """Igual a lpc_paraNivel(): amostra -> valor do OCR2A (0..255). ganho em Q6 (64 = 1x)."""
    v = (a.astype(np.int64) * ganho) >> 10
    return (128 + np.clip(v, -128, 127)).astype(np.uint8)


# ================================================================ áudio
def ler_wav(caminho: str) -> tuple[np.ndarray, int]:
    with wave.open(caminho, "rb") as w:
        fs = w.getframerate()
        nc = w.getnchannels()
        sw = w.getsampwidth()
        raw = w.readframes(w.getnframes())
    if sw == 2:
        x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    elif sw == 1:
        x = (np.frombuffer(raw, dtype=np.uint8).astype(np.float64) - 128) / 128.0
    elif sw == 4:
        x = np.frombuffer(raw, dtype="<i4").astype(np.float64) / 2147483648.0
    else:
        raise ValueError(f"WAV com {8 * sw} bits não suportado")
    if nc > 1:
        x = x.reshape(-1, nc).mean(axis=1)
    return x, fs


def grava_wav(caminho: str, x: np.ndarray, fs: float):
    """x em float (-1..1). fs pode ser fracionário; o cabeçalho leva o inteiro mais próximo."""
    y = np.clip(np.round(x * 32767), -32768, 32767).astype("<i2")
    with wave.open(caminho, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(int(round(fs)))
        w.writeframes(y.tobytes())


def reamostra(x: np.ndarray, fs_ent: float, fs_sai: float) -> np.ndarray:
    r = Fraction(fs_sai / fs_ent).limit_denominator(2000)
    return signal.resample_poly(x, r.numerator, r.denominator)


def para_taxa_do_avr(x: np.ndarray, fs: int) -> np.ndarray:
    """Leva o áudio para 7843,14 Hz (8000 Hz e depois 50/51)."""
    x8 = reamostra(x, fs, 8000) if fs != 8000 else x
    return signal.resample_poly(x8, 50, 51)


def _trecho_principal(env: np.ndarray, lim: float, curto_ms: float = 160.0, vao_ms: float = 80.0,
                      fraco_db: float = -20.0) -> tuple[int, int]:
    """Início e fim da fala, ignorando estalos/respiração soltos nas pontas.

    A TTS às vezes começa com um estalo ou respiração seguido de ~0,3 s de
    silêncio antes da fala. Um trecho com som nas pontas, separado do resto
    por pelo menos vao_ms de silêncio, é descartado se for curto (menos de
    curto_ms) ou fraco (pico fraco_db abaixo do pico da fala).
    """
    acima = env > lim
    d = np.diff(np.concatenate(([0], acima.astype(np.int8), [0])))
    ini = list(np.where(d == 1)[0])
    fim = list(np.where(d == -1)[0])           # exclusivo
    curto = int(curto_ms / 1000 * TAXA)
    vao = int(vao_ms / 1000 * TAXA)
    junta = int(0.030 * TAXA)                  # vãos de menos de 30 ms não separam trechos
    i = 1
    while i < len(ini):
        if ini[i] - fim[i - 1] < junta:
            fim[i - 1] = fim[i]; ini.pop(i); fim.pop(i)
        else:
            i += 1
    fraco = env.max() * 10 ** (fraco_db / 20)

    def descartavel(j: int) -> bool:
        return fim[j] - ini[j] < curto or env[ini[j]:fim[j]].max() < fraco

    while len(ini) > 1 and ini[1] - fim[0] >= vao and descartavel(0):
        ini.pop(0); fim.pop(0)
    while len(ini) > 1 and ini[-1] - fim[-2] >= vao and descartavel(-1):
        ini.pop(); fim.pop()
    return ini[0], fim[-1] - 1


def _limites_fala(env: np.ndarray, lim: float) -> tuple[int, int]:
    """Acha a fala com um limiar 10 dB acima (o ruído entre o estalo e a fala
    não engana) e depois estende as pontas até o limiar normal, para não
    cortar começos e finais suaves (s, f, m...)."""
    a, b = _trecho_principal(env, lim * 10 ** (10 / 20))
    junta = int(0.030 * TAXA)
    baixo = 0
    while a > 0 and baixo < junta:
        a -= 1
        baixo = baixo + 1 if env[a] <= lim else 0
    a += baixo
    baixo = 0
    while b < len(env) - 1 and baixo < junta:
        b += 1
        baixo = baixo + 1 if env[b] <= lim else 0
    b -= baixo
    return a, b


def prepara(x: np.ndarray, passa_alta_hz: float = 100.0, limiar_silencio_db: float = -40.0,
            margem_ms: float = 40.0) -> np.ndarray:
    """Passa-alta, corta silêncio no começo/fim e normaliza o pico em 0,95."""
    sos = signal.butter(2, passa_alta_hz, "highpass", fs=TAXA, output="sos")
    x = signal.sosfilt(sos, x)
    env = np.convolve(np.abs(x), np.ones(80) / 80, mode="same")
    lim = env.max() * 10 ** (limiar_silencio_db / 20)
    idx = np.where(env > lim)[0]
    if len(idx):
        ini, fim = _limites_fala(env, lim)
        m = int(margem_ms / 1000 * TAXA)
        x = x[max(0, ini - m): min(len(x), fim + m)]
    pico = np.max(np.abs(x))
    if pico > 0:
        x = x / pico * 0.95
    return x


# ================================================================ análise
def _levinson(r: np.ndarray, ordem: int = 10) -> np.ndarray:
    """Coeficientes de reflexão no padrão do TMS5220 (k1 ~ -0,9 em vogais)."""
    a = np.zeros(ordem + 1)
    a[0] = 1.0
    erro = r[0]
    k = np.zeros(ordem)
    for i in range(1, ordem + 1):
        acc = r[i] + np.dot(a[1:i], r[i - 1:0:-1])
        ki = -acc / erro if erro > 0 else 0.0
        ki = float(np.clip(ki, -0.999, 0.999))
        k[i - 1] = ki
        a_novo = a.copy()
        for j in range(1, i):
            a_novo[j] = a[j] + ki * a[i - j]
        a_novo[i] = ki
        a = a_novo
        erro *= (1 - ki * ki)
    return k


def _quantiza_k(k: np.ndarray) -> list[int]:
    idx = []
    for i in range(10):
        tab = np.array(TAB_K[i]) / ESCALA_K[i]
        idx.append(int(np.argmin(np.abs(tab - k[i]))))
    return idx


def _k_quantizado(idx: list[int]) -> np.ndarray:
    return np.array([TAB_K[i][idx[i]] / ESCALA_K[i] for i in range(10)])


def _rms_excitacao_unitaria(periodo_amostras: int) -> float:
    """RMS da excitação do decodificador por unidade de energia."""
    if periodo_amostras == 0:
        return 4.0
    c = np.array(CHIRP[:min(periodo_amostras, len(CHIRP))], dtype=np.float64) / 16.0
    return math.sqrt(np.sum(c * c) / periodo_amostras)


def _indice_energia(valor: float) -> int:
    if valor <= 0:
        return 0
    tab = np.array(TAB_ENERGIA[1:15], dtype=np.float64)
    i = int(np.argmin(np.abs(np.log(tab) - math.log(valor))))
    return i + 1


@dataclass
class Parametros:
    pitch_min_hz: float = 70.0
    pitch_max_hz: float = 320.0
    limiar_sonoro: float = 0.38      # correlação normalizada mínima p/ quadro sonoro
    limiar_silencio_db: float = -38  # quadros abaixo disto (rel. ao pico) viram silêncio
    pre_enfase: float = 0.9375
    limiar_repeticao: int = 3        # soma das diferenças de índice para reaproveitar os K
    usar_repeticao: bool = True
    clip_max: float = 0.006          # fração de amostras saturadas aceitável
    ganho_db: float = 0.0            # ganho extra após a calibração automática
    iteracoes_abs: int = 3


@dataclass
class AnaliseQuadro:
    sonoro: bool
    periodo_amostras: float
    k: np.ndarray
    rms_alvo: float         # RMS desejado (relativo) do sinal pré-enfatizado
    silencio: bool


def _detecta_pitch(seg: np.ndarray, lag_min: int, lag_max: int) -> tuple[float, float]:
    seg = seg - seg.mean()
    n = len(seg)
    if np.dot(seg, seg) < 1e-12:
        return 0.0, 0.0
    melhor_lag, melhor_r = 0, 0.0
    rs = np.zeros(lag_max + 2)
    for lag in range(lag_min - 1, min(lag_max + 2, n - 1)):
        a, b = seg[:n - lag], seg[lag:]
        den = math.sqrt(np.dot(a, a) * np.dot(b, b))
        rs[lag] = np.dot(a, b) / den if den > 0 else 0
    for lag in range(lag_min, min(lag_max + 1, n - 2)):
        if rs[lag] > melhor_r and rs[lag] >= rs[lag - 1] and rs[lag] >= rs[lag + 1]:
            melhor_r, melhor_lag = rs[lag], lag
    if melhor_lag == 0:
        return 0.0, 0.0
    # prefere o menor múltiplo (evita erro de oitava para baixo)
    for div in (2, 3):
        cand = int(round(melhor_lag / div))
        if cand >= lag_min:
            faixa = range(max(lag_min, cand - 2), min(lag_max, cand + 2) + 1)
            lc = max(faixa, key=lambda L: rs[L])
            if rs[lc] > 0.85 * melhor_r:
                melhor_lag, melhor_r = lc, rs[lc]
                break
    # interpolação parabólica
    y0, y1, y2 = rs[melhor_lag - 1], rs[melhor_lag], rs[melhor_lag + 1]
    den = (y0 - 2 * y1 + y2)
    desl = 0.5 * (y0 - y2) / den if den != 0 else 0.0
    return melhor_lag + float(np.clip(desl, -0.5, 0.5)), melhor_r


def analisa(x: np.ndarray, p: Parametros) -> list[AnaliseQuadro]:
    """x já na taxa do AVR, normalizado."""
    pre = np.append(x[0], x[1:] - p.pre_enfase * x[:-1])
    sos = signal.butter(4, 900, "lowpass", fs=TAXA, output="sos")
    xl = signal.sosfiltfilt(sos, x)
    nq = int(math.ceil(len(x) / QUADRO))
    pad = 400
    prep = np.pad(pre, (pad, pad + QUADRO))
    xlp = np.pad(xl, (pad, pad + QUADRO))
    janela_lpc = np.hamming(256)
    lag_min = max(16, int(TAXA / p.pitch_max_hz))
    lag_max = min(160, int(math.ceil(TAXA / p.pitch_min_hz)))
    n_pitch = max(320, 2 * lag_max + 40)
    lagwin = np.exp(-0.5 * (2 * np.pi * 40.0 * np.arange(11) / TAXA) ** 2)

    brutos = []
    for i in range(nq):
        c = (i + 1) * QUADRO - QUADRO // 4 + pad      # centro um pouco antes do fim do quadro
        seg = prep[c - 128: c + 128] * janela_lpc
        r = np.array([np.dot(seg[:256 - j], seg[j:]) for j in range(11)])
        r[0] *= 1.0001
        r *= lagwin
        k = _levinson(r) if r[0] > 1e-12 else np.zeros(10)
        rms = math.sqrt(np.mean(prep[c - QUADRO // 2: c + QUADRO // 2] ** 2))
        segp = xlp[c - n_pitch // 2: c + n_pitch // 2]
        lag, corr = _detecta_pitch(segp, lag_min, lag_max)
        brutos.append([k, rms, lag, corr])

    rms_all = np.array([b[1] for b in brutos])
    ref = rms_all.max() if len(rms_all) else 1.0
    lim_sil = ref * 10 ** (p.limiar_silencio_db / 20)
    sonoro = np.array([(b[3] >= p.limiar_sonoro and b[1] > lim_sil * 2) for b in brutos])
    # remove quadros sonoros/surdos isolados
    s2 = sonoro.copy()
    for i in range(1, len(sonoro) - 1):
        if sonoro[i - 1] == sonoro[i + 1] != sonoro[i]:
            s2[i] = sonoro[i - 1]
    sonoro = s2
    # suaviza o pitch (mediana de 3 entre quadros sonoros) e corrige oitava
    lags = np.array([b[2] for b in brutos])
    lag_s = lags.copy()
    for i in range(len(lags)):
        if not sonoro[i]:
            continue
        viz = [lags[j] for j in range(max(0, i - 2), min(len(lags), i + 3)) if sonoro[j] and lags[j] > 0]
        if len(viz) >= 3:
            med = float(np.median(viz))
            if lags[i] > 0 and (abs(lags[i] / med - 2) < 0.15 or abs(lags[i] / med - 0.5) < 0.08):
                lag_s[i] = med
            elif lags[i] == 0:
                lag_s[i] = med
    out = []
    for i, b in enumerate(brutos):
        sil = b[1] <= lim_sil
        v = bool(sonoro[i]) and lag_s[i] > 0 and not sil
        out.append(AnaliseQuadro(sonoro=v, periodo_amostras=float(lag_s[i]) if v else 0.0,
                                 k=b[0], rms_alvo=b[1] / ref, silencio=sil))
    return out


def _indice_periodo(lag: float) -> int:
    tab = np.array(TAB_PERIODO[1:], dtype=np.float64)
    return int(np.argmin(np.abs(np.log(tab) - math.log(lag)))) + 1


def monta_quadros(anal: list[AnaliseQuadro], ganho: float, p: Parametros,
                  correcao: np.ndarray | None = None) -> list[Quadro]:
    quadros = []
    ultimo_k = None
    ultimo_sonoro = None
    for i, a in enumerate(anal):
        if a.silencio:
            quadros.append(Quadro(energia=0))
            ultimo_k = None
            continue
        q = Quadro()
        q.periodo = _indice_periodo(a.periodo_amostras) if a.sonoro else 0
        kidx = _quantiza_k(a.k)
        kq = _k_quantizado(kidx)
        n = 10 if q.periodo else 4
        ganho_filtro = 1.0 / np.prod(1 - kq[:n] ** 2)
        per = TAB_PERIODO[q.periodo]
        exc = _rms_excitacao_unitaria(per)
        e = ganho * a.rms_alvo / (exc * math.sqrt(ganho_filtro))
        if correcao is not None:
            e *= correcao[i]
        q.energia = min(14, _indice_energia(e))
        if q.energia == 0:
            quadros.append(Quadro(energia=0))
            ultimo_k = None
            continue
        if (p.usar_repeticao and ultimo_k is not None and ultimo_sonoro == bool(q.periodo)):
            dif = [abs(kidx[j] - ultimo_k[j]) for j in range(n)]
            if sum(dif) <= p.limiar_repeticao and max(dif[:2]) <= 1:
                q.repete = True
                q.k = list(ultimo_k)
        if not q.repete:
            q.k = kidx
            ultimo_k = kidx
            ultimo_sonoro = bool(q.periodo)
        quadros.append(q)
    return quadros


def _rms_por_quadro(amostras: np.ndarray, nq: int) -> np.ndarray:
    a = amostras.astype(np.float64)
    out = np.zeros(nq)
    for i in range(nq):
        c = (i + 1) * QUADRO - QUADRO // 4
        seg = a[max(0, c - QUADRO // 2): c + QUADRO // 2]
        out[i] = math.sqrt(np.mean(seg ** 2)) if len(seg) else 0.0
    return out


@dataclass
class Resultado:
    dados: bytes
    quadros: list
    amostras: np.ndarray       # saída do decodificador (bit-exata)
    clip: float
    entrada: np.ndarray        # áudio de entrada na taxa do AVR (normalizado)


def codifica(x: np.ndarray, fs: int, p: Parametros | None = None, log=print) -> Resultado:
    """Áudio qualquer -> bitstream LPC calibrado."""
    p = p or Parametros()
    x = para_taxa_do_avr(x, fs)
    x = prepara(x, limiar_silencio_db=p.limiar_silencio_db - 2)
    anal = analisa(x, p)
    nq = len(anal)

    # 1) ganho global: o quadro mais forte deve encostar perto da saturação
    ganho = 700.0
    melhor = None
    for _ in range(10):
        quadros = monta_quadros(anal, ganho, p)
        dados = empacota(quadros)
        amostras = decodifica(dados)
        clip = float(np.mean(np.abs(amostras) >= SAIDA_MAX)) if len(amostras) else 0
        pico = int(np.max(np.abs(amostras))) if len(amostras) else 0
        melhor = (ganho, quadros, dados, amostras, clip)
        if clip > p.clip_max:
            ganho /= 1.15
        elif pico < 0.85 * SAIDA_MAX:
            ganho *= 1.2
        else:
            break

    ganho, quadros, dados, amostras, clip = melhor
    ganho *= 10 ** (p.ganho_db / 20)

    # 2) análise por síntese: corrige a energia quadro a quadro
    corr = np.ones(nq)
    alvo = np.array([a.rms_alvo for a in anal]) * ganho
    for _ in range(p.iteracoes_abs):
        quadros = monta_quadros(anal, ganho, p, corr)
        dados = empacota(quadros)
        amostras = decodifica(dados)
        obtido = _rms_por_quadro(amostras, nq)
        for i, q in enumerate(quadros):
            if q.energia and obtido[i] > 1 and alvo[i] > 0:
                razao = alvo[i] / obtido[i]
                corr[i] *= float(np.clip(razao, 0.6, 1.6)) ** 0.7
    quadros = monta_quadros(anal, ganho, p, corr)
    dados = empacota(quadros)
    amostras = decodifica(dados)
    clip = float(np.mean(np.abs(amostras) >= SAIDA_MAX)) if len(amostras) else 0
    return Resultado(dados=dados, quadros=quadros, amostras=amostras, clip=clip, entrada=x)


def estatisticas(r: Resultado) -> str:
    n = len(r.quadros)
    tipos = {"silêncio": 0, "repetição": 0, "surdo": 0, "sonoro": 0}
    for q in r.quadros:
        if q.energia == 0:
            tipos["silêncio"] += 1
        elif q.repete:
            tipos["repetição"] += 1
        elif q.periodo == 0:
            tipos["surdo"] += 1
        else:
            tipos["sonoro"] += 1
    dur = len(r.amostras) / TAXA
    t = ", ".join(f"{v} {k}" for k, v in tipos.items() if v)
    return (f"{len(r.dados)} bytes, {dur:.2f} s ({len(r.dados) / max(dur, 1e-9):.0f} B/s), "
            f"{n} quadros ({t}), saturação {100 * r.clip:.2f}%")


# ================================================================ simulação do buzzer
def simula_buzzer(x: np.ndarray, fs: float, ressonancia_hz: float = 2700.0, q: float = 2.0,
                  corte_grave_hz: float = 600.0) -> np.ndarray:
    """Aproximação grosseira de um buzzer piezo: passa-alta + pico na ressonância."""
    sos1 = signal.butter(2, corte_grave_hz, "highpass", fs=fs, output="sos")
    y = signal.sosfilt(sos1, x)
    b, a = signal.iirpeak(min(ressonancia_hz, fs / 2 * 0.95), q, fs=fs)
    y = 0.4 * y + signal.lfilter(b, a, y) * 1.5
    pico = np.max(np.abs(y)) or 1
    return y / pico * 0.9


def simula_buzzer_medido(x: np.ndarray, fs: float, curva_hz: np.ndarray, curva_db: np.ndarray) -> np.ndarray:
    """Usa uma curva de resposta medida (frequência x dB) como filtro FIR."""
    ntaps = 255
    freqs = np.linspace(0, fs / 2, 512)
    ganho = 10 ** (np.interp(freqs, curva_hz, curva_db, left=curva_db[0], right=curva_db[-1]) / 20)
    taps = signal.firwin2(ntaps, freqs / (fs / 2), ganho / ganho.max())
    y = signal.lfilter(taps, 1, x)
    pico = np.max(np.abs(y)) or 1
    return y / pico * 0.9
