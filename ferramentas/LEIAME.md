# Voz e música no buzzer do Paçoca

## Arquivos

| Arquivo | O que é |
|---|---|
| `../lpc.h` | Decodificador LPC em C puro, sem hardware. É o mesmo código no AVR e no teste do PC. |
| `../motorAudio.h` | Motor de áudio no Timer2 (PWM de 31,37 kHz e interrupções). |
| `../voz.h` | Classe `Voz` (só na porta S3). |
| `../musica.h` | Classe `Musica` (RTTTL, em qualquer porta; volume e timbre só na S3). |
| `../falas.h` | Falas geradas por `gerar_falas.py`. |
| `lpc_codec.py` | Codificador LPC e decodificador em Python, bit-exato com o `lpc.h`. |
| `gerar_falas.py` | Gera `falas.h` e as prévias: frases.txt → Piper → LPC. |
| `enviar_fala.py` | Manda uma fala para o brick pela serial, em tempo real. |
| `teste/` | Teste de bit-exatidão (Python × C compilado no PC). |

## Instalação (uma vez)

```
pip install -r requirements.txt
```

Na primeira execução, o `gerar_falas.py` baixa a voz do Piper (~60 MB) para `vozes/`.

## Gerar as falas

1. Edite o `frases.txt` (`NOME: texto`). Escreva como se fala: siglas e números por extenso.
2. Rode:
   ```
   python gerar_falas.py frases.txt
   ```
3. Ouça as prévias em `previas/`:
   - `NOME.wav`: exatamente as amostras que o AVR vai gerar.
   - `NOME_buzzer.wav`: o mesmo passado por um modelo grosseiro de piezo.
   - `NOME_tts.wav`: o que a TTS gerou, antes do LPC.
4. Compile o sketch. As falas ficam como `FALA_NOME` no `falas.h`.

Opções úteis:

| Opção | Para quê |
|---|---|
| `--voz pt_BR-cadu-medium` | Trocar a voz (outras: `pt_BR-faber-medium`, `pt_BR-jeff-medium`). |
| `--velocidade 1.4` | Fala mais devagar. As consoantes sobrevivem melhor ao LPC. |
| `--pitch-min 140 --pitch-max 350` | Faixa de pitch para voz feminina. O padrão (70–320 Hz) cobre a masculina. |
| `--limiar-sonoro 0.3` | Menos quadros de ruído. Ajuda se a fala sair "chiada". |
| `--sem-repeticao` | Arquivo um pouco maior, às vezes mais nítido. |
| `--motor espeak --voz pt-br` | Gerar sem o Piper. |

Para usar uma gravação própria em vez da TTS: `NOME: @gravacoes/arquivo.wav`.

## No sketch

```cpp
#define SUPORTE_VOZ 1
#define SUPORTE_MUSICA 1
#include "brickSimples.h"
#include "falas.h"

Voz voz = Voz(PORTA_BUZZER_3);
Musica musica = Musica(PORTA_BUZZER_3);

brick.adiciona(voz);
brick.adiciona(musica);
voz.falar(FALA_VITIMA_ENCONTRADA);   // bloqueia
voz.iniciaFala(FALA_OBSTACULO);      // não bloqueia; voz.falando() / voz.aguarda()
voz.volume(250);                     // 100 = limpo; acima disso corta picos e fica mais alto
musica.tocar(MUSICA_ODE_ALEGRIA);    // não bloqueia
```

Veja `../exemplos/vozMusica.ino`.

## Fala pela serial

1. No sketch, defina `SUPORTE_VOZ_SERIAL` (usa mais ~145 bytes de RAM).
2. Chame `voz.atualizaSerial(Serial)` no `loop()`.
3. No PC:
   ```
   python enviar_fala.py --porta COM5 "Qualquer frase"
   python enviar_fala.py --porta COM5 --bin previas/OLA.bin
   ```

O brick começa a falar quando chegam ~0,3 s de fala e continua enquanto o resto chega. Os pacotes têm soma de verificação, e o PC só manda o que cabe na FIFO.

Por padrão, o script tenta não reiniciar o brick ao abrir a porta. Se ele reiniciar mesmo assim, o script espera o brick responder (até 6 s).

## Números (medidos no simulador simavr, ATmega328 a 16 MHz)

| Item | Valor |
|---|---|
| Taxa de amostragem | 7843 Hz (16 MHz / 510 / 4) |
| Tamanho da fala | ~190–210 bytes por segundo |
| CPU livre enquanto fala | ~45% (decodificador ~800 ciclos por amostra) |
| CPU livre tocando música | ~65–75% |
| Voz | +4,4 KB de flash, +130 bytes de RAM (mais as falas) |
| Voz com `SUPORTE_VOZ_SERIAL` | +5,3 KB de flash, +275 bytes de RAM |
| Música | +2,4 KB de flash, +45 bytes de RAM |

## Conflitos

Usam o Timer2: um anula o outro enquanto o som toca.

- Os servos param de receber pulso durante o som e voltam sozinhos quando ele acaba.
- Voz e música não tocam juntas: começar uma para a outra.
- Código que desliga as interrupções por muito tempo causa estalos. Exemplos: fita de LED, leitura do giroscópio ou do sensor de linha dentro de `brick.atualiza()`, e `SoftwareSerial`. `voz.atrasos()` conta as amostras que atrasaram.

## Teste de bit-exatidão (opcional, precisa de gcc)

```
cd teste
python teste_bitexato.py ../previas/*_tts.wav
```
