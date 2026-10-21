/*
 * voz.h - Fala sintetizada por LPC no buzzer passivo do Paçoca.
 *
 * Só funciona na porta S3 (D11 / OC2A / MOSI): o áudio sai pelo PWM do Timer2.
 *
 * As falas são geradas no PC com ferramentas/gerar_falas.py, que cria o
 * arquivo falas.h com vetores em PROGMEM (~150-250 bytes por segundo de fala).
 *
 * Uso:
 *   #define SUPORTE_VOZ 1
 *   #define SUPORTE_VOZ_SERIAL 1      // opcional: receber fala pela serial (+145 B de RAM)
 *   #include "brickSimples.h"
 *   #include "falas.h"
 *
 *   Voz voz = Voz(PORTA_BUZZER_3);
 *   brick.adiciona(voz);
 *   voz.falar(FALA_VITIMA_ENCONTRADA);       // espera terminar
 *   voz.iniciaFala(FALA_OBSTACULO);          // volta na hora, fala em segundo plano
 *   voz.enfileirar(FALA_SALA_TRES);          // fala depois da que está tocando
 *
 * Números por extenso (precisa do falasNumeros.h, gerado de ferramentas/numeros.txt):
 *   #define SUPORTE_VOZ_NUMEROS 1
 *   voz.falarNumero(123);                    // "cento e vinte e três", espera terminar
 *   voz.iniciaFala(FALA_NIVEL);              // "Nível ...
 *   voz.enfileirarNumero(3);                 //  ... três" (volta na hora)
 *   O falasNumeros.h ocupa ~6 KB de flash (41 palavras, de zero a mil).
 *
 * Enquanto fala: ~40% da CPU vai para o decodificador, servos ficam sem pulso
 * e o Timer2 fica ocupado (veja motorAudio.h).
 */
#ifndef VOZ_H
#define VOZ_H

#include "motorAudio.h"

#ifdef SUPORTE_VOZ_NUMEROS
#include "numerosPorExtenso.h"
#include "falasNumeros.h"
/* mesma ordem do enum de numerosPorExtenso.h */
static const uint8_t * const voz_palavrasNumero[NUMERO_TOTAL_PALAVRAS] PROGMEM = {
    FALA_NUM_0, FALA_NUM_1, FALA_NUM_2, FALA_NUM_3, FALA_NUM_4, FALA_NUM_5, FALA_NUM_6,
    FALA_NUM_7, FALA_NUM_8, FALA_NUM_9, FALA_NUM_10, FALA_NUM_11, FALA_NUM_12, FALA_NUM_13,
    FALA_NUM_14, FALA_NUM_15, FALA_NUM_16, FALA_NUM_17, FALA_NUM_18, FALA_NUM_19,
    FALA_NUM_20, FALA_NUM_30, FALA_NUM_40, FALA_NUM_50, FALA_NUM_60, FALA_NUM_70,
    FALA_NUM_80, FALA_NUM_90,
    FALA_NUM_100, FALA_NUM_CENTO,
    FALA_NUM_200, FALA_NUM_300, FALA_NUM_400, FALA_NUM_500, FALA_NUM_600, FALA_NUM_700,
    FALA_NUM_800, FALA_NUM_900,
    FALA_NUM_E, FALA_NUM_MENOS, FALA_NUM_MIL
};
#endif

class Voz {
private:
    uint8_t pino;
    bool invertido;
    bool inicializado;
    char descricaoPorta[6];

#ifdef SUPORTE_VOZ_SERIAL
    /* recepção de pacotes: A5 tipo tamanho dados... soma */
    uint8_t rxEstado = 0, rxTipo = 0, rxTam = 0, rxCont = 0, rxSoma = 0;
    uint8_t rxPosicao = 0;          /* onde os dados estão sendo escritos (ainda não confirmados) */
    uint8_t rxErro = 0;
    uint32_t rxUltimoMs = 0;
    bool streamAtivo = false;
#endif

    bool podeFalar() {
        if (!inicializado) { Serial.println(F("Erro: Voz nao inicializada!")); return false; }
        if (pino != AUDIO_PINO_OC2A) return false;
        return true;
    }

public:
    Voz(PortaBuzzer porta, bool invertido = true) {
        pino = porta.pino;
        this->invertido = invertido;
        inicializado = false;
        strcpy(descricaoPorta, porta.descricao);
    }

    void inicializa() {
        if (pino != AUDIO_PINO_OC2A) {
            Serial.print(F("Erro: a voz so funciona na porta S3 (PWM do Timer2). Porta usada: "));
            Serial.println(descricaoPorta);
            return;
        }
        audio_configuraPino(pino, invertido);
        inicializado = true;
        Serial.print(F("Voz inicializada na porta "));
        Serial.println(descricaoPorta);
    }

    /* Fala gravada na flash (vetor PROGMEM do falas.h) e espera terminar. */
    void falar(const uint8_t *fala) {
        iniciaFala(fala);
        aguarda();
    }

    /* Começa a falar e volta na hora. Para o que estiver falando antes
     * (e esvazia a fila). */
    void iniciaFala(const uint8_t *fala) {
        if (!podeFalar()) return;
        audio_configuraPino(pino, invertido);
        voz_filaLe = voz_filaEscreve = 0;
        lpc_iniciaFlash(&voz_dec, fala);
        voz_inicia();
    }

    /* Coloca uma fala na fila: ela toca logo depois da que está tocando
     * (ou na hora, se a voz estiver quieta). Volta na hora.
     * Ex.: voz.iniciaFala(FALA_BOTAO); voz.enfileirar(FALA_NUM_3); */
    void enfileirar(const uint8_t *fala) {
        if (!podeFalar()) return;
        uint8_t sreg = SREG;
        cli();
        uint8_t est = voz_estado;
        if (voz_ativa() && (est == VOZ_SUBINDO || est == VOZ_FALANDO || est == VOZ_PAUSA)) {
            /* a interrupção ainda vai olhar a fila: basta colocar no fim */
            uint8_t e = voz_filaEscreve;
            uint8_t prox = (uint8_t)((e + 1) & VOZ_FILA_MASC);
            if (prox != voz_filaLe) {                 /* fila cheia: descarta */
                voz_fila[e] = fala;
                __asm__ __volatile__("" ::: "memory");  /* grava a fala antes de liberar */
                voz_filaEscreve = prox;
            }
            SREG = sreg;
            return;
        }
        SREG = sreg;
        aguarda();                 /* se estava terminando (rampa de ~8 ms), espera */
        iniciaFala(fala);
    }

    /* Silêncio entre as falas da fila, em ms (padrão 50). */
    void pausaEntreFalas(uint16_t ms) {
        uint32_t a = (uint32_t)ms * 7843UL / 1000UL;
        voz_pausaAmostras = (a > 65535UL) ? 65535U : (uint16_t)a;
    }

#ifdef SUPORTE_VOZ_NUMEROS
    /* Fala o número (de -32768 a 32767) e espera terminar. */
    void falarNumero(int16_t numero) {
        iniciaNumero(numero);
        aguarda();
    }
    /* Começa a falar o número e volta na hora (para o que estava falando). */
    void iniciaNumero(int16_t numero) {
        parar();
        enfileirarNumero(numero);
    }
    /* Põe as palavras do número na fila, depois do que estiver falando. */
    void enfileirarNumero(int16_t numero) {
        uint8_t palavras[NUMERO_MAX_PALAVRAS];
        uint8_t n = numeroPorExtenso(numero, palavras);
        for (uint8_t i = 0; i < n; i++)
            enfileirar((const uint8_t *)pgm_read_ptr(&voz_palavrasNumero[palavras[i]]));
    }
#endif

    /* Fala que está num vetor em RAM (ex.: recebida inteira pela serial). */
    void falarRam(const uint8_t *dados, uint16_t tamanho) {
        iniciaFalaRam(dados, tamanho);
        aguarda();
    }
    void iniciaFalaRam(const uint8_t *dados, uint16_t tamanho) {
        if (!podeFalar()) return;
        audio_configuraPino(pino, invertido);
        voz_filaLe = voz_filaEscreve = 0;
        lpc_iniciaRam(&voz_dec, dados, tamanho);
        voz_inicia();
    }

    bool falando() { return voz_ativa(); }

    void aguarda() { while (voz_ativa()) { } }

    void parar() {
        if (voz_ativa()) {
            voz_estado = VOZ_PARADA;
            audio_libera();
        }
        voz_filaLe = voz_filaEscreve = 0;
    }

    /* Volume em %: 100 = sem distorção. Acima de 100 os picos são cortados,
     * o que deixa a fala mais alta (bom para buzzer). Padrão 200, máximo 400. */
    void volume(uint16_t porcento) {
        if (porcento > 400) porcento = 400;
        uint16_t g = (porcento * 64UL + 50) / 100;
        voz_ganho = (g > 255) ? 255 : (uint8_t)g;
    }

    /* Quantas amostras não ficaram prontas a tempo na última fala (diagnóstico:
     * deve ser 0; se não for, algo está desligando as interrupções). */
    uint16_t atrasos() { return voz_atrasos; }

#ifdef SUPORTE_VOZ_SERIAL
    /*
     * Recebe fala pela serial (protocolo do ferramentas/enviar_fala.py).
     * Chame sempre no loop (não bloqueia). Pacote:
     *   0xA5, tipo, tamanho, dados[tamanho], soma   (soma = tipo+tamanho+dados, 8 bits)
     *   tipo 'D' = dados de fala, 'F' = fim da fala, 'P' = parar, 'S' = só status
     * Resposta a cada pacote: 0x5A, código, bytes livres, estado, verificação
     *   verificação = (código + livres + estado) XOR 0xA5
     *   código 0 = ok, 1 = soma errada, 2 = sem espaço, 3 = tipo desconhecido
     *   estado bit0 = falando
     */
    void atualizaSerial(Stream &s) {
        while (s.available()) {
            uint8_t b = (uint8_t)s.read();
            rxUltimoMs = millis();
            switch (rxEstado) {
            case 0:
                if (b == 0xA5) rxEstado = 1;
                break;
            case 1:
                rxTipo = b; rxSoma = b; rxEstado = 2;
                break;
            case 2:
                rxTam = b; rxSoma += b; rxCont = 0; rxErro = 0;
                if (rxTipo == 'D') {
                    if (!streamAtivo) comecaStream();
                    rxPosicao = voz_fifo.escreve;
                }
                rxEstado = (rxTam == 0) ? 4 : 3;
                break;
            case 3:
                rxSoma += b;
                if (rxTipo == 'D') {
                    /* escreve sem confirmar: só vale se a soma bater */
                    uint8_t prox = (uint8_t)((rxPosicao + 1) & (LPC_TAMANHO_FIFO - 1));
                    if (prox == voz_fifo.le) rxErro = 2;
                    else { voz_fifo.buf[rxPosicao] = b; rxPosicao = prox; }
                }
                if (++rxCont >= rxTam) rxEstado = 4;
                break;
            case 4: {
                uint8_t codigo = 0;
                rxEstado = 0;
                if (b != rxSoma) codigo = 1;
                else if (rxErro) codigo = rxErro;
                else if (rxTipo == 'D') {
                    voz_fifo.escreve = rxPosicao;              /* confirma */
                    /* começa a tocar quando tiver ~0,3 s guardado */
                    if (!voz_ativa() && streamAtivo && lpc_fifoQuantidade(&voz_fifo) >= 64) iniciaStream();
                } else if (rxTipo == 'F') {
                    voz_fifo.terminou = 1;
                    if (!voz_ativa() && streamAtivo && lpc_fifoQuantidade(&voz_fifo) > 0) iniciaStream();
                    streamAtivo = false;
                } else if (rxTipo == 'P') {
                    parar();
                    streamAtivo = false;
                    lpc_fifoLimpa(&voz_fifo);
                } else if (rxTipo != 'S') codigo = 3;
                uint8_t livre = lpc_fifoLivre(&voz_fifo);
                uint8_t estado = voz_ativa() ? 1 : 0;
                s.write((uint8_t)0x5A);
                s.write(codigo);
                s.write(livre);
                s.write(estado);
                s.write((uint8_t)((codigo + livre + estado) ^ 0xA5));
                break;
            }
            }
        }
        /* pacote pela metade há muito tempo: descarta */
        if (rxEstado != 0 && (millis() - rxUltimoMs) > 200) rxEstado = 0;
    }

    /* Fica recebendo e falando até a fala terminar ou passar esperaMaxMs sem
     * chegar nada. Retorna true se falou alguma coisa. */
    bool falarDaSerial(Stream &s, uint16_t esperaMaxMs = 3000) {
        bool falou = false;
        uint32_t t0 = millis();
        rxUltimoMs = t0;
        for (;;) {
            atualizaSerial(s);
            if (voz_ativa()) falou = true;
            if (falou && !voz_ativa() && !streamAtivo) return true;
            if ((millis() - rxUltimoMs) > esperaMaxMs) {
                if (streamAtivo) { voz_fifo.terminou = 1; streamAtivo = false; }
                if (!voz_ativa()) return falou;
            }
        }
        (void)t0;
    }

private:
    void comecaStream() {
        parar();
        lpc_fifoLimpa(&voz_fifo);
        streamAtivo = true;
    }
    void iniciaStream() {
        if (!podeFalar()) return;
        audio_configuraPino(pino, invertido);
        voz_filaLe = voz_filaEscreve = 0;
        lpc_iniciaFifo(&voz_dec, &voz_fifo);
        voz_inicia();
    }
#endif
};

#endif /* VOZ_H */
