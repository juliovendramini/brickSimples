/*
 * numerosPorExtenso.h - Quebra um número nas palavras que a voz fala.
 *
 * Usado por voz.falarNumero() (voz.h com SUPORTE_VOZ_NUMEROS). Não depende
 * do AVR, então também compila no PC para testar.
 *
 * Exemplo: 1234 -> mil, duzentos, e, trinta, e, quatro
 *          -45  -> menos, quarenta, e, cinco
 */
#ifndef NUMEROS_POR_EXTENSO_H
#define NUMEROS_POR_EXTENSO_H

#include <stdint.h>

/* Índices das palavras (mesma ordem da tabela em voz.h) */
enum {
    NUMERO_0 = 0,          /* 0..19: zero, um, dois ... dezenove */
    NUMERO_20 = 20,        /* 20..27: vinte, trinta ... noventa */
    NUMERO_CEM = 28,
    NUMERO_CENTO = 29,
    NUMERO_200 = 30,       /* 30..37: duzentos, trezentos ... novecentos */
    NUMERO_E = 38,
    NUMERO_MENOS = 39,
    NUMERO_MIL = 40,
    NUMERO_TOTAL_PALAVRAS = 41
};

/* O maior número de palavras: "menos trinta e dois mil setecentos e sessenta e sete" */
#define NUMERO_MAX_PALAVRAS 10

/* 1..999 */
static uint8_t numero_ateMil(uint16_t n, uint8_t *p) {
    uint8_t q = 0;
    if (n == 100) { p[q++] = NUMERO_CEM; return q; }
    uint8_t c = (uint8_t)(n / 100);
    uint8_t r = (uint8_t)(n % 100);
    if (c) {
        p[q++] = (c == 1) ? (uint8_t)NUMERO_CENTO : (uint8_t)(NUMERO_200 + c - 2);
        if (r) p[q++] = NUMERO_E;
    }
    if (r) {
        if (r < 20) p[q++] = r;
        else {
            p[q++] = (uint8_t)(NUMERO_20 + r / 10 - 2);
            if (r % 10) { p[q++] = NUMERO_E; p[q++] = (uint8_t)(r % 10); }
        }
    }
    return q;
}

/* Preenche palavras[] (pelo menos NUMERO_MAX_PALAVRAS) e devolve quantas são. */
static uint8_t numeroPorExtenso(int16_t valor, uint8_t *palavras) {
    uint8_t q = 0;
    uint16_t n;
    if (valor < 0) { palavras[q++] = NUMERO_MENOS; n = (uint16_t)(-(int32_t)valor); }
    else n = (uint16_t)valor;
    if (n == 0) { palavras[q++] = NUMERO_0; return q; }

    uint16_t milhares = n / 1000;
    uint16_t resto = n % 1000;
    if (milhares) {
        if (milhares > 1) q += numero_ateMil(milhares, palavras + q);   /* "mil", não "um mil" */
        palavras[q++] = NUMERO_MIL;
        /* "mil e vinte", "mil e quinhentos", mas "mil duzentos e trinta" */
        if (resto && (resto < 100 || resto % 100 == 0)) palavras[q++] = NUMERO_E;
    }
    if (resto) q += numero_ateMil(resto, palavras + q);
    return q;
}

#endif
