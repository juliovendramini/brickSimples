/* Teste de bit-exatidão: compila ../../lpc.h no PC e decodifica um .bin.
 * gcc -O2 -o decodifica_pc decodifica_pc.c && ./decodifica_pc fala.bin saida.raw
 * saida.raw = amostras int16 little-endian, uma por amostra. */
#include <stdio.h>
#include <stdlib.h>
#include "../../lpc.h"

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "uso: %s fala.bin saida.raw\n", argv[0]); return 1; }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    static uint8_t dados[65536];
    size_t n = fread(dados, 1, sizeof dados, f);
    fclose(f);
    LpcDecodificador d;
    lpc_iniciaRam(&d, dados, (uint16_t)n);
    FILE *o = fopen(argv[2], "wb");
    long total = 0;
    for (;;) {
        int16_t a = lpc_proximaAmostra(&d);
        if (!d.ativo) break;
        fwrite(&a, 2, 1, o);
        total++;
    }
    fclose(o);
    printf("%ld amostras\n", total);
    return 0;
}
