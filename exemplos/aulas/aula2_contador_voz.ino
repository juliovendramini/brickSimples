/*
 * Aula 2 - Contador: apertar é diferente de segurar (versão com voz)
 *
 * O que faz: um contador de 0 a 255 controlado pelo teclado.
 *   Botão 1: diminui 1
 *   Botão 2: aumenta 1
 *   Botão 3: volta para zero
 *   Botão 4: fala o número em binário (zeros e uns)
 * A cada mudança o Paçoca fala o novo valor. Se você apertar de novo
 * enquanto ele fala, ele para e fala o valor novo (como um leitor de tela).
 * Os 4 LEDs do teclado mostram os 4 últimos bits do número.
 *
 * Ligações: buzzer passivo na porta S3 e teclado na porta I2C-3.
 * Versão com tela: aula2_contador_tela.ino (mesma lógica).
 *
 * Ideia principal: o loop roda dezenas de vezes por segundo. Se a gente
 * somasse 1 sempre que o botão está APERTADO, um toque rápido somaria
 * vários. Por isso contamos só no momento em que o botão PASSA de solto
 * para apertado (a "borda"). Quem faz isso é a função apertou().
 */
#define SUPORTE_VOZ 1
#define SUPORTE_VOZ_NUMEROS 1
#define SUPORTE_TECLADO 1
#include "brickSimples.h"
#include "falasAulas.h"

Voz voz = Voz(PORTA_BUZZER_3);
Teclado teclado = Teclado(PORTA_I2C_3);

int contador = 0;

/*
 * apertou(botao) devolve true só uma vez a cada toque.
 * Guardamos como o botão estava na última vez (estadoAnterior) e só
 * respondemos quando ele estava solto e agora está apertado.
 * Também ignoramos mudanças muito rápidas (menos de 30 ms): o contato do
 * botão "pula" algumas vezes quando encosta (isso se chama bounce).
 */
uint8_t estadoAnterior[5] = {Teclado::LIBERADO, Teclado::LIBERADO, Teclado::LIBERADO,
                             Teclado::LIBERADO, Teclado::LIBERADO};
uint32_t tempoMudanca[5] = {0, 0, 0, 0, 0};

bool apertou(uint8_t botao) {
    uint8_t agora = teclado.leBotao(botao);
    if (agora == estadoAnterior[botao]) return false;          /* nada mudou */
    if (millis() - tempoMudanca[botao] < 30) return false;     /* bounce: ignora */
    estadoAnterior[botao] = agora;
    tempoMudanca[botao] = millis();
    return agora == Teclado::APERTADO;                         /* true só ao apertar */
}

void mostraContador() {
    /* iniciaNumero() começa a falar e volta na hora, sem esperar.
     * Se já estava falando, ele para e começa o número novo. */
    voz.iniciaNumero(contador);

    /* LEDs: os 4 bits mais baixos. LED 4 = bit 0, LED 1 = bit 3 */
    for (int bit = 0; bit < 4; bit++) {
        teclado.alteraLed(4 - bit, (contador >> bit) & 1);
    }
}

void falaBinario() {
    voz.parar();
    /* do bit 7 (o que vale 128) até o bit 0 (o que vale 1),
     * pulando os zeros da frente (5 em binário é 101, não 00000101) */
    bool comecou = false;
    for (int bit = 7; bit >= 0; bit--) {
        int valor = (contador >> bit) & 1;
        if (valor == 1) comecou = true;
        if (comecou || bit == 0) {
            voz.enfileirarNumero(valor);     /* põe "um" ou "zero" na fila */
        }
    }
}

void setup() {
    brick.inicializa();
    brick.adiciona(voz);
    brick.adiciona(teclado);
    voz.falar(FALA_APERTE_UM_BOTAO);
    mostraContador();
}

void loop() {
    brick.atualiza();

    if (apertou(1)) {
        if (contador > 0) {
            contador--;
            mostraContador();
        } else {
            voz.iniciaFala(FALA_MINIMO);     /* "Mínimo." */
        }
    }
    if (apertou(2)) {
        if (contador < 255) {
            contador++;
            mostraContador();
        } else {
            voz.iniciaFala(FALA_MAXIMO);     /* "Máximo." */
        }
    }
    if (apertou(3)) {
        contador = 0;
        mostraContador();
    }
    if (apertou(4)) {
        falaBinario();
    }
}

/*
 * Experimente:
 * 1. Troque apertou(2) por teclado.leBotao(2) == Teclado::APERTADO. Dê um
 *    toque rápido no botão 2. Quanto o contador sobe? Por quê?
 * 2. Troque voz.iniciaNumero por voz.falarNumero (que espera terminar).
 *    Aperte o botão 2 várias vezes bem rápido. O que muda?
 * 3. Na versão com tela, o botão 1 no zero não avisa nada. Aqui ele fala
 *    "Mínimo". Qual das duas é melhor para quem usa?
 * 4. Desafio: segurando o botão 2 por mais de meio segundo, o contador
 *    passa a subir sozinho e o Paçoca fala só quando você soltar.
 * 5. Desafio: no botão 4, depois do binário, fale também o número em
 *    decimal ("um, zero, um ... cinco"). Dica: voz.enfileirarNumero(contador).
 */
