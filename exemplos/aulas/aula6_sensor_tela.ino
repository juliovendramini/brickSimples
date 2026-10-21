/*
 * Aula 6 - Sensor de distância, tela e servo (versão com tela)
 *
 * O que faz: lê a distância com o sensor VL53L0X, mostra na tela e usa o
 * servo de dois jeitos (modos):
 *   Modo ponteiro: o servo vira um medidor. Perto = 0 graus, longe = 180.
 *   Modo cancela: o servo abre (90 graus) quando algo chega mais perto
 *                 que o limite, e fecha (0 grau) quando vai embora.
 * Botões:
 *   Botão 1: diminui o limite (5 cm)
 *   Botão 2: aumenta o limite (5 cm)
 *   Botão 3: troca de modo
 *   Botão 4: congela/descongela a leitura na tela
 *
 * Ligações: sensor VL53L0X na porta I2C-1, servo na porta S1, tela na
 * porta I2C-5 e teclado na porta I2C-3.
 * Versão falada: aula6_sensor_voz.ino (mesma lógica).
 *
 * Com o sensor ultrassônico: troque SUPORTE_SENSOR_VL53L0X por
 * SUPORTE_SENSOR_ULTRASSONICO, o objeto por
 * Ultrassonico sensor = Ultrassonico(PORTA_ULTRASSONICO_1); e a leitura por
 * sensor.getDistancia() * 10 (ele já mede em centímetros).
 */
#define SUPORTE_DISPLAY_SSD1306 1
#define SUPORTE_TECLADO 1
#define SUPORTE_SERVO 1
#define SUPORTE_SENSOR_VL53L0X 1
#include "brickSimples.h"

SSD1306 tela = SSD1306(PORTA_I2C_5);
Teclado teclado = Teclado(PORTA_I2C_3);
VL53L0X sensor = VL53L0X(PORTA_I2C_1);

const int MODO_PONTEIRO = 0;
const int MODO_CANCELA = 1;
int modo = MODO_PONTEIRO;
int limite = 15;                  /* cm: mais perto que isto abre a cancela */
bool congelado = false;
bool cancelaAberta = false;
int anguloServo = -1;             /* -1 = ainda não moveu */

/* Mesma função da aula 2: true só no momento em que o botão é apertado. */
uint8_t estadoAnterior[5] = {Teclado::LIBERADO, Teclado::LIBERADO, Teclado::LIBERADO,
                             Teclado::LIBERADO, Teclado::LIBERADO};
uint32_t tempoMudanca[5] = {0, 0, 0, 0, 0};

bool apertou(uint8_t botao) {
    uint8_t agora = teclado.leBotao(botao);
    if (agora == estadoAnterior[botao]) return false;
    if (millis() - tempoMudanca[botao] < 30) return false;
    estadoAnterior[botao] = agora;
    tempoMudanca[botao] = millis();
    return agora == Teclado::APERTADO;
}

/* Distância em cm. Devolve -1 se não tem nada na frente (mais de 120 cm). */
int leDistancia() {
    uint16_t mm = sensor.getDistancia();     /* sem nada na frente ele devolve 8190 ou mais */
    if (mm > 1200) return -1;
    return mm / 10;
}

/* Só manda o servo se o ângulo mudou pelo menos 3 graus (evita tremedeira). */
void moveServo(int angulo) {
    if (anguloServo < 0 || abs(angulo - anguloServo) >= 3) {
        anguloServo = angulo;
        servos.moveServo(PORTA_SERVO_1, angulo);
    }
}

void desenhaFixos() {
    tela.setCursor(0, 0);
    if (modo == MODO_PONTEIRO) tela.print("Ponteiro  ");
    else tela.print("Cancela   ");
    tela.setCursor(0, 2);
    tela.print("Lim ");
    tela.print(limite);
    tela.print(" cm  ");
    tela.setCursor(0, 3);
    tela.print("       ");            /* apaga o "ABERTA"/"fechada" do modo cancela */
    if (congelado) {
        tela.setCursor(9, 0);
        tela.print("*");
    }
}

void setup() {
    brick.inicializa();
    brick.adiciona(tela);
    brick.adiciona(teclado);
    brick.adiciona(sensor);
    servos.iniciaServo(PORTA_SERVO_1);
    tela.setFonte(SSD1306::FONTE_MEDIA);
    desenhaFixos();
}

void loop() {
    brick.atualiza();                     /* aqui o brick lê o sensor */

    int distancia = leDistancia();

    /* Botões */
    if (apertou(1)) {
        limite = max(5, limite - 5);
        desenhaFixos();
    }
    if (apertou(2)) {
        limite = min(100, limite + 5);
        desenhaFixos();
    }
    if (apertou(3)) {
        modo = (modo == MODO_PONTEIRO) ? MODO_CANCELA : MODO_PONTEIRO;
        anguloServo = -1;                 /* força mover o servo no modo novo */
        cancelaAberta = false;            /* todo modo começa com a cancela fechada */
        desenhaFixos();
    }
    if (apertou(4)) {
        congelado = !congelado;
        tela.setCursor(9, 0);
        tela.print(congelado ? "*" : " ");
    }

    /* Tela */
    if (!congelado) {
        tela.setCursor(0, 1);
        if (distancia < 0) tela.print("longe     ");
        else {
            tela.print(distancia);
            tela.print(" cm     ");
        }
    }

    /* Servo */
    if (modo == MODO_PONTEIRO) {
        int d = (distancia < 0) ? 100 : distancia;
        d = constrain(d, 0, 100);
        moveServo(map(d, 0, 100, 0, 180));        /* 0..100 cm vira 0..180 graus */
    } else {
        /* Histerese: abre abaixo do limite, só fecha 3 cm acima dele.
         * Sem isso, um objeto parado bem no limite faz a cancela abrir e
         * fechar sem parar. */
        if (!cancelaAberta && distancia >= 0 && distancia < limite) cancelaAberta = true;
        if (cancelaAberta && (distancia < 0 || distancia > limite + 3)) cancelaAberta = false;
        moveServo(cancelaAberta ? 90 : 0);
        tela.setCursor(0, 3);
        tela.print(cancelaAberta ? "ABERTA " : "fechada");
    }
}

/*
 * Experimente:
 * 1. No modo ponteiro, mude a escala para 0..50 cm. O ponteiro fica mais
 *    sensível ou menos?
 * 2. No modo cancela, tire a histerese (use só "distancia < limite").
 *    Coloque a mão bem no limite. O que acontece?
 * 3. Por que o servo só é mandado quando o ângulo muda 3 graus ou mais?
 * 4. Desafio: crie um terceiro modo, "radar": o servo varre de 0 a 180 e
 *    para quando encontra algo mais perto que o limite.
 */
