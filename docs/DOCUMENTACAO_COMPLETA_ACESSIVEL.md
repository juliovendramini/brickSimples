# Documentacao Completa da Biblioteca brickSimples

Este documento foi escrito para explicar toda a biblioteca de forma linear, com frases claras e previsiveis, pensando em uso com leitor de tela.

Se voce usa leitor de tela, leia na ordem das secoes numeradas. Cada secao foi feita para funcionar de forma independente.

## 1. O que esta biblioteca faz

A biblioteca `brickSimples` e um conjunto de classes para um robo educacional baseado em ATmega328PB (compatibilidade estilo Arduino Nano).

Ela permite controlar:

- motores DC
- servos
- buzzer
- fita de LED RGB WS2812B
- sensores de distancia (ultrassonico e VL53L0X)
- sensor de cor TCS34725
- IMU BMI160
- sensor de linha por serial
- display OLED SSD1306
- teclado I2C (PCF8574A)
- modulo Bluetooth serial

## 2. Como a biblioteca e organizada

Arquivos de nucleo:

- `portas.h`: definicao das portas fisicas e estruturas de configuracao
- `brickSimples.h`: classe principal `BrickSimples`, classe `Motor` e classe auxiliar `Servos`
- `interrupcoes.h`: interrupcoes de pino (PCINT) para sensores e Bluetooth

Arquivos de comunicacao:

- `SoftWire.h` e `SoftWire.cpp`: I2C por software
- `SoftwareSerial.h` e `SoftwareSerial.cpp`: serial por software
- `AsyncDelay.h`: temporizador sem bloqueio

Arquivos de sensores:

- `ultrassonico.h`
- `VL53L0X.h` e `VL53L0X.cpp`
- `TCS34725.h` e `TCS34725.cpp`
- `BMI160.h` e `BMI160.cpp`
- `sensorLinha.h`
- `giroscopio.h`

Arquivos de atuadores e interface:

- `Servo.h`, `Servo.cpp`, `ServoTimers.h`
- `led.h`
- `buzzer.h`
- `SSD1306.h` e `SSD1306.cpp`
- `Teclado.h` e `Teclado.cpp`
- `Bluetooth.h`

Outros arquivos:

- `brickSimples.ino`: sketch de teste no diretorio raiz
- `exemplos/brickSimples.ino`: exemplo mais completo
- `SOFTWIRE_OTIMIZACOES.md`: notas tecnicas sobre SoftWire
- `README.md`: entrada principal
- `328p16mhz-combootloader.bat`: script utilitario de gravacao
- `seguidor.h.txt` e `eiaextensions.h`: arquivos auxiliares/legado

## 3. Regras obrigatorias de uso

Estas duas linhas sao obrigatorias:

1. no inicio de `setup()`: `brick.inicializa();`
2. no inicio de `loop()`: `brick.atualiza();`

Se essas linhas faltarem, o funcionamento da placa, leitura de sensores e seguranca de energia podem ficar incorretos.

## 4. Flags de compilacao (`SUPORTE_*`)

A biblioteca usa compilacao condicional.

Voce ativa modulos no seu sketch com `#define` antes de incluir `brickSimples.h`.

Exemplo:

```cpp
#define SUPORTE_MOTOR 1
#define SUPORTE_SENSOR_LINHA 1
#include "brickSimples.h"
```

Flags comuns:

- `SUPORTE_MOTOR`
- `SUPORTE_SERVO`
- `SUPORTE_LED`
- `SUPORTE_BUZZER`
- `SUPORTE_BLUETOOTH`
- `SUPORTE_TECLADO`
- `SUPORTE_DISPLAY_SSD1306`
- `SUPORTE_SENSOR_ULTRASSONICO`
- `SUPORTE_SENSOR_VL53L0X`
- `SUPORTE_SENSOR_TCS34725`
- `SUPORTE_SENSOR_BMI160`
- `SUPORTE_SENSOR_GIROSCOPIO`
- `SUPORTE_SENSOR_LINHA`

## 5. Mapeamento de portas (`portas.h`)

As estruturas principais sao:

- `PortaI2C`
- `PortaServo`
- `PortaLed`
- `PortaBuzzer`
- `PortaUltrassonico`
- `PortaSerial`
- `PortaMotor`

Macros prontas para uso:

- I2C: `PORTA_I2C_1` ate `PORTA_I2C_5`
- Serial: `PORTA_SERIAL_1` ate `PORTA_SERIAL_5`
- Servo: `PORTA_SERVO_1` ate `PORTA_SERVO_4`
- LED: `PORTA_LED_1` ate `PORTA_LED_4`
- Buzzer: `PORTA_BUZZER_1` ate `PORTA_BUZZER_4`
- Ultrassonico: `PORTA_ULTRASSONICO_1` ate `PORTA_ULTRASSONICO_5`
- Motor: `PORTA_MOTOR_1` e `PORTA_MOTOR_2`

Constantes importantes:

- `PINO_BATERIA` (A7)
- `MOTOR_INVERTIDO`
- `MOTOR_NORMAL`

## 6. Classe principal `BrickSimples`

A biblioteca expone um objeto global:

- `brick`

### 6.1 Metodos de inicializacao e ciclo

- `void inicializa()`
- `void atualiza()`
- `void espera(uint32_t ms)`
- `uint32_t millis()`

### 6.2 Metodos gerais

- `bool botaoApertado()`
- `void ativaLedInterno()`
- `void desativaLedInterno()`
- `void reset()`

### 6.3 Motores (quando `SUPORTE_MOTOR`)

- `void setPotenciaPadrao(int potencia)`
- `int getPotenciaPadrao()`
- `void potenciaMotores()`
- `void potenciaMotores(int potencia)`
- `void potenciaMotores(int pot1, int pot2)`
- `void acionaMotoresPorTempo(unsigned long tempoMs)`
- `void acionaMotoresPorTempo(int potencia, unsigned long tempoMs)`
- `void acionaMotoresPorTempo(int potencia1, int potencia2, unsigned long tempoMs)`
- `void gireMotoresPorTempo(int pot1, int pot2, unsigned long tempoMs)`
- `void andarPraFrente(int potencia, bool giroscopioInvertido=false)`
- `void pararMotores()`
- `void frearMotores()`

### 6.4 Cadastro de dispositivos

A classe usa sobrecarga de `adiciona(...)` para registrar componentes.

Exemplos:

- `brick.adiciona(Motor1, Motor2);`
- `brick.adiciona(sensorLinha);`
- `brick.adiciona(display);`

Existem limites internos:

- ate 2 motores
- ate 4 portas servo/led/buzzer
- ate 5 sensores por tipo em vetores
- 1 instancia para alguns componentes (exemplo: BMI160, display, teclado, Bluetooth)

## 7. Classe `Motor` (em `brickSimples.h`)

Construtor:

- `Motor(PortaMotor porta, bool invertido=false)`

Metodos:

- `void setPotenciaPadrao(int potencia)`
- `int getPotenciaPadrao()`
- `void potencia()`
- `void potencia(int potencia)`
- `void acionaPorTempo(unsigned long tempoMs)`
- `void acionaPorTempo(int potencia, unsigned long tempoMs)`
- `void frear()`
- `void parar()`
- `void setInvertido(bool invertido)`

Faixa de potencia:

- de -100 ate 100
- positivo para frente
- negativo para tras

## 8. Classe `Servos` e classe `Servo`

Quando `SUPORTE_SERVO` esta ativo:

- objeto global `servos` (classe `Servos`) em `brickSimples.h`

Metodos de `Servos`:

- `void iniciaServo(PortaServo porta)`
- `void moveServo(PortaServo porta, int angulo)`
- `void moveServoTempo(PortaServo porta, int anguloDestino, unsigned long tempoMs)`
- `void desanexaServo(PortaServo porta)`

Classe `Servo` (arquivos `Servo.h` e `Servo.cpp`) usa Timer2 interno.

Metodos principais de `Servo`:

- `attach(...)`
- `detach()`
- `write(...)`
- `read()`
- `attached()`

## 9. Comunicacao serial e I2C

### 9.1 `SoftWire` (I2C software)

Use quando um sensor I2C usa as portas definidas no brick.

Metodos utilitarios principais:

- `begin()`
- `beginTransmission(...)`
- `endTransmission(...)`
- `requestFrom(...)`
- `setTimeout_ms(...)`
- `setClock(...)`

### 9.2 `SoftwareSerial`

Metodos principais:

- `begin(long speed)`
- `end()`
- `available()`
- `read()`
- `write(...)`
- `readBytes(...)`
- `setTimeout(...)`

### 9.3 `Bluetooth`

Classe baseada em `Stream`, com taxa fixa de 9600 no `begin()`.

Metodos principais:

- `begin()`
- `end()`
- `available()`
- `read()`
- `write(...)`
- `listen()`
- `stopListening()`

## 10. Sensores

### 10.1 `Ultrassonico` (`ultrassonico.h`)

Metodos:

- `inicializa()`
- `iniciaMedicao()`
- `medicaoPronta()`
- `lerPulso()`
- `lerPulsoUltimo()`
- `getDistancia()`

Notas:

- usa interrupcoes de mudanca de pino (PCINT)
- timeout interno para evitar travamento de leitura

### 10.2 `VL53L0X` (`VL53L0X.h/.cpp`)

Metodos de uso comum:

- `init(...)`
- `readRangeSingleMillimeters()`
- `readRangeContinuousMillimeters()`
- `startContinuous(...)`
- `stopContinuous()`
- `getDistancia()`

### 10.3 `TCS34725` (`TCS34725.h/.cpp`)

Metodos de uso comum:

- `begin()`
- `setGain(...)`
- `setIntegrationTime(...)`
- `getRawData()`
- `getRGBC(...)`
- `detectaCorBasica()`
- `calibrar()`
- `carregarCalibracao()`

### 10.4 `BMI160` (`BMI160.h/.cpp`)

Metodos de uso comum:

- `initialize()`
- `setFullScaleAccelRange(...)`
- `setFullScaleGyroRange(...)`
- `setAccelRate(...)`
- `setGyroRate(...)`
- `getAccelX/Y/Z()`
- `getGyroX/Y/Z()`

### 10.5 `SensorLinha` (`sensorLinha.h`)

Metodos de uso comum:

- `inicializa()`
- `setModo(...)`
- `calibrarBranco()`
- `calibrarPreto()`
- `lerDados()`
- `getLinha(indice)`
- `getLinhas(...)`
- `getPosicaoLinha()`
- `getCorEsquerda/Meio/Direita(...)`

### 10.6 `Giroscopio` (`giroscopio.h`)

Metodos de uso comum:

- `inicializa()`
- `setModo(...)`
- `calibrar()`
- `lerDados()`
- `getAnguloX/Y/Z()`
- `getFrequencia()`

## 11. Atuadores e interface

### 11.1 `LEDStrip` (`led.h`)

Metodos:

- `inicializa()`
- `setLED(indice, r, g, b)`
- `setTodos(r, g, b)`
- `setBrilho(0..100)`
- `limpar()`
- `atualiza()`
- `teste()`

Limite importante observado:

- validacao interna para ate 10 LEDs por instancia

### 11.2 `Buzzer` (`buzzer.h`)

Metodos:

- `inicializa()`
- `tocar(frequencia, duracao)`
- `parar()`
- `beep(...)`
- `alerta()`
- `sucesso()`
- `erro()`
- `jingleBells()`
- `powerRangers()`

### 11.3 `SSD1306` (`SSD1306.h/.cpp`)

Metodos principais:

- `init()`
- `begin()`
- `clear(...)`
- `setCursor(...)`
- `setFonte(...)`
- `limpaLinha(...)`
- `display()`
- `noDisplay()`
- `dim(...)`
- `write(...)`

Fontes disponiveis:

- `FONTE_PEQUENA`
- `FONTE_MEDIA`
- `FONTE_GRANDE`

### 11.4 `Teclado` (`Teclado.h/.cpp`)

Metodos principais:

- `init()`
- `leBotao(1..4)`
- `alteraLed(1..4, bool)`

Constantes:

- `LIBERADO`
- `APERTADO`

## 12. Exemplo minimo funcional

```cpp
#define SUPORTE_MOTOR 1
#define SUPORTE_SENSOR_LINHA 1

#include "brickSimples.h"

Motor motorEsq(PORTA_MOTOR_1, MOTOR_NORMAL);
Motor motorDir(PORTA_MOTOR_2, MOTOR_NORMAL);
SensorLinha linha(PORTA_SERIAL_3);

void setup() {
  brick.inicializa();
  brick.adiciona(motorEsq, motorDir);
  brick.adiciona(linha);
}

void loop() {
  brick.atualiza();

  int16_t pos = linha.getPosicaoLinha();
  if (pos > 30) {
    brick.potenciaMotores(30, 60);
  } else if (pos < -30) {
    brick.potenciaMotores(60, 30);
  } else {
    brick.potenciaMotores(50, 50);
  }
}
```

## 13. Ordem recomendada para comecar projeto

1. Defina apenas os modulos que realmente vai usar.
2. Crie objetos com portas corretas de `portas.h`.
3. Chame `brick.inicializa()` primeiro no `setup()`.
4. Chame `brick.adiciona(...)` para cada dispositivo.
5. Chame `brick.atualiza()` primeiro no `loop()`.
6. Depois disso, use os metodos de leitura e acionamento.

## 14. Problemas comuns e solucao

Problema: motor nao responde.

- Verifique se `SUPORTE_MOTOR` esta definido.
- Verifique se ambos os motores foram adicionados com `brick.adiciona(m1, m2)`.
- Verifique se a bateria esta ligada e com tensao valida.

Problema: sensor nao atualiza.

- Garanta que `brick.atualiza()` esta no loop.
- Verifique a porta usada no construtor.
- Verifique se o modulo correto (`SUPORTE_*`) foi habilitado.

Problema: travamento na inicializacao.

- Tensao baixa pode causar bloqueio por seguranca em `brick.inicializa()`.
- Verifique alimentacao da placa e chave liga/desliga.

## 15. Acessibilidade desta documentacao

- as informacoes estao em listas lineares e previsiveis
- cada secao tem titulo numerado
- evitamos depender de tabela visual para entender conteudo
- os nomes de classe e metodo sao sempre escritos por extenso
- ha um fluxo recomendado passo a passo
