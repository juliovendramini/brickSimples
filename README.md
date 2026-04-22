# brickSimples

Biblioteca para controle de robos educacionais baseados em ATmega328PB (estilo Arduino Nano), com suporte a motores, sensores, LED, buzzer, display, teclado e Bluetooth.

## Documentacao principal

- Guia completo e acessivel: [docs/DOCUMENTACAO_COMPLETA_ACESSIVEL.md](docs/DOCUMENTACAO_COMPLETA_ACESSIVEL.md)

## Inicio rapido

1. No seu sketch, defina os modulos que deseja usar com `#define SUPORTE_*`.
2. Inclua `#include "brickSimples.h"`.
3. Chame `brick.inicializa();` como primeira linha de `setup()`.
4. Chame `brick.atualiza();` como primeira linha de `loop()`.
5. Adicione os dispositivos com `brick.adiciona(...)`.

Exemplo minimo:

```cpp
#define SUPORTE_MOTOR 1

#include "brickSimples.h"

Motor motorEsq(PORTA_MOTOR_1, MOTOR_NORMAL);
Motor motorDir(PORTA_MOTOR_2, MOTOR_NORMAL);

void setup() {
	brick.inicializa();
	brick.adiciona(motorEsq, motorDir);
}

void loop() {
	brick.atualiza();
	brick.potenciaMotores(50, 50);
}
```

## Observacoes

- A biblioteca usa compilacao condicional. Se um modulo nao estiver com `SUPORTE_*` habilitado, os metodos daquele modulo nao serao compilados.
- O projeto possui dois sketches de referencia: `brickSimples.ino` e `exemplos/brickSimples.ino`.
