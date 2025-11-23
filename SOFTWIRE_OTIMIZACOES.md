# Otimizações SoftWire - Acesso Direto a Registradores

## Resumo das Modificações

A classe SoftWire foi otimizada para usar **acesso direto aos registradores do AVR** ao invés das funções Arduino (`digitalWrite`, `pinMode`, `digitalRead`), resultando em comunicação I2C significativamente mais rápida.

## Principais Alterações

### 1. Manipulação Direta de Registradores

As funções foram reescritas para acessar diretamente os registradores:
- **PORT** (controle de saída)
- **DDR** (direção do pino - input/output)
- **PIN** (leitura do pino)

#### Funções Otimizadas:
- `sdaLow()` - Força SDA para LOW usando registradores
- `sdaHigh()` - Libera SDA para HIGH usando registradores
- `sclLow()` - Força SCL para LOW usando registradores
- `sclHigh()` - Libera SCL para HIGH usando registradores
- `readSda()` - Lê estado de SDA diretamente do registrador PIN
- `readScl()` - Lê estado de SCL diretamente do registrador PIN

### 2. Limitação de Velocidade

A velocidade máxima foi limitada a **400kHz** (Fast Mode I2C):
- Delay padrão ajustado para **2 microsegundos**
- Função `setClock()` limita frequência máxima a 400kHz
- Período mínimo de 2.5us (400kHz) garantido

### 3. Benefícios da Otimização

#### Velocidade:
- **digitalWrite/pinMode**: ~50-100 ciclos de clock
- **Acesso direto**: ~2-4 ciclos de clock
- **Ganho**: ~10-25x mais rápido na manipulação de pinos

#### Determinismo:
- Timing mais preciso e consistente
- Menos overhead de chamadas de função
- Melhor comportamento em comunicação I2C

## Compatibilidade com Pinagem

A biblioteca mantém compatibilidade total com a estrutura de portas I2C:

```cpp
constexpr static PortaI2C PORTA_I2C_1 = {14, 15, "I2C-1"};
constexpr static PortaI2C PORTA_I2C_2 = {16, 17, "I2C-2"};
constexpr static PortaI2C PORTA_I2C_3 = {8, 9, "I2C-3"};
constexpr static PortaI2C PORTA_I2C_4 = {2, 3, "I2C-4"}; 
constexpr static PortaI2C PORTA_I2C_5 = {A4, A5, "I2C-5"}; // Pinos I2C nativos
```

## Exemplo de Uso

```cpp
#include "SoftWire.h"

// Criar instância usando uma das portas I2C
SoftWire wire(PORTA_I2C_1.sda, PORTA_I2C_1.scl);

void setup() {
  wire.begin();
  wire.setClock(400000); // 400kHz - velocidade máxima suportada
}

void loop() {
  // Uso normal da biblioteca Wire
  wire.beginTransmission(0x50);
  wire.write(0x00);
  wire.endTransmission();
}
```

## Notas Técnicas

### Velocidades I2C Suportadas:
- **Standard Mode**: 100 kHz ✓
- **Fast Mode**: 400 kHz ✓ (máximo)
- **Fast Mode Plus**: 1 MHz ✗ (não suportado)
- **High Speed Mode**: 3.4 MHz ✗ (não suportado)

### Limitações:
- Código otimizado especificamente para **AVR (Arduino Uno, Mega, etc.)**
- Requer conhecimento dos registradores PORT/DDR/PIN da arquitetura AVR
- Velocidade máxima limitada a 400kHz por segurança e confiabilidade

## Performance Estimada

Com acesso direto a registradores em 16MHz (Arduino Uno):

| Operação | Tempo Estimado | Ciclos |
|----------|----------------|--------|
| Alternar pino | ~0.25 µs | ~4 |
| Ler pino | ~0.125 µs | ~2 |
| Bit completo I2C | ~2.5 µs | ~40 |
| Byte completo (8 bits + ACK) | ~25 µs | ~400 |

**Taxa real**: ~350-400 kHz (considerando overhead do protocolo)
