// SoftWire falso: liga o RFID.cpp a um MFRC522 simulado
#pragma once
#include "Arduino.h"
#include "AsyncDelay.h"
struct Rc522Sim;
extern Rc522Sim *sim_rc522;
void sim_i2c_write(uint8_t addr, const uint8_t *d, uint8_t n, uint8_t &status);
uint8_t sim_i2c_read(uint8_t addr, uint8_t *d, uint8_t n);
extern long sim_transacoes;
class SoftWire : public Stream { public:
  uint8_t tx[32]; uint8_t ntx=0, addr=0; uint8_t rx[128]; uint8_t nrx=0, irx=0;
  SoftWire(uint8_t, uint8_t){}
  void begin() const {}
  void setTimeout_ms(uint16_t){}
  void beginTransmission(uint8_t a){ addr=a; ntx=0; }
  size_t write(uint8_t b){ if(ntx>=32) return 0; tx[ntx++]=b; return 1; }
  size_t write(const uint8_t*d,size_t n){ size_t r=0; for(size_t i=0;i<n;i++) r+=write(d[i]); return r; }
  uint8_t endTransmission(uint8_t s=1){ uint8_t st; sim_i2c_write(addr,tx,ntx,st); sim_transacoes++; sim_us += 100 + 110*(ntx+1); return st; }
  uint8_t requestFrom(uint8_t a,uint8_t n,uint8_t s=1){ nrx=sim_i2c_read(a,rx,n); irx=0; sim_transacoes++; sim_us += 100 + 110*(n+1); return nrx; }
  int available(){ return nrx-irx; }
  int read(){ return irx<nrx? rx[irx++] : -1; }
};
