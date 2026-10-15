// MFRC522 + cartão ISO14443A simulados, para testar RFID.cpp sem hardware.
#include "SoftWire.h"
#include <vector>
#include <assert.h>

uint64_t sim_us = 0;
long sim_transacoes = 0;
Print Serial;

// CRC_A bit a bit (implementação independente da do RFID.cpp)
static uint16_t crcA(const uint8_t *d, size_t n) {
    uint16_t crc = 0x6363;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }
    return crc;
}
static bool crcOk(const std::vector<uint8_t> &f) {
    if (f.size() < 3) return false;
    uint16_t c = crcA(f.data(), f.size() - 2);
    return f[f.size() - 2] == (c & 0xFF) && f[f.size() - 1] == (c >> 8);
}
static void addCrc(std::vector<uint8_t> &f) { uint16_t c = crcA(f.data(), f.size()); f.push_back(c & 0xFF); f.push_back(c >> 8); }

struct Cartao {
    enum { IDLE, READY, ACTIVE, HALT } estado = IDLE;
    std::vector<uint8_t> uid;
    uint8_t sak;
    bool classic;
    int nivel = 0;               // nível de cascata atual (0..)
    int setorAuth = -1;
    int escrevendo = -1;         // bloco aguardando os 16 bytes do WRITE
    uint8_t mem[256][16] = {};   // classic: blocos; UL: páginas*4 em mem[p/4][(p%4)*4]
    uint8_t chave[40][6];
    bool presente = true;
    Cartao(std::vector<uint8_t> u, bool cl) : uid(u), classic(cl) {
        sak = cl ? 0x08 : 0x00;
        memset(chave, 0xFF, sizeof chave);
        for (int i = 0; i < 16; i++) mem[0][i] = 0xA0 + i;   // fabricante
    }
    std::vector<uint8_t> nivelUid(int n) {      // 4 bytes do nível n (com CT se preciso)
        std::vector<uint8_t> r;
        if (uid.size() == 4) { r.assign(uid.begin(), uid.end()); }
        else if (uid.size() == 7) {
            if (n == 0) r = { 0x88, uid[0], uid[1], uid[2] };
            else r = { uid[3], uid[4], uid[5], uid[6] };
        }
        return r;
    }
    int niveis() { return uid.size() == 4 ? 1 : 2; }
    // devolve true se respondeu; resp e bitsUltimo (0 = 8)
    bool recebe(std::vector<uint8_t> f, uint8_t bits, std::vector<uint8_t> &resp, uint8_t &bitsResp) {
        resp.clear(); bitsResp = 0;
        if (!presente) return false;
        if (bits == 7 && f.size() == 1) {
            if ((f[0] == 0x26 && estado == IDLE) || (f[0] == 0x52 && (estado == IDLE || estado == HALT))) {
                estado = READY; nivel = 0; setorAuth = -1;
                resp = { (uint8_t)(uid.size() == 4 ? 0x04 : 0x44), 0x00 };
                return true;
            }
            if (estado == READY || estado == ACTIVE) estado = IDLE;   // ISO 14443-3: volta ao IDLE
            return false;
        }
        if (escrevendo >= 0) {               // segunda fase do WRITE
            int b = escrevendo; escrevendo = -1;
            if (f.size() != 18 || !crcOk(f)) { estado = IDLE; return false; }
            memcpy(mem[b], f.data(), 16);
            resp = { 0x0A }; bitsResp = 4; return true;
        }
        if (estado == READY && f.size() == 2 && (f[0] == 0x93 + 2 * nivel) && f[1] == 0x20) {
            auto u = nivelUid(nivel);
            resp = u; resp.push_back(u[0] ^ u[1] ^ u[2] ^ u[3]);
            return true;
        }
        if (estado == READY && f.size() == 9 && (f[0] == 0x93 + 2 * nivel) && f[1] == 0x70) {
            if (!crcOk(f)) return false;
            auto u = nivelUid(nivel);
            if (memcmp(&f[2], u.data(), 4) != 0) return false;
            bool ultimo = (nivel + 1 == niveis());
            resp = { (uint8_t)(ultimo ? sak : 0x04) }; addCrc(resp);
            if (ultimo) estado = ACTIVE; else nivel++;
            return true;
        }
        if (estado == ACTIVE && f.size() == 4 && f[0] == 0x50 && f[1] == 0x00 && crcOk(f)) {
            estado = HALT; setorAuth = -1; return false;
        }
        if (estado == ACTIVE && f.size() == 4 && f[0] == 0x30 && crcOk(f)) {
            uint8_t a = f[1];
            if (classic) {
                int setor = a < 128 ? a / 4 : 32 + (a - 128) / 16;
                if (setor != setorAuth) { estado = IDLE; resp = { 0x04 }; bitsResp = 4; return true; }
                resp.assign(mem[a], mem[a] + 16);
            } else {
                for (int p = 0; p < 4; p++) { int pg = (a + p) % 45; for (int i = 0; i < 4; i++) resp.push_back(mem[pg / 4][(pg % 4) * 4 + i]); }
            }
            addCrc(resp); return true;
        }
        if (estado == ACTIVE && classic && f.size() == 4 && f[0] == 0xA0 && crcOk(f)) {
            int a = f[1]; int setor = a < 128 ? a / 4 : 32 + (a - 128) / 16;
            if (setor != setorAuth) { estado = IDLE; resp = { 0x04 }; bitsResp = 4; return true; }
            escrevendo = a; resp = { 0x0A }; bitsResp = 4; return true;
        }
        if (estado == ACTIVE && !classic && f.size() == 8 && f[0] == 0xA2 && crcOk(f)) {
            int pg = f[1]; if (pg < 2 || pg >= 45) { resp = { 0x00 }; bitsResp = 4; return true; }
            memcpy(&mem[pg / 4][(pg % 4) * 4], &f[2], 4);
            resp = { 0x0A }; bitsResp = 4; return true;
        }
        estado = IDLE;   // comando inesperado
        return false;
    }
    bool autentica(const uint8_t *b) {   // FIFO do MFAuthent
        if (!presente || estado != ACTIVE || !classic) return false;
        int a = b[1]; int setor = a < 128 ? a / 4 : 32 + (a - 128) / 16;
        if (memcmp(&b[8], &uid[uid.size() - 4], 4) != 0) return false;
        if (memcmp(&b[2], chave[setor], 6) != 0) { estado = IDLE; return false; }
        setorAuth = setor; return true;
    }
};

struct Rc522Sim {
    uint8_t reg[64] = {};
    std::vector<uint8_t> fifo;
    Cartao *cartao = nullptr;
    Cartao *cartao2 = nullptr;   // segundo cartão para colisão
    uint8_t versao = 0x92;
    long nTransceive = 0;
    bool semResposta = false; uint64_t tFim = 0;
    void reset() { memset(reg, 0, sizeof reg); reg[0x37] = versao; reg[0x14] = 0x80; reg[0x26] = 0x48; fifo.clear(); }
    void escreve(uint8_t r, uint8_t v) {
        switch (r) {
        case 0x01:
            if (v == 0x0F) { reset(); return; }
            reg[1] = v;
            if (v == 0x0E) {                       // MFAuthent
                reg[4] &= ~0x31;
                bool ok = fifo.size() >= 12 && cartao && cartao->autentica(fifo.data());
                if (ok) { reg[0x08] |= 0x08; reg[4] |= 0x10; } else reg[4] |= 0x01;
                fifo.clear();
            }
            return;
        case 0x04: if (v & 0x80) reg[4] |= (v & 0x7F); else reg[4] &= ~(v & 0x7F); return;
        case 0x09: fifo.push_back(v); return;
        case 0x0A: if (v & 0x80) fifo.clear(); return;
        case 0x0D:
            reg[0x0D] = v & 0x7F;
            if ((v & 0x80) && reg[1] == 0x0C) transceive(v & 0x07);
            return;
        default: reg[r] = v;
        }
    }
    uint8_t le(uint8_t r) {
        switch (r) {
        case 0x09: { if (fifo.empty()) return 0; uint8_t b = fifo.front(); fifo.erase(fifo.begin()); return b; }
        case 0x0A: return (uint8_t)fifo.size();
        case 0x04: if (semResposta && sim_us >= tFim) { reg[4] |= 0x01; semResposta = false; } return reg[4];
        default: return reg[r];
        }
    }
    void transceive(uint8_t bits) {
        nTransceive++; semResposta = false;
        std::vector<uint8_t> f = fifo; fifo.clear();
        reg[6] = 0; reg[4] &= ~0x31;
        std::vector<uint8_t> r1, r2; uint8_t b1 = 0, b2 = 0;
        bool a1 = cartao && cartao->recebe(f, bits, r1, b1);
        bool a2 = cartao2 && cartao2->recebe(f, bits, r2, b2);
        if (!a1 && !a2) { semResposta = true; tFim = sim_us + (uint64_t)(((reg[0x2C] << 8) | reg[0x2D]) * 25); return; }
        std::vector<uint8_t> &r = a1 ? r1 : r2;
        if (a1 && a2 && r1 != r2) reg[6] |= 0x08;            // colisão
        fifo = r; reg[0x0C] = a1 ? b1 : b2; reg[4] |= 0x30;
    }
};

static Rc522Sim rc;
Rc522Sim *sim_rc522 = &rc;
bool sim_conectado = true;

void sim_i2c_write(uint8_t addr, const uint8_t *d, uint8_t n, uint8_t &st) {
    if (!sim_conectado || addr != 0x28) { st = 2; return; }
    st = 0;
    if (n == 0) return;
    uint8_t r = d[0];
    for (uint8_t i = 1; i < n; i++) rc.escreve(r, d[i]);
    rc.reg[63] = r;                 // "ponteiro" de registrador para a leitura
}
uint8_t sim_i2c_read(uint8_t addr, uint8_t *d, uint8_t n) {
    if (!sim_conectado || addr != 0x28) return 0;
    for (uint8_t i = 0; i < n; i++) d[i] = rc.le(rc.reg[63]);
    return n;
}

// ======================================================================= testes
#include "RFID.h"
static int falhas = 0;
#define CHECA(c) do { if (!(c)) { printf("FALHOU linha %d: %s\n", __LINE__, #c); falhas++; } } while (0)

int main() {
    rc.reset();
    // vetor conhecido: HLTA 50 00 -> CRC 57 CD
    { uint8_t h[2] = {0x50, 0x00}; uint16_t c = crcA(h, 2); CHECA(c == 0xCD57); }

    RFID rfid((PortaI2C)PORTA_I2C_2);
    CHECA(rfid.inicializa());
    CHECA((rc.reg[0x14] & 3) == 3);                    // antena ligada

    // sem cartão
    uint64_t t0 = sim_us;
    CHECA(!rfid.cartaoPresente());
    printf("cartaoPresente sem cartao: %.1f ms\n", (sim_us - t0) / 1000.0);
    CHECA(!rfid.cartaoNovo());

    // cartão Classic 1K de 4 bytes
    Cartao c1({0xDE, 0xAD, 0xBE, 0xEF}, true);
    rc.cartao = &c1;
    t0 = sim_us;
    CHECA(rfid.cartaoNovo());
    printf("cartaoNovo com cartao: %.1f ms, %ld transacoes I2C no total ate aqui\n", (sim_us - t0) / 1000.0, sim_transacoes);
    CHECA(rfid.tamanhoUid() == 4 && rfid.uid32() == 0xDEADBEEF);
    CHECA(rfid.tipo() == RFID::TIPO_MIFARE_1K);
    rfid.imprimeUid();
    for (int i = 0; i < 5; i++) { CHECA(!rfid.cartaoNovo()); CHECA(rfid.cartaoPresente()); }
    t0 = sim_us; rfid.cartaoPresente(); printf("cartaoPresente com cartao parado: %.1f ms\n", (sim_us - t0) / 1000.0);

    // dados
    CHECA(rfid.escreveTexto(1, "VITIMA") == RFID::OK);
    char txt[17];
    t0 = sim_us;
    CHECA(rfid.lerTexto(1, txt) == RFID::OK && strcmp(txt, "VITIMA") == 0);
    printf("lerTexto (cartao ja selecionado e autenticado no setor): %.1f ms\n", (sim_us - t0) / 1000.0);
    rfid.cartaoPresente(); t0 = sim_us; rfid.lerTexto(1, txt); printf("lerTexto (com autenticacao): %.1f ms\n", (sim_us - t0) / 1000.0);
    t0 = sim_us; rfid.escreveTexto(1, "VITIMA"); printf("escreveTexto: %.1f ms\n", (sim_us - t0) / 1000.0);
    uint8_t b[16];
    CHECA(rfid.lerBloco(0, b) == RFID::OK && b[0] == 0xA0);        // fabricante
    CHECA(rfid.escreveBloco(0, b) == RFID::PROIBIDO);
    CHECA(rfid.escreveBloco(3, b) == RFID::PROIBIDO);
    CHECA(rfid.escreveTexto(9, "sala 3, outro setor") == RFID::OK);   // corta em 16
    CHECA(rfid.lerTexto(9, txt) == RFID::OK && strcmp(txt, "sala 3, outro se") == 0);
    CHECA(rfid.lerTexto(1, txt) == RFID::OK && strcmp(txt, "VITIMA") == 0);  // volta ao setor 0
    // presença depois de ler (cartão estava ATIVO e autenticado)
    CHECA(rfid.cartaoPresente());
    CHECA(rfid.lerTexto(1, txt) == RFID::OK && strcmp(txt, "VITIMA") == 0);

    // chave errada
    memset(c1.chave[2], 0x11, 6);
    CHECA(rfid.lerBloco(8, b) == RFID::AUTENTICACAO);
    CHECA(rfid.lerTexto(1, txt) == RFID::OK && strcmp(txt, "VITIMA") == 0);  // recupera sozinho
    uint8_t k[6]; memset(k, 0x11, 6); rfid.defineChave(k);
    CHECA(rfid.lerBloco(8, b) == RFID::OK);
    uint8_t kff[6]; memset(kff, 0xFF, 6); rfid.defineChave(kff);

    // tira o cartão: precisa de 2 leituras sem ele para considerar que saiu
    c1.presente = false; c1.estado = Cartao::IDLE;
    CHECA(!rfid.cartaoNovo());
    CHECA(!rfid.cartaoPresente());
    CHECA(!rfid.cartaoNovo());
    CHECA(rfid.lerBloco(1, b) == RFID::SEM_CARTAO);
    c1.presente = true;
    CHECA(rfid.cartaoNovo());

    // um cartão detectado por cartaoPresente() ainda conta como novo no próximo cartaoNovo()
    c1.presente = false; CHECA(!rfid.cartaoPresente()); CHECA(!rfid.cartaoPresente());
    c1.presente = true; c1.estado = Cartao::IDLE; CHECA(rfid.cartaoPresente());
    CHECA(rfid.cartaoNovo()); CHECA(!rfid.cartaoNovo());

    // troca por um NTAG de 7 bytes
    Cartao c2({0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x80}, false);
    rc.cartao = &c2;
    CHECA(rfid.cartaoNovo());                              // UID diferente = novo
    CHECA(rfid.tamanhoUid() == 7 && rfid.byteUid(6) == 0x80 && rfid.tipo() == RFID::TIPO_ULTRALIGHT);
    rfid.imprimeUid();
    CHECA(rfid.escreveTexto(1, "SALA 3") == RFID::OK);
    CHECA(memcmp(&c2.mem[1][0], "SALA 3", 6) == 0);       // páginas 4..7
    CHECA(rfid.lerTexto(1, txt) == RFID::OK && strcmp(txt, "SALA 3") == 0);
    CHECA(rfid.escreveBloco(0, b) == RFID::PROIBIDO);
    CHECA(!rfid.cartaoNovo() && rfid.cartaoPresente());

    // dois cartões ao mesmo tempo
    c1.estado = Cartao::IDLE;
    rc.cartao2 = &c1;
    CHECA(!rfid.cartaoPresente());
    printf("dois cartoes: %s\n", (const char *)rfid.textoStatus(rfid.ultimoStatus()));
    rc.cartao2 = nullptr;
    CHECA(rfid.cartaoPresente());

    printf(falhas ? "%d FALHAS\n" : "TODOS OS TESTES PASSARAM\n", falhas);
    return falhas != 0;
}
