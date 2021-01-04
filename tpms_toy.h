#ifndef __TPMS_TOY
#define __TPMS_TOY

void decode(byte* inp, byte* realtemp, float* pressure, uint32_t* id, byte* state);
void encode(byte* out, byte realtemp, float pressure, uint32_t id, byte state);
byte crc8(byte* out, int len);
#endif
