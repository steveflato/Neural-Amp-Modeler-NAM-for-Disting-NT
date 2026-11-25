// Reference: run the A2-Lite engine directly on raw float32 weights (block 48).
#include <stdio.h>
#include <stdlib.h>
#include "../engine/nam_a2lite.h"
static nam_a2lite_weights_t W; static nam_a2lite_t I; static nam_a2lite_state_t S;
int main(int c, char** v) {
  float w[2000]; FILE* f = fopen(v[1], "rb"); int n = fread(w, 4, 2000, f); fclose(f);
  if (nam_a2lite_load_weights(&W, w, n)) { fprintf(stderr, "bad count %d\n", n); return 1; }
  nam_a2lite_init(&I, &W, &S); nam_a2lite_prewarm(&I);
  f = fopen(v[2], "rb"); FILE* o = fopen(v[3], "wb"); float in[48], out[48]; int k;
  while ((k = fread(in, 4, 48, f)) > 0) { const float* ip[1] = {in}; float* op[1] = {out};
    nam_a2lite_process(&I, ip, op, k); fwrite(out, 4, k, o); }
  fclose(f); fclose(o); return 0; }
