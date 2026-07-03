// Zero-copy RKNN runner: init once with DMA-mapped io mems, per-frame fill+run.
// Build: gcc -O2 -shared -fPIC rknn_zc.c -o librknn_zc.so -lrknnrt
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "rknn_api.h"

#define MAX_IO 8

typedef struct {
  rknn_context ctx;
  uint32_t n_input, n_output;
  rknn_tensor_attr in_attr[MAX_IO], out_attr[MAX_IO];
  rknn_tensor_mem *in_mem[MAX_IO], *out_mem[MAX_IO];
} ZC;

void* zc_init(const char* model_path, int verbose) {
  FILE* fp = fopen(model_path, "rb");
  if (!fp) { fprintf(stderr, "zc: cannot open %s\n", model_path); return NULL; }
  fseek(fp, 0, SEEK_END); long sz = ftell(fp); fseek(fp, 0, SEEK_SET);
  void* buf = malloc(sz);
  if (fread(buf, 1, sz, fp) != (size_t)sz) { fclose(fp); free(buf); return NULL; }
  fclose(fp);
  ZC* z = calloc(1, sizeof(ZC));
  int ret = rknn_init(&z->ctx, buf, sz, 0, NULL);
  free(buf);
  if (ret < 0) { fprintf(stderr, "zc: rknn_init ret=%d\n", ret); free(z); return NULL; }
  rknn_input_output_num io;
  rknn_query(z->ctx, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io));
  z->n_input = io.n_input; z->n_output = io.n_output;
  for (uint32_t i = 0; i < io.n_input; i++) {
    memset(&z->in_attr[i], 0, sizeof(rknn_tensor_attr));
    z->in_attr[i].index = i;
    rknn_query(z->ctx, RKNN_QUERY_INPUT_ATTR, &z->in_attr[i], sizeof(rknn_tensor_attr));
    z->in_mem[i] = rknn_create_mem(z->ctx, z->in_attr[i].size_with_stride);
    ret = rknn_set_io_mem(z->ctx, z->in_mem[i], &z->in_attr[i]);
    if (verbose)
      printf("zc in[%d] '%s' dims=[%d,%d,%d,%d] type=%d fmt=%d n_elems=%d size=%d stride_sz=%d w_stride=%d set_io=%d\n",
             i, z->in_attr[i].name, z->in_attr[i].dims[0], z->in_attr[i].dims[1], z->in_attr[i].dims[2], z->in_attr[i].dims[3],
             z->in_attr[i].type, z->in_attr[i].fmt, z->in_attr[i].n_elems, z->in_attr[i].size, z->in_attr[i].size_with_stride,
             z->in_attr[i].w_stride, ret);
    if (ret < 0) { fprintf(stderr, "zc: set_io_mem in[%d] ret=%d\n", i, ret); return NULL; }
  }
  for (uint32_t i = 0; i < io.n_output; i++) {
    memset(&z->out_attr[i], 0, sizeof(rknn_tensor_attr));
    z->out_attr[i].index = i;
    rknn_query(z->ctx, RKNN_QUERY_OUTPUT_ATTR, &z->out_attr[i], sizeof(rknn_tensor_attr));
    z->out_mem[i] = rknn_create_mem(z->ctx, z->out_attr[i].size_with_stride);
    ret = rknn_set_io_mem(z->ctx, z->out_mem[i], &z->out_attr[i]);
    if (verbose)
      printf("zc out[%d] '%s' type=%d fmt=%d n_elems=%d size=%d set_io=%d\n",
             i, z->out_attr[i].name, z->out_attr[i].type, z->out_attr[i].fmt, z->out_attr[i].n_elems, z->out_attr[i].size, ret);
    if (ret < 0) { fprintf(stderr, "zc: set_io_mem out[%d] ret=%d\n", i, ret); return NULL; }
  }
  return z;
}

// uint8 NCHW source -> input mem in the tensor's native type/format (single pass)
int zc_fill_u8(void* zp, int idx, const uint8_t* src) {
  ZC* z = zp;
  rknn_tensor_attr* a = &z->in_attr[idx];
  void* dst = z->in_mem[idx]->virt_addr;
  int H, W, C;
  if (a->fmt == RKNN_TENSOR_NHWC) { H = a->dims[1]; W = a->dims[2]; C = a->dims[3]; }
  else { C = a->dims[1]; H = a->dims[2]; W = a->dims[3]; }
  int ws = a->w_stride ? (int)a->w_stride : W;
  size_t HW = (size_t)H * W;
  if (a->type == RKNN_TENSOR_FLOAT16) {
    __fp16* d = (__fp16*)dst;
    if (a->fmt == RKNN_TENSOR_NHWC) {
      for (int h = 0; h < H; h++)
        for (int w = 0; w < W; w++) {
          __fp16* dp = d + ((size_t)h * ws + w) * C;
          const uint8_t* sp = src + (size_t)h * W + w;
          for (int c = 0; c < C; c++) dp[c] = (__fp16)sp[c * HW];
        }
    } else {
      size_t n = (size_t)C * HW;
      for (size_t i = 0; i < n; i++) d[i] = (__fp16)src[i];
    }
  } else if (a->type == RKNN_TENSOR_FLOAT32) {
    float* d = (float*)dst;
    if (a->fmt == RKNN_TENSOR_NHWC) {
      for (int h = 0; h < H; h++)
        for (int w = 0; w < W; w++) {
          float* dp = d + ((size_t)h * ws + w) * C;
          const uint8_t* sp = src + (size_t)h * W + w;
          for (int c = 0; c < C; c++) dp[c] = (float)sp[c * HW];
        }
    } else {
      size_t n = (size_t)C * HW;
      for (size_t i = 0; i < n; i++) d[i] = (float)src[i];
    }
  } else return -1;
  return 0;
}

// float32 source -> input mem (linear, converts to fp16 if needed)
int zc_fill_f32(void* zp, int idx, const float* src) {
  ZC* z = zp;
  rknn_tensor_attr* a = &z->in_attr[idx];
  void* dst = z->in_mem[idx]->virt_addr;
  size_t n = a->n_elems;
  if (a->type == RKNN_TENSOR_FLOAT16) {
    __fp16* d = (__fp16*)dst;
    for (size_t i = 0; i < n; i++) d[i] = (__fp16)src[i];
  } else if (a->type == RKNN_TENSOR_FLOAT32) {
    memcpy(dst, src, n * 4);
  } else return -1;
  return 0;
}

int zc_run(void* zp) { ZC* z = zp; return rknn_run(z->ctx, NULL); }

int zc_out_elems(void* zp, int idx) { ZC* z = zp; return (int)z->out_attr[idx].n_elems; }

int zc_get_output_f32(void* zp, int idx, float* dst) {
  ZC* z = zp;
  rknn_tensor_attr* a = &z->out_attr[idx];
  void* srcv = z->out_mem[idx]->virt_addr;
  size_t n = a->n_elems;
  if (a->type == RKNN_TENSOR_FLOAT16) {
    __fp16* s = (__fp16*)srcv;
    for (size_t i = 0; i < n; i++) dst[i] = (float)s[i];
  } else {
    memcpy(dst, srcv, n * 4);
  }
  return (int)n;
}

void zc_release(void* zp) {
  ZC* z = zp;
  for (uint32_t i = 0; i < z->n_input; i++) if (z->in_mem[i]) rknn_destroy_mem(z->ctx, z->in_mem[i]);
  for (uint32_t i = 0; i < z->n_output; i++) if (z->out_mem[i]) rknn_destroy_mem(z->ctx, z->out_mem[i]);
  rknn_destroy(z->ctx);
  free(z);
}

// core_mask: 1=CORE_0, 2=CORE_1, 4=CORE_2, 3=CORE_0_1, 7=CORE_0_1_2
int zc_set_core(void* zp, int mask) {
  ZC* z = zp;
  return rknn_set_core_mask(z->ctx, (rknn_core_mask)mask);
}
