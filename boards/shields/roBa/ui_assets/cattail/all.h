#pragma once
#include <stdint.h>

/* 個別フレームとdimフレームをまとめて取り込む */
#include "ani_1.h"
#include "ani_2.h"
#include "ani_3.h"
#include "ani_4.h"
#include "ani_dim.h"

/* status_screen.c で使う共通型 */
typedef struct {
    const uint8_t *data; /* 32x64 1bpp MSB-first, row-major */
} art_frame_t;

/* 明るい時のフレーム群 */
static const art_frame_t g_frames[] = {
    { ani_1 },
    { ani_2 },
    { ani_3 },
    { ani_4 },
};

/* 暗い時のフレーム */
static const art_frame_t g_dim_frame = {
    .data = ani_dim,
};

#define FRAME_COUNT (sizeof(g_frames) / sizeof(g_frames[0]))
