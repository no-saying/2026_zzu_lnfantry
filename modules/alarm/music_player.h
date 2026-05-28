/**
 * @file music_player.h
 * @brief 音乐播放器 — 非阻塞 PWM 蜂鸣器乐谱播放 (整活功能)
 *
 * 使用方式:
 *   1. Makefile C_DEFS 添加 -DMUSIC_PLAYER_ENABLED
 *   2. BuzzerInit() 之后调用 MusicPlayerInit(BuzzerGetPWM())
 *   3. 触发播放: MusicPlayerPlay(song_haruhikage)
 *   4. BuzzerTask() 中自动调用 MusicPlayerTask()
 *
 * 乐谱格式: {NOTE_X, duration, ... , 0xFF}
 *   duration 单位: SPEED/8 ms (八分音符≈62.5ms @ SPEED=500)
 */

#ifndef MUSIC_PLAYER_H
#define MUSIC_PLAYER_H

#include "bsp_pwm.h"
#include <stdint.h>

#ifdef MUSIC_PLAYER_ENABLED

/* ──── 乐谱速度 (八分音符时长 ms) ──── */
#define MUSIC_SPEED 500

/* ──── 音符枚举 A0~C8 ──── */
typedef enum {
    NOTE_P = 0, /* 休止符 */
    NOTE_A0, NOTE_A0S, NOTE_B0,
    NOTE_C1, NOTE_C1S, NOTE_D1, NOTE_D1S, NOTE_E1, NOTE_F1, NOTE_F1S, NOTE_G1, NOTE_G1S, NOTE_A1, NOTE_A1S, NOTE_B1,
    NOTE_C2, NOTE_C2S, NOTE_D2, NOTE_D2S, NOTE_E2, NOTE_F2, NOTE_F2S, NOTE_G2, NOTE_G2S, NOTE_A2, NOTE_A2S, NOTE_B2,
    NOTE_C3, NOTE_C3S, NOTE_D3, NOTE_D3S, NOTE_E3, NOTE_F3, NOTE_F3S, NOTE_G3, NOTE_G3S, NOTE_A3, NOTE_A3S, NOTE_B3,
    NOTE_C4, NOTE_C4S, NOTE_D4, NOTE_D4S, NOTE_E4, NOTE_F4, NOTE_F4S, NOTE_G4, NOTE_G4S, NOTE_A4, NOTE_A4S, NOTE_B4,
    NOTE_C5, NOTE_C5S, NOTE_D5, NOTE_D5S, NOTE_E5, NOTE_F5, NOTE_F5S, NOTE_G5, NOTE_G5S, NOTE_A5, NOTE_A5S, NOTE_B5,
    NOTE_C6, NOTE_C6S, NOTE_D6, NOTE_D6S, NOTE_E6, NOTE_F6, NOTE_F6S, NOTE_G6, NOTE_G6S, NOTE_A6, NOTE_A6S, NOTE_B6,
    NOTE_C7, NOTE_C7S, NOTE_D7, NOTE_D7S, NOTE_E7, NOTE_F7, NOTE_F7S, NOTE_G7, NOTE_G7S, NOTE_A7, NOTE_A7S, NOTE_B7,
    NOTE_C8,
    NOTE_COUNT
} Note_t;

/* ──── 音符频率表 (A4=440Hz 基准) ──── */
extern const uint16_t noteFrequencies[NOTE_COUNT];

/* ──── 歌曲声明 (定义在 music_songs/) ──── */
extern const uint8_t song_haruhikage[];

#endif /* MUSIC_PLAYER_ENABLED */

/* ──── 播放器 API (始终声明, 未启用时链接到空桩) ──── */

void MusicPlayerInit(PWMInstance *pwm);
void MusicPlayerPlay(const uint8_t *song);
void MusicPlayerStop(void);
void MusicPlayerTask(void);
uint8_t MusicPlayerIsPlaying(void);
void MusicPlayerSetVolume(uint8_t vol);

#endif /* MUSIC_PLAYER_H */
