/**
 * @file music_player.c
 * @brief 非阻塞音乐播放器 — PWM 蜂鸣器乐谱播放
 *
 * 状态机: IDLE → NOTE_ON → NOTE_GAP → NOTE_ON → ... → IDLE (0xFF)
 * 计时基于 FreeRTOS tick, 不阻塞任务循环.
 */

#ifdef MUSIC_PLAYER_ENABLED

#include "music_player.h"
#include "cmsis_os.h"
#include <stddef.h>

/* ──── 音符频率表 (A4=440Hz, C4 等有小数的取整) ──── */
const uint16_t noteFrequencies[NOTE_COUNT] = {
    0,
    28,   29,   31,                                                          /* Octave 0  */
    33,   35,   37,   39,   41,   44,   46,   49,   52,   55,   58,   62,     /* Octave I  */
    65,   69,   73,   78,   82,   87,   93,   98,   104,  110,  117,  123,    /* Octave II */
    131,  139,  147,  156,  165,  175,  186,  196,  208,  220,  233,  247,    /* Octave III*/
    262,  277,  294,  311,  330,  349,  370,  392,  415,  440,  466,  494,    /* Octave IV */
    523,  554,  587,  622,  659,  698,  740,  784,  831,  880,  932,  988,    /* Octave V  */
    1046, 1109, 1175, 1245, 1318, 1397, 1480, 1568, 1661, 1760, 1865, 1976,  /* Octave VI */
    2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951,  /* Octave VII*/
    4186                                                                      /* Octave VIII*/
};

/* ──── 播放器内部状态 ──── */
typedef enum {
    STATE_IDLE,
    STATE_NOTE_ON,
    STATE_NOTE_GAP,
} MusicState_e;

static PWMInstance *music_pwm;
static const uint8_t *song_data;
static uint16_t song_idx;
static uint8_t  music_volume = 50; /* 1~100, 默认50=25%占空比 */
static MusicState_e state = STATE_IDLE;
static TickType_t next_tick;

#define NOTE_GAP_MS 5 /* 音符间隔 ms */

/* ──── 内部辅助 ──── */

static void play_note(Note_t note)
{
    uint16_t freq = noteFrequencies[note];
    if (freq == 0) {
        PWMSetDutyRatio(music_pwm, 0);
        return;
    }
    PWMSetPeriod(music_pwm, 1.0f / freq);
    PWMSetDutyRatio(music_pwm, music_volume * 0.5f / 100.0f);
}

static void stop_sound(void)
{
    PWMSetDutyRatio(music_pwm, 0);
}

static uint32_t duration_to_ms(uint8_t dur)
{
    return (uint32_t)MUSIC_SPEED / 8 * dur;
}

/* ──── 公开 API ──── */

void MusicPlayerInit(PWMInstance *pwm)
{
    music_pwm = pwm;
    state = STATE_IDLE;
}

void MusicPlayerPlay(const uint8_t *song)
{
    if (song == NULL) return;
    song_data = song;
    song_idx  = 0;
    state     = STATE_NOTE_ON;
    next_tick = xTaskGetTickCount(); /* 立即开始第一个音符 */
}

void MusicPlayerStop(void)
{
    state = STATE_IDLE;
    stop_sound();
}

uint8_t MusicPlayerIsPlaying(void)
{
    return state != STATE_IDLE;
}

void MusicPlayerSetVolume(uint8_t vol)
{
    if (vol > 100) vol = 100;
    if (vol < 1)   vol = 1;
    music_volume = vol;
}

void MusicPlayerTask(void)
{
    TickType_t now = xTaskGetTickCount();

    switch (state) {

    case STATE_IDLE:
        break;

    case STATE_NOTE_ON: {
        uint8_t byte0 = song_data[song_idx];
        if (byte0 == 0xFF) {
            stop_sound();
            state = STATE_IDLE;
            break;
        }
        Note_t note = (Note_t)byte0;
        song_idx++;
        uint8_t dur = song_data[song_idx];
        song_idx++;

        play_note(note);
        next_tick = now + pdMS_TO_TICKS(duration_to_ms(dur));
        state = STATE_NOTE_GAP;
        break;
    }

    case STATE_NOTE_GAP:
        if (now >= next_tick) {
            stop_sound();
            next_tick = now + pdMS_TO_TICKS(NOTE_GAP_MS);
            state = STATE_NOTE_ON;
        }
        break;
    }
}

#else /* MUSIC_PLAYER_ENABLED 未定义 — 空桩 */

#include "music_player.h"
#include "bsp_pwm.h"

const uint16_t noteFrequencies[1] = {0};
const uint8_t song_haruhikage[] = {0xFF};

void MusicPlayerInit(PWMInstance *pwm)      { (void)pwm; }
void MusicPlayerPlay(const uint8_t *song)   { (void)song; }
void MusicPlayerStop(void)                  {}
void MusicPlayerTask(void)                  {}
uint8_t MusicPlayerIsPlaying(void)          { return 0; }
void MusicPlayerSetVolume(uint8_t vol)      { (void)vol; }

#endif /* MUSIC_PLAYER_ENABLED */
