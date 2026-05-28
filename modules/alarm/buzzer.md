# buzzer

蜂鸣器驱动模块，支持两种工作模式:

1. **警报模式** — 多级警报，按优先级触发不同音阶
2. **音乐模式** — 非阻塞 PWM 乐谱播放 (整活功能，需 `-DMUSIC_PLAYER_ENABLED`)

## 硬件

| 引脚 | 功能 |
| ---- | ---- |
| TIM12 CH2 | PWM 蜂鸣器输出 |

## 警报模式

```c
Buzzer_config_s buzzer_config = {
    .alarm_level = ALARM_LEVEL_HIGH,
    .loudness    = 0.4,
    .octave      = OCTAVE_1,
};
robocmd_alarm = BuzzerRegister(&buzzer_config);
AlarmSetStatus(robocmd_alarm, ALARM_ON);
AlarmSetStatus(robocmd_alarm, ALARM_OFF);
```

优先级: `HIGH > ABOVE_MEDIUM > MEDIUM > BELOW_MEDIUM > LOW`

## 音乐模式

### 启用

Makefile `C_DEFS` 添加 `-DMUSIC_PLAYER_ENABLED`，`BuzzerInit()` 中自动初始化播放器，开机自动播放《春日影》。

### 触发播放

```c
#include "music_player.h"
#include "music_songs/haruhikage.h"

MusicPlayerPlay(song_haruhikage);  // 开始播放
MusicPlayerStop();                 // 停止播放
MusicPlayerSetVolume(5);           // 音量 1~100 (占空比 0.5%~50%)
```

### 添加新歌曲

在 `modules/alarm/music_songs/` 下新建 `.h` 文件：

```c
#include "music_player.h"

static const uint8_t song_xxx[] = {
    NOTE_C4, 4, NOTE_D4, 4, NOTE_E4, 4, NOTE_C4, 4,
    // ...  音符和时值交替 ...
    0xFF  // 结束标记
};
```

然后在 `buzzer.c` 中 `#include` 并调用 `MusicPlayerPlay(song_xxx)`。

### 乐谱编码

- 音符: `NOTE_P` (休止) / `NOTE_C4` ~ `NOTE_B4` / 范围 `A0` ~ `C8`
- 时值: `SPEED=500`, 八分音符 = 62.5ms, `4` = 十六分, `8` = 八分, `12` = 附点四分
- 频率表: A4=440Hz 基准, 89 个音符

### 实现

- 状态机 `IDLE → NOTE_ON → NOTE_GAP → NOTE_ON → ... → IDLE (0xFF)`
- 使用 `xTaskGetTickCount()` + `pdMS_TO_TICKS()` 计时，不阻塞
- 音乐播放时自动跳过警报 (避免 PWM 冲突)
- 未启用 `MUSIC_PLAYER_ENABLED` 时编译为空桩，不增加固件体积
