// buffer.c
#include "buffer.h"
#include <stdlib.h>
#include <string.h>

buf_t *BUFRegister()
{
    buf_t *_buf = (buf_t *)malloc(sizeof(buf_t));
    memset(_buf, 0, sizeof(buf_t));
    _buf->id = 0;
    _buf->od = 0;  // 修复1：初始化为0
    return _buf;
}

float BUFUpdata(buf_t *_buf, float n, uint8_t time)
{
    uint8_t size = time + 1;
    if (size > 256) size = 256;
    
    // 写入新值
    _buf->quene[_buf->id] = n;
    
    // 计算读取位置（time步之前）
    uint8_t read_idx = (_buf->id + 256 - time) % size;
    
    // 移动写指针
    _buf->id = (_buf->id + 1) % size;
    
    // 保持读指针同步（修复2）
    _buf->od = (_buf->od + 1) % size;
    
    return _buf->quene[read_idx];  // 修复3：语法错误
}