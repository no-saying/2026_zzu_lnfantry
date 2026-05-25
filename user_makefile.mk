# user_makefile.mk — 用户自定义 Makefile 配置
# 此文件通过 Makefile 末尾的 -include 导入
# CubeMX 重新生成时不会被覆盖
#
# 当前 micro-ROS 相关配置已直接写在 Makefile 中：
#   C_SOURCES  += microros_transport/microros_transport.c
#   C_INCLUDES += -Imicroros_transport -Imicroros_include
#   LIBS += -Wl,--whole-archive lib/libmicroros.a -Wl,--no-whole-archive
#
# 如需覆盖或追加配置，在此文件中添加即可
