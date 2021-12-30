cmake_minimum_required(VERSION 3.00)

include_directories(../include)
#搜集MyClass目录下的文件
aux_source_directory(. SRC_SOURCE)
#生成链接库
add_library(SRC   ${SRC_SOURCE})
