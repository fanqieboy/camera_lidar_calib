/**
 * @file color.h
 * @brief ANSI 颜色代码定义，用于终端彩色输出
 *
 * 定义了 ANSI 转义序列颜色代码，支持基本颜色和粗体颜色，
 * 用于在支持 ANSI 的终端中显示彩色文本输出。
 */

#pragma once

// 重置颜色
#define COLOR_RESET "\033[0m"

// 基本颜色
#define COLOR_BLACK "\033[30m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN "\033[36m"
#define COLOR_WHITE "\033[37m"
#define COLOR_RED_PURPLE "\033[95m"

// 粗体颜色
#define COLOR_BOLD_BLACK "\033[1m\033[30m"
#define COLOR_BOLD_RED "\033[1m\033[31m"
#define COLOR_BOLD_GREEN "\033[1m\033[32m"
#define COLOR_BOLD_YELLOW "\033[1m\033[33m"
#define COLOR_BOLD_BLUE "\033[1m\033[34m"
#define COLOR_BOLD_MAGENTA "\033[1m\033[35m"
#define COLOR_BOLD_CYAN "\033[1m\033[36m"
#define COLOR_BOLD_WHITE "\033[1m\033[37m"
#define COLOR_BOLD_RED_PURPLE "\033[1m\033[95m"