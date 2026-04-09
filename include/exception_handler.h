#pragma once

#include <exception>
#include <string>
#include <functional>

// 包含新库的异常类型
#include "dataset_loader/dataset_loader.h"
#include "pcd_merger/pcd_merger.h"
#include "video_extractor/video_extractor.h"

/**
 * @file exception_handler.h
 * @brief 统一异常处理系统
 * @author Claude
 * @date 2025-09-14
 * @version 2.1.0
 */

namespace lidar_camera_calib {

/**
 * @brief 异常处理结果结构
 */
struct ExceptionHandleResult {
    bool handled;               // 是否成功处理
    int exit_code;              // 建议的退出码
    std::string error_message;  // 错误消息
    std::string user_message;   // 用户友好的消息

    ExceptionHandleResult() : handled(false), exit_code(1) {}
};

/**
 * @brief 统一异常处理器
 *
 * 负责处理所有新库的异常类型，提供统一的错误报告和处理机制
 */
class ExceptionHandler {
public:
    /**
     * @brief 构造函数
     */
    ExceptionHandler();

    /**
     * @brief 析构函数
     */
    ~ExceptionHandler();

    /**
     * @brief 处理异常并返回处理结果
     * @param e 异常对象
     * @return ExceptionHandleResult 处理结果
     */
    ExceptionHandleResult HandleException(const std::exception& e) const;

    /**
     * @brief 设置详细错误信息输出开关
     * @param verbose 是否输出详细信息
     */
    void SetVerbose(bool verbose);

    /**
     * @brief 设置静默模式开关
     * @param quiet 是否启用静默模式
     */
    void SetQuiet(bool quiet);

    /**
     * @brief 设置自定义错误回调函数
     * @param callback 错误回调函数
     */
    void SetErrorCallback(std::function<void(const std::string&)> callback);

    /**
     * @brief 安全执行函数并处理异常
     * @tparam Func 函数类型
     * @param func 要执行的函数
     * @param operation_name 操作名称（用于错误报告）
     * @return int 执行结果（0表示成功，非0表示失败）
     */
    template<typename Func>
    int SafeExecute(Func func, const std::string& operation_name) const;

    /**
     * @brief 报告错误信息
     * @param message 错误信息
     * @param is_fatal 是否为致命错误
     */
    void ReportError(const std::string& message, bool is_fatal = false) const;

    /**
     * @brief 报告警告信息
     * @param message 警告信息
     */
    void ReportWarning(const std::string& message) const;

    /**
     * @brief 获取异常的用户友好描述
     * @param e 异常对象
     * @return std::string 用户友好的错误描述
     */
    std::string GetUserFriendlyMessage(const std::exception& e) const;

private:
    bool verbose_;                                              // 是否输出详细信息
    bool quiet_;                                                // 是否静默模式
    std::function<void(const std::string&)> error_callback_;    // 错误回调函数

    /**
     * @brief 处理数据集加载相关异常
     * @param e 异常对象
     * @return ExceptionHandleResult 处理结果
     */
    ExceptionHandleResult HandleDatasetException(const DatasetValidationError& e) const;

    /**
     * @brief 处理PCD合并相关异常
     * @param e 异常对象
     * @return ExceptionHandleResult 处理结果
     */
    ExceptionHandleResult HandlePcdMergeException(const PcdMergeError& e) const;

    /**
     * @brief 处理视频提取相关异常
     * @param e 异常对象
     * @return ExceptionHandleResult 处理结果
     */
    ExceptionHandleResult HandleVideoExtractionException(const VideoExtractionError& e) const;

    /**
     * @brief 获取建议的退出码
     * @param e 异常对象
     * @return int 退出码
     */
    int GetSuggestedExitCode(const std::exception& e) const;

    /**
     * @brief 获取异常类型名称
     * @param e 异常对象
     * @return std::string 异常类型名称
     */
    std::string GetExceptionTypeName(const std::exception& e) const;

    /**
     * @brief 输出格式化的错误信息
     * @param level 日志级别（ERROR, WARNING等）
     * @param message 消息内容
     */
    void LogMessage(const std::string& level, const std::string& message) const;
};

/**
 * @brief SafeExecute模板方法的实现
 */
template<typename Func>
int ExceptionHandler::SafeExecute(Func func, const std::string& operation_name) const {
    try {
        func();
        return 0;
    } catch (const std::exception& e) {
        ExceptionHandleResult result = HandleException(e);

        if (!quiet_) {
            LogMessage("ERROR", "Operation '" + operation_name + "' failed: " + result.user_message);
            if (verbose_ && !result.error_message.empty()) {
                LogMessage("DETAIL", result.error_message);
            }
        }

        if (error_callback_) {
            error_callback_(result.user_message);
        }

        return result.exit_code;
    }
}

/**
 * @brief 全局异常处理器实例
 */
extern ExceptionHandler g_exception_handler;

/**
 * @brief 便捷宏，用于安全执行操作
 */
#define SAFE_EXECUTE(operation, name) \
    g_exception_handler.SafeExecute([&]() { operation; }, name)

} // namespace lidar_camera_calib