# 微优化：WS发送失败处理与HTTPD超时收紧

## Core Features

- WebSocket发送失败限流日志

- 非INVALID_STATE错误触发会话关闭

- 标记断开避免僵尸连接

- HTTPD超时从5s收紧到3s

## Tech Stack

{
  "Web": {
    "arch": "html",
    "component": null
  },
  "iOS": "",
  "Android": ""
}

## Design

在WS推送与响应路径上：对发送失败进行1秒窗口的限流警告；INVALID_STATE静默断开，其它错误触发httpd_sess_trigger_close并标记断开。HTTPD配置将send/recv超时由5s降至3s，保持keep-alive关闭与LRU清理开启。

## Plan

Note: 

- [ ] is holding
- [/] is doing
- [X] is done

---

[X] 依赖修复

[X] WebSocket配置启用

[X] 异常处理兼容

[X] 函数签名一致性

[X] 干净构建验证

[/] 若仍有错误则增量修复

[X] WiFi连接修复

[X] 移除串口交互

[X] HTTPD启动修复

[X] 日志根因分析

[X] 代码审查: WiFi事件与重连

[X] 代码审查: WS/HTTP请求门控与心跳

[X] 代码审查: HTTPD配置与生命周期

[X] 实现: 退避与队列化恢复

[X] 实现: 资源限制与清理

[ ] 实现: 降级策略与熔断

[X] 运行验证

[ ] 运行验证: 10分钟串口监控

[X] 运行观察: WS发送失败后清理验证

[X] 微优化: 发送失败触发关闭与限流

[X] 微优化: 超时与Keep-Alive调整

[ ] 应用层门控与队列化实现

[X] 需求: 滑块范围调整

[X] 需求: 圆锥动作β接口

[X] UI: β滑块与调用逻辑

[X] WS: 默认β兼容

[X] 构建验证

[X] 刷写与运行验证

[X] UI: β滑块范围确认
