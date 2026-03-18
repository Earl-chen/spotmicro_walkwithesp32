# HTTP降级与幂等重试上线

## Core Features

- HTTP处理器加入最多3次指数退避重试

- 保持网络门控：不可用直接503关闭会话

- 失败后JSON错误响应以便前端降级

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

为pose、action、preset、emergency分别封装重试函数(100→200→400ms backoff)，在处理器中调用封装以提升不稳定网络下成功率；保留503门控与安全响应发送。

## Plan

Note: 

- [ ] is holding
- [/] is doing
- [X] is done

---

[X] 下一步: HTTP降级与幂等重试

[/] 下一步: 统一网络状态门控检查

[/] 下一步: 断链原因分级处理

[ ] 构建验证

[ ] 烧录与运行验证

[ ] 验证安排
