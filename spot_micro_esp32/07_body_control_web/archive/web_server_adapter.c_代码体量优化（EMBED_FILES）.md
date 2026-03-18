# web_server_adapter.c 代码体量优化（EMBED_FILES）

## Core Features

- 移除内嵌长字符串（已屏蔽）

- 根路由改为嵌入资源返回

- CMake配置嵌入 index.html

- 修正嵌入符号名以匹配ESP-IDF规则

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

采用 EMBED_TXTFILES 嵌入 main/web/index.html，符号名使用 _binary_index_html_start/_end；root_get_handler 发送嵌入内容。

## Plan

Note: 

- [ ] is holding
- [/] is doing
- [X] is done

---

[X] 资源拆分方案选择

[X] 实现: 内嵌头文件拆分(html_page.h/或EMBED_FILES)

[X] 构建验证

[/] 烧录与运行验证

[ ] 实现: 静态文件方案(SPIFFS/LittleFS)

[ ] 实现: 前端minify压缩
