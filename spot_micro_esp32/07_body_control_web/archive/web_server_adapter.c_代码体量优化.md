# web_server_adapter.c 代码体量优化

## Core Features

- HTML/JS/CSS资源分离或内嵌拆分

- 保持HTTPD与WS路由不变

- 可选启用gzip压缩提升传输效率

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

先用 EMBED_FILES 将页面资源外置为文件并嵌入，主C文件仅保留路由逻辑；随后可切换到 SPIFFS/LittleFS 读取文件，复用同一资源加载适配层。

## Plan

Note: 

- [ ] is holding
- [/] is doing
- [X] is done

---

[X] 资源拆分方案选择

[/] 实现: 内嵌头文件拆分(html_page.h/或EMBED_FILES)

[ ] 实现: 静态文件方案(SPIFFS/LittleFS)

[ ] 实现: 前端minify压缩
