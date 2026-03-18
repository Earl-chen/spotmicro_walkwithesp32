# Git 代码管理规范

## 目录

- [启动时检查](#启动时检查)
- [分支管理规范](#分支管理规范)
- [版本号管理](#版本号管理)
- [提交信息规范](#提交信息规范)
- [注意事项](#注意事项)
- [执行流程](#执行流程)
- [参考文献](#参考文献)

## 启动时检查

### 首次运行检查
- **时机**：Claude Code 打开项目时
- **操作**：检查工程根目录（CLAUDE.md 同级目录）是否存在 `.git` 目录
- **如果不存在**：自动执行初始化
```bash
git init
git add .
git commit -m "chore: 项目初始化"
```
- **如果存在**：跳过，直接开始工作

## 分支管理规范

### 主要分支

#### master / main
1. 与线上版本代码一致
2. 确定上线之后的代码合并到此分支，并基于版本打 Tag
3. 不可以直接修改，通常从 hotfix 或 release 分支合并过来

#### hotfix/xxx
1. 线上版本需要紧急修复时从 master 派生出来的临时分支
2. 修复代码上线之后合并到 master 和 dev 分支，并删除此分支
3. 例：`hotfix/v3.6.6_outofmemory`

#### release
1. 以 master 分支为基础，合并许多准备上线的 feature 分支
2. 上线后合并到 master

#### test
1. 以 master 分支为基础，合并了许多要提测的 feature 分支
2. 测试环境部署此分支
3. 测试人员基于此分支在测试环境测试

#### dev
1. 以 master 分支为基础，合并了许多开发并自测完成的 feature 分支
2. 开发环境部署此分支进行联调自测

#### feature/xxx
1. 以 master 分支为基础开发具体功能的分支
2. 可以不提交到远程仓库，只保留在本地
3. 功能上线之后删除此分支
4. 合并时如有冲突，先从 test、release、dev 分支合并到 feature/xxx 解决冲突之后再提交到具体的分支
5. 例：`feature/public-rentalhouse`

### 独立发展的特殊分支
- `other/功能别名/master`
- `other/功能别名/hotfix/xxx`
- `other/功能别名/release`
- `other/功能别名/test`
- `other/功能别名/dev`
- `other/功能别名/feature/xxx`

## 版本号管理

- 分三段 **A.B.C** 或四段 **A.B.C.D**，以数字表示
- **A** - 涉及重大的改版或者重构大版本，从 1 开始
- **B** - 新增独立功能的版本，从 0 开始
- **C** - 功能优化及 bug 修复
- **D** - hotfix 类型的线上 bug 修复，例：`3.5.2.1`
- 独立发展的特殊分支版本号要在 A.B.C 的基础上加后缀，例：`3.5.2.ningbo`

## 提交信息规范

### 格式（Conventional Commits）

采用 **英文类型 + 中文描述** 的混合格式：

```
<type>(<scope>): <description>
```

- **type** - 提交类型（必填）
- **scope** - 影响范围（可选）
- **description** - 简要描述（中文）

### 类型说明

| 类型 | 说明 | 示例 |
|------|------|------|
| `feat` | 新功能 | `feat(auth): 添加用户登录功能` |
| `fix` | 修复 bug | `fix(api): 修复超时问题` |
| `docs` | 文档变更 | `docs(readme): 更新安装说明` |
| `style` | 代码格式（不影响运行） | `style: 格式化代码` |
| `refactor` | 重构（非新功能/非修复） | `refactor(utils): 重构工具函数` |
| `perf` | 性能优化 | `perf(render): 优化渲染性能` |
| `test` | 测试相关 | `test(unit): 添加单元测试` |
| `build` | 构建系统/外部依赖变更 | `build(deps): 升级依赖版本` |
| `ci` | CI/CD 配置变更 | `ci(github): 添加自动化测试流程` |
| `chore` | 其他杂项（不修改源码/测试） | `chore: 更新 .gitignore` |
| `revert` | 回滚提交 | `revert: 回滚 feat(auth)` |

### 提交时机

#### 1. 文件修改后立即提交
- **时机**：完成任何文件的修改后
- **目的**：记录修改意图，保留变更历史
- **示例**：
  - `feat(user): 添加用户认证逻辑`
  - `docs(readme): 更新使用说明`
  - `chore(config): 调整数据库连接参数`

#### 2. 编译/测试成功后再次提交（仅代码文件）
- **时机**：代码修改并成功编译/测试后
- **目的**：确认代码可运行
- **格式**：`test(<scope>): 验证<功能描述>` 或在原提交中追加说明
- **示例**：`test(auth): 验证用户认证模块编译通过`
- **注意**：
  - 仅在修改了需要编译的代码文件时执行
  - 文档、配置等非代码文件修改无需此步骤
  - 编译失败时不提交，修复后重新走流程

### 提交位置

在工程根目录（CLAUDE.md 同级目录）执行所有 git 命令

## 注意事项

### 分支管理
- 已经确定开发完成、本地自测完成、确定在接下来的版本中要上线的分支才可以从 feature/xxx 合并到 dev、test、release
- 开发周期比较长的需求要经常从 master 分支合并代码到 feature/xxx
- 临时分支完成使命后及时删除

### 提交规范
- 确保配置 `.gitignore` 排除编译产物、临时文件、依赖库等
- 提交前检查变更内容，优先选择性添加文件（`git add <file>`），避免误提交
- 代码提交尽量描述清楚提交的内容

## 执行流程

```
Claude Code 启动
       ↓
检查是否有 .git 目录？
       ↓
    否 → git init → git add . → git commit -m "chore: 项目初始化"
       ↓
    是 → 跳过
       ↓
开始正常工作（按提交策略执行）
```

## 参考文献

- [Conventional Commits 规范](https://www.conventionalcommits.org/zh-hans/)
- [Gitflow 工作流程](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow)
- [GitLab 分支管理规范](https://docs.gitlab.com/ee/topics/gitlab_flow.html)

