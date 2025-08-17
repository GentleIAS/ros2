# 📚 Git使用指南 - HTTPS版本

> 🎯 **快速导航**: [新手入门](#-新手快速入门) | [常用命令](#-常用命令速查) | [问题解决](#-常见问题解决) | [高级技巧](#-高级技巧与优化)

---

## 📋 详细目录

### 🚀 基础入门
- [Git简介](#git简介)
- [Git安装与配置](#git安装与配置)
  - [检查安装](#1-检查git是否已安装)
  - [用户配置](#2-配置用户信息)
  - [其他配置](#3-其他有用的配置)
- [基础概念](#基础概念)
  - [工作区域](#工作区域)
  - [文件状态](#文件状态)
  - [项目级别仓库](#项目级别的git仓库)

### 💻 核心操作
- [常用命令详解](#常用命令详解)
  - [初始化仓库](#初始化仓库)
  - [文件操作](#查看状态)
  - [提交管理](#提交更改)
  - [历史查看](#查看历史记录)
  - [撤销操作](#撤销操作)
- [HTTPS方式配置GitHub](#https方式配置github)
  - [创建仓库](#1-创建github仓库)
  - [Token配置](#2-获取personal-access-token)
  - [远程仓库](#3-配置远程仓库)
  - [推送拉取](#4-推送到github)

### 🔄 工作流程
- [日常工作流程](#日常工作流程)
  - [标准流程](#标准工作流程)
  - [新项目初始化](#新项目完整初始化流程)
  - [现有项目处理](#从现有项目创建新项目)
- [分支管理](#分支管理)
  - [基本操作](#基本分支操作)
  - [合并策略](#分支合并)
  - [分支策略](#常用分支策略)
- [版本回退与恢复](#版本回退与恢复)
  - [回退方法](#回退到指定提交)
  - [恢复技巧](#恢复误删的提交)

### 🛠️ 问题解决
- [常见问题解决](#常见问题解决)
  - [推送问题](#1-推送被拒绝)
  - [合并冲突](#2-合并冲突详解)
  - [Windows问题](#3-windows系统特有问题)
  - [凭据问题](#4-凭据管理问题)
  - [大文件处理](#5-大文件处理)
- [快速故障排除](#快速故障排除)
  - [错误信息对照](#常见错误信息及解决方案)
  - [紧急恢复](#紧急恢复命令)

### 🎯 进阶技巧
- [高级技巧与优化](#高级技巧与优化)
  - [Git别名](#1-git别名配置)
  - [交互式操作](#2-交互式暂存)
  - [储藏功能](#3-储藏stash功能)
  - [标签管理](#4-标签管理)
  - [子模块](#5-子模块管理)
- [团队协作最佳实践](#团队协作最佳实践)
  - [提交规范](#1-提交信息规范)
  - [分支策略](#2-分支策略详解)
  - [代码审查](#3-代码审查流程)
- [性能优化与维护](#性能优化与维护)
  - [仓库清理](#1-仓库清理)
  - [大仓库优化](#2-大仓库优化)
  - [钩子应用](#3-钩子hooks应用)

### 📖 快速参考
- [常用命令速查表](#常用命令速查表)
  - [项目初始化](#项目初始化命令)
  - [基础命令](#基础命令)
  - [分支操作](#分支操作)
  - [历史查看](#历史查看)
  - [撤销操作](#撤销操作)
  - [远程操作](#远程操作)
- [最佳实践](#最佳实践)
  - [项目初始化](#1-项目初始化最佳实践)
  - [开发习惯](#2-日常开发习惯)
  - [安全注意](#4-安全注意事项)

---

## 🚀 新手快速入门

### ⚡ 5分钟上手Git

```bash
# 1️⃣ 配置用户信息（首次使用）
git config --global user.name "你的姓名"
git config --global user.email "你的邮箱"

# 2️⃣ 初始化项目
git init

# 3️⃣ 添加文件
git add .

# 4️⃣ 提交更改
git commit -m "初始提交"

# 5️⃣ 连接GitHub（需要先在GitHub创建仓库）
git remote add origin https://github.com/你的用户名/仓库名.git
git push -u origin main
```

### 🎯 最常用的5个命令

| 命令 | 作用 | 使用频率 |
|------|------|----------|
| `git status` | 查看文件状态 | ⭐⭐⭐⭐⭐ |
| `git add .` | 添加所有文件到暂存区 | ⭐⭐⭐⭐⭐ |
| `git commit -m "信息"` | 提交更改 | ⭐⭐⭐⭐⭐ |
| `git push` | 推送到远程仓库 | ⭐⭐⭐⭐ |
| `git pull` | 拉取远程更改 | ⭐⭐⭐⭐ |

---

## Git简介

Git是一个分布式版本控制系统，用于跟踪文件的变化，协调多人协作开发。它可以记录文件的每一次修改，让你能够回到任何历史版本。

### 为什么使用Git？
- **版本控制**：记录文件的每次修改
- **协作开发**：多人同时开发同一项目
- **备份安全**：分布式存储，数据安全
- **分支管理**：支持并行开发不同功能

## Git安装与配置

### 1. 检查Git是否已安装

```bash
git --version
```

如果显示版本号，说明已安装。如果没有，请到[Git官网](https://git-scm.com/)下载安装。

### 2. 配置用户信息

```bash
# 配置用户名（必须）
git config --global user.name "你的用户名"

# 配置邮箱（必须）
git config --global user.email "你的邮箱@example.com"

# 查看配置信息
git config --list
```

**参数说明：**
- `--global`：全局配置，对所有Git仓库生效
- `user.name`：设置提交者姓名
- `user.email`：设置提交者邮箱

### 3. 其他有用的配置

```bash
# 设置默认编辑器
git config --global core.editor "notepad"

# 设置默认分支名为main
git config --global init.defaultBranch main

# 启用颜色输出
git config --global color.ui auto
```

## 基础概念

### 工作区域

1. **工作目录（Working Directory）**：你正在编辑的文件所在的目录
2. **暂存区（Staging Area）**：准备提交的文件临时存放区域
3. **本地仓库（Local Repository）**：存储项目历史记录的地方
4. **远程仓库（Remote Repository）**：托管在服务器上的仓库（如GitHub）

### 文件状态

- **未跟踪（Untracked）**：新创建的文件，Git还不知道
- **已修改（Modified）**：文件被修改但还没有暂存
- **已暂存（Staged）**：文件已添加到暂存区
- **已提交（Committed）**：文件已保存到本地仓库

### 项目级别的Git仓库

**重要概念**：每个项目都需要独立的Git仓库

- **一个项目 = 一个Git仓库**：每个项目目录都需要自己的`.git`文件夹
- **独立的版本历史**：不同项目有各自的提交记录、分支和标签
- **独立的远程仓库**：每个项目可以连接到不同的GitHub仓库

**为什么需要重新初始化？**
1. Git仓库是项目级别的，不是全局的
2. `.git`文件夹存储该项目的所有版本信息
3. 不同项目需要不同的远程仓库地址
4. 项目间的历史记录应该相互独立

## 常用命令详解

### 初始化仓库

#### 新项目初始化流程
```bash
# 方式一：在现有项目中初始化
cd your-project-directory
git init

# 方式二：创建新项目并初始化
mkdir my-new-project
cd my-new-project
git init

# 方式三：从远程仓库克隆（自动初始化）
git clone https://github.com/用户名/仓库名.git
```

#### 验证初始化状态
```bash
# 检查是否已初始化Git仓库
ls -la                    # 查看是否有.git文件夹
git status               # 检查Git状态
git remote -v            # 查看远程仓库配置
```

### 查看状态

```bash
# 查看仓库状态
git status

# 查看文件差异
git diff

# 查看暂存区与最后提交的差异
git diff --staged
```

### 添加文件到暂存区

```bash
# 添加单个文件
git add 文件名.txt

# 添加所有.md文件
git add *.md

# 添加当前目录所有文件
git add .

# 添加所有文件（包括删除的文件）
git add -A
```

### 提交更改

```bash
# 提交暂存区的文件
git commit -m "提交信息"

# 添加并提交（跳过暂存区）
git commit -am "提交信息"

# 修改最后一次提交信息
git commit --amend -m "新的提交信息"
```

**提交信息规范：**
- 使用现在时态："添加功能" 而不是 "添加了功能"
- 简洁明了，50字符以内
- 如果需要详细说明，第一行写概要，空一行后写详细内容

### 查看历史记录

```bash
# 查看提交历史
git log

# 简洁显示（一行一个提交）
git log --oneline

# 显示最近3次提交
git log -3

# 图形化显示分支
git log --graph --oneline
```

### 撤销操作

```bash
# 撤销工作目录的修改
git checkout -- 文件名.txt

# 撤销暂存区的文件（保留工作目录修改）
git reset HEAD 文件名.txt

# 撤销最后一次提交（保留修改）
git reset --soft HEAD~1

# 撤销最后一次提交（不保留修改）
git reset --hard HEAD~1
```

## HTTPS方式配置GitHub

### 1. 创建GitHub仓库

1. 登录GitHub网站
2. 点击右上角的 "+" → "New repository"
3. 填写仓库名称和描述
4. 选择公开或私有
5. 点击 "Create repository"

### 2. 获取Personal Access Token

由于GitHub不再支持密码认证，需要使用Personal Access Token：

1. 登录GitHub → 点击头像 → **Settings**
2. 左侧菜单 → **Developer settings**
3. **Personal access tokens** → **Tokens (classic)**
4. **Generate new token** → **Generate new token (classic)**
5. 设置Token信息：
   - **Note**：描述这个token的用途
   - **Expiration**：选择过期时间
   - **Select scopes**：至少勾选 `repo`（完整仓库访问权限）
6. 点击 **Generate token**
7. **重要**：立即复制生成的token（只显示一次！）

### 3. 配置远程仓库

```bash
# 添加远程仓库（HTTPS方式）
git remote add origin https://github.com/用户名/仓库名.git

# 查看远程仓库信息
git remote -v

# 修改远程仓库地址
git remote set-url origin https://github.com/用户名/新仓库名.git

# 删除远程仓库
git remote remove origin
```

### 4. 推送到GitHub

```bash
# 第一次推送（设置上游分支）
git push -u origin main

# 后续推送
git push

# 推送所有分支
git push --all

# 推送标签
git push --tags
```

**认证说明：**
- 用户名：你的GitHub用户名
- 密码：使用Personal Access Token（不是登录密码）

### 5. 从GitHub拉取更新

```bash
# 拉取并合并远程更改
git pull

# 等同于以下两个命令
git fetch  # 获取远程更新
git merge  # 合并到当前分支

# 拉取指定分支
git pull origin main
```

## 日常工作流程

### 标准工作流程

```bash
# 1. 查看当前状态
git status

# 2. 拉取最新代码（如果是协作项目）
git pull

# 3. 进行文件修改
# ... 编辑文件 ...

# 4. 查看修改内容
git diff

# 5. 添加文件到暂存区
git add .

# 6. 提交更改
git commit -m "描述你的修改"

# 7. 推送到远程仓库
git push
```

### 新项目完整初始化流程

```bash
# 步骤1：创建项目目录（如果还没有）
mkdir my-awesome-project
cd my-awesome-project

# 步骤2：初始化Git仓库
git init

# 步骤3：创建基本文件
echo "# My Awesome Project" > README.md
echo "*.log" > .gitignore
echo "node_modules/" >> .gitignore

# 步骤4：添加文件到暂存区
git add .

# 步骤5：创建第一次提交
git commit -m "feat: 初始化项目结构"

# 步骤6：添加远程仓库（需要先在GitHub创建仓库）
git remote add origin https://github.com/你的用户名/my-awesome-project.git

# 步骤7：推送到远程仓库
git push -u origin main

# 步骤8：验证设置
git remote -v
git status
```

### 从现有项目创建新项目

```bash
# 如果要基于现有项目创建新项目
cp -r old-project new-project
cd new-project

# 删除旧的Git历史
rm -rf .git

# 重新初始化Git
git init
git add .
git commit -m "feat: 基于old-project创建新项目"

# 连接到新的远程仓库
git remote add origin https://github.com/你的用户名/new-project.git
git push -u origin main
```

## 分支管理

### 基本分支操作

```bash
# 查看所有分支
git branch

# 查看远程分支
git branch -r

# 查看所有分支（本地+远程）
git branch -a

# 创建新分支
git branch 新分支名

# 切换分支
git checkout 分支名

# 创建并切换到新分支
git checkout -b 新分支名

# 删除分支
git branch -d 分支名

# 强制删除分支
git branch -D 分支名
```

### 分支合并

```bash
# 合并指定分支到当前分支
git merge 分支名

# 创建合并提交（即使可以快进合并）
git merge --no-ff 分支名

# 取消合并
git merge --abort
```

### 常用分支策略

- **main/master**：主分支，存放稳定版本
- **develop**：开发分支，日常开发
- **feature/功能名**：功能分支，开发新功能
- **hotfix/修复名**：热修复分支，紧急修复

## 版本回退与恢复

### 回退到指定提交

假设你有提交历史：a → b → c，现在想回退到提交a：

#### 方法一：git reset（本地回退）

```bash
# 查看提交历史，找到目标提交的哈希值
git log --oneline

# 软回退：保留工作目录和暂存区的修改
git reset --soft <提交a的哈希值>

# 混合回退：保留工作目录修改，清空暂存区（默认方式）
git reset --mixed <提交a的哈希值>
# 简写为：
git reset <提交a的哈希值>

# 硬回退：完全恢复到提交a，丢弃所有后续修改
git reset --hard <提交a的哈希值>
```

**使用场景：**
- `--soft`：想重新组织提交，保留所有修改
- `--mixed`：想重新暂存文件，保留工作目录修改
- `--hard`：完全放弃后续修改，回到干净状态

#### 方法二：git revert（安全回退）

```bash
# 创建新提交来撤销指定提交的修改
git revert <提交c的哈希值>
git revert <提交b的哈希值>

# 批量撤销多个提交
git revert <提交a的哈希值>..<提交c的哈希值>

# 撤销合并提交
git revert -m 1 <合并提交的哈希值>
```

**优点：**
- 不会改变历史记录
- 适合已推送到远程的情况
- 可以随时撤销revert操作

#### 方法三：git checkout（临时查看）

```bash
# 临时切换到指定提交（分离HEAD状态）
git checkout <提交a的哈希值>

# 基于该提交创建新分支
git checkout -b recovery-branch <提交a的哈希值>

# 返回原分支
git checkout main
```

### 恢复误删的提交

```bash
# 查看所有操作历史（包括已删除的提交）
git reflog

# 恢复到指定的提交
git reset --hard HEAD@{n}

# 或者使用提交哈希
git reset --hard <提交哈希>
```

### 恢复单个文件到指定版本

```bash
# 恢复文件到指定提交的版本
git checkout <提交哈希> -- <文件路径>

# 恢复文件到上一个提交的版本
git checkout HEAD~1 -- <文件路径>

# 恢复已删除的文件
git checkout <删除前的提交哈希> -- <文件路径>
```

## 常见问题解决

### 1. 推送被拒绝

**错误信息：** `Updates were rejected because the remote contains work that you do not have locally`

**解决方案：**
```bash
# 方案一：拉取并合并（推荐）
git pull
# 解决冲突后再推送
git push

# 方案二：拉取并变基（保持线性历史）
git pull --rebase
git push

# 方案三：强制推送（危险，谨慎使用）
git push --force
```

### 2. 合并冲突详解

**冲突标记说明：**
```
<<<<<<< HEAD
当前分支的内容
=======
要合并分支的内容
>>>>>>> 分支名
```

**解决步骤：**
```bash
# 1. 查看冲突文件
git status

# 2. 编辑冲突文件，选择要保留的内容
# 删除 <<<<<<<、=======、>>>>>>> 标记

# 3. 标记冲突已解决
git add <冲突文件>

# 4. 完成合并
git commit -m "解决合并冲突"

# 如果想取消合并
git merge --abort
```

**使用合并工具：**
```bash
# 配置合并工具（Windows推荐）
git config --global merge.tool vimdiff
# 或使用VS Code
git config --global merge.tool vscode
git config --global mergetool.vscode.cmd 'code --wait $MERGED'

# 启动合并工具
git mergetool
```

### 3. Windows系统特有问题

**路径分隔符问题：**
```bash
# 配置Git处理路径分隔符
git config --global core.autocrlf true
git config --global core.safecrlf false
```

**中文文件名显示问题：**
```bash
# 正确显示中文文件名
git config --global core.quotepath false
```

**PowerShell编码问题：**
```powershell
# 在PowerShell中设置UTF-8编码
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
```

### 4. 凭据管理问题

**清除Windows凭据：**
1. 按 `Win + R`，输入 `control`
2. 进入「用户账户」→「凭据管理器」
3. 删除GitHub相关的凭据
4. 重新推送时输入新的Token

**或使用命令行：**
```bash
# 清除存储的凭据
git config --global --unset credential.helper

# 重新配置凭据助手
git config --global credential.helper manager-core
```

### 5. 大文件处理

**移除大文件：**
```bash
# 从历史记录中完全删除大文件
git filter-branch --force --index-filter \
  'git rm --cached --ignore-unmatch <大文件路径>' \
  --prune-empty --tag-name-filter cat -- --all

# 清理和回收空间
git for-each-ref --format='delete %(refname)' refs/original | git update-ref --stdin
git reflog expire --expire=now --all
git gc --prune=now
```

**使用Git LFS处理大文件：**
```bash
# 安装Git LFS
git lfs install

# 跟踪大文件类型
git lfs track "*.psd"
git lfs track "*.zip"

# 提交.gitattributes文件
git add .gitattributes
git commit -m "添加Git LFS支持"
```

### 6. 提交信息和历史修改

```bash
# 修改最后一次提交信息（未推送）
git commit --amend -m "正确的提交信息"

# 修改最后一次提交内容（添加遗漏文件）
git add <遗漏的文件>
git commit --amend --no-edit

# 交互式修改多个提交
git rebase -i HEAD~3

# 修改提交作者信息
git commit --amend --author="新作者 <email@example.com>"
```

### 7. 网络和代理问题

```bash
# 设置HTTP代理
git config --global http.proxy http://proxy.company.com:8080
git config --global https.proxy https://proxy.company.com:8080

# 取消代理设置
git config --global --unset http.proxy
git config --global --unset https.proxy

# 设置Git使用系统代理
git config --global http.proxy 'socks5://127.0.0.1:1080'

# 只对GitHub设置代理
git config --global http.https://github.com.proxy socks5://127.0.0.1:1080
```

## 高级技巧与优化

### 1. Git别名配置

```bash
# 配置常用命令别名
git config --global alias.st status
git config --global alias.co checkout
git config --global alias.br branch
git config --global alias.ci commit
git config --global alias.unstage 'reset HEAD --'
git config --global alias.last 'log -1 HEAD'
git config --global alias.visual '!gitk'

# 美化日志显示
git config --global alias.lg "log --color --graph --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset' --abbrev-commit"

# 使用别名
git st          # 等同于 git status
git lg          # 美化的日志显示
```

### 2. 交互式暂存

```bash
# 交互式添加文件部分内容
git add -p <文件名>

# 交互式暂存所有修改
git add -i

# 只暂存文件的部分行
git add --patch
```

### 3. 储藏（Stash）功能

```bash
# 储藏当前工作进度
git stash

# 储藏时添加描述
git stash save "工作进度描述"

# 查看储藏列表
git stash list

# 应用最新的储藏
git stash apply

# 应用指定的储藏
git stash apply stash@{2}

# 应用储藏并删除
git stash pop

# 删除储藏
git stash drop stash@{0}

# 清空所有储藏
git stash clear

# 储藏包括未跟踪的文件
git stash -u
```

### 4. 标签管理

```bash
# 创建轻量标签
git tag v1.0

# 创建带注释的标签
git tag -a v1.0 -m "版本1.0发布"

# 为指定提交创建标签
git tag -a v0.9 <提交哈希> -m "版本0.9"

# 查看所有标签
git tag

# 查看标签信息
git show v1.0

# 推送标签到远程
git push origin v1.0
git push origin --tags

# 删除本地标签
git tag -d v1.0

# 删除远程标签
git push origin --delete tag v1.0
```

### 5. 子模块管理

```bash
# 添加子模块
git submodule add https://github.com/user/repo.git path/to/submodule

# 克隆包含子模块的项目
git clone --recursive <主项目URL>

# 初始化子模块
git submodule init
git submodule update

# 更新子模块
git submodule update --remote

# 删除子模块
git submodule deinit path/to/submodule
git rm path/to/submodule
```

## 团队协作最佳实践

### 1. 提交信息规范

**约定式提交格式：**
```
<类型>[可选的作用域]: <描述>

[可选的正文]

[可选的脚注]
```

**类型说明：**
- `feat`: 新功能
- `fix`: 修复bug
- `docs`: 文档更新
- `style`: 代码格式调整
- `refactor`: 重构代码
- `test`: 添加测试
- `chore`: 构建过程或辅助工具的变动

**示例：**
```bash
git commit -m "feat(用户模块): 添加用户注册功能"
git commit -m "fix: 修复登录页面样式问题"
git commit -m "docs: 更新API文档"
```

### 2. 分支策略详解

**Git Flow 工作流：**
```bash
# 主要分支
main/master     # 生产环境代码
develop         # 开发环境代码

# 辅助分支
feature/*       # 功能开发分支
release/*       # 发布准备分支
hotfix/*        # 紧急修复分支

# 创建功能分支
git checkout -b feature/user-login develop

# 完成功能后合并到develop
git checkout develop
git merge --no-ff feature/user-login
git branch -d feature/user-login

# 创建发布分支
git checkout -b release/1.0.0 develop

# 发布完成后合并到main和develop
git checkout main
git merge --no-ff release/1.0.0
git tag -a v1.0.0 -m "版本1.0.0"
git checkout develop
git merge --no-ff release/1.0.0
```

**GitHub Flow（简化版）：**
```bash
# 1. 从main创建功能分支
git checkout -b feature/new-feature main

# 2. 开发并提交
git add .
git commit -m "实现新功能"

# 3. 推送分支
git push -u origin feature/new-feature

# 4. 在GitHub创建Pull Request
# 5. 代码审查通过后合并到main
# 6. 删除功能分支
git branch -d feature/new-feature
git push origin --delete feature/new-feature
```

### 3. 代码审查流程

**Pull Request最佳实践：**
1. **清晰的标题和描述**
2. **小而专注的变更**
3. **包含测试用例**
4. **更新相关文档**
5. **自我审查代码**

**审查者指南：**
- 检查代码逻辑和性能
- 验证测试覆盖率
- 确保符合编码规范
- 提供建设性反馈

### 4. 冲突预防策略

```bash
# 定期同步主分支
git checkout feature/my-feature
git fetch origin
git rebase origin/main

# 使用rebase保持线性历史
git pull --rebase origin main

# 小步提交，频繁推送
git add .
git commit -m "完成部分功能"
git push
```

## 性能优化与维护

### 1. 仓库清理

```bash
# 清理不必要的文件和优化仓库
git gc --aggressive --prune=now

# 查看仓库大小
git count-objects -vH

# 清理远程跟踪分支
git remote prune origin

# 清理本地分支
git branch --merged | grep -v "\*\|main\|develop" | xargs -n 1 git branch -d
```

### 2. 大仓库优化

```bash
# 浅克隆（只获取最近的提交）
git clone --depth 1 <仓库URL>

# 部分克隆（只获取指定分支）
git clone --single-branch --branch main <仓库URL>

# 稀疏检出（只检出部分文件）
git config core.sparseCheckout true
echo "src/" >> .git/info/sparse-checkout
git read-tree -m -u HEAD
```

### 3. 钩子（Hooks）应用

**预提交钩子示例：**
```bash
# .git/hooks/pre-commit
#!/bin/sh
# 检查代码格式
npm run lint
if [ $? -ne 0 ]; then
    echo "代码格式检查失败，请修复后再提交"
    exit 1
fi
```

**提交信息钩子：**
```bash
# .git/hooks/commit-msg
#!/bin/sh
# 检查提交信息格式
if ! grep -qE "^(feat|fix|docs|style|refactor|test|chore)(\(.+\))?: .+" "$1"; then
    echo "提交信息格式不正确，请使用约定式提交格式"
    exit 1
fi
```

## 最佳实践

### 1. 项目初始化最佳实践

#### 使用项目模板
创建一个标准的项目模板目录：
```bash
# 创建模板目录
mkdir project-template
cd project-template

# 添加常用文件
echo "# 项目名称" > README.md
echo "*.log" > .gitignore
echo "node_modules/" >> .gitignore
echo "__pycache__/" >> .gitignore
echo ".env" >> .gitignore

# 初始化Git
git init
git add .
git commit -m "feat: 创建项目模板"
```

#### 自动化脚本
创建新项目初始化脚本（Windows PowerShell）：
```powershell
# new-project.ps1
param([string]$ProjectName)

if (-not $ProjectName) {
    Write-Host "请提供项目名称: .\new-project.ps1 项目名称"
    exit
}

# 创建项目目录
New-Item -ItemType Directory -Name $ProjectName
Set-Location $ProjectName

# 初始化Git
git init

# 创建基本文件
"# $ProjectName" | Out-File -FilePath "README.md" -Encoding UTF8
"*.log`nnode_modules/`n__pycache__/`n.env" | Out-File -FilePath ".gitignore" -Encoding UTF8

# 提交初始文件
git add .
git commit -m "feat: 初始化 $ProjectName 项目"

Write-Host "项目 $ProjectName 初始化完成！"
Write-Host "下一步："
Write-Host "1. 在GitHub创建同名仓库"
Write-Host "2. 运行: git remote add origin https://github.com/你的用户名/$ProjectName.git"
Write-Host "3. 运行: git push -u origin main"
```

#### IDE集成
- **VS Code**: 使用Git扩展，可以直接在界面中初始化仓库
- **PyCharm**: VCS → Import into Version Control → Create Git Repository
- **命令行工具**: 使用`gh`（GitHub CLI）快速创建仓库

### 2. 日常开发习惯

- **频繁提交**：小步快跑，每完成一个小功能就提交
- **有意义的提交信息**：清楚描述这次提交做了什么
- **原子性提交**：每次提交只做一件事
- **提交前自检**：使用 `git diff --staged` 检查暂存内容

### 2. 分支管理原则

- **保持main分支稳定**：不要直接在main分支开发
- **使用功能分支**：为每个新功能创建独立分支
- **及时删除无用分支**：合并后删除功能分支
- **定期同步主分支**：避免分支偏离太远

### 3. .gitignore文件

创建`.gitignore`文件，忽略不需要版本控制的文件：

```gitignore
# 操作系统文件
.DS_Store
Thumbs.db

# 编辑器文件
.vscode/
.idea/
*.swp
*.swo

# 依赖目录
node_modules/
__pycache__/
*.pyc

# 构建输出
dist/
build/
*.exe

# 日志文件
*.log

# 环境配置
.env
config.local.js

# 临时文件
*.tmp
*.temp
```

### 4. 安全注意事项

- **不要提交敏感信息**：密码、API密钥、个人信息等
- **使用.gitignore**：确保敏感文件不被跟踪
- **定期更新Token**：设置合理的过期时间
- **不要在公共场所输入凭据**：避免被他人看到

### 5. 协作建议

- **拉取前推送**：推送前先拉取最新代码
- **解决冲突要谨慎**：确保不会覆盖他人的工作
- **使用Pull Request**：在GitHub上使用PR进行代码审查
- **保持沟通**：与团队成员及时沟通代码变更

---

## 🆘 快速故障排除

> 🔍 **快速查找**: 按 `Ctrl+F` 搜索错误关键词

### ❌ 常见错误信息及解决方案

| 🚨 错误信息 | 🔍 原因 | ✅ 解决方案 | 🔥 频率 |
|-------------|---------|-------------|----------|
| `fatal: not a git repository` | 当前目录不是Git仓库 | `git init` 或切换到正确目录 | ⭐⭐⭐⭐⭐ |
| `error: pathspec did not match any file(s)` | 文件路径错误或文件不存在 | 检查文件路径和文件名 | ⭐⭐⭐⭐ |
| `fatal: remote origin already exists` | 远程仓库已存在 | `git remote remove origin` 后重新添加 | ⭐⭐⭐⭐ |
| `Permission denied (publickey)` | SSH密钥问题 | 使用HTTPS或配置SSH密钥 | ⭐⭐⭐⭐ |
| `fatal: refusing to merge unrelated histories` | 合并无关历史 | `git pull --allow-unrelated-histories` | ⭐⭐⭐ |
| `error: Your local changes would be overwritten` | 本地有未提交的修改 | 先提交或储藏本地修改 | ⭐⭐⭐⭐ |
| `error: src refspec main does not exist` | 本地没有main分支或没有提交 | 创建第一次提交或检查分支名 | ⭐⭐⭐ |
| `Reinitialized existing Git repository` | 在已有Git仓库中重复初始化 | 检查是否已经是Git仓库 | ⭐⭐ |
| `Updates were rejected` | 远程有新提交 | `git pull` 然后 `git push` | ⭐⭐⭐⭐ |
| `merge conflict` | 合并冲突 | 手动解决冲突后 `git add` 和 `git commit` | ⭐⭐⭐ |

### 🚑 紧急恢复命令

> ⚠️ **警告**: 以下命令可能会丢失数据，使用前请确认！

#### 🔄 文件恢复
```bash
# 恢复误删的文件
git checkout HEAD -- <文件名>

# 恢复到上一次提交状态
git reset --hard HEAD

# 恢复指定文件到指定版本
git checkout <提交哈希> -- <文件名>
```

#### 🔍 查找丢失的提交
```bash
# 查找丢失的提交
git reflog
git reset --hard <提交哈希>

# 恢复误删的分支
git reflog
git checkout -b <分支名> <提交哈希>
```

#### 📁 仓库恢复
```bash
# 恢复误删的.git文件夹（如果有远程备份）
git clone https://github.com/username/repo.git temp-repo
cp -r temp-repo/.git ./
rm -rf temp-repo

# 重新初始化丢失的仓库（⚠️ 会丢失历史记录）
git init
git add .
git commit -m "重新初始化项目"
git remote add origin https://github.com/username/repo.git
```

#### ⚡ 快速撤销
```bash
# 撤销最近的合并
git reset --hard HEAD~1

# 撤销推送（🚨 危险操作）
git reset --hard HEAD~1
git push --force

# 临时保存当前工作
git stash
# 恢复保存的工作
git stash pop
```

---

## 📖 常用命令速查

> 💡 **使用提示**: 按 `Ctrl+F` 搜索命令关键词快速定位

### 🆕 项目初始化命令

| 🎯 命令 | 📝 说明 | 💻 示例 |
|---------|---------|----------|
| `git init` | 在当前目录初始化Git仓库 | `git init` |
| `git init <name>` | 创建新目录并初始化 | `git init my-project` |
| `git clone <url>` | 克隆远程仓库（自动初始化） | `git clone https://github.com/user/repo.git` |
| `ls -la` | 查看是否有.git文件夹 | `ls -la` |
| `git status` | 检查Git状态 | `git status` |
| `git remote -v` | 查看远程仓库配置 | `git remote -v` |
| `git config user.name` | 设置项目用户名 | `git config user.name "张三"` |
| `git config user.email` | 设置项目邮箱 | `git config user.email "zhangsan@example.com"` |

### ⚙️ 基础命令

| 🎯 命令 | 📝 说明 | 💻 示例 | 🔥 频率 |
|---------|---------|----------|----------|
| `git add <file>` | 添加文件到暂存区 | `git add index.html` | ⭐⭐⭐⭐⭐ |
| `git add .` | 添加所有文件到暂存区 | `git add .` | ⭐⭐⭐⭐⭐ |
| `git commit -m "message"` | 提交更改 | `git commit -m "添加新功能"` | ⭐⭐⭐⭐⭐ |
| `git push` | 推送到远程仓库 | `git push origin main` | ⭐⭐⭐⭐ |
| `git pull` | 拉取远程更改 | `git pull origin main` | ⭐⭐⭐⭐ |
| `git status` | 查看仓库状态 | `git status` | ⭐⭐⭐⭐⭐ |
| `git diff` | 查看文件差异 | `git diff` | ⭐⭐⭐ |

### 🌿 分支操作

| 🎯 命令 | 📝 说明 | 💻 示例 | 🔥 频率 |
|---------|---------|----------|----------|
| `git branch` | 查看分支 | `git branch -a` | ⭐⭐⭐⭐ |
| `git checkout <branch>` | 切换分支 | `git checkout develop` | ⭐⭐⭐⭐ |
| `git checkout -b <branch>` | 创建并切换分支 | `git checkout -b feature/login` | ⭐⭐⭐⭐ |
| `git merge <branch>` | 合并分支 | `git merge feature/login` | ⭐⭐⭐ |
| `git branch -d <branch>` | 删除分支 | `git branch -d feature/login` | ⭐⭐⭐ |
| `git push origin --delete <branch>` | 删除远程分支 | `git push origin --delete feature/login` | ⭐⭐ |

### 📚 历史查看

| 🎯 命令 | 📝 说明 | 💻 示例 | 🔥 频率 |
|---------|---------|----------|----------|
| `git log` | 查看提交历史 | `git log --oneline` | ⭐⭐⭐⭐ |
| `git log --graph` | 图形化显示历史 | `git log --graph --oneline` | ⭐⭐⭐ |
| `git diff` | 查看文件差异 | `git diff HEAD~1` | ⭐⭐⭐ |
| `git show <commit>` | 查看提交详情 | `git show abc123` | ⭐⭐ |
| `git blame <file>` | 查看文件修改历史 | `git blame index.html` | ⭐⭐ |
| `git reflog` | 查看操作历史 | `git reflog` | ⭐⭐ |

### ↩️ 撤销操作

| 🎯 命令 | 📝 说明 | 💻 示例 | ⚠️ 风险 |
|---------|---------|----------|----------|
| `git reset --soft HEAD~1` | 软撤销提交（保留修改） | `git reset --soft HEAD~1` | 🟡 低 |
| `git reset --hard HEAD~1` | 硬撤销提交（丢弃修改） | `git reset --hard HEAD~1` | 🔴 高 |
| `git revert <commit>` | 安全撤销提交 | `git revert abc123` | 🟢 无 |
| `git checkout -- <file>` | 撤销文件修改 | `git checkout -- index.html` | 🟡 中 |
| `git reset HEAD <file>` | 取消暂存文件 | `git reset HEAD index.html` | 🟢 无 |

### 🌐 远程操作

| 🎯 命令 | 📝 说明 | 💻 示例 | 🔥 频率 |
|---------|---------|----------|----------|
| `git remote -v` | 查看远程仓库 | `git remote -v` | ⭐⭐⭐ |
| `git remote add <name> <url>` | 添加远程仓库 | `git remote add origin <URL>` | ⭐⭐⭐ |
| `git fetch` | 获取远程更新 | `git fetch origin` | ⭐⭐⭐ |
| `git push -u origin <branch>` | 推送并设置上游 | `git push -u origin main` | ⭐⭐⭐ |
| `git clone <url>` | 克隆远程仓库 | `git clone https://github.com/user/repo.git` | ⭐⭐⭐⭐ |

### 🆘 紧急救援命令

| 🎯 命令 | 📝 说明 | 💻 示例 | 🚨 紧急度 |
|---------|---------|----------|----------|
| `git stash` | 临时保存工作进度 | `git stash` | 🟡 中 |
| `git stash pop` | 恢复保存的工作进度 | `git stash pop` | 🟡 中 |
| `git reflog` | 查找丢失的提交 | `git reflog` | 🔴 高 |
| `git reset --hard <commit>` | 强制回到指定提交 | `git reset --hard abc123` | 🔴 高 |
| `git checkout HEAD -- <file>` | 恢复文件到最新提交状态 | `git checkout HEAD -- index.html` | 🟡 中 |

---

## 📚 快速索引

### 🔍 按场景查找

| 🎯 我想要... | 📍 跳转到 | 🔥 重要度 |
|-------------|-----------|----------|
| 第一次使用Git | [新手快速入门](#-新手快速入门) | ⭐⭐⭐⭐⭐ |
| 创建新项目 | [项目初始化](#新项目完整初始化流程) | ⭐⭐⭐⭐⭐ |
| 连接GitHub | [HTTPS配置GitHub](#https方式配置github) | ⭐⭐⭐⭐⭐ |
| 解决推送问题 | [推送被拒绝](#1-推送被拒绝) | ⭐⭐⭐⭐ |
| 解决合并冲突 | [合并冲突详解](#2-合并冲突详解) | ⭐⭐⭐⭐ |
| 撤销错误操作 | [撤销操作](#撤销操作) | ⭐⭐⭐⭐ |
| 找回丢失的代码 | [紧急恢复命令](#-紧急恢复命令) | ⭐⭐⭐ |
| 查看命令用法 | [常用命令速查](#-常用命令速查) | ⭐⭐⭐⭐⭐ |
| 学习高级技巧 | [高级技巧与优化](#高级技巧与优化) | ⭐⭐⭐ |
| 团队协作规范 | [团队协作最佳实践](#团队协作最佳实践) | ⭐⭐⭐ |

### 🚨 紧急情况处理

| 🆘 紧急情况 | 🚑 快速解决 |
|-------------|-------------|
| 代码丢失了 | `git reflog` → `git reset --hard <哈希>` |
| 推送失败 | `git pull` → 解决冲突 → `git push` |
| 提交错了 | `git reset --soft HEAD~1` → 重新提交 |
| 分支搞错了 | `git checkout <正确分支>` |
| 想撤销修改 | `git checkout -- <文件>` 或 `git reset --hard HEAD` |

### 💡 学习路径建议

1. **🚀 新手阶段** (1-2周)
   - [Git简介](#git简介) → [基础概念](#基础概念) → [新手快速入门](#-新手快速入门)
   - 重点掌握：`git add`, `git commit`, `git push`, `git pull`

2. **💻 进阶阶段** (2-4周)
   - [分支管理](#分支管理) → [版本回退与恢复](#版本回退与恢复)
   - 重点掌握：分支操作、冲突解决、历史管理

3. **🎯 高级阶段** (1-2个月)
   - [高级技巧与优化](#高级技巧与优化) → [团队协作最佳实践](#团队协作最佳实践)
   - 重点掌握：工作流程、代码审查、性能优化

---

## 🎉 结语

🎯 **恭喜你！** 现在你已经掌握了Git的完整使用指南。

### 📈 持续提升建议

- 🏃‍♂️ **多练习**：通过实际项目练习Git操作
- 💾 **多备份**：重要代码要及时推送到远程仓库
- 📚 **多学习**：Git功能丰富，持续学习新特性
- 🤝 **多交流**：与其他开发者交流Git使用经验
- 🔧 **善用工具**：结合IDE和图形化工具提高效率

### 🆘 遇到问题时

1. 先查看 [快速故障排除](#-快速故障排除)
2. 使用 `Ctrl+F` 搜索关键词
3. 查阅 [常用命令速查](#-常用命令速查)
4. 不要害怕尝试，Git很难真正"搞坏"项目

> 💡 **记住**: Git是你的朋友，不是敌人。熟练掌握后，它将成为你开发过程中最得力的助手！

---

**📝 文档版本**: v2.0 | **🔄 最后更新**: 2024年 | **👨‍💻 适用于**: Windows 11 + HTTPS方式