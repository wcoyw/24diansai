# 仓库协作约定

本文件约束所有在本仓库工作的 AI agent（Codex / Claude Code / Cursor / DSH 等）。

**版本状态以 git 为唯一事实来源。** 回退走 git，恢复文件走 git，改动历史写在 commit message 里。

## 每个检查点

一个**检查点** = 一次可验证的小步完成。完成的信号：编译通过、功能跑通、某个 bug 修好、一次实验做完。

1. 动手前先看状态：

       git status --short

   工作区里有与本次任务无关的改动时，先给它们一个提交（`chore: 保存当前进度`），拿到干净的回退锚点。

2. 小步完成即提交，不要攒到最后：

       git add -A
       git commit -m "fix(pid): 修正积分饱和导致的超调"

3. 提交后由 `.githooks/post-commit` 自动推送到远端。这一步让历史离开本机，
   换会话、换机器、换 agent 都能靠 git 回退。

   推送失败时把**原始报错**交给用户，不要静默重试、不要擅自换远端、不要改认证配置。

提交信息格式：`<type>(<scope>): <中文描述>`

- type：`feat` / `fix` / `refactor` / `chore` / `docs` / `test`
- scope 可省略，例如 `fix: 修复 JY61P 串口丢帧`

## 回退

1. 撤销某次提交、保留历史：`git revert <sha>`
2. 单个文件回到历史版本：`git checkout <sha> -- <路径>`
3. 看现在与某个历史点的差异：`git diff <sha> -- <路径>`
4. 丢弃未提交的改动：`git restore <路径>`
5. 查看历史：`git log --oneline`、`git show <sha>`

`git reset --hard`、`git push --force`、改写已推送历史，只在用户明确点名时使用。

## 什么不进版本控制

规则见 `.gitignore`。出现新的产物类型时补进 `.gitignore`，并在提交信息里写明原因。

SysConfig 是引脚、外设、时钟、中断的来源：改 `.syscfg` 后重新生成，
生成出来的 `ti_msp_dl_config.c` / `.h`、`device_linker.cmd`、`Objects/`、`Listings/` 都保持在版本控制之外。

## 工程结构

- `24diansai/`：STM32F103 + Keil MDK 主工程（循迹小车）
- `mspm0_24diansai/`：MSPM0G3507 + CCS 工程
- `CAR_Keil/`：MSPM0G3507 CAN / IMU 参考工程；自写代码在 `project/user`，`libraries/` 是厂商 SDK
- 根目录 `*.md` / `*.txt`：调试记录、引脚表、移植计划（`第N次修改说明.md` 是历史存档，保留原样）

## 新机器 / 新克隆

    pwsh -File tools/setup-hooks.ps1

启用 `.githooks/`（提交拦截 + 自动推送）。每个克隆执行一次，否则 hooks 不生效。

## 一键检查点

    pwsh -File tools/checkpoint.ps1 "fix(pid): 修正积分饱和"

等价于 `git add -A` + `git commit` + 自动推送。在 VSCode 里也可以用
命令面板的 `Tasks: Run Task` → `检查点：提交并推送`。
