<#
    一键检查点：git add -A -> git commit -> 自动推送（由 .githooks/post-commit 完成）

    用法:
        pwsh -File tools/checkpoint.ps1 "fix(pid): 修正积分饱和"
        pwsh -File tools/checkpoint.ps1          # 用默认提交信息

    提交信息格式见 AGENTS.md：<type>(<scope>): <中文描述>
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$Message
)

$ErrorActionPreference = 'Stop'

try {
    [Console]::OutputEncoding = [System.Text.Encoding]::UTF8
    $OutputEncoding          = [System.Text.Encoding]::UTF8
} catch { }

$repoRoot = (& git rev-parse --show-toplevel 2>$null)
if (-not $repoRoot) {
    Write-Host '[!] 当前目录不在 git 仓库里。' -ForegroundColor Red
    exit 1
}
Set-Location $repoRoot.Trim()

$dirty = & git status --porcelain
if (-not $dirty) {
    Write-Host '[i] 工作区干净，没有需要提交的改动。' -ForegroundColor Yellow
    exit 0
}

Write-Host '将要提交的改动:'
& git status --short
Write-Host ''

if (-not $Message) {
    $Message = "chore: 检查点 $(Get-Date -Format 'yyyy-MM-dd HH:mm')"
}
Write-Host "提交信息: $Message"
Write-Host ''

& git add -A
& git commit -m $Message
if ($LASTEXITCODE -ne 0) {
    Write-Host ''
    Write-Host '[!] 提交未成功。若被 pre-commit 拦截，请按上面提示处理；' -ForegroundColor Red
    Write-Host '    确认要提交时可用:  git commit --no-verify -m "<信息>"'
    exit $LASTEXITCODE
}

Write-Host ''
Write-Host '[ok] 已提交。post-commit 钩子会尝试推送到远端。' -ForegroundColor Green
