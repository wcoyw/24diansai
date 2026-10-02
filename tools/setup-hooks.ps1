<#
    启用本仓库的版本化 git hooks。

    每个克隆 / 每台机器执行一次：
        pwsh -File tools/setup-hooks.ps1

    作用：把 core.hooksPath 指向 .githooks，从而启用
      - pre-commit  拦截编译产物、生成物与超大文件
      - post-commit 每次提交后自动推送

    脚本最后会用 git 自己的执行路径跑一次 pre-commit 自检，
    确认钩子真的能被执行，而不是配上了却让每次提交都被拦住。
#>
[CmdletBinding()]
param()

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
$repoRoot = $repoRoot.Trim()
Set-Location $repoRoot
Write-Host "仓库根目录: $repoRoot"

$hooksRel = '.githooks'
$hooksAbs = Join-Path $repoRoot $hooksRel
if (-not (Test-Path $hooksAbs)) {
    Write-Host "[!] 找不到 $hooksRel/，无法启用。" -ForegroundColor Red
    exit 1
}

& git config --local core.hooksPath $hooksRel
if ($LASTEXITCODE -ne 0) {
    Write-Host '[!] 设置 core.hooksPath 失败。' -ForegroundColor Red
    exit 1
}
Write-Host "[ok] core.hooksPath = $hooksRel" -ForegroundColor Green

Write-Host ''
Write-Host '已启用的 hooks:'
Get-ChildItem -LiteralPath $hooksAbs -File | ForEach-Object {
    Write-Host ("  {0}" -f $_.Name)
}

Write-Host ''
Write-Host '配置位置与当前值:'
Write-Host ("  git config --get core.hooksPath  ->  " + (& git config --get core.hooksPath))

Write-Host ''
Write-Host '自检: 用 git 自己的执行路径跑一次 pre-commit（此刻没有暂存改动，应当直接通过）...'
& git hook run pre-commit
$hookExit = $LASTEXITCODE

Write-Host ''
if ($hookExit -eq 0) {
    Write-Host '[ok] pre-commit 能正常执行，钩子已生效。' -ForegroundColor Green
} else {
    Write-Host "[!] pre-commit 自检未通过（exit $hookExit）。" -ForegroundColor Red
    Write-Host '    在修好之前，提交仍可进行：  git commit --no-verify -m "<信息>"'
    Write-Host '    请把上面的输出交给 agent 处理。'
}

Write-Host ''
Write-Host '接下来:'
Write-Host '  - post-commit 会在每次提交后自动推送，无法在此处空跑（会真的推）。'
Write-Host '  - 用一次真实提交验证:  pwsh -File tools/checkpoint.ps1 "chore: 验证钩子"'
Write-Host '  - 新分支首次推送:      git push -u origin <分支名>'
Write-Host '  - 临时关闭自动推送:    $env:DSH_NO_AUTOPUSH=1'
