param(
    [string]$UnityEditorPath = 'C:\Program Files\Unity\Hub\Editor\2022.3.46f1',
    [string]$OutputPath = '',
    [switch]$CompileUnitySources,
    [string]$BundleOutputRoot = '',
    [string]$VerifyBundle = ''
)
$ErrorActionPreference = 'Stop'
$taskRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$taskBuild = Join-Path ([IO.Path]::GetTempPath()) ('transition-sim-threephase3d-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $taskBuild | Out-Null
$taskMono = Join-Path $UnityEditorPath 'Editor\Data\MonoBleedingEdge\bin\mono.exe'
$taskCsc = Join-Path $UnityEditorPath 'Editor\Data\MonoBleedingEdge\lib\mono\4.5\csc.exe'
$taskEditorVersion = [Diagnostics.FileVersionInfo]::GetVersionInfo((Join-Path $UnityEditorPath 'Editor\Unity.exe')).ProductVersion.Split('_')[0]
if ($taskEditorVersion -ne '2022.3.46f1') { throw ('Expected pinned Unity 2022.3.46f1; found ' + $taskEditorVersion) }
$taskSources = @(Get-ChildItem (Join-Path $taskRoot 'Assets\Scripts\ThreePhase3D') -Filter '*.cs' -Recurse |
    Where-Object { $_.FullName -notmatch '[\\/](Editor|Rendering)[\\/]' } | ForEach-Object { $_.FullName })
$taskExe = Join-Path $taskBuild 'FoundationChecks.exe'
$taskArgs = @('/nologo', '/target:exe', ('/out:' + $taskExe), '/reference:System.Runtime.Serialization.dll', '/reference:System.Numerics.dll') + $taskSources + (Join-Path $PSScriptRoot 'CheckMain.cs')
& $taskMono $taskCsc @taskArgs
if ($LASTEXITCODE -ne 0) { throw 'Foundation compilation failed.' }
if ($CompileUnitySources) {
    $taskRefs = @(Get-ChildItem (Join-Path $UnityEditorPath 'Editor\Data\NetStandard\ref\2.1.0') -Filter '*.dll' | ForEach-Object { '/reference:' + $_.FullName })
    $taskRefs += @(Get-ChildItem (Join-Path $UnityEditorPath 'Editor\Data\Managed\UnityEngine') -Filter '*.dll' | ForEach-Object { '/reference:' + $_.FullName })
    $taskRefs += @(Get-ChildItem (Join-Path $UnityEditorPath 'Editor\Data\Managed') -Filter 'UnityEditor*.dll' |
        Where-Object { $_.Name -ne 'UnityEditor.dll' } | ForEach-Object { '/reference:' + $_.FullName })
    $taskAllSources = @(Get-ChildItem (Join-Path $taskRoot 'Assets\Scripts\ThreePhase3D') -Filter '*.cs' -Recurse | ForEach-Object { $_.FullName })
    $taskCompileArgs = @('/nologo', '/noconfig', '/nostdlib+', '/target:library', ('/out:' + (Join-Path $taskBuild 'ThreePhase3D.dll'))) + $taskRefs + $taskAllSources
    & $taskMono $taskCsc @taskCompileArgs
    if ($LASTEXITCODE -ne 0) { throw 'Unity API compilation failed.' }
    Write-Output 'PASS C# compile against installed Unity 2022.3 assemblies (not an Editor import or runtime test).'
}
if (-not [string]::IsNullOrWhiteSpace($VerifyBundle)) {
    & $taskMono $taskExe '--verify' ([IO.Path]::GetFullPath($VerifyBundle)) $taskRoot
} else {
    if ([string]::IsNullOrWhiteSpace($BundleOutputRoot)) { $BundleOutputRoot = Join-Path $taskRoot 'ValidationRuns\local' }
    $taskResultOutput = if ([string]::IsNullOrWhiteSpace($OutputPath)) { '' } else { [IO.Path]::GetFullPath($OutputPath) }
    & $taskMono $taskExe $taskRoot ([IO.Path]::GetFullPath($BundleOutputRoot)) $taskResultOutput
}
if ($LASTEXITCODE -ne 0) { throw 'Foundation checks failed.' }
