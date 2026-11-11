param([string]$UnityEditorPath='C:\Program Files\Unity\Hub\Editor\2022.3.46f1')
$ErrorActionPreference='Stop'
$taskRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$taskBuild=Join-Path ([IO.Path]::GetTempPath()) ('transition-geometry-'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $taskBuild | Out-Null
$taskMono=Join-Path $UnityEditorPath 'Editor\Data\MonoBleedingEdge\bin\mono.exe'
$taskCsc=Join-Path $UnityEditorPath 'Editor\Data\MonoBleedingEdge\lib\mono\4.5\csc.exe'
$taskVersion=[Diagnostics.FileVersionInfo]::GetVersionInfo((Join-Path $UnityEditorPath 'Editor\Unity.exe')).ProductVersion.Split('_')[0]
if($taskVersion -ne '2022.3.46f1'){throw 'Pinned Unity 2022.3.46f1 required.'}
$taskSources=@(Get-ChildItem (Join-Path $taskRoot 'Assets\Scripts\ThreePhase3D') -Filter '*.cs' -Recurse | Where-Object {$_.FullName -notmatch '[\\/](Editor|Rendering)[\\/]'} | ForEach-Object {$_.FullName})
$taskExe=Join-Path $taskBuild 'GeometryChecks.exe'
$taskArgs=@('/nologo','/target:exe',('/out:'+$taskExe),'/reference:System.Runtime.Serialization.dll','/reference:System.Numerics.dll')+$taskSources+(Join-Path $PSScriptRoot 'GeometryMain.cs')
& $taskMono $taskCsc @taskArgs
if($LASTEXITCODE -ne 0){throw 'Geometry compilation failed.'}
& $taskMono $taskExe $taskRoot
if($LASTEXITCODE -ne 0){throw 'Geometry verification failed.'}
