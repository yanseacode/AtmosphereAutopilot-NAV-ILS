$ErrorActionPreference = 'Stop'

$msbuild = 'C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe'
if (-not (Test-Path -LiteralPath $msbuild)) { throw 'MSBuild 2022 Community was not found.' }

$artifactRoot = Join-Path $PSScriptRoot 'artifacts'
New-Item -ItemType Directory -Force -Path $artifactRoot | Out-Null

& $msbuild "$PSScriptRoot\src\AtmosphereAutopilotNav.csproj" /t:Rebuild /v:minimal /nologo /p:UseSharedCompilation=false "/p:OutputPath=$artifactRoot\" /p:BaseIntermediateOutputPath=obj\CleanPhase\
if ($LASTEXITCODE -ne 0) { throw 'Navigation plugin build failed.' }

& $msbuild "$PSScriptRoot\tests\NavigationCoreTests.csproj" /t:Rebuild /v:minimal /nologo /p:UseSharedCompilation=false "/p:OutputPath=$artifactRoot\tests\" /p:BaseIntermediateOutputPath=obj\CleanPhase\
if ($LASTEXITCODE -ne 0) { throw 'Navigation test build failed.' }

& "$artifactRoot\tests\NavigationCoreTests.exe"
if ($LASTEXITCODE -ne 0) { throw 'Navigation tests failed.' }

$forbidden = @('KramaxAABridge', 'PilotAssistant', 'Pilot Assistant', 'using KramaxAutoPilot',
    '<Reference Include="KramaxAutoPilot')
$sourceFiles = Get-ChildItem -LiteralPath "$PSScriptRoot\src" -Recurse -File -Include *.cs,*.csproj
foreach ($term in $forbidden) {
    $hits = $sourceFiles | Select-String -SimpleMatch $term
    if ($hits) { throw "Forbidden dependency or source reference '$term' found in clean release source." }
}

$assembly = [Reflection.Assembly]::LoadFrom("$artifactRoot\AtmosphereAutopilotNav.dll")
$references = $assembly.GetReferencedAssemblies().Name
if ($references -contains 'KramaxAutoPilot' -or $references -contains 'KramaxAABridge') {
    throw 'Built DLL contains a forbidden prototype dependency.'
}
if ($references -notcontains 'AtmosphereAutopilot') { throw 'Built DLL is missing its expected AA dependency.' }

Write-Host "PASS: clean dependency boundary; references: $($references -join ', ')"
Write-Host "Built $artifactRoot\AtmosphereAutopilotNav.dll"
