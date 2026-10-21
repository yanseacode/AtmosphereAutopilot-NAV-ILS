$ErrorActionPreference = 'Stop'

& "$PSScriptRoot\build.ps1"
if ($LASTEXITCODE -ne 0) { throw 'Build failed; package was not created.' }

$artifactRoot = Join-Path $PSScriptRoot 'artifacts'
$stage = Join-Path $artifactRoot 'package'
$resolvedProject = [IO.Path]::GetFullPath($PSScriptRoot).TrimEnd([IO.Path]::DirectorySeparatorChar) + [IO.Path]::DirectorySeparatorChar
$resolvedStage = [IO.Path]::GetFullPath($stage)
if (-not $resolvedStage.StartsWith($resolvedProject, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Package staging path escaped the project directory.'
}
if (Test-Path -LiteralPath $stage) { Remove-Item -LiteralPath $stage -Recurse -Force }

$modRoot = Join-Path $stage 'GameData\AtmosphereAutopilotNav'
$pluginRoot = Join-Path $modRoot 'Plugins'
New-Item -ItemType Directory -Force -Path $pluginRoot | Out-Null
Copy-Item -LiteralPath "$artifactRoot\AtmosphereAutopilotNav.dll" -Destination $pluginRoot
Copy-Item -LiteralPath "$PSScriptRoot\GameData\AtmosphereAutopilotNav\AtmosphereAutopilotNav.version" -Destination $modRoot
Copy-Item -LiteralPath "$PSScriptRoot\GameData\AtmosphereAutopilotNav\FlightPlans" -Destination $modRoot -Recurse
Copy-Item -LiteralPath "$PSScriptRoot\LICENSE" -Destination $modRoot
Copy-Item -LiteralPath "$PSScriptRoot\LICENSE-NOTICE.md" -Destination $modRoot
Copy-Item -LiteralPath "$PSScriptRoot\README.md" -Destination $modRoot
Copy-Item -LiteralPath "$PSScriptRoot\TESTING.md" -Destination $modRoot
Copy-Item -LiteralPath "$PSScriptRoot\PROVENANCE.md" -Destination $modRoot

$zip = Join-Path $artifactRoot 'AtmosphereAutopilot-NAV-ILS-0.6.5-dev.zip'
if (Test-Path -LiteralPath $zip) { Remove-Item -LiteralPath $zip -Force }
Compress-Archive -LiteralPath (Join-Path $stage 'GameData') -DestinationPath $zip -CompressionLevel Optimal
Write-Host "Created $zip"
