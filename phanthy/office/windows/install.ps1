# Install or upgrade the front-desk side. The robot does this by itself the first
# time a job finds the front desk missing its version: it copies this directory
# to C:\ProgramData\VisitorBadge\upload (with the pinned dependencies in deps\)
# and runs this script over SSH as an administrator. Nothing is downloaded here.
# ASCII only: Windows PowerShell 5.1 reads a BOM-less script in the ANSI code page.
param([Parameter(Mandatory = $true)][string]$Version)
$ErrorActionPreference = 'Stop'
$root = 'C:\ProgramData\VisitorBadge'
$upload = Join-Path $root 'upload'
$deps = Join-Path $upload 'deps'
$app = Join-Path $root 'app'

$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
if (!(New-Object Security.Principal.WindowsPrincipal($identity)).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'VISITOR_NOT_ADMIN'
}

foreach ($line in Get-Content -LiteralPath (Join-Path $upload 'deps.txt')) {
    if (!$line.Trim() -or $line.StartsWith('#')) { continue }
    $name, $sha = ($line -split '\s+')[0, 1]
    if ((Get-FileHash -LiteralPath (Join-Path $deps $name) -Algorithm SHA256).Hash -ne $sha) {
        throw "VISITOR_BAD_DEPENDENCY $name"
    }
}

# Any Python 3.12 will do: it only creates the venv the worker runs from. One
# already registered for this account has to be used rather than installed
# over - the installer then just reports success and puts nothing in TargetDir
# (seen on a front desk where somebody had installed 3.12.10 by hand).
function Find-Python {
    $candidates = @(Join-Path $root 'Python312\python.exe')
    foreach ($hive in 'HKCU', 'HKLM') {
        $key = "${hive}:\Software\Python\PythonCore\3.12\InstallPath"
        $dir = (Get-ItemProperty -LiteralPath $key -ErrorAction SilentlyContinue).'(default)'
        if ($dir) { $candidates += Join-Path $dir 'python.exe' }
    }
    $candidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
$python = Find-Python
if (!$python) {
    $installer = Join-Path $deps 'python-3.12.10-amd64.exe'
    $signature = Get-AuthenticodeSignature -LiteralPath $installer
    if ($signature.Status -ne 'Valid' -or $signature.SignerCertificate.Subject -notmatch 'Python Software Foundation') {
        throw 'VISITOR_BAD_PYTHON_SIGNATURE'
    }
    $process = Start-Process -FilePath $installer -Wait -PassThru -WindowStyle Hidden -ArgumentList @(
        '/quiet', 'InstallAllUsers=0', "TargetDir=`"$(Join-Path $root 'Python312')`"", 'PrependPath=0',
        'Include_launcher=0', 'Include_test=0', 'Include_symbols=0', 'Include_debug=0')
    if ($process.ExitCode -notin @(0, 3010)) { throw "VISITOR_PYTHON_INSTALL_FAILED $($process.ExitCode)" }
    $python = Find-Python
    if (!$python) { throw 'VISITOR_PYTHON_NOT_FOUND' }
}
$venv = Join-Path $root 'venv'
if (!(Test-Path -LiteralPath (Join-Path $venv 'Scripts\python.exe'))) {
    & $python -m venv $venv
    if ($LASTEXITCODE) { throw 'VISITOR_VENV_FAILED' }
}
& (Join-Path $venv 'Scripts\python.exe') -m pip install --no-index --find-links $deps `
    --disable-pip-version-check --quiet 'playwright==1.51.0'
if ($LASTEXITCODE) { throw 'VISITOR_PIP_FAILED' }

New-Item -ItemType Directory -Force -Path (Join-Path $app 'data\jobs') | Out-Null
foreach ($name in @('worker.py', 'visitor_site.py', 'run_job.ps1')) {
    Copy-Item -LiteralPath (Join-Path $upload $name) -Destination $app -Force
}

# Kiosk printing sends the job to whatever print preview would preselect, and
# Edge preselects the printer this profile last printed to, not the Windows
# default. Without this the card's printer setting (applied by switching the
# Windows default, visitor_site.use_printer) is ignored after the first print:
# every badge kept going to the label printer it had first printed on.
$policy = 'HKLM:\SOFTWARE\Policies\Microsoft\Edge'
New-Item -Path $policy -Force | Out-Null
New-ItemProperty -Path $policy -Name 'PrintPreviewUseSystemDefaultPrinter' -PropertyType DWord -Value 1 -Force | Out-Null

# Edge must run on the user's desktop, which an SSH session does not have: the
# worker runs as an interactive scheduled task of this same account. pythonw,
# so it does not flash a console over the front-desk screen.
$action = New-ScheduledTaskAction -Execute (Join-Path $venv 'Scripts\pythonw.exe') `
    -Argument "`"$(Join-Path $app 'worker.py')`"" -WorkingDirectory $app
$principal = New-ScheduledTaskPrincipal -UserId $identity.Name -LogonType Interactive -RunLevel Highest
$settings = New-ScheduledTaskSettingsSet -MultipleInstances IgnoreNew -StartWhenAvailable `
    -ExecutionTimeLimit (New-TimeSpan -Minutes 20) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
Register-ScheduledTask -TaskName 'Visitor Badge Worker' -Action $action -Principal $principal `
    -Settings $settings -Force | Out-Null

Remove-Item -LiteralPath $upload -Recurse -Force
# Written last: the robot reinstalls whenever this does not match its own version.
Set-Content -LiteralPath (Join-Path $app 'version.txt') -Value $Version -Encoding ascii
'VISITOR_INSTALL_OK'
