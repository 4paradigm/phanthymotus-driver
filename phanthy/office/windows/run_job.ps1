# Called by the robot over SSH: hand one job to the desktop-session worker and
# print its result. ASCII only: Windows PowerShell 5.1 reads a BOM-less script
# in the ANSI code page.
param([Parameter(Mandatory = $true)][string]$Job, [int]$TimeoutSec = 50)
$ErrorActionPreference = 'Stop'
$task = 'Visitor Badge Worker'
$jobs = Join-Path $PSScriptRoot 'data\jobs'
New-Item -ItemType Directory -Force -Path $jobs | Out-Null
$id = [guid]::NewGuid().ToString('N')
$request = Join-Path $jobs "$id.request.json"
$result = Join-Path $jobs "$id.result.json"
[IO.File]::WriteAllBytes("$request.tmp", [Convert]::FromBase64String($Job))
Move-Item -LiteralPath "$request.tmp" -Destination $request -Force
Start-ScheduledTask -TaskName $task
$deadline = (Get-Date).AddSeconds($TimeoutSec)
while (!(Test-Path -LiteralPath $result)) {
    if ((Get-Date) -gt $deadline) {
        # Withdraw the job if the worker has not taken it, so it cannot run
        # (and print) after the robot has already reported a timeout.
        Remove-Item -LiteralPath $request -Force -ErrorAction SilentlyContinue
        throw "VISITOR_TIMEOUT task=$((Get-ScheduledTask -TaskName $task).State)"
    }
    # The worker exits after a few idle seconds; a job landing as it leaves
    # needs the task started again.
    if ((Get-ScheduledTask -TaskName $task).State -notin @('Running', 'Queued')) {
        Start-ScheduledTask -TaskName $task
    }
    Start-Sleep -Milliseconds 500
}
$bytes = [IO.File]::ReadAllBytes($result)
Remove-Item -LiteralPath $result -Force
'VISITOR_RESULT:' + [Convert]::ToBase64String($bytes)
