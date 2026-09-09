param(
    [string]$Compiler = 'C:/Users/Lenovo/Downloads/Compressed/x86_64-8.1.0-release-posix-seh-rt_v6-rev0/mingw64/bin/gcc.exe'
)
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    # Refuse to test stale copies of main.c callbacks.
    $source = [IO.File]::ReadAllText((Join-Path (Get-Location) 'Core/Src/main.c'))
    $callbacks = ($source -split '/\* USER CODE BEGIN 4 \*/')[1] -split '/\* USER CODE END 4 \*/'
    $expected = $callbacks[0].Replace("`r", '').Trim()
    $tested = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'icm_callbacks_under_test.c')).Replace("`r", '')
    if (!$tested.Contains($expected)) { throw 'main.c callbacks changed: refresh icm_callbacks_under_test.c first' }
    & $Compiler -std=c11 -Wall -Wextra -Werror `
        -Iverification/icm_stubs -IUser/Device -IUser/Task -IUser/Control -IUser/Estimate `
        verification/test_icm42688.c verification/icm_callbacks_under_test.c `
        User/Device/icm42688.c User/Device/OLED.c User/Device/OLED_Data.c `
        User/Task/display.c User/Task/baremetal.c User/Task/remote_task.c `
        User/Estimate/attitude.c User/Control/flight_control.c User/Control/PID.c User/Control/mixer.c `
        -lm -o verification/test_icm42688.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host build failed' }
    & ./verification/test_icm42688.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
} finally {
    Pop-Location
}
