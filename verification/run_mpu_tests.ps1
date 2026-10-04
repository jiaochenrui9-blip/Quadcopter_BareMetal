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
    $tested = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'mpu_callbacks_under_test.c')).Replace("`r", '')
    if (!$tested.Contains($expected)) { throw 'main.c callbacks changed: refresh mpu_callbacks_under_test.c first' }

    # The stub main.h repeats the pin labels CubeMX generated from the .ioc.
    $mainH = [IO.File]::ReadAllText((Join-Path (Get-Location) 'Core/Inc/main.h')).Replace("`r", '')
    $stubH = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'mpu_stubs/main.h')).Replace("`r", '')
    foreach ($label in 'GYRO_INT_Pin', 'GYRO_INT_GPIO_Port', 'NRF_IRQ_Pin', 'NRF_IRQ_GPIO_Port') {
        $real = [regex]::Match($mainH, "#define $label (\S+)").Groups[1].Value
        $stub = [regex]::Match($stubH, "#define $label (\S+)").Groups[1].Value
        if (!$real -or ($real -ne $stub)) {
            throw "Core/Inc/main.h has $label = '$real' but mpu_stubs/main.h has '$stub': refresh the stub"
        }
    }

    & $Compiler -std=c11 -Wall -Wextra -Werror `
        -Iverification/mpu_stubs -IUser/Device -IUser/Task -IUser/Control -IUser/Estimate `
        verification/test_mpu6050.c verification/mpu_callbacks_under_test.c `
        User/Device/mpu6050.c User/Device/OLED.c User/Device/OLED_Data.c `
        User/Task/display.c User/Task/baremetal.c User/Task/remote_task.c `
        User/Estimate/attitude.c User/Control/flight_control.c User/Control/PID.c User/Control/mixer.c `
        -lm -o verification/test_mpu6050.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host build failed' }
    & ./verification/test_mpu6050.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }

    # Closed-loop sign/stability check against a rigid-body model.
    & $Compiler -std=c11 -Wall -Wextra -Werror `
        -Iverification/mpu_stubs -IUser/Device -IUser/Task -IUser/Control -IUser/Estimate `
        verification/test_control_sign.c User/Estimate/attitude.c `
        User/Control/flight_control.c User/Control/PID.c User/Control/mixer.c `
        -lm -o verification/test_control_sign.exe
    if ($LASTEXITCODE -ne 0) { throw 'Control sign test build failed' }
    & ./verification/test_control_sign.exe
    if ($LASTEXITCODE -ne 0) { throw 'Control sign test failed' }
} finally {
    Pop-Location
}
