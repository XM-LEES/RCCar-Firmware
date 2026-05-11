param(
    [string]$Port = "COM4",
    [int]$Baud = 115200,
    [double[]]$Targets = @(0.5, 1.0, 2.0),
    [int]$DurationMs = 2500,
    [int]$SettleMs = 800,
    [int]$GapMs = 1500,
    [int]$SendPeriodMs = 50,
    [string[]]$CandidateNames = @(),
    [string]$OutDir = ""
)

$ErrorActionPreference = "Stop"
$Culture = [System.Globalization.CultureInfo]::InvariantCulture
[System.Threading.Thread]::CurrentThread.CurrentCulture = $Culture
[System.Threading.Thread]::CurrentThread.CurrentUICulture = $Culture

$FrameLen = 11
$TelemetryLen = 24
$Head = [byte]0x7B
$Tail = [byte]0x7D
$CmdMotion = [byte]0
$CmdPidParam = [byte]5

$ParamEnable = [byte]1
$ParamKpX100 = [byte]2
$ParamKiX100 = [byte]3
$ParamTrimUs = [byte]4
$ParamResetI = [byte]5

$PidMagic = [byte[]](0x50, 0x49, 0x44, 0x31)

$CandidateSets = @(
    @{ Name = "openloop"; Enable = 0; Kp = 10.0; Ki = 3.0; Trim = 12 },
    @{ Name = "p8_i0_t12"; Enable = 1; Kp = 8.0; Ki = 0.0; Trim = 12 },
    @{ Name = "p12_i0_t12"; Enable = 1; Kp = 12.0; Ki = 0.0; Trim = 12 },
    @{ Name = "p14_i0_t12"; Enable = 1; Kp = 14.0; Ki = 0.0; Trim = 12 },
    @{ Name = "p16_i0_t12"; Enable = 1; Kp = 16.0; Ki = 0.0; Trim = 12 },
    @{ Name = "p18_i0_t12"; Enable = 1; Kp = 18.0; Ki = 0.0; Trim = 12 },
    @{ Name = "p20_i0_t10"; Enable = 1; Kp = 20.0; Ki = 0.0; Trim = 10 },
    @{ Name = "p20_i0_t12"; Enable = 1; Kp = 20.0; Ki = 0.0; Trim = 12 },
    @{ Name = "p20_i0_t14"; Enable = 1; Kp = 20.0; Ki = 0.0; Trim = 14 },
    @{ Name = "p20_i0_t16"; Enable = 1; Kp = 20.0; Ki = 0.0; Trim = 16 },
    @{ Name = "p22_i0_t12"; Enable = 1; Kp = 22.0; Ki = 0.0; Trim = 12 },
    @{ Name = "p24_i0_t12"; Enable = 1; Kp = 24.0; Ki = 0.0; Trim = 12 },
    @{ Name = "p16_i1_t16"; Enable = 1; Kp = 16.0; Ki = 1.0; Trim = 16 },
    @{ Name = "p20_i1_t20"; Enable = 1; Kp = 20.0; Ki = 1.0; Trim = 20 },
    @{ Name = "p24_i2_t24"; Enable = 1; Kp = 24.0; Ki = 2.0; Trim = 24 }
)

if ($CandidateNames.Count -gt 0) {
    $wanted = @{}
    foreach ($name in $CandidateNames) {
        $wanted[$name] = $true
    }
    $CandidateSets = @($CandidateSets | Where-Object { $wanted.ContainsKey([string]$_["Name"]) })
    if ($CandidateSets.Count -eq 0) {
        throw "No candidate sets matched: $($CandidateNames -join ', ')"
    }
}

function Get-Bcc([byte[]]$Bytes, [int]$Length) {
    [int]$bcc = 0
    for ($i = 0; $i -lt $Length; $i++) {
        $bcc = $bcc -bxor [int]$Bytes[$i]
    }
    return [byte]($bcc -band 0xFF)
}

function Set-Int16BE([byte[]]$Bytes, [int]$Offset, [int]$Value) {
    if ($Value -lt -32768 -or $Value -gt 32767) {
        throw "int16 value out of range: $Value"
    }
    [int]$u = $Value
    if ($u -lt 0) {
        $u += 65536
    }
    $Bytes[$Offset] = [byte](($u -shr 8) -band 0xFF)
    $Bytes[$Offset + 1] = [byte]($u -band 0xFF)
}

function Get-Int16BE([byte[]]$Bytes, [int]$Offset) {
    [int]$u = ([int]$Bytes[$Offset] -shl 8) -bor [int]$Bytes[$Offset + 1]
    if ($u -ge 32768) {
        $u -= 65536
    }
    return $u
}

function New-MotionFrame([double]$VxMps, [double]$VyMps, [double]$VzRadS) {
    [byte[]]$frame = New-Object byte[] $FrameLen
    $frame[0] = $Head
    $frame[1] = $CmdMotion
    $frame[2] = 0
    Set-Int16BE $frame 3 ([int][Math]::Round($VxMps * 1000.0))
    Set-Int16BE $frame 5 ([int][Math]::Round($VyMps * 1000.0))
    Set-Int16BE $frame 7 ([int][Math]::Round($VzRadS * 1000.0))
    $frame[9] = Get-Bcc $frame 9
    $frame[10] = $Tail
    return $frame
}

function New-PidParamFrame([byte]$ParamId, [int]$Value) {
    [byte[]]$frame = New-Object byte[] $FrameLen
    $frame[0] = $Head
    $frame[1] = $CmdPidParam
    $frame[2] = $ParamId
    Set-Int16BE $frame 3 $Value
    $frame[5] = $PidMagic[0]
    $frame[6] = $PidMagic[1]
    $frame[7] = $PidMagic[2]
    $frame[8] = $PidMagic[3]
    $frame[9] = Get-Bcc $frame 9
    $frame[10] = $Tail
    return $frame
}

function Write-SerialFrame($Serial, [byte[]]$Frame) {
    $Serial.Write($Frame, 0, $Frame.Length)
}

function Send-PidParam($Serial, [byte]$ParamId, [int]$Value) {
    $frame = New-PidParamFrame $ParamId $Value
    for ($i = 0; $i -lt 2; $i++) {
        Write-SerialFrame $Serial $frame
        Start-Sleep -Milliseconds 25
    }
}

function Send-PidSet($Serial, [hashtable]$Set) {
    Send-PidParam $Serial $ParamResetI 0
    Send-PidParam $Serial $ParamKpX100 ([int][Math]::Round([double]$Set["Kp"] * 100.0))
    Send-PidParam $Serial $ParamKiX100 ([int][Math]::Round([double]$Set["Ki"] * 100.0))
    Send-PidParam $Serial $ParamTrimUs ([int]$Set["Trim"])
    Send-PidParam $Serial $ParamEnable ([int]$Set["Enable"])
    Send-PidParam $Serial $ParamResetI 0
}

function Convert-FrameHex([byte[]]$Frame) {
    return (($Frame | ForEach-Object { $_.ToString("X2") }) -join " ")
}

function Parse-TelemetryFrame([byte[]]$Frame, [int64]$NowMs) {
    [double]$vx = (Get-Int16BE $Frame 2) / 1000.0
    [double]$vy = (Get-Int16BE $Frame 4) / 1000.0
    [double]$vz = (Get-Int16BE $Frame 6) / 1000.0
    [double]$bat = (Get-Int16BE $Frame 20) / 1000.0
    return [pscustomobject]@{
        TimeMs = $NowMs
        Vx = $vx
        Vy = $vy
        Vz = $vz
        Battery = $bat
        Hex = Convert-FrameHex $Frame
    }
}

function Read-AvailableTelemetry($Serial, [System.Collections.Generic.List[byte]]$Rx, [byte[]]$ReadBuffer, [System.Diagnostics.Stopwatch]$Clock) {
    while ($Serial.BytesToRead -gt 0) {
        $wanted = [Math]::Min($Serial.BytesToRead, $ReadBuffer.Length)
        $read = $Serial.Read($ReadBuffer, 0, $wanted)
        for ($i = 0; $i -lt $read; $i++) {
            [void]$Rx.Add($ReadBuffer[$i])
        }
    }

    $samples = New-Object System.Collections.Generic.List[object]
    while ($Rx.Count -ge $TelemetryLen) {
        $headIndex = -1
        for ($i = 0; $i -lt $Rx.Count; $i++) {
            if ($Rx[$i] -eq $Head) {
                $headIndex = $i
                break
            }
        }
        if ($headIndex -lt 0) {
            $Rx.Clear()
            break
        }
        if ($headIndex -gt 0) {
            $Rx.RemoveRange(0, $headIndex)
        }
        if ($Rx.Count -lt $TelemetryLen) {
            break
        }

        [byte[]]$frame = New-Object byte[] $TelemetryLen
        $Rx.CopyTo(0, $frame, 0, $TelemetryLen)
        if ($frame[$TelemetryLen - 1] -ne $Tail -or $frame[$TelemetryLen - 2] -ne (Get-Bcc $frame 22)) {
            $Rx.RemoveAt(0)
            continue
        }

        [void]$samples.Add((Parse-TelemetryFrame $frame $Clock.ElapsedMilliseconds))
        $Rx.RemoveRange(0, $TelemetryLen)
    }
    return $samples
}

function Write-RawSamples($Writer, [string]$SetName, [double]$Target, [double]$Kp, [double]$Ki, [int]$Trim, [int]$Enable, $Samples) {
    foreach ($sample in $Samples) {
        $line = [string]::Format(
            $Culture,
            "{0},{1},{2:F3},{3:F1},{4:F1},{5},{6},{7:F3},{8:F3},{9:F3},{10:F3},{11}",
            [object[]]@(
                $sample.TimeMs,
                $SetName,
                $Target,
                $Kp,
                $Ki,
                $Trim,
                $Enable,
                $sample.Vx,
                $sample.Vy,
                $sample.Vz,
                $sample.Battery,
                $sample.Hex
            ))
        $Writer.WriteLine($line)
    }
}

function Send-ZeroFor($Serial, [System.Collections.Generic.List[byte]]$Rx, [byte[]]$ReadBuffer, [System.Diagnostics.Stopwatch]$Clock, [int]$Duration) {
    $frame = New-MotionFrame 0.0 0.0 0.0
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    [int64]$nextSend = 0
    while ($sw.ElapsedMilliseconds -lt $Duration) {
        if ($sw.ElapsedMilliseconds -ge $nextSend) {
            Write-SerialFrame $Serial $frame
            $nextSend += $SendPeriodMs
        }
        [void](Read-AvailableTelemetry $Serial $Rx $ReadBuffer $Clock)
        Start-Sleep -Milliseconds 5
    }
}

function Run-MotionTrial($Serial, [System.Collections.Generic.List[byte]]$Rx, [byte[]]$ReadBuffer, [System.Diagnostics.Stopwatch]$Clock, [double]$Target) {
    $frame = New-MotionFrame $Target 0.0 0.0
    $trialClock = [System.Diagnostics.Stopwatch]::StartNew()
    $samples = New-Object System.Collections.Generic.List[object]
    [int64]$nextSend = 0

    while ($trialClock.ElapsedMilliseconds -lt $DurationMs) {
        if ($trialClock.ElapsedMilliseconds -ge $nextSend) {
            Write-SerialFrame $Serial $frame
            $nextSend += $SendPeriodMs
        }

        $newSamples = Read-AvailableTelemetry $Serial $Rx $ReadBuffer $Clock
        foreach ($sample in $newSamples) {
            $sample | Add-Member -NotePropertyName TrialMs -NotePropertyValue $trialClock.ElapsedMilliseconds -Force
            [void]$samples.Add($sample)
        }
        Start-Sleep -Milliseconds 5
    }

    return $samples
}

function Measure-Trial([string]$SetName, [hashtable]$Set, [double]$Target, $Samples) {
    $eligible = @($Samples | Where-Object { $_.TrialMs -ge $SettleMs })
    if ($eligible.Count -eq 0) {
        return [pscustomobject]@{
            SetName = $SetName
            Target = $Target
            Kp = [double]$Set["Kp"]
            Ki = [double]$Set["Ki"]
            Trim = [int]$Set["Trim"]
            Enable = [int]$Set["Enable"]
            Samples = 0
            MeanVx = 0.0
            Mae = 99.0
            Std = 0.0
            Overshoot = 99.0
            Score = 199.0
        }
    }

    [double]$sum = 0.0
    [double]$absErr = 0.0
    [double]$maxVx = -999.0
    [double]$minVx = 999.0
    foreach ($sample in $eligible) {
        $sum += $sample.Vx
        $absErr += [Math]::Abs($Target - $sample.Vx)
        if ($sample.Vx -gt $maxVx) { $maxVx = $sample.Vx }
        if ($sample.Vx -lt $minVx) { $minVx = $sample.Vx }
    }
    [double]$mean = $sum / $eligible.Count
    [double]$mae = $absErr / $eligible.Count

    [double]$var = 0.0
    foreach ($sample in $eligible) {
        $var += ($sample.Vx - $mean) * ($sample.Vx - $mean)
    }
    [double]$std = [Math]::Sqrt($var / $eligible.Count)

    [double]$overshoot = 0.0
    if ($Target -ge 0.0) {
        $overshoot = [Math]::Max(0.0, $maxVx - $Target)
    } else {
        $overshoot = [Math]::Max(0.0, $Target - $minVx)
    }
    [double]$score = $mae + 0.40 * $overshoot + 0.05 * $std

    return [pscustomobject]@{
        SetName = $SetName
        Target = $Target
        Kp = [double]$Set["Kp"]
        Ki = [double]$Set["Ki"]
        Trim = [int]$Set["Trim"]
        Enable = [int]$Set["Enable"]
        Samples = $eligible.Count
        MeanVx = $mean
        Mae = $mae
        Std = $std
        Overshoot = $overshoot
        Score = $score
    }
}

if ([string]::IsNullOrWhiteSpace($OutDir)) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $OutDir = Join-Path (Join-Path (Get-Location) "logs") "pid_tune_$stamp"
}
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$rawPath = Join-Path $OutDir "raw_telemetry.csv"
$summaryPath = Join-Path $OutDir "summary.csv"
$choicePath = Join-Path $OutDir "recommended.txt"

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 20
$serial.WriteTimeout = 200
$serial.DtrEnable = $false
$serial.RtsEnable = $false

$rx = [System.Collections.Generic.List[byte]]::new()
[byte[]]$readBuffer = New-Object byte[] 4096
$clock = [System.Diagnostics.Stopwatch]::StartNew()
$allSummaries = New-Object System.Collections.Generic.List[object]
$rawWriter = [System.IO.StreamWriter]::new($rawPath, $false, [System.Text.Encoding]::ASCII)

try {
    $rawWriter.WriteLine("time_ms,set,target_mps,kp,ki,trim_us,enable,vx_mps,vy_mps,vz_rad_s,battery_v,frame_hex")
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    foreach ($set in $CandidateSets) {
        $name = [string]$set["Name"]
        Write-Host ([string]::Format($Culture, "Testing {0}: enable={1} kp={2:F1} ki={3:F1} trim={4}", $name, $set["Enable"], $set["Kp"], $set["Ki"], $set["Trim"]))
        Send-PidSet $serial $set
        Send-ZeroFor $serial $rx $readBuffer $clock $GapMs

        foreach ($target in $Targets) {
            Write-Host ([string]::Format($Culture, "  target {0:F3} m/s", $target))
            $samples = Run-MotionTrial $serial $rx $readBuffer $clock $target
            Write-RawSamples $rawWriter $name $target ([double]$set["Kp"]) ([double]$set["Ki"]) ([int]$set["Trim"]) ([int]$set["Enable"]) $samples
            $summary = Measure-Trial $name $set $target $samples
            [void]$allSummaries.Add($summary)
            Write-Host ([string]::Format($Culture, "    mean={0:F3} mae={1:F3} std={2:F3} score={3:F3} samples={4}", $summary.MeanVx, $summary.Mae, $summary.Std, $summary.Score, $summary.Samples))
            Send-ZeroFor $serial $rx $readBuffer $clock $GapMs
        }
    }

    Send-PidParam $serial $ParamResetI 0
    Send-ZeroFor $serial $rx $readBuffer $clock 1200
} finally {
    try {
        if ($serial.IsOpen) {
            $zero = New-MotionFrame 0.0 0.0 0.0
            for ($i = 0; $i -lt 6; $i++) {
                Write-SerialFrame $serial $zero
                Start-Sleep -Milliseconds 30
            }
            $serial.Close()
        }
    } catch {
    }
    $rawWriter.Close()
}

$allSummaries | Export-Csv -Path $summaryPath -NoTypeInformation -Encoding ASCII

$setScores = $allSummaries |
    Group-Object SetName |
    ForEach-Object {
        $rows = $_.Group
        [pscustomobject]@{
            SetName = $_.Name
            MeanScore = ($rows | Measure-Object -Property Score -Average).Average
            MaxMae = ($rows | Measure-Object -Property Mae -Maximum).Maximum
            MinSamples = ($rows | Measure-Object -Property Samples -Minimum).Minimum
        }
    } |
    Sort-Object MeanScore

$best = $setScores | Select-Object -First 1
$bestSet = $CandidateSets | Where-Object { $_["Name"] -eq $best.SetName } | Select-Object -First 1

$recommend = @(
    "recommended_set=$($best.SetName)",
    ([string]::Format($Culture, "enable={0}", $bestSet["Enable"])),
    ([string]::Format($Culture, "kp={0:F1}", [double]$bestSet["Kp"])),
    ([string]::Format($Culture, "ki={0:F1}", [double]$bestSet["Ki"])),
    ([string]::Format($Culture, "trim_us={0}", [int]$bestSet["Trim"])),
    ([string]::Format($Culture, "mean_score={0:F4}", [double]$best.MeanScore)),
    "raw=$rawPath",
    "summary=$summaryPath"
)
$recommend | Set-Content -Path $choicePath -Encoding ASCII

try {
    $applySerial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $applySerial.ReadTimeout = 20
    $applySerial.WriteTimeout = 200
    $applySerial.DtrEnable = $false
    $applySerial.RtsEnable = $false
    $applySerial.Open()
    Send-PidSet $applySerial $bestSet
    $zero = New-MotionFrame 0.0 0.0 0.0
    for ($i = 0; $i -lt 6; $i++) {
        Write-SerialFrame $applySerial $zero
        Start-Sleep -Milliseconds 30
    }
    $applySerial.Close()
    Write-Host "Applied recommended set to board RAM."
} catch {
    Write-Warning "Could not apply recommended set after tuning: $_"
}

Write-Host ""
Write-Host "Recommended:"
$recommend | ForEach-Object { Write-Host "  $_" }
