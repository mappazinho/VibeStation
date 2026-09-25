param(
    [Parameter(Mandatory = $true)]
    [string]$BiosPath,

    [string]$BuildDir = "build-ps2",
    [int]$Runs = 3,
    [UInt64]$Budget = 400000000,
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $BiosPath)) {
    throw "BIOS not found: $BiosPath"
}
if ($Runs -lt 1) {
    throw "Runs must be at least 1."
}

$trace = Join-Path $BuildDir "Release\vibestation_ps2_bios_trace.exe"
if (-not $SkipBuild) {
    cmake --build $BuildDir --config Release --target vibestation_ps2_bios_trace
    if ($LASTEXITCODE -ne 0) {
        throw "Release trace build failed."
    }
}
if (-not (Test-Path -LiteralPath $trace)) {
    $trace = Join-Path $BuildDir "vibestation_ps2_bios_trace"
}
if (-not (Test-Path -LiteralPath $trace)) {
    throw "Trace executable not found under $BuildDir."
}

function Parse-Metric {
    param([string]$Text, [string]$Name)
    $match = [regex]::Match(
        $Text,
        "(?m)(?:^|\s)" + [regex]::Escape($Name) + "=([^\s]+)")
    if (-not $match.Success) { return $null }
    return $match.Groups[1].Value
}

function Metric-OrZero {
    param([string]$Text, [string]$Name)
    $value = Parse-Metric $Text $Name
    if ($null -eq $value -or $value -eq "") {
        return "0"
    }
    return $value
}

function Run-Backend {
    param([string]$Name, [string[]]$ExtraArgs)

    $rows = @()
    for ($i = 1; $i -le $Runs; ++$i) {
        Write-Host "[$Name] run $i/$Runs"
        $arguments = @(
            $BiosPath,
            [string]$Budget,
            "--profile",
            "--gs-thread"
        ) + $ExtraArgs

        $output = & $trace @arguments 2>&1 | Out-String
        if ($LASTEXITCODE -ne 0) {
            Write-Host $output
            throw "$Name trace failed with exit code $LASTEXITCODE."
        }

        $row = [pscustomobject]@{
            Backend = $Name
            Run = $i
            FieldRate = [double](Parse-Metric $output "PROFILE_FIELD_RATE")
            RunMs = [double](Parse-Metric $output "PROFILE_RUN_MS")
            DisplayHash = Parse-Metric $output "DISPLAY_HASH"
            RasterPixels = [UInt64](Parse-Metric $output "GS_RASTER_PIXELS")
            DynarecInstructions = [UInt64](Metric-OrZero $output "EE_DYNAREC_INSTRUCTIONS")
            DynarecBlocks = [UInt64](Metric-OrZero $output "EE_DYNAREC_BLOCKS_EXECUTED")
            LinkHits = [UInt64](Metric-OrZero $output "EE_DYNAREC_LINK_HITS")
            LinkMisses = [UInt64](Metric-OrZero $output "EE_DYNAREC_LINK_MISSES")
            GuardExits = [UInt64](Metric-OrZero $output "EE_DYNAREC_GUARD_EXITS")
            Cop0Exits = [UInt64](Metric-OrZero $output "EE_DYNAREC_COP0_WRITE_EXITS")
        }
        $rows += $row
        $row | Format-List
    }
    return ,$rows
}

function Median {
    param([double[]]$Values)
    $sorted = @($Values | Sort-Object)
    $count = $sorted.Count
    if (($count % 2) -eq 1) {
        return $sorted[[int]($count / 2)]
    }
    return ($sorted[$count / 2 - 1] + $sorted[$count / 2]) / 2.0
}

$interpreter = Run-Backend "cached-interpreter" @()
$dynarec = Run-Backend "second-gen-x64-dynarec" @("--ee-dynarec")

$referenceHash = $interpreter[0].DisplayHash
$referencePixels = $interpreter[0].RasterPixels
foreach ($row in $interpreter + $dynarec) {
    if ($row.DisplayHash -ne $referenceHash) {
        throw "Display hash mismatch: $($row.Backend) run $($row.Run) produced $($row.DisplayHash), expected $referenceHash."
    }
    if ($row.RasterPixels -ne $referencePixels) {
        throw "Raster pixel mismatch: $($row.Backend) run $($row.Run) produced $($row.RasterPixels), expected $referencePixels."
    }
}

$summary = @(
    [pscustomobject]@{
        Backend = "cached-interpreter"
        MedianFieldRate = Median @($interpreter.FieldRate)
        MedianRunMs = Median @($interpreter.RunMs)
        DisplayHash = $referenceHash
        RasterPixels = $referencePixels
    },
    [pscustomobject]@{
        Backend = "second-gen-x64-dynarec"
        MedianFieldRate = Median @($dynarec.FieldRate)
        MedianRunMs = Median @($dynarec.RunMs)
        DisplayHash = $referenceHash
        RasterPixels = $referencePixels
    }
)

Write-Host ""
Write-Host "400M EE backend comparison"
$summary | Format-Table -AutoSize

$dynMedian = Median @($dynarec.RunMs)
$intMedian = Median @($interpreter.RunMs)
$delta = (($intMedian - $dynMedian) / $intMedian) * 100.0
Write-Host ("Dynarec wall-time delta vs interpreter: {0:N2}%" -f $delta)
Write-Host ("Reference output: DISPLAY_HASH={0} GS_RASTER_PIXELS={1}" -f $referenceHash, $referencePixels)
