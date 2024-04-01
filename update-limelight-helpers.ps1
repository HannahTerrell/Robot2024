$sourceFile = "https://raw.githubusercontent.com/LimelightVision/limelightlib-wpijava/main/LimelightHelpers.java"
$destFile = "src/main/java/frc/robot/LimelightHelpers.java"


Write-Host "Downloading $sourceFile..."
Invoke-WebRequest $sourceFile -OutFile $destFile | out-null
Write-Host "Written to $destFile"
