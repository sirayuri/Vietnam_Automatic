@echo off
setlocal

set "MX_JAVA=C:\ST\STM32CubeIDE_1.18.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.jre.win64_3.4.0.202409160955\jre\bin\java.exe"
set "MX_JAR=C:\ST\STM32CubeIDE_1.18.0\STM32CubeIDE\plugins\com.st.stm32cube.common.mx_6.15.0.202507011659\STM32CubeMX.jar"

if not exist "%MX_JAVA%" (
  echo CubeIDE bundled Java was not found: %MX_JAVA%
  pause
  exit /b 1
)
if not exist "%MX_JAR%" (
  echo CubeMX 6.15 JAR was not found: %MX_JAR%
  pause
  exit /b 1
)

start "STM32CubeMX 6.15" "%MX_JAVA%" -jar "%MX_JAR%" %*
endlocal
