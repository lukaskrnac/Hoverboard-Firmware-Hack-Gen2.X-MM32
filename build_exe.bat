@echo off
rem Builds MM32Flasher.exe. Run it in the repo root (where the tools\ folder is).
rem Needs Python 3.10+ for the build only; the resulting exe does not need Python.
setlocal
cd /d "%~dp0"
if not exist tools\mm32flash.py (
  echo Run this file in the repo root, where tools\mm32flash.py is.
  goto :end
)
if not exist tools\mm32flash_gui.py (
  echo tools\mm32flash_gui.py is missing.
  goto :end
)
python -m venv .build-venv || goto :end
call .build-venv\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install pyocd libusb-package pyinstaller || goto :end
pyinstaller --onefile --noconsole --name MM32Flasher --paths tools ^
  --collect-all pyocd --collect-all cmsis_pack_manager --collect-all libusb_package ^
  --copy-metadata pyocd tools\mm32flash_gui.py || goto :end
echo.
echo Done: dist\MM32Flasher.exe
:end
pause
