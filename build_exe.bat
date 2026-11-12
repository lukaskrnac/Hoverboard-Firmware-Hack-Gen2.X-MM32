@echo off
rem Zostavi MM32Flasher.exe. Spusti v koreni repozitara (tam kde je priecinok tools\).
rem Treba nainstalovany Python 3.10+ - len na zostavenie, vysledne exe uz Python nepotrebuje.
setlocal
cd /d "%~dp0"
if not exist tools\mm32flash.py (
  echo Spusti tento subor v koreni repozitara, kde je tools\mm32flash.py
  goto :end
)
if not exist tools\mm32flash_gui.py (
  echo Chyba tools\mm32flash_gui.py - skopiruj ho do tools\
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
echo Hotovo: dist\MM32Flasher.exe
:end
pause
