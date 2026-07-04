# RA6M5 Robot Serial Console

Minimal Windows upper-computer for the RA6M5 robot arm.

The EXE starts a native PySide6 desktop GUI.

Run from source:

```powershell
python "Robot_FreeRTOS - Arm\UpperComputer\ra6m5_upper_console.py"
```

Self-test:

```powershell
python "Robot_FreeRTOS - Arm\UpperComputer\ra6m5_upper_console.py" --gui-self-test
```

Build EXE after installing PyInstaller:

```powershell
python -m PyInstaller --noconfirm --clean --onefile --windowed --name RA6M5_Robot_Console "Robot_FreeRTOS - Arm\UpperComputer\ra6m5_upper_console.py"
```
