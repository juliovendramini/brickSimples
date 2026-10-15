"""
Testa RFID.cpp no PC contra um MFRC522 e cartões simulados (sim.cpp).
Uso: python rodar.py     (precisa de g++ no PATH)
"""
import os, shutil, subprocess, sys, tempfile
AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, "..", ".."))
with tempfile.TemporaryDirectory() as t:
    for f in ("sim.cpp", "Arduino.h", "SoftWire.h", "AsyncDelay.h"):
        shutil.copy(os.path.join(AQUI, f), t)
    for f in ("RFID.h", "RFID.cpp", "portas.h"):
        shutil.copy(os.path.join(RAIZ, f), t)
    exe = os.path.join(t, "teste" + (".exe" if os.name == "nt" else ""))
    subprocess.check_call(["g++", "-std=gnu++11", "-I", t, "-o", exe,
                           os.path.join(t, "sim.cpp"), os.path.join(t, "RFID.cpp")])
    sys.exit(subprocess.call([exe]))
