"""
Instala a ponte serial do braco 5250 no app do Claude para desktop (Windows).

Uso: deixe este arquivo na mesma pasta que braco_serial_mcp.py e rode
    python instalar_ponte.py
Depois feche o app do Claude por completo (inclusive na bandeja do relogio) e abra de novo.
"""

import glob
import json
import os
import shutil
import subprocess
import sys
import time

PASTA = os.path.dirname(os.path.abspath(__file__))
SERVIDOR = os.path.join(PASTA, "braco_serial_mcp.py")


def pastas_config():
    candidatas = []
    appdata = os.environ.get("APPDATA")
    if appdata:
        candidatas.append(os.path.join(appdata, "Claude"))
    local = os.environ.get("LOCALAPPDATA")
    if local:  # versao instalada pela Microsoft Store
        candidatas += glob.glob(os.path.join(local, "Packages", "Claude_*", "LocalCache", "Roaming", "Claude"))
    existentes = [p for p in candidatas if os.path.isdir(p)]
    return existentes or candidatas[:1]


def main():
    if not os.path.isfile(SERVIDOR):
        sys.exit("Nao achei braco_serial_mcp.py nesta pasta.")

    print("1/2 Instalando bibliotecas (mcp e pyserial)...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", "mcp>=1.12,<2", "pyserial"])

    print("2/2 Registrando a ponte no app do Claude...")
    entrada = {"command": sys.executable, "args": [SERVIDOR]}
    alvos = pastas_config()
    if not alvos:
        sys.exit("Nao achei a pasta de configuracao do app do Claude. O app esta instalado neste PC?")
    for pasta in alvos:
        os.makedirs(pasta, exist_ok=True)
        arq = os.path.join(pasta, "claude_desktop_config.json")
        cfg = {}
        if os.path.isfile(arq):
            with open(arq, encoding="utf-8") as f:
                texto = f.read().strip()
            if texto:
                try:
                    cfg = json.loads(texto)
                except json.JSONDecodeError:
                    sys.exit(f"O arquivo {arq} esta com erro de formatacao. Corrija ou apague-o e rode de novo.")
            shutil.copy2(arq, arq + f".backup-{time.strftime('%Y%m%d-%H%M%S')}")
        cfg.setdefault("mcpServers", {})["braco_serial"] = entrada
        with open(arq, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
        print("   gravado em", arq)

    print("\nPronto. Feche o app do Claude por completo e abra de novo.")
    print("Depois abra a Bancada Virtual 5250 no app e clique em 'Conectar braco real'.")


if __name__ == "__main__":
    main()
