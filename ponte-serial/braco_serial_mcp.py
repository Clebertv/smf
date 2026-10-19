"""
Ponte serial do braco Lab-Volt 5250 para o app do Claude (servidor MCP local).

A Bancada Virtual 5250, aberta no app do Claude para desktop, usa este programa
para conversar com o Arduino pela porta COM. Ele faz o mesmo papel do PuTTY:
envia as teclas e devolve o texto que o Arduino escreve.

Instalacao (uma vez, no PC da bancada):
    pip install "mcp>=1.12,<2" pyserial
Depois registre este arquivo no app do Claude (veja LEIA-ME.txt).

Seguranca:
  - So uma janela pode usar a porta COM por vez: feche o PuTTY antes de conectar.
  - Nenhum comando fica esperando: ESPACO chega ao Arduino na hora.
  - "segurar" repete uma tecla de movimento por no maximo 2 segundos a cada pedido.
    Se a pagina travar ou fechar, a repeticao para sozinha e o Arduino para o eixo.
  - Ao desconectar (ou se este programa fechar) a repeticao para na hora.
"""

import asyncio
import threading
import time

import serial
import serial.tools.list_ports
from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations

mcp = FastMCP("braco_serial")

BUFFER_MAX = 200_000          # caracteres guardados do que o Arduino escreveu
INTERVALO_REPETICAO = 0.03    # 30 ms entre repeticoes, como o teclado do Windows
DURACAO_MAXIMA = 2.0          # cada "segurar" vale no maximo 2 s
VERSAO = 3

_lock = threading.RLock()
_porta = None                 # objeto serial aberto
_nome_porta = ""
_texto = ""                   # texto recebido (janela final do buffer)
_base = 0                     # posicao absoluta do primeiro caractere de _texto
_erro = ""
_hold_tecla = None
_hold_id = 0
_hold_ate = 0.0
_solto_ate_id = 0


def _fim() -> int:
    return _base + len(_texto)


def _guardar(s: str) -> None:
    global _texto, _base
    _texto += s
    if len(_texto) > BUFFER_MAX:
        corte = len(_texto) - BUFFER_MAX
        _texto = _texto[corte:]
        _base += corte


def _leitor(p) -> None:
    global _erro
    while True:
        with _lock:
            if _porta is not p:
                return
        try:
            dados = p.read(p.in_waiting or 1)
        except Exception as e:  # cabo desligado, porta sumiu
            with _lock:
                _erro = f"Leitura falhou: {e}"
                _fechar()
            return
        if dados:
            with _lock:
                _guardar(dados.decode("latin-1"))


def _repetidor() -> None:
    global _hold_tecla
    while True:
        time.sleep(INTERVALO_REPETICAO)
        with _lock:
            if _hold_tecla is None or _porta is None:
                continue
            if time.monotonic() > _hold_ate:
                _hold_tecla = None
                continue
            try:
                _porta.write(_hold_tecla.encode("latin-1"))
            except Exception:
                _hold_tecla = None


def _parar_repeticao() -> None:
    global _hold_tecla
    _hold_tecla = None


def _novo(desde) -> dict:
    """Texto recebido desde a posicao 'desde' (para devolver junto com cada comando)."""
    if desde is None:
        return {}
    inicio = max(desde, _base)
    return {"texto": _texto[inicio - _base:], "pos": _fim(), "conectado": _porta is not None}


def _fechar() -> None:
    global _porta, _nome_porta
    _parar_repeticao()
    p = _porta
    _porta = None
    _nome_porta = ""
    if p is not None:
        try:
            p.close()
        except Exception:
            pass


@mcp.tool(annotations=ToolAnnotations(readOnlyHint=True))
def listar_portas() -> dict:
    """Lista as portas seriais (COM) deste computador."""
    portas = [
        {"porta": p.device, "descricao": p.description or ""}
        for p in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    ]
    with _lock:
        return {"portas": portas, "conectado": _porta is not None, "porta_atual": _nome_porta}


@mcp.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=False))
def conectar(porta: str, baud: int = 9600) -> dict:
    """Abre a porta serial do Arduino. O Arduino reinicia e faz o zeramento."""
    global _porta, _nome_porta, _erro
    with _lock:
        _fechar()
        _erro = ""
        try:
            p = serial.serial_for_url(porta, baudrate=baud, timeout=0.1, write_timeout=1)
        except Exception as e:
            raise ValueError(
                f"Nao consegui abrir {porta}: {e}. Feche o PuTTY ou outro programa que use a porta."
            )
        _porta = p
        _nome_porta = porta
        threading.Thread(target=_leitor, args=(p,), daemon=True).start()
        return {"conectado": True, "porta": porta, "pos": _fim()}


@mcp.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=False))
def desconectar() -> dict:
    """Fecha a porta serial."""
    with _lock:
        _fechar()
        return {"conectado": False}


@mcp.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=False))
def enviar(texto: str, desde: int | None = None) -> dict:
    """Envia texto ao Arduino, como digitar no PuTTY. Para qualquer tecla segurada.
    Com 'desde', devolve tambem o texto novo que o Arduino escreveu."""
    with _lock:
        if _porta is None:
            raise ValueError("Porta nao conectada.")
        _parar_repeticao()
        _porta.write(texto.encode("latin-1", errors="replace"))
        return {"enviado": len(texto), **_novo(desde)}


@mcp.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=False))
def segurar(tecla: str, id: int, duracao_ms: int = 450, desde: int | None = None) -> dict:
    """Repete uma tecla de movimento como se estivesse segurada.
    Renove antes de duracao_ms terminar; para sozinha depois disso (maximo 2 s)."""
    global _hold_tecla, _hold_id, _hold_ate
    with _lock:
        if _porta is None:
            raise ValueError("Porta nao conectada.")
        if id <= _solto_ate_id:
            return {"ignorado": True, **_novo(desde)}
        novo = _hold_tecla != tecla or _hold_id != id
        _hold_tecla = tecla
        _hold_id = id
        _hold_ate = time.monotonic() + min(max(duracao_ms, 50), DURACAO_MAXIMA * 1000) / 1000
        if novo:
            _porta.write(tecla.encode("latin-1"))
        return {"segurando": tecla, **_novo(desde)}


@mcp.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=False))
def soltar(id: int, desde: int | None = None) -> dict:
    """Solta a tecla segurada (para a repeticao na hora)."""
    global _solto_ate_id
    with _lock:
        _solto_ate_id = max(_solto_ate_id, id)
        if _hold_id <= id:
            _parar_repeticao()
        return {"solto": True, **_novo(desde)}


@mcp.tool(annotations=ToolAnnotations(readOnlyHint=True))
async def ler(desde: int = 0, esperar_ms: int = 0) -> dict:
    """Devolve o texto que o Arduino escreveu a partir da posicao 'desde'.
    Com esperar_ms, aguarda ate esse tempo (maximo 1 s) por texto novo."""
    limite = time.monotonic() + min(max(esperar_ms, 0), 1000) / 1000
    while True:
        with _lock:
            tem = _fim() > desde or _porta is None
        if tem or time.monotonic() >= limite:
            break
        await asyncio.sleep(0.04)
    if tem and esperar_ms > 0:
        await asyncio.sleep(0.25)   # junta o resto da mensagem numa resposta so
    with _lock:
        inicio = max(desde, _base)
        return {
            "texto": _texto[inicio - _base:],
            "pos": _fim(),
            "perdido": desde < _base,
            "conectado": _porta is not None,
            "porta": _nome_porta,
            "erro": _erro,
            "versao": VERSAO,
        }


if __name__ == "__main__":
    threading.Thread(target=_repetidor, daemon=True).start()
    mcp.run()
