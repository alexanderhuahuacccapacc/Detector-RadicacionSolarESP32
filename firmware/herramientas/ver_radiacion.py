"""Muestra en el navegador las mediciones que llegan por el USB del ESP32."""

import argparse
import json
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import urlopen

import serial


PAGINA = """<!doctype html>
<html lang="es">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Radiación solar en vivo</title>
<style>
  * { box-sizing: border-box }
  body { margin: 0; padding: 8vh 1.25rem; background: #101817; color: #edf8f1;
    font: 1rem/1.5 system-ui, sans-serif }
  main { max-width: 38rem; margin: auto }
  h1 { font-size: clamp(1.6rem, 5vw, 2.5rem); margin: 0 0 1rem }
  section { padding: clamp(1.25rem, 5vw, 2.25rem); background: #1b2824;
    border: 1px solid #42554b; border-radius: 1rem }
  #valor { display: block; color: #ccf078; font-size: clamp(2.5rem, 10vw, 5rem);
    font-weight: 650; line-height: 1.15; font-variant-numeric: tabular-nums }
  .unidad { font-size: 1.2rem }
  #tension, #luxometro, #detalle, #estado { color: #bfcec6; overflow-wrap: anywhere }
</style>
<main>
  <h1>Radiación solar en vivo</h1>
  <section aria-label="Última lectura del ESP32">
    <output id="valor" aria-live="polite">—</output>
    <span class="unidad">W/m²</span>
    <p id="tension"></p>
    <p id="luxometro"></p>
    <p id="detalle"></p>
    <p id="estado" role="status">Esperando la primera lectura del ESP32…</p>
    <div id="registro"></div>
  </section>
</main>
<script>
const valor = document.getElementById('valor');
const tension = document.getElementById('tension');
const luxometro = document.getElementById('luxometro');
const detalle = document.getElementById('detalle');
const estado = document.getElementById('estado');
const registro = document.getElementById('registro');
async function actualizar() {
  try {
    const respuesta = await fetch('/lectura', {cache: 'no-store'});
    if (!respuesta.ok) throw new Error(respuesta.status);
    const dato = await respuesta.json();
    valor.textContent = dato.valor === null ? '—' :
      new Intl.NumberFormat('es-PE', {minimumFractionDigits: 1}).format(dato.valor);
    tension.textContent = dato.tension_adc ? 'Tensión en A0 · ' + dato.tension_adc : '';
    luxometro.textContent = dato.luxometro ? 'BH1750 · ' + dato.luxometro : '';
    detalle.textContent = dato.detalle;
    registro.textContent = dato.registro;
    estado.textContent = dato.estado || (dato.hora
      ? 'Última lectura recibida a las ' + new Date(dato.hora * 1000).toLocaleTimeString('es-PE')
      : 'Esperando la primera lectura del ESP32…');
  } catch {
    estado.textContent = 'Se perdió la comunicación con el servidor de la laptop.';
  } finally {
    setTimeout(actualizar, 1000);
  }
}
actualizar();
</script>
</html>"""


class Lecturas:
    def __init__(self):
        self.lock = threading.Lock()
        self.datos = dict(
            valor=None, tension_adc="", luxometro="", detalle="", registro="", hora=None, estado=""
        )

    def actualizar(self, **cambios):
        with self.lock:
            self.datos.update(cambios)

    def obtener(self):
        with self.lock:
            datos = self.datos.copy()
        if datos["hora"] and time.time() - datos["hora"] > 15:
            datos["estado"] = datos["estado"] or "No llegan lecturas nuevas del ESP32."
        return datos


def leer_esp32(puerto, lecturas):
    while True:
        try:
            with serial.Serial(puerto, 115200, timeout=1) as conexion:
                lecturas.actualizar(estado="")
                tension_adc = ""
                luxometro = ""
                while True:
                    texto = conexion.readline().decode("utf-8", errors="replace").strip()
                    if texto.startswith("Tension ADC:"):
                        tension_adc = texto.partition(":")[2].strip()
                    elif texto.startswith("Lux BH1750:"):
                        luxometro = texto.partition(":")[2].strip()
                    elif texto.startswith("Radiacion solar:"):
                        lectura = texto.partition(":")[2].strip()
                        valor, unidad, detalle = lectura.partition(" W/m2")
                        lecturas.actualizar(
                            valor=float(valor) if unidad else None,
                            tension_adc=tension_adc,
                            luxometro=luxometro,
                            detalle=detalle.strip() if unidad else lectura,
                            hora=time.time(),
                        )
                    elif texto.startswith("Estado:"):
                        lecturas.actualizar(registro=texto)
        except serial.SerialException:
            lecturas.actualizar(estado=f"Esperando {puerto}. Si el Monitor serie está abierto, ciérralo.")
        time.sleep(1)


class Vista(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            contenido = PAGINA.encode("utf-8")
            tipo = "text/html; charset=utf-8"
        elif self.path == "/lectura":
            contenido = json.dumps(self.server.lecturas.obtener()).encode("utf-8")
            tipo = "application/json"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", tipo)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(contenido)))
        self.end_headers()
        self.wfile.write(contenido)

    def log_message(self, *args):
        pass


def ver_terminal(puerto_web):
    url = f"http://127.0.0.1:{puerto_web}/lectura"
    print("Radiación solar en vivo · Ctrl+C para cerrar la lectura")
    print("I = V / R. Radiación = 1000 × (I − I₀) / Iref.")
    ultima_hora = None
    ultimo_aviso = None
    while True:
        try:
            with urlopen(url, timeout=3) as respuesta:
                lectura = json.load(respuesta)
        except OSError:
            lectura = {"estado": "Esperando el servidor de lecturas de la laptop."}
        aviso = lectura["estado"] or ("" if lectura["hora"] else "Esperando una lectura del ESP32.")
        if aviso and aviso != ultimo_aviso:
            print(aviso)
        ultimo_aviso = aviso
        if not aviso and lectura["hora"] != ultima_hora:
            ultima_hora = lectura["hora"]
            hora = datetime.fromtimestamp(ultima_hora).strftime("%H:%M:%S")
            valor = lectura["valor"]
            radiacion = f"{valor:.1f} W/m²" if valor is not None else "sin dato"
            tension = lectura.get("tension_adc") or "sin dato"
            lux = lectura.get("luxometro") or "sin dato"
            print(f"[{hora}]  Shunt {tension}  ·  Radiación {radiacion}  ·  BH1750 {lux}")
            print(f"            {lectura['detalle']} {lectura['registro']}".rstrip())
        time.sleep(1)


if __name__ == "__main__":
    argumentos = argparse.ArgumentParser(description=__doc__)
    argumentos.add_argument("--com", default="COM8")
    argumentos.add_argument("--puerto-web", type=int, default=8765)
    argumentos.add_argument("--terminal", action="store_true", help="ver las lecturas en esta terminal")
    opciones = argumentos.parse_args()
    if opciones.terminal:
        try:
            ver_terminal(opciones.puerto_web)
        except KeyboardInterrupt:
            pass
        raise SystemExit

    lecturas = Lecturas()
    with ThreadingHTTPServer(("127.0.0.1", opciones.puerto_web), Vista) as servidor:
        servidor.lecturas = lecturas
        threading.Thread(target=leer_esp32, args=(opciones.com, lecturas), daemon=True).start()
        servidor.serve_forever()
