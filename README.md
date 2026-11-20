# Seguridad para bodegas

Sistema de seguridad para una bodega con **Arduino UNO R4 WiFi**: sensor PIR, sensor ultrasónico, display de 7 segmentos con 74HC595, buzzer, motor con puente H L298N, reloj RTC DS3231 y Bluetooth HC-05.

**Simulador:** https://edwincedexab4356-boop.github.io/seguridad-bodega/

## Cómo funciona
1. **Desarmado** (display `–`): no vigila. La puerta se abre solo con la clave.
2. **Salida** (9 → 0): después de `ON` hay 9 segundos para salir.
3. **Armado** (`A`): vigila el PIR y el ultrasónico.
4. **Cuenta** (9 → 0): alguien fue detectado; hay que poner la clave.
5. **Alarma** (`E`): sirena 60 s y el motor cierra la puerta.

La clave (`CODIGO:1234`) desarma el sistema y abre la puerta. 3 claves malas = alarma.

## Archivos
- `index.html`: simulador (Arduino virtual o conexión al Arduino real por USB / Bluetooth).
- `seguridad_bodega/seguridad_bodega.ino`: programa del Arduino.
- `conexiones.html` e `imagenes/`: cómo conectar cada pieza.
- `resumen.txt`: conexiones, estados y órdenes en texto.
