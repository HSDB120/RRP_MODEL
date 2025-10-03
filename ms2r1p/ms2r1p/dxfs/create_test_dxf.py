#!/usr/bin/env python3
import ezdxf

# Crear un nuevo documento DXF
doc = ezdxf.new('R2010')
msp = doc.modelspace()

# OFFSET para mover la trayectoria al workspace alcanzable
# El robot puede alcanzar de 0.4m a 2.8m desde el centro
offset_x = 150  # 1.5m después de escalar
offset_y = 0

# Dibujar un cuadrado exterior (80x80 mm) - más pequeño para estar seguro
square_size = 80
msp.add_lwpolyline([
    (offset_x + 0, offset_y + 0),
    (offset_x + square_size, offset_y + 0),
    (offset_x + square_size, offset_y + square_size),
    (offset_x + 0, offset_y + square_size),
    (offset_x + 0, offset_y + 0)
])

# Dibujar un círculo interno (radio 15 mm)
msp.add_circle((offset_x + 25, offset_y + 25), radius=15)

# Dibujar un rectángulo interno (15x20 mm)
msp.add_lwpolyline([
    (offset_x + 50, offset_y + 50),
    (offset_x + 65, offset_y + 50),
    (offset_x + 65, offset_y + 70),
    (offset_x + 50, offset_y + 70),
    (offset_x + 50, offset_y + 50)
])

# Guardar el archivo
doc.saveas('/home/hsdb/ros2_ws_2502/src/ms2r1p/ms2r1p/dxfs/traj_shape_amg.dxf')
print("Archivo DXF con offset creado exitosamente!")
print(f"Trayectoria centrada en X={offset_x/100:.2f}m, Y={offset_y/100:.2f}m")
print(f"Rango X: {offset_x/100:.2f}m a {(offset_x+square_size)/100:.2f}m")
print(f"Rango Y: {offset_y/100:.2f}m a {(offset_y+square_size)/100:.2f}m")