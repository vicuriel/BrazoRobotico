# Limites fisicos y de hardware de las articulaciones
# q1, q2, q4 [°], q3 [cm]

LIMITS = {
    'q1': (-90.0 , 90.0 ),   # Brazo 1 (grados)
    'q2': (-106.0, 106.0),   # Brazo 2 (grados)
    'q3': ( 0.0  , 4.4  ),   # Cremallera (cm, desplazamiento lineal)
    'q4': (-180.0, 180.0),   # Rotacion del efector (grados)
}