# Limites fisicos y de hardware de las articulaciones
LIMITS = {
    'q1': (-90.0 , 90.0 ),   # Brazo 1 (grados)
    'q2': (-106.0, 106.0),   # Brazo 2 (grados)
    'q3': ( 0.0  , 4.4  ),   # Cremallera (cm, desplazamiento lineal)
    'q4': (-180.0, 180.0),   # Rotacion del efector (grados)
}

# Constantes geométricas y parámetros DH (en cm)
l1 = 9.97    # Longitud efectiva del primer eslabón  
l2 = 7.16    # Longitud efectiva del segundo eslabón
l3 = 14.46   # Longitud efectiva del tercer eslabón
l4 = 8.22    # Longitud del efector
a = 11.55    # Distancia vertical entre S0 y S1
b = 14.3     # Distancia vertical entre S1 y S2
c = 1.96     # Distancia horizontal entre S1 y S2 (direccion y0)
d = 1.55     # Distancia horizontal entre S2 y S3 (direccion y0)
e = 1.47     # Distancia horizontal entre S2 y S3 (direccion x0)
