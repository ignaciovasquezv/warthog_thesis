from math import radians, sin, cos, sqrt, atan2

def haversine_distance(lat1, lon1, lat2, lon2):
    # Radio de la Tierra en kilómetros
    R = 6371.0
    
    # Convertir grados a radianes
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    lat2 = radians(lat2)
    lon2 = radians(lon2)
    
    # Diferencias de latitud y longitud
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    # Fórmula de Haversine
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    
    # Calcular la distancia en metros
    distance = R * c * 1000  # Convertir kilómetros a metros
    return distance

# Coordenadas de los puntos
lat1 = -34.39607250349149
lon1 = -70.80957378173314
lat2 = -34.39607262744198
lon2 = -70.80957378059475

# Calcular la distancia
distance = haversine_distance(lat1, lon1, lat2, lon2)
print(f"La distancia entre los puntos es: {distance:.2f} metros")
