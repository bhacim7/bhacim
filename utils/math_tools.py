import math


def haversine(lat1, lon1, lat2, lon2):
    """
    İki GPS koordinatı arasındaki mesafeyi (metre) hesaplar.
    """
    if None in (lat1, lon1, lat2, lon2):
        return 0.0

    R = 6371000.0  # Dünya yarıçapı (metre)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * \
        math.sin(delta_lambda / 2) ** 2

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    İki nokta arasındaki pusula açısını (0-360 derece) hesaplar.
    """
    if None in (lat1, lon1, lat2, lon2):
        return 0.0

    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - \
        math.sin(lat1) * math.cos(lat2) * math.cos(dLon)

    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360


def signed_angle_difference(target, current):
    """
    İki açı arasındaki en kısa dönüş farkını (-180, +180) verir.
    Pozitif: Saat yönü (Sağa dön), Negatif: Saat tersi (Sola dön)
    """
    diff = (target - current + 180) % 360 - 180
    return diff


def nfloat(x):
    """Güvenli float dönüşümü (NaN/Inf kontrolü)."""
    try:
        if x is None: return None
        val = float(x)
        return val if math.isfinite(val) else None
    except:
        return None


def nint(x):
    """Güvenli int dönüşümü."""
    try:
        if x is None: return None
        val = float(x)
        if not math.isfinite(val): return None
        return int(round(val))
    except:
        return None