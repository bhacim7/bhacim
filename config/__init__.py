# Dosya Yolu: config/__init__.py

"""
RoboBoat 2026 Konfigürasyon Paketi

Bu paket, robotun donanım ayarlarını (settings) ve
görev koordinatlarını (waypoints) merkezi bir noktadan yönetir.
"""

# settings.py dosyasından ana konfigürasyon nesnesini (cfg) dışarı açıyoruz.
# Böylece projenin herhangi bir yerinden 'from config import cfg' diyerek erişebileceğiz.
from .settings import cfg

# waypoints.py dosyasından koordinat nesnesini (wp) dışarı açıyoruz.
# Erişim: 'from config import wp'
from .waypoints import wp

"""
Bu dosya, Python'un config klasörünü bir paket olarak tanımasını sağlar.

Diğer dosyalar (örneğin main.py),
ayarları çağırmak istediğinde uzun uzun dosya yolu yazmak yerine sadece from config import cfg diyebilecekler.
"""