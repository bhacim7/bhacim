from dataclasses import dataclass


@dataclass
class Waypoints:
    """
    RoboBoat 2026 Görev Koordinatları
    Tüm GPS noktaları (Latitude/Longitude) burada ayrı ayrı tutulur.
    """

    # --- TASK 1: Evacuation Route (Kanal Geçişi) ---
    T1_GATE_ENTER_LAT: float = 40.8630501
    T1_GATE_ENTER_LON: float = 29.2599517

    T1_GATE_EXIT_LAT: float = 40.8629223
    T1_GATE_EXIT_LON: float = 29.2599123

    # Refactored Task 1 Waypoints
    TASK1_START_LAT: float = 40.8630501
    TASK1_START_LON: float = 29.2599517

    TASK1_MID_LAT: float = 40.8629862
    TASK1_MID_LON: float = 29.2599320

    TASK1_FINAL_LAT: float = 40.8629223
    TASK1_FINAL_LON: float = 29.2599123

    # --- TASK 2: Debris Clearance (Engel ve Işık Sahası) ---
    # T2 Başlangıcı (Referans)
    T2_ZONE_ENTRY_LAT: float = 40.80916
    T2_ZONE_ENTRY_LON: float = 29.261915

    # T2 Bitişi (Hedef)
    T2_ZONE_END_LAT: float = 40.8089552
    T2_ZONE_END_LON: float = 29.2619292

    # --- TASK 3: Speed Challenge (Sürat Kapısı) ---
    T3_GATE_SEARCH_LAT: float = 40.8089735
    T3_GATE_SEARCH_LON: float = 29.2620113

    T3_YELLOW_APPROACH_LAT: float = 40.8089423
    T3_YELLOW_APPROACH_LON: float = 29.2621796

    # --- TASK 5: Docking (Park Etme) ---
    T5_DOCK_APPROACH_LAT: float = 40.8632559
    T5_DOCK_APPROACH_LON: float = 29.2594437


# Tek bir instance oluşturup dışarı açıyoruz
wp = Waypoints()
