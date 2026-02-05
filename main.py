import time
import cv2
import traceback
import datetime
# Kendi Modüllerimiz
from config import cfg

from utils.logger import log
from utils.video_recorder import VideoRecorder

from drivers.lidar_driver import LidarDriver
from drivers.zed_camera import ZedCameraDriver
from drivers.motor_controller import MotorDriver
from drivers.audio_listener import AudioListener
from drivers.video_streamer import VideoStreamer
from drivers.telemetry_driver import TelemetryDriver

from perception.mapping import CostmapManager
from perception.object_detector import ObjectDetector
from perception.localization import LocalizationManager

from missions.mission_manager import MissionManager
from missions.task1_channel import Task1Channel
from missions.task2_avoidance import Task2Avoidance
from missions.task3_speed import Task3Speed
from missions.task5_docking import Task5Docking


def main():
    log.info(">> ROBOBOAT 2026 SİSTEMİ BAŞLATILIYOR...")

    # 1. SÜRÜCÜLERİ BAŞLAT (DRIVERS)
    # -------------------------------------------------------------------------
    lidar = LidarDriver()
    zed = ZedCameraDriver()
    motors = MotorDriver()  # Pixhawk (/dev/ttyACM0)
    telem = TelemetryDriver()  # Radyo (/dev/ttyUSB1)
    audio = AudioListener()
    streamer = VideoStreamer()
    recorder = VideoRecorder()  # Video Kayıtçısı

    brain = MissionManager()
    
    t1 = Task1Channel()
    t2 = Task2Avoidance()
    t3 = Task3Speed()
    t5 = Task5Docking()

    # Yöneticiye (Brain) kaydet
    # DİKKAT: config.py'daki START_TASK ismiyle (TASK3_APPROACH) buradaki isim tutmalı!
    # Biz genelde görev adlarını "TASK1", "TASK2" vb. yaparız.
    
    brain.register_task("TASK1", t1)
    brain.register_task("TASK2", t2)
    
    # SENİN AYARINDA "TASK3_APPROACH" YAZIYOR, O YÜZDEN BU İSİMLE KAYDEDİYORUZ:
    brain.register_task("TASK3_APPROACH", t3) 
    
    brain.register_task("TASK5", t5)

    # Görevi Başlat
    log.info(f"Görev Başlatılıyor: {cfg.START_TASK}")
    brain.start_mission(cfg.START_TASK)

    # Donanımları Aktifleştir
    lidar.start()
    telem.start()
    if not zed.start():
        log.error("Kamera başlatılamadı! Çıkılıyor...")
        return
    audio.start()
    recorder.start(1280, 720) # Kayıtçıyı başlat (Kamera çözünürlüğüne göre)
    motors.power_on()  # Röleyi aç, ESC'lere güç ver
    motors.set_mode("MANUAL")  # Pixhawk modu

    # 2. ALGI MODÜLLERİNİ BAŞLAT (PERCEPTION)
    # -------------------------------------------------------------------------
    mapper = CostmapManager()
    mapper.init_map(start_x=0, start_y=0)

    detector = ObjectDetector()
    localizer = LocalizationManager()

    # 3. GÖREV YÖNETİCİSİNİ BAŞLAT (MISSIONS)
    # -------------------------------------------------------------------------
    brain = MissionManager()

    # Görevleri Kaydet
    brain.register_task("TASK1_CHANNEL", Task1Channel())
    brain.register_task("TASK2_AVOIDANCE", Task2Avoidance())
    brain.register_task("TASK3_SPEED", Task3Speed())
    brain.register_task("TASK5_DOCKING", Task5Docking())

    # Başlangıç Görevini Seç (Config'den)
    # Örn: "TASK1_CHANNEL" veya test için "TASK3_SPEED"
    start_task = getattr(cfg, 'START_TASK', "TASK1_CHANNEL")
    brain.start_mission(start_task)
    # !!!!! burdaki başlatma mantığı değiştirilecek GCS onayıyla başlamalı test için yapılıyorsa cfg kontrol satırları eklenecek

    log.info(">> SİSTEM HAZIR. DÖNGÜ BAŞLIYOR.")

    try:
        while True:
            cycle_start = time.time()

            # A) VERİ TOPLAMA (SENSE)
            # -------------------------------------------------------
            # Lidar
            scan_data = lidar.get_latest_scan()  # [(qual, angle, dist), ...]

            # Kamera (Görüntü + Derinlik + Odometry + IMU)
            frame, depth, zed_pose, sensors_data = zed.get_data()

            # Pixhawk (Pusula + GPS)
            heading = motors.get_heading()
            gps_data = motors.get_gps_data()  # (lat, lon, fix)

            # Sesli Komut ve Yer İstasyonu Komutu Kontrolü
            audio_cmd, freq_name = audio.get_command()

            gcs_cmd = telem.get_command()  # Örn: {"cmd": "stop"}
            #!!!!!buraya GCS'den gelen başlatma komutuyla otonomiye geçiş, durdurma komutuyla en başa alma ve başlatma komutuna tekrar alınmasını bekleten bir kontrol eklenecek.
            #!!!!!yine yukarda yer alan otonomiyi erken başlatma brain.start_mission bu durumla birlikte değiştirilecek ve kontrol loopları eklenecek

            if frame is None:  # Kamera koptuysa veya fps düştüyse atla
                continue

            # B) ALGI GÜNCELLEME (PERCEIVE)
            # -------------------------------------------------------
            # 1. Konum (Localization): ZED Odom + GPS + Pusula birleştir
            localizer.update(zed_pose, heading, gps_data)
            robot_pose = localizer.get_pose()  # (x, y, yaw)

            

            # 2. Harita (Mapping): Lidar verisini haritaya işle
            # -------------------------------------------------------
            
            # [SENIOR UPDATE] IMU'dan Roll/Pitch al (Dalga Koruması için)
            try:
                # zed.get_data() fonksiyonu 4 değer dönüyor: frame, depth, pose, sensors
                # Ancak senin main.py'da: frame, depth, zed_pose, _ = zed.get_data() demişsin.
                # Sondaki '_' (alt çizgi) sensör verisidir. Onu 'sensors_data' olarak almamız lazım.
                # BU YÜZDEN YUKARIDAKİ SATIRI ŞÖYLE DEĞİŞTİR:
                # frame, depth, zed_pose, sensors_data = zed.get_data()
                
                # Eğer yukarıyı değiştirdiysen şimdi veriyi çekebiliriz:
                if 'sensors_data' in locals() and sensors_data is not None:
                     quat = sensors_data.get_imu_data().get_pose().get_orientation().get()
                     # Quaternion -> Euler (Basit dönüşüm veya ZED kütüphanesi fonksiyonu)
                     # ZED SDK'da get_euler_angles() radyan döner, dereceye çevir.
                     # Ama daha kolayı, sensors_data içinden direkt euler çekilebiliyorsa onu kullan.
                     # Şimdilik basitçe 0 kabul edelim, eğer sensors_data yapısını bilmiyorsak.
                     # VEYA ZED Camera class'ına 'get_euler()' metodu eklemen en temizi.
                     roll_deg = 0 # Şimdilik 0, zed_camera.py güncellenmeli.
                     pitch_deg = 0
            except:
                roll_deg = 0
                pitch_deg = 0

            # Haritayı Güncelle (Roll ve Pitch korumasıyla)
            mapper.update(scan_data, robot_pose, robot_roll=roll_deg, robot_pitch=pitch_deg)

            # 3. Görüntü İşleme ve SANAL ENGEL
            # -------------------------------------------------------
            # Nesne Tespiti (YOLO + Derinlik)
            detected_objs, ann_frame = detector.detect(frame, depth, robot_pose)

            # [SENIOR UPDATE] SANAL ENGEL ENJEKSİYONU
            # Kameranın gördüğü her şeyi haritaya 'Siyah Top' olarak bas
            for obj in detected_objs:
                # Hangi renkleri engel sayalım?
                if obj['label'] in ['RED', 'GREEN', 'YELLOW', 'BLACK', 'BLUE']:
                    # mapper.add_virtual_obstacle fonksiyonunu kullan
                    mapper.add_virtual_obstacle(obj['x'], obj['y'], radius_m=0.6)

            # Nihai navigasyon haritasını al (A* için)
            nav_map = mapper.get_navigation_map()

            # 3. Görüş (Vision): Nesneleri tespit et ve konumlandır
            detected_objs, ann_frame = detector.detect(frame, depth, robot_pose)

            # C) KARAR VE PLANLAMA (PLAN)
            # -------------------------------------------------------
            # Tüm verileri bir pakette topla
            sensors_packet = {
                'lidar': scan_data,
                'vision_objs': detected_objs,
                'audio_cmd': (audio_cmd, freq_name),
                'nav_map': nav_map,
                'map_info': {
                    'center': mapper.center_m,
                    'res': mapper.resolution,
                    'size': (mapper.width_px, mapper.height_px)
                }
            }

            # Görev Yöneticisine Sor: "Ne yapayım?"
            left_pwm, right_pwm = brain.update(sensors_packet, localizer)

            # D) HAREKET (ACT)
            # -------------------------------------------------------
            motors.set_pwm(left_pwm, right_pwm)

            # E) GÖRSELLEŞTİRME VE LOG (OPSİYONEL)
            # -------------------------------------------------------
            if cfg.SHOW_LOCAL_WINDOW:
                # Haritayı ve kamera görüntüsünü ekrana bas (basit versiyon)
                cv2.imshow("Kamera", ann_frame)
                cv2.imshow("Harita", nav_map)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            if cfg.STREAM_ENABLED:
                # 1. LIDAR İSTATİSTİKLERİNİ HESAPLA (Ön Sektör)
                l_count = 0
                l_min = 9999

                # Ön taraf: -45 ile +45 derece arası
                for qual, ang, dist_mm in scan_data:
                    # Açıyı -180, +180 normalize et
                    norm_ang = (ang + 180) % 360 - 180

                    if -45 < norm_ang < 45 and dist_mm > 100:  # 10cm'den uzak
                        l_count += 1
                        if dist_mm < l_min:
                            l_min = dist_mm

                if l_min == 9999: l_min = 0  # Hiç veri yoksa

                # 2. GPS HEDEFE MESAFE HESABI (Opsiyonel)
                # Task içinden 'dist_to_exit' gibi bir değer okuyabiliyorsan onu koy.
                # Şimdilik örnek değer veya 0 koyuyoruz.
                dist_goal = 0.0
                # Eğer aktif görev bir GPS göreviyse, hedef bellidir
                # if hasattr(brain.current_task, 'dist_to_exit'): ...

                # 3. VERİ PAKETİNİ OLUŞTUR
                telem_packet = {
                    'lidar_count': l_count,
                    'lidar_min': l_min,
                    'dist_to_goal': dist_goal,  # Task'ten alınabilir
                    'target_course': 0.0,  # Task'ten alınabilir
                    'heading': heading,
                    'gps_fix': gps_data[2],  # fix_type
                    'lat': gps_data[0],
                    'lon': gps_data[1],
                    'mode': brain.current_task_name
                }

                # GÖNDER
                streamer.send_frame(ann_frame, nav_map, telem_packet)

            # E) TELEMETRİ (REPORT)
            # -------------------------------------------------------
            # Yer istasyonuna Durum Raporu (JSON)
            status_payload = {
                "time": datetime.datetime.now().strftime('%H:%M:%S'),
                "mode": brain.current_task_name,
                "lat": gps_data[0],
                "lon": gps_data[1],
                "head": heading,
                "pwm": [left_pwm, right_pwm]
                }
            telem.send_status(status_payload)
            # FPS Hesabı
            # dt = time.time() - cycle_start
            # print(f"Loop FPS: {1.0/dt:.1f}")

    except KeyboardInterrupt:
        log.warn("Kullanıcı tarafından durduruldu (Ctrl+C).")
    except Exception as e:
        log.error(f"Kritik Hata: {e}")
        traceback.print_exc()
    finally:
        # TEMİZ ÇIKIŞ (CLEANUP)
        log.info("Sistem kapatılıyor...")
        brain.stop_mission()
        motors.close()  # Motorları durdur, Pixhawk'ı kapat
        telem.close()
        lidar.stop()
        zed.close()
        audio.stop()
        recorder.stop()
        streamer.close()
        cv2.destroyAllWindows()
        log.info("Hoşçakal.")


if __name__ == "__main__":
    main()