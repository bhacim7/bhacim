import logging
import os
import datetime
from colorama import Fore, Style, init

# Colorama başlat (Windows uyumluluğu için)
init(autoreset=True)


class RoboLogger:
    def __init__(self):
        # Log klasörü oluştur
        if not os.path.exists('logs'):
            os.makedirs('logs')

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"logs/mission_{timestamp}.log"

        # Logger ayarları
        self.logger = logging.getLogger("RoboBoat")
        self.logger.setLevel(logging.DEBUG)

        # Dosya Handler
        file_handler = logging.FileHandler(filename)
        file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        self.logger.addHandler(file_handler)

        # Konsol Handler (Sadece INFO ve üstü)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(logging.Formatter('%(message)s'))
        self.logger.addHandler(console_handler)

    def info(self, msg, color=Fore.WHITE):
        self.logger.info(color + str(msg) + Style.RESET_ALL)

    def warn(self, msg):
        self.logger.warning(Fore.YELLOW + "[UYARI] " + str(msg) + Style.RESET_ALL)

    def error(self, msg):
        self.logger.error(Fore.RED + "[HATA] " + str(msg) + Style.RESET_ALL)

    def debug(self, msg):
        self.logger.debug(str(msg))


# Global logger instance
log = RoboLogger()