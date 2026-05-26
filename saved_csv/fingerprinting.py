import re
import subprocess
import time


def frequency_to_channel(freq_mhz):
    if freq_mhz is None:
        return None
    if 2412 <= freq_mhz <= 2472:
        return int(round((freq_mhz - 2407) / 5))
    if freq_mhz == 2484:
        return 14
    if 5000 <= freq_mhz <= 5900:
        return int(round((freq_mhz - 5000) / 5))
    return None


def band_from_frequency(freq_mhz):
    if freq_mhz is None:
        return "unknown"
    return "5GHz" if freq_mhz >= 5000 else "2.4GHz"


class WiFiScanner:
    def __init__(self, interface="wlan1"):
        # RSSI scan 전용 외장 동글 인터페이스. 핫스팟/SSH용 wlan0는 넣지 않는다.
        self.interface = interface

    def scan_networks(self):
        try:
            result = subprocess.run(
                ["iwlist", self.interface, "scan"],
                capture_output=True,
                text=True,
                timeout=6,
            )
        except Exception as exc:
            print(f"Wi-Fi scan failed on {self.interface}: {exc}", flush=True)
            return []

        if result.returncode != 0:
            print(
                f"Wi-Fi scan failed on {self.interface}: {result.stderr.strip()}",
                flush=True,
            )
            return []

        aps = self._parse_scan_result(result.stdout + result.stderr)
        print(f"Wi-Fi scan result on {self.interface}: {len(aps)} APs", flush=True)
        return aps

    def _parse_scan_result(self, raw_data):
        aps = []
        cells = raw_data.split("Cell ")
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

        for cell in cells[1:]:
            bssid_match = re.search(r"Address:\s*([0-9A-Fa-f:]{17})", cell)
            ssid_match = re.search(r'ESSID:"([^"]*)"', cell)
            signal_match = re.search(r"Signal level=(-?\d+(?:\.\d+)?)\s*dBm", cell)
            freq_match = re.search(r"Frequency:([0-9.]+)\s*GHz", cell)
            channel_match = re.search(r"\(Channel\s+(\d+)\)", cell)

            if not signal_match:
                continue

            frequency = None
            if freq_match:
                frequency = int(round(float(freq_match.group(1)) * 1000.0))
            channel = int(channel_match.group(1)) if channel_match else frequency_to_channel(frequency)
            bssid = bssid_match.group(1).upper() if bssid_match else ""

            aps.append(
                {
                    "bssid": bssid,
                    "mac": bssid,
                    "ssid": ssid_match.group(1) if ssid_match else "",
                    "rssi": float(signal_match.group(1)),
                    "frequency": frequency,
                    "frequency_mhz": frequency,
                    "channel": channel,
                    "band": band_from_frequency(frequency),
                    "interface": self.interface,
                    "timestamp": timestamp,
                    "source": "iwlist",
                }
            )

        return sorted(aps, key=lambda ap: ap["rssi"], reverse=True)

    def capture_fingerprint(self, current_x, current_y):
        return {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "x": current_x,
            "y": current_y,
            "aps": self.scan_networks(),
        }


if __name__ == "__main__":
    scanner = WiFiScanner("wlan1")
    print(scanner.capture_fingerprint(None, None))
