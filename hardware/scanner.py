import re
import subprocess
import time


class WiFiScanner:
    def __init__(
        self,
        interface="wlan1",
        interfaces=None,
        scan_interfaces=None,
        scan_all_interfaces=True,
        auto_discover_usb_dongle=True,
        exclude_connected_interfaces=True,
        interface_power_control_enable=False,
        esp32_wifi_source=None,
        use_esp32_wifi=True,
        use_local_wifi=True,
        target_ssids=None,
    ):
        self.interface = interface
        self.interfaces = interfaces or [interface]
        self.scan_interfaces = scan_interfaces or [interface]
        self.scan_all_interfaces = scan_all_interfaces
        self.auto_discover_usb_dongle = auto_discover_usb_dongle
        self.exclude_connected_interfaces = exclude_connected_interfaces
        self.interface_power_control_enable = bool(interface_power_control_enable)
        self.esp32_wifi_source = esp32_wifi_source
        self.use_esp32_wifi = use_esp32_wifi
        self.use_local_wifi = use_local_wifi
        self.target_ssids = {
            str(ssid).strip().casefold()
            for ssid in (target_ssids or [])
            if str(ssid).strip()
        }
        self.local_wifi_power_enabled = False
        self.local_wifi_power_interfaces = []
        self.last_power_change = None
        self.last_net_status = {}
        self._last_net_status_log_key = None

    def set_sources(self, use_esp32_wifi=None, use_local_wifi=None):
        if use_esp32_wifi is not None:
            self.use_esp32_wifi = bool(use_esp32_wifi)
        if use_local_wifi is not None:
            self.use_local_wifi = bool(use_local_wifi)

    def set_local_wifi_power(self, enabled):
        interfaces = self._resolve_scan_interfaces(log=True)
        touched = []
        if enabled and not interfaces:
            self.local_wifi_power_enabled = False
            self.local_wifi_power_interfaces = []
            self.last_power_change = time.strftime("%Y-%m-%d %H:%M:%S")
            print("[WiFi] RSSI dongle ON skipped: no safe scan interface", flush=True)
            return

        for interface in interfaces:
            if interface:
                if enabled or self.interface_power_control_enable:
                    self._set_interface_power(interface, enabled)
                touched.append(interface)
        self.local_wifi_power_enabled = bool(enabled and touched)
        self.local_wifi_power_interfaces = touched
        self.last_power_change = time.strftime("%Y-%m-%d %H:%M:%S")
        print(
            f"[WiFi] RSSI scan interface power {'ON' if enabled else 'OFF'}: {touched}",
            flush=True,
        )

    def get_power_status(self):
        return {
            "local_wifi_power_enabled": self.local_wifi_power_enabled,
            "local_wifi_power_interfaces": list(self.local_wifi_power_interfaces),
            "last_power_change": self.last_power_change,
            "use_esp32_wifi": self.use_esp32_wifi,
            "use_local_wifi": self.use_local_wifi,
            "network": self.get_network_status(log=False),
        }

    def get_network_status(self, log=True):
        self._resolve_scan_interfaces(log=log)
        return dict(self.last_net_status)

    def capture_fingerprint(self, x=None, y=None, duration_sec=0.0, sample_interval_sec=0.25, average_recent_count=5):
        aps = self._sample_scan(duration_sec, sample_interval_sec, average_recent_count)
        if not aps and self.use_local_wifi:
            fallback_ap = self._read_current_connection_rssi()
            if fallback_ap and self._ap_allowed(fallback_ap):
                aps = [fallback_ap]

        return {
            "x": x,
            "y": y,
            "aps": aps,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }

    def _sample_scan(self, duration_sec=0.0, sample_interval_sec=0.25, average_recent_count=5):
        duration_sec = max(0.0, float(duration_sec or 0.0))
        sample_interval_sec = max(0.05, float(sample_interval_sec or 0.25))
        average_recent_count = max(1, int(average_recent_count or 5))

        if duration_sec <= 0.0:
            return self.scan()

        samples_by_key = {}
        deadline = time.monotonic() + duration_sec

        while time.monotonic() < deadline:
            for ap in self.scan():
                key = (
                    str(ap.get("bssid", "")).lower(),
                    str(ap.get("ssid", "")),
                    str(ap.get("band", "")),
                )
                samples = samples_by_key.setdefault(key, [])
                samples.append(dict(ap))
                if len(samples) > average_recent_count:
                    del samples[:-average_recent_count]
            time.sleep(min(sample_interval_sec, max(0.0, deadline - time.monotonic())))

        averaged = []
        for samples in samples_by_key.values():
            if not samples:
                continue
            latest = dict(samples[-1])
            rssis = [sample.get("rssi") for sample in samples if isinstance(sample.get("rssi"), (int, float))]
            if rssis:
                latest["rssi"] = round(sum(rssis) / len(rssis), 1)
                latest["rssi_samples"] = len(rssis)
            averaged.append(latest)

        return self._dedupe_aps(averaged)

    def scan(self):
        all_aps = []

        if self.use_esp32_wifi and self.esp32_wifi_source:
            try:
                all_aps.extend(self.esp32_wifi_source.get_esp32_wifi_aps())
            except Exception as exc:
                print(f"ESP32 Wi-Fi AP read failed: {exc}", flush=True)

        if self.use_local_wifi:
            interfaces = self._resolve_scan_interfaces(log=True)
            if not interfaces:
                print("Wi-Fi scan unavailable: no safe USB dongle scan interface.", flush=True)
                return self._dedupe_aps(self._normalize_aps(all_aps))
            for interface in interfaces:
                print(f"[WiFi] scanning RSSI interface: {interface}", flush=True)
                all_aps.extend(self._scan_interface(interface))

        aps = self._filter_target_aps(self._dedupe_aps(self._normalize_aps(all_aps)))
        print(f"[WiFi] scan result AP count: {len(aps)}", flush=True)
        return aps

    def _filter_target_aps(self, aps):
        if not self.target_ssids:
            return aps
        filtered = [ap for ap in aps if self._ap_allowed(ap)]
        print(
            f"[WiFi] target SSID filter: {len(filtered)}/{len(aps)} "
            f"({', '.join(sorted(self.target_ssids))})",
            flush=True,
        )
        return filtered

    def _ap_allowed(self, ap):
        if not self.target_ssids:
            return True
        return str((ap or {}).get("ssid", "")).strip().casefold() in self.target_ssids

    def _scan_interface(self, interface):
        connected = set(self._connected_wireless_interfaces())
        if interface in connected:
            print(f"[WiFi] skip connected interface scan: {interface}", flush=True)
            return []

        self._bring_interface_up(interface)

        try:
            result = subprocess.run(
                ["iw", "dev", interface, "scan"],
                capture_output=True,
                text=True,
                timeout=5,
            )
        except Exception as exc:
            print(f"Wi-Fi scan failed on {interface}: {exc}", flush=True)
            return []

        text = result.stdout + result.stderr
        aps = self._parse_iw_scan(text, interface)
        if aps:
            return aps

        try:
            result = subprocess.run(
                ["iwlist", interface, "scan"],
                capture_output=True,
                text=True,
                timeout=6,
            )
            return self._parse_iwlist_scan(result.stdout + result.stderr, interface)
        except Exception as exc:
            print(f"Wi-Fi scan fallback failed on {interface}: {exc}", flush=True)
            return []

    def close(self):
        self.set_local_wifi_power(False)

    def _resolve_scan_interfaces(self, log=True):
        all_wifi = self._discover_wireless_interfaces()
        connected = self._connected_wireless_interfaces()
        candidates = []

        configured_scan = self.scan_interfaces or [self.interface]
        discovered = all_wifi if self.scan_all_interfaces else []
        usb_candidates = self._usb_dongle_candidates(all_wifi) if self.auto_discover_usb_dongle else []

        for candidate in [*configured_scan, *discovered, *usb_candidates]:
            if candidate and candidate not in candidates:
                candidates.append(candidate)

        found = []
        skipped_connected = []
        for candidate in candidates:
            if self.exclude_connected_interfaces and candidate in connected:
                skipped_connected.append(candidate)
                continue
            try:
                result = subprocess.run(
                    ["ip", "link", "show", candidate],
                    capture_output=True,
                    text=True,
                    timeout=2,
                )
                if result.returncode == 0:
                    found.append(candidate)
            except Exception:
                continue

        if found:
            self.interface = found[0]
            scan_interfaces = found if self.scan_all_interfaces else found
        else:
            scan_interfaces = []

        excluded = list(dict.fromkeys([*connected, *skipped_connected]))
        self.last_net_status = {
            "all_wifi_interfaces": all_wifi,
            "connected_interfaces": connected,
            "scan_interfaces": scan_interfaces,
            "excluded_interfaces": excluded,
            "configured_scan_interfaces": list(configured_scan),
            "scan_all_interfaces": self.scan_all_interfaces,
            "auto_discover_usb_dongle": self.auto_discover_usb_dongle,
            "exclude_connected_interfaces": self.exclude_connected_interfaces,
            "wifi_scan_available": bool(scan_interfaces),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        if log:
            self._log_net_status_if_changed()
        return scan_interfaces

    def _log_net_status_if_changed(self):
        status = self.last_net_status or {}
        log_key = (
            tuple(status.get("all_wifi_interfaces", [])),
            tuple(status.get("connected_interfaces", [])),
            tuple(status.get("scan_interfaces", [])),
            tuple(status.get("excluded_interfaces", [])),
        )
        if log_key == self._last_net_status_log_key:
            return
        self._last_net_status_log_key = log_key
        for interface in status.get("connected_interfaces", []):
            print(f"[WiFi] connected interface skipped: {interface}", flush=True)
        print(f"[WiFi] all interfaces: {status.get('all_wifi_interfaces', [])}", flush=True)
        print(f"[WiFi] scan interfaces: {status.get('scan_interfaces', [])}", flush=True)
        print(f"[WiFi] excluded interfaces: {status.get('excluded_interfaces', [])}", flush=True)

    def _connected_wireless_interfaces(self):
        connected = []

        try:
            result = subprocess.run(
                ["nmcli", "-t", "-f", "DEVICE,TYPE,STATE", "dev", "status"],
                capture_output=True,
                text=True,
                timeout=3,
            )
            for line in result.stdout.splitlines():
                parts = line.split(":")
                if len(parts) >= 3 and parts[1] == "wifi" and parts[2] in ("connected", "connecting"):
                    if parts[0] not in connected:
                        connected.append(parts[0])
        except Exception:
            pass

        for interface in self._discover_wireless_interfaces():
            try:
                result = subprocess.run(
                    ["iw", "dev", interface, "link"],
                    capture_output=True,
                    text=True,
                    timeout=2,
                )
                if "Connected to" in (result.stdout + result.stderr) and interface not in connected:
                    connected.append(interface)
            except Exception:
                continue

        return connected

    def _discover_wireless_interfaces(self):
        interfaces = []

        try:
            result = subprocess.run(
                ["iw", "dev"],
                capture_output=True,
                text=True,
                timeout=3,
            )
            for match in re.finditer(r"^\s*Interface\s+(\S+)", result.stdout, re.MULTILINE):
                interfaces.append(match.group(1))
        except Exception:
            pass

        try:
            result = subprocess.run(
                ["find", "/sys/class/net", "-maxdepth", "1", "-type", "l", "-printf", "%f\n"],
                capture_output=True,
                text=True,
                timeout=3,
            )
            for name in result.stdout.splitlines():
                if name.startswith(("wl", "wlan")) and name not in interfaces:
                    interfaces.append(name)
        except Exception:
            pass

        return interfaces

    @staticmethod
    def _usb_dongle_candidates(interfaces):
        candidates = []
        for interface in interfaces:
            if interface.startswith("wlx") or interface.startswith("wlan1"):
                candidates.append(interface)
        return candidates

    def _bring_interface_up(self, interface):
        self._set_interface_power(interface, True)

    def _set_interface_power(self, interface, enabled):
        if not enabled and self._is_protected_connection_interface(interface):
            print(f"[WiFi] keep connected/protected interface UP: {interface}", flush=True)
            return

        try:
            subprocess.run(
                ["ip", "link", "set", interface, "up" if enabled else "down"],
                capture_output=True,
                text=True,
                timeout=2,
            )
        except Exception:
            pass

    def _is_protected_connection_interface(self, interface):
        if not interface:
            return True
        if interface == "wlan0":
            return True
        try:
            if interface in self._connected_wireless_interfaces():
                return True
        except Exception:
            pass
        try:
            with open("/proc/net/route", "r", encoding="utf-8") as route_file:
                for line in route_file.readlines()[1:]:
                    fields = line.split()
                    if len(fields) >= 2 and fields[0] == interface and fields[1] == "00000000":
                        return True
        except Exception:
            pass
        return False

    def _read_current_connection_rssi(self):
        interfaces = self._resolve_scan_interfaces(log=True)
        if not interfaces:
            return None
        interface = interfaces[0]
        try:
            result = subprocess.run(
                ["iwconfig", interface],
                capture_output=True,
                text=True,
                timeout=3,
            )
        except Exception:
            return None

        text = result.stdout + result.stderr
        signal_match = re.search(r"Signal level=(-?\d+)", text)
        freq_match = re.search(r"Frequency:([0-9.]+)\s*GHz", text)
        essid_match = re.search(r'ESSID:"([^"]*)"', text)

        if not signal_match:
            return None

        frequency_ghz = float(freq_match.group(1)) if freq_match else 2.4
        frequency_mhz = int(round(frequency_ghz * 1000.0))
        return WiFiScanner._normalize_ap({
            "ssid": essid_match.group(1) if essid_match else "Current AP",
            "bssid": "",
            "rssi": float(signal_match.group(1)),
            "frequency_mhz": frequency_mhz,
            "interface": interface,
            "source": "iwconfig",
        })

    @staticmethod
    def _parse_iw_scan(text, interface):
        aps = []
        current = {}

        for raw_line in text.splitlines():
            line = raw_line.strip()

            if line.startswith("BSS "):
                if current:
                    aps.append(current)
                current = {
                    "bssid": line.split()[1].split("(")[0],
                    "interface": interface,
                    "source": "iw",
                }
            elif line.startswith("SSID:") and current:
                current["ssid"] = line.partition(":")[2].strip()
            elif line.startswith("signal:") and current:
                match = re.search(r"(-?\d+(?:\.\d+)?)", line)
                if match:
                    current["rssi"] = float(match.group(1))
            elif line.startswith("freq:") and current:
                freq_text = line.partition(":")[2].strip()
                try:
                    frequency_mhz = int(round(float(freq_text)))
                except ValueError:
                    continue
                current["frequency_mhz"] = frequency_mhz
                current["channel"] = WiFiScanner._channel_from_frequency_mhz(frequency_mhz)
            elif "DS Parameter set:" in line and "channel" in line and current:
                match = re.search(r"channel\s+(\d+)", line)
                if match:
                    current["channel"] = int(match.group(1))

        if current:
            aps.append(current)

        return [WiFiScanner._normalize_ap(ap) for ap in aps if "rssi" in ap]

    @staticmethod
    def _parse_iwlist_scan(text, interface):
        aps = []
        cells = text.split("Cell ")

        for cell in cells[1:]:
            bssid_match = re.search(r"Address:\s*([0-9A-Fa-f:]{17})", cell)
            ssid_match = re.search(r'ESSID:"([^"]*)"', cell)
            signal_match = re.search(r"Signal level=(-?\d+)\s*dBm", cell)
            freq_match = re.search(r"Frequency:([0-9.]+)\s*GHz", cell)
            channel_match = re.search(r"\(Channel\s+(\d+)\)", cell)

            if not signal_match:
                continue

            frequency_mhz = None
            if freq_match:
                frequency_mhz = int(round(float(freq_match.group(1)) * 1000.0))
            aps.append(WiFiScanner._normalize_ap({
                "ssid": ssid_match.group(1) if ssid_match else "",
                "bssid": bssid_match.group(1) if bssid_match else "",
                "rssi": float(signal_match.group(1)),
                "frequency_mhz": frequency_mhz,
                "channel": int(channel_match.group(1)) if channel_match else None,
                "interface": interface,
                "source": "iwlist",
            }))

        return aps

    @staticmethod
    def _normalize_aps(aps):
        normalized = []
        for ap in aps:
            normalized_ap = WiFiScanner._normalize_ap(ap)
            if normalized_ap and "rssi" in normalized_ap:
                normalized.append(normalized_ap)
        return normalized

    @staticmethod
    def _normalize_ap(ap):
        if not ap:
            return None

        normalized = dict(ap)
        bssid = str(normalized.get("bssid", normalized.get("mac", "")) or "").strip()
        normalized["bssid"] = bssid.upper()
        normalized["mac"] = normalized["bssid"]
        normalized["ssid"] = str(normalized.get("ssid", "") or "")

        try:
            rssi = float(normalized.get("rssi"))
        except (TypeError, ValueError):
            return None
        normalized["rssi"] = rssi
        normalized["rssi_dbm"] = rssi

        channel = WiFiScanner._parse_int(normalized.get("channel"))
        frequency_mhz = WiFiScanner._parse_frequency_mhz(
            normalized.get("frequency_mhz", normalized.get("freq", normalized.get("frequency")))
        )
        if frequency_mhz is None and channel is not None:
            frequency_mhz = WiFiScanner._frequency_mhz_from_channel(channel)
        if channel is None and frequency_mhz is not None:
            channel = WiFiScanner._channel_from_frequency_mhz(frequency_mhz)

        normalized["channel"] = channel
        normalized["frequency"] = frequency_mhz
        normalized["frequency_mhz"] = frequency_mhz
        normalized["frequency_ghz"] = round(frequency_mhz / 1000.0, 3) if frequency_mhz is not None else None
        normalized["band"] = WiFiScanner._band_from_frequency_or_channel(frequency_mhz, channel)
        normalized["interface"] = str(normalized.get("interface", "") or "")
        normalized["source"] = str(normalized.get("source", "wifi_scan") or "wifi_scan")
        timestamp = normalized.get("timestamp") or normalized.get("scanned_at") or time.strftime("%Y-%m-%d %H:%M:%S")
        normalized["timestamp"] = timestamp
        normalized["scanned_at"] = timestamp
        normalized["fingerprint_key"] = normalized["bssid"]
        return normalized

    @staticmethod
    def _parse_int(value):
        try:
            if value is None or value == "":
                return None
            return int(float(value))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _parse_frequency_mhz(value):
        try:
            if value is None or value == "":
                return None
            frequency = float(value)
        except (TypeError, ValueError):
            return None

        if frequency < 100.0:
            return int(round(frequency * 1000.0))
        return int(round(frequency))

    @staticmethod
    def _band_from_frequency_or_channel(frequency_mhz, channel):
        if frequency_mhz is not None:
            return "5GHz" if frequency_mhz >= 5000 else "2.4GHz"
        if channel is not None:
            return "5GHz" if channel > 14 else "2.4GHz"
        return "unknown"

    @staticmethod
    def _frequency_mhz_from_channel(channel):
        if channel is None:
            return None
        if 1 <= channel <= 13:
            return 2407 + channel * 5
        if channel == 14:
            return 2484
        if channel >= 36:
            return 5000 + channel * 5
        return None

    @staticmethod
    def _channel_from_frequency_mhz(frequency_mhz):
        if frequency_mhz is None:
            return None
        if 2412 <= frequency_mhz <= 2472:
            return int(round((frequency_mhz - 2407) / 5))
        if frequency_mhz == 2484:
            return 14
        if 5000 <= frequency_mhz <= 5900:
            return int(round((frequency_mhz - 5000) / 5))
        return None

    @staticmethod
    def frequency_to_channel(freq_mhz):
        return WiFiScanner._channel_from_frequency_mhz(freq_mhz)

    @staticmethod
    def _dedupe_aps(aps):
        best_by_key = {}

        for ap in aps:
            key = (
                str(ap.get("bssid", "")).lower(),
                str(ap.get("ssid", "")),
                str(ap.get("band", "")),
            )
            if key not in best_by_key or ap.get("rssi", -999) > best_by_key[key].get("rssi", -999):
                best_by_key[key] = ap

        return sorted(best_by_key.values(), key=lambda ap: ap.get("rssi", -999), reverse=True)
