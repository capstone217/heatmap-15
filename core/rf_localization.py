import math


MISSING_RSSI_DBM = -100.0


def normalize_bssid(value):
    return str(value or "").strip().upper()


def aps_to_rssi_vector(aps):
    vector = {}
    for ap in aps or []:
        bssid = normalize_bssid(ap.get("bssid") or ap.get("mac"))
        if not bssid:
            continue
        try:
            rssi = float(ap.get("rssi"))
        except (TypeError, ValueError):
            continue
        if math.isfinite(rssi):
            vector[bssid] = rssi
    return vector


class RFLocalizer:
    def __init__(self, missing_rssi=MISSING_RSSI_DBM):
        self.missing_rssi = float(missing_rssi)

    def euclidean_distance(self, current_vector, db_vector):
        current_vector = self._normalize_vector(current_vector)
        db_vector = self._normalize_vector(db_vector)
        bssids = set(current_vector) | set(db_vector)
        if not bssids:
            return None

        total = 0.0
        for bssid in bssids:
            current_rssi = current_vector.get(bssid, self.missing_rssi)
            db_rssi = db_vector.get(bssid, self.missing_rssi)
            total += (current_rssi - db_rssi) ** 2
        return math.sqrt(total)

    def estimate_location(self, current_vector, grid_vectors, k=3):
        current_vector = self._normalize_vector(current_vector)
        if not current_vector:
            raise ValueError("current_vector is empty")
        if not grid_vectors:
            raise ValueError("DB fingerprint vectors are empty")

        candidates = []
        for grid_id, grid_data in grid_vectors.items():
            vector = self._normalize_vector((grid_data or {}).get("vector", {}))
            if not vector:
                continue
            distance = self.euclidean_distance(current_vector, vector)
            if distance is None:
                continue
            candidates.append(
                {
                    "grid_id": grid_id,
                    "distance": distance,
                    "x": (grid_data or {}).get("x_center"),
                    "y": (grid_data or {}).get("y_center"),
                    "floor": (grid_data or {}).get("floor"),
                }
            )

        if not candidates:
            raise ValueError("No usable DB fingerprint vectors")

        candidates.sort(key=lambda item: item["distance"])
        k = max(1, min(int(k or 1), len(candidates)))
        nearest = candidates[:k]
        estimated = self._weighted_average(nearest)
        best = nearest[0]

        return {
            "estimated_x": estimated["x"],
            "estimated_y": estimated["y"],
            "estimated_floor": estimated.get("floor") or best.get("floor"),
            "estimated_grid_id": best["grid_id"],
            "nearest_candidates": [
                {
                    "grid_id": item["grid_id"],
                    "distance": round(item["distance"], 3),
                    "x": item.get("x"),
                    "y": item.get("y"),
                    "floor": item.get("floor"),
                }
                for item in nearest
            ],
        }

    def _weighted_average(self, candidates):
        exact = [item for item in candidates if item["distance"] <= 1e-9]
        if exact:
            item = exact[0]
            return {"x": item.get("x"), "y": item.get("y"), "floor": item.get("floor")}

        weighted_x = 0.0
        weighted_y = 0.0
        total_weight = 0.0
        floor_weights = {}

        for item in candidates:
            x = item.get("x")
            y = item.get("y")
            if x is None or y is None:
                continue
            weight = 1.0 / max(item["distance"], 1e-9)
            weighted_x += float(x) * weight
            weighted_y += float(y) * weight
            total_weight += weight
            floor = item.get("floor")
            if floor:
                floor_weights[floor] = floor_weights.get(floor, 0.0) + weight

        if total_weight <= 0:
            item = candidates[0]
            return {"x": item.get("x"), "y": item.get("y"), "floor": item.get("floor")}

        floor = None
        if floor_weights:
            floor = max(floor_weights.items(), key=lambda item: item[1])[0]
        return {
            "x": weighted_x / total_weight,
            "y": weighted_y / total_weight,
            "floor": floor,
        }

    @staticmethod
    def _normalize_vector(vector):
        normalized = {}
        for bssid, rssi in (vector or {}).items():
            key = normalize_bssid(bssid)
            if not key:
                continue
            try:
                value = float(rssi)
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                normalized[key] = value
        return normalized


if __name__ == "__main__":
    localizer = RFLocalizer()
    current = {
        "AA:BB:CC:11:22:33": -47,
        "AA:BB:CC:44:55:66": -70,
        "AA:BB:CC:77:88:99": -80,
    }
    grids = {
        "F1_4_2": {
            "x_center": 1.35,
            "y_center": 0.75,
            "floor": "F1",
            "vector": {
                "AA:BB:CC:11:22:33": -45.2,
                "AA:BB:CC:44:55:66": -68.1,
                "AA:BB:CC:77:88:99": -82.0,
            },
        }
    }
    print(localizer.estimate_location(current, grids))
