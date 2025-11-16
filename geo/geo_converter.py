# geo/geo_converter.py
import math
from typing import Tuple

# WGS-84
_A  = 6378137.0              # semi-major axis (m)
_E2 = 6.69437999014e-3       # first eccentricity squared

def _radii(lat_rad: float) -> Tuple[float, float]:
    # Radii of curvature: meridian (M) and prime vertical (N)
    sinl = math.sin(lat_rad)
    denom = math.sqrt(1.0 - _E2 * sinl * sinl)
    N = _A / denom
    M = _A * (1.0 - _E2) / (denom**3)
    return M, N

class GeoConverter:
    """
    Simple local ENU <-> LLA around a small area.
    Assumes local frame X=East, Y=North, Z=Up.
    """
    def __init__(self):
        self.has_origin = False
        self.lat0 = self.lon0 = self.alt0 = 0.0
        self.x0 = self.y0 = self.z0 = 0.0
        self._lat0_rad = 0.0
        self._M = self._N = 0.0

    def set_origin(self, lat0: float, lon0: float, alt0: float,
                   x0: float = 0.0, y0: float = 0.0, z0: float = 0.0):
        self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
        self.x0, self.y0, self.z0 = x0, y0, z0
        self._lat0_rad = math.radians(lat0)
        self._M, self._N = _radii(self._lat0_rad)
        self.has_origin = True

    def enu_to_lla(self, x: float, y: float, z: float):
        if not self.has_origin:
            raise RuntimeError("GeoConverter origin not set")
        # local deltas relative to local origin
        de = x - self.x0   # East
        dn = y - self.y0   # North
        du = z - self.z0   # Up
        # Convert meters to degrees using local radii
        dlat = (dn / self._M) * (180.0 / math.pi)
        dlon = (de / (self._N * math.cos(self._lat0_rad))) * (180.0 / math.pi)
        lat = self.lat0 + dlat
        lon = self.lon0 + dlon
        alt = self.alt0 + du
        return lat, lon, alt