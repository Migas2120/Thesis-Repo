# telemetry/health_monitor.py
from dataclasses import dataclass
from collections import deque
from typing import Optional, Deque, Tuple, Dict
import time
import math
import statistics

@dataclass
class TelemetryHealth:
    ok: bool
    degraded: bool
    reason: str
    pose_stale: bool
    gps_stale: bool
    jump_detected: bool
    vel_spike: bool
    jitter_high: bool
    gps_poor: bool
    nsats: Optional[int] = None
    hdop: Optional[float] = None
    pose_age_s: Optional[float] = None
    gps_age_s: Optional[float] = None

class TelemetryHealthMonitor:
    """
    Detects erratic/stale telemetry using robust stats (Hampel-like via MAD),
    jump checks, velocity spikes, and basic GPS quality gates (nsats/HDOP).
    """
    def __init__(
        self,
        window_sec: float = 3.0,
        max_stale_sec_pose: float = 3.0,
        max_stale_sec_gps: float = 4.0,
        max_jump_m: float = 5.0,
        max_vel_mps: float = 10.0,
        jitter_m_mad_limit: float = 1.0,
        mad_k: float = 3.5
    ):
        self.window_sec = window_sec
        self.max_stale_sec_pose = max_stale_sec_pose
        self.max_stale_sec_gps = max_stale_sec_gps
        self.max_jump_m = max_jump_m
        self.max_vel_mps = max_vel_mps
        self.jitter_m_mad_limit = jitter_m_mad_limit
        self.mad_k = mad_k

        self._pose_hist: Deque[Tuple[float, float, float, float]] = deque()   # (t,x,y,z)
        self._gps_hist: Deque[Tuple[float, float, float]] = deque()           # (t, hdop, nsats)

    @staticmethod
    def _dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        dx, dy, dz = a[0]-b[0], a[1]-b[1], a[2]-b[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    @staticmethod
    def _mad(values):
        if not values:
            return 0.0
        med = statistics.median(values)
        devs = [abs(v - med) for v in values]
        return statistics.median(devs)

    def _trim_window(self, now: float):
        cutoff = now - self.window_sec
        while self._pose_hist and self._pose_hist[0][0] < cutoff:
            self._pose_hist.popleft()
        while self._gps_hist and self._gps_hist[0][0] < cutoff:
            self._gps_hist.popleft()

    def update(
        self,
        pose: Optional[Dict],
        gps: Optional[Dict],
        now: Optional[float] = None
    ) -> TelemetryHealth:
        now = now if now is not None else time.time()
        pose_age = None
        gps_age = None

        # Record pose
        if pose and all(k in pose for k in ("x","y","z")):
            self._pose_hist.append((now, float(pose["x"]), float(pose["y"]), float(pose["z"])))
            pose_age = 0.0
        elif self._pose_hist:
            pose_age = now - self._pose_hist[-1][0]

        # Record GPS meta if available
        nsats = None
        hdop = None
        if gps:
            nsats = gps.get("satellites") or gps.get("nsats")
            hdop  = gps.get("hdop") or gps.get("h_acc")  # accept alt keys
            self._gps_hist.append((now, float(hdop) if hdop is not None else float("nan"),
                                        int(nsats) if nsats is not None else -1))
            gps_age = 0.0
        elif self._gps_hist:
            gps_age = now - self._gps_hist[-1][0]

        self._trim_window(now)

        pose_stale = (pose_age is not None and pose_age > self.max_stale_sec_pose)
        gps_stale  = (gps_age  is not None and gps_age  > self.max_stale_sec_gps)

        # Jump + velocity + jitter, from pose history
        jump_detected = False
        vel_spike = False
        jitter_high = False

        if len(self._pose_hist) >= 2:
            t2,x2,y2,z2 = self._pose_hist[-1]
            t1,x1,y1,z1 = self._pose_hist[-2]
            dt = max(t2 - t1, 1e-3)
            step = self._dist((x2,y2,z2),(x1,y1,z1))
            v = step / dt
            if step > self.max_jump_m:
                jump_detected = True
            if v > self.max_vel_mps:
                vel_spike = True

            # Jitter: MAD of per-step distances over window
            steps = []
            prev = None
            for (tt,xx,yy,zz) in self._pose_hist:
                if prev:
                    steps.append(self._dist((xx,yy,zz), prev))
                prev = (xx,yy,zz)
            if steps:
                mad = self._mad(steps)
                # Hampel-like: if MAD exceeds limit, mark jitter
                if mad > self.jitter_m_mad_limit:
                    jitter_high = True

        # GPS quality (soft gates)
        gps_poor = False
        if self._gps_hist:
            # last few readings
            recent = list(self._gps_hist)[-min(5, len(self._gps_hist)):]
            ns_list = [ns for (_, _, ns) in recent if ns is not None and ns >= 0]
            hd_list = [hd for (_, hd, _) in recent if not math.isnan(hd)]
            avg_ns  = sum(ns_list)/len(ns_list) if ns_list else None
            avg_hd  = sum(hd_list)/len(hd_list) if hd_list else None

            # Typical indoor: nsats low, hdop high. Tune thresholds to your GNSS.
            if (avg_ns is not None and avg_ns < 6) or (avg_hd is not None and avg_hd > 2.0):
                gps_poor = True
            nsats = int(avg_ns) if avg_ns is not None else nsats
            hdop = float(avg_hd) if avg_hd is not None else hdop

        degraded_flags = [pose_stale, gps_stale, jump_detected, vel_spike, jitter_high, gps_poor]
        degraded = any(degraded_flags)
        ok = not degraded

        reason_bits = []
        if pose_stale:    reason_bits.append("pose_stale")
        if gps_stale:     reason_bits.append("gps_stale")
        if jump_detected: reason_bits.append("jump")
        if vel_spike:     reason_bits.append("vel_spike")
        if jitter_high:   reason_bits.append("jitter")
        if gps_poor:      reason_bits.append("gps_poor")

        return TelemetryHealth(
            ok=ok,
            degraded=degraded,
            reason="|".join(reason_bits) if reason_bits else "ok",
            pose_stale=pose_stale,
            gps_stale=gps_stale,
            jump_detected=jump_detected,
            vel_spike=vel_spike,
            jitter_high=jitter_high,
            gps_poor=gps_poor,
            nsats=nsats,
            hdop=hdop,
            pose_age_s=pose_age,
            gps_age_s=gps_age,
        )
