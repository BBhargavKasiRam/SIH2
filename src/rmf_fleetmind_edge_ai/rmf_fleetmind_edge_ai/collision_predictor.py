"""
FleetMind Edge AI - Collision Predictor (Deterministic Kinematics + ML Risk Inference).
"""
from __future__ import annotations

import math
import os
from typing import Optional, Tuple, Any

try:
    import joblib
    import pandas as pd
    ML_AVAILABLE = True
except ImportError:
    ML_AVAILABLE = False
    joblib = None
    pd = None

DEFAULT_ROBOT_RADIUS = 0.3
DEFAULT_SAFETY_MARGIN = 0.5


def calculate_ttc(
    mx0: float, my0: float, mvx: float, mvy: float,
    px0: float, py0: float, pvx: float, pvy: float,
    robot_radius: float = DEFAULT_ROBOT_RADIUS,
    safety_margin: float = DEFAULT_SAFETY_MARGIN,
    horizon: float = 5.0,
    dt: float = 0.1
) -> Tuple[Optional[float], float, str]:
    """
    Simulates linear trajectory over `horizon` seconds with `dt` steps.
    Returns (ttc_seconds, min_separation, risk_level).
    Risk levels: "SAFE", "WARNING", "CRITICAL"
    """
    unsafe_distance = (robot_radius * 2.0) + safety_margin
    warning_distance = unsafe_distance * 1.5

    min_sep = float('inf')
    ttc = None

    t = 0.0
    while t <= horizon + 1e-5:
        mx = mx0 + mvx * t
        my = my0 + mvy * t
        px = px0 + pvx * t
        py = py0 + pvy * t

        sep = math.hypot(mx - px, my - py)
        if sep < min_sep:
            min_sep = sep

        if ttc is None and sep < unsafe_distance:
            ttc = t

        t += dt

    if min_sep < unsafe_distance:
        if ttc is not None and ttc <= (horizon * 0.4):
            return ttc, min_sep, "CRITICAL"
        else:
            return ttc, min_sep, "WARNING"
    elif min_sep < warning_distance:
        return ttc, min_sep, "WARNING"

    return None, min_sep, "SAFE"


class MLCollisionPredictor:
    """
    Combines kinematic trajectory simulation with ML classification.
    Gracefully falls back to deterministic kinematics if ML is unavailable.
    """

    def __init__(self, model_path: Optional[str] = None):
        self.model_path = model_path
        self.model_dict: Optional[dict[str, Any]] = None
        self.ml_loaded = False

        if model_path and os.path.exists(model_path) and ML_AVAILABLE:
            try:
                loaded = joblib.load(model_path)
                if isinstance(loaded, dict) and 'model' in loaded:
                    self.model_dict = loaded
                    self.ml_loaded = True
            except Exception as e:
                self.ml_loaded = False

    def predict_risk(
        self,
        mx: float, my: float, mvx: float, mvy: float,
        px: float, py: float, pvx: float, pvy: float,
        robot_radius: float = DEFAULT_ROBOT_RADIUS,
        safety_margin: float = DEFAULT_SAFETY_MARGIN,
        horizon: float = 5.0,
        dt: float = 0.1
    ) -> Tuple[Optional[float], float, str, str]:
        """
        Computes collision metrics and classifies risk.
        Returns (ttc, min_sep, risk_level, method_used).
        Method used is either 'ML' or 'DETERMINISTIC'.
        """
        det_ttc, min_sep, det_risk = calculate_ttc(
            mx, my, mvx, mvy,
            px, py, pvx, pvy,
            robot_radius=robot_radius,
            safety_margin=safety_margin,
            horizon=horizon,
            dt=dt
        )

        if not self.ml_loaded or not ML_AVAILABLE or pd is None:
            return det_ttc, min_sep, det_risk, "DETERMINISTIC"

        try:
            dx = px - mx
            dy = py - my
            dvx = pvx - mvx
            dvy = pvy - mvy
            dist = math.hypot(dx, dy)
            rel_heading = math.atan2(dvy, dvx + 1e-6) - math.atan2(mvy, mvx + 1e-6)
            det_ttc_val = det_ttc if det_ttc is not None else horizon

            features = pd.DataFrame([{
                'dx': dx,
                'dy': dy,
                'dvx': dvx,
                'dvy': dvy,
                'dist': dist,
                'rel_heading': rel_heading,
                'det_ttc': det_ttc_val,
                'det_min_sep': min_sep
            }])

            model = self.model_dict['model']
            scaler = self.model_dict.get('scaler', None)
            X = scaler.transform(features) if scaler is not None else features

            ml_pred = int(model.predict(X)[0])
            if ml_pred == 2:
                ml_risk = "CRITICAL"
            elif ml_pred == 1:
                ml_risk = "WARNING"
            else:
                ml_risk = "SAFE"

            # Fail-safe consensus: if deterministic says CRITICAL, trust the worse risk
            if det_risk == "CRITICAL" and ml_risk != "CRITICAL":
                return det_ttc, min_sep, "CRITICAL", "SAFETY_CONSENSUS"

            return det_ttc, min_sep, ml_risk, "ML"

        except Exception:
            return det_ttc, min_sep, det_risk, "DETERMINISTIC"
