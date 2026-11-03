"""반복 JTS 데이터에서 식별한 하중 전달 곡선을 사용하는 서보 제어기."""
import json
from pathlib import Path

import numpy as np

from sim.trajectory import T_GRASP, T_RELEASE

MODE = "servo"
PROFILE_PATH = Path(__file__).resolve().parent / "results/load_transfer_identification/learned_profile.json"


class Controller:
    MODE = MODE

    def reset(self):
        profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        self.pick_t = 1e-3 * np.asarray(profile["pick_t_ms"], float) + T_GRASP
        self.pick_alpha = np.asarray(profile["pick_alpha"], float)
        self.place_t = 1e-3 * np.asarray(profile["place_t_ms"], float) + T_RELEASE
        self.place_alpha = np.asarray(profile["place_alpha"], float)
        self.alpha = 0.0

    def compute(self, t, th, thd, tau_s, grip, q_d, qd_d, qdd_d):
        if t < self.pick_t[0]:
            self.alpha = 0.0
        elif t <= self.pick_t[-1]:
            self.alpha = float(np.interp(t, self.pick_t, self.pick_alpha))
        elif t < self.place_t[0]:
            self.alpha = 1.0
        elif t <= self.place_t[-1]:
            self.alpha = float(np.interp(t, self.place_t, self.place_alpha))
        else:
            self.alpha = 0.0
        tau_empty = self.nom.inverse_dynamics(q_d, qd_d, qdd_d)
        tau_loaded = self.nom_box.inverse_dynamics(q_d, qd_d, qdd_d)
        self.payload_tau = tau_loaded - tau_empty
        tau_link = tau_empty + self.alpha * self.payload_tau
        return q_d, qd_d, qd_d, self.nom.B * qdd_d + tau_link
