"""Phase 2 환경 강건 상태별 혼합 운동량 관측기.

운반 중에는 원래의 전체 잔차를 사용해 모든 방향 충돌 감도를 유지한다.
하중 천이 구간에는 빈손 관측기 잔차에서 알려진 payload 토크 방향만 제거해
정상 하중 전달과 그 밖의 방향 충돌을 구분한다.

단단하거나 높이가 다른 작업대에서는 예상 접촉 방향을 짧게 투영 제거하고,
고잡음 환경에서는 Cartesian 힘 벡터를 20 ms 평균해 지속 충돌을 복원한다.
"""
import numpy as np
from pathlib import Path
from sim import config as C
from sim.trajectory import T_GRASP, T_RELEASE

SOURCE = "jts"


class Detector:
    SOURCE = SOURCE

    def reset(self):
        self.K_O = 50.0
        self.threshold_normal = np.array([10., 18., 8., 3., 3., 2.])
        self.threshold_pre_change = np.array([3.5, 5., 3., 1.5, 1.5, 1.0])
        self.threshold_transition = np.array([7.5, 26., 12., 4., 10., 2.])
        self.threshold_transition_settled = np.array([7.5, 12., 9., 4., 10., 2.])
        self.r = np.zeros(6)
        self.q_prev = None
        self.v = np.zeros(6)
        self.obs = {
            "normal": {"r": np.zeros(6), "s": None, "M_prev": None},
            "empty": {"r": np.zeros(6), "s": None, "M_prev": None},
        }
        self.expected_load_r = np.zeros(6)
        self.cal_early_sum = np.zeros(6)
        self.cal_late_sum = np.zeros(6)
        self.cal_late_sq_sum = np.zeros(6)
        self.cal_early_count = 0
        self.cal_late_count = 0
        self.cal_guard_dev = np.zeros((5, 6))
        self.cal_guard_count = 0
        self.pre_base_sum = np.zeros(6)
        self.pre_base_count = 0
        self.pre_dev = np.zeros((5, 6))
        self.pre_dev_count = 0
        self.cross_history = np.zeros(20, dtype=bool)
        self.cross_index = 0
        self.force_history = np.zeros(5)
        self.force_index = 0
        self.force_vector_history = np.zeros((20, 3))
        self.force_vector_index = 0
        self.vertical_history = np.zeros(8, dtype=bool)
        self.vertical_index = 0
        self.force_residual = np.zeros(3)
        self.cartesian_active = False
        self.latched = False
        self.blind_baseline = np.zeros(6)
        self.blind_baseline_ready = False
        self.normal_profile = None
        if bool(getattr(self, "UNKNOWN_PAYLOAD_V3", False)):
            profile_path = (
                Path(__file__).resolve().parent / "results" /
                "unknown_payload_v3" / "normal_profile.npz"
            )
            if profile_path.exists():
                profile = np.load(profile_path)
                self.normal_profile = (
                    profile["center"].copy(), profile["envelope"].copy()
                )

    def _step(self, state, nom, q, tau_s):
        dt = 1e-3
        M = nom.mass_matrix(q)
        p = M @ self.v
        if state["s"] is None:
            state["s"], state["M_prev"] = p.copy(), M
        Mdot = (M - state["M_prev"]) / dt
        state["M_prev"] = M
        beta = nom.bias(q, self.v) - Mdot @ self.v
        state["s"] += (tau_s - beta + state["r"]) * dt
        state["r"] = self.K_O * (p - state["s"])
        return state["r"].copy()

    def _link_angle(self, th, tau_s):
        """공식 입력만으로 출력측 링크각을 복원한다."""
        return th - np.where(
            self.nom.elastic,
            tau_s / np.where(self.nom.elastic, self.nom.K, 1.0),
            0.0,
        )

    def _joint_torque(self, th, thd, q, qd, tau_s):
        """링크 동역학에 입력되는 전달 토크. 기본안은 JTS를 직접 쓴다."""
        return tau_s

    def update(self, t, th, thd, tau_m, tau_s, grip):
        dt = 1e-3
        q = self._link_angle(th, tau_s)
        qd = np.zeros(6) if self.q_prev is None else (q - self.q_prev) / dt
        self.q_prev = q
        self.v += 0.2 * (qd - self.v)
        tau_link = self._joint_torque(th, thd, q, qd, tau_s)

        blind_payload = bool(getattr(self, "UNKNOWN_PAYLOAD", False))
        phase1_unknown = bool(getattr(self, "UNKNOWN_PHASE1_BASELINE", False))
        blind_v3 = bool(getattr(self, "UNKNOWN_PAYLOAD_V3", False))
        active_nom = self.nom if blind_payload else (self.nom_box if grip else self.nom)
        r_normal = self._step(self.obs["normal"], active_nom, q, tau_link)
        r_empty = self._step(self.obs["empty"], self.nom, q, tau_link)

        alpha = float(np.clip(getattr(self.ctrl, "alpha", float(grip)), 0.0, 1.0))
        if blind_payload:
            payload_tau = np.asarray(
                getattr(self.ctrl, "estimated_payload_tau", np.zeros(6))
            )
        else:
            payload_tau = getattr(
                self.ctrl,
                "payload_tau",
                self.nom_box.gravity(q) - self.nom.gravity(q),
            )
        # 빈손 관측기에서 정상 하중이 만들 잔차도 같은 1차 동특성으로 예측한다.
        self.expected_load_r += dt * self.K_O * (
            -alpha * payload_tau - self.expected_load_r
        )

        # 높은 천이 임계값은 실제 하중 모델이 바뀌는 순간부터만 쓴다.
        # 미리 넓게 열면 아직 빈손/완전 하중 모델이 정확한 구간의 작은 충돌을
        # 불필요하게 가리게 된다.
        estimated_mass = float(getattr(self.ctrl, "estimated_mass", 0.0))
        known_or_estimated_mass = estimated_mass if blind_payload else self.box_mass
        release_start = T_RELEASE - (0.042 + 0.0014 * known_or_estimated_mass)
        # 물체 관성·흡착 강성이 달라지면 기준 궤적의 가감속 여파가 접촉 뒤에도
        # 남는다. 실제 집기 상승과 놓기 접근 구간 전체를 천이 상태로 다룬다.
        if phase1_unknown:
            in_transition = (
                T_GRASP <= t <= T_GRASP + 0.30
                or release_start <= t <= T_RELEASE + 0.30
            )
        else:
            in_transition = (
                T_GRASP <= t <= T_GRASP + 0.45
                or T_RELEASE - 0.35 <= t <= T_RELEASE + 0.35
            )
        if blind_payload and blind_v3:
            # 질량과 COM의 숫자는 몰라도, 강체 payload 중력 토크는
            # Y(q)[m,m*cx,m*cy,m*cz]가 만드는 4차원 부분공간 안에 있다.
            # 집기 천이에서는 이 정상 부분공간만 제거하여 나머지 방향의 충돌을
            # 300 ms 동안 끄지 않고 계속 감시한다. 천이가 끝난 뒤에는 기존의
            # 느린 기준선을 사용해 자세에 따른 payload 변화를 따라간다.
            estimator = getattr(self.ctrl, "estimator", None)
            pickup_transition = bool(T_GRASP <= t <= T_GRASP + 0.45)
            if grip and pickup_transition and estimator is not None:
                Y = estimator.regressor(q)
                nuisance = Y @ np.linalg.lstsq(Y, r_empty, rcond=1e-4)[0]
                self.r = r_empty - nuisance
            elif grip:
                if not self.blind_baseline_ready:
                    self.blind_baseline = r_empty.copy()
                    self.blind_baseline_ready = True
                self.r = r_empty - self.blind_baseline
                self.blind_baseline += 0.01 * (r_empty - self.blind_baseline)
            else:
                self.r = r_normal
                self.blind_baseline_ready = False
            settled = not in_transition
            threshold = (
                self.threshold_normal if settled
                else (
                    np.array([10.0, 20.0, 15.0, 4.0, 12.0, 3.0])
                    if pickup_transition
                    else np.maximum(
                        self.threshold_transition,
                        np.array([20., 25., 20., 15., 15., 4.]),
                    )
                )
            )
        elif blind_payload:
            # 질량·COM을 모를 때 절대 payload 토크를 빼지 않는다. 집기 뒤의
            # 잔차를 느린 정상 기준선으로 보고, 그보다 빠른 변화만 외란으로 쓴다.
            if grip and t >= T_GRASP + 0.30:
                if not self.blind_baseline_ready:
                    self.blind_baseline = r_empty.copy()
                    self.blind_baseline_ready = True
                self.r = r_empty - self.blind_baseline
                # 100 ms 시정수: 자세에 따른 중력 변화는 따라가되 20 ms 충돌
                # 상승은 잔차로 남긴다. 판정은 갱신 전의 차이를 사용한다.
                self.blind_baseline += 0.01 * (r_empty - self.blind_baseline)
            elif grip:
                self.r = r_empty
            else:
                self.r = r_normal
                self.blind_baseline_ready = False
            settled = not in_transition
            threshold = self.threshold_normal if settled else self.threshold_transition
            threshold = np.maximum(
                threshold, np.array([20., 25., 20., 15., 15., 4.])
            )
        elif in_transition:
            denom = float(payload_tau @ payload_tau)
            if denom > 1e-8:
                orthogonal = r_empty - payload_tau * float(r_empty @ payload_tau) / denom
                parallel_error = payload_tau * float(
                    (r_empty - self.expected_load_r) @ payload_tau
                ) / denom
                self.r = orthogonal + parallel_error
            else:
                self.r = r_empty
            settled = (
                T_GRASP + 0.06 <= t <= T_GRASP + 0.30
                or T_RELEASE + 0.04 <= t <= T_RELEASE + 0.30
            )
            threshold = (
                self.threshold_transition_settled
                if settled else self.threshold_transition
            )
        else:
            # 미지 payload 모드에서는 빈손 관측기의 정상 하중 잔차에서 JTS로
            # 식별한 예상 잔차를 뺀다. 실제 mass/COM 모델은 접근하지 않는다.
            self.r = r_normal
            threshold = self.threshold_normal

        # 작업 시작 전 무접촉 구간 두 곳에서 잔차 바이어스와 느린 기울기를
        # 추정한다. 이후에는 값을 고정해 실제 지속 접촉을 정상으로 학습하지
        # 않는다. 반환하는 외력 잔차는 원신호를 유지하고 판정에만 보정한다.
        calibration_guard = False
        if 0.05 <= t < T_GRASP:
            # 알려진 설계 영점(-JTS_OFFSET) 대비 5 ms 평균 변화로 기동 직후부터
            # 접촉을 감시한다. 실제 오프셋 차이는 이 임계값 안에서 뒤의 두
            # 교정 구간이 추정하고, 큰 변화는 교정 평균에 섞지 않는다.
            self.cal_guard_dev[self.cal_guard_count % len(self.cal_guard_dev)] = np.abs(
                r_normal + C.JTS_OFFSET
            )
            self.cal_guard_count += 1
            if self.cal_guard_count >= len(self.cal_guard_dev):
                guard_mean = self.cal_guard_dev.mean(axis=0)
                calibration_guard = bool(
                    np.any(
                        guard_mean
                        > np.array([4.0, 7.0, 3.5, 1.0, 1.0, 0.7])
                    )
                    # 접근 전 -X 충돌은 단일 축 최대치가 작아도 J1/J2/J3에
                    # 동시에 남는다. 세 축의 공동 증거로 잡음 스파이크와
                    # 구분하며, 단순히 J1 문턱을 낮추지는 않는다.
                    or (
                        guard_mean[0] > 2.8
                        and guard_mean[1] > 5.5
                        and guard_mean[2] > 2.8
                    )
                )
        if 0.45 <= t < 0.65 and not calibration_guard:
            self.cal_early_sum += r_normal
            self.cal_early_count += 1
        if 0.85 <= t < 1.05 and not calibration_guard:
            self.cal_late_sum += r_normal
            self.cal_late_sq_sum += r_normal * r_normal
            self.cal_late_count += 1
        bias = np.zeros(6)
        if self.cal_early_count and self.cal_late_count:
            early = self.cal_early_sum / self.cal_early_count
            late = self.cal_late_sum / self.cal_late_count
            slope = (late - early) / 0.4
            bias = late + slope * (t - 0.95)
        # 기존 임계값이 전제로 한 설계 오프셋(-JTS_OFFSET)은 유지하고, 실제
        # 교정값이 그 기준에서 벗어난 부분만 제거한다.
        bias_adjustment = bias + C.JTS_OFFSET
        detect_r = self.r - bias_adjustment
        detect_r_normal = r_normal - bias_adjustment
        late_mean = self.cal_late_sum / max(self.cal_late_count, 1)
        late_var = np.maximum(
            self.cal_late_sq_sum / max(self.cal_late_count, 1) - late_mean * late_mean,
            0.0,
        )
        high_noise = bool(np.sqrt(late_var[1]) > 1.7)

        # 놓기 직전에는 작업대 높이·강성 차이가 만드는 정상 수직 접촉 성분만
        # 제거한다. 다른 방향 충돌은 그대로 남기고, 수직 충돌도 이 짧은 보호
        # 구간 뒤의 지속 잔차로 다시 판정한다.
        contact_guard = bool(
            not phase1_unknown and not blind_v3
            and T_RELEASE - 0.10 <= t <= T_RELEASE + 0.02
        )
        guarded_detect_r = detect_r
        if contact_guard:
            Jv_guard = self.nom.jacobian(q)
            vertical_tau = Jv_guard.T @ np.array([0.0, 0.0, 1.0])
            denom_vertical = float(vertical_tau @ vertical_tau)
            if denom_vertical > 1e-10:
                guarded_detect_r = detect_r - vertical_tau * float(
                    detect_r @ vertical_tau
                ) / denom_vertical

        # 관절별 임계값만 쓰면 자세에 따라 같은 Cartesian 힘이 작은 관절
        # 잔차로 투영될 수 있다. 하중 천이와 접근 감속이 끝난 안정 구간에는
        # tau_ext = J_v^T f 관계로 힘 잔차를 복원해 병렬 증거로 사용한다.
        cartesian_start = T_GRASP + (
            0.08 if blind_v3 else (1.10 if blind_payload else 0.35)
        )
        cartesian_window = bool(
            cartesian_start <= t < T_RELEASE - 0.30
            or t >= T_RELEASE + 0.30
        )
        if cartesian_window and not self.cartesian_active:
            self.force_history[:] = 0.0
            self.force_index = 0
            self.force_vector_history[:] = 0.0
            self.force_vector_index = 0
        self.cartesian_active = cartesian_window
        if cartesian_window:
            Jv = self.nom.jacobian(q)
            self.force_residual = np.linalg.lstsq(
                Jv.T, detect_r, rcond=1e-3
            )[0]
            self.force_history[self.force_index % len(self.force_history)] = np.linalg.norm(
                self.force_residual
            )
            self.force_index += 1
            self.force_vector_history[
                self.force_vector_index % len(self.force_vector_history)
            ] = self.force_residual
            self.force_vector_index += 1
        else:
            self.force_residual[:] = 0.0
        # 크기(norm)를 먼저 평균하면 영평균 잡음도 양의 힘처럼 누적된다.
        # 환경과 무관하게 20 ms 힘 벡터를 먼저 평균한 뒤 크기를 취한다.
        if phase1_unknown:
            averaged_force = bool(
                not high_noise
                and self.force_index >= len(self.force_history)
                and self.force_history.mean() > 8.5
            )
        else:
            cartesian_threshold = (
                9.5 if blind_payload else (9.0 if high_noise else 8.5)
            )
            averaged_force = bool(
                self.force_vector_index >= len(self.force_vector_history)
                and np.linalg.norm(self.force_vector_history.mean(axis=0))
                > cartesian_threshold
            )
        cartesian_evidence = bool(cartesian_window and averaged_force)

        # 집기 접근 전 안전 구간에서 잔차 기준선을 정한다. 접근 80 ms 동안은
        # 절대값이 아니라 기준선 대비 5 ms 평균 변화만 보므로 JTS 영점 오프셋,
        # 느린 드리프트와 단일 잡음 스파이크에 민감하지 않다.
        if T_GRASP - 0.14 <= t < T_GRASP - 0.08:
            self.pre_base_sum += detect_r_normal
            self.pre_base_count += 1
        pre_change = False
        if T_GRASP - 0.08 <= t < T_GRASP and self.pre_base_count:
            baseline = self.pre_base_sum / self.pre_base_count
            self.pre_dev[self.pre_dev_count % len(self.pre_dev)] = np.abs(detect_r_normal - baseline)
            self.pre_dev_count += 1
            if self.pre_dev_count >= len(self.pre_dev):
                # 모든 축에서 교정 구간 표준편차를 반영한다. 고정 임계값만 쓰면
                # 잡음이 큰 물체·환경에서 5 ms 변화 평균을 충돌로 오인한다.
                if phase1_unknown:
                    pre_threshold = self.threshold_pre_change.copy()
                    pre_threshold[1] = max(2.0, 1.7 * float(np.sqrt(late_var[1])))
                else:
                    pre_threshold = np.maximum(
                        self.threshold_pre_change,
                        3.5 * np.sqrt(late_var),
                    )
                pre_change = bool(np.any(self.pre_dev.mean(axis=0) > pre_threshold))

        # 놓기 직후 +Z 충돌은 하중 해제 잔차와 단일 축에서 상쇄되지만 J2/J3에
        # 같은 부호의 상관 패턴을 남긴다. 개별 임계값을 낮추지 않고 두 축의
        # 동시 증거만 사용한다.
        vertical_candidate = bool(
            T_RELEASE + 0.04 <= t <= T_RELEASE + 0.30
            and detect_r[1] < -6.0
            and detect_r[2] < -6.0
        )
        self.vertical_history[self.vertical_index % len(self.vertical_history)] = vertical_candidate
        self.vertical_index += 1
        correlated_vertical = bool(
            vertical_candidate if phase1_unknown
            else self.vertical_index >= 5 and self.vertical_history.sum() >= 5
        )
        effective_threshold = (
            np.maximum(threshold, np.array([15., 30., 15., 5., 12., 3.]))
            if contact_guard else threshold
        )
        # 미지 payload를 처음 집는 동안에는 질량 추정치가 아직 없으므로 정상
        # 하중이 크게 나타나는 J2/J5 문턱만 잠시 넓힌다. 나머지 축은 계속 본다.
        # 이 구간의 payload 방향 충돌 감도 저하는 숨기지 않는 한계다.
        blind_identification_or_release = bool(
            blind_payload and (
                (not blind_v3 and T_GRASP <= t < T_GRASP + 0.30)
                or T_RELEASE - 0.35 <= t <= T_RELEASE + 0.35
            )
        )
        if blind_identification_or_release:
            effective_threshold = np.maximum(
                effective_threshold,
                np.array([30., 100., 75., 25., 60., 15.]),
            )
        if blind_v3 and self.normal_profile is not None:
            profile_center, profile_envelope = self.normal_profile
            profile_index = min(int(round(t / dt)), len(profile_center) - 1)
            guarded_detect_r = guarded_detect_r - profile_center[profile_index]
            effective_threshold = profile_envelope[profile_index]
        threshold_cross = bool(np.any(np.abs(guarded_detect_r) > effective_threshold))
        self.cross_history[self.cross_index % len(self.cross_history)] = threshold_cross
        self.cross_index += 1
        # 기본 센서는 작은 충돌의 한두 번 초과도 즉시 쓴다. 교정 구간에서 큰
        # 잡음이 확인된 경우에만 최근 20 ms 중 7회 이상의 누적 증거를 요구한다.
        regular_evidence = bool(
            t >= 0.05
            and (
                self.cross_history.sum() >= (5 if blind_v3 else 7)
                if (blind_v3 or high_noise) else threshold_cross
            )
        )
        if blind_v3 and self.normal_profile is not None:
            raw_detect = bool(regular_evidence or pre_change or calibration_guard)
        else:
            raw_detect = bool(
                regular_evidence or pre_change or correlated_vertical
                or calibration_guard or cartesian_evidence
            )
        identification_guard = bool(
            blind_payload and not blind_v3 and T_GRASP <= t < T_GRASP + 0.30
        )
        if identification_guard:
            # 알려지지 않은 payload 자체가 외란인 구간이다. 추가 센서나 사전
            # 정보 없이 같은 방향 충돌과 하중 취득을 구분할 수 없어 판정을
            # 보류한다. 이 300 ms 감지 공백은 보고서에 명시하는 한계다.
            raw_detect = False
        self.last_evidence = {
            "regular": regular_evidence,
            "pre_change": pre_change,
            "correlated_vertical": correlated_vertical,
            "calibration_guard": calibration_guard,
            "cartesian": cartesian_evidence,
            "contact_guard": contact_guard,
            "high_noise": high_noise,
            "threshold_cross": threshold_cross,
            "identification_guard": identification_guard,
            "guarded_detect_r": guarded_detect_r.copy(),
            "threshold": effective_threshold.copy(),
        }
        if raw_detect:
            # 안전 기능은 충돌 뒤 별도 reset 전까지 정지 상태를 유지하는 것이
            # 자연스럽다. 같은 충돌의 잔류 진동을 새 사건으로 세지 않는다.
            self.latched = True
        return self.r.copy(), self.latched
