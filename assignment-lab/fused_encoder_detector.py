"""분석 전용: 공식 JTS 감지기와 두 엔코더 감지기의 병렬 안전 결합."""
import dual_encoder_detector
import hybrid_detector


class Detector:
    USES_LINK_ENCODER = True
    SOURCE = "jts+dual_encoder"

    def reset(self):
        self.jts = hybrid_detector.Detector()
        self.dual = dual_encoder_detector.Detector()
        for child in (self.jts, self.dual):
            child.nom = self.nom
            child.nom_box = self.nom_box
            child.ctrl = self.ctrl
            child.box_mass = self.box_mass
            child.box_com = self.box_com
            child.reset()

    def update(self, t, th, thd, tau_m, tau_s, grip):
        self.dual.q_link = self.q_link
        r_jts, hit_jts = self.jts.update(t, th, thd, tau_m, tau_s, grip)
        _, hit_dual = self.dual.update(t, th, thd, tau_m, tau_s, grip)
        return r_jts, hit_jts or hit_dual
