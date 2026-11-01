"""M1013 MuJoCo 모델 생성.

공개 모델 파일을 직접 읽어 메모리로 전달한다 (Windows 한글 경로에서도 동작).
공식 모델의 위치 구동기·큰 관절 감쇠는 시각화용이라 토크 구동으로 바꾸고, 그리퍼·박스·TCP 를 붙인다.

elastic=True (가상 로봇): 탄성 관절(J1~J3)의 링크마다 같은 축 힌지 2개를 둔다.
  motor_k : 모터 각도 θ   — armature = 모터 관성 B, damping = 점성 마찰
  joint_k : 처짐 φ = q − θ — stiffness = 감속기 강성 K, damping = 감쇠 D
  강체 관절(J4~J6)은 joint_k 하나에 armature = B, damping = 점성 마찰.
elastic=False (명목 모델): 공개 모델 그대로의 강체 6관절.
"""
import os
import numpy as np
import mujoco
from . import config as C

MODEL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model", "m1013")


def build(elastic=False, spec_only=False):
    assets = {}
    for f in os.listdir(os.path.join(MODEL_DIR, "assets")):
        with open(os.path.join(MODEL_DIR, "assets", f), "rb") as fh:
            assets["m1013/assets/" + f] = fh.read()
    with open(os.path.join(MODEL_DIR, "m1013.xml"), encoding="utf-8") as fh:
        spec = mujoco.MjSpec.from_string(fh.read(), assets=assets)
    spec.compiler.degree = False          # 공식 scene.xml 의 angle="radian" (m1013.xml 단독 로드 시 기본값이 degree)
    spec.option.timestep = C.DT
    spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.05], pos=[0, 0, C.FLOOR_Z],
                            rgba=[.22, .27, .32, 1], contype=0, conaffinity=0)
    spec.worldbody.add_geom(name="pedestal", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.12, -C.FLOOR_Z / 2],
                            pos=[0, 0, C.FLOOR_Z / 2], rgba=[.35, .38, .42, 1], contype=0, conaffinity=0)
    spec.worldbody.add_light(pos=[0, -1, 2.5], dir=[0, 0.4, -1])
    for g in spec.geoms:
        g.contype, g.conaffinity = 0, 0
    for j in spec.joints:
        j.damping = [0., 0., 0.]
        j.frictionloss = 0.
    if elastic:
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_RK4
        for k in range(6):
            j = spec.joint(f"joint_{k + 1}")
            j.armature, j.damping = C.B[k], [C.VISCOUS[k], 0., 0.]
            if C.ELASTIC[k]:
                j.name = f"motor_{k + 1}"
                dj = j.parent.add_joint(name=f"joint_{k + 1}", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
                dj.stiffness, dj.damping = [C.K[k], 0., 0.], [C.D[k], 0., 0.]
    for a in list(spec.actuators):
        spec.delete(a)
    l6 = spec.body("link_6")
    tool = l6.add_body(name="tool")
    tool.add_geom(name="tool_geom", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.04, 0.08], pos=C.TOOL_COM, density=0,
                  rgba=[.3, .3, .3, 1], contype=0, conaffinity=0)
    tool.mass, tool.ipos, tool.inertia, tool.explicitinertial = C.TOOL_MASS, C.TOOL_COM, C.TOOL_INERTIA, True
    tool.add_site(name="tcp", pos=C.TCP, size=[0.01])
    box = l6.add_body(name="box", pos=C.BOX_POS)
    box.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[s / 2 for s in C.BOX_SIZE], density=0,
                 rgba=[.8, .55, .25, 0], contype=0, conaffinity=0)
    box.mass, box.inertia, box.explicitinertial = 1e-6, [1e-9] * 3, True
    if spec_only:
        return spec
    m = spec.compile()
    return m, mujoco.MjData(m)


def set_box(m, mass, com=(0., 0., 0.)):
    """박스 질량·관성·COM 설정. mass <= 0 이면 빈 손."""
    b = m.body("box").id
    g = m.body_geomadr[b]
    if mass <= 0:
        m.body_mass[b], m.body_inertia[b], m.geom_rgba[g, 3] = 1e-6, 1e-9, 0.0
        return
    a, w, h = C.BOX_SIZE
    m.body_mass[b] = mass
    m.body_inertia[b] = mass / 12 * np.array([w**2 + h**2, a**2 + h**2, a**2 + w**2])
    m.body_ipos[b] = np.asarray(com, float)
    m.geom_rgba[g, 3] = 0.9
