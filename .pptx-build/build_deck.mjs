import fs from "node:fs/promises";
import path from "node:path";
import { pathToFileURL } from "node:url";
import { Presentation, PresentationFile } from "@oai/artifact-tool";

const workspaceDir = "/home/jwj/Documents/ChatGPT/Lecture_ADV";
const SKILL_DIR = "/home/jwj/.codex/plugins/cache/openai-primary-runtime/presentations/26.1026.10807/skills/presentations";
const RUNTIME_PYTHON = "/home/jwj/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3";
const buildDir = path.join(workspaceDir, ".pptx-build");
const previewDir = path.join(buildDir, "previews");
const stagingDir = path.join(workspaceDir, ".codex-finalizer");
const outputDir = path.join(workspaceDir, "presentation-output");
const FINAL_PPTX = path.join(outputDir, "Doosan_M1013_충돌감지_개선_미지Payload가정_최종수정.pptx");

const W = 1280, H = 720;
const C = {
  navy: "#14213D", ink: "#243447", gray: "#66788A", pale: "#F4F6F8",
  white: "#FFFFFF", orange: "#E76F51", coral: "#F5B39F", teal: "#2A9D8F",
  tealPale: "#DCEFEA", blue: "#6E8399", bluePale: "#E7EDF2", red: "#D64550",
  sand: "#F2E8D5", line: "#D8E0E6"
};
const FONT = "Noto Sans CJK KR";

await fs.mkdir(buildDir, { recursive: true });
await fs.mkdir(previewDir, { recursive: true });
await fs.mkdir(outputDir, { recursive: true });

const presentation = Presentation.create({ slideSize: { width: W, height: H } });

function shape(slide, geometry, left, top, width, height, fill="none", lineFill="none", lineWidth=0, radius) {
  return slide.shapes.add({
    geometry,
    position: { left, top, width, height },
    fill,
    line: { style: "solid", fill: lineFill, width: lineWidth },
    ...(radius ? { borderRadius: radius } : {})
  });
}

function textBox(slide, text, left, top, width, height, opts={}) {
  const s = shape(slide, "textbox", left, top, width, height, opts.fill ?? "none", opts.lineFill ?? "none", opts.lineWidth ?? 0);
  s.text = text;
  s.text.style = {
    typeface: FONT,
    fontSize: opts.fontSize ?? 24,
    bold: opts.bold ?? false,
    color: opts.color ?? C.ink,
    alignment: opts.align ?? "left",
    verticalAlignment: opts.valign ?? "top",
    lineSpacing: opts.lineSpacing ?? 1.1,
  };
  s.text.insets = opts.insets ?? 0;
  s.text.autoFit = "shrinkText";
  return s;
}

function addHeader(slide, title, no, subtitle) {
  textBox(slide, title, 72, 42, 1050, 58, { fontSize: 43, bold: true, color: C.navy, valign: "middle" });
  if (subtitle) textBox(slide, subtitle, 74, 102, 1020, 28, { fontSize: 18, color: C.gray, valign: "middle" });
  shape(slide, "rect", 72, 132, 1136, 3, C.orange);
  textBox(slide, String(no).padStart(2, "0"), 1150, 49, 58, 26, { fontSize: 17, bold: true, color: C.orange, align: "right" });
}

function addFooter(slide, source) {
  if (source) textBox(slide, source, 72, 685, 1080, 18, { fontSize: 12, color: "#82909D" });
}

function addPill(slide, label, left, top, width, fill, color=C.white) {
  const p = shape(slide, "roundRect", left, top, width, 36, fill, "none", 0, "rounded-xl");
  p.text = label;
  p.text.style = { typeface: FONT, fontSize: 17, bold: true, color, alignment: "center", verticalAlignment: "middle" };
  p.text.insets = 0;
  return p;
}

function addMetric(slide, value, label, left, top, width, color=C.orange) {
  textBox(slide, value, left, top, width, 66, { fontSize: 47, bold: true, color, align: "center", valign: "middle" });
  textBox(slide, label, left, top + 68, width, 54, { fontSize: 19, color: C.ink, align: "center", valign: "top" });
}

function addBullet(slide, label, body, left, top, width, accent=C.orange) {
  shape(slide, "ellipse", left, top + 8, 14, 14, accent);
  textBox(slide, label, left + 28, top, width - 28, 28, { fontSize: 22, bold: true, color: C.navy });
  textBox(slide, body, left + 28, top + 32, width - 28, 62, { fontSize: 19, color: C.gray, lineSpacing: 1.15 });
}

async function imageBytes(imagePath) {
  const bytes = await fs.readFile(imagePath);
  return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
}

async function addImage(slide, imagePath, left, top, width, height, fit="contain", alt="") {
  slide.images.add({
    blob: await imageBytes(imagePath), contentType: "image/png", alt,
    fit, position: { left, top, width, height }
  });
}

function notes(slide, duration, script, sources=[]) {
  slide.speakerNotes.text = [
    `발표 시간: 약 ${duration}`,
    script,
    ...(sources.length ? ["근거: " + sources.join("; ")] : [])
  ];
  slide.speakerNotes.setVisible(true);
}

// 1. Cover
{
  const s = presentation.slides.add({ background: { fill: C.navy } });
  shape(s, "rect", 0, 0, 1280, 14, C.orange);
  textBox(s, "고급 매니퓰레이션 IC-PBL", 76, 68, 480, 28, { fontSize: 18, bold: true, color: C.coral });
  textBox(s, "하중 천이에서 발생하는\n충돌 오경보 줄이기", 76, 138, 760, 178, { fontSize: 58, bold: true, color: C.white, lineSpacing: 1.03 });
  textBox(s, "Doosan M1013 · MuJoCo · 추가 센서 없음", 80, 336, 650, 38, { fontSize: 24, color: "#D8E2EC" });
  shape(s, "rect", 80, 416, 1040, 2, "#53657A");
  textBox(s, "1차", 80, 462, 100, 30, { fontSize: 19, bold: true, color: "#AAB7C4" });
  textBox(s, "77.3%", 80, 494, 220, 72, { fontSize: 55, bold: true, color: C.white });
  textBox(s, "→", 310, 493, 80, 72, { fontSize: 54, bold: true, color: C.orange, align: "center" });
  textBox(s, "2차", 410, 462, 100, 30, { fontSize: 19, bold: true, color: C.coral });
  textBox(s, "94.1%", 410, 494, 250, 72, { fontSize: 55, bold: true, color: C.orange });
  textBox(s, "충돌 구분률 · payload 값 제공 조건", 80, 585, 530, 30, { fontSize: 20, color: "#B9C6D2" });
  addPill(s, "미지 payload 재검증 51.5%", 790, 500, 370, C.teal);
  notes(s, "30초", "안녕하세요. 이 발표에서는 정상 하중 전달의 충돌 오경보를 줄이는 과정을 설명합니다. 94.1퍼센트는 질량과 COM 설정값을 제공한 조건입니다. 마지막에는 두 값을 숨긴 재검증 결과 51.5퍼센트와 그 한계를 함께 보고합니다.", ["assignment-lab/PAIRED_1000_REPORT.md", "assignment-lab/UNKNOWN_PAYLOAD_REPORT.md"]);
}

// 2. Goal
{
  const s = presentation.slides.add({ background: { fill: C.white } });
  addHeader(s, "과제의 핵심 질문", 2, "정상 하중 전달은 지나가게 하고, 실제 충돌은 놓치지 않을 수 있는가?");
  textBox(s, "정상 동작", 105, 188, 230, 34, { fontSize: 23, bold: true, color: C.blue, align: "center" });
  shape(s, "roundRect", 90, 235, 260, 180, C.bluePale, "#C4D0DB", 1, "rounded-xl");
  textBox(s, "집기 · 운반 · 놓기", 110, 272, 220, 34, { fontSize: 25, bold: true, color: C.navy, align: "center" });
  textBox(s, "하중이 바뀌며\n큰 잔차가 잠깐 생김", 120, 325, 200, 58, { fontSize: 20, color: C.gray, align: "center" });
  shape(s, "rightArrow", 388, 280, 180, 86, C.orange, "none", 0);
  textBox(s, "검출기", 420, 300, 116, 34, { fontSize: 23, bold: true, color: C.white, align: "center" });
  textBox(s, "실제 충돌", 622, 188, 230, 34, { fontSize: 23, bold: true, color: C.red, align: "center" });
  shape(s, "roundRect", 608, 235, 260, 180, "#FCE7E9", "#EEC2C6", 1, "rounded-xl");
  textBox(s, "외부 물체와 부딪힘", 628, 272, 220, 34, { fontSize: 25, bold: true, color: C.red, align: "center" });
  textBox(s, "모델로 설명되지 않는\n힘이 잔차로 남음", 638, 325, 200, 58, { fontSize: 20, color: C.gray, align: "center" });
  shape(s, "rightArrow", 905, 280, 180, 86, C.teal, "none", 0);
  textBox(s, "판단", 937, 300, 116, 34, { fontSize: 23, bold: true, color: C.white, align: "center" });
  addMetric(s, "낮게", "정상 동작 오경보", 190, 480, 250, C.teal);
  addMetric(s, "높게", "실제 충돌 구분률", 515, 480, 250, C.orange);
  addMetric(s, "짧게", "검출 지연", 840, 480, 250, C.navy);
  notes(s, "50초", "목표는 모든 힘을 작게 만드는 것이 아닙니다. 정상적인 집기와 놓기에서 생기는 힘은 충돌로 보지 않고, 외부 충돌은 빠르게 구분하는 것입니다. 그래서 오경보, 충돌 구분률, 검출 지연을 함께 봅니다.", ["assignment-lab/REPORT.md"]);
}

// 3. Original detector
{
  const s = presentation.slides.add({ background: { fill: C.pale } });
  addHeader(s, "원본: 측정값과 모델의 차이로 충돌을 판단", 3, "센서 힘 하나를 바로 임계값과 비교하는 구조가 아니다");
  const xs = [78, 330, 582, 834];
  const labels = [
    ["센서 입력", "모터각 θ\n관절토크 τs"],
    ["동역학 모델", "중력·관성·마찰\n예상 토크 계산"],
    ["운동량 관측기", "예상과 실제 차이\n잔차 r 생성"],
    ["임계값 비교", "|rᵢ| > hᵢ이면\n충돌 경보"]
  ];
  labels.forEach((d,i)=>{
    shape(s, "roundRect", xs[i], 215, 210, 190, i===3?C.tealPale:C.white, i===3?C.teal:C.line, 2, "rounded-xl");
    textBox(s, String(i+1), xs[i]+18, 232, 30, 30, { fontSize: 18, bold: true, color: i===3?C.teal:C.orange, align: "center" });
    textBox(s, d[0], xs[i]+20, 276, 170, 34, { fontSize: 23, bold: true, color: C.navy, align: "center" });
    textBox(s, d[1], xs[i]+22, 326, 166, 55, { fontSize: 19, color: C.gray, align: "center" });
    if(i<3) shape(s, "rightArrow", xs[i]+215, 280, 34, 46, C.orange);
  });
  textBox(s, "원본 관절별 임계값", 160, 472, 250, 32, { fontSize: 23, bold: true, color: C.navy });
  addPill(s, "J1 10", 150, 526, 110, C.blue);
  addPill(s, "J2 18", 275, 526, 110, C.blue);
  addPill(s, "J3 8", 400, 526, 110, C.blue);
  addPill(s, "J4 3", 525, 526, 110, C.blue);
  addPill(s, "J5 3", 650, 526, 110, C.blue);
  addPill(s, "J6 2 N·m", 775, 526, 145, C.blue);
  textBox(s, "한 축이라도 넘으면 경보", 950, 526, 230, 36, { fontSize: 19, bold: true, color: C.red, valign: "middle" });
  notes(s, "55초", "원본은 로봇의 센서값을 그대로 충돌 판단에 쓰지 않습니다. 동역학 모델이 예상한 운동과 실제 센서에서 계산된 운동의 차이, 즉 잔차를 만듭니다. 이 잔차가 관절별 임계값 중 하나라도 넘으면 충돌로 판단합니다.", ["assignment-lab/collision_detector.py", "assignment-lab/REPORT.md"]);
}

// 4. Baseline failure
{
  const s = presentation.slides.add({ background: { fill: C.white } });
  addHeader(s, "원본에서 발견한 이상: 충돌이 없는데 집기·놓기에서 경보", 4, "잔차가 한 제어 주기라도 임계값을 넘으면 경보가 latch된다");

  addPill(s, "이상 현상", 82, 172, 150, C.red);
  addMetric(s, "57.5 N·m", "집기 순간 토크 계단", 80, 232, 220, C.red);
  addMetric(s, "73.8 N·m", "놓기 순간 토크 계단", 300, 232, 220, C.red);
  textBox(s, "충돌이 없어도 경보 7회", 105, 370, 390, 48, { fontSize: 29, bold: true, color: C.red, align: "center", valign: "middle" });

  shape(s, "rightArrow", 535, 278, 92, 68, C.orange);
  textBox(s, "왜?", 551, 294, 60, 30, { fontSize: 22, bold: true, color: C.white, align: "center" });

  addPill(s, "원인 분석", 668, 172, 150, C.navy);
  textBox(s, "모델이 생각한 하중", 666, 232, 230, 28, { fontSize: 21, bold: true, color: C.navy });
  shape(s, "rect", 670, 310, 125, 6, C.blue);
  shape(s, "rect", 790, 270, 6, 46, C.blue);
  shape(s, "rect", 792, 270, 330, 6, C.blue);
  textBox(s, "grip 신호 순간 0 → 100%", 820, 292, 300, 28, { fontSize: 18, color: C.blue, align: "center" });

  textBox(s, "실제 전달되는 하중", 666, 350, 230, 28, { fontSize: 21, bold: true, color: C.navy });
  shape(s, "rect", 670, 430, 125, 6, C.orange);
  for(let i=0;i<15;i++) shape(s, "rect", 792+i*21, 430-i*3, 23, 6, C.orange);
  textBox(s, "접촉·흡착 과정에서 점진적으로 전달", 790, 458, 330, 28, { fontSize: 18, color: C.orange, align: "center" });

  textBox(s, "추가 문제: 집기·운반·놓기에 같은 판단 방식을 적용", 690, 510, 420, 34, { fontSize: 20, bold: true, color: C.red, align: "center" });
  textBox(s, "판단: 하중 천이 모델과 상태 구분이 실제 동작을 충분히 설명하지 못했다", 120, 596, 1040, 46, { fontSize: 27, bold: true, color: C.navy, align: "center", valign: "middle" });
  notes(s, "55초", "원본에서 먼저 발견한 이상은 실제 충돌이 없는데도 집기와 놓기에서 경보가 발생한 것입니다. 집기와 놓기 순간에는 각각 57.5와 73.8 뉴턴미터의 큰 토크 계단이 생겼습니다. 원인은 모델은 grip 신호 순간에 하중을 0에서 100퍼센트로 바꾸지만 실제 하중은 접촉과 흡착 과정에서 점진적으로 전달된다는 차이였습니다. 또한 모든 동작 상태에 같은 판단 방식을 사용했습니다.", ["assignment-lab/REPORT.md", "assignment-lab/results/transition_comparison.png"]);
}

// 5. Phase 1
{
  const s = presentation.slides.add({ background: { fill: C.pale } });
  addHeader(s, "1차 해결: 발견한 두 원인을 각각 수정", 5, "제어 측면에서는 피크를 줄이고, 검출 측면에서는 상태별 판단을 적용");

  const rows = [
    {
      y: 180, color: C.orange,
      problem: "문제 ①  하중 전달 피크",
      detail: "모델은 순간 전환\n실제 하중은 점진적 전달",
      solution: "25 ms 연속 하중 스케줄\n+ 놓기 전 1.25 mm 호버",
      effect: "집기·놓기 토크와 작업대 충격 감소"
    },
    {
      y: 380, color: C.teal,
      problem: "문제 ②  모든 상태에서 같은 충돌 판단",
      detail: "정상 하중 천이와\n외부 충돌을 같은 방식으로 비교",
      solution: "집기·운반·놓기 상태별 혼합 검출기",
      effect: "천이 방향은 보상하고 다른 방향 충돌은 감시"
    }
  ];
  rows.forEach((r)=>{
    shape(s, "roundRect", 78, r.y, 330, 150, C.white, r.color, 2, "rounded-xl");
    textBox(s, r.problem, 98, r.y+20, 290, 34, { fontSize: 22, bold: true, color: C.navy, align: "center" });
    textBox(s, r.detail, 104, r.y+70, 278, 58, { fontSize: 18, color: C.gray, align: "center", lineSpacing: 1.18 });
    shape(s, "rightArrow", 438, r.y+49, 94, 58, r.color);
    textBox(s, "해결", 456, r.y+63, 56, 28, { fontSize: 19, bold: true, color: C.white, align: "center" });
    shape(s, "roundRect", 560, r.y, 640, 150, C.white, r.color, 2, "rounded-xl");
    textBox(s, r.solution, 590, r.y+24, 580, 52, { fontSize: 24, bold: true, color: r.color, align: "center", valign: "middle" });
    textBox(s, r.effect, 600, r.y+96, 560, 34, { fontSize: 19, color: C.gray, align: "center" });
  });

  addPill(s, "6초 사이클 유지", 300, 596, 260, C.orange);
  addPill(s, "1 kHz 제어 유지", 610, 596, 260, C.teal);
  addPill(s, "추가 센서 없음", 920, 596, 230, C.blue);
  notes(s, "1분 5초", "그래서 1차 개선은 두 문제에 대응하도록 구성했습니다. 첫 번째로 순간적인 하중 전환을 25밀리초 연속 하중 스케줄로 바꾸고 놓기 전에 1.25밀리미터 호버해 피크를 줄였습니다. 두 번째로 집기, 운반, 놓기 상태를 구분해 정상 하중 전달 성분은 보상하고 다른 방향의 충돌은 계속 감시했습니다. 전체 6초 사이클과 1킬로헤르츠 제어 주기는 유지했습니다.", ["assignment-lab/REPORT.md"]);
}

// 6. Phase 1 result and limit
{
  const s = presentation.slides.add({ background: { fill: C.white } });
  addHeader(s, "1차 확대시험에서 다시 발견한 이상: 환경이 바뀌면 성능이 흔들림", 6, "표준 8 kg에서는 해결됐지만 물체·센서·접촉 조건을 바꾸자 새 문제가 나타남");

  shape(s, "roundRect", 78, 174, 360, 170, C.tealPale, C.teal, 2, "rounded-xl");
  textBox(s, "표준 8 kg에서는 정상", 102, 196, 312, 36, { fontSize: 26, bold: true, color: C.teal, align: "center" });
  textBox(s, "집기 계단 90.3% 감소\n놓기 계단 93.1% 감소\n오경보 7 → 0", 108, 252, 300, 76, { fontSize: 21, bold: true, color: C.navy, align: "center", lineSpacing: 1.2 });

  shape(s, "rightArrow", 466, 224, 82, 64, C.orange);
  textBox(s, "조건\n확대", 484, 234, 46, 42, { fontSize: 18, bold: true, color: C.white, align: "center" });

  shape(s, "roundRect", 576, 174, 624, 170, "#FCE7E9", C.red, 2, "rounded-xl");
  textBox(s, "다양한 환경에서 다시 이상 발생", 604, 196, 568, 36, { fontSize: 26, bold: true, color: C.red, align: "center" });
  addMetric(s, "4 / 17", "무충돌 환경 오경보", 620, 246, 230, C.red);
  addMetric(s, "47 / 51", "15 N 충돌 구분", 900, 246, 230, C.red);

  textBox(s, "원인 분석", 80, 392, 180, 34, { fontSize: 25, bold: true, color: C.navy });
  const causes = [
    ["센서 잡음 변화", "고정 임계값은 잡음이 커지면\n순간 스파이크를 충돌로 오인"],
    ["접촉 시점·반력 변화", "고정 천이 구간과 실제 접촉이 어긋나\n정상 힘과 충돌이 겹침"],
    ["자세·형상 변화", "외력이 여러 관절에 작게 분산되어\n각 관절 임계값을 못 넘김"]
  ];
  causes.forEach((d,i)=>{
    const x=78+i*382;
    shape(s, "roundRect", x, 438, 350, 142, C.pale, C.line, 1, "rounded-xl");
    textBox(s, d[0], x+22, 458, 306, 32, { fontSize: 22, bold: true, color: i===0?C.orange:(i===1?C.red:C.blue), align: "center" });
    textBox(s, d[1], x+24, 508, 302, 56, { fontSize: 18, color: C.gray, align: "center", lineSpacing: 1.16 });
  });
  textBox(s, "판단: 1차의 고정 임계값과 고정 천이 처리만으로는 다양한 환경을 감당하기 어렵다", 100, 616, 1080, 36, { fontSize: 23, bold: true, color: C.red, align: "center" });
  notes(s, "55초", "1차는 표준 8킬로그램 조건에서 오경보를 없앴습니다. 하지만 물체 형상, 마찰, 센서 잡음, 접촉 특성을 바꾸자 17개 환경 중 4개에서 오경보가 나고 15뉴턴 충돌 51개 중 4개를 놓쳤습니다. 원인을 다시 분석하니 센서 잡음 변화, 접촉 시점과 반력 변화, 그리고 자세에 따라 외력이 여러 관절로 분산되는 문제가 있었습니다. 즉 고정 임계값과 고정 천이 처리만으로는 환경 변화에 약했습니다.", ["assignment-lab/REPORT.md", "assignment-lab/PHASE2_REPORT.md"]);
}

// 7. Phase 2
{
  const s = presentation.slides.add({ background: { fill: C.pale } });
  addHeader(s, "2차 해결: 1차에서 드러난 세 원인에 각각 대응", 7, "제어기와 1.25 mm 호버는 유지하고 robust_detector_v2만 적용");

  const mappings = [
    ["센서 잡음·순간 스파이크", "축별 잡음 적응 임계값\n+ 고잡음에서 20 ms 중 7회 지속성 확인", C.orange],
    ["외력이 여러 관절에 분산", "Cartesian 외력 벡터를 복원하고\n20 ms 벡터 평균으로 작은 성분을 결합", C.blue],
    ["놓기 반력과 충돌이 겹침", "예상 수직 작업대 반력만 투영 제거\n+ 천이 보호구간 확장", C.teal]
  ];
  mappings.forEach((d,i)=>{
    const y=176+i*142;
    shape(s, "roundRect", 80, y, 360, 110, C.white, d[2], 2, "rounded-xl");
    textBox(s, d[0], 105, y+30, 310, 46, { fontSize: 23, bold: true, color: C.navy, align: "center", valign: "middle" });
    shape(s, "rightArrow", 470, y+28, 98, 54, d[2]);
    textBox(s, "해결", 489, y+40, 58, 28, { fontSize: 18, bold: true, color: C.white, align: "center" });
    shape(s, "roundRect", 598, y, 600, 110, C.white, d[2], 2, "rounded-xl");
    textBox(s, d[1], 630, y+24, 536, 62, { fontSize: 20, bold: true, color: d[2], align: "center", valign: "middle", lineSpacing: 1.15 });
  });

  textBox(s, "비교 조건", 88, 618, 130, 30, { fontSize: 21, bold: true, color: C.navy });
  addPill(s, "같은 hover_servo", 230, 614, 230, C.blue);
  addPill(s, "같은 1.25 mm 호버", 490, 614, 260, C.blue);
  addPill(s, "추가 센서 없음", 780, 614, 210, C.blue);
  addPill(s, "검출기 효과만 측정", 1020, 614, 190, C.orange);
  notes(s, "1분 5초", "2차 개선은 방금 확인한 세 원인에 각각 대응합니다. 센서 잡음과 순간 스파이크에는 축별 적응 임계값과 지속성 조건을 적용했습니다. 여러 관절로 분산된 외력은 카테시안 벡터로 복원해 평균했습니다. 놓기에서는 예상되는 수직 작업대 반력만 제거하고 천이 보호구간을 넓혔습니다. 제어기와 호버는 그대로 두어 검출기 변경의 효과만 측정했습니다.", ["assignment-lab/PHASE2_REPORT.md", "assignment-lab/robust_detector_v2.py"]);
}

// 8. Experiment design
{
  const s = presentation.slides.add({ background: { fill: C.white } });
  addHeader(s, "검증 설계: 동일 1,000조건을 1차와 2차에 쌍으로 적용", 8, "검출기마다 충돌 실행 + 같은 환경의 무충돌 실행 = 총 4,000 에피소드");
  const rows = [
    ["동작 구간", "초기·접근 150 / 집기 200 / 운반 250 / 놓기 300 / 복귀 100"],
    ["충돌력", "15–20 / 20–30 / 30–45 / 45–60 N, 각 250조건"],
    ["충돌 방향", "무작위 3D 방향"],
    ["불확실성", "질량·COM 값은 실행마다 제공; 형상·관성·마찰·접촉·흡착·센서 조건은 비공개"],
    ["성공 판정", "충돌 뒤 검출 + 충돌 전 경보 없음 + 기준 실행 경보와 겹치지 않음"]
  ];
  rows.forEach((r,i)=>{
    const y=174+i*82;
    shape(s, "rect", 82, y, 220, 64, i%2?C.bluePale:C.pale, "none", 0);
    shape(s, "rect", 302, y, 875, 64, i%2?"#F7F9FA":C.white, C.line, 1);
    textBox(s, r[0], 100, y, 184, 64, { fontSize: 21, bold: true, color: C.navy, align: "center", valign: "middle" });
    textBox(s, r[1], 326, y, 830, 64, { fontSize: 19, color: C.ink, valign: "middle" });
  });
  addPill(s, "독립 시드 10개", 162, 614, 230, C.blue);
  addPill(s, "최종 고정 테스트 세트", 480, 614, 300, C.orange);
  addPill(s, "payload 값 제공 조건", 850, 614, 270, C.teal);
  notes(s, "55초", "결과를 공정하게 비교하기 위해 1차와 2차에 완전히 같은 1,000개 조건을 넣었습니다. 각 조건에서 충돌 실행과 무충돌 기준 실행을 모두 했으므로 총 4,000회입니다. 다만 이 비교에서는 질량과 COM 설정값을 제어기와 검출기에 제공했습니다. 값은 변했지만 비공개 조건은 아닙니다.", ["assignment-lab/PAIRED_1000_REPORT.md", "assignment-lab/UNKNOWN_PAYLOAD_REPORT.md"]);
}

// 9. Main results
{
  const s = presentation.slides.add({ background: { fill: C.pale } });
  addHeader(s, "결과: 충돌 구분률 +16.8%p, 무충돌 오경보 93.0% 감소", 9, "질량·COM 설정값 제공 조건 · 검출 지연 중앙값 29.9 → 30.3 ms");
  await addImage(s, path.join(workspaceDir, "assignment-lab/results/phase2/paired_1000/paired_comparison_overview.png"), 52, 150, 850, 510, "contain", "동일 1,000조건에서 1차와 2차 검출기 비교");
  shape(s, "roundRect", 930, 162, 290, 148, C.white, C.orange, 2, "rounded-xl");
  textBox(s, "충돌 구분", 950, 180, 250, 30, { fontSize: 22, bold: true, color: C.navy, align: "center" });
  textBox(s, "77.3 → 94.1%", 944, 226, 262, 50, { fontSize: 34, bold: true, color: C.orange, align: "center" });
  shape(s, "roundRect", 930, 330, 290, 148, C.white, C.teal, 2, "rounded-xl");
  textBox(s, "무충돌 오경보", 950, 348, 250, 30, { fontSize: 22, bold: true, color: C.navy, align: "center" });
  textBox(s, "341 → 24건", 944, 394, 262, 50, { fontSize: 34, bold: true, color: C.teal, align: "center" });
  shape(s, "roundRect", 930, 498, 290, 148, C.white, C.blue, 2, "rounded-xl");
  textBox(s, "전체 실패", 950, 516, 250, 30, { fontSize: 22, bold: true, color: C.navy, align: "center" });
  textBox(s, "227 → 59건", 944, 562, 262, 50, { fontSize: 34, bold: true, color: C.blue, align: "center" });
  notes(s, "1분 10초", "질량과 COM 설정값을 제공한 조건에서는 충돌 구분률이 77.3퍼센트에서 94.1퍼센트로 올랐습니다. 이 수치를 미지 payload 성능으로 표현하면 안 됩니다. 값을 숨긴 별도 재검증은 다음 슬라이드에서 보여 드립니다.", ["assignment-lab/PAIRED_1000_REPORT.md", "assignment-lab/UNKNOWN_PAYLOAD_REPORT.md"]);
}

// 10. Failures and conclusion
{
  const s = presentation.slides.add({ background: { fill: C.white } });
  addHeader(s, "남은 한계와 결론", 10, "2차도 100%는 아니며, 놓기 천이의 약한 충돌이 가장 어렵다");
  await addImage(s, path.join(workspaceDir, "assignment-lab/results/phase2/paired_1000/v2_failure_map.png"), 54, 160, 760, 385, "contain", "2차 검출기의 실패가 놓기 천이에 집중된 산점도");
  shape(s, "roundRect", 846, 170, 350, 188, "#FCE7E9", C.red, 2, "rounded-xl");
  textBox(s, "2차 실패 59건", 874, 194, 294, 38, { fontSize: 29, bold: true, color: C.red, align: "center" });
  textBox(s, "완전 미검출 41\n충돌 전 경보 18\n미검출 중 놓기 천이 30", 884, 252, 274, 92, { fontSize: 21, color: C.navy, align: "center", lineSpacing: 1.22 });
  shape(s, "roundRect", 846, 382, 350, 164, C.tealPale, C.teal, 2, "rounded-xl");
  textBox(s, "질량·COM 비공개 재검증", 864, 400, 314, 36, { fontSize: 23, bold: true, color: C.teal, align: "center" });
  textBox(s, "515 / 1000 (51.5%)\n무충돌 조건 경보 9.0%\n집기 18.5% · 놓기 31.3%", 874, 448, 294, 90, { fontSize: 20, bold: true, color: C.navy, align: "center", lineSpacing: 1.18 });
  textBox(s, "결론: 현재 방식은 미지 payload 강건성을 확보하지 못했다", 108, 596, 1060, 48, { fontSize: 25, bold: true, color: C.red, align: "center", valign: "middle" });
  notes(s, "50초", "기존 2차의 94.1퍼센트는 질량과 COM 설정값을 제공한 조건입니다. 두 값을 완전히 숨기고 1,000회 다시 검사하자 성공률은 51.5퍼센트였습니다. 질량 추정 오차는 작았지만 집기와 놓기에서 정상 하중과 충돌을 분리하지 못했습니다. 따라서 미지 payload까지 해결했다고 주장하지 않고, 이것을 남은 핵심 한계로 보고합니다.", ["assignment-lab/PAIRED_1000_REPORT.md", "assignment-lab/UNKNOWN_PAYLOAD_REPORT.md"]);
}

// 11. Future work
{
  const s = presentation.slides.add({ background: { fill: C.pale } });
  addHeader(s, "Future Work: 실제 M1013에서 강건성을 완성", 11, "놓기 천이 개선 → 실기 재교정 → 단계적 검증");

  const steps = [
    ["1", "놓기 상태 세분화", "접근 · 최초 접촉 · 하중 이탈 · 복귀를 분리하고\n속도와 잔차 변화율을 함께 사용", C.orange],
    ["2", "Payload 온라인 식별", "여러 자세의 짧은 식별 동작으로 질량·COM·관성을\n먼저 추정한 뒤 충돌 감시 시작", C.teal],
    ["3", "단계적 충돌 시험", "15 / 20 / 40 N, 6방향, 여러 물체로 시험하고\n시뮬레이션과 실기의 차이를 보정", C.blue]
  ];
  steps.forEach((d,i)=>{
    const x=74+i*397;
    shape(s, "roundRect", x, 185, 350, 278, C.white, d[3], 2, "rounded-xl");
    shape(s, "ellipse", x+138, 158, 74, 74, d[3]);
    textBox(s, d[0], x+138, 158, 74, 74, { fontSize: 31, bold: true, color: C.white, align: "center", valign: "middle" });
    textBox(s, d[1], x+28, 250, 294, 44, { fontSize: 27, bold: true, color: C.navy, align: "center", valign: "middle" });
    textBox(s, d[2], x+30, 330, 290, 88, { fontSize: 19, color: C.gray, align: "center", valign: "middle", lineSpacing: 1.18 });
    if(i<2) shape(s, "rightArrow", x+355, 294, 34, 48, C.orange);
  });

  textBox(s, "목표 지표", 86, 516, 150, 34, { fontSize: 24, bold: true, color: C.navy });
  addPill(s, "충돌 구분률 ≥ 95%", 80, 568, 260, C.orange);
  addPill(s, "무충돌 오경보 < 1%", 370, 568, 270, C.teal);
  addPill(s, "놓기 15–20 N 구분 ≥ 95%", 670, 568, 330, C.blue);
  addPill(s, "p95 지연 < 80 ms", 1030, 568, 185, C.navy);
  textBox(s, "외부 F/T 센서는 알고리즘 입력이 아니라 실험 정답값(ground truth) 측정에만 활용", 160, 636, 960, 28, { fontSize: 19, color: C.gray, align: "center" });
  notes(s, "50초", "향후 연구는 세 단계입니다. 먼저 놓기 동작을 접근, 최초 접촉, 하중 이탈, 복귀로 더 세분화해 정상 접촉과 충돌을 구분합니다. 다음으로 실제 M1013의 무충돌 반복 데이터를 이용해 잡음과 임계값을 재교정합니다. 마지막으로 여러 물체와 충돌 방향에서 단계적으로 시험합니다. 목표는 구분률 95퍼센트 이상, 오경보 1퍼센트 미만, 놓기 약충돌 구분률 95퍼센트 이상입니다. 외부 힘센서는 검출 입력이 아니라 실험의 정답값 측정에만 사용합니다.", ["assignment-lab/PAIRED_1000_REPORT.md", "assignment-lab/PHASE2_REPORT.md"]);
}

// Export previews and montage.
for (const [index, slide] of presentation.slides.items.entries()) {
  const stem = `slide-${String(index + 1).padStart(2, "0")}`;
  const png = await presentation.export({ slide, format: "png", scale: 1 });
  await fs.writeFile(path.join(previewDir, `${stem}.png`), new Uint8Array(await png.arrayBuffer()));
}
const montage = await presentation.export({ format: "webp", montage: true });
await fs.writeFile(path.join(buildDir, "deck-montage.webp"), new Uint8Array(await montage.arrayBuffer()));

const { finalizePresentation } = await import(pathToFileURL(path.join(SKILL_DIR, "container_tools/artifact_tool_utils.mjs")).href);
await fs.mkdir(stagingDir, { recursive: true });
const candidatePath = path.join(stagingDir, "Doosan_M1013_candidate.pptx");
await (await PresentationFile.exportPptx(presentation)).save(candidatePath);

const requirements = {
  explicitTotalSlideCount: 11,
  requiredNativeTableOwnerSlides: [],
  requiredNativeChartOwnerSlides: []
};
const result = await finalizePresentation({
  ...requirements,
  workspaceDir,
  candidatePath,
  finalPath: FINAL_PPTX,
  pythonExecutable: RUNTIME_PYTHON,
  integrityValidatorPath: path.join(SKILL_DIR, "container_tools/inspect_presentation_package_integrity.py"),
  layoutValidatorPath: path.join(SKILL_DIR, "container_tools/inspect_presentation_layout_geometry.py"),
  layoutArgs: ["--expected-slide-size-emu", "12192000,6858000", "--validate-bullet-geometry", "--validate-heading-fit"],
  requiredNativeTableOwnerSlides: [],
  fontPolicy: { basis: "design", families: [FONT] },
  verifyArtifactToolImport: true,
  receiptPath: path.join(stagingDir, `${path.basename(FINAL_PPTX)}.validation.json`)
});
console.log(JSON.stringify({ FINAL_PPTX, validation: result, previewDir }, null, 2));
