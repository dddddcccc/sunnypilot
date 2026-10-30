import glob, os, sys
import zstandard
from openpilot.cereal import log

ROOT = '/data/media/0/realdata'
routes = sorted({d.rsplit('--', 1)[0] for d in glob.glob(ROOT + '/*--*--*')}, key=lambda r: os.path.getmtime(r + '--0') if os.path.exists(r + '--0') else 0, reverse=True)
idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
print('最近幾趟 (0=最新):', ', '.join(f'{i}={os.path.basename(r)}' for i, r in enumerate(routes[:4])))
route = routes[idx] if idx < len(routes) else None
segs = sorted((d for d in glob.glob(str(route) + '--*') if glob.glob(d + '/rlog*')), key=lambda d: int(d.rsplit('--', 1)[1]))
if not segs:
  sys.exit('找不到行車紀錄')
print('route:', os.path.basename(str(route)), '| 段數:', len(segs))
import subprocess
def _git(*a):
  try:
    return subprocess.check_output(['git', *a], cwd='/data/openpilot', text=True, stderr=subprocess.DEVNULL).strip()
  except Exception:
    return '?'
print('C4 目前版本: sunnypilot', _git('log', '-1', '--format=%h'), '| opendbc', _git('-C', 'opendbc_repo', 'log', '-1', '--format=%h'))
try:
  import json
  from panda import Panda
  built = Panda.get_signature_from_firmware('/data/openpilot/panda/board/obj/panda_h7.bin.signed').hex()[:16]
  seen = [str(json.loads(l).get('msg$s', '')) for fn in sorted(glob.glob('/data/log/swaglog.*'))
          for l in open(fn, errors='ignore') if '"pandad"' in l and 'connected, version' in l]
  print('panda 韌體: 編譯出來的', built, '| 最近一次開機 panda 回報:', seen[-1].split('signature ')[-1] if seen else '?')
except Exception as ex:
  print('panda 韌體: 無法讀取 (', ex, ')')

t0 = None
last = {}
lines = []
sent = {}
angle = {'req': 0, 'n': 0, 'lo': 0.0, 'hi': 0.0}
tx_blocked = rx_invalid = None
cur = {}
seg_stats = {'total': 0, 'partial': [], 'skipped': []}
t_last = [0.0]
alerts = []

# (address, bus): [(label, start bit, width)]  -- bit positions from opendbc's Subaru global DBC
WATCH = {
  (0x119, 0): [('EPS Steer_Error_1', 12, 1), ('EPS Steer_Error_2', 28, 1), ('EPS Steer_Warning', 29, 1)],
  (0x118, 0): [('EPS Steering_Disabled', 63, 1)],
  (0x221, 1): [('EyeSight ES_Distance Cruise_Fault', 15, 1), ('EyeSight ES_Distance Soft_Disable', 34, 1)],
  (0x222, 1): [('EyeSight ES_Status Cruise_Fault', 15, 1), ('EyeSight ES_Status Cruise_Activated', 29, 1)],
  (0x220, 1): [('EyeSight ES_Brake AEB_Status', 32, 4), ('EyeSight ES_Brake Cruise_Brake_Fault', 37, 1),
               ('EyeSight ES_Brake Cruise_Brake_Active', 38, 1)],
  (0x321, 2): [('攝影機原始 ES_DashStatus Cruise_Fault', 48, 1), ('攝影機原始 ES_DashStatus Cruise_On', 49, 1),
               ('攝影機原始 ES_DashStatus LKAS_State_Msg', 20, 3), ('攝影機原始 ES_DashStatus Soft_Disable', 24, 1)],
  (0x322, 2): [('攝影機原始 ES_LKAS_State LKAS_Alert', 32, 5), ('攝影機原始 ES_LKAS_State LKAS_ACTIVE', 17, 1),
               ('攝影機原始 LKAS按鈕狀態(LKAS_Dash_State)', 18, 2)],
  (0x124, 2): [('攝影機原始 角度請求', 12, 1)],
}
# what the C4 itself sends to the car in place of the camera's blocked messages
SENT_WATCH = {
  0x321: [('C4送給車子 ES_DashStatus Cruise_Fault', 48, 1), ('C4送給車子 ES_DashStatus LKAS_State_Msg', 20, 3), ('C4送給車子 ES_DashStatus Cruise_On', 49, 1)],
  0x322: [('C4送給車子 ES_LKAS_State LKAS_Alert', 32, 5), ('C4送給車子 ES_LKAS_State LKAS_ACTIVE', 17, 1)],
}

def to_signed(v, bits):
  return v - (1 << bits) if v >= 1 << (bits - 1) else v

faults = []
safety_ready_t = None

def is_fault(key, val):
  if key in ('EPS Steer_Error_1', 'EPS Steer_Error_2', 'EyeSight ES_Distance Cruise_Fault', 'EyeSight ES_Status Cruise_Fault', 'EyeSight ES_Brake Cruise_Brake_Fault',
             '攝影機原始 ES_DashStatus Cruise_Fault', 'EPS Steering_Disabled'):
    return val == 1
  if key in ('EPS永久故障', 'EPS暫時故障'):
    return val is True
  if key == '畫面警示':
    return any(k in str(val) for k in ('TAKE CONTROL', 'Fault', 'Unavailable', 'Malfunction'))
  if key.startswith('被safety擋下'):
    return safety_ready_t is not None and t_now[0] > safety_ready_t + 2
  if key == 'panda故障':
    return safety_ready_t is not None and str(val) != 'none'
  return False

t_now = [0.0]

def note(t, key, val):
  if last.get(key) != val:
    last[key] = val
    lines.append((t, f'{t:8.2f}s  {key}: {val}', key, val))
    t_now[0] = t
    if is_fault(key, val):
      faults.append((t, key, val, dict(cur)))

for s in segs:
  fn = sorted(glob.glob(s + '/rlog*'))[0]
  dat = open(fn, 'rb').read()
  seg_stats['total'] += 1
  if dat[:4] == b'\x28\xb5\x2f\xfd':
    try:
      dat = zstandard.ZstdDecompressor().stream_reader(dat).read()
    except zstandard.ZstdError:
      try:  # segment still being written when power was cut: decode as much as exists
        dat = zstandard.ZstdDecompressor().decompressobj().decompress(dat)
        seg_stats['partial'].append(os.path.basename(s))
      except Exception:
        seg_stats['skipped'].append(os.path.basename(s))
        continue
  it = log.Event.read_multiple_bytes(dat)
  while True:
    try:
      e = next(it)
      w = e.which()
    except StopIteration:
      break
    except Exception:
      break
    if t0 is None:
      t0 = e.logMonoTime
    t = (e.logMonoTime - t0) / 1e9
    t_last[0] = max(t_last[0], t)
    try:
      if w == 'carParams':
        note(t, '車型', f'{e.carParams.carFingerprint} dashcam={e.carParams.dashcamOnly}')
      elif w == 'carState':
        cs = e.carState
        note(t, 'ACC主開關', cs.cruiseState.available)
        note(t, 'ACC啟用', cs.cruiseState.enabled)
        note(t, 'EPS暫時故障', cs.steerFaultTemporary)
        note(t, 'EPS永久故障', cs.steerFaultPermanent)
        note(t, '駕駛踩煞車', cs.brakePressed)
        cur['angle'], cur['v'], cur['torque'], cur['pressed'] = cs.steeringAngleDeg, cs.vEgo, cs.steeringTorque, cs.steeringPressed
        cur['acc'], cur['brake'], cur['gas'] = cs.cruiseState.enabled, cs.brakePressed, cs.gasPressed
        for be in cs.buttonEvents:
          if str(be.type) == 'lkas':
            note(t, 'sunnypilot收到LKAS按鈕', 'pressed' if be.pressed else 'released')
      elif w == 'selfdriveStateSP':
        m = e.selfdriveStateSP.mads
        note(t, 'sunnypilot MADS狀態', f'{m.state} enabled={m.enabled} active={m.active}')
      elif w == 'selfdriveState':
        ss = e.selfdriveState
        note(t, 'sunnypilot啟用', ss.enabled)
        note(t, 'sunnypilot控制中', ss.active)
        txt = f'{ss.alertText1} / {ss.alertText2}'.strip(' /')
        if txt and last.get('畫面警示') != txt:
          alerts.append((t, txt))
        note(t, '畫面警示', txt)
      elif w == 'onroadEvents':
        note(t, '事件', ','.join(sorted(str(ev.name) for ev in e.onroadEvents)) or '(無)')
      elif w == 'pandaStates' and len(e.pandaStates):
        ps = e.pandaStates[0]
        note(t, 'panda允許控制', ps.controlsAllowed)
        note(t, 'panda允許轉向(MADS)', ps.controlsAllowedLateral)
        note(t, 'panda MADS設定(alternativeExperience)', ps.alternativeExperience)
        note(t, 'panda接收檢查無效', ps.safetyRxChecksInvalid)
        note(t, 'panda安全模式', f'{ps.safetyModel} param={ps.safetyParam}')
        if safety_ready_t is None and str(ps.safetyModel) == 'subaru':
          safety_ready_t = t
        note(t, '線束狀態', str(ps.harnessStatus))
        note(t, 'panda故障', str(ps.faultStatus))
        if tx_blocked is not None and ps.safetyTxBlocked > tx_blocked:
          note(t, '被safety擋下的送出訊息(累計)', ps.safetyTxBlocked)
        if rx_invalid is not None and ps.safetyRxInvalid > rx_invalid:
          note(t, 'safety判定無效的接收訊息(累計)', ps.safetyRxInvalid)
        tx_blocked, rx_invalid = ps.safetyTxBlocked, ps.safetyRxInvalid
      elif w == 'can':
        for m in e.can:
          k = (m.address, m.src)
          if k not in WATCH or len(m.dat) < 8:
            continue
          v = int.from_bytes(m.dat, 'little')
          for label, bit, width in WATCH[k]:
            note(t, label, (v >> bit) & ((1 << width) - 1))
      elif w == 'sendcan':
        for m in e.sendcan:
          if m.address == 0x221:
            note(t, 'C4送出ES_Distance(取消巡航)', f'bus{m.src} cancel={(m.dat[7] & 1) if len(m.dat) >= 8 else "?"}')
          if m.address in SENT_WATCH and len(m.dat) >= 8:
            v = int.from_bytes(m.dat, 'little')
            for label, bit, width in SENT_WATCH[m.address]:
              note(t, label, (v >> bit) & ((1 << width) - 1))
        for m in e.sendcan:
          sent[(m.address, m.src)] = sent.get((m.address, m.src), 0) + 1
          if m.address == 0x124 and len(m.dat) >= 8:
            d = m.dat
            req = (d[1] >> 4) & 1
            a = to_signed((int.from_bytes(d, 'little') >> 40) & 0x1FFFF, 17) * -0.01
            angle['n'] += 1
            cur['cmd'], cur['req'] = a, req
            angle['req'] += req
            if req:
              angle['lo'], angle['hi'] = min(angle['lo'], a), max(angle['hi'], a)
            if last.get('C4送出角度請求') != req:
              note(t, 'C4送出角度請求', f"{req} (指令 {a:.2f}° / 方向盤實際 {cur.get('angle', float('nan')):.2f}° / 車速 {cur.get('v', float('nan')) * 3.6:.1f} km/h)")
              last['C4送出角度請求'] = req
    except Exception as ex:
      note(t, '解析錯誤', f'{w}: {ex}')

def fmt_snap(c):
  f = lambda k, n=1: f"{c[k]:.{n}f}" if isinstance(c.get(k), float) else str(c.get(k, '?'))
  return (f"車速 {c['v'] * 3.6:.1f} km/h | 方向盤 {f('angle')}° | C4指令 {f('cmd', 2)}° (請求={c.get('req', '?')}) | "
          f"駕駛扭力 {f('torque', 0)} (手動轉向={c.get('pressed', '?')}) | ACC啟用={c.get('acc', '?')} | 煞車={c.get('brake', '?')} | 油門={c.get('gas', '?')}") if 'v' in c else '(還沒有車輛狀態)'

print(f"\n紀錄讀取：共 {seg_stats['total']} 段，讀到第 {t_last[0]:.1f} 秒" +
      (f" | 未寫完但已讀取: {', '.join(seg_stats['partial'])}" if seg_stats['partial'] else '') +
      (f" | 無法讀取: {', '.join(seg_stats['skipped'])}" if seg_stats['skipped'] else ''))

print('\n=== 整趟出現過的畫面警示 ===')
print('\n'.join(f'{t:8.2f}s  {a}' for t, a in alerts) or '  (無)')

aeb = [(t, v) for t, _, k, v in lines if k == 'EyeSight ES_Brake AEB_Status']
print('\n=== EyeSight AEB_Status 變化（前 30 筆）===')
print('\n'.join(f'{t:8.2f}s  {v}' for t, v in aeb[:30]) or '  (無)')

print('\n=== 基本設定 ===')
for t, l, _, _ in lines:
  if any(k in l for k in ('車型:', 'panda安全模式: subaru', 'panda MADS設定')):
    print(l)

clusters = []
for f in faults:
  if not clusters or f[0] - clusters[-1][-1][0] > 30:
    clusters.append([f])
  else:
    clusters[-1].append(f)

if not clusters:
  print('\n沒有偵測到故障。最後 60 筆變化：')
  print('\n'.join(l for _, l, _, _ in lines[-60:]))
for i, cl in enumerate(clusters, 1):
  t_first = cl[0][0]
  print(f'\n=== 故障 {i}：第 {t_first:.2f} 秒 ===')
  print('最先出現：', ', '.join(f'{k}={v}' for _, k, v, _ in cl[:4]))
  print('當下狀態：', fmt_snap(cl[0][3]))
  before = {}
  for t, _, k, v in lines:
    if t < t_first - 20:
      before[k] = v
  keep = ('ACC', 'sunnypilot', 'panda允許', 'MADS', 'EPS', 'EyeSight', '車型', '畫面警示')
  print(f'--- 第 {max(t_first - 20, 0):.0f} 秒時的狀態 ---')
  print('  ' + ' | '.join(f'{k}={v}' for k, v in before.items() if any(x in k for x in keep)) if before else '  (從開機就開始)')
  print(f'--- 前 20 秒到後 5 秒的變化 ---')
  win = [l for t, l, _, _ in lines if t_first - 20 <= t <= t_first + 5]
  print('\n'.join(win[-150:]))

print(f'\n=== 行車最後 90 秒（第 {max(t_last[0] - 90, 0):.0f} 秒之後）的變化 ===')
end = [l for t, l, k, _ in lines if t >= t_last[0] - 90 and k not in ('事件', 'sunnypilot MADS狀態')]
print('\n'.join(end[-200:]) or '  (無)')

print('\n=== C4 送出的轉向相關訊息 (ID, bus): 筆數 ===')
for (a, b), n in sorted(sent.items()):
  if a in (0x124, 0x221, 0x321, 0x322, 0x323):
    print(f'  {hex(a)} bus{b}: {n}')
print(f"角度指令 0x124: 共 {angle['n']} 筆 | 請求=1 的 {angle['req']} 筆 | 請求時角度範圍 {angle['lo']:.2f} ~ {angle['hi']:.2f} 度")
