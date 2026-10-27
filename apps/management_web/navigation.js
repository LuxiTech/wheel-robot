// All automatic requests are GET. POST is emitted only by an explicit control click.
const $ = selector => document.querySelector(selector);
const states = {SUBMITTING:'正在提交', MOVING:'导航中', ARRIVED:'已到点，等待停车', STOPPING:'等待停车确认',
  SUCCEEDED:'已到达并停车', STOPPED:'已停车', CANCELLED:'已取消并停车', REJECTED:'请求被拒绝',
  UNKNOWN:'状态未知，请确认停车', FAULT:'导航异常，已确认停车'};
let current = null, sending = false, refreshing = false, controlEnabled = false;
let pending = null;
try { pending = JSON.parse(sessionStorage.getItem('wheel.navigation.pending') || 'null'); } catch { /* optional session cache */ }

function remember(value) {
  pending = value;
  try {
    if (value) sessionStorage.setItem('wheel.navigation.pending', JSON.stringify(value));
    else sessionStorage.removeItem('wheel.navigation.pending');
  } catch { /* backend command journal remains authoritative */ }
}
function node(tag, text, className) {
  const element = document.createElement(tag); element.textContent = text;
  if (className) element.className = className;
  return element;
}
function commandId() {
  if (crypto.randomUUID) return crypto.randomUUID();
  const bytes = crypto.getRandomValues(new Uint8Array(16));
  bytes[6] = (bytes[6] & 15) | 64; bytes[8] = (bytes[8] & 63) | 128;
  const hex = Array.from(bytes, byte => byte.toString(16).padStart(2, '0')).join('');
  return [hex.slice(0,8), hex.slice(8,12), hex.slice(12,16), hex.slice(16,20), hex.slice(20)].join('-');
}
function buttons() {
  document.querySelectorAll('[data-nav-go]').forEach(button => {
    button.disabled = sending || !!pending || !current?.can_go;
  });
  $('#nav-stop').disabled = sending || !controlEnabled;
}
async function api(path, body) {
  const response = await fetch('/api/v1/' + path, {
    method: body ? 'POST' : 'GET', cache:'no-store', signal:AbortSignal.timeout(12000),
    headers: body ? {'Content-Type':'application/json'} : {},
    body: body ? JSON.stringify(body) : undefined
  });
  let data;
  try { data = await response.json(); } catch { throw new Error('后台未返回有效 JSON'); }
  if (!response.ok) {
    const error = new Error(data.reason || data.detail || 'HTTP ' + response.status);
    error.status = response.status; error.command = data.command_id ? data : null;
    throw error;
  }
  return data;
}
function render(snapshot) {
  current = snapshot;
  controlEnabled = snapshot.enabled;
  $('#control-mode').textContent = snapshot.enabled ? 'A/B 导航控制已开启' : '观测模式 · 导航未开启';
  const sdkNode = $('#sdk-status'); sdkNode.replaceChildren();
  for (const [id, label] of [['atlas','陆图 · lucid-atlas'],['lift','升降机器人 · lift_robot_ros2']]) {
    const sdk = snapshot.devices[id]; const card = node('article', '', 'card');
    card.append(node('span', label), node('strong', sdk.connected ? '已连接' : '未连接', sdk.connected ? 'ready' : 'pending'),
      node('p', sdk.base_url, 'small'), node('p', sdk.reason || '状态未知', 'small'));
    if (id === 'lift') card.append(node('p', '底盘反馈：' + (sdk.base_ready ? '可用' : '未就绪') + ' · 静止：' + (sdk.stationary ? '已确认' : '未确认'), 'small'));
    sdkNode.append(card);
  }
  $('#nav-localization').textContent = '定位：' + snapshot.localization + ' · 地图版本：' + (snapshot.map_revision || '未知');
  const points = $('#nav-points'); points.replaceChildren();
  for (const name of ['A','B']) {
    const point = snapshot.points.find(p => p.name === name);
    const card = node('article', '', 'point-card');
    card.append(node('strong', name + ' 点'), node('p', point ?
      `x ${point.x.toFixed(3)} m · y ${point.y.toFixed(3)} m · yaw ${point.yaw.toFixed(3)} rad` : 'SDK 尚未提供该点位', 'small'));
    points.append(card);
  }
  const command = snapshot.command;
  $('#nav-state').textContent = command ? (states[command.state] || command.state) : snapshot.nav_state;
  $('#nav-command').textContent = command ? `${command.point ? command.point + ' 点 · ' : ''}${states[command.state] || command.state} · ${command.reason || ''} · ${command.command_id}` : '尚无导航命令';
  if (command && !sending) $('#nav-message').textContent = (states[command.state] || command.state) + '：' + (command.reason || '');
  $('#nav-reasons').replaceChildren(...snapshot.reasons.map(reason => node('li', reason)));
  if (pending && command?.command_id === pending.command_id) remember(null);
  buttons();
}
async function refreshNavigation() {
  if (refreshing) return;
  refreshing = true;
  try {
    const snapshot = await api('navigation');
    render(snapshot);
    if (pending) {
      try {
        await api('navigation/commands/' + pending.command_id);
        remember(null); buttons();
      } catch (error) {
        if (error.status === 404) {
          remember(null); buttons();
          $('#nav-message').textContent = '上一请求未写入命令记录，可在确认就绪后重新操作。';
        }
      }
    }
  } catch (error) {
    current = null; buttons();
    $('#control-mode').textContent = '后台状态不可用';
    $('#sdk-status').replaceChildren(node('p', 'SDK 连接状态未知：' + error.message));
    $('#nav-state').textContent = '状态未知';
    $('#nav-message').textContent = '查询失败，已禁用导航按钮：' + error.message;
  } finally {
    refreshing = false;
  }
}
async function send(action, point) {
  if (sending || !current?.enabled || (action === 'go' && (!current.can_go || pending))) return;
  sending = true;
  current.can_go = false;
  const command = {command_id:commandId(), ...(point ? {point} : {})};
  remember(command); buttons();
  $('#nav-message').textContent = action === 'go' ? '正在提交 ' + point + ' 点导航…' : '正在请求停止导航…';
  try {
    const result = await api('navigation/' + action, command);
    remember(null);
    $('#nav-message').textContent = (states[result.state] || result.state) + '：' + (result.reason || '');
  } catch (error) {
    if (error.status) remember(null); // Server response is known; uncertain command is journaled.
    $('#nav-message').textContent = error.status ? error.message : '请求结果未知；正在查询命令记录，请勿重复提交。';
  } finally {
    sending = false; buttons(); await refreshNavigation();
  }
}
document.querySelectorAll('[data-nav-go]').forEach(button => button.addEventListener('click', () => send('go', button.dataset.navGo)));
$('#nav-stop').addEventListener('click', () => send('stop'));
$('#refresh').addEventListener('click', refreshNavigation);
async function poll() { await refreshNavigation(); setTimeout(poll, 2000); }
poll();
