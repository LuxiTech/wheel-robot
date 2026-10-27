const labels = {
  navigation_ready: '工位导航', vision_source_ready: '同步视觉观测',
  motion_executor_ready: '运动执行器', hardware_execution_implemented: '真机执行闭环'
};
const phases = {
  PREPARE_EMPTY:'空载运输姿态', NAVIGATE_SOURCE:'前往源工位', OBSERVE:'获取观测',
  PLAN_PREGRASP:'预抓取规划', PREGRASP:'执行预抓取', REOBSERVE:'重新观测',
  PLAN_APPROACH:'接近规划', APPROACH:'接近箱体', CLOSE:'夹紧并确认', TEST_LIFT:'试抬验证',
  PREPARE_LOADED:'持载运输姿态', NAVIGATE_DESTINATION:'前往目标工位', PLACE:'放置并退出', COMPLETE:'完成'
};
function element(tag, value, className) {
  const node = document.createElement(tag); node.textContent = value;
  if (className) node.className = className;
  return node;
}
async function get(path) {
  const response = await fetch('/api/v1/' + path, {cache: 'no-store', signal: AbortSignal.timeout(8000)});
  if (!response.ok) throw new Error(path + '：HTTP ' + response.status);
  return response.json();
}
async function refresh() {
  const button = document.querySelector('#refresh'); button.disabled = true;
  document.querySelector('#message').textContent = '正在读取后台状态…';
  try {
    const [status, workflow, configurations, runs] = await Promise.all(
      ['status', 'workflow', 'configurations', 'runs'].map(get));
    const statusNode = document.querySelector('#status'); statusNode.replaceChildren();
    for (const [key, label] of Object.entries(labels)) {
      const card = element('article', '', 'card');
      card.append(element('span', label), element('strong', status[key] ? '已就绪' : '未就绪', status[key] ? 'ready' : 'pending'));
      statusNode.append(card);
    }
    document.querySelector('#job').textContent = workflow.job.target_id + ' · ' + workflow.job.source_station + ' → ' + workflow.job.destination_station;
    document.querySelector('#workflow').replaceChildren(...workflow.phases.map(p => element('li', phases[p] || p)));
    const configNode = document.querySelector('#configurations'); configNode.replaceChildren();
    for (const [group, files] of Object.entries(configurations)) {
      const list = element('ul', ''); list.append(...files.map(f => element('li', f)));
      configNode.append(element('h3', group), list);
    }
    const body = document.querySelector('#runs'); body.replaceChildren();
    for (const run of runs) {
      const row = element('tr', ''); row.append(...[run.run_id, run.phase, run.success ? '成功' : '未完成', run.reason || '—'].map(v => element('td', String(v)))); body.append(row);
    }
    if (!runs.length) { const row = element('tr', ''); const cell = element('td', '暂无任务运行记录'); cell.colSpan = 4; row.append(cell); body.append(row); }
    document.querySelector('#message').textContent = '更新于 ' + new Date().toLocaleTimeString('zh-CN') + ' · 默认配置查询成功';
  } catch (error) {
    document.querySelector('#message').textContent = '读取失败，页面内容可能已过期：' + error.message;
  } finally { button.disabled = false; }
}
document.querySelector('#refresh').addEventListener('click', refresh);
refresh();
import './navigation.js';

