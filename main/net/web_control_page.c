#include "net/web_control_page.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config/app_config.h"

static const char *INDEX_HTML_TEMPLATE =
    "<!doctype html><html><head><meta charset='utf-8'/>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
    "<title>ESP MotorBridge</title>"
    "<style>"
    "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;padding:12px;background:#edf2f7;color:#0f172a}"
    ".card{background:#fff;border-radius:12px;padding:14px;box-shadow:0 6px 24px rgba(2,6,23,.08);margin-bottom:12px}"
    "h1{font-size:20px;margin:0 0 8px}h2{font-size:16px;margin:0 0 8px}h3{font-size:16px;margin:0}"
    ".row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}.row>*{margin:4px 0}"
    "button{border:none;padding:8px 11px;border-radius:9px;background:#0f766e;color:#fff;font-weight:600;cursor:pointer}"
    "button.gray{background:#475569}button.red{background:#b91c1c}button.orange{background:#b45309}"
    "button.micro{padding:5px 8px;min-width:34px;font-size:13px;background:#334155}"
    "input,select{padding:7px;border-radius:8px;border:1px solid #cbd5e1;min-width:80px}"
    ".democtl{padding:10px 14px !important;min-height:40px;font-size:15px;font-weight:700}"
    "select.democtl{min-width:120px;background:#fff}"
    "input[type=range]{padding:0;min-width:180px;vertical-align:middle}"
    ".motors{display:grid;grid-template-columns:repeat(auto-fit,minmax(360px,1fr));gap:10px}"
    ".motor{border:1px solid #e2e8f0;border-radius:10px;padding:10px;background:#fcfdff}"
    ".hint{font-size:12px;color:#64748b}.ok{color:#047857}.err{color:#b91c1c}.mhide{display:none}"
    ".locked{opacity:.55;pointer-events:none;filter:grayscale(.2)}"
    "</style></head><body>"
    "<div class='card'><h1>ESP MotorBridge 控制台</h1>"
    "<div class='hint'>独立电机控制: 每台电机独立模式/参数/滑条。切模式默认走 disable->set_mode->enable。</div></div>"
    "<div class='card'><h2>全局操作</h2>"
    "<div class='row'>"
    "<button onclick='enableAll()'>Enable All</button>"
    "<button class='red' onclick='disableAll()'>Disable All</button>"
    "</div>"
    "<div class='row'>"
    "<select id='vendor'><option value='damiao'>Damiao</option><option value='robstride'>RobStride</option></select>"
    "<button class='gray' onclick='applyVendorAll()'>应用类型到全部电机</button>"
    "</div>"
    "<div class='row'>"
    "<select id='demo_id' class='democtl'><option value='1'>Demo1</option><option value='2'>Demo2</option><option value='3'>Demo3</option></select>"
    "<button class='orange democtl' onclick='demoStart()'>启动 Demo</button>"
    "</div>"
    "<div class='row'>"
    "<button class='red democtl' onclick='demoStop()'>停止 Demo</button>"
    "<button class='gray democtl' onclick='demoReset()'>复位</button>"
    "<button class='red democtl' onclick='estopAll()'>E-Stop</button>"
    "</div>"
    "<div class='row'>"
    "<input id='id_old' type='number' min='1' max='127' step='1' value='1'/>"
    "<input id='id_new' type='number' min='1' max='127' step='1' value='2'/>"
    "<button class='gray' onclick='setDamiaoId()'>改ID(Damiao)</button>"
    "</div></div>"
    "<div class='card'><h2>电机面板</h2><div id='motors' class='motors'></div><div id='global_msg' class='hint'></div></div>"
    "<script>"
    "const maxMotors=%d;"
    "const motors=document.getElementById('motors');"
    "const globalMsg=document.getElementById('global_msg');"
    "const allIds=Array.from({length:maxMotors},(_,i)=>i+1).join(',');"
    "const rtTimers={};"
    "let ws=null;let wsReqId=1;const wsPending=new Map();let wsReady=false;"
    "function wsUrl(){const proto=(location.protocol==='https:')?'wss':'ws';return `${proto}://${location.host}/ws`;}"
    "function wsConnect(){if(ws&&(ws.readyState===WebSocket.OPEN||ws.readyState===WebSocket.CONNECTING))return;ws=new WebSocket(wsUrl());"
    "ws.onopen=()=>{wsReady=true;showGlobal(true,'WS connected');};"
    "ws.onclose=()=>{wsReady=false;for(const [_,v] of wsPending){v({ok:false,text:'ws closed'});}wsPending.clear();setTimeout(wsConnect,800);};"
    "ws.onerror=()=>{wsReady=false;};"
    "ws.onmessage=(ev)=>{const t=String(ev.data||'');const m=t.match(/^rid=(\\d+);([\\s\\S]*)$/);if(!m)return;const rid=parseInt(m[1],10);const p=wsPending.get(rid);if(!p)return;wsPending.delete(rid);p({ok:m[2].startsWith('ok'),text:m[2]});};}"
    "let demoBusy=false;"
    "function showGlobal(ok,t){globalMsg.className=ok?'hint ok':'hint err';globalMsg.textContent=t;}"
    "function setPanelLocked(locked){const cards=document.querySelectorAll('.motor');cards.forEach(c=>c.classList.toggle('locked',locked));}"
    "async function refreshDemoStatus(){const r=await api({action:'demo_status'});if(!r.ok)return;const m=(r.text||'').match(/running=(\\d)\\s+id=(\\d+)/);if(!m)return;const running=m[1]==='1';if(running!==demoBusy){demoBusy=running;setPanelLocked(running);showGlobal(true,running?`demo running(id=${m[2]}), 请先停止 demo 再手动控制`:\"demo stopped, 手动控制已恢复\");}}"
    "function showOne(id,ok,t){const e=document.getElementById(`msg_${id}`);if(!e)return;e.className=ok?'hint ok':'hint err';e.textContent=t;}"
    "function val(id,k){const e=document.getElementById(`${k}_${id}`);return e?e.value:'';}"
    "function modeVal(id){return parseInt(val(id,'mode')||'1',10);}"
    "async function api(params){if(!wsReady||!ws||ws.readyState!==WebSocket.OPEN){return {ok:false,text:'ws not connected'};}const rid=wsReqId++;const q=new URLSearchParams({...params,rid:String(rid)}).toString();return await new Promise((resolve)=>{wsPending.set(rid,resolve);try{ws.send(q);}catch(e){wsPending.delete(rid);resolve({ok:false,text:'ws send error'});}setTimeout(()=>{if(wsPending.has(rid)){wsPending.delete(rid);resolve({ok:false,text:'ws timeout'});}},1200);});}"
    "function sleep(ms){return new Promise(r=>setTimeout(r,ms));}"
    "function parseStatePos(text){const m=(text||'').match(/\\bpos=([-+]?\\d*\\.?\\d+)/);return m?parseFloat(m[1]):null;}"
    "function updateStateUi(id,o){const e=document.getElementById(`st_${id}`);if(!e)return;if(!o){e.textContent='offline';e.className='hint err';return;}e.className=o.online?'hint ok':'hint err';e.textContent=`online=${o.online?1:0} mode=${o.mode} pos=${o.pos.toFixed(3)} vel=${o.vel.toFixed(3)} tau=${o.tau.toFixed(3)}`;}"
    "function parseStateAll(text){const out={};const i=text.indexOf('id=');if(i<0)return out;const body=text.slice(i);for(const item of body.split(';')){if(!item)continue;const m=item.match(/id=(\\d+),online=(\\d+),mode=(\\d+),pos=([-+]?\\d*\\.?\\d+),vel=([-+]?\\d*\\.?\\d+),tau=([-+]?\\d*\\.?\\d+)/);if(!m)continue;const id=parseInt(m[1],10);out[id]={online:m[2]==='1',mode:parseInt(m[3],10),pos:parseFloat(m[4]),vel:parseFloat(m[5]),tau:parseFloat(m[6])};}return out;}"
    "async function refreshStates(){const r=await api({action:'state_all'});if(!r.ok)return;const st=parseStateAll(r.text);for(let i=1;i<=maxMotors;i++){updateStateUi(i,st[i]||null);}}"
    "function setPosInput(id,p){if(typeof p!=='number'||Number.isNaN(p))return;const pe=document.getElementById(`pos_${id}`);const pr=document.getElementById(`posr_${id}`);if(pe)pe.value=p.toFixed(4);if(pr)pr.value=p.toFixed(4);}"
    "function bumpPos(id,delta){const pe=document.getElementById(`pos_${id}`);const pr=document.getElementById(`posr_${id}`);if(!pe||!pr)return;let p=parseFloat(pe.value||'0');if(Number.isNaN(p))p=0;p+=delta;const mn=parseFloat(pr.min||'-3.14');const mx=parseFloat(pr.max||'3.14');if(p<mn)p=mn;if(p>mx)p=mx;pe.value=p.toFixed(4);pr.value=p.toFixed(4);scheduleRt(id);}"
    "async function doAction(id,action){const r=await api({action:action,ids:String(id)});showOne(id,r.ok,r.text);return r.ok;}"
    "async function setGains(id){const r=await api({action:'set_gains',ids:String(id),kp:val(id,'kp'),kd:val(id,'kd')});showOne(id,r.ok,r.text);}"
    "async function setModeSafe(id){const mode=modeVal(id);const rt=document.getElementById(`rt_${id}`);const rtWas=!!(rt&&rt.checked);if(rt)rt.checked=false;"
    "const s1=await api({action:'disable',ids:String(id)});await sleep(30);"
    "const s2=await api({action:'clear_error',ids:String(id)});await sleep(25);"
    "const st=await api({action:'state',ids:String(id)});"
    "let holdPos=parseStatePos(st.text);"
    "if(holdPos===null||Number.isNaN(holdPos)){holdPos=parseFloat(val(id,'pos')||'0');}"
    "setPosInput(id,holdPos);"
    "const s3=await api({action:'set_mode',ids:String(id),mode:String(mode)});await sleep(25);"
    "const s4=await api({action:'enable',ids:String(id)});await sleep(25);"
    "const safeVel='0';const safeTau='0';const safeVlim=(mode===2||mode===4)?'0.5':'1.0';const safeRatio='0.1';"
    "const s5=await api({action:'apply',ids:String(id),mode:String(mode),pos:String(holdPos),vel:safeVel,tau:safeTau,vlim:safeVlim,ratio:safeRatio});"
    "if(rt&&rtWas)rt.checked=true;"
    "const ok=s1.ok&&s2.ok&&s3.ok&&s4.ok&&s5.ok;"
    "showOne(id,ok,ok?`mode=${mode} switched (hold pos=${holdPos.toFixed(3)})`:('switch failed: '+[s1,s2,s3,s4,s5].map(x=>x.text).join(' | ')));}"
    "async function apply(id){const mode=modeVal(id);const vlim=parseFloat(val(id,'vlim')||'0');if((mode===2||mode===4)&&vlim<=0){showOne(id,false,'vlim must be > 0');return;}"
    "const r=await api({action:'apply',ids:String(id),mode:String(mode),pos:val(id,'pos'),vel:val(id,'vel'),tau:val(id,'tau'),vlim:val(id,'vlim'),ratio:val(id,'ratio')});showOne(id,r.ok,r.text);}"
    "function scheduleRt(id){const rt=document.getElementById(`rt_${id}`);if(!rt||!rt.checked)return;clearTimeout(rtTimers[id]);rtTimers[id]=setTimeout(()=>apply(id),120);}"
    "function setShow(id,key,show){const e=document.getElementById(`${key}w_${id}`);if(!e)return;e.classList.toggle('mhide',!show);}"
    "function updateModeUi(id){const m=modeVal(id);const showPos=(m===1||m===2||m===4);setShow(id,'pos',showPos);setShow(id,'vel',m===1||m===3);setShow(id,'tau',m===1);setShow(id,'vlim',m===2||m===4);setShow(id,'ratio',m===4);const pr=document.getElementById(`posr_${id}`);if(pr)pr.classList.toggle('mhide',!showPos);const h=document.getElementById(`mh_${id}`);if(h){h.textContent=m===1?'MIT: pos/vel/tau + gains':(m===2?'PosVel: pos/vlim':(m===3?'Vel: vel':'ForcePos: pos/vlim/ratio'));}}"
    "function bindCard(id){const p=document.getElementById(`pos_${id}`);const r=document.getElementById(`posr_${id}`);if(p&&r){p.addEventListener('input',()=>{r.value=p.value;scheduleRt(id);});r.addEventListener('input',()=>{p.value=r.value;scheduleRt(id);});}"
    "const ms=document.getElementById(`mode_${id}`);if(ms)ms.addEventListener('change',()=>updateModeUi(id));"
    "['vel','tau','vlim','ratio'].forEach(k=>{const e=document.getElementById(`${k}_${id}`);if(e)e.addEventListener('input',()=>scheduleRt(id));});updateModeUi(id);}"
    "function motorCard(id){return `<div class='motor'><div class='row'><h3>M${id}</h3><button onclick='doAction(${id},\"enable\")'>Enable</button><button class='red' onclick='doAction(${id},\"disable\")'>Disable</button><button class='gray' onclick='setModeSafe(${id})'>切模式(安全)</button></div><div class='row'><label>mode <select id='mode_${id}'><option value='1'>MIT</option><option value='2'>PosVel</option><option value='3'>Vel</option><option value='4'>ForcePos</option></select></label><label>kp <input id='kp_${id}' type='number' step='0.01' value='0.5'></label><label>kd <input id='kd_${id}' type='number' step='0.001' value='0.08'></label><button class='gray' onclick='setGains(${id})'>Gains</button></div><div class='row'><label id='posw_${id}'>pos <input id='pos_${id}' type='number' step='0.001' value='0'></label><button class='micro' onclick='bumpPos(${id},-0.1)'>-</button><input id='posr_${id}' type='range' min='-3.14' max='3.14' step='0.001' value='0'><button class='micro' onclick='bumpPos(${id},0.1)'>+</button><label id='velw_${id}'>vel <input id='vel_${id}' type='number' step='0.01' value='0'></label><label id='tauw_${id}'>tau <input id='tau_${id}' type='number' step='0.01' value='0.3'></label><label id='vlimw_${id}'>vlim <input id='vlim_${id}' type='number' step='0.01' value='1'></label><label id='ratiow_${id}'>ratio <input id='ratio_${id}' type='number' step='0.0001' value='0.1'></label><label><input id='rt_${id}' type='checkbox' checked>实时</label><button class='orange' onclick='apply(${id})'>Apply</button></div><div id='st_${id}' class='hint'>offline</div><div id='mh_${id}' class='hint'></div><div id='msg_${id}' class='hint'>ready</div></div>`;}"
    "for(let i=1;i<=maxMotors;i++){motors.insertAdjacentHTML('beforeend',motorCard(i));bindCard(i);}"
    "wsConnect();"
    "setInterval(refreshDemoStatus,500);refreshDemoStatus();"
    "setInterval(refreshStates,260);refreshStates();"
    "async function enableAll(){const r=await api({action:'enable_all'});showGlobal(r.ok,r.text);}"
    "async function disableAll(){const r=await api({action:'disable_all'});showGlobal(r.ok,r.text);}"
    "async function estopAll(){const r=await api({action:'estop',ids:allIds});showGlobal(r.ok,r.text);}"
    "async function applyVendorAll(){const r=await api({action:'set_vendor',ids:allIds,vendor:document.getElementById('vendor').value});showGlobal(r.ok,r.text);}"
    "async function demoStart(){const did=document.getElementById('demo_id').value;const r=await api({action:'demo_start',demo_id:did});showGlobal(r.ok,r.text);}"
    "async function demoStop(){const r=await api({action:'demo_stop'});showGlobal(r.ok,r.text);}"
    "async function demoReset(){const r=await api({action:'demo_reset'});showGlobal(r.ok,r.text);}"
    "async function setDamiaoId(){const oldId=parseInt(document.getElementById('id_old').value||'0',10);const newId=parseInt(document.getElementById('id_new').value||'0',10);if(!oldId||!newId||oldId===newId){showGlobal(false,'bad id');return;}const r=await api({action:'set_id',id_old:String(oldId),id_new:String(newId)});showGlobal(r.ok,r.text);}"
    "</script></body></html>";

esp_err_t web_control_root_get_handler(httpd_req_t *req)
{
    const app_config_t *cfg = app_config_get();
    size_t cap = strlen(INDEX_HTML_TEMPLATE) + 32;
    char *html = calloc(1, cap);
    if (html == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "internal error");
        return ESP_FAIL;
    }

    int n = snprintf(html, cap, INDEX_HTML_TEMPLATE, cfg->max_motors);
    if (n <= 0 || (size_t)n >= cap) {
        free(html);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "internal error");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    esp_err_t err = httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    free(html);
    return err;
}

esp_err_t web_control_favicon_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}
