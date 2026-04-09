#include "net/web_control.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/twai.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/command_router.h"
#include "config/app_config.h"
#include "core/motor_manager.h"

static const char *TAG = "web_control";
static httpd_handle_t s_server = NULL;
static char s_last_query[256];
static int64_t s_last_query_us = 0;
static const int64_t WEB_DUPLICATE_GUARD_US = 800000;
static const int INTER_ID_DELAY_MS = 10;

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
    "input,select{padding:7px;border-radius:8px;border:1px solid #cbd5e1;min-width:80px}"
    "input[type=range]{padding:0;min-width:180px;vertical-align:middle}"
    ".motors{display:grid;grid-template-columns:repeat(auto-fit,minmax(360px,1fr));gap:10px}"
    ".motor{border:1px solid #e2e8f0;border-radius:10px;padding:10px;background:#fcfdff}"
    ".hint{font-size:12px;color:#64748b}.ok{color:#047857}.err{color:#b91c1c}.mhide{display:none}"
    "</style></head><body>"
    "<div class='card'><h1>ESP MotorBridge 控制台</h1>"
    "<div class='hint'>独立电机控制: 每台电机独立模式/参数/滑条。切模式默认走 disable->set_mode->enable。</div></div>"
    "<div class='card'><h2>全局操作</h2>"
    "<div class='row'>"
    "<button onclick='enableAll()'>Enable All</button>"
    "<button class='red' onclick='disableAll()'>Disable All</button>"
    "<button class='red' onclick='estopAll()'>E-Stop</button>"
    "<select id='vendor'><option value='damiao'>Damiao</option><option value='robstride'>RobStride</option></select>"
    "<button class='gray' onclick='applyVendorAll()'>应用类型到全部电机</button>"
    "<button class='orange' onclick='startRecord()'>开始录制</button>"
    "<button class='gray' onclick='stopRecord()'>停止录制</button>"
    "<button onclick='playRecord()'>播放序列</button>"
    "<button class='red' onclick='clearRecord()'>清空序列</button>"
    "</div></div>"
    "<div class='card'><h2>电机面板</h2><div id='motors' class='motors'></div><div id='global_msg' class='hint'></div></div>"
    "<script>"
    "const maxMotors=%d;"
    "const motors=document.getElementById('motors');"
    "const globalMsg=document.getElementById('global_msg');"
    "const allIds=Array.from({length:maxMotors},(_,i)=>i+1).join(',');"
    "const rtTimers={};"
    "let recording=false;let recordStartMs=0;let actionSeq=[];"
    "function showGlobal(ok,t){globalMsg.className=ok?'hint ok':'hint err';globalMsg.textContent=t;}"
    "function showOne(id,ok,t){const e=document.getElementById(`msg_${id}`);if(!e)return;e.className=ok?'hint ok':'hint err';e.textContent=t;}"
    "function val(id,k){const e=document.getElementById(`${k}_${id}`);return e?e.value:'';}"
    "function modeVal(id){return parseInt(val(id,'mode')||'1',10);}"
    "async function api(params){try{const r=await fetch('/api/control?'+new URLSearchParams(params).toString());const t=await r.text();return {ok:r.ok&&t.startsWith('ok'),text:t};}catch(e){return {ok:false,text:'network error'};}}"
    "function sleep(ms){return new Promise(r=>setTimeout(r,ms));}"
    "function parseStatePos(text){const m=(text||'').match(/\\bpos=([-+]?\\d*\\.?\\d+)/);return m?parseFloat(m[1]):null;}"
    "function setPosInput(id,p){if(typeof p!=='number'||Number.isNaN(p))return;const pe=document.getElementById(`pos_${id}`);const pr=document.getElementById(`posr_${id}`);if(pe)pe.value=p.toFixed(4);if(pr)pr.value=p.toFixed(4);}"
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
    "async function apply(id,fromReplay){const mode=modeVal(id);const vlim=parseFloat(val(id,'vlim')||'0');if((mode===2||mode===4)&&vlim<=0){showOne(id,false,'vlim must be > 0');return;}"
    "const payload={action:'apply',ids:String(id),mode:String(mode),pos:val(id,'pos'),vel:val(id,'vel'),tau:val(id,'tau'),vlim:val(id,'vlim'),ratio:val(id,'ratio')};"
    "const r=await api(payload);showOne(id,r.ok,r.text);"
    "if(recording&&!fromReplay&&r.ok){actionSeq.push({dt:Date.now()-recordStartMs,id:id,mode:payload.mode,pos:payload.pos,vel:payload.vel,tau:payload.tau,vlim:payload.vlim,ratio:payload.ratio});showGlobal(true,`recording... frames=${actionSeq.length}`);}}"
    "function scheduleRt(id){const rt=document.getElementById(`rt_${id}`);if(!rt||!rt.checked)return;clearTimeout(rtTimers[id]);rtTimers[id]=setTimeout(()=>apply(id),120);}"
    "function setShow(id,key,show){const e=document.getElementById(`${key}w_${id}`);if(!e)return;e.classList.toggle('mhide',!show);}"
    "function updateModeUi(id){const m=modeVal(id);const showPos=(m===1||m===2||m===4);setShow(id,'pos',showPos);setShow(id,'vel',m===1||m===3);setShow(id,'tau',m===1);setShow(id,'vlim',m===2||m===4);setShow(id,'ratio',m===4);const pr=document.getElementById(`posr_${id}`);if(pr)pr.classList.toggle('mhide',!showPos);const h=document.getElementById(`mh_${id}`);if(h){h.textContent=m===1?'MIT: pos/vel/tau + gains':(m===2?'PosVel: pos/vlim':(m===3?'Vel: vel':'ForcePos: pos/vlim/ratio'));}}"
    "function bindCard(id){const p=document.getElementById(`pos_${id}`);const r=document.getElementById(`posr_${id}`);if(p&&r){p.addEventListener('input',()=>{r.value=p.value;scheduleRt(id);});r.addEventListener('input',()=>{p.value=r.value;scheduleRt(id);});}"
    "const ms=document.getElementById(`mode_${id}`);if(ms)ms.addEventListener('change',()=>updateModeUi(id));"
    "['vel','tau','vlim','ratio'].forEach(k=>{const e=document.getElementById(`${k}_${id}`);if(e)e.addEventListener('input',()=>scheduleRt(id));});updateModeUi(id);}"
    "function motorCard(id){return `<div class='motor'><div class='row'><h3>M${id}</h3><button onclick='doAction(${id},\"enable\")'>Enable</button><button class='red' onclick='doAction(${id},\"disable\")'>Disable</button><button class='gray' onclick='setModeSafe(${id})'>切模式(安全)</button></div><div class='row'><label>mode <select id='mode_${id}'><option value='1'>MIT</option><option value='2'>PosVel</option><option value='3'>Vel</option><option value='4'>ForcePos</option></select></label><label>kp <input id='kp_${id}' type='number' step='0.01' value='0.5'></label><label>kd <input id='kd_${id}' type='number' step='0.001' value='0.08'></label><button class='gray' onclick='setGains(${id})'>Gains</button></div><div class='row'><label id='posw_${id}'>pos <input id='pos_${id}' type='number' step='0.001' value='0'></label><input id='posr_${id}' type='range' min='-3.14' max='3.14' step='0.001' value='0'><label id='velw_${id}'>vel <input id='vel_${id}' type='number' step='0.01' value='0'></label><label id='tauw_${id}'>tau <input id='tau_${id}' type='number' step='0.01' value='0.3'></label><label id='vlimw_${id}'>vlim <input id='vlim_${id}' type='number' step='0.01' value='1'></label><label id='ratiow_${id}'>ratio <input id='ratio_${id}' type='number' step='0.0001' value='0.1'></label><label><input id='rt_${id}' type='checkbox' checked>实时</label><button class='orange' onclick='apply(${id})'>Apply</button></div><div id='mh_${id}' class='hint'></div><div id='msg_${id}' class='hint'>ready</div></div>`;}"
    "for(let i=1;i<=maxMotors;i++){motors.insertAdjacentHTML('beforeend',motorCard(i));bindCard(i);}"
    "async function enableAll(){const r=await api({action:'enable_all'});showGlobal(r.ok,r.text);}"
    "async function disableAll(){const r=await api({action:'disable_all'});showGlobal(r.ok,r.text);}"
    "async function estopAll(){const r=await api({action:'estop',ids:allIds});showGlobal(r.ok,r.text);}"
    "async function applyVendorAll(){const r=await api({action:'set_vendor',ids:allIds,vendor:document.getElementById('vendor').value});showGlobal(r.ok,r.text);}"
    "function startRecord(){recording=true;recordStartMs=Date.now();actionSeq=[];showGlobal(true,'record started');}"
    "function stopRecord(){recording=false;showGlobal(true,`record stopped frames=${actionSeq.length}`);}"
    "function clearRecord(){recording=false;actionSeq=[];showGlobal(true,'record cleared');}"
    "function setModeInput(id,m){const e=document.getElementById(`mode_${id}`);if(e)e.value=String(m);updateModeUi(id);}"
    "async function playRecord(){if(actionSeq.length===0){showGlobal(false,'record is empty');return;}recording=false;showGlobal(true,`playing ${actionSeq.length} frames`);let prev=0;for(const f of actionSeq){const wait=Math.max(0,f.dt-prev);if(wait>0)await sleep(wait);prev=f.dt;setModeInput(f.id,f.mode);setPosInput(f.id,parseFloat(f.pos));const ve=document.getElementById(`vel_${f.id}`);if(ve)ve.value=f.vel;const te=document.getElementById(`tau_${f.id}`);if(te)te.value=f.tau;const vl=document.getElementById(`vlim_${f.id}`);if(vl)vl.value=f.vlim;const ra=document.getElementById(`ratio_${f.id}`);if(ra)ra.value=f.ratio;await apply(f.id,true);}showGlobal(true,'playback done');}"
    "</script></body></html>";

static int16_t clamp_i16_from_f(float x, float scale)
{
    float v = x * scale;
    if (v > 32767.0f) {
        v = 32767.0f;
    }
    if (v < -32768.0f) {
        v = -32768.0f;
    }
    return (int16_t)v;
}

static uint16_t clamp_u16_from_f(float x, float scale)
{
    float v = x * scale;
    if (v < 0.0f) {
        v = 0.0f;
    }
    if (v > 65535.0f) {
        v = 65535.0f;
    }
    return (uint16_t)v;
}

static bool parse_float(const char *query, const char *key, float *out)
{
    if (query == NULL || out == NULL) {
        return false;
    }
    char tmp[32] = {0};
    if (httpd_query_key_value(query, key, tmp, sizeof(tmp)) != ESP_OK) {
        return false;
    }
    *out = strtof(tmp, NULL);
    return true;
}

static bool parse_vendor_name(const char *query, char *out, size_t out_len)
{
    if (query == NULL || out == NULL || out_len == 0) {
        return false;
    }
    if (httpd_query_key_value(query, "vendor", out, out_len) != ESP_OK) {
        return false;
    }
    return strcmp(out, "damiao") == 0 || strcmp(out, "robstride") == 0;
}

static int parse_ids(const char *csv, uint8_t *ids, int max_ids)
{
    if (csv == NULL || ids == NULL || max_ids <= 0) {
        return 0;
    }

    int count = 0;
    const char *p = csv;
    while (*p != '\0' && count < max_ids) {
        while (*p == ' ' || *p == ',') {
            p++;
        }
        if (!isdigit((unsigned char)*p)) {
            if (*p == '\0') {
                break;
            }
            p++;
            continue;
        }
        long v = strtol(p, (char **)&p, 10);
        if (v >= MOTORBRIDGE_MIN_MOTOR_ID && v <= MOTORBRIDGE_MAX_MOTOR_ID) {
            ids[count++] = (uint8_t)v;
        }
        while (*p != '\0' && *p != ',') {
            p++;
        }
    }
    return count;
}

static void dispatch_admin(uint8_t id, uint8_t op, uint8_t mode)
{
    twai_message_t msg = {0};
    msg.identifier = MOTORBRIDGE_HOST_ADMIN_ID;
    msg.data_length_code = 8;
    msg.data[0] = op;
    msg.data[1] = id;
    msg.data[2] = mode;
    command_router_handle_can_rx(&msg);
}

static void inter_id_delay(int idx, int total)
{
    if (idx < total - 1) {
        vTaskDelay(pdMS_TO_TICKS(INTER_ID_DELAY_MS));
    }
}

static void pace_then_next(int idx, int total)
{
    inter_id_delay(idx, total);
}

static void dispatch_set_gains(uint8_t id, float kp, float kd)
{
    twai_message_t msg = {0};
    if (kp < 0.0f) {
        kp = 0.0f;
    }
    if (kd < 0.0f) {
        kd = 0.0f;
    }

    uint16_t kp_u = (uint16_t)clamp_u16_from_f(kp, 100.0f);
    uint16_t kd_u = (uint16_t)clamp_u16_from_f(kd, 1000.0f);

    msg.identifier = MOTORBRIDGE_HOST_ADMIN_ID;
    msg.data_length_code = 8;
    msg.data[0] = 8; // HOST_OP_SET_GAINS
    msg.data[1] = id;
    msg.data[2] = (uint8_t)(kp_u & 0xFF);
    msg.data[3] = (uint8_t)((kp_u >> 8) & 0xFF);
    msg.data[4] = (uint8_t)(kd_u & 0xFF);
    msg.data[5] = (uint8_t)((kd_u >> 8) & 0xFF);
    command_router_handle_can_rx(&msg);
}

static void dispatch_mode(uint8_t id, uint8_t mode, float pos, float vel, float tau, float vlim, float ratio)
{
    twai_message_t msg = {0};
    msg.data_length_code = 8;
    int16_t a = 0;
    int16_t b = 0;
    uint16_t c = 0;

    switch (mode) {
    case MOTOR_MODE_MIT:
        msg.identifier = MOTORBRIDGE_HOST_MIT_BASE_ID + id;
        a = clamp_i16_from_f(pos, 1000.0f);
        b = clamp_i16_from_f(vel, 100.0f);
        c = (uint16_t)clamp_i16_from_f(tau, 100.0f);
        msg.data[0] = (uint8_t)(a & 0xFF);
        msg.data[1] = (uint8_t)((a >> 8) & 0xFF);
        msg.data[2] = (uint8_t)(b & 0xFF);
        msg.data[3] = (uint8_t)((b >> 8) & 0xFF);
        msg.data[4] = (uint8_t)(c & 0xFF);
        msg.data[5] = (uint8_t)((c >> 8) & 0xFF);
        break;
    case MOTOR_MODE_POS_VEL:
        msg.identifier = MOTORBRIDGE_HOST_POS_VEL_BASE_ID + id;
        a = clamp_i16_from_f(pos, 1000.0f);
        b = clamp_i16_from_f(vlim, 100.0f);
        msg.data[0] = (uint8_t)(a & 0xFF);
        msg.data[1] = (uint8_t)((a >> 8) & 0xFF);
        msg.data[2] = (uint8_t)(b & 0xFF);
        msg.data[3] = (uint8_t)((b >> 8) & 0xFF);
        break;
    case MOTOR_MODE_VEL:
        msg.identifier = MOTORBRIDGE_HOST_VEL_BASE_ID + id;
        a = clamp_i16_from_f(vel, 100.0f);
        msg.data[0] = (uint8_t)(a & 0xFF);
        msg.data[1] = (uint8_t)((a >> 8) & 0xFF);
        break;
    case MOTOR_MODE_FORCE_POS:
        msg.identifier = MOTORBRIDGE_HOST_FORCE_POS_BASE_ID + id;
        a = clamp_i16_from_f(pos, 1000.0f);
        b = clamp_i16_from_f(vlim, 100.0f);
        c = clamp_u16_from_f(ratio, 10000.0f);
        msg.data[0] = (uint8_t)(a & 0xFF);
        msg.data[1] = (uint8_t)((a >> 8) & 0xFF);
        msg.data[2] = (uint8_t)(b & 0xFF);
        msg.data[3] = (uint8_t)((b >> 8) & 0xFF);
        msg.data[4] = (uint8_t)(c & 0xFF);
        msg.data[5] = (uint8_t)((c >> 8) & 0xFF);
        break;
    default:
        return;
    }

    command_router_handle_can_rx(&msg);
}

static void log_action(const char *action, const uint8_t *ids, int n, int mode, float pos, float vel, float tau, float vlim, float ratio)
{
    char ids_buf[96] = {0};
    size_t used = 0;
    for (int i = 0; i < n && used + 6 < sizeof(ids_buf); ++i) {
        int w = snprintf(ids_buf + used, sizeof(ids_buf) - used, "%s%u", (i == 0) ? "" : ",", ids[i]);
        if (w <= 0) {
            break;
        }
        used += (size_t)w;
    }

    if (strcmp(action, "set_gains") == 0) {
        ESP_LOGI(TAG, "web action=set_gains ids=[%s] kp=%.3f kd=%.3f", ids_buf, pos, vel);
        return;
    }
    ESP_LOGI(TAG,
             "web action=%s ids=[%s] mode=%d pos=%.3f vel=%.3f tau=%.3f vlim=%.3f ratio=%.4f",
             action,
             ids_buf,
             mode,
             pos,
             vel,
             tau,
             vlim,
             ratio);
}

static esp_err_t execute_action_from_query(const char *query, char *resp, size_t resp_len)
{
    char action[24] = {0};
    char ids_csv[96] = {0};
    char mode_s[8] = {0};
    uint8_t ids[32] = {0};

    if (httpd_query_key_value(query, "action", action, sizeof(action)) != ESP_OK) {
        snprintf(resp, resp_len, "err missing action");
        return ESP_FAIL;
    }
    if (strcmp(action, "enable_all") == 0) {
        dispatch_admin(0, 4, 0);
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok enable_all");
        return ESP_OK;
    }
    if (strcmp(action, "disable_all") == 0) {
        dispatch_admin(0, 5, 0);
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok disable_all");
        return ESP_OK;
    }
    if (httpd_query_key_value(query, "ids", ids_csv, sizeof(ids_csv)) != ESP_OK) {
        snprintf(resp, resp_len, "err missing ids");
        return ESP_FAIL;
    }
    if (query != NULL) {
        int64_t now_us = esp_timer_get_time();
        if (s_last_query_us > 0 &&
            (now_us - s_last_query_us) < WEB_DUPLICATE_GUARD_US &&
            strncmp(query, s_last_query, sizeof(s_last_query)) == 0) {
            snprintf(resp, resp_len, "ok duplicate ignored");
            return ESP_OK;
        }
    }

    int n = parse_ids(ids_csv, ids, (int)(sizeof(ids) / sizeof(ids[0])));
    const app_config_t *cfg = app_config_get();
    int max_id = cfg->max_motors;
    int filtered = 0;
    for (int i = 0; i < n; ++i) {
        if (ids[i] <= max_id) {
            ids[filtered++] = ids[i];
        }
    }
    n = filtered;
    if (n <= 0 && strcmp(action, "estop") != 0) {
        snprintf(resp, resp_len, "err invalid ids");
        return ESP_FAIL;
    }

    if (strcmp(action, "enable") == 0) {
        log_action(action, ids, n, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            dispatch_admin(ids[i], 4, 0);
            pace_then_next(i, n);
        }
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok enable count=%d", n);
        return ESP_OK;
    }
    if (strcmp(action, "disable") == 0) {
        log_action(action, ids, n, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            dispatch_admin(ids[i], 5, 0);
            pace_then_next(i, n);
        }
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok disable count=%d", n);
        return ESP_OK;
    }
    if (strcmp(action, "clear_error") == 0) {
        log_action(action, ids, n, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            dispatch_admin(ids[i], 7, 0);
            pace_then_next(i, n);
        }
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok clear_error count=%d", n);
        return ESP_OK;
    }
    if (strcmp(action, "set_zero") == 0) {
        log_action(action, ids, n, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            dispatch_admin(ids[i], 6, 0);
            pace_then_next(i, n);
        }
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok set_zero count=%d", n);
        return ESP_OK;
    }
    if (strcmp(action, "estop") == 0) {
        log_action(action, ids, n, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        dispatch_admin(0, 2, 0);
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok estop");
        return ESP_OK;
    }
    if (strcmp(action, "state") == 0) {
        uint8_t id = ids[0];
        motor_state_t m;
        if (!motor_manager_get_state(id, &m)) {
            snprintf(resp, resp_len, "err state not found");
            return ESP_FAIL;
        }
        snprintf(resp,
                 resp_len,
                 "ok state id=%u pos=%.5f vel=%.5f online=%d mode=%d",
                 (unsigned int)id,
                 m.position,
                 m.speed,
                 m.online ? 1 : 0,
                 (int)m.mode);
        return ESP_OK;
    }

    if (strcmp(action, "set_gains") == 0) {
        float kp = 0.0f;
        float kd = 0.0f;
        (void)parse_float(query, "kp", &kp);
        (void)parse_float(query, "kd", &kd);
        log_action(action, ids, n, 0, kp, kd, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            dispatch_set_gains(ids[i], kp, kd);
            pace_then_next(i, n);
        }
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok set_gains count=%d", n);
        return ESP_OK;
    }
    if (strcmp(action, "set_vendor") == 0) {
        char vendor[16] = {0};
        if (!parse_vendor_name(query, vendor, sizeof(vendor))) {
            snprintf(resp, resp_len, "err bad vendor");
            return ESP_FAIL;
        }
        int ok_count = 0;
        for (int i = 0; i < n; ++i) {
            if (motor_manager_set_vendor(ids[i], vendor)) {
                ok_count++;
            }
            pace_then_next(i, n);
        }
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok set_vendor=%s count=%d", vendor, ok_count);
        return ESP_OK;
    }

    if (httpd_query_key_value(query, "mode", mode_s, sizeof(mode_s)) != ESP_OK) {
        snprintf(resp, resp_len, "err missing mode");
        return ESP_FAIL;
    }
    int mode = atoi(mode_s);
    if (mode < MOTOR_MODE_MIT || mode > MOTOR_MODE_FORCE_POS) {
        snprintf(resp, resp_len, "err bad mode");
        return ESP_FAIL;
    }

    if (strcmp(action, "set_mode") == 0) {
        log_action(action, ids, n, mode, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            dispatch_admin(ids[i], 1, (uint8_t)mode);
            pace_then_next(i, n);
        }
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok set_mode count=%d", n);
        return ESP_OK;
    }

    if (strcmp(action, "apply") == 0) {
        float pos = 0.0f;
        float vel = 0.0f;
        float tau = 0.0f;
        float vlim = 0.0f;
        float ratio = 0.1f;
        (void)parse_float(query, "pos", &pos);
        (void)parse_float(query, "vel", &vel);
        (void)parse_float(query, "tau", &tau);
        (void)parse_float(query, "vlim", &vlim);
        (void)parse_float(query, "ratio", &ratio);
        if ((mode == MOTOR_MODE_POS_VEL || mode == MOTOR_MODE_FORCE_POS) && vlim <= 0.0f) {
            snprintf(resp, resp_len, "err vlim must be > 0 in this mode");
            return ESP_FAIL;
        }
        log_action(action, ids, n, mode, pos, vel, tau, vlim, ratio);

        for (int i = 0; i < n; ++i) {
            dispatch_mode(ids[i], (uint8_t)mode, pos, vel, tau, vlim, ratio);
            pace_then_next(i, n);
        }
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok apply count=%d", n);
        return ESP_OK;
    }

    snprintf(resp, resp_len, "err unknown action");
    return ESP_FAIL;
}

static esp_err_t root_get_handler(httpd_req_t *req)
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

static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t control_get_handler(httpd_req_t *req)
{
    char query[256] = {0};
    char resp[64] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing query");
        return ESP_FAIL;
    }
    esp_err_t err = execute_action_from_query(query, resp, sizeof(resp));
    if (err == ESP_OK) {
        httpd_resp_sendstr(req, resp);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, resp);
    }
    return err;
}

#if CONFIG_HTTPD_WS_SUPPORT
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "ws handshake complete");
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t *payload = calloc(1, frame.len + 1);
    if (payload == NULL) {
        return ESP_ERR_NO_MEM;
    }
    frame.payload = payload;
    err = httpd_ws_recv_frame(req, &frame, frame.len);
    if (err != ESP_OK) {
        free(payload);
        return err;
    }
    payload[frame.len] = '\0';

    char resp[64] = {0};
    (void)execute_action_from_query((char *)payload, resp, sizeof(resp));
    free(payload);

    httpd_ws_frame_t out = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)resp,
        .len = strlen(resp),
    };
    return httpd_ws_send_frame(req, &out);
}
#endif

esp_err_t web_control_start(void)
{
    if (s_server != NULL) {
        return ESP_OK;
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 8;
    cfg.server_port = 80;

    ESP_LOGI(TAG, "starting web control http://192.168.4.1/");
    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        return ESP_FAIL;
    }

    const httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t control = {
        .uri = "/api/control",
        .method = HTTP_GET,
        .handler = control_get_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t favicon = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = favicon_get_handler,
        .user_ctx = NULL,
    };
#if CONFIG_HTTPD_WS_SUPPORT
    const httpd_uri_t ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
    };
#endif

    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &control);
    httpd_register_uri_handler(s_server, &favicon);
#if CONFIG_HTTPD_WS_SUPPORT
    httpd_register_uri_handler(s_server, &ws);
#else
    ESP_LOGW(TAG, "WebSocket disabled in sdkconfig, fallback to HTTP GET control");
#endif
    return ESP_OK;
}
