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

static const char *TAG = "web_control";
static httpd_handle_t s_server = NULL;
static char s_last_query[256];
static int64_t s_last_query_us = 0;
static const int64_t WEB_DUPLICATE_GUARD_US = 800000;
static const int INTER_ID_DELAY_MS = 10;
static const int FEEDBACK_WAIT_MS = 80;

static const char *INDEX_HTML =
    "<!doctype html><html><head><meta charset='utf-8'/>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
    "<title>ESP MotorBridge</title>"
    "<style>"
    "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;padding:12px;background:#f4f7fb;color:#1f2937}"
    ".card{background:#fff;border-radius:12px;padding:14px;box-shadow:0 4px 20px rgba(0,0,0,.08);margin-bottom:12px}"
    "h1{font-size:20px;margin:0 0 8px}h2{font-size:16px;margin:0 0 8px}"
    ".row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}.row>*{margin:4px 0}"
    "button{border:none;padding:10px 12px;border-radius:10px;background:#0f766e;color:#fff;font-weight:600}"
    "button.gray{background:#475569}button.red{background:#b91c1c}"
    "input,select{padding:8px;border-radius:8px;border:1px solid #cbd5e1;min-width:96px}"
    "input[type=range]{padding:0;min-width:180px;vertical-align:middle}"
    ".motors{display:grid;grid-template-columns:repeat(4,minmax(56px,1fr));gap:8px}"
    ".hint{font-size:12px;color:#64748b}.ok{color:#0f766e}.err{color:#b91c1c}"
    ".hidden{display:none}"
    "</style></head><body>"
    "<div class='card'><h1>ESP MotorBridge 控制台</h1>"
    "<div class='hint'>AP: ESP_motorbridge / 12345678, UDP:9000, 网页 API: /api/control</div></div>"
    "<div class='card'><h2>电机选择</h2><div id='motors' class='motors'></div>"
    "<div class='row'><button onclick='allSel(true)'>全选</button><button class='gray' onclick='allSel(false)'>全不选</button></div></div>"
    "<div class='card'><h2>使能与模式</h2>"
    "<div class='row'>"
    "<button onclick=\"admin('enable')\">Enable</button>"
    "<button class='red' onclick=\"admin('disable')\">Disable</button>"
    "<button class='red' onclick=\"admin('estop')\">E-Stop</button>"
    "<select id='mode'><option value='1'>MIT</option><option value='2'>PosVel</option><option value='3'>Vel</option><option value='4'>ForcePos</option></select>"
    "<button class='gray' onclick='setMode()'>设模式</button>"
    "<label>kp <input id='kp' type='number' step='0.01' value='0.5'></label>"
    "<label>kd <input id='kd' type='number' step='0.001' value='0.08'></label>"
    "<button class='gray' onclick='setGains()'>设增益</button>"
    "</div></div>"
    "<div class='card'><h2>期望值</h2>"
    "<div class='row'>"
    "<label id='f_pos'>pos <input id='pos' type='number' step='0.001' value='0'>"
    "<input id='pos_r' type='range' min='-3.14' max='3.14' step='0.001' value='0'></label>"
    "<label id='f_vel'>vel <input id='vel' type='number' step='0.01' value='0'></label>"
    "<label id='f_tau'>tau <input id='tau' type='number' step='0.01' value='0.3'></label>"
    "<label id='f_vlim'>vlim <input id='vlim' type='number' step='0.01' value='1'></label>"
    "<label id='f_ratio'>ratio <input id='ratio' type='number' step='0.0001' value='0.1'></label>"
    "<label><input id='realtime' type='checkbox'> 实时下发</label>"
    "<button onclick='apply()'>下发控制</button>"
    "</div>"
    "<div id='modeHint' class='hint'></div>"
    "<div id='msg' class='hint'></div>"
    "</div>"
    "<script>"
    "const maxMotors=7;"
    "let ws=null;"
    "let inFlight=false;"
    "let lastSendTs=0;"
    "let rtTimer=null;"
    "const motors=document.getElementById('motors');"
    "for(let i=1;i<=maxMotors;i++){const d=document.createElement('label');d.innerHTML=`<input type='checkbox' value='${i}' checked> M${i}`;motors.appendChild(d);}"
    "function ids(){return [...document.querySelectorAll('#motors input:checked')].map(x=>x.value).join(',');}"
    "function allSel(v){document.querySelectorAll('#motors input').forEach(x=>x.checked=v);}"
    "function show(ok,t){const m=document.getElementById('msg');m.className=ok?'hint ok':'hint err';m.textContent=t;}"
    "function setHidden(id,h){const e=document.getElementById(id);if(e){e.classList.toggle('hidden',h);}}"
    "function maybeRealtime(){if(!document.getElementById('realtime').checked)return;clearTimeout(rtTimer);rtTimer=setTimeout(()=>apply(),120);}"
    "function bindRealtimeInputs(){['pos','vel','tau','vlim','ratio'].forEach(k=>{const n=document.getElementById(k);if(n)n.addEventListener('input',maybeRealtime);});}"
    "function bindPosSlider(){const p=document.getElementById('pos'),r=document.getElementById('pos_r');if(!p||!r)return;"
    "p.addEventListener('input',()=>{r.value=p.value;});"
    "r.addEventListener('input',()=>{p.value=r.value;maybeRealtime();});}"
    "function updateModeFields(){"
    "const m=parseInt(document.getElementById('mode').value||'1',10);"
    "const vPos=document.getElementById('pos');"
    "const vVel=document.getElementById('vel');"
    "const vTau=document.getElementById('tau');"
    "const vVlim=document.getElementById('vlim');"
    "const vRatio=document.getElementById('ratio');"
    "setHidden('f_pos',!(m===1||m===2||m===4));"
    "setHidden('f_vel',!(m===1||m===3));"
    "setHidden('f_tau',!(m===1));"
    "setHidden('f_vlim',!(m===2||m===4));"
    "setHidden('f_ratio',!(m===4));"
    "const h=document.getElementById('modeHint');"
    "if(m===1){h.textContent='MIT: pos[-3.14,3.14] / vel[0,1] / tau(默认0.3)';"
    "vPos.min='-3.14';vPos.max='3.14';vPos.step='0.001';"
    "document.getElementById('pos_r').min='-3.14';document.getElementById('pos_r').max='3.14';document.getElementById('pos_r').step='0.001';"
    "vVel.min='0';vVel.max='1';vVel.step='0.01';"
    "vTau.min='0';vTau.max='10';vTau.step='0.01';if(vTau.value==='0')vTau.value='0.3';"
    "if(vPos.value==='')vPos.value='0';if(vVel.value==='')vVel.value='0';}"
    "else if(m===2){h.textContent='PosVel: pos[-3.14,3.14] / vlim[0,5]';"
    "vPos.min='-3.14';vPos.max='3.14';vPos.step='0.001';"
    "document.getElementById('pos_r').min='-3.14';document.getElementById('pos_r').max='3.14';document.getElementById('pos_r').step='0.001';"
    "vVlim.min='0';vVlim.max='5';vVlim.step='0.01';if(vVlim.value==='')vVlim.value='1';if(vPos.value==='')vPos.value='0';}"
    "else if(m===3){h.textContent='Vel: vel[-4,4]';"
    "vVel.min='-4';vVel.max='4';vVel.step='0.01';if(vVel.value==='')vVel.value='0';}"
    "else if(m===4){h.textContent='ForcePos: pos[-3.14,3.14] / vlim / ratio';"
    "vPos.min='-3.14';vPos.max='3.14';vPos.step='0.001';if(vPos.value==='')vPos.value='0';"
    "document.getElementById('pos_r').min='-3.14';document.getElementById('pos_r').max='3.14';document.getElementById('pos_r').step='0.001';"
    "if(parseFloat(vVlim.value||'0')<=0)vVlim.value='2.0';if(parseFloat(vRatio.value||'0')<=0)vRatio.value='0.1';}"
    "if(vPos.value==='')vPos.value='0';"
    "document.getElementById('pos_r').value=vPos.value;"
    "}"
    "function startWs(){"
    "ws=new WebSocket(`ws://${location.host}/ws`);"
    "ws.onopen=()=>show(true,'WS connected');"
    "ws.onmessage=(e)=>show((e.data||'').startsWith('ok'),e.data||'');"
    "ws.onclose=()=>setTimeout(startWs,1000);"
    "ws.onerror=()=>{};"
    "}"
    "startWs();"
    "async function call(q){"
    "const now=Date.now();"
    "if(inFlight){show(false,'busy: previous command running');return;}"
    "if(now-lastSendTs<180){show(false,'rate limited: wait a moment');return;}"
    "inFlight=true;lastSendTs=now;"
    "try{"
    "if(ws&&ws.readyState===1){ws.send(q);show(true,'sent via ws');return;}"
    "const r=await fetch('/api/control?'+q);const t=await r.text();show(r.ok&&t.startsWith('ok'),t);"
    "}catch(e){show(false,'network error');}"
    "finally{setTimeout(()=>{inFlight=false;},180);}"
    "}"
    "function admin(a){const q=`action=${a}&ids=${ids()}`;call(q);}"
    "function setMode(){const q=`action=set_mode&ids=${ids()}&mode=${document.getElementById('mode').value}`;call(q);}"
    "function setGains(){const q=`action=set_gains&ids=${ids()}&kp=${document.getElementById('kp').value}&kd=${document.getElementById('kd').value}`;call(q);}"
    "function apply(){const mode=parseInt(document.getElementById('mode').value||'1',10);"
    "const vlim=parseFloat(document.getElementById('vlim').value||'0');"
    "if((mode===2||mode===4)&&vlim<=0){show(false,'vlim must be > 0 for this mode');return;}"
    "const q=`action=apply&ids=${ids()}&mode=${document.getElementById('mode').value}`+"
    "`&pos=${document.getElementById('pos').value}&vel=${document.getElementById('vel').value}`+"
    "`&tau=${document.getElementById('tau').value}&vlim=${document.getElementById('vlim').value}&ratio=${document.getElementById('ratio').value}`;"
    "call(q);}"
    "document.getElementById('mode').addEventListener('change',updateModeFields);"
    "bindRealtimeInputs();"
    "bindPosSlider();"
    "updateModeFields();"
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

static bool wait_feedback_then_next(uint8_t id, int idx, int total)
{
    bool got_feedback = command_router_wait_feedback(id, FEEDBACK_WAIT_MS);
    if (!got_feedback) {
        ESP_LOGW(TAG, "no feedback from id=%u within %dms, stop current batch", id, FEEDBACK_WAIT_MS);
    }
    inter_id_delay(idx, total);
    return got_feedback;
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
        if (ids[i] >= MOTORBRIDGE_MIN_MOTOR_ID && ids[i] <= max_id) {
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
            if (!wait_feedback_then_next(ids[i], i, n)) {
                snprintf(resp, resp_len, "err no feedback id=%u", ids[i]);
                return ESP_FAIL;
            }
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
            if (!wait_feedback_then_next(ids[i], i, n)) {
                snprintf(resp, resp_len, "err no feedback id=%u", ids[i]);
                return ESP_FAIL;
            }
        }
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok disable count=%d", n);
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

    if (strcmp(action, "set_gains") == 0) {
        float kp = 0.0f;
        float kd = 0.0f;
        (void)parse_float(query, "kp", &kp);
        (void)parse_float(query, "kd", &kd);
        log_action(action, ids, n, 0, kp, kd, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            dispatch_set_gains(ids[i], kp, kd);
            if (!wait_feedback_then_next(ids[i], i, n)) {
                snprintf(resp, resp_len, "err no feedback id=%u", ids[i]);
                return ESP_FAIL;
            }
        }
        strlcpy(s_last_query, query, sizeof(s_last_query));
        s_last_query_us = esp_timer_get_time();
        snprintf(resp, resp_len, "ok set_gains count=%d", n);
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
            if (!wait_feedback_then_next(ids[i], i, n)) {
                snprintf(resp, resp_len, "err no feedback id=%u", ids[i]);
                return ESP_FAIL;
            }
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
            if (!wait_feedback_then_next(ids[i], i, n)) {
                snprintf(resp, resp_len, "err no feedback id=%u", ids[i]);
                return ESP_FAIL;
            }
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
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
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
