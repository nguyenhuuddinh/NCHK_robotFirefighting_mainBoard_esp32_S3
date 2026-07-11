#include "TaskManager.h"
#include "RobotConfig.h"
#include "RobotMaster.h"
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

extern RobotMaster robotMaster;

// ============================================================
// WEB SERVER (Tam giu de test - se xoa o Phase 4)
// ============================================================
static WebServer server(80);

static const char* html_page = R"HTML(
<!DOCTYPE html><html>
<head>
    <meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
    <meta charset="utf-8"><title>Robot Control</title>
    <style>
        *{box-sizing:border-box;margin:0;padding:0}
        body{font-family:Arial;text-align:center;padding:20px;background:#0d1117;color:#e6edf3;touch-action:manipulation}
        h2{color:#58a6ff;margin-bottom:5px}
        p{color:#8b949e;font-size:13px;margin-bottom:12px}
        .card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:15px;margin:10px auto;max-width:380px}
        .row{display:flex;justify-content:center;gap:10px;margin:8px 0}
        .btn{padding:22px 10px;font-size:18px;font-weight:bold;color:#fff;border:none;border-radius:8px;cursor:pointer;flex:1;max-width:160px;transition:filter .08s;-webkit-user-select:none;user-select:none}
        .btn.on{filter:brightness(1.6) drop-shadow(0 0 8px #fff5)}
        .fwd{background:#238636}.rev{background:#da3633}.lt{background:#1f6feb}.rt{background:#9e6a03}
        .stop{background:#6e7681;width:100%;padding:18px;font-size:20px;margin-top:5px;border-radius:8px;border:none;color:#fff;font-weight:bold;cursor:pointer}
        .status{margin-top:10px;padding:7px;border-radius:6px;background:#0d1117;border:1px solid #30363d;font-size:13px;color:#3fb950}
        .info{font-size:11px;color:#6e7681;margin-top:8px}
    </style>
    <script>
        var _iv=null,_vx=0,_wz=0,_btn=null;
        function cmd(vx,wz){fetch('/cmd?vx='+vx+'&wz='+wz).catch(function(){});}
        function startCmd(el,vx,wz,lbl){
            if(_iv)return;
            _vx=vx;_wz=wz;_btn=el;
            el.classList.add('on');
            document.getElementById('st').innerText='>> '+lbl;
            cmd(vx,wz);
            _iv=setInterval(function(){cmd(_vx,_wz);},200);
        }
        function stopCmd(){
            if(_iv){clearInterval(_iv);_iv=null;}
            if(_btn){_btn.classList.remove('on');_btn=null;}
            document.getElementById('st').innerText='-- DUNG';
            cmd(0,0);
        }
        document.addEventListener('contextmenu',function(e){e.preventDefault();});
    </script>
</head>
<body>
    <h2>DIEU KHIEN ROBOT</h2>
    <p>GIU NUT = chay lien tuc | THA = dung ngay</p>
    <div class="card">
        <div class="row">
            <button id="bf" class="btn fwd"
                onmousedown="startCmd(this,0.5,0,'TIEN')" ontouchstart="startCmd(this,0.5,0,'TIEN')"
                onmouseup="stopCmd()" onmouseleave="stopCmd()" ontouchend="stopCmd()">&#9650; TIEN</button>
        </div>
        <div class="row">
            <button id="bl" class="btn lt"
                onmousedown="startCmd(this,0,8,'TRAI')" ontouchstart="startCmd(this,0,8,'TRAI')"
                onmouseup="stopCmd()" onmouseleave="stopCmd()" ontouchend="stopCmd()">&#9668; TRAI</button>
            <button id="br" class="btn rt"
                onmousedown="startCmd(this,0,-8,'PHAI')" ontouchstart="startCmd(this,0,-8,'PHAI')"
                onmouseup="stopCmd()" onmouseleave="stopCmd()" ontouchend="stopCmd()">PHAI &#9658;</button>
        </div>
        <div class="row">
            <button id="bb" class="btn rev"
                onmousedown="startCmd(this,-0.5,0,'LUI')" ontouchstart="startCmd(this,-0.5,0,'LUI')"
                onmouseup="stopCmd()" onmouseleave="stopCmd()" ontouchend="stopCmd()">&#9660; LUI</button>
        </div>
        <button class="stop" onclick="stopCmd()">&#9940; DUNG KHAN CAP</button>
        <div class="status" id="st">-- DUNG</div>
        <p class="info" style="margin-top:12px">Lenh gui 200ms/lan khi giu | Watchdog dung sau 1000ms</p>
    </div>
</body></html>
)HTML";

// Bien static luu context — set trong WebControl_init
static SharedContext* _webCtx = nullptr;

static void handleRoot() { server.send(200, "text/html", html_page); }

static void handleCmd() {
    if (_webCtx == nullptr) { server.send(500, "text/plain", "ERR"); return; }

    float vx = 0.0f, wz = 0.0f;
    if (server.hasArg("vx")) vx = server.arg("vx").toFloat();
    if (server.hasArg("wz")) wz = server.arg("wz").toFloat();

    xSemaphoreTake(_webCtx->cmdMutex, portMAX_DELAY);
    _webCtx->cmdVel.target_vx = vx;
    _webCtx->cmdVel.target_wz = wz;
    xSemaphoreGive(_webCtx->cmdMutex);

    // Thong bao cho RobotMaster: da nhan lenh (reset watchdog)
    if (vx != 0.0f || wz != 0.0f) {
        robotMaster.notifyCmdReceived();
    }

    server.send(200, "text/plain", "OK");
}

// ============================================================
// WEBCONTROL INIT — Goi truoc khi tao Task
// (Tam giu de test — se xoa o Phase 4.10)
// ============================================================
void WebControl_init(SharedContext* ctx) {
    _webCtx = ctx;

    WiFi.softAP("ESP32_Robot", "12345678");
    DBG.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

    server.on("/", handleRoot);
    server.on("/cmd", handleCmd);
    server.begin();
    DBG.println("[OK] WebServer started on port 80");
}

// ============================================================
// TASK WEB SERVER (Core 0)
// ============================================================
void Task_WebServer(void* pvParam) {
    SharedContext* ctx = (SharedContext*)pvParam;

    // Init WiFi + Web Server trong context cua Task (Core 0)
    WebControl_init(ctx);

    for (;;) {
        server.handleClient();

        // Cap nhat RobotMaster state machine
        robotMaster.update();

        vTaskDelay(pdMS_TO_TICKS(5)); // Nhuong CPU
    }
}
