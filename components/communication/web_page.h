#pragma once

/* Embedded single-page web frontend for drone control & telemetry */
#define WEB_PAGE_HTML \
"<!DOCTYPE html>\n" \
"<html lang=\"zh\">\n" \
"<head>\n" \
"<meta charset=\"UTF-8\">\n" \
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1,user-scalable=no\">\n" \
"<title>Drone Control</title>\n" \
"<style>\n" \
"*{margin:0;padding:0;box-sizing:border-box}\n" \
"body{background:#0f1117;color:#c9d1d9;font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;overflow-x:hidden;touch-action:manipulation}\n" \
".top-bar{display:flex;justify-content:space-between;align-items:center;padding:8px 12px;background:#161b22;border-bottom:1px solid #30363d}\n" \
".top-bar .title{font-size:16px;font-weight:700;color:#58a6ff}\n" \
".top-bar .status{font-size:12px}\n" \
".status-ok{color:#3fb950} .status-err{color:#f85149}\n" \
".container{display:flex;flex-wrap:wrap;gap:8px;padding:8px;max-width:800px;margin:0 auto}\n" \
".panel{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:8px 10px}\n" \
".panel h3{font-size:12px;color:#8b949e;margin-bottom:4px;text-transform:uppercase;letter-spacing:1px}\n" \
".data-row{display:flex;justify-content:space-between;font-size:13px;padding:1px 0}\n" \
".data-row span:last-child{font-family:monospace;color:#7ee787}\n" \
".sensors{flex:1;min-width:200px;display:flex;flex-direction:column;gap:6px}\n" \
".controls{flex:1;min-width:280px;display:flex;flex-direction:column;gap:8px;align-items:center}\n" \
".joy-area{display:flex;gap:16px;align-items:center}\n" \
".joy-box{text-align:center}\n" \
".joy-box label{font-size:11px;color:#8b949e}\n" \
"canvas{border:1px solid #30363d;border-radius:50%;background:#21262d;touch-action:none}\n" \
".yaw-box{display:flex;flex-direction:column;align-items:center;gap:4px}\n" \
".yaw-box label{font-size:11px;color:#8b949e}\n" \
"#yaw-slider,#throttle-slider{-webkit-appearance:none;height:6px;border-radius:3px;background:#30363d;outline:none}\n" \
"#yaw-slider{width:200px}\n" \
"#throttle-slider{width:140px;margin-top:60px}\n" \
"#yaw-slider::-webkit-slider-thumb,#throttle-slider::-webkit-slider-thumb{-webkit-appearance:none;width:28px;height:28px;border-radius:50%;background:#58a6ff;cursor:pointer}\n" \
".mode-row{display:flex;gap:6px;align-items:center;margin-top:4px}\n" \
".mode-row label{font-size:12px;color:#8b949e}\n" \
".mode-row select{background:#21262d;color:#c9d1d9;border:1px solid #30363d;border-radius:4px;padding:4px 8px;font-size:13px}\n" \
".disarmed{color:#f85149!important;font-weight:700}\n" \
"button{background:#21262d;color:#c9d1d9;border:1px solid #30363d;border-radius:4px;padding:6px 14px;font-size:13px;cursor:pointer}\n" \
"button:active{opacity:0.7}\n" \
".toast{position:fixed;top:50%;left:50%;transform:translate(-50%,-50%);background:#238636;color:#fff;padding:10px 24px;border-radius:8px;font-size:14px;z-index:999;pointer-events:none;opacity:0;transition:opacity 0.3s}\n" \
".toast.show{opacity:1}\n" \
".toast.warn{background:#d2991d}\n" \
".trim-val{font-size:11px;color:#8b949e;margin-left:4px}\n" \
".btn-danger{background:#3a1c1c;border-color:#f85149;color:#f85149}\n" \
".btn-danger:disabled{opacity:0.4}\n" \
".mode-btns{display:flex;gap:4px;flex-wrap:wrap;justify-content:center;margin-top:4px}\n" \
".mode-btn{flex:1;min-width:60px;padding:6px 4px;font-size:12px;border:1px solid #30363d;background:#21262d;color:#8b949e;border-radius:4px;cursor:pointer}\n" \
".mode-btn.active{background:#238636;color:#fff;border-color:#238636}\n" \
".mode-btn.disarm-btn.active{background:#f85149;border-color:#f85149}\n" \
".dpad-empty{visibility:hidden}\n" \
".move-cross{display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px;width:150px;margin:8px auto}\n" \
".move-btn{padding:10px;font-size:20px;background:#21262d;border:1px solid #30363d;border-radius:6px;color:#c9d1d9;cursor:pointer;-webkit-user-select:none;user-select:none;transition:background 0.1s}\n" \
".move-btn:active{background:#1f6feb;color:#fff}\n" \
".move-btn.pressed{background:#1f6feb;color:#fff;border-color:#58a6ff}\n" \
".move-stop-btn{padding:8px;font-size:12px;background:#3a1c1c;border:1px solid #f85149;border-radius:6px;color:#f85149;cursor:pointer;font-weight:700}\n" \
"</style>\n" \
"</head>\n" \
"<body>\n" \
"<div id=\"toast\" class=\"toast\"></div>\n" \
"<div class=\"top-bar\">\n" \
"<span class=\"title\">Drone</span>\n" \
"<span id=\"conn-status\" class=\"status status-err\">Disconnected</span>\n" \
"<span id=\"mode-display\" class=\"disarmed\">DISARMED</span>\n" \
"</div>\n" \
"<div class=\"container\">\n" \
"<div class=\"sensors\">\n" \
"<div class=\"panel\"><h3>Attitude <span class=\"trim-val\">Trim:<span id=\"trim-roll\">0.0</span>/<span id=\"trim-pitch\">0.0</span>°</span></h3>\n" \
"<div class=\"data-row\"><span>Roll</span><span id=\"att-roll\">0.0°</span></div>\n" \
"<div class=\"data-row\"><span>Pitch</span><span id=\"att-pitch\">0.0°</span></div>\n" \
"<div class=\"data-row\"><span>Yaw</span><span id=\"att-yaw\">0.0°</span></div>\n" \
"</div>\n" \
"<div class=\"panel\"><h3>TOF Altitude</h3>\n" \
"<div class=\"data-row\"><span>Distance</span><span id=\"tof-val\">-- mm</span></div>\n" \
"<div class=\"data-row\"><span>Target</span><span id=\"alt-target\">0.00 m</span></div>\n" \
"<div class=\"data-row\"><span>Vz</span><span id=\"alt-vz\">0.00 m/s</span></div>\n" \
"</div>\n" \
"<div class=\"panel\"><h3>Magnetometer (BN-880)</h3>\n" \
"<div class=\"data-row\"><span>Mag X/Y/Z (G)</span><span id=\"mag-xyz\">--</span></div>\n" \
"<div class=\"data-row\"><span>Heading (未标定)</span><span id=\"mag-hdg\">--</span></div>\n" \
"</div>\n" \
"<div class=\"panel\"><h3>Optical Flow Position</h3>\n" \
"<div class=\"data-row\"><span>X (m)</span><span id=\"flow-x\">0.00</span></div>\n" \
"<div class=\"data-row\"><span>Y (m)</span><span id=\"flow-y\">0.00</span></div>\n" \
"<div class=\"data-row\"><span>Qual / 融合权重</span><span><span id=\"flow-qual\">0</span> / <span id=\"flow-qg\">0.00</span></span></div>\n" \
"<div class=\"data-row\"><span>Vel vx/vy (m/s)</span><span id=\"flow-vel\">0.00 / 0.00</span></div>\n" \
"<div class=\"data-row\"><span>Pos State</span><span id=\"flow-ps\">idle</span></div>\n" \
"<div class=\"data-row\"><span>Pos Err X/Y (m)</span><span id=\"flow-perr\">-- / --</span></div>\n" \
"<div class=\"data-row\"><span>标定模式 (锁定手持标定)</span><span><button id=\"fc-btn\" onclick=\"toggleFlowCalib()\">OFF</button></span></div>\n" \
"</div>\n" \
"</div>\n" \
"<div class=\"controls\">\n" \
"<div class=\"joy-area\">\n" \
"<div class=\"joy-box\">\n" \
"<label>Throttle <span id=\"throttle-val\">0%</span></label><br>\n" \
"<input type=\"range\" id=\"throttle-slider\" min=\"0\" max=\"1\" step=\"0.01\" value=\"0\">\n" \
"</div>\n" \
"<div class=\"joy-box\">\n" \
"<label>Roll / Pitch</label><br>\n" \
"<canvas id=\"joy-rp\" width=\"140\" height=\"140\"></canvas>\n" \
"</div>\n" \
"</div>\n" \
"<div class=\"yaw-box\">\n" \
"<label>Yaw <span id=\"yaw-val\">0.00</span></label>\n" \
"<input type=\"range\" id=\"yaw-slider\" min=\"-1\" max=\"1\" step=\"0.01\" value=\"0\">\n" \
"</div>\n" \
"<div class=\"mode-btns\">\n" \
"<button class=\"mode-btn disarm-btn active\" data-mode=\"disarmed\">锁定</button>\n" \
"<button class=\"mode-btn\" data-mode=\"stabilize\">自稳</button>\n" \
"<button class=\"mode-btn\" data-mode=\"alt_hold\">定高</button>\n" \
"<button class=\"mode-btn\" data-mode=\"pos_hold\">悬停</button>\n" \
"</div>\n" \
"<div class=\"panel\" style=\"margin-top:6px;width:100%\">\n" \
"<h3>自动起飞</h3>\n" \
"<div style=\"margin:4px 0\"><label style=\"font-size:12px\">目标高度: <span id=\"takeoff-h-val\">0.5</span> m</label><br>\n" \
"<input type=\"range\" id=\"takeoff-h\" min=\"0.2\" max=\"2.0\" step=\"0.1\" value=\"0.5\" style=\"width:100%\"></div>\n" \
"<div style=\"margin:4px 0\"><label style=\"font-size:12px\">基准油门: <span id=\"takeoff-t-val\">0.40</span></label><br>\n" \
"<input type=\"range\" id=\"takeoff-t\" min=\"0.25\" max=\"0.6\" step=\"0.01\" value=\"0.4\" style=\"width:100%\"></div>\n" \
"<button id=\"takeoff-btn\" style=\"width:100%;margin-top:4px;padding:8px;font-size:14px;background:#238636;border-color:#2ea043;color:#fff\" onclick=\"doTakeoff()\" ontouchstart=\"event.preventDefault();doTakeoff()\">起飞</button>\n" \
"</div>\n" \
"<div style=\"display:flex;justify-content:center;margin-top:6px;gap:8px\">\n" \
"<button onclick=\"sendCmd('gyro_calib')\" ontouchstart=\"event.preventDefault();sendCmd('gyro_calib')\">陀螺仪校准</button>\n" \
"<button onclick=\"sendCmd('level_trim')\" ontouchstart=\"event.preventDefault();sendCmd('level_trim')\">水平校准</button>\n" \
"<button onclick=\"sendCmd('reset_trim')\" ontouchstart=\"event.preventDefault();sendCmd('reset_trim')\">重置水平</button>\n" \
"</div>\n" \
"<div style=\"text-align:center;margin-top:8px\"><label style=\"font-size:11px;color:#8b949e\">水平移动 (按住移动,松开停止)</label></div>\n" \
"<div class=\"move-cross\" id=\"move-cross\">\n" \
"<div class=\"dpad-empty\"></div>\n" \
"<button class=\"move-btn\" id=\"move-fwd\">▲前</button>\n" \
"<div class=\"dpad-empty\"></div>\n" \
"<button class=\"move-btn\" id=\"move-left\">◀左</button>\n" \
"<button class=\"move-stop-btn\" id=\"move-stop\">STOP</button>\n" \
"<button class=\"move-btn\" id=\"move-right\">▶右</button>\n" \
"<div class=\"dpad-empty\"></div>\n" \
"<button class=\"move-btn\" id=\"move-back\">▼后</button>\n" \
"<div class=\"dpad-empty\"></div>\n" \
"</div>\n" \
"<div class=\"panel\" style=\"margin-top:6px;width:100%\">\n" \
"<h3>机械爪 (MG995)</h3>\n" \
"<div class=\"data-row\"><span>当前角度</span><span id=\"grip-angle\">--</span></div>\n" \
"<div style=\"display:flex;gap:6px;margin-top:4px\">\n" \
"<button style=\"flex:1\" onclick=\"gripAction('open')\" ontouchstart=\"event.preventDefault();gripAction('open')\">张开</button>\n" \
"<button style=\"flex:1\" onclick=\"gripAction('close')\" ontouchstart=\"event.preventDefault();gripAction('close')\">闭合</button>\n" \
"</div>\n" \
"<div style=\"margin:4px 0\"><label style=\"font-size:12px\">调试角度 (0=张开, 90=闭合): <span id=\"grip-slider-val\">0</span>°</label><br>\n" \
"<input type=\"range\" id=\"grip-slider\" min=\"0\" max=\"90\" step=\"1\" value=\"0\" style=\"width:100%\"></div>\n" \
"</div>\n" \
"<div class=\"panel\" style=\"margin-top:6px;width:100%\">\n" \
"<h3>抓取任务</h3>\n" \
"<div class=\"data-row\"><span>状态 / P4链路</span><span><span id=\"grab-st\">待机</span> / <span id=\"grab-p4\">离线</span></span></div>\n" \
"<div style=\"margin:4px 0\"><label style=\"font-size:12px\">抓取触发高度 (TOF读数): <span id=\"grab-tof-val\">0.20</span> m</label><br>\n" \
"<input type=\"range\" id=\"grab-tof\" min=\"0.10\" max=\"0.40\" step=\"0.01\" value=\"0.20\" style=\"width:100%\"></div>\n" \
"<div style=\"display:flex;gap:6px;margin-top:4px\">\n" \
"<button style=\"flex:1;background:#238636;border-color:#2ea043;color:#fff\" onclick=\"grabStart()\" ontouchstart=\"event.preventDefault();grabStart()\">测试抓取</button>\n" \
"<button style=\"flex:1\" class=\"btn-danger\" onclick=\"grabAbort()\" ontouchstart=\"event.preventDefault();grabAbort()\">中止</button>\n" \
"</div>\n" \
"</div>\n" \
"<div class=\"panel\" style=\"margin-top:6px;width:100%\">\n" \
"<h3>Motor Trim <span style=\"font-size:10px;color:#8b949e\">(补偿硬件差异)</span></h3>\n" \
"<div style=\"display:flex;justify-content:space-between;gap:4px;margin-top:4px\">\n" \
"<div style=\"text-align:center\"><div style=\"font-size:11px;color:#8b949e\">M1(FR)</div><div><button style=\"padding:2px 8px;font-size:11px\" onclick=\"adjTrim(0,0.01)\">+</button><span id=\"mtrim-0\" style=\"font-family:monospace;font-size:12px;margin:0 4px\">0.00</span><button style=\"padding:2px 8px;font-size:11px\" onclick=\"adjTrim(0,-0.01)\">-</button></div><button style=\"margin-top:2px;font-size:10px;padding:2px 6px;background:#3a1c1c;border-color:#f85149;color:#f85149\" onclick=\"calibMotor(0)\">校准</button></div>\n" \
"<div style=\"text-align:center\"><div style=\"font-size:11px;color:#8b949e\">M2(FL)</div><div><button style=\"padding:2px 8px;font-size:11px\" onclick=\"adjTrim(1,0.01)\">+</button><span id=\"mtrim-1\" style=\"font-family:monospace;font-size:12px;margin:0 4px\">0.00</span><button style=\"padding:2px 8px;font-size:11px\" onclick=\"adjTrim(1,-0.01)\">-</button></div><button style=\"margin-top:2px;font-size:10px;padding:2px 6px;background:#3a1c1c;border-color:#f85149;color:#f85149\" onclick=\"calibMotor(1)\">校准</button></div>\n" \
"<div style=\"text-align:center\"><div style=\"font-size:11px;color:#8b949e\">M3(RL)</div><div><button style=\"padding:2px 8px;font-size:11px\" onclick=\"adjTrim(2,0.01)\">+</button><span id=\"mtrim-2\" style=\"font-family:monospace;font-size:12px;margin:0 4px\">0.00</span><button style=\"padding:2px 8px;font-size:11px\" onclick=\"adjTrim(2,-0.01)\">-</button></div><button style=\"margin-top:2px;font-size:10px;padding:2px 6px;background:#3a1c1c;border-color:#f85149;color:#f85149\" onclick=\"calibMotor(2)\">校准</button></div>\n" \
"<div style=\"text-align:center\"><div style=\"font-size:11px;color:#8b949e\">M4(RR)</div><div><button style=\"padding:2px 8px;font-size:11px\" onclick=\"adjTrim(3,0.01)\">+</button><span id=\"mtrim-3\" style=\"font-family:monospace;font-size:12px;margin:0 4px\">0.00</span><button style=\"padding:2px 8px;font-size:11px\" onclick=\"adjTrim(3,-0.01)\">-</button></div><button style=\"margin-top:2px;font-size:10px;padding:2px 6px;background:#3a1c1c;border-color:#f85149;color:#f85149\" onclick=\"calibMotor(3)\">校准</button></div>\n" \
"</div>\n" \
"</div>\n" \
"<div class=\"panel\" style=\"margin-top:6px;width:100%\">\n" \
"<h3>Motor PWM <span style=\"font-size:10px;color:#f85149\">(测试仅限锁定模式)</span></h3>\n" \
"<div style=\"display:flex;justify-content:space-between;font-size:12px;font-family:monospace;color:#7ee787\">\n" \
"<span>M1:<span id=\"mot-0\">0</span>%</span><span>M2:<span id=\"mot-1\">0</span>%</span><span>M3:<span id=\"mot-2\">0</span>%</span><span>M4:<span id=\"mot-3\">0</span>%</span>\n" \
"</div>\n" \
"<div style=\"display:flex;gap:4px;margin-top:6px\">\n" \
"<button onclick=\"allPWM(2000)\">All MAX</button>\n" \
"<button onclick=\"allPWM(1000)\">All MIN</button>\n" \
"<button class=\"btn-danger\" onclick=\"allPWM(1000);motorPWM=[1000,1000,1000,1000]\">STOP</button>\n" \
"</div>\n" \
"</div>\n" \
"</div>\n" \
"</div>\n" \
"<script>\n" \
"const wsUrl='ws://'+location.host+'/ws';\n" \
"let ws,throttle=0,roll=0,pitch=0,yaw=0,mode='disarmed',connected=false,vel_x=0,vel_y=0;\n" \
"function $(id){return document.getElementById(id)}\n" \
"function setStatus(ok,text){const e=$('conn-status');e.textContent=text||(ok?'Connected':'Disconnected');e.className='status '+(ok?'status-ok':'status-err')}\n" \
"let motorPWM=[1000,1000,1000,1000];\n" \
"function allPWM(val){for(let i=0;i<4;i++){motorPWM[i]=val;let e=$('mot-'+i);if(e)e.textContent=Math.round((val-1000)/10)}}\n" \
"function connect(){\n" \
"ws=new WebSocket(wsUrl);\n" \
"ws.onopen=function(){\n" \
"connected=true;setStatus(true);\n" \
"throttle=0;roll=0;pitch=0;yaw=0;mode='disarmed';\n" \
"motorPWM=[1000,1000,1000,1000];motorTrim=[0,0,0,0];\n" \
"flowCalib=0;if($('fc-btn'))setFlowCalibUI();\n" \
"$('throttle-slider').value=0;$('throttle-val').textContent='0%';\n" \
"$('yaw-slider').value=0;$('yaw-val').textContent='0.00';\n" \
"let btns=document.querySelectorAll('.mode-btn');for(let i=0;i<btns.length;i++)btns[i].classList.remove('active');\n" \
"let disBtn=document.querySelector('.disarm-btn');if(disBtn)disBtn.classList.add('active');\n" \
"send();\n" \
"}\n" \
"ws.onclose=function(){connected=false;setStatus(false);setTimeout(connect,2000)}\n" \
"ws.onerror=function(){ws.close()}\n" \
"ws.onmessage=function(e){\n" \
"try{let d=JSON.parse(e.data);\n" \
"if(d.attitude){$('att-roll').textContent=d.attitude.roll.toFixed(1)+'°';$('att-pitch').textContent=d.attitude.pitch.toFixed(1)+'°';$('att-yaw').textContent=d.attitude.yaw.toFixed(1)+'°'}\n" \
"if(d.tof!=null){$('tof-val').textContent=d.tof+' mm'}\n" \
"if(d.alt){$('alt-target').textContent=d.alt.target.toFixed(2)+' m';if(d.alt.vz!=null)$('alt-vz').textContent=d.alt.vz.toFixed(2)+' m/s'}\n" \
"if(d.mag){if(d.mag.ok){$('mag-xyz').textContent=d.mag.x.toFixed(2)+' / '+d.mag.y.toFixed(2)+' / '+d.mag.z.toFixed(2);$('mag-hdg').textContent=d.mag.hdg.toFixed(0)+'\\u00b0'}else{$('mag-xyz').textContent='-- (未检测到)';$('mag-hdg').textContent='--'}}\n" \
"if(d.flow){$('flow-x').textContent=d.flow.x.toFixed(2);$('flow-y').textContent=d.flow.y.toFixed(2);$('flow-qual').textContent=d.flow.qual;if(d.flow.qg!=null){var qge=$('flow-qg');qge.textContent=d.flow.qg.toFixed(2);qge.style.color=d.flow.qg>0.5?'#7ee787':(d.flow.qg>=0.05?'#d2991d':'#f85149')}if(d.flow.vx!=null)$('flow-vel').textContent=d.flow.vx.toFixed(2)+' / '+d.flow.vy.toFixed(2);if(d.flow.ps!=null){var psn=['idle','wait','lock','move'],pse=$('flow-ps');pse.textContent=psn[d.flow.ps]||'?';pse.style.color=d.flow.ps===2?'#7ee787':(d.flow.ps===1?'#d2991d':'#8b949e');if(d.flow.ps>=2){var ex=d.flow.tx-d.flow.x,ey=d.flow.ty-d.flow.y;$('flow-perr').textContent=ex.toFixed(2)+' / '+ey.toFixed(2)}else{$('flow-perr').textContent='-- / --'}}}\n" \
"if(d.mode){$('mode-display').textContent=d.mode.toUpperCase();$('mode-display').className=d.mode==='disarmed'?'disarmed':'';mode=d.mode;let t={'disarmed':0,'stabilize':1,'alt_hold':2,'pos_hold':3};let btns=document.querySelectorAll('.mode-btn');for(let i=0;i<btns.length;i++)btns[i].classList.remove('active');let idx=t[d.mode];if(idx!=null)btns[idx].classList.add('active')}\n" \
"if(d.motor){for(let i=0;i<4;i++){let e=$('mot-'+i);if(e)e.textContent=(d.motor[i]*100).toFixed(0)}}\n" \
"if(d.trim){$('trim-roll').textContent=d.trim.roll.toFixed(1)+'°';$('trim-pitch').textContent=d.trim.pitch.toFixed(1)+'°'}\n" \
"if(d.mtrim){for(let i=0;i<4;i++){let e=$('mtrim-'+i);if(e)e.textContent=d.mtrim[i].toFixed(2)}}\n" \
"if(d.grip!=null){$('grip-angle').textContent=d.grip.toFixed(0)+'\\u00b0'}\n" \
"if(d.grab){var gn=['待机','对准','下降','抓取','上升'],ge=$('grab-st');ge.textContent=gn[d.grab.st]||'?';ge.style.color=d.grab.st?'#d2991d':'#8b949e';var pe=$('grab-p4');pe.textContent=d.grab.p4?'在线':'离线';pe.style.color=d.grab.p4?'#7ee787':'#8b949e'}\n" \
"}catch(ex){}\n" \
"}\n" \
"}connect();\n" \
"let motorTrim=[0,0,0,0];\n" \
"function sendStick(){if(connected&&ws){ws.send(JSON.stringify({throttle,roll,pitch,yaw,vel_x,vel_y,motor:motorPWM.map(v=>(v-1000)/1000),mtrim:motorTrim}))}}\n" \
"function send(){if(connected&&ws){ws.send(JSON.stringify({throttle,roll,pitch,yaw,vel_x,vel_y,mode,motor:motorPWM.map(v=>(v-1000)/1000),mtrim:motorTrim}))}}\n" \
"let toastTimer=null;function showToast(msg,warn){let t=$('toast');t.textContent=msg;t.className='toast'+(warn?' warn':'')+' show';if(toastTimer)clearTimeout(toastTimer);toastTimer=setTimeout(function(){t.className='toast'},1500)}\n" \
"function sendCmd(c){if(connected&&ws){ws.send(JSON.stringify({cmd:c}));if(c==='gyro_calib')showToast('Gyro calibrating... (1s)');else if(c==='level_trim')showToast('Level trim captured');else if(c==='reset_trim')showToast('Trim reset to zero')}}\n" \
"function adjTrim(idx,d){motorTrim[idx]=Math.max(-0.15,Math.min(0.15,(motorTrim[idx]||0)+d));$('mtrim-'+idx).textContent=motorTrim[idx].toFixed(2);sendStick()}\n" \
"function gripAction(a){if(!connected)return;ws.send(JSON.stringify({cmd:'grip',action:a}));showToast(a==='open'?'张开机械爪':'闭合机械爪')}\n" \
"function sendGripAngle(v){if(!connected)return;ws.send(JSON.stringify({cmd:'grip',angle:v}))}\n" \
"function grabStart(){if(!connected)return;if(confirm('测试抓取任务?\\n\\n假设当前位置精确, 直接下降到触发高度闭爪, 然后返回当前高度。\\n确保正下方有目标, 且飞机处于定高/定点模式!')){ws.send(JSON.stringify({cmd:'grab_start',test:1}));showToast('测试抓取启动',true)}}\n" \
"function grabAbort(){if(!connected)return;ws.send(JSON.stringify({cmd:'grab_abort'}));showToast('抓取任务中止',true)}\n" \
"var flowCalib=0;\n" \
"function setFlowCalibUI(){var b=$('fc-btn');b.textContent=flowCalib?'ON':'OFF';b.style.background=flowCalib?'#238636':'';b.style.color=flowCalib?'#fff':''}\n" \
"function toggleFlowCalib(){if(!connected)return;flowCalib=flowCalib?0:1;ws.send(JSON.stringify({cmd:'flow_calib',on:flowCalib}));setFlowCalibUI();showToast(flowCalib?'标定模式开启: 保持锁定, 手持移动飞机, 看 X(m)':'标定模式关闭',!flowCalib)}\n" \
"function calibMotor(idx){if(!connected)return;let names=['FR','FL','RL','RR'];if(confirm('校准 M'+(idx+1)+'('+names[idx]+')?\\n\\n1. 拆下该电机螺旋桨！\\n2. 断开电调电池\\n3. 点确定后等待提示')){ws.send(JSON.stringify({cmd:'calibrate_motor',motor_index:idx}));showToast('M'+(idx+1)+' 校准中... 按提示接通电池',true)}}\n" \
"setInterval(sendStick,50);\n" \
"/* Joysticks */\n" \
"function joy(canvasId,onMove){\n" \
"let c=$(canvasId),ctx=c.getContext('2d'),r=c.width/2,cx=0,cy=0,active=false;\n" \
"function draw(){ctx.clearRect(0,0,c.width,c.height);ctx.beginPath();ctx.arc(r,r,r-4,0,2*Math.PI);ctx.strokeStyle='#30363d';ctx.lineWidth=2;ctx.stroke();ctx.beginPath();ctx.arc(r+cx,r+cy,18,0,2*Math.PI);ctx.fillStyle=active?'#58a6ff88':'#58a6ff33';ctx.fill()}\n" \
"function getPos(e){let rect=c.getBoundingClientRect();let t=e.touches?e.touches[0]:e;return{x:t.clientX-rect.left-r,y:t.clientY-rect.top-r}}\n" \
"c.addEventListener('touchstart',function(e){e.preventDefault();active=true;let p=getPos(e);cx=Math.max(-r+20,Math.min(r-20,p.x));cy=Math.max(-r+20,Math.min(r-20,p.y));draw();onMove(cx/(r-20),-cy/(r-20))})\n" \
"c.addEventListener('touchmove',function(e){e.preventDefault();if(!active)return;let p=getPos(e);cx=Math.max(-r+20,Math.min(r-20,p.x));cy=Math.max(-r+20,Math.min(r-20,p.y));draw();onMove(cx/(r-20),-cy/(r-20))})\n" \
"c.addEventListener('touchend',function(e){e.preventDefault();active=false;cx=0;cy=0;draw();onMove(0,0)})\n" \
"draw()\n" \
"}\n" \
"$('throttle-slider').addEventListener('input',function(){throttle=parseFloat(this.value);$('throttle-val').textContent=(throttle*100).toFixed(0)+'%'});\n" \
"joy('joy-rp',function(x,y){roll=x;pitch=y});\n" \
"$('yaw-slider').addEventListener('input',function(){yaw=parseFloat(this.value);$('yaw-val').textContent=yaw.toFixed(2)});\n" \
"function setMode(m){mode=m;let btns=document.querySelectorAll('.mode-btn');for(let i=0;i<btns.length;i++)btns[i].classList.remove('active');let t={'disarmed':0,'stabilize':1,'alt_hold':2,'pos_hold':3};let idx=t[m];if(idx!=null)btns[idx].classList.add('active');send()}\n" \
"document.querySelectorAll('.mode-btn').forEach(function(b){b.addEventListener('click',function(){setMode(this.dataset.mode)});b.addEventListener('touchstart',function(e){e.preventDefault();setMode(this.dataset.mode)})});\n" \
"/* 水平移动按钮 */\n" \
"function startMove(dx,dy){vel_x=dx;vel_y=dy;var btns=document.querySelectorAll('.move-btn');for(var i=0;i<btns.length;i++)btns[i].classList.remove('pressed');if(dx>0&&dy===0)$('move-fwd').classList.add('pressed');if(dx<0&&dy===0)$('move-back').classList.add('pressed');if(dx===0&&dy<0)$('move-left').classList.add('pressed');if(dx===0&&dy>0)$('move-right').classList.add('pressed')}\n" \
"function stopMove(){vel_x=0;vel_y=0;var btns=document.querySelectorAll('.move-btn');for(var i=0;i<btns.length;i++)btns[i].classList.remove('pressed')}\n" \
"$('move-fwd').addEventListener('mousedown',function(e){e.preventDefault();startMove(0.5,0)});$('move-fwd').addEventListener('touchstart',function(e){e.preventDefault();startMove(0.5,0)});\n" \
"$('move-back').addEventListener('mousedown',function(e){e.preventDefault();startMove(-0.5,0)});$('move-back').addEventListener('touchstart',function(e){e.preventDefault();startMove(-0.5,0)});\n" \
"$('move-left').addEventListener('mousedown',function(e){e.preventDefault();startMove(0,-0.5)});$('move-left').addEventListener('touchstart',function(e){e.preventDefault();startMove(0,-0.5)});\n" \
"$('move-right').addEventListener('mousedown',function(e){e.preventDefault();startMove(0,0.5)});$('move-right').addEventListener('touchstart',function(e){e.preventDefault();startMove(0,0.5)});\n" \
"['mouseup','touchend'].forEach(function(ev){document.addEventListener(ev,function(){stopMove()})});\n" \
"$('move-stop').addEventListener('click',function(){stopMove();sendCmd('move_stop')});$('move-stop').addEventListener('touchstart',function(e){e.preventDefault();stopMove();sendCmd('move_stop')});\n" \
"/* 起飞控制 */\n" \
"$('takeoff-h').addEventListener('input',function(){$('takeoff-h-val').textContent=parseFloat(this.value).toFixed(1)});\n" \
"$('takeoff-t').addEventListener('input',function(){$('takeoff-t-val').textContent=parseFloat(this.value).toFixed(2)});\n" \
"$('grip-slider').addEventListener('input',function(){var v=parseFloat(this.value);$('grip-slider-val').textContent=v.toFixed(0);sendGripAngle(v)});\n" \
"$('grab-tof').addEventListener('input',function(){var v=parseFloat(this.value);$('grab-tof-val').textContent=v.toFixed(2);if(connected&&ws)ws.send(JSON.stringify({cmd:'grab_cfg',tof:v}))});\n" \
"function doTakeoff(){if(!connected)return;var h=parseFloat($('takeoff-h').value);var t=parseFloat($('takeoff-t').value);ws.send(JSON.stringify({cmd:'takeoff',height:h,base_throttle:t}));throttle=t;$('throttle-slider').value=t;$('throttle-val').textContent=(t*100).toFixed(0)+'%';setMode('alt_hold');showToast('起飞: '+h.toFixed(1)+'m')}\n" \
 \
"</script>\n" \
"</body>\n" \
"</html>"
