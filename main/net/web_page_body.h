#pragma once

static const char *WEB_PAGE_BODY =
    "<div class='card'><h1>ESP MotorBridge 控制台</h1>"
    "<div class='hint'>独立电机控制: 每台电机独立模式/参数/滑条。切模式默认走 disable->set_mode->enable。</div></div>"
    "<div class='card'><h2>全局操作</h2>"
    "<div class='row'><button onclick='enableAll()'>Enable All</button><button class='red' onclick='disableAll()'>Disable All</button></div>"
    "<div class='row'><select id='vendor'><option value='damiao'>Damiao</option><option value='robstride'>RobStride</option></select><button class='gray' onclick='applyVendorAll()'>应用类型到全部电机</button></div>"
    "<div class='row'><select id='demo_id' class='democtl'><option value='1'>Demo1</option><option value='2'>Demo2</option><option value='3'>Demo3</option></select><button class='orange democtl' onclick='demoStart()'>启动 Demo</button></div>"
    "<div class='row'><button class='red democtl' onclick='demoStop()'>停止 Demo</button><button class='gray democtl' onclick='demoReset()'>复位</button><button class='red democtl' onclick='estopAll()'>E-Stop</button></div>"
    "<div class='row'><input id='id_old' type='number' min='1' max='127' step='1' value='1'/><input id='id_new' type='number' min='1' max='127' step='1' value='2'/><button class='gray' onclick='setDamiaoId()'>改ID(Damiao)</button></div></div>"
    "<div class='card'><h2>电机面板</h2><div id='motors' class='motors'></div><div id='global_msg' class='hint'></div></div>";
