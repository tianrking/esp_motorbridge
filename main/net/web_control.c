#include "net/web_control.h"

#include "net/web_control_server.h"

esp_err_t web_control_start(void)
{
    return web_control_server_start();
}
