#include "net/web_control_page.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config/app_config.h"
#include "net/web_page_body.h"
#include "net/web_page_script.h"
#include "net/web_page_style.h"

esp_err_t web_control_root_get_handler(httpd_req_t *req)
{
    const app_config_t *cfg = app_config_get();

    size_t script_cap = strlen(WEB_PAGE_SCRIPT) + 32;
    char *script = calloc(1, script_cap);
    if (script == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "internal error");
        return ESP_FAIL;
    }

    int sn = snprintf(script, script_cap, WEB_PAGE_SCRIPT, cfg->max_motors);
    if (sn <= 0 || (size_t)sn >= script_cap) {
        free(script);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "internal error");
        return ESP_FAIL;
    }

    const char *tpl = "<!doctype html><html><head><meta charset='utf-8'/>"
                      "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
                      "<title>ESP MotorBridge</title><style>%s</style></head><body>%s<script>%s</script></body></html>";

    size_t html_cap = strlen(tpl) + strlen(WEB_PAGE_STYLE) + strlen(WEB_PAGE_BODY) + strlen(script) + 64;
    char *html = calloc(1, html_cap);
    if (html == NULL) {
        free(script);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "internal error");
        return ESP_FAIL;
    }

    int hn = snprintf(html, html_cap, tpl, WEB_PAGE_STYLE, WEB_PAGE_BODY, script);
    free(script);
    if (hn <= 0 || (size_t)hn >= html_cap) {
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
