/*
 Name:		ESP32APRS_Audio
 Created:	13-10-2023 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/
#include <Arduino.h>
#include "webservice.h"
#include "base64.hpp"
#include "wireguard_vpn.h"
#include <LibAPRSesp.h>
#include <parse_aprs.h>
#include "jquery_min_js.h"

#ifdef PPPOS
#include <PPP.h>
#endif

#ifdef SH1106
#include <Adafruit_SH1106.h>
#else
#include "Adafruit_SSD1306.h"
#endif // SH1106

#define SCREEN_ADDRESS 0x3C

AsyncWebServer async_server(80);
AsyncWebServer async_websocket(81);
AsyncWebSocket ws("/ws");
AsyncWebSocket ws_gnss("/ws_gnss");

#ifdef MQTT
#include <PubSubClient.h>
extern PubSubClient clientMQTT;
#endif

extern pppType pppStatus;

// Create an Event Source on /events
AsyncEventSource lastheard_events("/eventHeard");
AsyncEventSource message_events("/eventMsg");

String webString;

extern unsigned long waitISRetry;
extern int8_t adcEn;
extern int8_t dacEn;
extern unsigned long upTimeStamp;
extern double VBat;
extern bool VBat_Flag;
#ifdef OLED
#ifdef SH1106
extern Adafruit_SH1106 display;
#else
extern Adafruit_SSD1306 display;
#endif
#endif // OLED

bool defaultSetting = false;

void serviceHandle()
{
	// server.handleClient();
}

void notFound(AsyncWebServerRequest *request)
{
	request->send(404, "text/plain", "Not found");
}

void handle_logout(AsyncWebServerRequest *request)
{
	webString = "Log out";
	request->send(200, "text/html", webString);
}

void setMainPage(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	webString = "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n";
	webString += "<meta name=\"robots\" content=\"index\" />\n";
	webString += "<meta name=\"robots\" content=\"follow\" />\n";
	webString += "<meta name=\"language\" content=\"English\" />\n";
	webString += "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\" />\n";
	webString += "<meta name=\"GENERATOR\" content=\"configure 20230924\" />\n";
	webString += "<meta name=\"Author\" content=\"Mr.Somkiat Nakhonthai (HS5TQA)\" />\n";
	webString += "<meta name=\"Description\" content=\"Web Embedded Configuration\" />\n";
	webString += "<meta name=\"KeyWords\" content=\"ESP32,ESP32C3,AFSK,APRS\" />\n";
	webString += "<meta http-equiv=\"Cache-Control\" content=\"no-cache, no-store, must-revalidate\" />\n";
	webString += "<meta http-equiv=\"pragma\" content=\"no-cache\" />\n";
	webString += "<link rel=\"shortcut icon\" href=\"http://aprs.dprns.com/favicon.ico\" type=\"image/x-icon\" />\n";
	webString += "<meta http-equiv=\"Expires\" content=\"0\" />\n";
	if(strlen(config.host_name) > 0)
		webString += "<title>" + String(config.host_name) + "</title>\n";
	else
		webString += "<title>ESP32APRS_Audio</title>\n";
	webString += "<link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\" />\n";
	webString += "<script src=\"/jquery-3.7.1.js\"></script>\n";
	webString += "<script type=\"text/javascript\">\n";
	webString += "function selectTab(evt, tabName) {\n";
	webString += "var i, tabcontent, tablinks;\n";
	webString += "tablinks = document.getElementsByClassName(\"nav-tabs\");\n";
	webString += "for (i = 0; i < tablinks.length; i++) {\n";
	webString += "tablinks[i].className = tablinks[i].className.replace(\" active\", \"\");\n";
	webString += "}\n";
	webString += "\n";
	webString += "//document.getElementById(tabName).style.display = \"block\";\n";
	webString += "if (tabName == 'DashBoard') {\n";
	webString += "$(\"#contentmain\").load(\"/dashboard\");\n";
	webString += "} else if (tabName == 'Radio') {\n";
	webString += "$(\"#contentmain\").load(\"/radio\");\n";
	webString += "} else if (tabName == 'IGATE') {\n";
	webString += "$(\"#contentmain\").load(\"/igate\");\n";
	webString += "} else if (tabName == 'DIGI') {\n";
	webString += "$(\"#contentmain\").load(\"/digi\");\n";
	webString += "} else if (tabName == 'TRACKER') {\n";
	webString += "$(\"#contentmain\").load(\"/tracker\");\n";
	webString += "} else if (tabName == 'WX') {\n";
	webString += "$(\"#contentmain\").load(\"/wx\");\n";
	webString += "} else if (tabName == 'TLM') {\n";
	webString += "$(\"#contentmain\").load(\"/tlm\");\n";
	webString += "} else if (tabName == 'SENSOR') {\n";
	webString += "$(\"#contentmain\").load(\"/sensor\");\n";
	webString += "} else if (tabName == 'VPN') {\n";
	webString += "$(\"#contentmain\").load(\"/vpn\");\n";
#ifdef MQTT
	webString += "} else if (tabName == 'MQTT') {\n";
	webString += "$(\"#contentmain\").load(\"/mqtt\");\n";
#endif
	webString += "} else if (tabName == 'MSG') {\n";
	webString += "$(\"#contentmain\").load(\"/msg\");\n";
	webString += "} else if (tabName == 'WiFi') {\n";
	webString += "$(\"#contentmain\").load(\"/wireless\");\n";
	webString += "} else if (tabName == 'MOD') {\n";
	webString += "$(\"#contentmain\").load(\"/mod\");\n";
	webString += "} else if (tabName == 'System') {\n";
	webString += "$(\"#contentmain\").load(\"/system\");\n";
	webString += "} else if (tabName == 'File') {\n";
	webString += "$(\"#contentmain\").load(\"/storage\");\n";
	webString += "} else if (tabName == 'About') {\n";
	webString += "$(\"#contentmain\").load(\"/about\");\n";
	webString += "}\n";
	webString += "\n";
	webString += "if (evt != null) evt.currentTarget.className += \" active\";\n";
	webString += "}\n";
	webString += "if (!!window.EventSource) {";
	webString += "var source = new EventSource('/eventHeard');";

	webString += "source.addEventListener('open', function(e) {";
	webString += "console.log(\"Events Connected\");";
	webString += "}, false);";
	webString += "source.addEventListener('error', function(e) {";
	webString += "if (e.target.readyState != EventSource.OPEN) {";
	webString += "console.log(\"Events Disconnected\");";
	webString += "}\n}, false);";
	webString += "source.addEventListener('lastHeard', function(e) {";
	// webString += "console.log(\"lastHeard\", e.data);";
	webString += "var lh=document.getElementById(\"lastHeard\");";
	webString += "if(lh != null) {lh.innerHTML = e.data;}";
	webString += "}, false);\n}";
	webString += "if (!!window.EventSource) {";
		webString += "var source = new EventSource('/eventMsg');";

		webString += "source.addEventListener('open', function(e) {";
		webString += "console.log(\"Events MSG Connected\");";
		webString += "}, false);";
		webString += "source.addEventListener('error', function(e) {";
		webString += "if (e.target.readyState != EventSource.OPEN) {";
		webString += "console.log(\"Events MSG Disconnected\");";
		webString += "}\n}, false);";
		webString += "source.addEventListener('chatMsg', function(e) {";
		// webString += "console.log(\"lastHeard\", e.data);";
		webString += "var lh=document.getElementById(\"chatMsg\");";
		webString += "if(lh != null) {lh.innerHTML = e.data;}";
		webString += "}, false);\n}";
	webString += "</script>\n";
	webString += "</head>\n";
	webString += "\n";
	webString += "<body onload=\"selectTab(event, 'DashBoard')\">\n";
	webString += "\n";
	webString += "<div class=\"container\">\n";
	webString += "<div class=\"header\">\n";
	// webString += "<div style=\"font-size: 8px; text-align: right; padding-right: 8px;\">ESP32IGate Firmware V" + String(VERSION) + "</div>\n";
	// webString += "<div style=\"font-size: 8px; text-align: right; padding-right: 8px;\"><a href=\"/logout\">[LOG OUT]</a></div>\n";
	if(strlen(config.host_name) > 0)
		webString += "<h1>" + String(config.host_name) + "</h1>\n";
	else
		webString += "<h1>ESP32APRS_Audio</h1>\n";
	webString += "<div style=\"font-size: 8px; text-align: right; padding-right: 8px;\"><a href=\"/logout\">[LOG OUT]</a></div>\n";
	webString += "<div class=\"row\">\n";
	webString += "<ul class=\"nav nav-tabs\" style=\"margin: 5px;\">\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'DashBoard')\">DashBoard</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'Radio')\" id=\"btnRadio\">Radio</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'IGATE')\">IGATE</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'DIGI')\">DIGI</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'TRACKER')\">TRACKER</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'WX')\">WX</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'TLM')\">TLM</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'SENSOR')\">SENSOR</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'VPN')\">VPN</button>\n";
#ifdef MQTT
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'MQTT')\">MQTT</button>\n";
#endif
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'MSG')\">MSG</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'WiFi')\">WiFi</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'MOD')\">MOD</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'System')\">System</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'File')\">File</button>\n";
	webString += "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'About')\">About</button>\n";
	webString += "</ul>\n";
	webString += "</div>\n";
	webString += "</div>\n";
	webString += "\n";

	webString += "<div class=\"contentwide\" id=\"contentmain\"  style=\"font-size: 2pt;\">\n";
	webString += "\n";
	webString += "</div>\n";
	webString += "<br />\n";
	webString += "<div class=\"footer\">\n";
	webString += "ESP32APRS_Audio Web Configuration<br />Copy right ©2023.\n";
	webString += "<br />\n";
	webString += "</div>\n";
	webString += "</div>\n";
	webString += "<!-- <script type=\"text/javascript\" src=\"/nice-select.min.js\"></script> -->\n";
	webString += "<script type=\"text/javascript\">\n";
	webString += "var selectize = document.querySelectorAll('select')\n";
	webString += "var options = { searchable: true };\n";
	webString += "selectize.forEach(function (select) {\n";
	webString += "if (select.length > 30 && null === select.onchange && !select.name.includes(\"ExtendedId\")) {\n";
	webString += "select.classList.add(\"small\", \"selectize\");\n";
	webString += "tabletd = select.closest('td');\n";
	webString += "tabletd.style.cssText = 'overflow-x:unset';\n";
	webString += "NiceSelect.bind(select, options);\n";
	webString += "}\n";
	webString += "});\n";
	webString += "</script>\n";
	webString += "</body>\n";
	webString += "</html>";
	request->send(200, "text/html", webString); // send to someones browser when asked
	lastHeardTimeout=0;
	lastHeard_Flag = true;
}

////////////////////////////////////////////////////////////
// handler for web server request: http://IpAddress/      //
////////////////////////////////////////////////////////////

void handle_css(AsyncWebServerRequest *request)
{
	const char *css = ".container{width:820px;text-align:left;margin:auto;border-radius:10px 10px 10px 10px;-moz-border-radius:10px 10px 10px 10px;-webkit-border-radius:10px 10px 10px 10px;-khtml-border-radius:10px 10px 10px 10px;-ms-border-radius:10px 10px 10px 10px;box-shadow:3px 3px 3px #707070;background:#fff;border-color: #2194ec;padding: 0px;border-width: 5px;border-style:solid;}body,font{font:12px verdana,arial,sans-serif;color:#fff}.header{background:#2194ec;text-decoration:none;color:#fff;font-family:verdana,arial,sans-serif;text-align:left;padding:5px 0;border-radius:10px 10px 0 0;-moz-border-radius:10px 10px 0 0;-webkit-border-radius:10px 10px 0 0;-khtml-border-radius:10px 10px 0 0;-ms-border-radius:10px 10px 0 0}.content{margin:0 0 0 166px;padding:1px 5px 5px;color:#000;background:#fff;text-align:center;font-size: 8pt;}.contentwide{padding:50px 5px 5px;color:#000;background:#fff;text-align:center}.contentwide h2{color:#000;font:1em verdana,arial,sans-serif;text-align:center;font-weight:700;padding:0;margin:0;font-size: 12pt;}.footer{background:#2194ec;text-decoration:none;color:#fff;font-family:verdana,arial,sans-serif;font-size:9px;text-align:center;padding:10px 0;border-radius:0 0 10px 10px;-moz-border-radius:0 0 10px 10px;-webkit-border-radius:0 0 10px 10px;-khtml-border-radius:0 0 10px 10px;-ms-border-radius:0 0 10px 10px;clear:both}#tail{height:450px;width:805px;overflow-y:scroll;overflow-x:scroll;color:#0f0;background:#000}table{vertical-align:middle;text-align:center;empty-cells:show;padding-left:3;padding-right:3;padding-top:3;padding-bottom:3;border-collapse:collapse;border-color:#0f07f2;border-style:solid;border-spacing:0px;border-width:3px;text-decoration:none;color:#fff;background:#000;font-family:verdana,arial,sans-serif;font-size : 12px;width:100%;white-space:nowrap}table th{font-size: 10pt;font-family:lucidia console,Monaco,monospace;text-shadow:1px 1px #0e038c;text-decoration:none;background:#0525f7;border:1px solid silver}table tr:nth-child(even){background:#f7f7f7}table tr:nth-child(odd){background:#eeeeee}table td{color:#000;font-family:lucidia console,Monaco,monospace;text-decoration:none;border:1px solid #010369}body{background:#edf0f5;color:#000}a{text-decoration:none}a:link,a:visited{text-decoration:none;color:#0000e0;font-weight:400}th:last-child a.tooltip:hover span{left:auto;right:0}ul{padding:5px;margin:10px 0;list-style:none;float:left}ul li{float:left;display:inline;margin:0 10px}ul li a{text-decoration:none;float:left;color:#999;cursor:pointer;font:900 14px/22px arial,Helvetica,sans-serif}ul li a span{margin:0 10px 0 -10px;padding:1px 8px 5px 18px;position:relative;float:left}h1{text-shadow:2px 2px #303030;text-align:center}.toggle{position:absolute;margin-left:-9999px;visibility:hidden}.toggle+label{display:block;position:relative;cursor:pointer;outline:none}input.toggle-round-flat+label{padding:1px;width:33px;height:18px;background-color:#ddd;border-radius:10px;transition:background .4s}input.toggle-round-flat+label:before,input.toggle-round-flat+label:after{display:block;position:absolute;}input.toggle-round-flat+label:before{top:1px;left:1px;bottom:1px;right:1px;background-color:#fff;border-radius:10px;transition:background .4s}input.toggle-round-flat+label:after{top:2px;left:2px;bottom:2px;width:16px;background-color:#ddd;border-radius:12px;transition:margin .4s,background .4s}input.toggle-round-flat:checked+label{background-color:#dd4b39}input.toggle-round-flat:checked+label:after{margin-left:14px;background-color:#dd4b39}@-moz-document url-prefix(){select,input{margin:0;padding:0;border-width:1px;font:12px verdana,arial,sans-serif}input[type=button],button,input[type=submit]{padding:0 3px;border-radius:3px 3px 3px 3px;-moz-border-radius:3px 3px 3px 3px}}.nice-select.small,.nice-select-dropdown li.option{height:24px!important;min-height:24px!important;line-height:24px!important}.nice-select.small ul li:nth-of-type(2){clear:both}.nav{margin-bottom:0;padding-left:10;list-style:none}.nav>li{position:relative;display:block}.nav>li>a{position:relative;display:block;padding:5px 10px}.nav>li>a:hover,.nav>li>a:focus{text-decoration:none;background-color:#eee}.nav>li.disabled>a{color:#999}.nav>li.disabled>a:hover,.nav>li.disabled>a:focus{color:#999;text-decoration:none;background-color:initial;cursor:not-allowed}.nav .open>a,.nav .open>a:hover,.nav .open>a:focus{background-color:#eee;border-color:#428bca}.nav .nav-divider{height:1px;margin:9px 0;overflow:hidden;background-color:#e5e5e5}.nav>li>a>img{max-width:none}.nav-tabs{border-bottom:1px solid #ddd}.nav-tabs>li{float:left;margin-bottom:-1px}.nav-tabs>li>a{margin-right:0;line-height:1.42857143;border:1px solid #ddd;border-radius:10px 10px 0 0}.nav-tabs>li>a:hover{border-color:#eee #eee #ddd}.nav-tabs>button{margin-right:0;line-height:1.42857143;border:2px solid #ddd;border-radius:10px 10px 0 0}.nav-tabs>button:hover{background-color:#25bbfc;border-color:#428bca;color:#eaf2f9;border-bottom-color:transparent;}.nav-tabs>button.active,.nav-tabs>button.active:hover,.nav-tabs>button.active:focus{color:#f7fdfd;background-color:#1aae0d;border:1px solid #ddd;border-bottom-color:transparent;cursor:default}.nav-tabs>li.active>a,.nav-tabs>li.active>a:hover,.nav-tabs>li.active>a:focus{color:#428bca;background-color:#e5e5e5;border:1px solid #ddd;border-bottom-color:transparent;cursor:default}.nav-tabs.nav-justified{width:100%;border-bottom:0}.nav-tabs.nav-justified>li{float:none}.nav-tabs.nav-justified>li>a{text-align:center;margin-bottom:5px}.nav-tabs.nav-justified>.dropdown .dropdown-menu{top:auto;left:auto}.nav-status{float:left;margin:0;padding:3px;width:160px;font-weight:400;min-height:600}#bar,#prgbar {background-color: #f1f1f1;border-radius: 14px}#bar {background-color: #3498db;width: 0%;height: 14px}.switch{position:relative;display:inline-block;width:34px;height:16px}.switch input{opacity:0;width:0;height:0}.slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:#f55959;-webkit-transition:.4s;transition:.4s}.slider:before{position:absolute;content:\"\";height:12px;width:12px;left:2px;bottom:2px;background-color:#fff;-webkit-transition:.4s;transition:.4s}input:checked+.slider{background-color:#5ca30a}input:focus+.slider{box-shadow:0 0 1px #5ca30a}input:checked+.slider:before{-webkit-transform:translateX(16px);-ms-transform:translateX(16px);transform:translateX(16px)}.slider.round{border-radius:34px}.slider.round:before{border-radius:50%}.button{border:1px solid #06c;background-color:#09c;color:#fff;padding:5px 10px;border-radius: 3px}.button:hover{border:1px solid #09c;background-color:#0ac;color:#fff}.button:disabled,button[disabled]{border:1px solid #999;background-color:#ccc;color:#666}\n";
	request->send_P(200, "text/css", css);
}

void handle_jquery(AsyncWebServerRequest *request)
{
	#if defined(CONFIG_IDF_TARGET_ESP32)
	adcEn=-1;
	dacEn=-1;
	delay(100);
	#endif
	AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("application/javascript")), (const uint8_t *)jquery_3_7_1_min_js_gz, jquery_3_7_1_min_js_gz_len);
	response->addHeader(String(F("Content-Encoding")), String(F("gzip")));
	response->setContentLength(jquery_3_7_1_min_js_gz_len);
	request->send(response);
	#if defined(CONFIG_IDF_TARGET_ESP32)
	delay(200);
	adcEn=1;
	dacEn=0;
	#endif
}

void handle_dashboard(AsyncWebServerRequest *request)
{
	// if (!request->authenticate(config.http_username, config.http_password))
	// {
	// 	return request->requestAuthentication();
	// }
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);
	webString = "<script type=\"text/javascript\">\n";
	webString += "function reloadSysInfo() {\n";
	webString += "$(\"#sysInfo\").load(\"/sysinfo\", function () { setTimeout(reloadSysInfo, 60000) });\n";
	webString += "}\n";
	webString += "setTimeout(reloadSysInfo(), 100);\n";
	webString += "function reloadSidebarInfo() {\n";
	webString += "$(\"#sidebarInfo\").load(\"/sidebarInfo\", function () { setTimeout(reloadSidebarInfo, 10000) });\n";
	webString += "}\n";
	webString += "setTimeout(reloadSidebarInfo, 1000);\n";
	webString += "$(window).trigger('resize');\n";

	webString += "</script>\n";

	webString += "<div id=\"sysInfo\">\n";
	webString += "</div>\n";

	webString += "<br />\n";
	webString += "<div class=\"nav-status\">\n";
	webString += "<div id=\"sidebarInfo\">\n";
	webString += "</div>\n";
	webString += "<br />\n";

	webString += "<table>\n";
	webString += "<tr>\n";
	webString += "<th colspan=\"2\">Radio Info</th>\n";
	webString += "</tr>\n";
	if(config.rf_en){
		webString += "<tr>\n";
		webString += "<td>Freq.TX</td>\n";
		webString += "<td style=\"background: #ffffff;\">" + String(config.freq_tx, 4) + " MHz</td>\n";
		webString += "</tr>\n";
		webString += "<tr>\n";
		webString += "<td>Freq.RX</td>\n";
		webString += "<td style=\"background: #ffffff;\">" + String(config.freq_rx, 4) + " MHz</td>\n";
		webString += "</tr>\n";	
		webString += "<tr>\n";
		webString += "<td>TX PWR</td>\n";
		if (config.rf_power)
			webString += "<td>HIGH</td>\n";
		else
			webString += "<td>LOW</td>\n";
		webString += "</tr>\n";		
	}
	webString += "<tr>\n";
	webString += "<td>MODEM</td>\n";
	webString += "<td style=\"background: #ffffff;\">" + String(MODEM_TYPE[config.modem_type]) + "</td>\n";
	webString += "</tr>\n";
	webString += "<tr>\n";
	webString += "<td>FX.25</td>\n";
	webString += "<td style=\"background: #ffffff;\">" + String(FX25_MODE[config.fx25_mode]) + "</td>\n";
	webString += "</tr>\n";
	webString += "</table>\n";
	webString += "\n";
	if (config.igate_en)
	{
		webString += "<br />\n";
		webString += "<table>\n";
		webString += "<tr>\n";
		webString += "<th colspan=\"2\">APRS-IS SERVER</th>\n";
		webString += "</tr>\n";
		webString += "<tr>\n";
		webString += "<td>HOST</td>\n";
		webString += "<td style=\"background: #ffffff;\">" + String(config.aprs_host) + "</td>\n";
		webString += "</tr>\n";
		webString += "<tr>\n";
		webString += "<td>PORT</td>\n";
		webString += "<td style=\"background: #ffffff;\">" + String(config.aprs_port) + "</td>\n";
		webString += "</tr>\n";
		webString += "</table>\n";
	}
	webString += "<br />\n";
	webString += "<table>\n";
	webString += "<tr>\n";
	webString += "<th colspan=\"2\">WiFi</th>\n";
	webString += "</tr>\n";
	webString += "<tr>\n";
	webString += "<td>MODE</td>\n";
	String strWiFiMode = "OFF";
	if (config.wifi_mode == WIFI_STA_FIX)
	{
		strWiFiMode = "STA";
	}
	else if (config.wifi_mode == WIFI_AP_FIX)
	{
		strWiFiMode = "AP";
	}
	else if (config.wifi_mode == WIFI_AP_STA_FIX)
	{
		strWiFiMode = "AP+STA";
	}
	webString += "<td style=\"background: #ffffff;\">" + strWiFiMode + "</td>\n";
	webString += "</tr>\n";
	webString += "<tr>\n";
	webString += "<td>SSID</td>\n";
	webString += "<td style=\"background: #ffffff;\">" + String(WiFi.SSID()) + "</td>\n";
	webString += "</tr>\n";
	webString += "<tr>\n";
	webString += "<td>RSSI</td>\n";
	if (WiFi.isConnected())
		webString += "<td style=\"background: #ffffff;\">" + String(WiFi.RSSI()) + " dBm</td>\n";
	else
		webString += "<td style=\"background:#606060; color:#b0b0b0;\" aria-disabled=\"true\">Disconnect</td>\n";
	webString += "</tr>\n";
	webString += "</table>\n";
	webString += "<br />\n";
#ifdef BLUETOOTH
	webString += "<table>\n";
	webString += "<tr>\n";

	webString += "<th colspan=\"2\">Bluetooth</th>\n";
	webString += "</tr>\n";
	webString += "<td>Master</td>\n";
	if (config.bt_master)
		webString += "<td style=\"background:#0b0; color:#030; width:50%;\">Enabled</td>\n";
	else
		webString += "<td style=\"background:#606060; color:#b0b0b0;\" aria-disabled=\"true\">Disabled</td>\n";
	webString += "</tr>\n";
	webString += "<tr>\n";
	webString += "<tr>\n";
	webString += "<td>NAME</td>\n";
	webString += "<td style=\"background: #ffffff;\">" + String(config.bt_name) + "</td>\n";
	webString += "</tr>\n";
	webString += "<tr>\n";
	webString += "<tr>\n";
	webString += "<td>MODE</td>\n";
	String btMode = "";
	if (config.bt_mode == 1)
	{
		btMode = "TNC2";
	}
	else if (config.bt_mode == 2)
	{
		btMode = "KISS";
	}
	else
	{
		btMode = "NONE";
	}
	webString += "<td style=\"background: #ffffff;\">" + btMode + "</td>\n";
	webString += "</tr>\n";
	webString += "<tr>\n";
	webString += "</table>\n";
#endif
	webString += "</div>\n";

	webString += "</div>\n";
	webString += "\n";
	webString += "<div class=\"content\">\n";
	webString += "<div id=\"lastHeard\">\n";
	webString += event_lastHeard(true);
	webString += "</div>\n";

	request->send(200, "text/html", webString); // send to someones browser when asked
	lastHeardTimeout=0;
	lastHeard_Flag = true;
}

void handle_sidebar(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	String html = "<table style=\"background:white;border-collapse: unset;\">\n";
	html += "<tr>\n";
	html += "<th colspan=\"2\">Modes Enabled</th>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	if (config.igate_en)
		html += "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">IGATE</th>\n";
	else
		html += "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">IGATE</th>\n";

	if (config.digi_en)
		html += "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">DIGI</th>\n";
	else
		html += "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">DIGI</th>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	if (config.wx_en)
		html += "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">WX</th>\n";
	else
		html += "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">WX</th>\n";
	if (config.trk_en)
		html += "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">TRACKER</th>\n";
	else
		html += "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">TRACKER</th>\n";
	html += "</tr>\n";
	html += "</table>\n";
	html += "<br />\n";
	html += "<table style=\"background:white;border-collapse: unset;\">\n";
	html += "<tr>\n";
	html += "<th colspan=\"2\">Network Status</th>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	if (aprsClient.connected() == true)
		html += "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">APRS-IS</th>\n";
	else
		html += "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">APRS-IS</th>\n";
	if (wireguard_active() == true)
		html += "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">VPN</th>\n";
	else
		html += "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">VPN</th>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	//html += "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">4G LTE</th>\n";
	#ifdef PPPOS
	if (PPP.connected())
		html += "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">PPPoS</th>\n";
	else
	#endif
		html += "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">PPPoS</th>\n";
	if(config.fx25_mode>0)
		html += "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">FX.25</th>\n";
	else
		html += "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">FX.25</th>\n";
	html += "</tr>\n";
	html += "</table>\n";
	html += "<br />\n";
	html += "<table>\n";
	html += "<tr>\n";
	html += "<th colspan=\"2\">STATISTICS</th>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td style=\"width: 60px;text-align: right;\">RADIO RX:</td>\n";
	html += "<td style=\"background: #ffffff;\">" + String(status.rxCount) + "</td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td style=\"width: 60px;text-align: right;\">PACKET RX:</td>\n";
	html += "<td style=\"background: #ffffff;\">" + String(status.allCount) + "</td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td style=\"width: 60px;text-align: right;\">PACKET TX:</td>\n";
	html += "<td style=\"background: #ffffff;\">" + String(status.txCount) + "</td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td style=\"width: 60px;text-align: right;\">RF2INET:</td>\n";
	html += "<td style=\"background: #ffffff;\">" + String(status.rf2inet) + "</td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td style=\"width: 60px;text-align: right;\">INET2RF:</td>\n";
	html += "<td style=\"background: #ffffff;\">" + String(status.inet2rf) + "</td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td style=\"width: 60px;text-align: right;\">DIGI:</td>\n";
	html += "<td style=\"background: #ffffff;\">" + String(status.digiCount) + "</td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td style=\"width: 60px;text-align: right;\">DROP/ERR:</td>\n";
	html += "<td style=\"background: #ffffff;\">" + String(status.dropCount) + "/" + String(status.errorCount) + "</td>\n";
	html += "</tr>\n";
	html += "</table>\n";
	html += "<br />\n";
	if (config.gnss_enable)
	{
		html += "<table>\n";
		html += "<tr>\n";
		html += "<th colspan=\"2\">GPS Info <a href=\"/gnss\" target=\"_gnss\" style=\"color: yellow;font-size:8pt\">[View]</a></th>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td>LAT:</td>\n";
		html += "<td style=\"background: #ffffff;text-align: left;\">" + String(gps.location.lat(), 5) + "</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td>LON:</td>\n";
		html += "<td style=\"background: #ffffff;text-align: left;\">" + String(gps.location.lng(), 5) + "</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td>ALT:</td>\n";
		html += "<td style=\"background: #ffffff;text-align: left;\">" + String(gps.altitude.meters(), 1) + "</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td>SAT:</td>\n";
		html += "<td style=\"background: #ffffff;text-align: left;\">" + String(gps.satellites.value()) + "</td>\n";
		html += "</tr>\n";
		html += "</table>\n";
	}
	html += "<script>\n";
	html += "$(window).trigger('resize');\n";
	html += "</script>\n";
	// request->send(200, "text/html", html); // send to someones browser when asked
	// delay(100);
	request->send(200, "text/html", html);
	html.clear();
}

void handle_symbol(AsyncWebServerRequest *request)
{
	int i;
	int sel = -1;
	for (i = 0; i < request->args(); i++)
	{
		if (request->argName(i) == "sel")
		{
			if (request->arg(i) != "")
			{
				if (isValidNumber(request->arg(i)))
				{
					sel = request->arg(i).toInt();
				}
			}
		}
	}

	char *web = (char *)calloc(22000, sizeof(char));
	if (web)
	{
		memset(web, 0, 22000);
		strcat(web, "<table border=\"1\" align=\"center\">\n");
		strcat(web, "<tr><th colspan=\"16\">Table '/'</th></tr>\n");
		strcat(web, "<tr>\n");
		char lnk[200];
		for (i = 33; i < 129; i++)
		{
			memset(lnk,0,sizeof(lnk));
			//<td><img onclick="window.opener.setValue(113,2);" src="http://aprs.dprns.com/symbols/icons/113-2.png"></td>			
			if (sel == -1)
				sprintf(lnk, "<td><img onclick=\"window.opener.setValue(%d,1);\" src=\"http://aprs.dprns.com/symbols/icons/%d-1.png\"></td>", i, i);
			else
				sprintf(lnk, "<td><img onclick=\"window.opener.setValue(%d,%d,1);\" src=\"http://aprs.dprns.com/symbols/icons/%d-1.png\"></td>", sel, i, i);
			strcat(web, lnk);

			if (((i % 16) == 0) && (i < 126))
				strcat(web, "</tr>\n<tr>\n");
		}
		strcat(web, "</tr>");
		strcat(web, "</table>\n<br />");
		strcat(web, "<table border=\"1\" align=\"center\">\n");
		strcat(web, "<tr><th colspan=\"16\">Table '\\'</th></tr>\n");
		strcat(web, "<tr>\n");
		for (i = 33; i < 129; i++)
		{
			memset(lnk,0,sizeof(lnk));
			if (sel == -1)
				sprintf(lnk, "<td><img onclick=\"window.opener.setValue(%d,2);\" src=\"http://aprs.dprns.com/symbols/icons/%d-2.png\"></td>", i, i);
			else
				sprintf(lnk, "<td><img onclick=\"window.opener.setValue(%d,%d,2);\" src=\"http://aprs.dprns.com/symbols/icons/%d-2.png\"></td>", sel, i, i);
			strcat(web, lnk);
			if (((i % 16) == 0) && (i < 126))
				strcat(web, "</tr>\n<tr>\n");
		}
		strcat(web, "</tr>");
		strcat(web, "</table>\n");
		request->send_P(200, "text/html", web);
		free(web);
	}
}

void handle_sysinfo(AsyncWebServerRequest *request)
{
	String html = "<table style=\"table-layout: fixed;border-collapse: unset;border-radius: 10px;border-color: #ee800a;border-style: ridge;border-spacing: 1px;border-width: 4px;background: #ee800a;\">\n";
	html += "<tr>\n";
	html += "<th><span><b>Up Time</b></span></th>\n";
	html += "<th><span>RAM(KByte)</span></th>\n";
	#ifdef BOARD_HAS_PSRAM
	html += "<th><span>PSRAM(KByte)</span></th>\n";
	#endif
	html += "<th><span>SPIFFS(KByte)</span></th>\n";
	if(VBat_Flag)
		html += "<th><span>VBat(V)</span></th>\n";
	html += "<th><span>CPU(Mhz)</span></th>\n";

	html += "</tr>\n";
	html += "<tr>\n";
	//time_t tn = time(NULL) - systemUptime;
	// String uptime = String(day(tn) - 1, DEC) + "D " + String(hour(tn), DEC) + ":" + String(minute(tn), DEC) + ":" + String(second(tn), DEC);
	//String uptime = String(day(tn) - 1, DEC) + "D " + String(hour(tn), DEC) + ":" + String(minute(tn), DEC);
	char strTime[20];
	convertSecondsToDHMS(strTime,(millis()/1000)-upTimeStamp);
	html += "<td><b>" + String(strTime) + "</b></td>\n";
	html += "<td><b>" + String((float)ESP.getFreeHeap() / 1000, 1) + "/" + String((float)ESP.getHeapSize() / 1000, 1) + "</b></td>\n";
	#ifdef BOARD_HAS_PSRAM
	html += "<td><b>" + String((float)ESP.getFreePsram() / 1000, 1) + "/" + String((float)ESP.getPsramSize() / 1000, 1) + "</b></td>\n";
	#endif
	unsigned long cardTotal = LITTLEFS.totalBytes();
	unsigned long cardUsed = LITTLEFS.usedBytes();
	html += "<td><b>" + String((double)cardUsed / 1024, 1) + "/" + String((double)cardTotal / 1024, 1) + "</b></td>\n";
	if(VBat_Flag)
		html += "<td><b>" + String(VBat, 2) + "</b></td>\n";
	html += "<td><b>" + String(ESP.getCpuFreqMHz()) + "</b></td>\n";
	// html += "<td style=\"background: #f00\"><b>" + String(ESP.getCycleCount()) + "</b></td>\n";
	html += "</tr>\n";
	html += "</table>\n";
	request->send(200, "text/html", html); // send to someones browser when asked
	html.clear();
}

// void handle_lastHeard(AsyncWebServerRequest *request)
// {
// 	struct pbuf_t aprs;
// 	ParseAPRS aprsParse;
// 	struct tm tmstruct;
// 	String html = "";
// 	sort(pkgList, PKGLISTSIZE);

// 	html = "<table>\n";
// 	html += "<th colspan=\"7\" style=\"background-color: #070ac2;\">LAST HEARD <a href=\"/tnc2\" target=\"_tnc2\" style=\"color: yellow;font-size:8pt\">[RAW]</a></th>\n";
// 	html += "<tr>\n";
// 	html += "<th style=\"min-width:10ch\"><span><b>Time (";
// 	if (config.timeZone >= 0)
// 		html += "+";
// 	// else
// 	//	html += "-";

// 	if (config.timeZone == (int)config.timeZone)
// 		html += String((int)config.timeZone) + ")</b></span></th>\n";
// 	else
// 		html += String(config.timeZone, 1) + ")</b></span></th>\n";
// 	html += "<th style=\"min-width:16px\">ICON</th>\n";
// 	html += "<th style=\"min-width:10ch\">Callsign</th>\n";
// 	html += "<th>VIA LAST PATH</th>\n";
// 	html += "<th style=\"min-width:5ch\">DX</th>\n";
// 	html += "<th style=\"min-width:5ch\">PACKET</th>\n";
// 	html += "<th style=\"min-width:5ch\">AUDIO</th>\n";
// 	html += "</tr>\n";

// 	for (int i = 0; i < PKGLISTSIZE; i++)
// 	{
// 		if (i >= PKGLISTSIZE)
// 			break;
// 		pkgListType pkg = getPkgList(i);
// 		if (pkg.time > 0)
// 		{
// 			String line = String(pkg.raw);
// 			int packet = pkg.pkg;
// 			int start_val = line.indexOf(">", 0); // หาตำแหน่งแรกของ >
// 			if (start_val > 3)
// 			{
// 				String src_call = line.substring(0, start_val);
// 				memset(&aprs, 0, sizeof(pbuf_t));
// 				aprs.buf_len = 300;
// 				aprs.packet_len = line.length();
// 				line.toCharArray(&aprs.data[0], aprs.packet_len);
// 				int start_info = line.indexOf(":", 0);
// 				int end_ssid = line.indexOf(",", 0);
// 				int start_dst = line.indexOf(">", 2);
// 				int start_dstssid = line.indexOf("-", start_dst);
// 				String path = "";

// 				if ((end_ssid > start_dst) && (end_ssid < start_info))
// 				{
// 					path = line.substring(end_ssid + 1, start_info);
// 				}
// 				if (end_ssid < 5)
// 					end_ssid = start_info;
// 				if ((start_dstssid > start_dst) && (start_dstssid < start_dst + 10))
// 				{
// 					aprs.dstcall_end_or_ssid = &aprs.data[start_dstssid];
// 				}
// 				else
// 				{
// 					aprs.dstcall_end_or_ssid = &aprs.data[end_ssid];
// 				}
// 				aprs.info_start = &aprs.data[start_info + 1];
// 				aprs.dstname = &aprs.data[start_dst + 1];
// 				aprs.dstname_len = end_ssid - start_dst;
// 				aprs.dstcall_end = &aprs.data[end_ssid];
// 				aprs.srccall_end = &aprs.data[start_dst];

// 				// Serial.println(aprs.info_start);
// 				if (aprsParse.parse_aprs(&aprs))
// 				{
// 					pkg.calsign[10] = 0;
// 					// time_t tm = pkg.time;
// 					localtime_r(&pkg.time, &tmstruct);
// 					char strTime[10];
// 					sprintf(strTime, "%02d:%02d:%02d", tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
// 					// String str = String(tmstruct.tm_hour, DEC) + ":" + String(tmstruct.tm_min, DEC) + ":" + String(tmstruct.tm_sec, DEC);

// 					html += "<tr><td>" + String(strTime) + "</td>";
// 					String fileImg = "";
// 					uint8_t sym = (uint8_t)aprs.symbol[1];
// 					if (sym > 31 && sym < 127)
// 					{
// 						if (aprs.symbol[0] > 64 && aprs.symbol[0] < 91) // table A-Z
// 						{
// 							html += "<td><b>" + String(aprs.symbol[0]) + "</b></td>";
// 						}
// 						else
// 						{
// 							fileImg = String(sym, DEC);
// 							if (aprs.symbol[0] == 92)
// 							{
// 								fileImg += "-2.png";
// 							}
// 							else if (aprs.symbol[0] == 47)
// 							{
// 								fileImg += "-1.png";
// 							}
// 							else
// 							{
// 								fileImg = "dot.png";
// 							}
// 							html += "<td><img src=\"http://aprs.dprns.com/symbols/icons/" + fileImg + "\"></td>";
// 						}
// 					}
// 					else
// 					{
// 						html += "<td><img src=\"http://aprs.dprns.com/symbols/icons/dot.png\"></td>";
// 					}
// 					html += "<td>" + src_call;
// 					if (aprs.srcname_len > 0 && aprs.srcname_len < 10) // Get Item/Object
// 					{
// 						char itemname[10];
// 						memset(&itemname, 0, sizeof(itemname));
// 						memcpy(&itemname, aprs.srcname, aprs.srcname_len);
// 						html += "(" + String(itemname) + ")";
// 					}
// 					html += +"</td>";
// 					if (path == "")
// 					{
// 						html += "<td style=\"text-align: left;\">RF: DIRECT</td>";
// 					}
// 					else
// 					{
// 						String LPath = path.substring(path.lastIndexOf(',') + 1);
// 						// if(path.indexOf("qAR")>=0 || path.indexOf("qAS")>=0 || path.indexOf("qAC")>=0){ //Via from Internet Server
// 						if (path.indexOf("qA") >= 0 || path.indexOf("TCPIP") >= 0)
// 						{
// 							html += "<td style=\"text-align: left;\">INET: " + LPath + "</td>";
// 						}
// 						else
// 						{
// 							if (path.indexOf("*") > 0)
// 							{
// 								html += "<td style=\"text-align: left;\">DIGI: " + path + "</td>";
// 							}
// 							else
// 							{
// 								html += "<td style=\"text-align: left;\">RF: " + path + "</td>";
// 							}
// 						}
// 					}
// 					// html += "<td>" + path + "</td>";
// 					if (aprs.flags & F_HASPOS)
// 					{
// 						double lat, lon;
// 						if (gps.location.isValid())
// 						{
// 							lat = gps.location.lat();
// 							lon = gps.location.lng();
// 						}
// 						else
// 						{
// 							lat = config.igate_lat;
// 							lon = config.igate_lon;
// 						}
// 						double dtmp = aprsParse.direction(lon, lat, aprs.lng, aprs.lat);
// 						double dist = aprsParse.distance(lon, lat, aprs.lng, aprs.lat);
// 						html += "<td>" + String(dist, 1) + "km/" + String(dtmp, 0) + "°</td>";
// 					}
// 					else
// 					{
// 						html += "<td>-</td>\n";
// 					}
// 					html += "<td>" + String(packet) + "</td>\n";
// 					if (pkg.audio_level == 0)
// 					{
// 						html += "<td>-</td></tr>\n";
// 					}
// 					else
// 					{
// 						double Vrms = (double)pkg.audio_level / 1000;
// 						double audBV = 20.0F * log10(Vrms);
// 						if (audBV < -20.0F)
// 						{
// 							html += "<td style=\"color: #0000f0;\">";
// 						}
// 						else if (audBV > -5.0F)
// 						{
// 							html += "<td style=\"color: #f00000;\">";
// 						}
// 						else
// 						{
// 							html += "<td style=\"color: #008000;\">";
// 						}
// 						html += String(audBV, 1) + "dBV</td></tr>\n";
// 					}
// 				}
// 			}
// 		}
// 	}
// 	html += "</table>\n";
// 	if ((ESP.getFreeHeap() / 1000) > 120)
// 	{
// 		request->send(200, "text/html", html); // send to someones browser when asked
// 	}
// 	else
// 	{
// 		size_t len = html.length();
// 		char *info = (char *)calloc(len, sizeof(char));
// 		if (info)
// 		{

// 			html.toCharArray(info, len, 0);
// 			html.clear();
// 			AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)info, len);

// 			response->addHeader("Sensor", "content");
// 			request->send(response);
// 			free(info);
// 		}
// 		else
// 		{
// 			log_d("Can't define calloc info size %d", len);
// 		}
// 	}
// 	// request->send(200, "text/html", html); // send to someones browser when asked
// 	// delay(100);
// 	// html.clear();
// }

String event_lastHeard(bool gethtml)
{
	// log_d("Event count: %d",lastheard_events.count());
	//if (lastheard_events.count() == 0)
	//	return;

	struct pbuf_t aprs;
	ParseAPRS aprsParse;
	struct tm tmstruct,tmNow;

	// adcEn=-1;
	// dacEn=-1;
	// delay(20);

	String html = "";
	String line = "";
	//sort(pkgList, PKGLISTSIZE);
	time_t timeNow = time(NULL);

	// log_d("Create html last heard");
	localtime_r(&timeNow, &tmNow);

	html = "<table>\n";
	html += "<th colspan=\"7\" style=\"background-color: #070ac2;\">LAST HEARD <a href=\"/tnc2\" target=\"_tnc2\" style=\"color: yellow;font-size:8pt\">[RAW]</a></th>\n";
	html += "<tr>\n";
	html += "<th style=\"min-width:10ch\"><span><b>Time (";
	if (config.timeZone >= 0)
		html += "+";
	// else
	//	html += "-";

	if (config.timeZone == (int)config.timeZone)
		html += String((int)config.timeZone) + ")</b></span></th>\n";
	else
		html += String(config.timeZone, 1) + ")</b></span></th>\n";
	html += "<th style=\"min-width:16px\">ICON</th>\n";
	html += "<th style=\"min-width:10ch\">Callsign</th>\n";
	html += "<th>VIA LAST PATH</th>\n";
	html += "<th style=\"min-width:5ch\">DX</th>\n";
	html += "<th style=\"min-width:5ch\">PACKET</th>\n";
	html += "<th style=\"min-width:5ch\">AUDIO</th>\n";
	html += "</tr>\n";

	for (int i = 0; i < PKGLISTSIZE; i++)
	{
		if (i >= PKGLISTSIZE)
			break;
		pkgListType pkg = getPkgList(i);
		if (pkg.time > 0)
		{
			line = String(pkg.raw);
			// log_d("IDX=%d RAW:%s",i,line.c_str());
			int packet = pkg.pkg;
			int start_val = line.indexOf(">", 0); // หาตำแหน่งแรกของ >
			if (start_val > 3)
			{
				String src_call = line.substring(0, start_val);
				memset(&aprs, 0, sizeof(pbuf_t));
				aprs.buf_len = 300;
				aprs.packet_len = line.length();
				line.toCharArray(&aprs.data[0], aprs.packet_len);
				int start_info = line.indexOf(":", 0);
				int end_ssid = line.indexOf(",", 0);
				int start_dst = line.indexOf(">", 2);
				int start_dstssid = line.indexOf("-", start_dst);
				String path = "";

				if ((end_ssid > start_dst) && (end_ssid < start_info))
				{
					path = line.substring(end_ssid + 1, start_info);
				}
				if (end_ssid < 5)
					end_ssid = start_info;
				if ((start_dstssid > start_dst) && (start_dstssid < start_dst + 10))
				{
					aprs.dstcall_end_or_ssid = &aprs.data[start_dstssid];
				}
				else
				{
					aprs.dstcall_end_or_ssid = &aprs.data[end_ssid];
				}
				aprs.info_start = &aprs.data[start_info + 1];
				aprs.dstname = &aprs.data[start_dst + 1];
				aprs.dstname_len = end_ssid - start_dst;
				aprs.dstcall_end = &aprs.data[end_ssid];
				aprs.srccall_end = &aprs.data[start_dst];

				// Serial.println(aprs.info_start);
				if (aprsParse.parse_aprs(&aprs))
				{
					pkg.calsign[10] = 0;
					// time_t tm = pkg.time;
					localtime_r(&pkg.time, &tmstruct);
					char strTime[10];
					if(tmNow.tm_mday==tmstruct.tm_mday)
						sprintf(strTime, "%02d:%02d:%02d", tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
					else	
						sprintf(strTime, "%02dD %02d:%02d", tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min);
					// String str = String(tmstruct.tm_hour, DEC) + ":" + String(tmstruct.tm_min, DEC) + ":" + String(tmstruct.tm_sec, DEC);

					html += "<tr><td>" + String(strTime) + "</td>";
					String fileImg = "";
					uint8_t sym = (uint8_t)aprs.symbol[1];
					if (sym > 31 && sym < 127)
					{
						if (aprs.symbol[0] > 64 && aprs.symbol[0] < 91) // table A-Z
						{
							html += "<td><b>" + String(aprs.symbol[0]) + "</b></td>";
						}
						else
						{
							fileImg = String(sym, DEC);
							if (aprs.symbol[0] == 92)
							{
								fileImg += "-2.png";
							}
							else if (aprs.symbol[0] == 47)
							{
								fileImg += "-1.png";
							}
							else
							{
								fileImg = "dot.png";
							}
							html += "<td><img src=\"http://aprs.dprns.com/symbols/icons/" + fileImg + "\"></td>";
						}
						fileImg.clear();
					}
					else
					{
						html += "<td><img src=\"http://aprs.dprns.com/symbols/icons/dot.png\"></td>";
					}
					html += "<td>" + src_call;
					if (aprs.srcname_len > 0 && aprs.srcname_len < 10) // Get Item/Object
					{
						char itemname[10];
						memset(&itemname, 0, 10);
						memcpy(&itemname, aprs.srcname, aprs.srcname_len);
						html += "(" + String(itemname) + ")";
					}
					html += +"</td>";
					if (path == "")
					{
						html += "<td style=\"text-align: left;\">RF: DIRECT</td>";
					}
					else
					{
						String LPath = path.substring(path.lastIndexOf(',') + 1);
						// if(path.indexOf("qAR")>=0 || path.indexOf("qAS")>=0 || path.indexOf("qAC")>=0){ //Via from Internet Server
						if (path.indexOf("qA") >= 0 || path.indexOf("TCPIP") >= 0)
						{
							html += "<td style=\"text-align: left;\">INET: " + LPath + "</td>";
						}
						else
						{
							if (path.indexOf("*") > 0)
								html += "<td style=\"text-align: left;\">DIGI: " + path + "</td>";
							else
								html += "<td style=\"text-align: left;\">RF: " + path + "</td>";
						}
						LPath.clear();
					}
					// html += "<td>" + path + "</td>";
					if (aprs.flags & F_HASPOS)
					{
						double lat, lon;
						if (gps.location.isValid())
						{
							lat = gps.location.lat();
							lon = gps.location.lng();
						}
						else
						{
							lat = config.igate_lat;
							lon = config.igate_lon;
						}
						double dtmp = aprsParse.direction(lon, lat, aprs.lng, aprs.lat);
						double dist = aprsParse.distance(lon, lat, aprs.lng, aprs.lat);
						html += "<td>" + String(dist, 1) + "km/" + String(dtmp, 0) + "°</td>";
					}
					else
					{
						html += "<td>-</td>\n";
					}
					html += "<td>" + String(packet) + "</td>\n";
					if (pkg.audio_level == 0)
					{
						html += "<td>-</td></tr>\n";
					}
					else
					{
						double Vrms = (double)pkg.audio_level / 1000;
						double audBV = 20.0F * log10(Vrms);
						if (audBV < -20.0F)
						{
							html += "<td style=\"color: #0000f0;\">";
						}
						else if (audBV > -5.0F)
						{
							html += "<td style=\"color: #f00000;\">";
						}
						else
						{
							html += "<td style=\"color: #008000;\">";
						}
						html += String(audBV, 1) + "dBV</td></tr>\n";
					}
				}
				path.clear();
				src_call.clear();
			}
			line.clear();
		}
	}
	html += "</table>\n";
	// log_d("HTML Length=%d Byte",html.length());
	if(gethtml) return html;
	size_t len = html.length();
	char *info = (char *)calloc(len+1, sizeof(char));
	if (info)
	{
		memset(info,0,len+1);
		html.toCharArray(info, len, 0);
		html.clear();
		lastheard_events.send(info, "lastHeard", millis()/1000, 3000);
		free(info);
	}
	// lastheard_events.send(html.c_str(), "lastHeard", millis());
	// adcEn=1;
	// dacEn=0;
	return "";
}

String event_chatMessage(bool gethtml)
{
	// log_d("Event count: %d",lastheard_events.count());
	// if (message_events.count() == 0)
	//	return "NO";

	struct tm tmstruct, tmNow;

	String html = "";

	time_t timen = time(NULL);
	localtime_r(&timen, &tmNow);

	html += "<tr>\n";
	html += "<th style=\"width:60pt\"><span><b>Time (";
	if (config.timeZone >= 0)
		html += "+";

	if (config.timeZone == (int)config.timeZone)
		html += String((int)config.timeZone) + ")</b></span></th>\n";
	else
		html += String(config.timeZone, 1) + ")</b></span></th>\n";
	// html += "<th style=\"min-width:16px\">ICON</th>\n";

	html += "<th style=\"width:70pt\">Callsign</th>\n";
	html += "<th>Message</th>\n";
	html += "<th style=\"width:10pt\">ACK</th>\n";
	html += "<th style=\"width:20pt\">msgID</th>\n";
	html += "</tr>\n";

	pkgMsgSort(msgQueue);
	for (int i = 0; i < PKGLISTSIZE; i++)
	{
		if (i >= PKGLISTSIZE)
			break;
		msgType pkg = getMsgList(i);
		if (pkg.time > 0)
		{
			String line = String(pkg.text);

			pkg.callsign[10] = 0;
			// time_t tm = pkg.time;
			localtime_r(&pkg.time, &tmstruct);
			char strTime[10];
			// sprintf(strTime, "%02d:%02d:%02d", tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
			if (tmNow.tm_mday == tmstruct.tm_mday)
				sprintf(strTime, "%02d:%02d:%02d", tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
			else
				sprintf(strTime, "%dd %02d:%02d", tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min);
			// String str = String(tmstruct.tm_hour, DEC) + ":" + String(tmstruct.tm_min, DEC) + ":" + String(tmstruct.tm_sec, DEC);

			if (pkg.ack > 0)
			{
				html += "<tr style=\"background-color: #f1697dff;\">";
			}
			else if (pkg.ack == -1)
			{
				html += "<tr style=\"background-color: #7ff1c5ff;\">";
			}
			else if (pkg.ack == -2)
			{
				html += "<tr style=\"background-color: #73caf0ff;\">";
			}
			else
			{
				html += "<tr style=\"background-color: #f55353ff;\">";
			}
			html += "<td>" + String(strTime) + "</td>";
			html += "<td>" + String(pkg.callsign) + "</td>";
			html += "<td style=\"text-align: left;\">" + String(pkg.text) + "</td>";
			if (pkg.ack > 0)
			{
				html += "<td>" + String(pkg.ack) + "/" + String(config.msg_retry) + "</td>";
			}
			else if (pkg.ack == -1)
			{
				html += "<td>RX</td>";
			}
			else if (pkg.ack == -2)
			{
				html += "<td>TX</td>";
			}
			else
			{
				html += "<td>TF</td>";
			}
			html += "<td>" + String(pkg.msgID) + "</td></tr>";
		}
	}
	log_d("HTML Length=%d Byte gethtml:%d event_cnt:%d", html.length(),gethtml,message_events.count());
	if(gethtml) return html;
	if (message_events.count() >0){
		size_t len = html.length();
		char *info = (char *)calloc(len, sizeof(char));
		if (info)
		{
			html.toCharArray(info, len, 0);
			html.clear();
			message_events.send(info, "chatMsg", time(NULL), 5000);
			free(info);
		}	
	}
	return "";
}

void handle_storage(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	adcEn=-1;
	dacEn=-1;
	delay(100);

	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	String dirname = "/";
	char strTime[100];

	unsigned long cardTotal = LITTLEFS.totalBytes();
	unsigned long cardUsed = LITTLEFS.usedBytes();

	String webString = "<div style=\"font-size: 8pt;text-align:left;\">";
	webString += "<b>Total space: </b>";
	if (cardTotal > 1000000)
		webString += String((double)cardTotal / 1048576, 2) + " MByte ,";
	else
		webString += String((double)cardTotal / 1024, 2) + " KByte ,";
	webString += "<b>Used space: </b>";
	webString += String((double)cardUsed / 1024, 2) + " KByte";

	webString += "</br>Listing directory: </b>" + dirname + "</div>\n";

	File root = LITTLEFS.open(dirname);
	if (!root)
	{
		webString += "Failed to open directory\n";
		// return;
	}
	if (!root.isDirectory())
	{
		webString += "Not a directory";
		// return;
	}

	File file = root.openNextFile();
	webString += "<table border=\"1\"><tr align=\"center\" bgcolor=\"#03DDFC\"><td><b>DIRECTORY</b></td><td width=\"150\"><b>FILE NAME</b></td><td width=\"100\"><b>SIZE(Byte)</b></td><td width=\"170\"><b>DATE TIME</b></td><td><b>DEL</b></td></tr>";
	while (file)
	{
		if (file.isDirectory())
		{
			// webString += "<tr><td>DIR : ");
			webString += "<tr><td>" + String(file.name()) + "</td>";
			time_t t = file.getLastWrite();
			struct tm *tmstruct = localtime(&t);
			sprintf(strTime, "<td></td><td></td><td align=\"right\">%d-%02d-%02d %02d:%02d:%02d</td>", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
			webString += String(strTime);
			// if (levels) {
			//	listDir(fs, file.name(), levels - 1);
			// }
			webString += "<td></td></tr>\n";
		}
		else
		{
			/*Serial.print("  FILE: ");
			Serial.print(file.name());*/
			// String fName = String(file.name()).substring(1);
			String fName = String(file.name());
			webString += "<tr><td>/</td><td align=\"right\"><a href=\"/download?FILE=" + fName + "\" target=\"_blank\">" + fName + "</a></td>";
			// Serial.print("  SIZE: ");
			webString += "<td align=\"right\">" + String(file.size()) + "</td>";
			time_t t = file.getLastWrite();
			struct tm *tmstruct = localtime(&t);
			sprintf(strTime, "<td align=\"right\">%d-%02d-%02d %02d:%02d:%02d</td>", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
			webString += String(strTime);
			webString += "<td align=\"center\"><a href=\"/delete?FILE=" + fName + "\">X</a></td></tr>\n";
		}
		file = root.openNextFile();
	}
	webString += "</table>\n";
	webString += "<form accept-charset=\"UTF-8\" action=\"/format\" class=\"form-horizontal\" id=\"format_form\" method=\"post\">\n";
	webString += "<div><button class=\"button\" type='submit' id='format_form_sumbit'  name=\"commit\"> FORMAT </button></div>\n";
	webString += "</form><br/>\n";
	webString += "</body>\n</html>\n";
	char *info = (char *)calloc(webString.length(), sizeof(char));
	if (info)
	{
		webString.toCharArray(info, webString.length(), 0);
		webString.clear();
		request->send(200, "text/html", info); // send to someones browser when asked
		free(info);
	}
	adcEn=1;
	dacEn=0;	
}

void handle_download(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	String dataType = "";
	String path = "";

	if (request->args() > 0)
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "FILE")
			{
				path = request->arg(i);
				break;
			}
		}
	}

	if (path.endsWith(".src"))
		path = path.substring(0, path.lastIndexOf("."));
	else if (path.endsWith(".htm"))
		dataType = "text/html";
	else if (path.endsWith(".csv"))
		dataType = "text/csv";
	else if (path.endsWith(".css"))
		dataType = "text/css";
	else if (path.endsWith(".xml"))
		dataType = "text/xml";
	else if (path.endsWith(".png"))
		dataType = "image/png";
	else if (path.endsWith(".gif"))
		dataType = "image/gif";
	else if (path.endsWith(".jpg"))
		dataType = "image/jpeg";
	else if (path.endsWith(".ico"))
		dataType = "image/x-icon";
	else if (path.endsWith(".svg"))
		dataType = "image/svg+xml";
	else if (path.endsWith(".ico"))
		dataType = "image/x-icon";
	else if (path.endsWith(".js"))
		dataType = "application/javascript";
	else if (path.endsWith(".pdf"))
		dataType = "application/pdf";
	else if (path.endsWith(".zip"))
		dataType = "application/zip";
	else if (path.endsWith(".cfg"))
		dataType = "text/html";
	else if (path.endsWith(".json"))
		dataType = "application/json";
	else if (path.endsWith(".gz"))
	{
		if (path.startsWith("/gz/htm"))
			dataType = "text/html";
		else if (path.startsWith("/gz/css"))
			dataType = "text/css";
		else if (path.startsWith("/gz/csv"))
			dataType = "text/csv";
		else if (path.startsWith("/gz/xml"))
			dataType = "text/xml";
		else if (path.startsWith("/gz/js"))
			dataType = "application/javascript";
		else if (path.startsWith("/gz/svg"))
			dataType = "image/svg+xml";
		else
			dataType = "application/x-gzip";
	}

	if (path != "" && dataType != "")
	{
		String file = "/" + path;
		request->send(LITTLEFS, file, dataType, true);
		// AsyncWebServerResponse *response = request->beginResponse(LITTLEFS, file, dataType, true);
		// response->addHeader("Content-Disposition","attachment");
		// request->send(response);
	}
	else
	{
		if (dataType != "")
			request->send_P(404, PSTR("text/plain"), PSTR("ContentType Not Support"));
		else
			request->send_P(404, PSTR("text/plain"), PSTR("File Not found"));
	}
}

void handle_delete(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	String html = "FAIL";
	String dataType = "text/plain";
	String path;
	if (request->args() > 0)
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "FILE")
			{
				path = request->arg(i);
#ifdef DEBUG
				Serial.println("Deleting file: " + path);
#endif
				if (LITTLEFS.remove("/" + path))
				{
					html = "File deleted";
#ifdef DEBUG
					Serial.println("File deleted");
#endif
				}
				else
				{
					html = "Delete failed";
#ifdef DEBUG
					Serial.println("Delete failed");
#endif
				}
				break;
			}
		}
	}
	request->send(200, "text/html", html); // send to someones browser when asked
}

void handle_format(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	String html = "FAIL";
	if (request->args() > 0)
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "commit")
			{
				if (request->arg(i) == "FORMAT")
				{
					LITTLEFS.format();
					html = "OK";
					break;
				}
			}
		}
	}

	request->send(200, "text/html", html); // send to someones browser when asked
}

void handle_radio(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	// bool noiseEn=false;
	bool radioEnable = false;
	if (request->hasArg("commitRadio"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));
			if (request->argName(i) == "radioEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						radioEnable = true;
					}
				}
			}

			if (request->argName(i) == "nw_band")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.band = request->arg(i).toInt();
						// if (request->arg(i).toInt())
						// 	config.band = 1;
						// else
						// 	config.band = 0;
					}
				}
			}

			if (request->argName(i) == "volume")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.volume = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rf_power")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						if (request->arg(i).toInt())
							config.rf_power = true;
						else
							config.rf_power = false;
					}
				}
			}

			if (request->argName(i) == "sql_level")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.sql_level = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "tx_freq")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.freq_tx = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "rx_freq")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.freq_rx = request->arg(i).toFloat();
				}
			}

			if (request->argName(i) == "tx_offset")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.offset_tx = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "rx_offset")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.offset_rx = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "tx_ctcss")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tone_tx = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "rx_ctcss")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tone_rx = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "rf_type")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.rf_type = request->arg(i).toInt();
				}
			}
		}
		// config.noise=noiseEn;
		// config.agc=agcEn;
		config.rf_en = radioEnable;
		String html = "OK";
		request->send(200, "text/html", html); // send to someones browser when asked
		saveConfiguration("/default.cfg", config);
		delay(500);
		RF_MODULE(false);
	}
	else if (request->hasArg("commitTNC"))
	{
		bool hpf = 0;
		bool lpf = 0;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "HPF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						hpf = true;
					}
				}
			}
			if (request->argName(i) == "LPF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						lpf = true;
					}
				}
			}
			if (request->argName(i) == "timeSlot")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.tx_timeslot = request->arg(i).toInt();
					}
				}
			}
			if (request->argName(i) == "preamble")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.preamble = request->arg(i).toInt();
					}
				}
			}
			if (request->argName(i) == "modem_type")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.modem_type = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "fx25_mode")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.fx25_mode = request->arg(i).toInt();
				}
			}
		}
		config.audio_hpf = hpf;
		config.audio_lpf = lpf;
		String html = "OK";
		request->send(200, "text/html", html); // send to someones browser when asked
		saveConfiguration("/default.cfg", config);
		afskSetModem(config.modem_type,config.audio_lpf,config.tx_timeslot,config.preamble*100,config.fx25_mode);
	}
	else
	{
		String html = "<script type=\"text/javascript\">\n";
		html += "var sliderVol = document.getElementById(\"sliderVolume\");\n";
		html += "var outputVol = document.getElementById(\"volShow\");\n";
		html += "var sliderSql = document.getElementById(\"sliderSql\");\n";
		html += "var outputSql = document.getElementById(\"sqlShow\");\n";
		html += "outputVol.innerHTML = sliderVol.value;\n";
		html += "outputSql.innerHTML = sliderSql.value;\n";
		html += "\n";
		html += "sliderVol.oninput = function () {\n";
		html += "outputVol.innerHTML = this.value;\n";
		html += "}\n";
		html += "sliderSql.oninput = function () {\n";
		html += "outputSql.innerHTML = this.value;\n";
		html += "}\n";
		html += "\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		html += "if(e.currentTarget.id===\"formRadio\") document.getElementById(\"submitRadio\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formTNC\") document.getElementById(\"submitTNC\").disabled=true;\n";
		html += "$.ajax({\n";
		html += "url: '/radio',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "alert(\"Submited Successfully\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";
		html += "function rfType(){\n";
		html += "var type = document.getElementById(\"rf_type\").value;\n";
		html += "if(type==1||type==4||type==7){document.getElementById(\"tx_freq\").setAttribute(\"max\",174);document.getElementById(\"rx_freq\").setAttribute(\"max\",174);};\n";
		html += "if(type==1){document.getElementById(\"tx_freq\").setAttribute(\"min\",134);document.getElementById(\"rx_freq\").setAttribute(\"min\",134);};\n";
		html += "if(type==4||type==7){document.getElementById(\"tx_freq\").setAttribute(\"min\",136);document.getElementById(\"rx_freq\").setAttribute(\"min\",136);};\n";
		html += "if(type==2||type==5||type==8){document.getElementById(\"tx_freq\").setAttribute(\"max\",470);document.getElementById(\"rx_freq\").setAttribute(\"max\",470);};\n";
		html += "if(type==2||type==5||type==8){document.getElementById(\"tx_freq\").setAttribute(\"min\",400);document.getElementById(\"rx_freq\").setAttribute(\"min\",400);};\n";
		html += "if(type==3){document.getElementById(\"tx_freq\").setAttribute(\"min\",320);document.getElementById(\"rx_freq\").setAttribute(\"min\",320);};\n";
		html += "if(type==3){document.getElementById(\"tx_freq\").setAttribute(\"max\",400);document.getElementById(\"rx_freq\").setAttribute(\"max\",400);};\n";
		html += "if(type==6){document.getElementById(\"tx_freq\").setAttribute(\"min\",350);document.getElementById(\"rx_freq\").setAttribute(\"min\",350);};\n";
		html += "if(type==6){document.getElementById(\"tx_freq\").setAttribute(\"max\",390);document.getElementById(\"rx_freq\").setAttribute(\"max\",390);};\n";
		html += "if(type==1||type==4||type==7){document.getElementById(\"tx_freq\").setAttribute(\"value\",144.390);document.getElementById(\"rx_freq\").setAttribute(\"value\",144.390);};\n";
		html += "if(type==2||type==5||type==8){document.getElementById(\"tx_freq\").setAttribute(\"value\",432.5);document.getElementById(\"rx_freq\").setAttribute(\"value\",432.5);};\n";
		html += "\n";
		html += "}\n";
		html += "</script>\n";
		html += "<form id='formRadio' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>RF Analog Module</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Enable:</b></td>\n";
		String radioEnFlag = "";
		if (config.rf_en)
			radioEnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"radioEnable\" value=\"OK\" " + radioEnFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Module Type:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"rf_type\" id=\"rf_type\" onchange=\"rfType()\">\n";
		for (int i = 0; i < 10; i++)
		{
			if (config.rf_type == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(RF_TYPE[i]) + "</option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(RF_TYPE[i]) + "</option>\n";
		}
		html += "</select>\n";
		html += "</td>\n";
		float freqMin = 0;
		float freqMax = 0;
		switch (config.rf_type)
		{
		case RF_SA868_VHF:
			freqMin = 134.0F;
			freqMax = 174.0F;
			break;
		case RF_SR_1WV:
		case RF_SR_2WVS:
			freqMin = 136.0F;
			freqMax = 174.0F;
			break;
		case RF_SA868_350:
			freqMin = 320.0F;
			freqMax = 400.0F;
			break;
		case RF_SR_1W350:
			freqMin = 350.0F;
			freqMax = 390.0F;
			break;
		case RF_SA868_UHF:
		case RF_SR_1WU:
		case RF_SR_2WUS:
			freqMin = 400.0F;
			freqMax = 470.0F;
			break;
		default:
			freqMin = 134.0F;
			freqMax = 500.0F;
			break;
		}
		html += "<tr>\n";
		html += "<td align=\"right\"><b>TX Frequency:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input type=\"number\" id=\"tx_freq\" name=\"tx_freq\" min=\"" + String(freqMin, 4) + "\" max=\"" + String(freqMax, 4) + "\"\n";
		html += "step=\"0.0001\" value=\"" + String(config.freq_tx, 4) + "\" /> MHz</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>RX Frequency:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input type=\"number\" id=\"rx_freq\" name=\"rx_freq\" min=\"" + String(freqMin, 4) + "\" max=\"" + String(freqMax, 4) + "\"\n";
		html += "step=\"0.0001\" value=\"" + String(config.freq_rx, 4) + "\" /> Mhz</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>TX CTCSS:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"tx_ctcss\" id=\"tx_ctcss\">\n";
		for (int i = 0; i < 39; i++)
		{
			if (config.tone_tx == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(ctcss[i], 1) + "</option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(ctcss[i], 1) + "</option>\n";
		}
		html += "</select> Hz\n";
		html += "</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>RX CTCSS:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"rx_ctcss\" id=\"rx_ctcss\">\n";
		html += "<option value=\"0\" selected>0.0</option>\n";
		for (int i = 0; i < 39; i++)
		{
			if (config.tone_rx == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(ctcss[i], 1) + "</option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(ctcss[i], 1) + "</option>\n";
		}
		html += "</select> Hz\n";
		html += "</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Narrow/Wide:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"nw_band\" id=\"nw_band\">\n";
		String cmSelNWT = "";
		String cmSelNWF = "";
		if (config.band)
		{
			cmSelNWT = "selected";
		}
		else
		{
			cmSelNWF = "selected";
		}
		html += "<option value=\"0\" " + cmSelNWF + ">12.5KHz</option>\n";
		html += "<option value=\"1\" " + cmSelNWT + ">25.0KHz</option>\n";		
		html += "</select>\n";
		html += "</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>TX Power:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"rf_power\" id=\"rf_power\">\n";
		String cmRfPwrF = "";
		String cmRfPwrT = "";
		if (config.rf_power)
		{
			cmRfPwrT = "selected";
		}
		else
		{
			cmRfPwrF = "selected";
		}
		html += "<option value=\"1\" " + cmRfPwrT + ">HIGH</option>\n";
		html += "<option value=\"0\" " + cmRfPwrF + ">LOW</option>\n";
		html += "</select>\n";
		html += "</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>VOLUME:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input id=\"sliderVolume\" name=\"volume\" type=\"range\"\n";
		html += "min=\"1\" max=\"8\" value=\"" + String(config.volume) + "\" /><b><span style=\"font-size: 14pt;\" id=\"volShow\">" + String(config.volume) + "</span></b></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>SQL Level:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input id=\"sliderSql\" name=\"sql_level\" type=\"range\"\n";
		html += "min=\"0\" max=\"8\" value=\"" + String(config.sql_level) + "\" /><b><span style=\"font-size: 14pt;\" id=\"sqlShow\">" + String(config.sql_level) + "</span></b></td>\n";
		html += "</tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitRadio'  name=\"commitRadio\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitRadio\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form>";

		// AFSK,TNC Configuration
		html += "<form id='formTNC' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>AFSK/TNC Configuration</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Modem Type:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"modem_type\" id=\"modem_type\" \">\n";
		for (int i = 0; i < 3; i++)
		{
			if (config.modem_type == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(MODEM_TYPE[i]) + "</option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(MODEM_TYPE[i]) + "</option>\n";
		}
		html += "</select>\n";
		html += "</td>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>FX.25 Mode:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"fx25_mode\" id=\"fx25_mode\" \">\n";
		for (int i = 0; i < 3; i++)
		{
			if (config.fx25_mode == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(FX25_MODE[i]) + "</option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(FX25_MODE[i]) + "</option>\n";
		}
		html += "</select>  (FX.25 = AX.25 + FEC)\n";
		html += "</td>\n";
		html += "<tr>\n";
		// html += "<td align=\"right\"><b>Audio HPF:</b></td>\n";
		// String strFlag = "";
		// if (config.audio_hpf)
		// 	strFlag = "checked";
		// html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"HPF\" value=\"OK\" " + strFlag + "><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Audio high pass filter >1KHz cutoff 10Khz</i></label></td>\n";
		// html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Deemphasis Audio:</b></td>\n";
		String strFlag = "";
		if (config.audio_lpf)
			strFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"LPF\" value=\"OK\" " + strFlag + "><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Audio low pass filter 1hz-2.5KHz</i></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>TX Time Slot:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input type=\"number\" name=\"timeSlot\" min=\"200\" max=\"99999\"\n";
		html += "step=\"100\" value=\"" + String(config.tx_timeslot) + "\" /> mSec.</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Preamble:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"preamble\">\n";
		for (int i = 1; i < 11; i++)
		{
			if (config.preamble == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(i * 100) + "</option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(i * 100) + "</option>\n";
		}
		html += "</select> mSec.\n";
		html += "</td>\n";
		html += "</tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitTNC'  name=\"commitTNC\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitTNC\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form>";
		request->send(200, "text/html", html); // send to someones browser when asked
	}
}

void handle_vpn(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	if (request->hasArg("commitVPN"))
	{
		bool vpnEn = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "vpnEnable")
			{
				if (request->arg(i) != "")
				{
					// if (isValidNumber(request->arg(i)))
					if (String(request->arg(i)) == "OK")
						vpnEn = true;
				}
			}

			// if (request->argName(i) == "taretime") {
			//	if (request->arg(i) != "")
			//	{
			//		//if (isValidNumber(request->arg(i)))
			//		if (String(request->arg(i)) == "OK")
			//			taretime = true;
			//	}
			// }
			if (request->argName(i) == "wg_port")
			{
				if (request->arg(i) != "")
				{
					config.wg_port = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "wg_public_key")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_public_key, request->arg(i).c_str());
					config.wg_public_key[44] = 0;
				}
			}

			if (request->argName(i) == "wg_private_key")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_private_key, request->arg(i).c_str());
					config.wg_private_key[44] = 0;
				}
			}

			if (request->argName(i) == "wg_peer_address")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_peer_address, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "wg_local_address")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_local_address, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "wg_netmask_address")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_netmask_address, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "wg_gw_address")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_gw_address, request->arg(i).c_str());
				}
			}
		}

		config.vpn = vpnEn;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else
	{

		String html = "<script type=\"text/javascript\">\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		html += "if(e.currentTarget.id===\"formVPN\") document.getElementById(\"submitVPN\").disabled=true;\n";
		html += "$.ajax({\n";
		html += "url: '/vpn',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "alert(\"Submited Successfully\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";
		html += "</script>\n";

		// html += "<h2>System Setting</h2>\n";
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromVPN\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>Wireguard Configuration</b></span></th>\n";
		html += "<tr>";

		String syncFlage = "";
		if (config.vpn)
			syncFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"vpnEnable\" value=\"OK\" " + syncFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Server Address</b></td>\n";
		html += "<td style=\"text-align: left;\"><input  size=\"20\" maxlength=\"32\" id=\"wg_peer_address\" name=\"wg_peer_address\" type=\"text\" value=\"" + String(config.wg_peer_address) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Server Port</b></td>\n";
		html += "<td style=\"text-align: left;\"><input id=\"wg_port\" size=\"5\" name=\"wg_port\" type=\"number\" value=\"" + String(config.wg_port) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Local Address</b></td>\n";
		html += "<td style=\"text-align: left;\"><input id=\"wg_local_address\" name=\"wg_local_address\" type=\"text\" value=\"" + String(config.wg_local_address) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Netmask</b></td>\n";
		html += "<td style=\"text-align: left;\"><input id=\"wg_netmask_address\" name=\"wg_netmask_address\" type=\"text\" value=\"" + String(config.wg_netmask_address) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Gateway</b></td>\n";
		html += "<td style=\"text-align: left;\"><input id=\"wg_gw_address\" name=\"wg_gw_address\" type=\"text\" value=\"" + String(config.wg_gw_address) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Public Key</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"44\" id=\"wg_public_key\" name=\"wg_public_key\" type=\"text\" value=\"" + String(config.wg_public_key) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Private Key</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"44\" id=\"wg_private_key\" name=\"wg_private_key\" type=\"text\" value=\"" + String(config.wg_private_key) + "\" /></td>\n";
		html += "</tr>\n";

		html += "</table><br />\n";
		html += "<td><input class=\"button\" id=\"submitVPN\" name=\"commitVPN\" type=\"submit\" value=\"Save Config\" maxlength=\"80\"/></td>\n";
		html += "<input type=\"hidden\" name=\"commitVPN\"/>\n";
		html += "</form>\n";

		request->send(200, "text/html", html); // send to someones browser when asked
	}
}

#ifdef MQTT
void handle_mqtt(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	if (request->hasArg("commitMQTT"))
	{
		bool mqttEn = false;
		config.mqtt_topic_flag = 0;
		config.mqtt_subscribe_flag = 0;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						mqttEn = true;
				}
			}

			if (request->argName(i) == "host")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.mqtt_host, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "port")
			{
				if (request->arg(i) != "")
				{
					config.mqtt_port = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "user")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.mqtt_user, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "pass")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.mqtt_pass, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "topic")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.mqtt_topic, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "subscribe")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.mqtt_subscribe, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "TopicTNC")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.mqtt_topic_flag |= MQTT_TOPIC_TNC;
				}
			}
			if (request->argName(i) == "TopicSts")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.mqtt_topic_flag |= MQTT_TOPIC_STATUS;
				}
			}
			if (request->argName(i) == "TopicTlm")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.mqtt_topic_flag |= MQTT_TOPIC_TELEMETRY;
				}
			}
			if (request->argName(i) == "TopicWX")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.mqtt_topic_flag |= MQTT_TOPIC_WX;
				}
			}
			if (request->argName(i) == "TopicSensor")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.mqtt_topic_flag |= MQTT_TOPIC_SENSOR;
				}
			}

			if (request->argName(i) == "subCMD")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.mqtt_topic_flag |= MQTT_SUBSCRIBE_CMD;
				}
			}
			if (request->argName(i) == "subTNC")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.mqtt_topic_flag |= MQTT_SUBSCRIBE_TNC;
				}
			}
			if (request->argName(i) == "subMsg")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.mqtt_topic_flag |= MQTT_SUBSCRIBE_MESSAGE;
				}
			}
		}

		config.en_mqtt = mqttEn;
		clientMQTT.disconnect();
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else
	{

		String html = "<script type=\"text/javascript\">\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		html += "if(e.currentTarget.id===\"formVPN\") document.getElementById(\"submitMQTT\").disabled=true;\n";
		html += "$.ajax({\n";
		html += "url: '/mqtt',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "alert(\"Submited Successfully\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";
		html += "</script>\n";

		// html += "<h2>System Setting</h2>\n";
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromMQTT\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>MQTT Configuration</b></span></th>\n";
		html += "<tr>";

		String syncFlage = "";
		if (config.en_mqtt)
			syncFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"enable\" value=\"OK\" " + syncFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Server Address:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input  size=\"30\" maxlength=\"32\" name=\"host\" type=\"text\" value=\"" + String(config.mqtt_host) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Server Port:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"5\"  maxlength=\"5\"  name=\"port\" type=\"number\" value=\"" + String(config.mqtt_port) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>User:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"32\" name=\"user\" type=\"text\" value=\"" + String(config.mqtt_user) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Password:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"40\" maxlength=\"63\" name=\"pass\" type=\"password\" value=\"" + String(config.mqtt_pass) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Topic:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"32\" id=\"topic\" name=\"topic\" type=\"text\" value=\"" + String(config.mqtt_topic) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Topic Flag:</b></td>\n";

		html += "<td align=\"center\">\n";
		html += "<fieldset id=\"TopicGrp\">\n";
		html += "<legend>Topic Flags Send out MQTT</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">";
		html += "<tr style=\"background:unset;\">";

		String topicFlageEn = "";
		if (config.mqtt_topic_flag & MQTT_TOPIC_TNC)
			topicFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicTNC\" type=\"checkbox\" value=\"OK\" " + topicFlageEn + "/>TNC</td>\n";

		topicFlageEn = "";
		if (config.mqtt_topic_flag & MQTT_TOPIC_STATUS)
			topicFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicSts\" type=\"checkbox\" value=\"OK\" " + topicFlageEn + "/>Status</td>\n";

		topicFlageEn = "";
		if (config.mqtt_topic_flag & MQTT_TOPIC_TELEMETRY)
			topicFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicTlm\" type=\"checkbox\" value=\"OK\" " + topicFlageEn + "/>Telemetry</td>\n";

		topicFlageEn = "";
		if (config.mqtt_topic_flag & MQTT_TOPIC_WX)
			topicFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicWX\" type=\"checkbox\" value=\"OK\" " + topicFlageEn + "/>Weather</td>\n";

		topicFlageEn = "";
		if (config.mqtt_topic_flag & MQTT_TOPIC_SENSOR)
			topicFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicSensor\" type=\"checkbox\" value=\"OK\" " + topicFlageEn + "/>Sensor</td>\n";

		html += "<td style=\"border:unset;\"></td>";
		html += "</tr></table></fieldset>\n";
		html += "</td></tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Subscription:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"32\" name=\"subscribe\" type=\"text\" value=\"" + String(config.mqtt_subscribe) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Subscription Flag:</b></td>\n";

		html += "<td align=\"center\">\n";
		html += "<fieldset id=\"SubGrp\">\n";
		html += "<legend>Subscription Flags Receive</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">";
		html += "<tr style=\"background:unset;\">";

		String subFlageEn = "";
		if (config.mqtt_subscribe_flag & MQTT_SUBSCRIBE_CMD)
			subFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subCMD\" type=\"checkbox\" value=\"OK\" " + subFlageEn + "/>AT-Command</td>\n";

		subFlageEn = "";
		if (config.mqtt_subscribe_flag & MQTT_SUBSCRIBE_TNC)
			subFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subTNC\" type=\"checkbox\" value=\"OK\" " + subFlageEn + "/>TNC</td>\n";

		subFlageEn = "";
		if (config.mqtt_subscribe_flag & MQTT_SUBSCRIBE_MESSAGE)
			subFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subMsg\" type=\"checkbox\" value=\"OK\" " + subFlageEn + "/>Message</td>\n";

		html += "<td style=\"border:unset;\"></td>";
		html += "</tr></table></fieldset>\n";
		html += "</td></tr>\n";

		html += "</table><br />\n";
		html += "<td><input class=\"button\" id=\"submitMQTT\" name=\"commitMQTT\" type=\"submit\" value=\"Save Config\" maxlength=\"80\"/></td>\n";
		html += "<input type=\"hidden\" name=\"commitMQTT\"/>\n";
		html += "</form>\n";

		request->send(200, "text/html", html); // send to someones browser when asked
	}
}
#endif

void handle_msg(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	if (request->hasArg("commitChat"))
	{
		String toCall;
		String msg = "";
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "toCall")
			{
				if (request->arg(i) != "")
				{
					// strcpy(toCall, request->arg(i).c_str());
					toCall = request->arg(i);
				}
			}
			if (request->argName(i) == "msg")
			{
				if (request->arg(i) != "")
				{
					// strcpy(toCall, request->arg(i).c_str());
					msg = request->arg(i);
				}
			}
		}
		log_d("Chat to %s | msg %s", toCall.c_str(), msg.c_str());
		sendAPRSMessage(toCall, msg, config.msg_encrypt);
		String html = "Send completed";
		request->send(200, "text/html", html); // send to someones browser when asked
	}
	else if (request->hasArg("commitMSG"))
	{
		bool msgEn = false;
		bool msgRf = false;
		bool msgInet = false;
		bool msgEncrypt = false;

		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						msgEn = true;
				}
			}
			if (request->argName(i) == "msgRf")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						msgRf = true;
				}
			}
			if (request->argName(i) == "msgInet")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						msgInet = true;
				}
			}
			if (request->argName(i) == "encrypt")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						msgEncrypt = true;
				}
			}

			if (request->argName(i) == "mycall")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.msg_mycall, request->arg(i).c_str());
					config.msg_mycall[9] = 0;
				}
			}

			if (request->argName(i) == "key")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.msg_key, request->arg(i).c_str());
					config.msg_key[32] = 0;
				}
			}

			if (request->argName(i) == "retry")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.msg_retry = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "path")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.msg_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "timeout")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.msg_interval = request->arg(i).toInt();
				}
			}
		}

		config.msg_enable = msgEn;
		config.msg_rf = msgRf;
		config.msg_inet = msgInet;
		config.msg_encrypt = msgEncrypt;

		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else
	{

		String html = "<script type=\"text/javascript\">\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		html += "if(e.currentTarget.id===\"formMSG\") document.getElementById(\"submitMSG\").disabled=true;\n";
		// html += "if(e.currentTarget.id===\"formChat\") document.getElementById(\"submitI2C0\").disabled=true;\n";
		html += "$.ajax({\n";
		html += "url: '/msg',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "if(e.currentTarget.id===\"formMSG\") alert(\"Submited Successfully\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "if(e.currentTarget.id===\"formMSG\") alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";

		// html += "if (!!window.EventSource) {";
		// html += "var source = new EventSource('/eventMsg');";

		// html += "source.addEventListener('open', function(e) {";
		// html += "console.log(\"Events MSG Connected\");";
		// html += "}, false);";
		// html += "source.addEventListener('error', function(e) {";
		// html += "if (e.target.readyState != EventSource.OPEN) {";
		// html += "console.log(\"Events MSG Disconnected\");";
		// html += "}\n}, false);";
		// html += "source.addEventListener('chatMsg', function(e) {";
		// // webString += "console.log(\"lastHeard\", e.data);";
		// html += "var lh=document.getElementById(\"chatMsg\");";
		// html += "if(lh != null) {lh.innerHTML = e.data;}";
		// html += "}, false);\n}";
		html += "</script>\n";

		// html += "<h2>System Setting</h2>\n";
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formMSG\" method=\"post\">\n";
		html += "<table width=\"90%\">\n";
		html += "<th colspan=\"2\"><span><b>Message Configuration</b></span></th>\n";
		html += "<tr>";

		String syncFlage = "";
		if (config.msg_enable)
			syncFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"enable\" value=\"OK\" " + syncFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>My Callsign:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input  size=\"20\" maxlength=\"9\" name=\"mycall\" type=\"text\" value=\"" + String(config.msg_mycall) + "\" /> *<i>Callsign with SSID (Ex. HS5TQA-12)</i></td>\n";
		html += "</tr>\n";

		String msg2RFFlag = "";
		String msg2INETFlag = "";
		if (config.msg_rf)
			msg2RFFlag = "checked";
		if (config.msg_inet)
			msg2INETFlag = "checked";
		html += "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"msgRf\" value=\"OK\" " + msg2RFFlag + "/>RF <input type=\"checkbox\" name=\"msgInet\" value=\"OK\" " + msg2INETFlag + "/>Internet </td></tr>\n";

		html += "<tr>";
		syncFlage = "";
		if (config.msg_encrypt)
			syncFlage = "checked";
		html += "<td align=\"right\"><b>Encryption</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"encrypt\" value=\"OK\" " + syncFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>AES Key:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input  size=\"40\" maxlength=\"33\" name=\"key\" type=\"text\" value=\"" + String(config.msg_key) + "\" /> *<i>ASCII HEX 16Byte</i></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Send Retry:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input  min=\"0\" max=\"99\"   name=\"retry\" type=\"number\" value=\"" + String(config.msg_retry) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Send Timeout:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input  min=\"1\" max=\"9999\"   name=\"timeout\" type=\"number\" value=\"" + String(config.msg_interval) + "\" /> Sec.</td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>PATH:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"path\" id=\"path\">\n";
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			if (config.msg_path == pthIdx)
			{
				html += "<option value=\"" + String(pthIdx) + "\" selected>" + String(PATH_NAME[pthIdx]) + "</option>\n";
			}
			else
			{
				html += "<option value=\"" + String(pthIdx) + "\">" + String(PATH_NAME[pthIdx]) + "</option>\n";
			}
		}
		html += "</select></td>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitMSG'  name=\"commitMSG\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitMSG\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form><br /><br />";

		html += "<table width=\"90%\">\n";
		html += "<th style=\"background-color: #070ac2;\">CHAT MESSAGE</th>\n";

		html += "<tr><td>\n";
		html += "<table id=\"chatMsg\">\n";
		html += event_chatMessage(true);
		html += "</table>\n";

		html += "</td></tr><tr><td colspan=\"5\">";

		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formChat\" method=\"post\">\n";
		html += "<table>\n";

		html += "<tr>\n";
		html += "<td align=\"left\"><b>TO:</b><input size=\"10\" name=\"toCall\" id=\"toCall\" type=\"text\" value=\"\" /> <b>MSG:</b><input size=\"80\" name=\"msg\" id=\"msg\" type=\"text\" value=\"\" /></td>\n";

		html += "<td align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitChat\" name=\"commitChat\" type=\"submit\" value=\"Send\"/>\n";
		html += "<input type=\"hidden\" name=\"commitChat\"/>\n";
		html += "</td></tr></table>\n";
		html += "</form><br />\n";

		html += "</td></tr></table>";

		request->send(200, "text/html", html); // send to someones browser when asked		
	}
}

void handle_mod(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	if (request->hasArg("commitGNSS"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					// if (isValidNumber(request->arg(i)))
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "atc")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.gnss_at_command, request->arg(i).c_str());
				}
				else
				{
					memset(config.gnss_at_command, 0, sizeof(config.gnss_at_command));
				}
			}

			if (request->argName(i) == "Host")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.gnss_tcp_host, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "Port")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.gnss_tcp_port = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "channel")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.gnss_channel = request->arg(i).toInt();
				}
			}
		}

		config.gnss_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitUART0"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "baudrate")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart0_baudrate = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rx")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart0_rx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "tx")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart0_tx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rts")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart0_rts_gpio = request->arg(i).toInt();
				}
			}
		}

		config.uart0_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitUART1"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "baudrate")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart1_baudrate = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rx")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart1_rx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "tx")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart1_tx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rts")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart1_rts_gpio = request->arg(i).toInt();
				}
			}
		}

		config.uart1_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	// else if (request->hasArg("commitUART2"))
	// {
	// 	bool En = false;
	// 	for (uint8_t i = 0; i < request->args(); i++)
	// 	{
	// 		// Serial.print("SERVER ARGS ");
	// 		// Serial.print(request->argName(i));
	// 		// Serial.print("=");
	// 		// Serial.println(request->arg(i));

	// 		if (request->argName(i) == "Enable")
	// 		{
	// 			if (request->arg(i) != "")
	// 			{
	// 				if (String(request->arg(i)) == "OK")
	// 					En = true;
	// 			}
	// 		}

	// 	// 	if (request->argName(i) == "baudrate")
	// 	// 	{
	// 	// 		if (isValidNumber(request->arg(i)))
	// 	// 		{
	// 	// 			config.uart2_baudrate = request->arg(i).toInt();
	// 	// 		}
	// 	// 	}

	// 	// 	if (request->argName(i) == "rx")
	// 	// 	{
	// 	// 		if (isValidNumber(request->arg(i)))
	// 	// 		{
	// 	// 			config.uart2_rx_gpio = request->arg(i).toInt();
	// 	// 		}
	// 	// 	}

	// 	// 	if (request->argName(i) == "tx")
	// 	// 	{
	// 	// 		if (isValidNumber(request->arg(i)))
	// 	// 		{
	// 	// 			config.uart2_tx_gpio = request->arg(i).toInt();
	// 	// 		}
	// 	// 	}

	// 	// 	if (request->argName(i) == "rts")
	// 	// 	{
	// 	// 		if (isValidNumber(request->arg(i)))
	// 	// 		{
	// 	// 			config.uart2_rts_gpio = request->arg(i).toInt();
	// 	// 		}
	// 	// 	}
	// 	// }

	// 	// config.uart2_enable = En;
	// 	saveConfiguration("/default.cfg", config);
	// 	String html = "OK";
	// 	request->send(200, "text/html", html);
	// }
	else if (request->hasArg("commitMODBUS"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					// if (isValidNumber(request->arg(i)))
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "channel")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.modbus_channel = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "address")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.modbus_address = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "de")
			{
				if (request->arg(i) != "")
				{
					config.modbus_de_gpio = request->arg(i).toInt();
				}
			}
		}

		config.modbus_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitTNC"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					// if (isValidNumber(request->arg(i)))
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "channel")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.ext_tnc_channel = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "mode")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.ext_tnc_mode = request->arg(i).toInt();
				}
			}
		}

		config.ext_tnc_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitONEWIRE"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					// if (isValidNumber(request->arg(i)))
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "data")
			{
				if (request->arg(i) != "")
				{
					config.onewire_gpio = request->arg(i).toInt();
				}
			}
		}

		config.onewire_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitRF"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "sql_active")
			{
				if (request->arg(i) != "")
				{
					config.rf_sql_active = (bool)request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "pd_active")
			{
				if (request->arg(i) != "")
				{
					config.rf_pd_active = (bool)request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "pwr_active")
			{
				if (request->arg(i) != "")
				{
					config.rf_pwr_active = (bool)request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "ptt_active")
			{
				if (request->arg(i) != "")
				{
					config.rf_ptt_active = (bool)request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "baudrate")
			{
				if (request->arg(i) != "")
				{
					config.rf_baudrate = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rx")
			{
				if (request->arg(i) != "")
				{
					config.rf_rx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "tx")
			{
				if (request->arg(i) != "")
				{
					config.rf_tx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "pd")
			{
				if (request->arg(i) != "")
				{
					config.rf_pd_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "pwr")
			{
				if (request->arg(i) != "")
				{
					config.rf_pwr_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "ptt")
			{
				if (request->arg(i) != "")
				{
					config.rf_ptt_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "sql")
			{
				if (request->arg(i) != "")
				{
					config.rf_sql_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "atten")
			{
				if (request->arg(i) != "")
				{
					config.adc_atten = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "offset")
			{
				if (request->arg(i) != "")
				{
					config.adc_dc_offset = request->arg(i).toInt();
				}
			}
		}
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitI2C0"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "sda")
			{
				if (request->arg(i) != "")
				{
					config.i2c_sda_pin = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "sck")
			{
				if (request->arg(i) != "")
				{
					config.i2c_sck_pin = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "freq")
			{
				if (request->arg(i) != "")
				{
					config.i2c_freq = request->arg(i).toInt();
				}
			}
		}

		config.i2c_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitI2C1"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "sda")
			{
				if (request->arg(i) != "")
				{
					config.i2c1_sda_pin = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "sck")
			{
				if (request->arg(i) != "")
				{
					config.i2c1_sck_pin = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "freq")
			{
				if (request->arg(i) != "")
				{
					config.i2c1_freq = request->arg(i).toInt();
				}
			}
		}

		config.i2c1_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitCOUNTER0"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "gpio")
			{
				if (request->arg(i) != "")
				{
					config.counter0_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "active")
			{
				if (request->arg(i) != "")
				{
					config.counter0_active = (bool)request->arg(i).toInt();
				}
			}
		}

		config.counter0_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitCOUNTER1"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "gpio")
			{
				if (request->arg(i) != "")
				{
					config.counter1_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "active")
			{
				if (request->arg(i) != "")
				{
					config.counter1_active = (bool)request->arg(i).toInt();
				}
			}
		}

		config.counter0_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}else if (request->hasArg("commitCMD"))
	{
		bool mqtt = false;
		bool msg = false;
		bool bluetooth = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "mqtt")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						mqtt = true;
					}
				}
			}

			if (request->argName(i) == "msg")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						msg = true;
					}
				}
			}

			if (request->argName(i) == "bluetooth")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						bluetooth = true;
					}
				}
			}

			if (request->argName(i) == "uart")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.at_cmd_uart = request->arg(i).toInt();
				}
			}
		}
		config.at_cmd_mqtt = mqtt;
		config.at_cmd_msg = msg;
		config.at_cmd_bluetooth = bluetooth;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	#ifdef PPPOS
	else if (request->hasArg("commitPPPoS"))
	{
		bool pppEn = false;
		bool pppGnss = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "pppEn")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						pppEn = true;
					}
				}
			}

			if (request->argName(i) == "pppGnss")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						pppGnss = true;
					}
				}
			}

			if (request->argName(i) == "pppAPN")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.ppp_apn, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "pppPin")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.ppp_pin, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "rstDly")
			{
				if (request->arg(i) != "")
				{
					config.ppp_rst_delay = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "baudrate")
			{
				if (request->arg(i) != "")
				{
					config.ppp_serial_baudrate = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "port")
			{
				if (request->arg(i) != "")
				{
					config.ppp_serial = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rx")
			{
				if (request->arg(i) != "")
				{
					config.ppp_rx_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "tx")
			{
				if (request->arg(i) != "")
				{
					config.ppp_tx_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "rst")
			{
				if (request->arg(i) != "")
				{
					config.ppp_rst_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "rst_active")
			{
				if (request->arg(i) != "")
				{
					config.ppp_rst_active = (bool)request->arg(i).toInt();
				}
			}

			// if (request->argName(i) == "pppSerial")
			// {
			// 	if (request->arg(i) != "")
			// 	{
			// 		if (isValidNumber(request->arg(i)))
			// 			config.ppp_serial = request->arg(i).toInt();
			// 	}
			// }
		}
		config.ppp_enable = pppEn;
		config.ppp_gnss = pppGnss;
		if (config.ppp_enable)
		{
			if (config.ppp_serial == 0)
			{
				config.uart0_enable = false;
			}
			else if (config.ppp_serial == 1)
			{
				config.uart1_enable = false;
			}
			// else if (config.ppp_serial == 2)
			// {
			// 	config.uart2_enable = false;
			// }
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	#endif
	else
	{

		String html = "<script type=\"text/javascript\">\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		html += "if(e.currentTarget.id===\"formUART0\") document.getElementById(\"submitURAT0\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formUART1\") document.getElementById(\"submitURAT1\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formUART1\") document.getElementById(\"submitURAT1\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formGNSS\") document.getElementById(\"submitGNSS\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formMODBUS\") document.getElementById(\"submitMODBUS\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formTNC\") document.getElementById(\"submitTNC\").disabled=true;\n";
		// html += "if(e.currentTarget.id===\"formONEWIRE\") document.getElementById(\"submitONEWIRE\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formRF\") document.getElementById(\"submitRF\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formI2C0\") document.getElementById(\"submitI2C0\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formI2C1\") document.getElementById(\"submitI2C1\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formCOUNT0\") document.getElementById(\"submitCOUNT0\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formCOUNT1\") document.getElementById(\"submitCOUNT1\").disabled=true;\n";
		#ifdef PPPOS
		html += "if(e.currentTarget.id===\"formPPPoS\") document.getElementById(\"submitPPPoS\").disabled=true;\n";
		#endif
		html += "$.ajax({\n";
		html += "url: '/mod',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "alert(\"Submited Successfully\\nRequire hardware RESET!\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";
		html += "</script>\n";

		html += "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"32%\" style=\"border:unset;\">";
		// html += "<h2>System Setting</h2>\n";
		/**************UART0(USB) Modify******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromUART0\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>UART0 Modify</b></span></th>\n";
		html += "<tr>";

		String enFlage = "";
		if (config.uart0_enable)
			enFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + enFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>RX GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"rx\" type=\"number\" value=\"" + String(config.uart0_rx_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>TX GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"tx\" type=\"number\" value=\"" + String(config.uart0_tx_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>RTS/DE GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\"  name=\"rts\" type=\"number\" value=\"" + String(config.uart0_rts_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Baudrate:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"baudrate\" id=\"baudrate\">\n";
		for (int i = 0; i < 13; i++)
		{
			if (config.uart0_baudrate == baudrate[i])
				html += "<option value=\"" + String(baudrate[i]) + "\" selected>" + String(baudrate[i]) + " </option>\n";
			else
				html += "<option value=\"" + String(baudrate[i]) + "\" >" + String(baudrate[i]) + " </option>\n";
		}
		html += "</select> bps\n";
		html += "</td>\n";
		html += "</tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitUART0\" name=\"commitUART0\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		html += "<input type=\"hidden\" name=\"commitUART0\"/>\n";
		html += "</td></tr></table>\n";

		html += "</form><br />\n";
		html += "</td><td width=\"32%\" style=\"border:unset;\">";

		/**************UART1 Modify******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromUART1\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>UART1 Modify</b></span></th>\n";
		html += "<tr>";

		enFlage = "";
		if (config.uart1_enable)
			enFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + enFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>RX GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"rx\" type=\"number\" value=\"" + String(config.uart1_rx_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>TX GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"tx\" type=\"number\" value=\"" + String(config.uart1_tx_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>RTS/DE GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\"  name=\"rts\" type=\"number\" value=\"" + String(config.uart1_rts_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Baudrate:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"baudrate\" id=\"baudrate\">\n";
		for (int i = 0; i < 13; i++)
		{
			if (config.uart1_baudrate == baudrate[i])
				html += "<option value=\"" + String(baudrate[i]) + "\" selected>" + String(baudrate[i]) + " </option>\n";
			else
				html += "<option value=\"" + String(baudrate[i]) + "\" >" + String(baudrate[i]) + " </option>\n";
		}
		html += "</select> bps\n";
		html += "</td>\n";
		html += "</tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitUART1\" name=\"commitUART1\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		html += "<input type=\"hidden\" name=\"commitUART1\"/>\n";
		html += "</td></tr></table>\n";

		html += "</form><br />\n";
		//html += "</td><td width=\"32%\" style=\"border:unset;\">";

		/**************UART2 Modify******************/
		// html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromUART2\" method=\"post\">\n";
		// html += "<table>\n";
		// html += "<th colspan=\"2\"><span><b>UART2 Modify</b></span></th>\n";
		// html += "<tr>";

		// enFlage = "";
		// if (config.uart2_enable)
		// 	enFlage = "checked";
		// html += "<td align=\"right\"><b>Enable</b></td>\n";
		// html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + enFlage + "><span class=\"slider round\"></span></label></td>\n";
		// html += "</tr>\n";

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>RX GPIO:</b></td>\n";
		// html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"rx\" type=\"number\" value=\"" + String(config.uart2_rx_gpio) + "\" /></td>\n";
		// html += "</tr>\n";

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>TX GPIO:</b></td>\n";
		// html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"tx\" type=\"number\" value=\"" + String(config.uart2_tx_gpio) + "\" /></td>\n";
		// html += "</tr>\n";

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>RTS/DE GPIO:</b></td>\n";
		// html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\"  name=\"rts\" type=\"number\" value=\"" + String(config.uart2_rts_gpio) + "\" /></td>\n";
		// html += "</tr>\n";

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>Baudrate:</b></td>\n";
		// html += "<td style=\"text-align: left;\">\n";
		// html += "<select name=\"baudrate\" id=\"baudrate\">\n";
		// for (int i = 0; i < 13; i++)
		// {
		// 	if (config.uart2_baudrate == baudrate[i])
		// 		html += "<option value=\"" + String(baudrate[i]) + "\" selected>" + String(baudrate[i]) + " </option>\n";
		// 	else
		// 		html += "<option value=\"" + String(baudrate[i]) + "\" >" + String(baudrate[i]) + " </option>\n";
		// }
		// html += "</select> bps\n";
		// html += "</td>\n";
		// html += "</tr>\n";
		// html += "<tr><td colspan=\"2\" align=\"right\">\n";
		// html += "<input class=\"btn btn-primary\" id=\"submitUART2\" name=\"commitUART2\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		// html += "<input type=\"hidden\" name=\"commitUART2\"/>\n";
		// html += "</td></tr></table>\n";

		// html += "</form><br />\n";
		// html += "</td></tr></table>\n";

		html += "</td><td width=\"32%\" style=\"border:unset;\">";

		/**************1-Wire Modify******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromONEWIRE\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>1-Wire Bus Modify</b></span></th>\n";
		html += "<tr>";

		String syncFlage = "";
		if (config.onewire_enable)
			syncFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + syncFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"data\" type=\"number\" value=\"" + String(config.onewire_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitONEWIRE\" name=\"commitONEWIRE\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		html += "<input type=\"hidden\" name=\"commitONEWIRE\"/>\n";
		html += "</td></tr></table>\n";
		html += "</form><br />\n";

		html += "</td></tr></table>\n";

		html += "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">";
		/**************RF GPIO******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromRF\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>RF GPIO Modify</b></span></th>\n";
		html += "<tr>";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>ADC Attenuation:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"atten\" id=\"atten\">\n";
		for (int i = 0; i < 5; i++)
		{
			if (config.adc_atten == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(ADC_ATTEN[i]) + " </option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(ADC_ATTEN[i]) + " </option>\n";
		}
		html += "</select> DC-Offset: " + String(offset) + "mV\n";
		html += "</td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>UART2 Baudrate:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"baudrate\" id=\"baudrate\">\n";
		for (int i = 0; i < 13; i++)
		{
			if (config.rf_baudrate == baudrate[i])
				html += "<option value=\"" + String(baudrate[i]) + "\" selected>" + String(baudrate[i]) + " </option>\n";
			else
				html += "<option value=\"" + String(baudrate[i]) + "\" >" + String(baudrate[i]) + " </option>\n";
		}
		html += "</select> bps\n";
		html += "</td>\n";
		html += "</tr>\n";		

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>ADC DC OFFSET:</b></td>\n";
		// html += "<td style=\"text-align: left;\"><input min=\"100\" max=\"2500\" name=\"offset\" type=\"number\" value=\"" + String(config.adc_dc_offset) + "\" /> mV     (Current: " + String(offset) + " mV)</td>\n";
		// html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>UART2 RX GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"rx\" type=\"number\" value=\"" + String(config.rf_rx_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>UART2 TX GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"tx\" type=\"number\" value=\"" + String(config.rf_tx_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		String LowFlag = "", HighFlag = "";
		LowFlag = "";
		HighFlag = "";
		if (config.rf_pd_active)
			HighFlag = "checked=\"checked\"";
		else
			LowFlag = "checked=\"checked\"";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PD GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\"  name=\"pd\" type=\"number\" value=\"" + String(config.rf_pd_gpio) + "\" /> Active:<input type=\"radio\" name=\"pd_active\" value=\"0\" " + LowFlag + "/>LOW <input type=\"radio\" name=\"pd_active\" value=\"1\" " + HighFlag + "/>HIGH </td>\n";
		html += "</tr>\n";

		LowFlag = "";
		HighFlag = "";
		if (config.rf_pwr_active)
			HighFlag = "checked=\"checked\"";
		else
			LowFlag = "checked=\"checked\"";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>H/L GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\"  name=\"pwr\" type=\"number\" value=\"" + String(config.rf_pwr_gpio) + "\" /> Active:<input type=\"radio\" name=\"pwr_active\" value=\"0\" " + LowFlag + "/>LOW <input type=\"radio\" name=\"pwr_active\" value=\"1\" " + HighFlag + "/>HIGH </td>\n";
		html += "</tr>\n";

		LowFlag = "";
		HighFlag = "";
		if (config.rf_sql_active)
			HighFlag = "checked=\"checked\"";
		else
			LowFlag = "checked=\"checked\"";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>SQL GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\"  name=\"sql\" type=\"number\" value=\"" + String(config.rf_sql_gpio) + "\" /> Active:<input type=\"radio\" name=\"sql_active\" value=\"0\" " + LowFlag + "/>LOW <input type=\"radio\" name=\"sql_active\" value=\"1\" " + HighFlag + "/>HIGH </td>\n";
		html += "</tr>\n";

		LowFlag = "";
		HighFlag = "";
		if (config.rf_ptt_active)
			HighFlag = "checked=\"checked\"";
		else
			LowFlag = "checked=\"checked\"";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PTT GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\"  name=\"ptt\" type=\"number\" value=\"" + String(config.rf_ptt_gpio) + "\" /> Active:<input type=\"radio\" name=\"ptt_active\" value=\"0\" " + LowFlag + "/>LOW <input type=\"radio\" name=\"ptt_active\" value=\"1\" " + HighFlag + "/>HIGH </td>\n";
		html += "</tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitRF\" name=\"commitRF\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		html += "<input type=\"hidden\" name=\"commitRF\"/>\n";
		html += "</td></tr></table>\n";
		html += "</form>\n";

		html += "</td><td width=\"23%\" style=\"border:unset;\">";

		/**************I2C_0 Modify******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromI2C0\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>I2C_0(OLED) Modify</b></span></th>\n";
		html += "<tr>";

		syncFlage = "";
		if (config.i2c_enable)
			syncFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + syncFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>SDA GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"sda\" type=\"number\" value=\"" + String(config.i2c_sda_pin) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>SCK GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"sck\" type=\"number\" value=\"" + String(config.i2c_sck_pin) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Frequency:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"1000\" max=\"800000\" name=\"freq\" type=\"number\" value=\"" + String(config.i2c_freq) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitI2C0\" name=\"commitI2C0\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		html += "<input type=\"hidden\" name=\"commitI2C0\"/>\n";
		html += "</td></tr></table>\n";
		html += "</form>\n";

		/**************Counter_0 Modify******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromCOUNTER0\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>Counter_0 Modify</b></span></th>\n";
		html += "<tr>";

		syncFlage = "";
		if (config.counter0_enable)
			syncFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + syncFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>INPUT GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"gpio\" type=\"number\" value=\"" + String(config.counter0_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		LowFlag = "";
		HighFlag = "";
		if (config.counter0_active)
			HighFlag = "checked=\"checked\"";
		else
			LowFlag = "checked=\"checked\"";
		html += "<tr>\n";
		html += "<td align=\"right\">Active</td>\n";
		html += "<td style=\"text-align: left;\"><input type=\"radio\" name=\"active\" value=\"0\" " + LowFlag + "/>LOW <input type=\"radio\" name=\"active\" value=\"1\" " + HighFlag + "/>HIGH </td>\n";
		html += "</tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitCOUNTER0\" name=\"commitCOUNTER0\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		html += "<input type=\"hidden\" name=\"commitCOUNTER0\"/>\n";
		html += "</td></tr></table>\n";
		html += "</form>\n";

		html += "</td><td width=\"23%\" style=\"border:unset;\">";
		/**************I2C_1 Modify******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromI2C1\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>I2C_1 Modify</b></span></th>\n";
		html += "<tr>";

		syncFlage = "";
		if (config.i2c1_enable)
			syncFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + syncFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>SDA GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"sda\" type=\"number\" value=\"" + String(config.i2c1_sda_pin) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>SCK GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"sck\" type=\"number\" value=\"" + String(config.i2c1_sck_pin) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Frequency:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"1000\" max=\"800000\" name=\"freq\" type=\"number\" value=\"" + String(config.i2c1_freq) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitI2C1\" name=\"commitI2C1\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		html += "<input type=\"hidden\" name=\"commitI2C1\"/>\n";
		html += "</td></tr></table>\n";
		html += "</form>\n";

		/**************Counter_1 Modify******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromCOUNTER1\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>Counter_1 Modify</b></span></th>\n";
		html += "<tr>";

		syncFlage = "";
		if (config.counter1_enable)
			syncFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + syncFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>INPUT GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"gpio\" type=\"number\" value=\"" + String(config.counter1_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		LowFlag = "";
		HighFlag = "";
		if (config.counter1_active)
			HighFlag = "checked=\"checked\"";
		else
			LowFlag = "checked=\"checked\"";
		html += "<tr>\n";
		html += "<td align=\"right\">Active</td>\n";
		html += "<td style=\"text-align: left;\"><input type=\"radio\" name=\"active\" value=\"0\" " + LowFlag + "/>LOW <input type=\"radio\" name=\"active\" value=\"1\" " + HighFlag + "/>HIGH </td>\n";
		html += "</tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitCOUNTER1\" name=\"commitCOUNTER1\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		html += "<input type=\"hidden\" name=\"commitCOUNTER1\"/>\n";
		html += "</td></tr></table>\n";
		html += "</form>\n";

		html += "</td></tr></table>\n";
		html += "<br />\n";

		//******************
		html += "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">";
		/**************GNSS Modify******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromGNSS\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>GNSS Modify</b></span></th>\n";
		html += "<tr>";

		enFlage = "";
		if (config.gnss_enable)
			enFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + enFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>PORT:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"channel\" id=\"channel\">\n";
		for (int i = 0; i < 5; i++)
		{
			if (config.gnss_channel == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(GNSS_PORT[i]) + " </option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(GNSS_PORT[i]) + " </option>\n";
		}
		html += "</select>\n";
		html += "</td>\n";
		html += "</tr>\n";

		html += "<td align=\"right\"><b>AT Command:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"30\" size=\"20\" id=\"atc\" name=\"atc\" type=\"text\" value=\"" + String(config.gnss_at_command) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<td align=\"right\"><b>TCP Host:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"20\" size=\"15\" id=\"Host\" name=\"Host\" type=\"text\" value=\"" + String(config.gnss_tcp_host) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>TCP Port:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"1024\" max=\"65535\"  id=\"Port\" name=\"Port\" type=\"number\" value=\"" + String(config.gnss_tcp_port) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitGNSS\" name=\"commitGNSS\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		html += "<input type=\"hidden\" name=\"commitGNSS\"/>\n";
		html += "</td></tr></table>\n";

		html += "</form><br />\n";

		html += "</td><td width=\"23%\" style=\"border:unset;\">";

		/**************MODBUS Modify******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromMODBUS\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>MODBUS Modify</b></span></th>\n";
		html += "<tr>";

		enFlage = "";
		if (config.modbus_enable)
			enFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + enFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>PORT:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"channel\" id=\"channel\">\n";
		for (int i = 0; i < 5; i++)
		{
			if (config.modbus_channel == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(GNSS_PORT[i]) + " </option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(GNSS_PORT[i]) + " </option>\n";
		}
		html += "</select>\n";
		html += "</td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Address:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"address\" type=\"number\" value=\"" + String(config.modbus_address) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>DE:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"de\" type=\"number\" value=\"" + String(config.modbus_de_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitMODBUS\" name=\"commitMODBUS\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		html += "<input type=\"hidden\" name=\"commitMODBUS\"/>\n";
		html += "</td></tr></table>\n";
		html += "</form>\n";

		html += "</td><td width=\"23%\" style=\"border:unset;\">";

		/**************External TNC Modify******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromTNC\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>External TNC Modify</b></span></th>\n";
		html += "<tr>";

		enFlage = "";
		if (config.ext_tnc_enable)
			enFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + enFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>PORT:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"channel\" id=\"channel\">\n";
		for (int i = 0; i < 5; i++)
		{
			if (config.ext_tnc_channel == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(TNC_PORT[i]) + " </option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(TNC_PORT[i]) + " </option>\n";
		}
		html += "</select>\n";
		html += "</td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>MODE:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"mode\" id=\"mode\">\n";
		for (int i = 0; i < 4; i++)
		{
			if (config.ext_tnc_mode == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(TNC_MODE[i]) + " </option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(TNC_MODE[i]) + " </option>\n";
		}
		html += "</select>\n";
		html += "</td>\n";
		html += "</tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<input class=\"button\" id=\"submitTNC\" name=\"commitTNC\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		html += "<input type=\"hidden\" name=\"commitTNC\"/>\n";
		html += "</td></tr></table>\n";
		html += "</form>\n";
		html += "</td></tr></table>\n";
		html += "<br />\n";

		html += "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">";

		/************************ AT-COMMAND **************************/
		html += "<form id='formATCommand' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>AT-COMMAND CHANNEL</b></span></th>\n";
		html += "<tr>\n";
		html += "<td width=\"150\" align=\"right\"><b>MQTT:</b></td>\n";
		String cmdFlag = "";
		if (config.at_cmd_mqtt)
			cmdFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"mqtt\" value=\"OK\" " + cmdFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>MESSAGE:</b></td>\n";
		cmdFlag = "";
		if (config.at_cmd_msg)
			cmdFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"msg\" value=\"OK\" " + cmdFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>BLUETOOTH:</b></td>\n";
		cmdFlag = "";
		if (config.at_cmd_bluetooth)
			cmdFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"bluetooth\" value=\"OK\" " + cmdFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>UART PORT:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"uart\" id=\"cmdUart\">\n";
		for (int i = 0; i < 5; i++)
		{
			if (config.at_cmd_uart == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(TNC_PORT[i]) + " </option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(TNC_PORT[i]) + " </option>\n";
		}
		html += "</select>\n";
		html += "</td>\n";
		html += "</tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitCMD'  name=\"commit\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitCMD\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form><br />";

		#ifdef PPPOS
		html += "<br />\n";

		html += "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">";

		/************************ PPPoS **************************/

		html += "<form id='formPPPoS' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>PPP Over Serial (GSM/4G-LTE)</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Enable:</b></td>\n";
		String pppEnFlag = "";
		if (config.ppp_enable)
			pppEnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"pppEn\" value=\"OK\" " + pppEnFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<td align=\"right\"><b>GNSS:</b></td>\n";
		pppEnFlag = "";
		if (config.ppp_gnss)
			pppEnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"pppGnss\" value=\"OK\" " + pppEnFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>NAPT:</b></td>\n";
		pppEnFlag = "";
		if (config.ppp_napt)
			pppEnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"pppNapt\" value=\"OK\" " + pppEnFlag + "><span class=\"slider round\"></span></label> *WiFi NAT</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>APN:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"20\" name=\"pppAPN\" type=\"text\" value=\"" + String(config.ppp_apn) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";

		html += "<td align=\"right\"><b>PIN:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"0\" max=\"999999\" name=\"pppPin\" type=\"number\" value=\"" + String(config.ppp_pin) + "\" /> <i>*PIN of SIM</i></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";

		html += "<td align=\"right\"><b>RX GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\" name=\"rx\" type=\"number\" value=\"" + String(config.ppp_rx_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>TX GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\" name=\"tx\" type=\"number\" value=\"" + String(config.ppp_tx_gpio) + "\" /></td>\n";
		html += "</tr>\n";

		LowFlag = "";
		HighFlag = "";
		if (config.ppp_rst_active)
			HighFlag = "checked=\"checked\"";
		else
			LowFlag = "checked=\"checked\"";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>RESET GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\"  name=\"rst\" type=\"number\" value=\"" + String(config.ppp_rst_gpio) + "\" /> Active:<input type=\"radio\" name=\"rst_active\" value=\"0\" " + LowFlag + "/>LOW <input type=\"radio\" name=\"rst_active\" value=\"1\" " + HighFlag + "/>HIGH </td>\n";
		html += "</tr>\n";

		html += "<td align=\"right\"><b>RESET DELAY:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"0\" max=\"999999\" name=\"rstDly\" type=\"number\" value=\"" + String(config.ppp_rst_delay, DEC) + "\" /> mSec.</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>PORT:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"port\" id=\"port\">\n";
		for (int i = 0; i < 2; i++)
		{
			if (config.ppp_serial == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(GNSS_PORT[i + 1]) + " </option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(GNSS_PORT[i + 1]) + " </option>\n";
		}
		html += "</select>\n";
		html += "</td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Baudrate:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"baudrate\" id=\"baudrate\">\n";
		for (int i = 0; i < 13; i++)
		{
			if (config.ppp_serial_baudrate == baudrate[i])
				html += "<option value=\"" + String(baudrate[i]) + "\" selected>" + String(baudrate[i]) + " </option>\n";
			else
				html += "<option value=\"" + String(baudrate[i]) + "\" >" + String(baudrate[i]) + " </option>\n";
		}
		html += "</select> bps\n";
		html += "</td>\n";
		html += "</tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitPPPoS'  name=\"commitPPPoS\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitPPPoS\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form>";

		html += "</td></tr></table>\n";
		#endif
		if ((ESP.getFreeHeap() / 1000) > 120)
		{
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			size_t len = html.length();
			char *info = (char *)calloc(len, sizeof(char));
			if (info)
			{

				html.toCharArray(info, len, 0);
				html.clear();
				AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)info, len);

				response->addHeader("Sensor", "content");
				request->send(response);
				free(info);
			}
			else
			{
				log_d("Can't define calloc info size %d", len);
			}
		}
		//request->send(200, "text/html", html); // send to someones browser when asked
	}
}

void handle_system(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	if (request->hasArg("updateTimeZone"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "SetTimeZone")
			{
				if (request->arg(i) != "")
				{
					config.timeZone = request->arg(i).toFloat();
					// Serial.println("WEB Config Time Zone);
					configTime(3600 * config.timeZone, 0, config.ntp_host);
				}
				break;
			}
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}else if (request->hasArg("updateHostName"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "SetHostName")
			{
				if (request->arg(i) != "")
				{
					// Serial.println("WEB Config NTP");
					strcpy(config.host_name, request->arg(i).c_str());
				}
				break;
			}
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else if (request->hasArg("updateTimeNtp"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));
			if (request->argName(i) == "SetTimeNtp")
			{
				if (request->arg(i) != "")
				{
					// Serial.println("WEB Config NTP");
					strcpy(config.ntp_host, request->arg(i).c_str());
					configTime(3600 * config.timeZone, 0, config.ntp_host);
				}
				break;
			}
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}else if (request->hasArg("updateAutoReset"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{

			if (request->argName(i) == "SetAutoReset")
			{
				if (request->arg(i) != "")
				{
					config.reset_timeout = request->arg(i).toInt();
				}
				break;
			}
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else if (request->hasArg("updateTime"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));
			if (request->argName(i) == "SetTime")
			{
				if (request->arg(i) != "")
				{
					// struct tm tmn;
					String date = getValue(request->arg(i), ' ', 0);
					String time = getValue(request->arg(i), ' ', 1);
					int yyyy = getValue(date, '-', 0).toInt();
					int mm = getValue(date, '-', 1).toInt();
					int dd = getValue(date, '-', 2).toInt();
					int hh = getValue(time, ':', 0).toInt();
					int ii = getValue(time, ':', 1).toInt();
					int ss = getValue(time, ':', 2).toInt();
					// int ss = 0;

					tmElements_t timeinfo;
					timeinfo.Year = yyyy - 1970;
					timeinfo.Month = mm;
					timeinfo.Day = dd;
					timeinfo.Hour = hh;
					timeinfo.Minute = ii;
					timeinfo.Second = ss;
					time_t timeStamp = makeTime(timeinfo);

					// tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec

					time_t rtc = timeStamp - (config.timeZone * 3600);
					timeval tv = {rtc, 0};
					timezone tz = {(0) + DST_MN, 0};
					settimeofday(&tv, &tz);

					// Serial.println("Update TIME " + request->arg(i));
					Serial.print("Set New Time at ");
					Serial.print(dd);
					Serial.print("/");
					Serial.print(mm);
					Serial.print("/");
					Serial.print(yyyy);
					Serial.print(" ");
					Serial.print(hh);
					Serial.print(":");
					Serial.print(ii);
					Serial.print(":");
					Serial.print(ss);
					Serial.print(" ");
					Serial.println(timeStamp);
				}
				break;
			}
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else if (request->hasArg("REBOOT"))
	{
		TLM_SEQ = 0;
		IGATE_TLM_SEQ = 0;
		DIGI_TLM_SEQ = 0;
		esp_restart();
	}
	else if (request->hasArg("Factory"))
	{
		defaultConfig();
	}
	else if (request->hasArg("LoadCFG"))
	{
		if (loadConfiguration("/default.cfg", config))
		{
			String html = "OK";
			request->send(200, "text/html", html);
		}
	}
	else if (request->hasArg("commitWebAuth"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));
			if (request->argName(i) == "webauth_user")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.http_username, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "webauth_pass")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.http_password, request->arg(i).c_str());
				}
			}
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else if (request->hasArg("commitPath"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));
			if (request->argName(i) == "path1")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.path[0], request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "path2")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.path[1], request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "path3")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.path[1], request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "path4")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.path[3], request->arg(i).c_str());
				}
			}
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else if (request->hasArg("commitPWR"))
	{
		bool PwrEn = false;
		config.pwr_sleep_activate = 0;

		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));
			if (request->argName(i) == "pwr_active")
			{
				if (request->arg(i) != "")
				{
					config.pwr_active = (bool)request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						PwrEn = true;
					}
				}
			}
			if (request->argName(i) == "pwr")
			{
				if (request->arg(i) != "")
				{
					config.pwr_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "sleep")
			{
				if (request->arg(i) != "")
				{
					config.pwr_sleep_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "stb")
			{
				if (request->arg(i) != "")
				{
					config.pwr_stanby_delay = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "mode")
			{
				if (request->arg(i) != "")
				{
					config.pwr_mode = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "FilterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.pwr_sleep_activate |= ACTIVATE_TELEMETRY;
				}
			}

			if (request->argName(i) == "FilterStatus")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.pwr_sleep_activate |= ACTIVATE_STATUS;
				}
			}

			if (request->argName(i) == "FilterWeather")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.pwr_sleep_activate |= ACTIVATE_WX;
				}
			}

			if (request->argName(i) == "FilterTracker")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.pwr_sleep_activate |= ACTIVATE_TRACKER;
				}
			}

			if (request->argName(i) == "FilterIGate")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.pwr_sleep_activate |= ACTIVATE_IGATE;
				}
			}

			if (request->argName(i) == "FilterDigi")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.pwr_sleep_activate |= ACTIVATE_DIGI;
				}
			}

			if (request->argName(i) == "FilterQuery")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.pwr_sleep_activate |= ACTIVATE_QUERY;
				}
			}

			if (request->argName(i) == "FilterWifi")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.pwr_sleep_activate |= ACTIVATE_WIFI;
				}
			}
		}
		config.pwr_en = PwrEn;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else if (request->hasArg("commitLOG"))
	{
		bool PwrEn = false;
		config.log = 0;

		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "logStatus")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.log |= LOG_STATUS;
				}
			}

			if (request->argName(i) == "logWeather")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.log |= LOG_WX;
				}
			}

			if (request->argName(i) == "logTracker")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.log |= LOG_TRACKER;
				}
			}

			if (request->argName(i) == "logIgate")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.log |= LOG_IGATE;
				}
			}

			if (request->argName(i) == "logDigi")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.log |= LOG_DIGI;
				}
			}
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else if (request->hasArg("commitDISP"))
	{
		bool dispRX = false;
		bool dispTX = false;
		bool dispRF = false;
		bool dispINET = false;
		bool oledEN = false;
		bool dispFlip = false;

		config.dispFilter = 0;

		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));
			if (request->argName(i) == "oledEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						oledEN = true;
					}
				}
			}
			if (request->argName(i) == "dispFlip")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						dispFlip = true;
					}
				}
			}
			if (request->argName(i) == "filterMessage")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.dispFilter |= FILTER_MESSAGE;
				}
			}

			if (request->argName(i) == "filterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.dispFilter |= FILTER_TELEMETRY;
				}
			}

			if (request->argName(i) == "filterStatus")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.dispFilter |= FILTER_STATUS;
				}
			}

			if (request->argName(i) == "filterWeather")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.dispFilter |= FILTER_WX;
				}
			}

			if (request->argName(i) == "filterObject")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.dispFilter |= FILTER_OBJECT;
				}
			}

			if (request->argName(i) == "filterItem")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.dispFilter |= FILTER_ITEM;
				}
			}

			if (request->argName(i) == "filterQuery")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.dispFilter |= FILTER_QUERY;
				}
			}
			if (request->argName(i) == "filterBuoy")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.dispFilter |= FILTER_BUOY;
				}
			}
			if (request->argName(i) == "filterPosition")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.dispFilter |= FILTER_POSITION;
				}
			}

			if (request->argName(i) == "dispRF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						dispRF = true;
				}
			}

			if (request->argName(i) == "dispINET")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						dispINET = true;
				}
			}
			if (request->argName(i) == "txdispEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						dispTX = true;
				}
			}
			if (request->argName(i) == "rxdispEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						dispRX = true;
				}
			}

			if (request->argName(i) == "dispBright")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.disp_brightness = request->arg(i).toInt();
#ifdef ST7735_LED_K_Pin
						ledcWrite(0, (uint32_t)config.disp_brightness);
#endif
					}
				}
			}

			if (request->argName(i) == "dispDelay")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.dispDelay = request->arg(i).toInt();
						if (config.dispDelay < 0)
							config.dispDelay = 0;
					}
				}
			}

			if (request->argName(i) == "oled_timeout")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.oled_timeout = request->arg(i).toInt();
						if (config.oled_timeout < 0)
							config.oled_timeout = 0;
					}
				}
			}
			if (request->argName(i) == "filterDX")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.filterDistant = request->arg(i).toInt();
					}
				}
			}
		}

		#ifdef OLED
		if(oledEN && !config.oled_enable)
		{
			//display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false); // initialize with the I2C addr 0x3C (for the 128x64)
            // Initialising the UI will init the display too.
#ifdef SH1106
            display.begin(SH1106_SWITCHCAPVCC, SCREEN_ADDRESS, false);
#else
            display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, false, false);
#endif
            display.clearDisplay();
		}
		config.oled_enable = oledEN;
		config.dispINET = dispINET;
		config.dispRF = dispRF;
		config.rx_display = dispRX;
		config.tx_display = dispTX;
		config.disp_flip = dispFlip;
		#endif // OLED
		// config.filterMessage = filterMessage;
		// config.filterStatus = filterStatus;
		// config.filterTelemetry = filterTelemetry;
		// config.filterWeather = filterWeather;
		// config.filterTracker = filterTracker;
		// config.filterMove = filterMove;
		// config.filterPosition = filterPosition;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else
	{
		struct tm tmstruct;
		char strTime[20];
		tmstruct.tm_year = 0;
		getLocalTime(&tmstruct, 100);
		sprintf(strTime, "%d-%02d-%02d %02d:%02d:%02d", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);

		String html = "<script type=\"text/javascript\">\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		html += "if(e.currentTarget.id===\"formHostName\") document.getElementById(\"updateHostName\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formTime\") document.getElementById(\"updateTime\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formNTP\") document.getElementById(\"updateTimeNtp\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formTimeZone\") document.getElementById(\"updateTimeZone\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formReboot\") document.getElementById(\"REBOOT\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formDisp\") document.getElementById(\"submitDISP\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formWebAuth\") document.getElementById(\"submitWebAuth\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formPWR\") document.getElementById(\"submitPWR\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formPath\") document.getElementById(\"submitPath\").disabled=true;\n";
		html += "$.ajax({\n";
		html += "url: '/system',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "alert(\"Submited Successfully\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";
		html += "</script>\n";

		// html += "<h2>System Setting</h2>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>System Setting</b></span></th>\n";
		html += "<tr>\n";
		html += "<td style=\"text-align: right;\">Host Name:</td>\n";
		html += "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formHostName\" method=\"post\"><input name=\"SetHostName\" type=\"text\" value=\"" + String(config.host_name) + "\" />\n";
		html += "<button type='submit' id='updateHostName'  name=\"commit\"> Apply </button>\n";
		html += "<input type=\"hidden\" name=\"updateHostName\"/></form>\n</td>\n";		
		html += "</tr>\n";
		html += "<tr>";
		// html += "<form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formTime\" method=\"post\">\n";
		html += "<td style=\"text-align: right;\">LOCAL DATE/TIME </td>\n";
		html += "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formTime\" method=\"post\">\n<input name=\"SetTime\" type=\"text\" value=\"" + String(strTime) + "\" />\n";
		html += "<span class=\"input-group-addon\">\n<span class=\"glyphicon glyphicon-calendar\">\n</span></span>\n";
		// html += "<div class=\"col-sm-3 col-xs-6\"><button class=\"btn btn-primary\" data-args=\"[true]\" data-method=\"getDate\" type=\"button\" data-related-target=\"#SetTime\" />Get Date</button></div>\n";
		html += "<button type='submit' id='updateTime'  name=\"commit\"> Time Update </button>\n";
		html += "<input type=\"hidden\" name=\"updateTime\"/></form>\n</td>\n";
		// html += "<input class=\"btn btn-primary\" id=\"updateTime\" name=\"updateTime\" type=\"submit\" value=\"Time Update\" maxlength=\"80\"/></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td style=\"text-align: right;\">NTP Host </td>\n";
		html += "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formNTP\" method=\"post\"><input name=\"SetTimeNtp\" type=\"text\" value=\"" + String(config.ntp_host) + "\" />\n";
		html += "<button type='submit' id='updateTimeNtp'  name=\"commit\"> NTP Update </button>\n";
		html += "<input type=\"hidden\" name=\"updateTimeNtp\"/></form>\n</td>\n";
		// html += "<input class=\"btn btn-primary\" id=\"updateTimeNtp\" name=\"updateTimeNtp\" type=\"submit\" value=\"NTP Update\" maxlength=\"80\"/></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td style=\"text-align: right;\">Auto REBOOT:</td>\n";
		html += "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formAutoReset\" method=\"post\"><input  min=\"0\" max=\"65535\"  name=\"SetAutoReset\" type=\"number\" value=\"" + String(config.reset_timeout) + "\" /> Minutes\n";
		html += "<button type='submit' id='updateAutoReset'  name=\"commit\"> Update </button> *<i>0=No reset</i>\n";
		html += "<input type=\"hidden\" name=\"updateAutoReset\"/></form>\n</td>\n";
		// html += "<input class=\"button\" id=\"updateTimeNtp\" name=\"updateTimeNtp\" type=\"submit\" value=\"NTP Update\" maxlength=\"80\"/></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td style=\"text-align: right;\">Time Zone </td>\n";
		html += "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formTimeZone\" method=\"post\">\n";
		html += "<select name=\"SetTimeZone\" id=\"SetTimeZone\">\n";
		for (int i = 0; i < 40; i++)
		{
			if (config.timeZone == tzList[i].tz)
				html += "<option value=\"" + String(tzList[i].tz, 1) + "\" selected>" + String(tzList[i].name) + " Sec</option>\n";
			else
				html += "<option value=\"" + String(tzList[i].tz, 1) + "\" >" + String(tzList[i].name) + " Sec</option>\n";
		}
		html += "</select>";
		html += "<button type='submit' id='updateTimeZone'  name=\"commit\"> TZ Update </button>\n";
		html += "<input type=\"hidden\" name=\"updateTimeZone\"/></form>\n</td>\n";
		// html += "<input class=\"btn btn-primary\" id=\"updateTimeZone\" name=\"updateTimeZone\" type=\"submit\" value=\"TZ Update\" maxlength=\"80\"/></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td style=\"text-align: right;\">SYSTEM CONTROL </td>\n";
		html += "<td style=\"text-align: left;\"><table><tr><td width=\"100\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formReboot\" method=\"post\"> <button type='submit' id='REBOOT'  name=\"commit\" style=\"background-color:red;color:white\"> REBOOT </button>\n";
		html += " <input type=\"hidden\" name=\"REBOOT\"/></form></td><td width=\"100\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formFactory\" method=\"post\"> <button type='submit' id='Factory'  name=\"commit\" style=\"background-color:orange;color:white\"> Factory Reset </button>\n";
		html += " <input type=\"hidden\" name=\"Factory\"/></form></td><td width=\"100\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formLoad\" method=\"post\"> <button type='submit' id='LoadCFG'  name=\"commit\" style=\"background-color:green;color:white\"> Load Default </button>\n";
		html += " <input type=\"hidden\" name=\"LoadCFG\"/></form></td></tr></table></td>\n";
		// html += "<td style=\"text-align: left;\"><input type='submit' class=\"btn btn-danger\" id=\"REBOOT\" name=\"REBOOT\" value='REBOOT'></td>\n";
		html += "</tr></table><br /><br />\n";

		/************************ WEB AUTH **************************/
		html += "<form id='formWebAuth' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>Web Authentication</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Web USER:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"32\" maxlength=\"32\" class=\"form-control\" name=\"webauth_user\" type=\"text\" value=\"" + String(config.http_username) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Web PASSWORD:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"63\" maxlength=\"63\" class=\"form-control\" name=\"webauth_pass\" type=\"password\" value=\"" + String(config.http_password) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitWebAuth'  name=\"commit\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitWebAuth\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form><br /><br />";

		/**************Power Mode******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formPWR\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>Power Save Mode</b></span></th>\n";
		html += "<tr>";

		String enFlage = "";
		if (config.pwr_en)
			enFlage = "checked";
		html += "<td align=\"right\"><b>Enable</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + enFlage + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		String LowFlag = "", HighFlag = "";
		LowFlag = "";
		HighFlag = "";
		if (config.pwr_active)
			HighFlag = "checked=\"checked\"";
		else
			LowFlag = "checked=\"checked\"";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PWR GPIO:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\"  name=\"pwr\" type=\"number\" value=\"" + String(config.pwr_gpio) + "\" /> Output Active:<input type=\"radio\" name=\"pwr_active\" value=\"0\" " + LowFlag + "/>LOW <input type=\"radio\" name=\"pwr_active\" value=\"1\" " + HighFlag + "/>HIGH </td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Sleep Interval:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"0\" max=\"9999\" name=\"sleep\" type=\"number\" value=\"" + String(config.pwr_sleep_interval) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>StandBy Delay:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"0\" max=\"9999\" name=\"stb\" type=\"number\" value=\"" + String(config.pwr_stanby_delay) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Power Mode:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"mode\" id=\"mode\">\n";
		for (int i = 0; i < 3; i++)
		{
			if (config.pwr_mode == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(PWR_MODE[i]) + " </option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(PWR_MODE[i]) + " </option>\n";
		}
		html += "</select> A=Reduce Speed(PWR Off),B=Light Sleep(WiFi/PWR Off),C=Deep Sleep(All Off)\n";
		html += "</td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Event Activate:</b><br/>(For Mode C)</td>\n";
		html += "<td style=\"text-align: left;\">";
		html += "<fieldset id=\"FilterGrp\">\n";
		html += "<legend>Events</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">";
		html += "<tr style=\"background:unset;\">";

		String filterFlageEn = "";
		if (config.pwr_sleep_activate & ACTIVATE_TRACKER)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterTracker\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Tracker</td>\n";

		filterFlageEn = "";
		if (config.pwr_sleep_activate & ACTIVATE_STATUS)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterStatus\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Status</td>\n";

		filterFlageEn = "";
		if (config.pwr_sleep_activate & ACTIVATE_TELEMETRY)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterTelemetry\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Telemetry</td>\n";

		filterFlageEn = "";
		if (config.pwr_sleep_activate & ACTIVATE_WX)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterWeather\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Weather</td>\n";

		filterFlageEn = "";
		if (config.pwr_sleep_activate & ACTIVATE_IGATE)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterIGate\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>IGate</td>\n";

		filterFlageEn = "";
		if (config.pwr_sleep_activate & ACTIVATE_DIGI)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterDigi\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Digi</td>\n";

		filterFlageEn = "";
		if (config.pwr_sleep_activate & ACTIVATE_QUERY)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterQuery\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Query</td>\n";

		filterFlageEn = "";
		if (config.pwr_sleep_activate & ACTIVATE_WIFI)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterWifi\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>WiFi</td>\n";

		html += "<td style=\"border:unset;\"></td>";
		html += "</tr></table></fieldset>\n";
		html += "</td>\n";
		html += "</tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitPWR'  name=\"commitPWR\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitPWR\"/>\n";
		html += "</td></tr></table>\n";

		html += "</form><br /><br />\n";

		/**************Log File******************/
		html += "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formLOG\" method=\"post\">\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>Log File</b></span></th>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Activate:</b></td>\n";
		html += "<td style=\"text-align: left;\">";
		html += "<fieldset id=\"FilterGrp\">\n";
		html += "<legend>Events</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">";
		html += "<tr style=\"background:unset;\">";

		filterFlageEn = "";
		if (config.log & LOG_TRACKER)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logTracker\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Tracker</td>\n";

		filterFlageEn = "";
		if (config.log & LOG_IGATE)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logIgate\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>IGate</td>\n";

		filterFlageEn = "";
		if (config.log & LOG_DIGI)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logDigi\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>DIGI</td>\n";

		filterFlageEn = "";
		if (config.log & LOG_WX)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logWeather\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Weather</td>\n";

		html += "<td style=\"border:unset;\"></td>";
		html += "</tr></table></fieldset>\n";
		html += "</td>\n";
		html += "</tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitLOG'  name=\"commitLOG\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitLOG\"/>\n";
		html += "</td></tr></table>\n";

		html += "</form><br /><br />\n";

		/************************ PATH USER define **************************/
		html += "<form id='formPath' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>PATH USER Define</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PATH_1:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path1\" type=\"text\" value=\"" + String(config.path[0]) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PATH_2:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path2\" type=\"text\" value=\"" + String(config.path[1]) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PATH_3:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path3\" type=\"text\" value=\"" + String(config.path[2]) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PATH_4:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path4\" type=\"text\" value=\"" + String(config.path[3]) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitPath'  name=\"commitPath\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitPath\"/>\n";
		html += "</td></tr></table>\n";
		html += "</form><br /><br />";
		// delay(1);
// log_d("%s",html.c_str());
// log_d("Length: %d",html.length());
#if defined OLED || defined ST7735_160x80
		html += "<form id='formDisp' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		// html += "<h2>Display Setting</h2>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>Display Setting</b></span></th>\n";
		html += "<tr>\n";
		html += "<td style=\"text-align: right;\"><b>OLED/TFT Enable</b></td>\n";
		String oledFlageEn = "";
		if (config.oled_enable == true)
			oledFlageEn = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"oledEnable\" value=\"OK\" " + oledFlageEn + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		oledFlageEn = "";
		if (config.disp_flip == true)
			oledFlageEn = "checked";
		html += "<tr>\n";
		html += "<td style=\"text-align: right;\"><b>Flip Rotate</b></td>\n";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"dispFlip\" value=\"OK\" " + oledFlageEn + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td style=\"text-align: right;\"><b>TX Display</b></td>\n";
		String txdispFlageEn = "";
		if (config.tx_display == true)
			txdispFlageEn = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"txdispEnable\" value=\"OK\" " + txdispFlageEn + "><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*All TX Packet for display affter filter.</i></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td style=\"text-align: right;\"><b>RX Display</b></td>\n";
		String rxdispFlageEn = "";
		if (config.rx_display == true)
			rxdispFlageEn = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"rxdispEnable\" value=\"OK\" " + rxdispFlageEn + "><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*All RX Packet for display affter filter.</i></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td style=\"text-align: right;\"><b>Head Up</b></td>\n";
		String hupFlageEn = "";
		if (config.h_up == true)
			hupFlageEn = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"hupEnable\" value=\"OK\" " + hupFlageEn + "><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*The compass will rotate in the direction of movement.</i></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td style=\"text-align: right;\"><b>TFT Brightness</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"dispBright\" id=\"dispBright\">\n";
		for (int i = 0; i < 255; i += 25)
		{
			if (config.disp_brightness == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(i) + "</option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(i) + "</option>\n";
		}
		html += "</select>\n";
		html += "</td></tr>\n";

		html += "<tr>\n";
		html += "<td style=\"text-align: right;\"><b>Popup Delay</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"dispDelay\" id=\"dispDelay\">\n";
		for (int i = 0; i < 16; i += 1)
		{
			if (config.dispDelay == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(i) + " Sec</option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(i) + " Sec</option>\n";
		}
		html += "</select>\n";
		html += "</td></tr>\n";
		html += "<tr>\n";
		html += "<td style=\"text-align: right;\"><b>OLED/TFT Sleep</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"oled_timeout\" id=\"oled_timeout\">\n";
		for (int i = 0; i <= 600; i += 30)
		{
			if (config.oled_timeout == i)
				html += "<option value=\"" + String(i) + "\" selected>" + String(i) + " Sec</option>\n";
			else
				html += "<option value=\"" + String(i) + "\" >" + String(i) + " Sec</option>\n";
		}
		html += "</select>\n";
		html += "</td></tr>\n";
		String rfFlageEn = "";
		if (config.dispRF == true)
			rfFlageEn = "checked";
		String inetFlageEn = "";
		if (config.dispINET == true)
			inetFlageEn = "checked";
		html += "<tr><td style=\"text-align: right;\"><b>RX Channel</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"dispRF\" value=\"OK\" " + rfFlageEn + "/>RF <input type=\"checkbox\" name=\"dispINET\" value=\"OK\" " + inetFlageEn + "/>Internet </td></tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Filter DX:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input type=\"number\" name=\"filterDX\" min=\"0\" max=\"9999\"\n";
		html += "step=\"1\" value=\"" + String(config.filterDistant) + "\" /> Km.  <label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*Value 0 is all distant allow.</i></label></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Filter:</b></td>\n";

		html += "<td align=\"center\">\n";
		html += "<fieldset id=\"filterDispGrp\">\n";
		html += "<legend>Filter popup display</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">";
		html += "<tr style=\"background:unset;\">";

		// html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"dispTNC\" name=\"dispTNC\" type=\"checkbox\" value=\"OK\" " + rfFlageEn + "/>From RF</td>\n";

		// html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"dispINET\" name=\"dispINET\" type=\"checkbox\" value=\"OK\" " + inetFlageEn + "/>From INET</td>\n";

		String filterMessageFlageEn = "";
		if (config.dispFilter & FILTER_MESSAGE)
			filterMessageFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterMessage\" name=\"filterMessage\" type=\"checkbox\" value=\"OK\" " + filterMessageFlageEn + "/>Message</td>\n";

		String filterStatusFlageEn = "";
		if (config.dispFilter & FILTER_STATUS)
			filterStatusFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterStatus\" name=\"filterStatus\" type=\"checkbox\" value=\"OK\" " + filterStatusFlageEn + "/>Status</td>\n";

		String filterTelemetryFlageEn = "";
		if (config.dispFilter & FILTER_TELEMETRY)
			filterTelemetryFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterTelemetry\" name=\"filterTelemetry\" type=\"checkbox\" value=\"OK\" " + filterTelemetryFlageEn + "/>Telemetry</td>\n";

		String filterWeatherFlageEn = "";
		if (config.dispFilter & FILTER_WX)
			filterWeatherFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterWeather\" name=\"filterWeather\" type=\"checkbox\" value=\"OK\" " + filterWeatherFlageEn + "/>Weather</td>\n";

		String filterObjectFlageEn = "";
		if (config.dispFilter & FILTER_OBJECT)
			filterObjectFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterObject\" name=\"filterObject\" type=\"checkbox\" value=\"OK\" " + filterObjectFlageEn + "/>Object</td>\n";

		String filterItemFlageEn = "";
		if (config.dispFilter & FILTER_ITEM)
			filterItemFlageEn = "checked";
		html += "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterItem\" name=\"filterItem\" type=\"checkbox\" value=\"OK\" " + filterItemFlageEn + "/>Item</td>\n";

		String filterQueryFlageEn = "";
		if (config.dispFilter & FILTER_QUERY)
			filterQueryFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterQuery\" name=\"filterQuery\" type=\"checkbox\" value=\"OK\" " + filterQueryFlageEn + "/>Query</td>\n";

		String filterBuoyFlageEn = "";
		if (config.dispFilter & FILTER_BUOY)
			filterBuoyFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterBuoy\" name=\"filterBuoy\" type=\"checkbox\" value=\"OK\" " + filterBuoyFlageEn + "/>Buoy</td>\n";

		String filterPositionFlageEn = "";
		if (config.dispFilter & FILTER_POSITION)
			filterPositionFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterPosition\" name=\"filterPosition\" type=\"checkbox\" value=\"OK\" " + filterPositionFlageEn + "/>Position</td>\n";

		html += "<td style=\"border:unset;\"></td>";
		html += "</tr></table></fieldset>\n";

		html += "</td></tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitDISP'  name=\"commitDISP\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitDISP\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form><br />";
#endif

		if ((ESP.getFreeHeap() / 1000) > 120)
		{
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			size_t len = html.length();
			char *info = (char *)calloc(len, sizeof(char));
			if (info)
			{

				html.toCharArray(info, len, 0);
				html.clear();
				AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)info, len);

				response->addHeader("Sensor", "content");
				request->send(response);
				free(info);
			}
			else
			{
				log_d("Can't define calloc info size %d", len);
			}
		}
		// if ((ESP.getFreeHeap() / 1000) > 100)
		//{
		// request->send(200, "text/html", html); // send to someones browser when asked
		// }
		// else
		// {
		// 	AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)html.c_str(), html.length());
		// 	response->addHeader("System", "content");
		// 	request->send(response);
		// }
		// html.clear();
	}
}

void handle_igate(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	bool aprsEn = false;
	bool rf2inetEn = false;
	bool inet2rfEn = false;
	bool posGPS = false;
	bool bcnEN = false;
	bool pos2RF = false;
	bool pos2INET = false;
	bool timeStamp = false;

	if (request->hasArg("commitIGATE"))
	{

		for (int i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "igateEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						aprsEn = true;
				}
			}
			if (request->argName(i) == "myCall")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					name.toUpperCase();
					strcpy(config.aprs_mycall, name.c_str());
				}
			}
			if (request->argName(i) == "igateObject")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					strcpy(config.igate_object, name.c_str());
				}
				else
				{
					memset(config.igate_object, 0, sizeof(config.igate_object));
				}
			}
			if (request->argName(i) == "mySSID")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.aprs_ssid = request->arg(i).toInt();
					if (config.aprs_ssid > 15)
						config.aprs_ssid = 13;
				}
			}
			if (request->argName(i) == "igatePosInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "igateSTSInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_sts_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "igatePosLat")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_lat = request->arg(i).toFloat();
				}
			}

			if (request->argName(i) == "igatePosLon")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_lon = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "igatePosAlt")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_alt = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "igatePosSel")
			{
				if (request->arg(i) != "")
				{
					if (request->arg(i).toInt() == 1)
						posGPS = true;
				}
			}

			if (request->argName(i) == "igateTable")
			{
				if (request->arg(i) != "")
				{
					config.igate_symbol[0] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "igateSymbol")
			{
				if (request->arg(i) != "")
				{
					config.igate_symbol[1] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "aprsHost")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.aprs_host, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "aprsPort")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.aprs_port = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "aprsFilter")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.aprs_filter, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "igatePath")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "igateComment")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.igate_comment, request->arg(i).c_str());
				}
				else
				{
					memset(config.igate_comment, 0, sizeof(config.igate_comment));
				}
			}
			if (request->argName(i) == "igateStatus")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.igate_status, request->arg(i).c_str());
				}
				else
				{
					memset(config.igate_comment, 0, sizeof(config.igate_comment));
				}
			}
			if (request->argName(i) == "texttouse")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.igate_phg, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "rf2inetEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						rf2inetEn = true;
				}
			}
			if (request->argName(i) == "inet2rfEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						inet2rfEn = true;
				}
			}
			if (request->argName(i) == "igatePos2RF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2RF = true;
				}
			}
			if (request->argName(i) == "igatePos2INET")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2INET = true;
				}
			}
			if (request->argName(i) == "igateBcnEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						bcnEN = true;
				}
			}
			if (request->argName(i) == "igateTimeStamp")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						timeStamp = true;
				}
			}
			if (request->argName(i) == "igateTlmInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_tlm_interval = request->arg(i).toInt();
				}
			}

			String arg;
			for (int x = 0; x < 5; x++)
			{
				arg = "sensorCH" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.igate_tlm_sensor[x] = request->arg(i).toInt();
				}
				arg = "param" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.igate_tlm_PARM[x], request->arg(i).c_str());
					}
				}
				arg = "unit" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.igate_tlm_UNIT[x], request->arg(i).c_str());
					}
				}
				arg = "precision" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.igate_tlm_precision[x] = request->arg(i).toInt();
				}
				arg = "offset" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.igate_tlm_offset[x] = request->arg(i).toFloat();
				}
				for (int y = 0; y < 3; y++)
				{
					arg = "eqns" + String(x) + String((char)(y + 'a'));
					if (request->argName(i) == arg)
					{
						if (isValidNumber(request->arg(i)))
							config.igate_tlm_EQNS[x][y] = request->arg(i).toFloat();
					}
				}
			}
		}

		//waitISRetry = millis() + 10000; // Retry connect 5Sec
		config.igate_en = aprsEn;
		config.rf2inet = rf2inetEn;
		config.inet2rf = inet2rfEn;
		config.igate_gps = posGPS;
		config.igate_bcn = bcnEN;
		config.igate_loc2rf = pos2RF;
		config.igate_loc2inet = pos2INET;
		config.igate_timestamp = timeStamp;

		initInterval = true;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else if (request->hasArg("commitIGATEfilter"))
	{
		config.rf2inetFilter = 0;
		config.inet2rfFilter = 0;
		for (int i = 0; i < request->args(); i++)
		{
			// config rf2inet filter
			if (request->argName(i) == "rf2inetFilterMessage")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_MESSAGE;
				}
			}

			if (request->argName(i) == "rf2inetFilterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_TELEMETRY;
				}
			}

			if (request->argName(i) == "rf2inetFilterStatus")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_STATUS;
				}
			}

			if (request->argName(i) == "rf2inetFilterWeather")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_WX;
				}
			}

			if (request->argName(i) == "rf2inetFilterObject")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_OBJECT;
				}
			}

			if (request->argName(i) == "rf2inetFilterItem")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_ITEM;
				}
			}

			if (request->argName(i) == "rf2inetFilterQuery")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_QUERY;
				}
			}
			if (request->argName(i) == "rf2inetFilterBuoy")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_BUOY;
				}
			}
			if (request->argName(i) == "rf2inetFilterPosition")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_POSITION;
				}
			}
			// config inet2rf filter

			if (request->argName(i) == "inet2rfFilterMessage")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_MESSAGE;
				}
			}

			if (request->argName(i) == "inet2rfFilterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_TELEMETRY;
				}
			}

			if (request->argName(i) == "inet2rfFilterStatus")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_STATUS;
				}
			}

			if (request->argName(i) == "inet2rfFilterWeather")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_WX;
				}
			}

			if (request->argName(i) == "inet2rfFilterObject")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_OBJECT;
				}
			}

			if (request->argName(i) == "inet2rfFilterItem")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_ITEM;
				}
			}

			if (request->argName(i) == "inet2rfFilterQuery")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_QUERY;
				}
			}
			if (request->argName(i) == "inet2rfFilterBuoy")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_BUOY;
				}
			}
			if (request->argName(i) == "inet2rfFilterPosition")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_POSITION;
				}
			}
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else
	{

		String html = "<script type=\"text/javascript\">\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		// html += "document.getElementById(\"submitIGATE\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formIgate\") document.getElementById(\"submitIGATE\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formIgateFilter\") document.getElementById(\"submitIGATEfilter\").disabled=true;\n";
		html += "$.ajax({\n";
		html += "url: '/igate',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "alert(\"Submited Successfully\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";
		html += "</script>\n<script type=\"text/javascript\">\n";

		html += "function openWindowSymbol() {\n";
		html += "var i, l, options = [{\n";
		html += "value: 'first',\n";
		html += "text: 'First'\n";
		html += "}, {\n";
		html += "value: 'second',\n";
		html += "text: 'Second'\n";
		html += "}],\n";
		html += "newWindow = window.open(\"/symbol\", null, \"height=400,width=400,status=no,toolbar=no,menubar=no,location=no\");\n";
		html += "}\n";

		html += "function setValue(symbol,table) {\n";
		html += "document.getElementById('igateSymbol').value = String.fromCharCode(symbol);\n";
		html += "if(table==1){\n document.getElementById('igateTable').value='/';\n";
		html += "}else if(table==2){\n document.getElementById('igateTable').value='\\\\';\n}\n";
		html += "document.getElementById('igateImgSymbol').src = \"http://aprs.dprns.com/symbols/icons/\"+symbol.toString()+'-'+table.toString()+'.png';\n";
		html += "\n}\n";
		html += "function calculatePHGR(){document.forms.formIgate.texttouse.value=\"PHG\"+calcPower(document.forms.formIgate.power.value)+calcHeight(document.forms.formIgate.haat.value)+calcGain(document.forms.formIgate.gain.value)+calcDirection(document.forms.formIgate.direction.selectedIndex)}function Log2(e){return Math.log(e)/Math.log(2)}function calcPerHour(e){return e<10?e:String.fromCharCode(65+(e-10))}function calcHeight(e){return String.fromCharCode(48+Math.round(Log2(e/10),0))}function calcPower(e){if(e<1)return 0;if(e>=1&&e<4)return 1;if(e>=4&&e<9)return 2;if(e>=9&&e<16)return 3;if(e>=16&&e<25)return 4;if(e>=25&&e<36)return 5;if(e>=36&&e<49)return 6;if(e>=49&&e<64)return 7;if(e>=64&&e<81)return 8;if(e>=81)return 9}function calcDirection(e){if(e==\"0\")return\"0\";if(e==\"1\")return\"1\";if(e==\"2\")return\"2\";if(e==\"3\")return\"3\";if(e==\"4\")return\"4\";if(e==\"5\")return\"5\";if(e==\"6\")return\"6\";if(e==\"7\")return\"7\";if(e==\"8\")return\"8\"}function calcGain(e){return e>9?\"9\":e<0?\"0\":Math.round(e,0)}\n";
		html += "function onRF2INETCheck() {\n";
		html += "if (document.querySelector('#rf2inetEnable').checked) {\n";
		// Checkbox has been checked
		html += "document.getElementById(\"rf2inetFilterGrp\").disabled=false;\n";
		html += "} else {\n";
		// Checkbox has been unchecked
		html += "document.getElementById(\"rf2inetFilterGrp\").disabled=true;\n";
		html += "}\n}\n";
		html += "function onINET2RFCheck() {\n";
		html += "if (document.querySelector('#inet2rfEnable').checked) {\n";
		// Checkbox has been checked
		html += "document.getElementById(\"inet2rfFilterGrp\").disabled=false;\n";
		html += "} else {\n";
		// Checkbox has been unchecked
		html += "document.getElementById(\"inet2rfFilterGrp\").disabled=true;\n";
		html += "}\n}\n";

		html += "function selPrecision(idx) {\n";
		html += "var x=0;\n";
		html += "x = document.getElementsByName(\"precision\"+idx)[0].value;\n";
		html += "document.getElementsByName(\"eqns\"+idx+\"b\")[0].value=1/Math.pow(10,x);\n";
		html += "}\n";
		html += "function selOffset(idx) {\n";
		html += "var x=0;\n";
		html += "x = document.getElementsByName(\"offset\"+idx)[0].value;\n";
		html += "document.getElementsByName(\"eqns\"+idx+\"c\")[0].value=x*(-1);\n";
		html += "}\n";
		html += "</script>\n";
		delay(1);
		/************************ IGATE Mode **************************/
		html += "<form id='formIgate' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		// html += "<h2>[IGATE] Internet Gateway Mode</h2>\n";
		html += "<table>\n";
		// html += "<tr>\n";
		// html += "<th width=\"200\"><span><b>Setting</b></span></th>\n";
		// html += "<th><span><b>Value</b></span></th>\n";
		// html += "</tr>\n";
		html += "<th colspan=\"2\"><span><b>[IGATE] Internet Gateway Mode</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Enable:</b></td>\n";
		String igateEnFlag = "";
		if (config.igate_en)
			igateEnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"igateEnable\" value=\"OK\" " + igateEnFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Station Callsign:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"" + String(config.aprs_mycall) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Station SSID:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"mySSID\" id=\"mySSID\">\n";
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			if (config.aprs_ssid == ssid)
			{
				html += "<option value=\"" + String(ssid) + "\" selected>" + String(ssid) + "</option>\n";
			}
			else
			{
				html += "<option value=\"" + String(ssid) + "\">" + String(ssid) + "</option>\n";
			}
		}
		html += "</select></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Station Symbol:</b></td>\n";
		String table = "1";
		if (config.igate_symbol[0] == 47)
			table = "1";
		if (config.igate_symbol[0] == 92)
			table = "2";
		html += "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"igateTable\" name=\"igateTable\" type=\"text\" value=\"" + String(config.igate_symbol[0]) + "\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"igateSymbol\" name=\"igateSymbol\" type=\"text\" value=\"" + String(config.igate_symbol[1]) + "\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"igateImgSymbol\" onclick=\"openWindowSymbol();\" src=\"http://aprs.dprns.com/symbols/icons/" + String((int)config.igate_symbol[1]) + "-" + table + ".png\"> <i>*Click icon for select symbol</i></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Item/Obj Name:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"9\" size=\"9\" id=\"igateObject\" name=\"igateObject\" type=\"text\" value=\"" + String(config.igate_object) + "\" /><i> *If not used, leave it blank.In use 3-9 charactor</i></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PATH:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"igatePath\" id=\"igatePath\">\n";
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			if (config.igate_path == pthIdx)
			{
				html += "<option value=\"" + String(pthIdx) + "\" selected>" + String(PATH_NAME[pthIdx]) + "</option>\n";
			}
			else
			{
				html += "<option value=\"" + String(pthIdx) + "\">" + String(PATH_NAME[pthIdx]) + "</option>\n";
			}
		}
		html += "</select></td>\n";
		// html += "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" id=\"igatePath\" name=\"igatePath\" type=\"text\" value=\"" + String(config.igate_path) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Server Host:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"20\" size=\"20\" id=\"aprsHost\" name=\"aprsHost\" type=\"text\" value=\"" + String(config.aprs_host) + "\" /> *APRS-IS by T2THAI at <a href=\"http://aprs.dprns.com:14501\" target=\"_t2thai\">aprs.dprns.com:14580</a>,CBAPRS at <a href=\"http://aprs.dprns.com:24501\" target=\"_t2thai\">aprs.dprns.com:24580</a></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Server Port:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"1\" max=\"65535\" step=\"1\" id=\"aprsPort\" name=\"aprsPort\" type=\"number\" value=\"" + String(config.aprs_port) + "\" /> *AMPR Host at <a href=\"http://aprs.hs5tqa.ampr.org:14501\" target=\"_t2thai\">aprs.hs5tqa.ampr.org:14580</a></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Server Filter:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"30\" size=\"30\" id=\"aprsFilter\" name=\"aprsFilter\" type=\"text\" value=\"" + String(config.aprs_filter) + "\" /> *Filter: <a target=\"_blank\" href=\"http://www.aprs-is.net/javAPRSFilter.aspx\">http://www.aprs-is.net/javAPRSFilter.aspx</a></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Text Comment:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"25\" size=\"30\" id=\"igateComment\" name=\"igateComment\" type=\"text\" value=\"" + String(config.igate_comment) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Text Status:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"60\" id=\"igateStatus\" name=\"igateStatus\" type=\"text\" value=\"" + String(config.igate_status) + "\" />  Interval:<input min=\"0\" max=\"3600\" step=\"1\" name=\"igateSTSInv\" type=\"number\" value=\"" + String(config.igate_sts_interval) + "\" />Sec.</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>RF2INET:</b></td>\n";
		String rf2inetEnFlag = "";
		if (config.rf2inet)
			rf2inetEnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"rf2inetEnable\" name=\"rf2inetEnable\" onclick=\"onRF2INETCheck()\" value=\"OK\" " + rf2inetEnFlag + "><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch RF to Internet gateway</i></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>INET2RF:</b></td>\n";
		String inet2rfEnFlag = "";
		if (config.inet2rf)
			inet2rfEnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"inet2rfEnable\" name=\"inet2rfEnable\" onclick=\"onINET2RFCheck()\" value=\"OK\" " + inet2rfEnFlag + "><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch Internet to RF gateway</i></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Time Stamp:</b></td>\n";
		String timeStampFlag = "";
		if (config.igate_timestamp)
			timeStampFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"igateTimeStamp\" value=\"OK\" " + timeStampFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n<tr>";

		html += "<td align=\"right\"><b>POSITION:</b></td>\n";
		html += "<td align=\"center\">\n";
		html += "<table>";
		String igateBcnEnFlag = "";
		if (config.igate_bcn)
			igateBcnEnFlag = "checked";

		html += "<tr><td style=\"text-align: right;\">Beacon:</td><td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"igateBcnEnable\" value=\"OK\" " + igateBcnEnFlag + "><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\">  Interval:<input min=\"0\" max=\"3600\" step=\"1\" id=\"igatePosInv\" name=\"igatePosInv\" type=\"number\" value=\"" + String(config.igate_interval) + "\" />Sec.</label></td></tr>";
		String igatePosFixFlag = "";
		String igatePosGPSFlag = "";
		String igatePos2RFFlag = "";
		String igatePos2INETFlag = "";
		if (config.igate_gps)
			igatePosGPSFlag = "checked=\"checked\"";
		else
			igatePosFixFlag = "checked=\"checked\"";

		if (config.igate_loc2rf)
			igatePos2RFFlag = "checked";
		if (config.igate_loc2inet)
			igatePos2INETFlag = "checked";
		html += "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"igatePosSel\" value=\"0\" " + igatePosFixFlag + "/>Fix <input type=\"radio\" name=\"igatePosSel\" value=\"1\" " + igatePosGPSFlag + "/>GPS </td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"igatePos2RF\" value=\"OK\" " + igatePos2RFFlag + "/>RF <input type=\"checkbox\" name=\"igatePos2INET\" value=\"OK\" " + igatePos2INETFlag + "/>Internet </td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" id=\"igatePosLat\" name=\"igatePosLat\" type=\"number\" value=\"" + String(config.igate_lat, 5) + "\" />degrees (positive for North, negative for South)</td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" id=\"igatePosLon\" name=\"igatePosLon\" type=\"number\" value=\"" + String(config.igate_lon, 5) + "\" />degrees (positive for East, negative for West)</td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" id=\"igatePosAlt\" name=\"igatePosAlt\" type=\"number\" value=\"" + String(config.igate_alt, 2) + "\" /> meter. *Value 0 is not send height</td></tr>\n";
		html += "</table></td>";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>PHG:</b></td>\n";
		html += "<td align=\"center\">\n";
		html += "<table>";
		html += "<tr>\n";
		html += "<td align=\"right\">Radio TX Power</td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"power\" id=\"power\">\n";
		html += "<option value=\"1\" selected>1</option>\n";
		html += "<option value=\"5\">5</option>\n";
		html += "<option value=\"10\">10</option>\n";
		html += "<option value=\"15\">15</option>\n";
		html += "<option value=\"25\">25</option>\n";
		html += "<option value=\"35\">35</option>\n";
		html += "<option value=\"50\">50</option>\n";
		html += "<option value=\"65\">65</option>\n";
		html += "<option value=\"80\">80</option>\n";
		html += "</select> Watts</td>\n";
		html += "</tr>\n";
		html += "<tr><td style=\"text-align: right;\">Antenna Gain</td><td style=\"text-align: left;\"><input size=\"3\" min=\"0\" max=\"100\" step=\"0.1\" id=\"gain\" name=\"gain\" type=\"number\" value=\"6\" /> dBi</td></tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\">Height</td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"haat\" id=\"haat\">\n";
		int k = 10;
		for (uint8_t w = 0; w < 10; w++)
		{
			if (w == 0)
			{
				html += "<option value=\"" + String(k) + "\" selected>" + String(k) + "</option>\n";
			}
			else
			{
				html += "<option value=\"" + String(k) + "\">" + String(k) + "</option>\n";
			}
			k += k;
		}
		html += "</select> Feet</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\">Antenna/Direction</td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"direction\" id=\"direction\">\n";
		html += "<option>Omni</option><option>NE</option><option>E</option><option>SE</option><option>S</option><option>SW</option><option>W</option><option>NW</option><option>N</option>\n";
		html += "</select></td>\n";
		html += "</tr>\n";

		html += "<tr><td align=\"right\"><b>PHG Text</b></td><td align=\"left\"><input name=\"texttouse\" type=\"text\" size=\"6\" style=\"background-color: rgb(97, 239, 170);\" value=\"" + String(config.igate_phg) + "\"/> <input type=\"button\" value=\"Calculate PHG\" onclick=\"javascript:calculatePHGR()\" /></td></tr>\n";
		html += "</table></td>";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Telemetry:</b><br />(v=0->8280)</td>\n";
		html += "<td align=\"center\"><table>\n";
		html += "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"1000\" step=\"1\" id=\"igateTlmInv\" name=\"igateTlmInv\" type=\"number\" value=\"" + String(config.igate_tlm_interval) + "\" /> *Number of packets interval,<i>Example: 0 not send,1 send every packet</i></label></td></tr>";
		for (int ax = 0; ax < 5; ax++)
		{
			html += "<tr><td align=\"right\"><b>CH A" + String(ax + 1) + ":</b></td>\n";
			html += "<td align=\"center\">\n";
			html += "<table>";

			html += "<tr><td style=\"text-align: right;\">Sensor:</td>\n";
			html += "<td style=\"text-align: left;\">CH: ";
			html += "<select name=\"sensorCH" + String(ax) + "\" id=\"sensorCH" + String(ax) + "\">\n";
			for (uint8_t idx = 0; idx < 11; idx++)
			{
				if (idx == 0)
				{
					if (config.igate_tlm_sensor[ax] == idx)
					{
						html += "<option value=\"" + String(idx) + "\" selected>NONE</option>\n";
					}
					else
					{
						html += "<option value=\"" + String(idx) + "\">NONE</option>\n";
					}
				}
				else
				{
					if (config.igate_tlm_sensor[ax] == idx)
					{
						html += "<option value=\"" + String(idx) + "\" selected>SENSOR#" + String(idx) + "</option>\n";
					}
					else
					{
						html += "<option value=\"" + String(idx) + "\">SENSOR#" + String(idx) + "</option>\n";
					}
				}
			}
			html += "</select></td>\n";

			html += "<td style=\"text-align: left;\">Name: <input maxlength=\"10\" size=\"8\" name=\"param" + String(ax) + "\" type=\"text\" value=\"" + String(config.igate_tlm_PARM[ax]) + "\" /></td>\n";
			html += "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit" + String(ax) + "\" type=\"text\" value=\"" + String(config.igate_tlm_UNIT[ax]) + "\" /></td>\n";
			html += "<td style=\"text-align: left;\">Precision: <input min=\"0\" max=\"5\" step=\"1\" type=\"number\" style=\"width: 2em\" name=\"precision" + String(ax) + "\" type=\"text\" value=\"" + String(config.igate_tlm_precision[ax]) + "\"  onchange=\"selPrecision(" + String(ax) + ")\"/></td></tr>\n";

			html += "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns" + String(ax) + "a\" type=\"number\" value=\"" + String(config.igate_tlm_EQNS[ax][0], 5) + "\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns" + String(ax) + "b\" type=\"number\" value=\"" + String(config.igate_tlm_EQNS[ax][1], 5) + "\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns" + String(ax) + "c\" type=\"number\" value=\"" + String(config.igate_tlm_EQNS[ax][2], 5) + "\" /> (av<sup>2</sup>+bv+c) </td>\n";
			html += "<td style=\"text-align: left;\">Offset: <input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" type=\"number\" name=\"offset" + String(ax) + "\" type=\"text\" value=\"" + String(config.igate_tlm_offset[ax], 5) + "\" onchange=\"selOffset(" + String(ax) + ")\"/></td></tr>\n";
			html += "</table></td>";
			html += "</tr>\n";
		}
		html += "</table></td></tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitIGATE'  name=\"commitIGATE\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitIGATE\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form><br /><br />";

		html += "<form id='formIgateFilter' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>[IGATE] Filter</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>RF2INET Filter:</b></td>\n";

		html += "<td align=\"center\">\n";
		if (config.rf2inet)
			html += "<fieldset id=\"rf2inetFilterGrp\">\n";
		else
			html += "<fieldset id=\"rf2inetFilterGrp\" disabled>\n";
		html += "<legend>Filter RF to Internet</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">";
		html += "<tr style=\"background:unset;\">";

		String filterFlageEn = "";
		if (config.rf2inetFilter & FILTER_MESSAGE)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterMessage\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Message</td>\n";

		filterFlageEn = "";
		if (config.rf2inetFilter & FILTER_STATUS)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterStatus\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Status</td>\n";

		filterFlageEn = "";
		if (config.rf2inetFilter & FILTER_TELEMETRY)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterTelemetry\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Telemetry</td>\n";

		filterFlageEn = "";
		if (config.rf2inetFilter & FILTER_WX)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterWeather\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Weather</td>\n";

		filterFlageEn = "";
		if (config.rf2inetFilter & FILTER_OBJECT)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterObject\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Object</td>\n";

		filterFlageEn = "";
		if (config.rf2inetFilter & FILTER_ITEM)
			filterFlageEn = "checked";
		html += "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterItem\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Item</td>\n";

		filterFlageEn = "";
		if (config.rf2inetFilter & FILTER_QUERY)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterQuery\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Query</td>\n";

		filterFlageEn = "";
		if (config.rf2inetFilter & FILTER_BUOY)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterBuoy\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Buoy</td>\n";

		filterFlageEn = "";
		if (config.rf2inetFilter & FILTER_POSITION)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterPosition\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Position</td>\n";

		html += "<td style=\"border:unset;\"></td>";
		html += "</tr></table></fieldset>\n";
		html += "</td></tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>INET2RF Filter:</b></td>\n";

		html += "<td align=\"center\">\n";
		if (config.inet2rf)
			html += "<fieldset id=\"inet2rfFilterGrp\">\n";
		else
			html += "<fieldset id=\"inet2rfFilterGrp\" disabled>\n";
		html += "<legend>Filter Internet to RF</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">";
		html += "<tr style=\"background:unset;\">";

		filterFlageEn = "";
		if (config.inet2rfFilter & FILTER_MESSAGE)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterMessage\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Message</td>\n";

		filterFlageEn = "";
		if (config.inet2rfFilter & FILTER_STATUS)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterStatus\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Status</td>\n";

		filterFlageEn = "";
		if (config.inet2rfFilter & FILTER_TELEMETRY)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterTelemetry\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Telemetry</td>\n";

		filterFlageEn = "";
		if (config.inet2rfFilter & FILTER_WX)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterWeather\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Weather</td>\n";

		filterFlageEn = "";
		if (config.inet2rfFilter & FILTER_OBJECT)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterObject\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Object</td>\n";

		filterFlageEn = "";
		if (config.inet2rfFilter & FILTER_ITEM)
			filterFlageEn = "checked";
		html += "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterItem\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Item</td>\n";

		filterFlageEn = "";
		if (config.inet2rfFilter & FILTER_QUERY)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterQuery\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Query</td>\n";

		filterFlageEn = "";
		if (config.inet2rfFilter & FILTER_BUOY)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterBuoy\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Buoy</td>\n";

		filterFlageEn = "";
		if (config.inet2rfFilter & FILTER_POSITION)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterPosition\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Position</td>\n";

		html += "<td style=\"border:unset;\"></td>";
		html += "</tr></table></fieldset>\n";
		html += "</td></tr>\n";

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitIGATEfilter'  name=\"commitIGATEfilter\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitIGATEfilter\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form><br />";
		if ((ESP.getFreeHeap() / 1000) > 120)
		{
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			size_t len = html.length();
			char *info = (char *)calloc(len, sizeof(char));
			if (info)
			{

				html.toCharArray(info, len, 0);
				html.clear();
				AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)info, len);

				response->addHeader("Sensor", "content");
				request->send(response);
				free(info);
			}
			else
			{
				log_d("Can't define calloc info size %d", len);
			}
		}
		// request->send(200, "text/html", html); // send to someones browser when asked
		//  if ((ESP.getFreeHeap() / 1000) > 100)
		//  {
		//  	request->send(200, "text/html", html); // send to someones browser when asked
		//  }
		//  else
		//  {
		//  	AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)html.c_str(), html.length());
		//  	response->addHeader("IGate", "content");
		//  	request->send(response);
		//  }
		// html.clear();
	}
}

void handle_digi(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	bool digiEn = false;
	bool digiAuto = false;
	bool posGPS = false;
	bool bcnEN = false;
	bool pos2RF = false;
	bool pos2INET = false;
	bool timeStamp = false;

	if (request->hasArg("commitDIGI"))
	{
		config.digiFilter = 0;
		for (int i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "digiEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						digiEn = true;
				}
			}
			if (request->argName(i) == "digiAuto")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						digiAuto = true;
				}
			}
			if (request->argName(i) == "myCall")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					name.toUpperCase();
					strcpy(config.digi_mycall, name.c_str());
				}
			}
			if (request->argName(i) == "mySSID")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_ssid = request->arg(i).toInt();
					if (config.digi_ssid > 15)
						config.digi_ssid = 3;
				}
			}
			if (request->argName(i) == "digiDelay")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_delay = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "digiPosInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "digiSTSInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_sts_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "digiPosLat")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_lat = request->arg(i).toFloat();
				}
			}

			if (request->argName(i) == "digiPosLon")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_lon = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "digiPosAlt")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_alt = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "digiPosSel")
			{
				if (request->arg(i) != "")
				{
					if (request->arg(i).toInt() == 1)
						posGPS = true;
				}
			}

			if (request->argName(i) == "digiTable")
			{
				if (request->arg(i) != "")
				{
					config.digi_symbol[0] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "digiSymbol")
			{
				if (request->arg(i) != "")
				{
					config.digi_symbol[1] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "digiPath")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "digiComment")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.digi_comment, request->arg(i).c_str());
				}
				else
				{
					memset(config.digi_comment, 0, sizeof(config.digi_comment));
				}
			}
			if (request->argName(i) == "texttouse")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.digi_phg, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "digiStatus")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.digi_status, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "digiPos2RF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2RF = true;
				}
			}
			if (request->argName(i) == "digiPos2INET")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2INET = true;
				}
			}
			if (request->argName(i) == "digiBcnEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						bcnEN = true;
				}
			}
			// Filter
			if (request->argName(i) == "FilterMessage")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.digiFilter |= FILTER_MESSAGE;
				}
			}

			if (request->argName(i) == "FilterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.digiFilter |= FILTER_TELEMETRY;
				}
			}

			if (request->argName(i) == "FilterStatus")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.digiFilter |= FILTER_STATUS;
				}
			}

			if (request->argName(i) == "FilterWeather")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.digiFilter |= FILTER_WX;
				}
			}

			if (request->argName(i) == "FilterObject")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.digiFilter |= FILTER_OBJECT;
				}
			}

			if (request->argName(i) == "FilterItem")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.digiFilter |= FILTER_ITEM;
				}
			}

			if (request->argName(i) == "FilterQuery")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.digiFilter |= FILTER_QUERY;
				}
			}
			if (request->argName(i) == "FilterBuoy")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.digiFilter |= FILTER_BUOY;
				}
			}
			if (request->argName(i) == "FilterPosition")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.digiFilter |= FILTER_POSITION;
				}
			}
			if (request->argName(i) == "digiTimeStamp")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						timeStamp = true;
				}
			}
			if (request->argName(i) == "digiTlmInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_tlm_interval = request->arg(i).toInt();
				}
			}

			String arg;
			for (int x = 0; x < 5; x++)
			{
				arg = "sensorCH" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.digi_tlm_sensor[x] = request->arg(i).toInt();
				}
				arg = "param" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.digi_tlm_PARM[x], request->arg(i).c_str());
					}
				}
				arg = "unit" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.digi_tlm_UNIT[x], request->arg(i).c_str());
					}
				}
				arg = "precision" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.digi_tlm_precision[x] = request->arg(i).toInt();
				}
				arg = "offset" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.digi_tlm_offset[x] = request->arg(i).toFloat();
				}
				for (int y = 0; y < 3; y++)
				{
					arg = "eqns" + String(x) + String((char)(y + 'a'));
					if (request->argName(i) == arg)
					{
						if (isValidNumber(request->arg(i)))
							config.digi_tlm_EQNS[x][y] = request->arg(i).toFloat();
					}
				}
			}
		}
		config.digi_en = digiEn;
		config.digi_auto = digiAuto;
		config.digi_gps = posGPS;
		config.digi_bcn = bcnEN;
		config.digi_loc2rf = pos2RF;
		config.digi_loc2inet = pos2INET;
		config.digi_timestamp = timeStamp;

		initInterval = true;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else
	{

		String html = "<script type=\"text/javascript\">\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		html += "document.getElementById(\"submitDIGI\").disabled=true;\n";
		html += "$.ajax({\n";
		html += "url: '/digi',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "alert(\"Submited Successfully\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";
		html += "</script>\n<script type=\"text/javascript\">\n";
		html += "function openWindowSymbol() {\n";
		html += "var i, l, options = [{\n";
		html += "value: 'first',\n";
		html += "text: 'First'\n";
		html += "}, {\n";
		html += "value: 'second',\n";
		html += "text: 'Second'\n";
		html += "}],\n";
		html += "newWindow = window.open(\"/symbol\", null, \"height=400,width=400,status=no,toolbar=no,menubar=no,titlebar=no,location=no\");\n";
		html += "}\n";

		html += "function setValue(symbol,table) {\n";
		html += "document.getElementById('digiSymbol').value = String.fromCharCode(symbol);\n";
		html += "if(table==1){\n document.getElementById('digiTable').value='/';\n";
		html += "}else if(table==2){\n document.getElementById('digiTable').value='\\\\';\n}\n";
		html += "document.getElementById('digiImgSymbol').src = \"http://aprs.dprns.com/symbols/icons/\"+symbol.toString()+'-'+table.toString()+'.png';\n";
		html += "\n}\n";
		html += "function calculatePHGR(){document.forms.formDIGI.texttouse.value=\"PHG\"+calcPower(document.forms.formDIGI.power.value)+calcHeight(document.forms.formDIGI.haat.value)+calcGain(document.forms.formDIGI.gain.value)+calcDirection(document.forms.formDIGI.direction.selectedIndex)}function Log2(e){return Math.log(e)/Math.log(2)}function calcPerHour(e){return e<10?e:String.fromCharCode(65+(e-10))}function calcHeight(e){return String.fromCharCode(48+Math.round(Log2(e/10),0))}function calcPower(e){if(e<1)return 0;if(e>=1&&e<4)return 1;if(e>=4&&e<9)return 2;if(e>=9&&e<16)return 3;if(e>=16&&e<25)return 4;if(e>=25&&e<36)return 5;if(e>=36&&e<49)return 6;if(e>=49&&e<64)return 7;if(e>=64&&e<81)return 8;if(e>=81)return 9}function calcDirection(e){if(e==\"0\")return\"0\";if(e==\"1\")return\"1\";if(e==\"2\")return\"2\";if(e==\"3\")return\"3\";if(e==\"4\")return\"4\";if(e==\"5\")return\"5\";if(e==\"6\")return\"6\";if(e==\"7\")return\"7\";if(e==\"8\")return\"8\"}function calcGain(e){return e>9?\"9\":e<0?\"0\":Math.round(e,0)}\n";
		html += "function selPrecision(idx) {\n";
		html += "var x=0;\n";
		html += "x = document.getElementsByName(\"precision\"+idx)[0].value;\n";
		html += "document.getElementsByName(\"eqns\"+idx+\"b\")[0].value=1/Math.pow(10,x);\n";
		html += "}\n";
		html += "function selOffset(idx) {\n";
		html += "var x=0;\n";
		html += "x = document.getElementsByName(\"offset\"+idx)[0].value;\n";
		html += "document.getElementsByName(\"eqns\"+idx+\"c\")[0].value=x*(-1);\n";
		html += "}\n";
		html += "</script>\n";

		/************************ DIGI Mode **************************/
		html += "<form id='formDIGI' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		// html += "<h2>[DIGI] Digital Repeater Mode</h2>\n";
		html += "<table>\n";
		// html += "<tr>\n";
		// html += "<th width=\"200\"><span><b>Setting</b></span></th>\n";
		// html += "<th><span><b>Value</b></span></th>\n";
		// html += "</tr>\n";
		html += "<th colspan=\"2\"><span><b>[DIGI] Digital Repeater Mode</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Enable:</b></td>\n";
		String digiEnFlag = "";
		if (config.digi_en)
			digiEnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiEnable\" value=\"OK\" " + digiEnFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Auto Enable:</b></td>\n";
		String digiAutoFlag = "";
		if (config.digi_auto)
			digiAutoFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiAuto\" value=\"OK\" " + digiAutoFlag + "><span class=\"slider round\"></span></label> <i>*Automatic enable when APRS-IS disconnected</i></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Station Callsign:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"" + String(config.digi_mycall) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Station SSID:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"mySSID\" id=\"mySSID\">\n";
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			if (config.digi_ssid == ssid)
			{
				html += "<option value=\"" + String(ssid) + "\" selected>" + String(ssid) + "</option>\n";
			}
			else
			{
				html += "<option value=\"" + String(ssid) + "\">" + String(ssid) + "</option>\n";
			}
		}
		html += "</select></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Station Symbol:</b></td>\n";
		String table = "1";
		if (config.digi_symbol[0] == 47)
			table = "1";
		if (config.digi_symbol[0] == 92)
			table = "2";
		html += "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"digiTable\" name=\"digiTable\" type=\"text\" value=\"" + String(config.digi_symbol[0]) + "\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"digiSymbol\" name=\"digiSymbol\" type=\"text\" value=\"" + String(config.digi_symbol[1]) + "\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"digiImgSymbol\" onclick=\"openWindowSymbol();\" src=\"http://aprs.dprns.com/symbols/icons/" + String((int)config.digi_symbol[1]) + "-" + table + ".png\"> <i>*Click icon for select symbol</i></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PATH:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"digiPath\" id=\"digiPath\">\n";
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			if (config.digi_path == pthIdx)
			{
				html += "<option value=\"" + String(pthIdx) + "\" selected>" + String(PATH_NAME[pthIdx]) + "</option>\n";
			}
			else
			{
				html += "<option value=\"" + String(pthIdx) + "\">" + String(PATH_NAME[pthIdx]) + "</option>\n";
			}
		}
		html += "</select></td>\n";
		// html += "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" id=\"digiPath\" name=\"digiPath\" type=\"text\" value=\"" + String(config.digi_path) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Text Comment:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"25\" size=\"30\" id=\"digiComment\" name=\"digiComment\" type=\"text\" value=\"" + String(config.digi_comment) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Text Status:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"60\" id=\"digiStatus\" name=\"digiStatus\" type=\"text\" value=\"" + String(config.digi_status) + "\" />  Interval:<input min=\"0\" max=\"3600\" step=\"1\" name=\"digiSTSInv\" type=\"number\" value=\"" + String(config.digi_sts_interval) + "\" />Sec.</td>\n";
		html += "</tr>\n";

		html += "<tr><td style=\"text-align: right;\"><b>Repeat Delay:</b></td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"100\" id=\"digiDelay\" name=\"digiDelay\" type=\"number\" value=\"" + String(config.digi_delay) + "\" /> mSec. <i>*0 is auto,Other random of delay time</i></td></tr>";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Time Stamp:</b></td>\n";
		String timeStampFlag = "";
		if (config.digi_timestamp)
			timeStampFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiTimeStamp\" value=\"OK\" " + timeStampFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr><td align=\"right\"><b>POSITION:</b></td>\n";
		html += "<td align=\"center\">\n";
		html += "<table>";
		String digiBcnEnFlag = "";
		if (config.digi_bcn)
			digiBcnEnFlag = "checked";

		html += "<tr><td style=\"text-align: right;\">Beacon:</td><td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiBcnEnable\" value=\"OK\" " + digiBcnEnFlag + "><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\">  Interval:<input min=\"0\" max=\"3600\" step=\"1\" id=\"digiPosInv\" name=\"digiPosInv\" type=\"number\" value=\"" + String(config.digi_interval) + "\" />Sec.</label></td></tr>";
		String digiPosFixFlag = "";
		String digiPosGPSFlag = "";
		String digiPos2RFFlag = "";
		String digiPos2INETFlag = "";
		if (config.digi_gps)
			digiPosGPSFlag = "checked=\"checked\"";
		else
			digiPosFixFlag = "checked=\"checked\"";

		if (config.digi_loc2rf)
			digiPos2RFFlag = "checked";
		if (config.digi_loc2inet)
			digiPos2INETFlag = "checked";
		html += "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"digiPosSel\" value=\"0\" " + digiPosFixFlag + "/>Fix <input type=\"radio\" name=\"digiPosSel\" value=\"1\" " + digiPosGPSFlag + "/>GPS </td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"digiPos2RF\" value=\"OK\" " + digiPos2RFFlag + "/>RF <input type=\"checkbox\" name=\"digiPos2INET\" value=\"OK\" " + digiPos2INETFlag + "/>Internet </td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" id=\"digiPosLat\" name=\"digiPosLat\" type=\"number\" value=\"" + String(config.digi_lat, 5) + "\" />degrees (positive for North, negative for South)</td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" id=\"digiPosLon\" name=\"digiPosLon\" type=\"number\" value=\"" + String(config.digi_lon, 5) + "\" />degrees (positive for East, negative for West)</td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" id=\"digiPosAlt\" name=\"digiPosAlt\" type=\"number\" value=\"" + String(config.digi_alt, 2) + "\" /> meter. *Value 0 is not send height</td></tr>\n";
		html += "</table></td>";
		html += "</tr>\n";
		delay(1);
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PHG:</b></td>\n";
		html += "<td align=\"center\">\n";
		html += "<table>";
		html += "<tr>\n";
		html += "<td align=\"right\">Radio TX Power</td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"power\" id=\"power\">\n";
		html += "<option value=\"1\" selected>1</option>\n";
		html += "<option value=\"5\">5</option>\n";
		html += "<option value=\"10\">10</option>\n";
		html += "<option value=\"15\">15</option>\n";
		html += "<option value=\"25\">25</option>\n";
		html += "<option value=\"35\">35</option>\n";
		html += "<option value=\"50\">50</option>\n";
		html += "<option value=\"65\">65</option>\n";
		html += "<option value=\"80\">80</option>\n";
		html += "</select> Watts</td>\n";
		html += "</tr>\n";
		html += "<tr><td style=\"text-align: right;\">Antenna Gain</td><td style=\"text-align: left;\"><input size=\"3\" min=\"0\" max=\"100\" step=\"0.1\" id=\"gain\" name=\"gain\" type=\"number\" value=\"6\" /> dBi</td></tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\">Height</td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"haat\" id=\"haat\">\n";
		int k = 10;
		for (uint8_t w = 0; w < 10; w++)
		{
			if (w == 0)
			{
				html += "<option value=\"" + String(k) + "\" selected>" + String(k) + "</option>\n";
			}
			else
			{
				html += "<option value=\"" + String(k) + "\">" + String(k) + "</option>\n";
			}
			k += k;
		}
		html += "</select> Feet</td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\">Antenna/Direction</td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"direction\" id=\"direction\">\n";
		html += "<option>Omni</option><option>NE</option><option>E</option><option>SE</option><option>S</option><option>SW</option><option>W</option><option>NW</option><option>N</option>\n";
		html += "</select></td>\n";
		html += "</tr>\n";
		html += "<tr><td align=\"right\"><b>PHG Text</b></td><td align=\"left\"><input name=\"texttouse\" type=\"text\" size=\"6\" style=\"background-color: rgb(97, 239, 170);\" value=\"" + String(config.digi_phg) + "\"/> <input type=\"button\" value=\"Calculate PHG\" onclick=\"javascript:calculatePHGR()\" /></td></tr>\n";

		html += "</table></tr>";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Filter:</b></td>\n";

		html += "<td align=\"center\">\n";
		html += "<fieldset id=\"FilterGrp\">\n";
		html += "<legend>Filter repeater</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">";
		html += "<tr style=\"background:unset;\">";

		String filterFlageEn = "";
		if (config.digiFilter & FILTER_MESSAGE)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterMessage\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Message</td>\n";

		filterFlageEn = "";
		if (config.digiFilter & FILTER_STATUS)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterStatus\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Status</td>\n";

		filterFlageEn = "";
		if (config.digiFilter & FILTER_TELEMETRY)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterTelemetry\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Telemetry</td>\n";

		filterFlageEn = "";
		if (config.digiFilter & FILTER_WX)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterWeather\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Weather</td>\n";

		filterFlageEn = "";
		if (config.digiFilter & FILTER_OBJECT)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterObject\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Object</td>\n";

		filterFlageEn = "";
		if (config.digiFilter & FILTER_ITEM)
			filterFlageEn = "checked";
		html += "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterItem\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Item</td>\n";

		filterFlageEn = "";
		if (config.digiFilter & FILTER_QUERY)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterQuery\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Query</td>\n";

		filterFlageEn = "";
		if (config.digiFilter & FILTER_BUOY)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterBuoy\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Buoy</td>\n";

		filterFlageEn = "";
		if (config.digiFilter & FILTER_POSITION)
			filterFlageEn = "checked";
		html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterPosition\" type=\"checkbox\" value=\"OK\" " + filterFlageEn + "/>Position</td>\n";

		html += "<td style=\"border:unset;\"></td>";
		html += "</tr></table></fieldset>\n";
		html += "</td></tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Telemetry:</b><br />(v=0->8280)</td>\n";
		html += "<td align=\"center\"><table>\n";
		html += "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"1000\" step=\"1\" id=\"digiTlmInv\" name=\"digiTlmInv\" type=\"number\" value=\"" + String(config.digi_tlm_interval) + "\" /> *Number of packets interval,<i>Example: 0 not send,1 send every packet</i></label></td></tr>";
		for (int ax = 0; ax < 5; ax++)
		{
			html += "<tr><td align=\"right\"><b>CH A" + String(ax + 1) + ":</b></td>\n";
			html += "<td align=\"center\">\n";
			html += "<table>";

			html += "<tr><td style=\"text-align: right;\">Sensor:</td>\n";
			html += "<td style=\"text-align: left;\">CH: ";
			html += "<select name=\"sensorCH" + String(ax) + "\" id=\"sensorCH" + String(ax) + "\">\n";
			for (uint8_t idx = 0; idx < 11; idx++)
			{
				if (idx == 0)
				{
					if (config.digi_tlm_sensor[ax] == idx)
					{
						html += "<option value=\"" + String(idx) + "\" selected>NONE</option>\n";
					}
					else
					{
						html += "<option value=\"" + String(idx) + "\">NONE</option>\n";
					}
				}
				else
				{
					if (config.digi_tlm_sensor[ax] == idx)
					{
						html += "<option value=\"" + String(idx) + "\" selected>SENSOR#" + String(idx) + "</option>\n";
					}
					else
					{
						html += "<option value=\"" + String(idx) + "\">SENSOR#" + String(idx) + "</option>\n";
					}
				}
			}
			html += "</select></td>\n";

			html += "<td style=\"text-align: left;\">Name: <input maxlength=\"10\" size=\"8\" name=\"param" + String(ax) + "\" type=\"text\" value=\"" + String(config.digi_tlm_PARM[ax]) + "\" /></td>\n";
			html += "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit" + String(ax) + "\" type=\"text\" value=\"" + String(config.digi_tlm_UNIT[ax]) + "\" /></td>\n";
			html += "<td style=\"text-align: left;\">Precision: <input min=\"0\" max=\"5\" step=\"1\" type=\"number\" style=\"width: 2em\" name=\"precision" + String(ax) + "\" type=\"text\" value=\"" + String(config.digi_tlm_precision[ax]) + "\"  onchange=\"selPrecision(" + String(ax) + ")\"/></td></tr>\n";

			html += "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns" + String(ax) + "a\" type=\"number\" value=\"" + String(config.digi_tlm_EQNS[ax][0], 5) + "\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns" + String(ax) + "b\" type=\"number\" value=\"" + String(config.digi_tlm_EQNS[ax][1], 5) + "\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns" + String(ax) + "c\" type=\"number\" value=\"" + String(config.digi_tlm_EQNS[ax][2], 5) + "\" /> (av<sup>2</sup>+bv+c) </td>\n";
			html += "<td style=\"text-align: left;\">Offset: <input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" type=\"number\" name=\"offset" + String(ax) + "\" type=\"text\" value=\"" + String(config.digi_tlm_offset[ax], 5) + "\" onchange=\"selOffset(" + String(ax) + ")\" /></td></tr>\n";
			html += "</table></td>";
			html += "</tr>\n";
		}
		html += "</table></td></tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitDIGI'  name=\"commitDIGI\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitDIGI\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form><br />";
		if ((ESP.getFreeHeap() / 1000) > 120)
		{
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			size_t len = html.length();
			char *info = (char *)calloc(len, sizeof(char));
			if (info)
			{

				html.toCharArray(info, len, 0);
				html.clear();
				AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)info, len);

				response->addHeader("Sensor", "content");
				request->send(response);
				free(info);
			}
			else
			{
				log_d("Can't define calloc info size %d", len);
			}
		}
		// request->send(200, "text/html", html); // send to someones browser when asked
		//  AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)html.c_str(), html.length());
		//  response->addHeader("Digi", "content");
		//  request->send(response);
		// html.clear();
	}
}

void handle_wx(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	bool En = false;
	bool posGPS = false;
	bool pos2RF = false;
	bool pos2INET = false;
	bool timeStamp = false;
	String arg = "";

	if (request->hasArg("commitWX"))
	{
		for (int x = 0; x < WX_SENSOR_NUM; x++)
			config.wx_sensor_enable[x] = false;

		for (int i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}
			if (request->argName(i) == "Object")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					strcpy(config.wx_object, name.c_str());
				}
				else
				{
					config.wx_object[0] = 0;
				}
			}
			if (request->argName(i) == "myCall")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					name.toUpperCase();
					strcpy(config.wx_mycall, name.c_str());
				}
			}
			if (request->argName(i) == "mySSID")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_ssid = request->arg(i).toInt();
					if (config.wx_ssid > 15)
						config.wx_ssid = 3;
				}
			}
			// if (request->argName(i) == "channel")
			// {
			// 	if (request->arg(i) != "")
			// 	{
			// 		if (isValidNumber(request->arg(i)))
			// 			config.wx_channel = request->arg(i).toInt();
			// 	}
			// }
			if (request->argName(i) == "PosInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "PosLat")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_lat = request->arg(i).toFloat();
				}
			}

			if (request->argName(i) == "PosLon")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_lon = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "PosAlt")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_alt = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "PosSel")
			{
				if (request->arg(i) != "")
				{
					if (request->arg(i).toInt() == 1)
						posGPS = true;
				}
			}

			if (request->argName(i) == "Path")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "Comment")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wx_comment, request->arg(i).c_str());
				}
				else
				{
					memset(config.wx_comment, 0, sizeof(config.wx_comment));
				}
			}
			if (request->argName(i) == "Pos2RF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2RF = true;
				}
			}
			if (request->argName(i) == "Pos2INET")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2INET = true;
				}
			}
			if (request->argName(i) == "wxTimeStamp")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						timeStamp = true;
				}
			}
			for (int x = 0; x < WX_SENSOR_NUM; x++)
			{
				arg = "senEn" + String(x);
				if (request->argName(i) == arg)
				{

					if (request->arg(i) != "")
					{
						if (String(request->arg(i)) == "OK")
							config.wx_sensor_enable[x] = true;
					}
				}
				arg = "sensorCH" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.wx_sensor_ch[x] = request->arg(i).toInt();
				}
				arg = "avgSel" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						if (request->arg(i).toInt() == 1)
							config.wx_sensor_avg[x] = true;
						else if (request->arg(i).toInt() == 0)
							config.wx_sensor_avg[x] = false;
					}
				}
			}
		}
		config.wx_en = En;
		config.wx_gps = posGPS;
		config.wx_2rf = pos2RF;
		config.wx_2inet = pos2INET;
		config.wx_timestamp = timeStamp;

		initInterval = true;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else
	{

		String html = "<script type=\"text/javascript\">\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		html += "document.getElementById(\"submitWX\").disabled=true;\n";
		html += "$.ajax({\n";
		html += "url: '/wx',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "alert(\"Submited Successfully\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";
		html += "</script>\n";

		/************************ WX Mode **************************/
		html += "<form id='formWX' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>[WX] Weather Station</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Enable:</b></td>\n";
		String EnFlag = "";
		if (config.wx_en)
			EnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + EnFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Station Callsign:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"" + String(config.wx_mycall) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Station SSID:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"mySSID\" id=\"mySSID\">\n";
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			if (config.wx_ssid == ssid)
			{
				html += "<option value=\"" + String(ssid) + "\" selected>" + String(ssid) + "</option>\n";
			}
			else
			{
				html += "<option value=\"" + String(ssid) + "\">" + String(ssid) + "</option>\n";
			}
		}
		html += "</select></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Object Name:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"9\" size=\"9\" name=\"Object\" type=\"text\" value=\"" + String(config.wx_object) + "\" /><i> *If not used, leave it blank.In use 3-9 charactor</i></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PATH:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"Path\" id=\"Path\">\n";
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			if (config.wx_path == pthIdx)
			{
				html += "<option value=\"" + String(pthIdx) + "\" selected>" + String(PATH_NAME[pthIdx]) + "</option>\n";
			}
			else
			{
				html += "<option value=\"" + String(pthIdx) + "\">" + String(PATH_NAME[pthIdx]) + "</option>\n";
			}
		}
		html += "</select></td>\n";
		// html += "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" name=\"Path\" type=\"text\" value=\"" + String(config.wx_path) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Text Comment:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"50\" name=\"Comment\" type=\"text\" value=\"" + String(config.wx_comment) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>Time Stamp:</b></td>\n";
		String timeStampFlag = "";
		if (config.wx_timestamp)
			timeStampFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wxTimeStamp\" value=\"OK\" " + timeStampFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";

		html += "<tr><td align=\"right\"><b>POSITION:</b></td>\n";
		html += "<td align=\"center\">\n";
		html += "<table>";
		html += "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" id=\"PosInv\" name=\"PosInv\" type=\"number\" value=\"" + String(config.wx_interval) + "\" />Sec.</td></tr>";
		String PosFixFlag = "";
		String PosGPSFlag = "";
		String Pos2RFFlag = "";
		String Pos2INETFlag = "";
		if (config.wx_gps)
			PosGPSFlag = "checked=\"checked\"";
		else
			PosFixFlag = "checked=\"checked\"";

		if (config.wx_2rf)
			Pos2RFFlag = "checked";
		if (config.wx_2inet)
			Pos2INETFlag = "checked";
		html += "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"PosSel\" value=\"0\" " + PosFixFlag + "/>Fix <input type=\"radio\" name=\"PosSel\" value=\"1\" " + PosGPSFlag + "/>GPS </td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"Pos2RF\" value=\"OK\" " + Pos2RFFlag + "/>RF <input type=\"checkbox\" name=\"Pos2INET\" value=\"OK\" " + Pos2INETFlag + "/>Internet </td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" name=\"PosLat\" type=\"number\" value=\"" + String(config.wx_lat, 5) + "\" />degrees (positive for North, negative for South)</td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" name=\"PosLon\" type=\"number\" value=\"" + String(config.wx_lon, 5) + "\" />degrees (positive for East, negative for West)</td></tr>\n";
		html += "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" name=\"PosAlt\" type=\"number\" value=\"" + String(config.wx_alt, 2) + "\" /> meter. *The altitude in meters(m) above sea level</td></tr>\n";
		html += "</table></td>";
		html += "</tr>\n";

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>PORT:</b></td>\n";
		// html += "<td style=\"text-align: left;\">\n";
		// html += "<select name=\"channel\" id=\"channel\">\n";
		// for (int i = 0; i < 5; i++)
		// {
		// 	if (config.wx_channel == i)
		// 		html += "<option value=\"" + String(i) + "\" selected>" + String(WX_PORT[i]) + " </option>\n";
		// 	else
		// 		html += "<option value=\"" + String(i) + "\" >" + String(WX_PORT[i]) + " </option>\n";
		// }
		// html += "</select>\n";
		// html += "</td>\n";
		// html += "</tr>\n";
		/************************ Sensor Config Mode **************************/
		// html += "<form id='formSENSOR' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		// html += "<table>\n";
		// html += "<th colspan=\"2\"><span><b>Sensor Config</b></span></th>\n";

		html += "<tr><td align=\"right\"><b>SENSOR:<br />Selection</b></td>\n";
		html += "<td align=\"center\">\n";
		html += "<table>";

		for (int ax = 0; ax < WX_SENSOR_NUM; ax++)
		{
			html += "<tr><td align=\"right\"><b>" + String(WX_SENSOR[ax]) + ":</b> \n";
			EnFlag = "";
			if (config.wx_sensor_enable[ax])
				EnFlag = "checked";
			html += "<label class=\"switch\"><input type=\"checkbox\" name=\"senEn" + String(ax) + "\" value=\"OK\" " + EnFlag + "><span class=\"slider round\"></span></label>";
			html += "</td>\n";

			// html += "<td style=\"text-align: lefe;\">Sensor:</td>\n";
			html += "<td style=\"text-align: left;\">Sensor Channel: ";
			html += "<select name=\"sensorCH" + String(ax) + "\" id=\"sensorCH" + String(ax) + "\">\n";
			for (uint8_t idx = 0; idx < 11; idx++)
			{
				if (idx == 0)
				{
					if (config.wx_sensor_ch[ax] == idx)
					{
						html += "<option value=\"" + String(idx) + "\" selected>NONE</option>\n";
					}
					else
					{
						html += "<option value=\"" + String(idx) + "\">NONE</option>\n";
					}
				}
				else
				{
					if (config.wx_sensor_ch[ax] == idx)
					{
						html += "<option value=\"" + String(idx) + "\" selected>SENSOR#" + String(idx) + "</option>\n";
					}
					else
					{
						html += "<option value=\"" + String(idx) + "\">SENSOR#" + String(idx) + "</option>\n";
					}
				}
			}
			html += "</select>\n";
			String avgFlag = "";
			String sampleFlag = "";
			if (config.wx_sensor_avg[ax])
				avgFlag = "checked=\"checked\"";
			else
				sampleFlag = "checked=\"checked\"";
			html += "<input type=\"radio\" name=\"avgSel" + String(ax) + "\" value=\"0\" " + sampleFlag + "/>Sample <input type=\"radio\" name=\"avgSel" + String(ax) + "\" value=\"1\" " + avgFlag + "/>Average";
			html += "</td></tr>";
		}
		html += "</table></td></tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitWX'  name=\"commitWX\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitWX\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form><br />";
		if ((ESP.getFreeHeap() / 1000) > 120)
		{
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			size_t len = html.length();
			char *info = (char *)calloc(len, sizeof(char));
			if (info)
			{

				html.toCharArray(info, len, 0);
				html.clear();
				AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)info, len);

				response->addHeader("Sensor", "content");
				request->send(response);
				free(info);
			}
			else
			{
				log_d("Can't define calloc info size %d", len);
			}
		}
		// request->send(200, "text/html", html); // send to someones browser when asked
		//  if ((ESP.getFreeHeap() / 1000) > 110)
		//  {
		//  	request->send(200, "text/html", html); // send to someones browser when asked
		//  }
		//  else
		//  {
		//  	AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)html.c_str(), html.length());
		//  	response->addHeader("Weather", "content");
		//  	request->send(response);
		//  }
		// html.clear();
	}
}

void handle_tlm(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	bool En = false;
	bool pos2RF = false;
	bool pos2INET = false;
	String arg = "";

	if (request->hasArg("commitTLM"))
	{
		for (int i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}
			if (request->argName(i) == "myCall")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					name.toUpperCase();
					strcpy(config.tlm0_mycall, name.c_str());
				}
			}
			if (request->argName(i) == "mySSID")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tlm0_ssid = request->arg(i).toInt();
					if (config.tlm0_ssid > 15)
						config.tlm0_ssid = 3;
				}
			}
			if (request->argName(i) == "infoInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tlm0_info_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "dataInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tlm0_data_interval = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "Path")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tlm0_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "Comment")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.tlm0_comment, request->arg(i).c_str());
				}
				else
				{
					memset(config.tlm0_comment, 0, sizeof(config.tlm0_comment));
				}
			}
			if (request->argName(i) == "Pos2RF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2RF = true;
				}
			}
			if (request->argName(i) == "Pos2INET")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2INET = true;
				}
			}
			for (int x = 0; x < 13; x++)
			{
				arg = "sensorCH" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.tml0_data_channel[x] = request->arg(i).toInt();
				}
				arg = "param" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.tlm0_PARM[x], request->arg(i).c_str());
					}
				}
				arg = "unit" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.tlm0_UNIT[x], request->arg(i).c_str());
					}
				}
				if (x < 5)
				{
					for (int y = 0; y < 3; y++)
					{
						arg = "eqns" + String(x) + String((char)(y + 'a'));
						if (request->argName(i) == arg)
						{
							if (isValidNumber(request->arg(i)))
								config.tlm0_EQNS[x][y] = request->arg(i).toFloat();
						}
					}
				}
			}
			uint8_t b = 1;
			for (int x = 0; x < 8; x++)
			{
				arg = "bitact" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
					{
						if (request->arg(i).toInt() == 1)
						{
							config.tlm0_BITS_Active |= b;
						}
						else
						{
							config.tlm0_BITS_Active &= ~b;
						}
					}
				}
				b <<= 1;
			}
		}
		config.tlm0_en = En;
		config.tlm0_2rf = pos2RF;
		config.tlm0_2inet = pos2INET;

		initInterval = true;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else
	{

		String html = "<script type=\"text/javascript\">\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		html += "document.getElementById(\"submitTLM\").disabled=true;\n";
		html += "$.ajax({\n";
		html += "url: '/tlm',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "alert(\"Submited Successfully\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";
		html += "</script>\n";

		/************************ TLM Mode **************************/
		html += "<form id='formTLM' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>System Telemetry</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Enable:</b></td>\n";
		String EnFlag = "";
		if (config.tlm0_en)
			EnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + EnFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Station Callsign:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"" + String(config.tlm0_mycall) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Station SSID:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"mySSID\" id=\"mySSID\">\n";
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			if (config.tlm0_ssid == ssid)
			{
				html += "<option value=\"" + String(ssid) + "\" selected>" + String(ssid) + "</option>\n";
			}
			else
			{
				html += "<option value=\"" + String(ssid) + "\">" + String(ssid) + "</option>\n";
			}
		}
		html += "</select></td>\n";
		html += "</tr>\n";

		html += "<tr>\n";
		html += "<td align=\"right\"><b>PATH:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"Path\" id=\"Path\">\n";
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			if (config.tlm0_path == pthIdx)
			{
				html += "<option value=\"" + String(pthIdx) + "\" selected>" + String(PATH_NAME[pthIdx]) + "</option>\n";
			}
			else
			{
				html += "<option value=\"" + String(pthIdx) + "\">" + String(PATH_NAME[pthIdx]) + "</option>\n";
			}
		}
		html += "</select></td>\n";
		// html += "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" name=\"Path\" type=\"text\" value=\"" + String(config.tlm0_path) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Text Comment:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"50\" name=\"Comment\" type=\"text\" value=\"" + String(config.tlm0_comment) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<tr><td style=\"text-align: right;\">Info Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" name=\"infoInv\" type=\"number\" value=\"" + String(config.tlm0_info_interval) + "\" />Sec.</td></tr>";
		html += "<tr><td style=\"text-align: right;\">Data Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" name=\"dataInv\" type=\"number\" value=\"" + String(config.tlm0_data_interval) + "\" />Sec.</td></tr>";

		String Pos2RFFlag = "";
		String Pos2INETFlag = "";
		if (config.tlm0_2rf)
			Pos2RFFlag = "checked";
		if (config.tlm0_2inet)
			Pos2INETFlag = "checked";
		html += "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"Pos2RF\" value=\"OK\" " + Pos2RFFlag + "/>RF <input type=\"checkbox\" name=\"Pos2INET\" value=\"OK\" " + Pos2INETFlag + "/>Internet </td></tr>\n";

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>Time Stamp:</b></td>\n";
		// String timeStampFlag = "";
		// if (config.wx_timestamp)
		// 	timeStampFlag = "checked";
		// html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wxTimeStamp\" value=\"OK\" " + timeStampFlag + "><span class=\"slider round\"></span></label></td>\n";
		// html += "</tr>\n";
		for (int ax = 0; ax < 5; ax++)
		{
			html += "<tr><td align=\"right\"><b>Channel A" + String(ax + 1) + ":</b></td>\n";
			html += "<td align=\"center\">\n";
			html += "<table>";

			// html += "<tr><td style=\"text-align: right;\">Name:</td><td style=\"text-align: center;\"><i>Sensor Type</i></td><td style=\"text-align: center;\"><i>Parameter</i></td><td style=\"text-align: center;\"><i>Unit</i></td></tr>\n";

			html += "<tr><td style=\"text-align: right;\">Type/Name:</td>\n";
			html += "<td style=\"text-align: left;\">Sensor Type: ";
			html += "<select name=\"sensorCH" + String(ax) + "\" id=\"sensorCH" + String(ax) + "\">\n";
			for (uint8_t idx = 0; idx < SYSTEM_LEN; idx++)
			{
				if (config.tml0_data_channel[ax] == idx)
				{
					html += "<option value=\"" + String(idx) + "\" selected>" + String(SYSTEM_NAME[idx]) + "</option>\n";
				}
				else
				{
					html += "<option value=\"" + String(idx) + "\">" + String(SYSTEM_NAME[idx]) + "</option>\n";
				}
			}
			html += "</select></td>\n";

			html += "<td style=\"text-align: left;\">Parameter: <input maxlength=\"10\" size=\"8\" name=\"param" + String(ax) + "\" type=\"text\" value=\"" + String(config.tlm0_PARM[ax]) + "\" /></td>\n";
			html += "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit" + String(ax) + "\" type=\"text\" value=\"" + String(config.tlm0_UNIT[ax]) + "\" /></td></tr>\n";
			html += "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.0001\" name=\"eqns" + String(ax) + "a\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][0], 3) + "\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.0001\" name=\"eqns" + String(ax) + "b\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][1], 3) + "\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.0001\" name=\"eqns" + String(ax) + "c\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][2], 3) + "\" /> (av<sup>2</sup>+bv+c)</td></tr>\n";
			html += "</table></td>";
			html += "</tr>\n";
		}

		uint8_t b = 1;
		for (int ax = 0; ax < 8; ax++)
		{
			html += "<tr><td align=\"right\"><b>Channel B" + String(ax + 1) + ":</b></td>\n";
			html += "<td align=\"center\">\n";
			html += "<table>";

			// html += "<tr><td style=\"text-align: right;\">Type/Name:</td>\n";
			html += "<td style=\"text-align: left;\">Type: ";
			html += "<select name=\"sensorCH" + String(ax + 5) + "\" id=\"sensorCH" + String(ax) + "\">\n";
			for (uint8_t idx = 0; idx < SYSTEM_BIT_LEN; idx++)
			{
				if (config.tml0_data_channel[ax + 5] == idx)
				{
					html += "<option value=\"" + String(idx) + "\" selected>" + String(SYSTEM_BITS_NAME[idx]) + "</option>\n";
				}
				else
				{
					html += "<option value=\"" + String(idx) + "\">" + String(SYSTEM_BITS_NAME[idx]) + "</option>\n";
				}
			}
			html += "</select></td>\n";

			html += "<td style=\"text-align: left;\">Parameter: <input maxlength=\"10\" size=\"8\" name=\"param" + String(ax + 5) + "\" type=\"text\" value=\"" + String(config.tlm0_PARM[ax + 5]) + "\" /></td>\n";
			html += "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit" + String(ax + 5) + "\" type=\"text\" value=\"" + String(config.tlm0_UNIT[ax + 5]) + "\" /></td>\n";
			String LowFlag = "", HighFlag = "";
			if (config.tlm0_BITS_Active & b)
				HighFlag = "checked=\"checked\"";
			else
				LowFlag = "checked=\"checked\"";
			html += "<td style=\"text-align: left;\"> Active:<input type=\"radio\" name=\"bitact" + String(ax) + "\" value=\"0\" " + LowFlag + "/>LOW <input type=\"radio\" name=\"bitact" + String(ax) + "\" value=\"1\" " + HighFlag + "/>HIGH </td>\n";
			html += "</tr>\n";
			// html += "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax + 1) + "a\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][0], 3) + "\" />  b:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax + 1) + "b\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][1], 3) + "\" /> c:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax + 1) + "c\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][2], 3) + "\" /> (av<sup>2</sup>+bv+c)</td></tr>\n";
			html += "</table></td>";
			html += "</tr>\n";
			b <<= 1;
		}

		// html += "<tr><td align=\"right\"><b>Parameter Name:</b></td>\n";
		// html += "<td align=\"center\">\n";
		// html += "<table>";

		// // html += "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" name=\"PosLat\" type=\"number\" value=\"" + String(config.wx_lat, 5) + "\" />degrees (positive for North, negative for South)</td></tr>\n";
		// // html += "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" name=\"PosLon\" type=\"number\" value=\"" + String(config.wx_lon, 5) + "\" />degrees (positive for East, negative for West)</td></tr>\n";
		// // html += "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" name=\"PosAlt\" type=\"number\" value=\"" + String(config.wx_alt, 2) + "\" /> meter. *Value 0 is not send height</td></tr>\n";
		// html += "</table></td>";
		// html += "</tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitTLM'  name=\"commitTLM\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitTLM\"/>\n";
		html += "</td></tr></table><br />\n";		
		html += "</form><br />";
		request->send(200, "text/html", html); // send to someones browser when asked
	}
}

extern TaskHandle_t taskSensorHandle;

void handle_sensor(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);
	String arg = "";

	if (request->hasArg("commitSENSOR"))
	{
		vTaskSuspend(taskSensorHandle);
		for (int x = 0; x < SENSOR_NUMBER; x++)
		{
			config.sensor[x].enable = false;
		}
		for (int i = 0; i < request->args(); i++)
		{

			for (int x = 0; x < SENSOR_NUMBER; x++)
			{
				arg = "En" + String(x);
				if (request->argName(i) == arg)
				{

					if (request->arg(i) != "")
					{
						if (String(request->arg(i)) == "OK")
							config.sensor[x].enable = true;
					}
				}
				arg = "sensorCH" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.sensor[x].type = request->arg(i).toInt();
				}
				arg = "sensorP" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.sensor[x].port = request->arg(i).toInt();
				}
				arg = "address" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.sensor[x].address = request->arg(i).toInt();
				}
				arg = "sample" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.sensor[x].samplerate = request->arg(i).toInt();
				}
				arg = "avg" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.sensor[x].averagerate = request->arg(i).toInt();
				}
				arg = "param" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.sensor[x].parm, request->arg(i).c_str());
					}
				}
				arg = "unit" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.sensor[x].unit, request->arg(i).c_str());
					}
				}
				for (int y = 0; y < 3; y++)
				{
					arg = "eqns" + String(x) + String((char)(y + 'a'));
					if (request->argName(i) == arg)
					{
						if (isValidNumber(request->arg(i)))
							config.sensor[x].eqns[y] = request->arg(i).toFloat();
					}
				}
				//}
			}
		}

		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
		vTaskResume(taskSensorHandle);
	}
	else
	{

		String html = "<script type=\"text/javascript\">\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		html += "document.getElementById(\"submitSENSOR\").disabled=true;\n";
		html += "$.ajax({\n";
		html += "url: '/sensor',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "alert(\"Submited Successfully\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";
		html += "function setElm(name,val) {\n";
		html += "document.getElementById(name).value=val;\n";
		html += "};\n";
		html += "function selSensorType(idx) {\n";
		html += "var x=0;\n";
		html += "var parm=\"param\"+idx;\n";
		html += "var unit=\"unit\"+idx;\n";
		html += "x = document.getElementById(\"sensorCH\"+idx).value;\n";
		html += "if (x==1) {\n";
		html += "setElm(parm,\"Co2\");";
		html += "setElm(unit,\"ppm\");\n";
		html += "}else if (x==2) {\n";
		html += "setElm(parm,\"CH2O\");";
		html += "setElm(unit,\"μg/m³\");\n";
		html += "}else if (x==3) {\n";
		html += "setElm(parm,\"TVOC\");";
		html += "setElm(unit,\"μg/m³\");\n";
		html += "}else if (x==4) {\n";
		html += "setElm(parm,\"PM2.5\");";
		html += "setElm(unit,\"μg/m³\");\n";
		html += "}else if (x==5) {\n";
		html += "setElm(parm,\"PM10.0\");";
		html += "setElm(unit,\"μg/m³\");\n";
		html += "}else if (x==6) {\n";
		html += "setElm(parm,\"Temperature\");";
		html += "setElm(unit,\"°C\");\n";
		html += "}else if (x==7) {\n";
		html += "setElm(parm,\"Humidity\");";
		html += "setElm(unit,\"%RH\");\n";
		html += "}else if (x==8) {\n";
		html += "setElm(parm,\"Pressure\");";
		html += "setElm(unit,\"hPa\");\n";
		html += "}else if (x==9) {\n";
		html += "setElm(parm,\"WindSpeed\");";
		html += "setElm(unit,\"kPh\");\n";
		html += "}else if (x==10) {\n";
		html += "setElm(parm,\"WindCourse\");";
		html += "setElm(unit,\"°\");\n";
		html += "}else if (x==11) {\n";
		html += "setElm(parm,\"Rain\");";
		html += "setElm(unit,\"mm\");\n";
		html += "}else if (x==12) {\n";
		html += "setElm(parm,\"Luminosity\");";
		html += "setElm(unit,\"W/m³\");\n";
		html += "}else if (x==13) {\n";
		html += "setElm(parm,\"SoilTemp\");";
		html += "setElm(unit,\"°C\");\n";
		html += "}else if (x==14) {\n";
		html += "setElm(parm,\"SoilMoisture\");";
		html += "setElm(unit,\"%VWC\");\n";
		html += "}else if (x==15) {\n";
		html += "setElm(parm,\"WaterTemp\");";
		html += "setElm(unit,\"°C\");\n";
		html += "}else if (x==16) {\n";
		html += "setElm(parm,\"WaterTDS\");";
		html += "setElm(unit,\" \");\n";
		html += "}else if (x==17) {\n";
		html += "setElm(parm,\"WaterLevel\");";
		html += "setElm(unit,\"mm\");\n";
		html += "}else if (x==18) {\n";
		html += "setElm(parm,\"WaterFlow\");";
		html += "setElm(unit,\"L/min\");\n";
		html += "}else if (x==19) {\n";
		html += "setElm(parm,\"Voltage\");";
		html += "setElm(unit,\"V\");\n";
		html += "}else if (x==20) {\n";
		html += "setElm(parm,\"Current\");";
		html += "setElm(unit,\"A\");\n";
		html += "}else if (x==21) {\n";
		html += "setElm(parm,\"Power\");";
		html += "setElm(unit,\"W\");\n";
		html += "}else if (x==22) {\n";
		html += "setElm(parm,\"Energy\");";
		html += "setElm(unit,\"Wh\");\n";
		html += "}else if (x==23) {\n";
		html += "setElm(parm,\"Frequency\");";
		html += "setElm(unit,\"Hz\");\n";
		html += "}else if (x==24) {\n";
		html += "setElm(parm,\"PF\");";
		html += "setElm(unit,\" \");\n";
		html += "}else if (x==25) {\n";
		html += "setElm(parm,\"Satellite\");";
		html += "setElm(unit,\" \");\n";
		html += "}else if (x==26) {\n";
		html += "setElm(parm,\"HDOP\");";
		html += "setElm(unit,\" \");\n";
		html += "}else if (x==27) {\n";
		html += "setElm(parm,\"Battery\");";
		html += "setElm(unit,\"V\");\n";
		html += "}else if (x==28) {\n";
		html += "setElm(parm,\"BattLevel\");";
		html += "setElm(unit,\"%\");\n";
		html += "}\n}\n";

		html += "function selSensor(idx) {\n";
		html += "var x=0;\n";
		html += "x = document.getElementById(\"sensorP\"+idx).value;\n";
		html += "if (x>=10 && x<=13) {\n";
#ifdef TTGO_T_Beam_S3_SUPREME_V3
		html += "document.getElementById(\"address\"+idx).value=119;\n";
#else
		html += "document.getElementById(\"address\"+idx).value=118;\n";
#endif
		html += "}else if (x==16 || x==17) {\n";
		html += "document.getElementById(\"address\"+idx).value=90;\n";
		html += "}else if (x==23) {\n";
		html += "document.getElementById(\"address\"+idx).value=1;\n";
		html += "}else if (x==24 || x==25) {\n";
		html += "document.getElementById(\"address\"+idx).value=1000;\n";
		html += "}else{\n";
		html += "document.getElementById(\"address\"+idx).value=0;\n";
		html += "}\n}\n";
		html += "</script>\n";

		/************************ Sensor Monitor **************************/
		// html += "<form id='formSENSOR' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"5\"><span><b>Sensor Monitor</b></span></th>\n";
		int ax = 0;
		for (int r = 0; r < 3; r++)
		{
			html += "<tr>\n";
			for (int c = 0; c < 5; c++)
			{

				html += "<td align=\"center\">\n";
				if (config.sensor[ax].enable)
					html += "<fieldset id=\"SenGrp" + String(ax + 1) + "\">\n";
				else
					html += "<fieldset id=\"SenGrp" + String(ax + 1) + "\" disabled>\n";

				html += "<legend>SEN#" + String(ax + 1) + "-" + String(config.sensor[ax].parm) + "</legend>\n";
				html += "<input id=\"sVal" + String(ax) + "\" style=\"text-align:right;\" size=\"5\" type=\"text\" value=\"" + String(sen[ax].sample, 2) + "\" readonly/> " + String(config.sensor[ax].unit) + "\n";
				html += "</td>\n";
				ax++;
				if (ax >= SENSOR_NUMBER)
					break;
			}
			html += "</tr>\n";
			if (ax >= SENSOR_NUMBER)
				break;
		}
		html += "</table>< /br>\n";

		/************************ Sensor Config Mode **************************/
		html += "<form id='formSENSOR' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>Sensor Config</b></span></th>\n";
		String EnFlag = "";

		for (int ax = 0; ax < SENSOR_NUMBER; ax++)
		{
			html += "<tr><td align=\"right\"><b>SENSOR#" + String(ax + 1) + ":</b><br />\n";
			EnFlag = "";
			if (config.sensor[ax].enable)
				EnFlag = "checked";
			html += "<label class=\"switch\"><input type=\"checkbox\" name=\"En" + String(ax) + "\" value=\"OK\" " + EnFlag + "><span class=\"slider round\"></span></label>";
			html += "</td><td align=\"center\">\n";
			html += "<table>";

			html += "<tr><td style=\"text-align: right;\">Type:</td>\n";
			html += "<td style=\"text-align: left;\">";
			html += "<select name=\"sensorCH" + String(ax) + "\" id=\"sensorCH" + String(ax) + "\" onchange=\"selSensorType(" + String(ax) + ")\">\n";
			// for (uint8_t idx = 0; idx < SENSOR_NAME_NUM; idx++)
			// {
			// 	if (config.sensor[ax].type == idx)
			// 	{
			// 		html += "<option value=\"" + String(idx) + "\" selected>" + String(SENSOR_NAME[idx]) + "</option>\n";
			// 	}
			// 	else
			// 	{
			// 		html += "<option value=\"" + String(idx) + "\">" + String(SENSOR_NAME[idx]) + "</option>\n";
			// 	}
			// }
			html += "</select></td>\n";

			html += "<td style=\"text-align: left;\">Name: <input maxlength=\"15\" size=\"15\" name=\"param" + String(ax) + "\" id=\"param" + String(ax) + "\" type=\"text\" value=\"" + String(config.sensor[ax].parm) + "\" /></td>\n";
			html += "<td style=\"text-align: left;\">Unit: <input maxlength=\"10\" size=\"5\" name=\"unit" + String(ax) + "\" id=\"unit" + String(ax) + "\" type=\"text\" value=\"" + String(config.sensor[ax].unit) + "\" /></td></tr>\n";
			// html += "<tr><td style=\"text-align: right;\">Port:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax) + "a\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[0], 3) + "\" />  b:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax) + "b\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[1], 3) + "\" /> c:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax) + "c\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[2], 3) + "\" /> (av<sup>2</sup>+bv+c)</td></tr>\n";
			html += "<tr><td style=\"text-align: right;\">PORT:</td>\n";
			html += "<td style=\"text-align: left;\">";
			html += "<select name=\"sensorP" + String(ax) + "\" id=\"sensorP" + String(ax) + "\" onchange=\"selSensor(" + String(ax) + ")\">\n";
			// for (uint8_t idx = 0; idx < SENSOR_PORT_NUM; idx++)
			// {
			// 	if (config.sensor[ax].port == idx)
			// 	{
			// 		html += "<option value=\"" + String(idx) + "\" selected>" + String(SENSOR_PORT[idx]) + "</option>\n";
			// 	}
			// 	else
			// 	{
			// 		html += "<option value=\"" + String(idx) + "\">" + String(SENSOR_PORT[idx]) + "</option>\n";
			// 	}
			// }
			html += "</select></td>\n";
			html += "<td style=\"text-align: left;\">Addr/Reg/GPIO: <input style=\"text-align:right;\" min=\"0\" max=\"6500\" step=\"1\" name=\"address" + String(ax) + "\" id=\"address" + String(ax) + "\" type=\"number\" value=\"" + String(config.sensor[ax].address) + "\" /></td>\n";
			html += "<td style=\"text-align: left;\">Sample: <input style=\"text-align:right;\" min=\"0\" max=\"9999\" step=\"1\" name=\"sample" + String(ax) + "\" type=\"number\" value=\"" + String(config.sensor[ax].samplerate) + "\" />Sec.\n";
			html += "Average: <input style=\"text-align:right;\" min=\"0\" max=\"999\" step=\"1\" name=\"avg" + String(ax) + "\" type=\"number\" value=\"" + String(config.sensor[ax].averagerate) + "\" />Sec.</td></tr>\n";
			html += "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-999\" max=\"999\" step=\"0.00001\" name=\"eqns" + String(ax) + "a\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[0], 5) + "\" />  b:<input min=\"-999\" max=\"999\" step=\"0.00001\" name=\"eqns" + String(ax) + "b\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[1], 5) + "\" /> c:<input min=\"-999\" max=\"999\" step=\"0.00001\" name=\"eqns" + String(ax) + "c\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[2], 5) + "\" /> (av<sup>2</sup>+bv+c)</td></tr>\n";
			html += "</table></td>";
			html += "</tr>\n";
		}

		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitSENSOR'  name=\"commitSENSOR\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitSENSOR\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form><br />";

		html += "<script type=\"text/javascript\">\n";
		html += "if (typeof typeArry === 'undefined'){let typeArry = [];};\n";
		html += "typeArry = new Array(";
		for (uint8_t idx = 0; idx < SENSOR_NAME_NUM; idx++)
		{
			html += "'" + String(SENSOR_NAME[idx]) + "'";
			if (idx < SENSOR_NAME_NUM - 1)
				html += ",";
		}
		html += ");\n";
		html += "if (typeof portArry === 'undefined'){let portArry = [];};\n";
		html += "portArry = new Array(";
		for (uint8_t idx = 0; idx < SENSOR_PORT_NUM; idx++)
		{
			html += "'" + String(SENSOR_PORT[idx]) + "'";
			if (idx < SENSOR_PORT_NUM - 1)
				html += ",";
		}
		html += ");\n";
		// html += "delete typeSel;delete listType;delete portSel;delete listPort;\n";
		html += "if (typeof typeSel === 'undefined'){var typeSel = [];};\n";
		html += "if (typeof listType === 'undefined'){var listType = [];};\n";
		html += "if (typeof portSel === 'undefined'){var portSel = [];};\n";
		html += "if (typeof listPort === 'undefined'){var listPort = [];};\n";
		for (int i = 0; i < 10; i++)
		{
			html += "listType[" + String(i) + "] = document.querySelector('#sensorCH" + String(i) + "');typeSel[" + String(i) + "]=" + String(config.sensor[i].type) + ";\n";
			html += "listPort[" + String(i) + "] = document.querySelector('#sensorP" + String(i) + "');portSel[" + String(i) + "]=" + String(config.sensor[i].port) + ";\n";
		}

		html += "for (let n = 0; n < 10; n++){\n";
		html += "for (let i = 0; i < typeArry.length; i++) {\n";
		html += "const optionType = new Option(typeArry[i], i);\n";
		html += "listType[n].add(optionType, undefined);\n";
		html += "};\n";
		html += "listType[n].options[typeSel[n]].selected = true;\n";
		html += "for (let p = 0; p < portArry.length; p++) {\n";
		html += "const optionPort = new Option(portArry[p], p);\n";
		html += "listPort[n].add(optionPort, undefined);\n";
		html += "};\n";
		html += "listPort[n].options[portSel[n]].selected = true;\n";
		html += "};\n";

		html += "</script>\n";
		log_d("FreeHeap=%i", ESP.getFreeHeap() / 1000);
		if ((ESP.getFreeHeap() / 1000) > 120)
		{
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			// 	AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)html.c_str(), html.length());
			// 	response->addHeader("Sensor", "/");
			// 	request->send(response);
			// 	delay(1000);
			// }
			// html.clear();
			size_t len = html.length();
			char *info = (char *)calloc(len, sizeof(char));
			if (info)
			{

				html.toCharArray(info, len, 0);
				html.clear();
				AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)info, len);

				response->addHeader("Sensor", "content");
				request->send(response);
				free(info);
			}
			else
			{
				log_d("Can't define calloc info size %d", len);
			}
		}
	}
}

void handle_tracker(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	bool trakerEn = false;
	bool smartEn = false;
	bool compEn = false;

	bool posGPS = false;
	bool bcnEN = false;
	bool pos2RF = false;
	bool pos2INET = false;
	bool optCST = false;
	bool optAlt = false;
	bool optBat = false;
	bool optSat = false;
	bool timeStamp = false;

	if (request->hasArg("commitTRACKER"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "trackerEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						trakerEn = true;
				}
			}
			if (request->argName(i) == "smartBcnEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						smartEn = true;
				}
			}
			if (request->argName(i) == "compressEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						compEn = true;
				}
			}
			if (request->argName(i) == "trackerOptCST")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						optCST = true;
				}
			}
			if (request->argName(i) == "trackerOptAlt")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						optAlt = true;
				}
			}
			if (request->argName(i) == "trackerOptBat")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						optBat = true;
				}
			}
			if (request->argName(i) == "trackerOptSat")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						optSat = true;
				}
			}
			if (request->argName(i) == "myCall")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					name.toUpperCase();
					strcpy(config.trk_mycall, name.c_str());
				}
			}
			if (request->argName(i) == "trackerObject")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					strcpy(config.trk_item, name.c_str());
				}
				else
				{
					memset(config.trk_item, 0, sizeof(config.trk_item));
				}
			}
			if (request->argName(i) == "mySSID")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_ssid = request->arg(i).toInt();
					if (config.trk_ssid > 15)
						config.trk_ssid = 13;
				}
			}
			if (request->argName(i) == "trackerPosInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "trkSTSInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_sts_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "trackerPosLat")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_lat = request->arg(i).toFloat();
				}
			}

			if (request->argName(i) == "trackerPosLon")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_lon = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "trackerPosAlt")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_alt = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "trackerPosSel")
			{
				if (request->arg(i) != "")
				{
					if (request->arg(i).toInt() == 1)
						posGPS = true;
				}
			}
			if (request->argName(i) == "hspeed")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_hspeed = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "lspeed")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_lspeed = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "slowInterval")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_slowinterval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "maxInterval")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_maxinterval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "minInterval")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_mininterval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "minAngle")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_minangle = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "trackerTable")
			{
				if (request->arg(i) != "")
				{
					config.trk_symbol[0] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "trackerSymbol")
			{
				if (request->arg(i) != "")
				{
					config.trk_symbol[1] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "moveTable")
			{
				if (request->arg(i) != "")
				{
					config.trk_symmove[0] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "moveSymbol")
			{
				if (request->arg(i) != "")
				{
					config.trk_symmove[1] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "stopTable")
			{
				if (request->arg(i) != "")
				{
					config.trk_symstop[0] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "stopSymbol")
			{
				if (request->arg(i) != "")
				{
					config.trk_symstop[1] = request->arg(i).charAt(0);
				}
			}

			if (request->argName(i) == "trackerPath")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "trkMicEType")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_mice_type = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "trackerComment")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.trk_comment, request->arg(i).c_str());
				}
				else
				{
					memset(config.trk_comment, 0, sizeof(config.trk_comment));
				}
			}
			if (request->argName(i) == "trkStatus")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.trk_status, request->arg(i).c_str());
				}
				else
				{
					memset(config.trk_status, 0, sizeof(config.trk_status));
				}
			}

			if (request->argName(i) == "trackerPos2RF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2RF = true;
				}
			}
			if (request->argName(i) == "trackerPos2INET")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2INET = true;
				}
			}
			if (request->argName(i) == "trackerTimeStamp")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						timeStamp = true;
				}
			}
			if (request->argName(i) == "trkTlmInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_tlm_interval = request->arg(i).toInt();
				}
			}
			String arg;
			for (int x = 0; x < 5; x++)
			{
				arg = "sensorCH" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.trk_tlm_sensor[x] = request->arg(i).toInt();
				}
				arg = "param" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.trk_tlm_PARM[x], request->arg(i).c_str());
					}
				}
				arg = "unit" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.trk_tlm_UNIT[x], request->arg(i).c_str());
					}
				}
				arg = "precision" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.trk_tlm_precision[x] = request->arg(i).toInt();
				}
				arg = "offset" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.trk_tlm_offset[x] = request->arg(i).toFloat();
				}
				for (int y = 0; y < 3; y++)
				{
					arg = "eqns" + String(x) + String((char)(y + 'a'));
					if (request->argName(i) == arg)
					{
						if (isValidNumber(request->arg(i)))
							config.trk_tlm_EQNS[x][y] = request->arg(i).toFloat();
					}
				}
			}
		}
		config.trk_en = trakerEn;
		config.trk_smartbeacon = smartEn;
		config.trk_compress = compEn;

		config.trk_gps = posGPS;
		config.trk_loc2rf = pos2RF;
		config.trk_loc2inet = pos2INET;

		config.trk_log = optCST;
		config.trk_altitude = optAlt;
		config.trk_rssi = optBat;
		config.trk_sat = optSat;
		config.trk_timestamp = timeStamp;

		initInterval = true;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}

	String html = "<script type=\"text/javascript\">\n";
	html += "$('form').submit(function (e) {\n";
	html += "e.preventDefault();\n";
	html += "var data = new FormData(e.currentTarget);\n";
	html += "document.getElementById(\"submitTRACKER\").disabled=true;\n";
	html += "$.ajax({\n";
	html += "url: '/tracker',\n";
	html += "type: 'POST',\n";
	html += "data: data,\n";
	html += "contentType: false,\n";
	html += "processData: false,\n";
	html += "success: function (data) {\n";
	html += "alert(\"Submited Successfully\");\n";
	html += "},\n";
	html += "error: function (data) {\n";
	html += "alert(\"An error occurred.\");\n";
	html += "}\n";
	html += "});\n";
	html += "});\n";
	html += "</script>\n<script type=\"text/javascript\">\n";
	html += "function openWindowSymbol(sel) {\n";
	html += "var i, l, options = [{\n";
	html += "value: 'first',\n";
	html += "text: 'First'\n";
	html += "}, {\n";
	html += "value: 'second',\n";
	html += "text: 'Second'\n";
	html += "}],\n";
	html += "newWindow = window.open(\"/symbol?sel=\"+sel.toString(), null, \"height=400,width=400,status=no,toolbar=no,menubar=no,location=no\");\n";
	html += "}\n";

	html += "function setValue(sel,symbol,table) {\n";
	html += "var txtsymbol=document.getElementById('trackerSymbol');\n";
	html += "var txttable=document.getElementById('trackerTable');\n";
	html += "var imgicon=document.getElementById('trackerImgSymbol');\n";
	html += "if(sel==1){\n";
	html += "txtsymbol=document.getElementById('moveSymbol');\n";
	html += "txttable=document.getElementById('moveTable');\n";
	html += "imgicon= document.getElementById('moveImgSymbol');\n";
	html += "}else if(sel==2){\n";
	html += "txtsymbol=document.getElementById('stopSymbol');\n";
	html += "txttable=document.getElementById('stopTable');\n";
	html += "imgicon= document.getElementById('stopImgSymbol');\n";
	html += "}\n";
	html += "txtsymbol.value = String.fromCharCode(symbol);\n";
	html += "if(table==1){\n txttable.value='/';\n";
	html += "}else if(table==2){\n txttable.value='\\\\';\n}\n";
	html += "imgicon.src = \"http://aprs.dprns.com/symbols/icons/\"+symbol.toString()+'-'+table.toString()+'.png';\n";
	html += "\n}\n";
	html += "function onSmartCheck() {\n";
	html += "if (document.querySelector('#smartBcnEnable').checked) {\n";
	// Checkbox has been checked
	html += "document.getElementById(\"smartbcnGrp\").disabled=false;\n";
	html += "} else {\n";
	// Checkbox has been unchecked
	html += "document.getElementById(\"smartbcnGrp\").disabled=true;\n";
	html += "}\n}\n";

	html += "function selPrecision(idx) {\n";
	html += "var x=0;\n";
	html += "x = document.getElementsByName(\"precision\"+idx)[0].value;\n";
	html += "document.getElementsByName(\"eqns\"+idx+\"b\")[0].value=1/Math.pow(10,x);\n";
	html += "}\n";
	html += "function selOffset(idx) {\n";
	html += "var x=0;\n";
	html += "x = document.getElementsByName(\"offset\"+idx)[0].value;\n";
	html += "document.getElementsByName(\"eqns\"+idx+\"c\")[0].value=x*(-1);\n";
	html += "}\n";
	html += "</script>\n";

	delay(1);
	/************************ tracker Mode **************************/
	html += "<form id='formtracker' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
	// html += "<h2>[TRACKER] Tracker Position Mode</h2>\n";
	html += "<table>\n";
	// html += "<tr>\n";
	// html += "<th width=\"200\"><span><b>Setting</b></span></th>\n";
	// html += "<th><span><b>Value</b></span></th>\n";
	// html += "</tr>\n";
	html += "<th colspan=\"2\"><span><b>[TRACKER] Tracker Position Mode</b></span></th>\n";
	html += "<tr>\n";
	html += "<td align=\"right\"><b>Enable:</b></td>\n";
	String trackerEnFlag = "";
	if (config.trk_en)
		trackerEnFlag = "checked";
	html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"trackerEnable\" value=\"OK\" " + trackerEnFlag + "><span class=\"slider round\"></span></label></td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td align=\"right\"><b>Station Callsign:</b></td>\n";
	html += "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"" + String(config.trk_mycall) + "\" /></td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td align=\"right\"><b>Station SSID:</b></td>\n";
	html += "<td style=\"text-align: left;\">\n";
	html += "<select name=\"mySSID\" id=\"mySSID\">\n";
	for (uint8_t ssid = 0; ssid <= 15; ssid++)
	{
		if (config.trk_ssid == ssid)
		{
			html += "<option value=\"" + String(ssid) + "\" selected>" + String(ssid) + "</option>\n";
		}
		else
		{
			html += "<option value=\"" + String(ssid) + "\">" + String(ssid) + "</option>\n";
		}
	}
	html += "</select></td>\n";
	html += "</tr>\n";

	html += "<tr>\n";
	html += "<td align=\"right\"><b>Item/Obj Name:</b></td>\n";
	html += "<td style=\"text-align: left;\"><input maxlength=\"9\" size=\"9\" id=\"trackerObject\" name=\"trackerObject\" type=\"text\" value=\"" + String(config.trk_item) + "\" /><i> *If not used, leave it blank.In use 3-9 charactor</i></td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td align=\"right\"><b>PATH:</b></td>\n";
	html += "<td style=\"text-align: left;\">\n";
	html += "<select name=\"trackerPath\" id=\"trackerPath\">\n";
	for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
	{
		if (config.trk_path == pthIdx)
		{
			html += "<option value=\"" + String(pthIdx) + "\" selected>" + String(PATH_NAME[pthIdx]) + "</option>\n";
		}
		else
		{
			html += "<option value=\"" + String(pthIdx) + "\">" + String(PATH_NAME[pthIdx]) + "</option>\n";
		}
	}
	html += "</select></td>\n";
	// html += "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" id=\"trackerPath\" name=\"trackerPath\" type=\"text\" value=\"" + String(config.trk_path) + "\" /></td>\n";
	html += "</tr>\n";

	html += "<tr>\n";
	html += "<td align=\"right\"><b>Text Comment:</b></td>\n";
	html += "<td style=\"text-align: left;\"><input maxlength=\"25\" size=\"30\" id=\"trackerComment\" name=\"trackerComment\" type=\"text\" value=\"" + String(config.trk_comment) + "\" /></td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
		html += "<td align=\"right\"><b>Text Status:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"60\" id=\"trkStatus\" name=\"trkStatus\" type=\"text\" value=\"" + String(config.trk_status) + "\" />  Interval:<input min=\"0\" max=\"3600\" step=\"1\" name=\"trkSTSInv\" type=\"number\" value=\"" + String(config.trk_sts_interval) + "\" />Sec.</td>\n";
		html += "</tr>\n";
	html += "<tr>\n";
	html += "<td align=\"right\"><b>Smart Beacon:</b></td>\n";
	String smartBcnEnFlag = "";
	if (config.trk_smartbeacon)
		smartBcnEnFlag = "checked";
	html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"smartBcnEnable\" name=\"smartBcnEnable\" onclick=\"onSmartCheck()\" value=\"OK\" " + smartBcnEnFlag + "><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch use to smart beacon mode</i></label></td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td align=\"right\"><b>Compress:</b></td>\n";
	String compressEnFlag = "";
	if (config.trk_compress)
		compressEnFlag = "checked";
	html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"compressEnable\" value=\"OK\" " + compressEnFlag + "><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch compress packet</i></label></td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td align=\"right\"><b>Mic-E Type:</b></td>\n";
	html += "<td style=\"text-align: left;\">\n";
	html += "<select name=\"trkMicEType\" id=\"trkMicEType\">\n";
	for (uint8_t micEIdx = 0; micEIdx < 8; micEIdx++)
	{
		if (config.trk_mice_type == micEIdx)
		{
			html += "<option value=\"" + String(micEIdx) + "\" selected>" + String(MIC_E_MSG[micEIdx]) + "</option>\n";
		}
		else
		{
			html += "<option value=\"" + String(micEIdx) + "\">" + String(MIC_E_MSG[micEIdx]) + "</option>\n";
		}
	}
	html += "</select><label style=\"vertical-align: bottom;font-size: 8pt;\"><i>*Support if Compress is enabled and not use Item/Obj,Time Stamp</i></label></td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td align=\"right\"><b>Time Stamp:</b></td>\n";
	String timeStampFlag = "";
	if (config.trk_timestamp)
		timeStampFlag = "checked";
	html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"trackerTimeStamp\" value=\"OK\" " + timeStampFlag + "><span class=\"slider round\"></span></label></td>\n";
	html += "</tr>\n";
	String trackerPos2RFFlag = "";
	String trackerPos2INETFlag = "";
	if (config.trk_loc2rf)
		trackerPos2RFFlag = "checked";
	if (config.trk_loc2inet)
		trackerPos2INETFlag = "checked";
	html += "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"trackerPos2RF\" value=\"OK\" " + trackerPos2RFFlag + "/>RF <input type=\"checkbox\" name=\"trackerPos2INET\" value=\"OK\" " + trackerPos2INETFlag + "/>Internet </td></tr>\n";
	String trackerOptBatFlag = "";
	String trackerOptSatFlag = "";
	String trackerOptAltFlag = "";
	String trackerOptCSTFlag = "";
	if (config.trk_rssi)
		trackerOptBatFlag = "checked";
	if (config.trk_sat)
		trackerOptSatFlag = "checked";
	if (config.trk_altitude)
		trackerOptAltFlag = "checked";
	if (config.trk_log)
		trackerOptCSTFlag = "checked";
	html += "<tr><td style=\"text-align: right;\"><b>Option:</b></td><td style=\"text-align: left;\">";
	html += "<input type=\"checkbox\" name=\"trackerOptCST\" value=\"OK\" " + trackerOptCSTFlag + "/>Telemetry ";
	html += "<input type=\"checkbox\" name=\"trackerOptAlt\" value=\"OK\" " + trackerOptAltFlag + "/>Altutude ";
	html += "<input type=\"checkbox\" name=\"trackerOptBat\" value=\"OK\" " + trackerOptBatFlag + "/>RSSI Request ";
	// html += "<input type=\"checkbox\" name=\"trackerOptSat\" value=\"OK\" " + trackerOptSatFlag + "/>Satellite";
	html += "</td></tr>\n";

	html += "<tr>";
	html += "<td align=\"right\"><b>POSITION:</b></td>\n";
	html += "<td align=\"center\">\n";
	html += "<table>";
	html += "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" id=\"trackerPosInv\" name=\"trackerPosInv\" type=\"number\" value=\"" + String(config.trk_interval) + "\" />Sec.</label></td></tr>";
	String trackerPosFixFlag = "";
	String trackerPosGPSFlag = "";

	if (config.trk_gps)
		trackerPosGPSFlag = "checked=\"checked\"";
	else
		trackerPosFixFlag = "checked=\"checked\"";

	html += "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"trackerPosSel\" value=\"0\" " + trackerPosFixFlag + "/>Fix <input type=\"radio\" name=\"trackerPosSel\" value=\"1\" " + trackerPosGPSFlag + "/>GPS </td></tr>\n";
	html += "<tr>\n";
	html += "<td align=\"right\">Symbol Icon:</td>\n";
	String table = "1";
	if (config.trk_symbol[0] == 47)
		table = "1";
	if (config.trk_symbol[0] == 92)
		table = "2";
	html += "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"trackerTable\" name=\"trackerTable\" type=\"text\" value=\"" + String(config.trk_symbol[0]) + "\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"trackerSymbol\" name=\"trackerSymbol\" type=\"text\" value=\"" + String(config.trk_symbol[1]) + "\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"trackerImgSymbol\" onclick=\"openWindowSymbol(0);\" src=\"http://aprs.dprns.com/symbols/icons/" + String((int)config.trk_symbol[1]) + "-" + table + ".png\"> <i>*Click icon for select symbol</i></td>\n";
	html += "</tr>\n";
	html += "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" id=\"trackerPosLat\" name=\"trackerPosLat\" type=\"number\" value=\"" + String(config.trk_lat, 5) + "\" />degrees (positive for North, negative for South)</td></tr>\n";
	html += "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" id=\"trackerPosLon\" name=\"trackerPosLon\" type=\"number\" value=\"" + String(config.trk_lon, 5) + "\" />degrees (positive for East, negative for West)</td></tr>\n";
	html += "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" id=\"trackerPosAlt\" name=\"trackerPosAlt\" type=\"number\" value=\"" + String(config.trk_alt, 2) + "\" /> meter. *Value 0 is not send height</td></tr>\n";
	html += "</table></td>";
	html += "</tr>\n";

	html += "<tr>\n";
	html += "<td align=\"right\"><b>Smart Beacon:</b></td>\n";
	html += "<td align=\"center\">\n";
	if (config.trk_smartbeacon)
		html += "<fieldset id=\"smartbcnGrp\">\n";
	else
		html += "<fieldset id=\"smartbcnGrp\" disabled>\n";
	html += "<legend>Smart beacon configuration</legend>\n<table>";
	html += "<tr>\n";
	html += "<td align=\"right\">Move Symbol:</td>\n";
	table = "1";
	if (config.trk_symmove[0] == 47)
		table = "1";
	if (config.trk_symmove[0] == 92)
		table = "2";
	html += "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"moveTable\" name=\"moveTable\" type=\"text\" value=\"" + String(config.trk_symmove[0]) + "\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"moveSymbol\" name=\"moveSymbol\" type=\"text\" value=\"" + String(config.trk_symmove[1]) + "\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"moveImgSymbol\" onclick=\"openWindowSymbol(1);\" src=\"http://aprs.dprns.com/symbols/icons/" + String((int)config.trk_symmove[1]) + "-" + table + ".png\"> <i>*Click icon for select MOVE symbol</i></td>\n";
	html += "</tr>\n";
	html += "<tr>\n";
	html += "<td align=\"right\">Stop Symbol:</td>\n";
	table = "1";
	if (config.trk_symstop[0] == 47)
		table = "1";
	if (config.trk_symstop[0] == 92)
		table = "2";
	html += "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"stopTable\" name=\"stopTable\" type=\"text\" value=\"" + String(config.trk_symstop[0]) + "\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"stopSymbol\" name=\"stopSymbol\" type=\"text\" value=\"" + String(config.trk_symstop[1]) + "\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"stopImgSymbol\" onclick=\"openWindowSymbol(2);\" src=\"http://aprs.dprns.com/symbols/icons/" + String((int)config.trk_symstop[1]) + "-" + table + ".png\"> <i>*Click icon for select STOP symbol</i></td>\n";
	html += "</tr>\n";
	html += "<tr><td style=\"text-align: right;\">High Speed:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"10\" max=\"1000\" step=\"1\" id=\"hspeed\" name=\"hspeed\" type=\"number\" value=\"" + String(config.trk_hspeed) + "\" /> km/h</td></tr>\n";
	html += "<tr><td style=\"text-align: right;\">Low Speed:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"1\" max=\"250\" step=\"1\" id=\"lspeed\" name=\"lspeed\" type=\"number\" value=\"" + String(config.trk_lspeed) + "\" /> km/h</td></tr>\n";
	html += "<tr><td style=\"text-align: right;\">Slow Interval:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"60\" max=\"3600\" step=\"1\" id=\"slowInterval\" name=\"slowInterval\" type=\"number\" value=\"" + String(config.trk_slowinterval) + "\" /> Sec.</td></tr>\n";
	html += "<tr><td style=\"text-align: right;\">Max Interval:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"10\" max=\"255\" step=\"1\" id=\"maxInterval\" name=\"maxInterval\" type=\"number\" value=\"" + String(config.trk_maxinterval) + "\" /> Sec.</td></tr>\n";
	html += "<tr><td style=\"text-align: right;\">Min Interval:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"1\" max=\"100\" step=\"1\" id=\"minInterval\" name=\"minInterval\" type=\"number\" value=\"" + String(config.trk_mininterval) + "\" /> Sec.</td></tr>\n";
	html += "<tr><td style=\"text-align: right;\">Min Angle:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"1\" max=\"359\" step=\"1\" id=\"minAngle\" name=\"minAngle\" type=\"number\" value=\"" + String(config.trk_minangle) + "\" /> Degree.</td></tr>\n";

	html += "</table></fieldset></tr>";

	html += "<tr>\n";
	html += "<td align=\"right\"><b>Telemetry:</b><br />(v=0->8280)</td>\n";
	html += "<td align=\"center\"><table>\n";
	html += "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"1000\" step=\"1\" id=\"trkTlmInv\" name=\"trkTlmInv\" type=\"number\" value=\"" + String(config.trk_tlm_interval) + "\" /> *Number of packets interval,<i>Example: 0 not send,1 send every packet</i></label></td></tr>";
	for (int ax = 0; ax < 5; ax++)
	{
		html += "<tr><td align=\"right\"><b>CH A" + String(ax + 1) + ":</b></td>\n";
		html += "<td align=\"center\">\n";
		html += "<table>";

		html += "<tr><td style=\"text-align: right;\">Sensor:</td>\n";
		html += "<td style=\"text-align: left;\">CH: ";
		html += "<select name=\"sensorCH" + String(ax) + "\" id=\"sensorCH" + String(ax) + "\">\n";
		for (uint8_t idx = 0; idx < 11; idx++)
		{
			if (idx == 0)
			{
				if (config.trk_tlm_sensor[ax] == idx)
				{
					html += "<option value=\"" + String(idx) + "\" selected>NONE</option>\n";
				}
				else
				{
					html += "<option value=\"" + String(idx) + "\">NONE</option>\n";
				}
			}
			else
			{
				if (config.trk_tlm_sensor[ax] == idx)
				{
					html += "<option value=\"" + String(idx) + "\" selected>SENSOR#" + String(idx) + "</option>\n";
				}
				else
				{
					html += "<option value=\"" + String(idx) + "\">SENSOR#" + String(idx) + "</option>\n";
				}
			}
		}
		html += "</select></td>\n";

		html += "<td style=\"text-align: left;\">Name: <input maxlength=\"10\" size=\"8\" name=\"param" + String(ax) + "\" type=\"text\" value=\"" + String(config.trk_tlm_PARM[ax]) + "\" /></td>\n";
		html += "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit" + String(ax) + "\" type=\"text\" value=\"" + String(config.trk_tlm_UNIT[ax]) + "\" /></td>\n";
		html += "<td style=\"text-align: left;\">Precision: <input min=\"0\" max=\"5\" step=\"1\" type=\"number\" style=\"width: 2em\" name=\"precision" + String(ax) + "\" type=\"text\" value=\"" + String(config.trk_tlm_precision[ax]) + "\" onchange=\"selPrecision(" + String(ax) + ")\" /></td></tr>\n";

		html += "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns" + String(ax) + "a\" type=\"number\" value=\"" + String(config.trk_tlm_EQNS[ax][0], 5) + "\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns" + String(ax) + "b\" type=\"number\" value=\"" + String(config.trk_tlm_EQNS[ax][1], 5) + "\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns" + String(ax) + "c\" type=\"number\" value=\"" + String(config.trk_tlm_EQNS[ax][2], 5) + "\" /> (av<sup>2</sup>+bv+c) </td>\n";
		html += "<td style=\"text-align: left;\">Offset: <input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" type=\"number\" name=\"offset" + String(ax) + "\" type=\"text\" value=\"" + String(config.trk_tlm_offset[ax], 5) + "\"  onchange=\"selOffset(" + String(ax) + ")\" /></td></tr>\n";
		html += "</table></td>";
		html += "</tr>\n";
	}
	html += "</table></td></tr>\n";
	html += "<tr><td colspan=\"2\" align=\"right\">\n";
	html += "<div><button class=\"button\" type='submit' id='submitTRACKER'  name=\"commitTRACKER\"> Apply Change </button></div>\n";
	html += "<input type=\"hidden\" name=\"commitTRACKER\"/>\n";
	html += "</td></tr></table><br />\n";
	html += "</form><br />";
	if ((ESP.getFreeHeap() / 1000) > 120)
		{
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			size_t len = html.length();
			char *info = (char *)calloc(len, sizeof(char));
			if (info)
			{

				html.toCharArray(info, len, 0);
				html.clear();
				AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)info, len);

				response->addHeader("Sensor", "content");
				request->send(response);
				free(info);
			}
			else
			{
				log_d("Can't define calloc info size %d", len);
			}
		}
	//request->send(200, "text/html", html); // send to someones browser when asked
}

void handle_wireless(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	if (request->hasArg("commitWiFiAP"))
	{
		bool wifiAP = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "wifiAP")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						wifiAP = true;
					}
				}
			}

			if (request->argName(i) == "wifi_ssidAP")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wifi_ap_ssid, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "wifi_passAP")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wifi_ap_pass, request->arg(i).c_str());
				}
			}
		}
		if (wifiAP)
		{
			config.wifi_mode |= WIFI_AP_FIX;
		}
		else
		{
			config.wifi_mode &= ~WIFI_AP_FIX;
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else if (request->hasArg("commitWiFiClient"))
	{
		bool wifiSTA = false;
		String nameSSID, namePASS;
		for (int n = 0; n < 5; n++)
			config.wifi_sta[n].enable = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "wificlient")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						wifiSTA = true;
					}
				}
			}

			for (int n = 0; n < 5; n++)
			{
				nameSSID = "wifiStation" + String(n);
				if (request->argName(i) == nameSSID)
				{
					if (request->arg(i) != "")
					{
						if (String(request->arg(i)) == "OK")
						{
							config.wifi_sta[n].enable = true;
						}
					}
				}
				nameSSID = "wifi_ssid" + String(n);
				if (request->argName(i) == nameSSID)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.wifi_sta[n].wifi_ssid, request->arg(i).c_str());
					}
				}
				namePASS = "wifi_pass" + String(n);
				if (request->argName(i) == namePASS)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.wifi_sta[n].wifi_pass, request->arg(i).c_str());
					}
				}
			}

			if (request->argName(i) == "wifi_pwr")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wifi_power = (int8_t)request->arg(i).toInt();
				}
			}
		}
		if (wifiSTA)
		{
			config.wifi_mode |= WIFI_STA_FIX;
		}
		else
		{
			config.wifi_mode &= ~WIFI_STA_FIX;
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
		WiFi.setTxPower((wifi_power_t)config.wifi_power);
	}
	else if (request->hasArg("commitBluetooth"))
	{
		bool btMaster = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "btMaster")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						btMaster = true;
					}
				}
			}

			if (request->argName(i) == "bt_name")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.bt_name, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "bt_uuid")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.bt_uuid, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "bt_uuid_rx")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.bt_uuid_rx, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "bt_uuid_tx")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.bt_uuid_tx, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "bt_mode")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.bt_mode = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "bt_pin")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.bt_pin = request->arg(i).toInt();
				}
			}
		}
		config.bt_master = btMaster;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else
	{
		String html = "<script type=\"text/javascript\">\n";
		html += "$('form').submit(function (e) {\n";
		html += "e.preventDefault();\n";
		html += "var data = new FormData(e.currentTarget);\n";
		html += "if(e.currentTarget.id===\"formBluetooth\") document.getElementById(\"submitBluetooth\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formWiFiAP\") document.getElementById(\"submitWiFiAP\").disabled=true;\n";
		html += "if(e.currentTarget.id===\"formWiFiClient\") document.getElementById(\"submitWiFiClient\").disabled=true;\n";
		html += "$.ajax({\n";
		html += "url: '/wireless',\n";
		html += "type: 'POST',\n";
		html += "data: data,\n";
		html += "contentType: false,\n";
		html += "processData: false,\n";
		html += "success: function (data) {\n";
		html += "alert(\"Submited Successfully\");\n";
		html += "},\n";
		html += "error: function (data) {\n";
		html += "alert(\"An error occurred.\");\n";
		html += "}\n";
		html += "});\n";
		html += "});\n";
		html += "</script>\n";
		/************************ WiFi AP **************************/
		html += "<form id='formWiFiAP' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		// html += "<h2>WiFi Access Point</h2>\n";
		html += "<table>\n";
		// html += "<tr>\n";
		// html += "<th width=\"200\"><span><b>Setting</b></span></th>\n";
		// html += "<th><span><b>Value</b></span></th>\n";
		// html += "</tr>\n";
		html += "<th colspan=\"2\"><span><b>WiFi Access Point</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\" width=\"120\"><b>Enable:</b></td>\n";
		String wifiAPEnFlag = "";
		if (config.wifi_mode & WIFI_AP_FIX)
			wifiAPEnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wifiAP\" value=\"OK\" " + wifiAPEnFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>WiFi AP SSID:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"32\" maxlength=\"32\" class=\"form-control\" id=\"wifi_ssidAP\" name=\"wifi_ssidAP\" type=\"text\" value=\"" + String(config.wifi_ap_ssid) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>WiFi AP PASSWORD:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input size=\"63\" maxlength=\"63\" class=\"form-control\" id=\"wifi_passAP\" name=\"wifi_passAP\" type=\"password\" value=\"" + String(config.wifi_ap_pass) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitWiFiAP'  name=\"commitWiFiAP\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitWiFiAP\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form><br />";
		/************************ WiFi Client **************************/
		html += "<br />\n";
		html += "<form id='formWiFiClient' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		html += "<table>\n";
		html += "<th colspan=\"2\"><span><b>WiFi Multi Station</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>WiFi STA Enable:</b></td>\n";
		String wifiClientEnFlag = "";
		if (config.wifi_mode & WIFI_STA_FIX)
			wifiClientEnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wificlient\" value=\"OK\" " + wifiClientEnFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>WiFi RF Power:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"wifi_pwr\" id=\"wifi_pwr\">\n";
		for (int i = 0; i < 12; i++)
		{
			if (config.wifi_power == (int8_t)wifiPwr[i][0])
				html += "<option value=\"" + String((int8_t)wifiPwr[i][0]) + "\" selected>" + String(wifiPwr[i][1], 1) + " dBm</option>\n";
			else
				html += "<option value=\"" + String((int8_t)wifiPwr[i][0]) + "\" >" + String(wifiPwr[i][1], 1) + " dBm</option>\n";
		}
		html += "</select>\n";
		html += "</td>\n";
		html += "</tr>\n";
		for (int n = 0; n < 5; n++)
		{
			html += "<tr>\n";
			html += "<td align=\"right\"><b>Station #" + String(n + 1) + ":</b></td>\n";
			html += "<td align=\"center\">\n";
			html += "<fieldset id=\"filterDispGrp" + String(n + 1) + "\">\n";
			html += "<legend>WiFi Station #" + String(n + 1) + "</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">";
			html += "<tr style=\"background:unset;\">";
			// html += "<tr>\n";
			html += "<td align=\"right\" width=\"120\"><b>Enable:</b></td>\n";
			String wifiClientEnFlag = "";
			if (config.wifi_sta[n].enable)
				wifiClientEnFlag = "checked";
			html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wifiStation" + String(n) + "\" value=\"OK\" " + wifiClientEnFlag + "><span class=\"slider round\"></span></label></td>\n";
			html += "</tr>\n";
			html += "<tr>\n";
			html += "<td align=\"right\"><b>WiFi SSID:</b></td>\n";
			html += "<td style=\"text-align: left;\"><input size=\"32\" maxlength=\"32\" name=\"wifi_ssid" + String(n) + "\" type=\"text\" value=\"" + String(config.wifi_sta[n].wifi_ssid) + "\" /></td>\n";
			html += "</tr>\n";
			html += "<tr>\n";
			html += "<td align=\"right\"><b>WiFi PASSWORD:</b></td>\n";
			html += "<td style=\"text-align: left;\"><input size=\"63\" maxlength=\"63\" name=\"wifi_pass" + String(n) + "\" type=\"password\" value=\"" + String(config.wifi_sta[n].wifi_pass) + "\" /></td>\n";
			html += "</tr>\n";
			html += "</tr></table></fieldset>\n";
			html += "</td></tr>\n";
		}
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitWiFiClient'  name=\"commitWiFiClient\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitWiFiClient\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form><br />";
		/************************ Bluetooth **************************/
#ifdef BLUETOOTH
		html += "<br />\n";
		html += "<form id='formBluetooth' method=\"POST\" action='#' enctype='multipart/form-data'>\n";
		// html += "<h2>Bluetooth Master (BLE)</h2>\n";
		html += "<table>\n";
		// html += "<tr>\n";
		// html += "<th width=\"200\"><span><b>Setting</b></span></th>\n";
		// html += "<th><span><b>Value</b></span></th>\n";
		// html += "</tr>\n";
		html += "<th colspan=\"2\"><span><b>Bluetooth Master (BLE)</b></span></th>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>Enable:</b></td>\n";
		String btEnFlag = "";
		if (config.bt_master)
			btEnFlag = "checked";
		html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"btMaster\" value=\"OK\" " + btEnFlag + "><span class=\"slider round\"></span></label></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>NAME:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"20\" id=\"bt_name\" name=\"bt_name\" type=\"text\" value=\"" + String(config.bt_name) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>PIN:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input min=\"0\" max=\"999999\" id=\"bt_pin\" name=\"bt_pin\" type=\"number\" value=\"" + String(config.bt_pin,DEC) + "\" /></td> <i>*Value 0 is no auth.</i>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>UUID:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"37\" size=\"38\" id=\"bt_uuid\" name=\"bt_uuid\" type=\"text\" value=\"" + String(config.bt_uuid) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>UUID RX:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"37\" size=\"38\" id=\"bt_uuid_rx\" name=\"bt_uuid_rx\" type=\"text\" value=\"" + String(config.bt_uuid_rx) + "\" /></td>\n";
		html += "</tr>\n";
		html += "<tr>\n";
		html += "<td align=\"right\"><b>UUID TX:</b></td>\n";
		html += "<td style=\"text-align: left;\"><input maxlength=\"37\" size=\"38\" id=\"bt_uuid_tx\" name=\"bt_uuid_tx\" type=\"text\" value=\"" + String(config.bt_uuid_tx) + "\" /></td>\n";
		html += "</tr>\n";

		html += "<td align=\"right\"><b>MODE:</b></td>\n";
		html += "<td style=\"text-align: left;\">\n";
		html += "<select name=\"bt_mode\" id=\"bt_mode\">\n";
		String btModeOff = "";
		String btModeTNC2 = "";
		String btModeKISS = "";
		if (config.bt_mode == 1)
		{
			btModeTNC2 = "selected";
		}
		else if (config.bt_mode == 2)
		{
			btModeKISS = "selected";
		}
		else
		{
			btModeOff = "selected";
		}
		html += "<option value=\"0\" " + btModeOff + ">NONE</option>\n";
		html += "<option value=\"1\" " + btModeTNC2 + ">TNC2</option>\n";
		html += "<option value=\"2\" " + btModeKISS + ">KISS</option>\n";
		html += "</select>\n";

		html += "<label style=\"font-size: 8pt;text-align: right;\">*See the following for generating UUIDs: <a href=\"https://www.uuidgenerator.net\" target=\"_blank\">https://www.uuidgenerator.net</a></label></td>\n";
		html += "</tr>\n";
		html += "<tr><td colspan=\"2\" align=\"right\">\n";
		html += "<div><button class=\"button\" type='submit' id='submitBluetooth'  name=\"commitBluetooth\"> Apply Change </button></div>\n";
		html += "<input type=\"hidden\" name=\"commitBluetooth\"/>\n";
		html += "</td></tr></table><br />\n";
		html += "</form>";
#endif
		request->send(200, "text/html", html); // send to someones browser when asked
	}
}

//extern String lastPkgRaw;
//extern float dBV;
//extern int mVrms;
// void handle_realtime(AsyncWebServerRequest *request)
// {
// 	// char jsonMsg[1000];
// 	char *jsonMsg;
// 	time_t timeStamp;
// 	time(&timeStamp);

// 	if (afskSync && (lastPkgRaw.length() > 5))
// 	{
// 		int input_length = lastPkgRaw.length();
// 		jsonMsg = (char *)malloc((input_length * 2) + 200);
// 		char *input_buffer = (char *)malloc(input_length + 2);
// 		char *output_buffer = (char *)malloc(input_length * 2);
// 		if (output_buffer)
// 		{
// 			// lastPkgRaw.toCharArray(input_buffer, lastPkgRaw.length(), 0);
// 			memcpy(input_buffer, lastPkgRaw.c_str(), lastPkgRaw.length());
// 			lastPkgRaw.clear();
// 			encode_base64((unsigned char *)input_buffer, input_length, (unsigned char *)output_buffer);
// 			// Serial.println(output_buffer);
// 			sprintf(jsonMsg, "{\"Active\":\"1\",\"mVrms\":\"%d\",\"RAW\":\"%s\",\"timeStamp\":\"%li\"}", mVrms, output_buffer, timeStamp);
// 			// Serial.println(jsonMsg);
// 			free(input_buffer);
// 			free(output_buffer);
// 		}
// 	}
// 	else
// 	{
// 		jsonMsg = (char *)malloc(100);
// 		if (afskSync)
// 			sprintf(jsonMsg, "{\"Active\":\"1\",\"mVrms\":\"%d\",\"RAW\":\"REVDT0RFIEZBSUwh\",\"timeStamp\":\"%li\"}", mVrms, timeStamp);
// 		else
// 			sprintf(jsonMsg, "{\"Active\":\"0\",\"mVrms\":\"0\",\"RAW\":\"\",\"timeStamp\":\"%li\"}", timeStamp);
// 	}
// 	afskSync = false;
// 	request->send(200, "text/html", String(jsonMsg));

// 	delay(100);
// 	free(jsonMsg);
// }

//void handle_ws(String Raw,uint16_t mVrms)
void handle_ws(char *Raw,size_t len,uint16_t mVrms)
{
	if (ws.count() < 1)
	return;

	char *jsonMsg;
	time_t timeStamp;
	time(&timeStamp);

	if (len > 5)
	{
		int input_length = len;
		jsonMsg = (char *)calloc((input_length * 2) + 200, sizeof(char));
		if (jsonMsg)
		{
			char *input_buffer = (char *)calloc(input_length + 2, sizeof(char));
			char *output_buffer = (char *)calloc(input_length * 2, sizeof(char));
			if (output_buffer)
			{
				memset(input_buffer, 0, (input_length + 2));
				memset(output_buffer, 0, (input_length * 2));
				// lastPkgRaw.toCharArray(input_buffer, input_length, 0);
				memcpy(input_buffer, Raw, len);
				encode_base64((unsigned char *)input_buffer, input_length, (unsigned char *)output_buffer);
				// Serial.println(output_buffer);
				sprintf(jsonMsg, "{\"Active\":\"1\",\"mVrms\":\"%d\",\"RAW\":\"%s\",\"timeStamp\":\"%li\"}", mVrms, output_buffer, timeStamp);
				// Serial.println(jsonMsg);
				free(input_buffer);
				free(output_buffer);
			}
			ws.textAll(jsonMsg);
			free(jsonMsg);
		}
	}
	else
	{
		jsonMsg = (char *)calloc(300, sizeof(char));
		if (jsonMsg)
		{
			if (mVrms>0)
				sprintf(jsonMsg, "{\"Active\":\"1\",\"mVrms\":\"%d\",\"RAW\":\"REVDT0RFIEZBSUwh\",\"timeStamp\":\"%li\"}", mVrms, timeStamp);
			else
				sprintf(jsonMsg, "{\"Active\":\"0\",\"mVrms\":\"0\",\"RAW\":\"\",\"timeStamp\":\"%li\"}", timeStamp);
			ws.textAll(jsonMsg);
			free(jsonMsg);
		}
	}
}

void handle_ws_gnss(char *nmea, size_t size)
{
	if(ws_gnss.count() < 1)
		return;

	time_t timeStamp;
	time(&timeStamp);
	unsigned int output_length = encode_base64_length(size);
	unsigned char nmea_enc[output_length];
	char jsonMsg[output_length + 100];
	encode_base64((unsigned char *)nmea, size, (unsigned char *)nmea_enc);
	sprintf(jsonMsg, "{\"en\":\"%d\",\"lat\":\"%.5f\",\"lng\":\"%.5f\",\"alt\":\"%.2f\",\"spd\":\"%.2f\",\"csd\":\"%.1f\",\"hdop\":\"%.2f\",\"sat\":\"%d\",\"time\":\"%d\",\"timeStamp\":\"%li\",\"RAW\":\"", (int)config.gnss_enable, gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.speed.kmph(), gps.course.deg(), gps.hdop.hdop(), gps.satellites.value(), gps.time.value(), timeStamp);
	strncat(jsonMsg, (const char *)nmea_enc, output_length);
	strcat(jsonMsg, "\"}");
	ws_gnss.textAll(jsonMsg);
}

void handle_test(AsyncWebServerRequest *request)
{
	// if (request->hasArg("sendBeacon"))
	// {
	// 	String tnc2Raw = send_fix_location();
	// 	if (config.rf_en)
	// 		pkgTxPush(tnc2Raw.c_str(), tnc2Raw.length(), 0);
	// 	// APRS_sendTNC2Pkt(tnc2Raw); // Send packet to RF
	// }
	// else if (request->hasArg("sendRaw"))
	// {
	// 	for (uint8_t i = 0; i < request->args(); i++)
	// 	{
	// 		if (request->argName(i) == "raw")
	// 		{
	// 			if (request->arg(i) != "")
	// 			{
	// 				String tnc2Raw = request->arg(i);
	// 				if (config.rf_en)
	// 				{
	// 					pkgTxPush(tnc2Raw.c_str(), tnc2Raw.length(), 0);
	// 					// APRS_sendTNC2Pkt(request->arg(i)); // Send packet to RF
	// 					// Serial.println("Send RAW: " + tnc2Raw);
	// 				}
	// 			}
	// 			break;
	// 		}
	// 	}
	// }
	// setHTML(6);

	webString = "<html>\n<head>\n";
	webString += "<script src=\"https://apps.bdimg.com/libs/jquery/2.1.4/jquery.min.js\"></script>\n";
	webString += "<script src=\"https://code.highcharts.com/highcharts.js\"></script>\n";
	webString += "<script src=\"https://code.highcharts.com/highcharts-more.js\"></script>\n";
	webString += "<script language=\"JavaScript\">";
	webString += "$(document).ready(function() {\nvar chart = {\ntype: 'gauge',plotBorderWidth: 1,plotBackgroundColor: {linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },stops: [[0, '#FFFFC6'],[0.3, '#FFFFFF'],[1, '#FFF4C6']]},plotBackgroundImage: null,height: 200};\n";
	webString += "var credits = {enabled: false};\n";
	webString += "var title = {text: 'RX/AUDIO VU Meter'};\n";
	webString += "var pane = [{startAngle: -45,endAngle: 45,background: null,center: ['50%', '145%'],size: 300}];\n";
	webString += "var yAxis = [{min: -40,max: 1,minorTickPosition: 'outside',tickPosition: 'outside',labels: {rotation: 'auto',distance: 20},\n";
	webString += "plotBands: [{from: -10,to: 1,color: '#C02316',innerRadius: '100%',outerRadius: '105%'},{from: -20,to: -10,color: '#00C000',innerRadius: '100%',outerRadius: '105%'},{from: -30,to: -20,color: '#AFFF0F',innerRadius: '100%',outerRadius: '105%'},{from: -40,to: -30,color: '#C0A316',innerRadius: '100%',outerRadius: '105%'}],\n";
	webString += "pane: 0,title: {text: '<span style=\"font-size:12px\">dBV</span>',y: -40}}];\n";
	webString += "var plotOptions = {gauge: {dataLabels: {enabled: false},dial: {radius: '100%'}}};\n";
	webString += "var series= [{data: [-40],yAxis: 0}];\n";
	webString += "var json = {};\n json.chart = chart;\n json.credits = credits;\n json.title = title;\n json.pane = pane;\n json.yAxis = yAxis;\n json.plotOptions = plotOptions;\n json.series = series;\n";
	// Add some life
	webString += "var chartFunction = function (chart) { \n"; // the chart may be destroyed
	webString += "var Vrms=0;\nvar dBV=-40;\nvar active=0;var raw=\"\";var timeStamp;\n";
	webString += "if (chart.series) {\n";
	webString += "var left = chart.series[0].points[0];\n";
	webString += "var host='ws://'+location.hostname+':81/ws'\n";
	webString += "const ws = new WebSocket(host);\n";
	webString += "ws.onopen = function() { console.log('Connection opened');};\n ws.onclose = function() { console.log('Connection closed');};\n";
	webString += "ws.onmessage = function(event) {\n  console.log(event.data);\n";
	webString += "const jsonR=JSON.parse(event.data);\n";
	webString += "active=parseInt(jsonR.Active);\n";
	webString += "Vrms=parseFloat(jsonR.mVrms)/1000;\n";
	webString += "dBV=20.0*Math.log10(Vrms);\n";
	webString += "if(dBV<-40) dBV=-40;\n";
	webString += "raw=jsonR.RAW;\n";
	webString += "timeStamp=Number(jsonR.timeStamp);\n";
	webString += "if(active==1){\nleft.update(dBV,false);\nchart.redraw();\n";
	webString += "var date=new Date(timeStamp * 1000);\n";
	webString += "var head=date+\"[\"+Vrms.toFixed(3)+\"Vrms,\"+dBV.toFixed(1)+\"dBV]\\n\";\n";
	// webString += "document.getElementById(\"raw_txt\").value+=head+atob(raw)+\"\\n\";\n";
	webString += "var textArea=document.getElementById(\"raw_txt\");\n";
	webString += "textArea.value+=head+atob(raw)+\"\\n\";\n";
	webString += "textArea.scrollTop = textArea.scrollHeight;\n";
	webString += "}\n";
	webString += "}\n";
	webString += "}};\n";
	webString += "$('#vumeter').highcharts(json, chartFunction);\n";
	webString += "});\n</script>\n";
	webString += "</head><body>\n<table>\n";
	// webString += "<tr><td><form accept-charset=\"UTF-8\" action=\"/test\" class=\"form-horizontal\" id=\"test_form\" method=\"post\">\n";
	// webString += "<div style=\"margin-left: 20px;\"><input type='submit' class=\"btn btn-danger\" name=\"sendBeacon\" value='SEND BEACON'></div><br />\n";
	// webString += "<div style=\"margin-left: 20px;\">TNC2 RAW: <input id=\"raw\" name=\"raw\" type=\"text\" size=\"60\" value=\"" + String(config.aprs_mycall) + ">APE32I,WIDE1-1:>Test Status\"/></div>\n";
	// webString += "<div style=\"margin-left: 20px;\"><input type='submit' class=\"btn btn-primary\" name=\"sendRaw\" value='SEND RAW'></div> <br />\n";
	// webString += "</form></td></tr>\n";
	// webString += "<tr><td><hr width=\"80%\" /></td></tr>\n";
	webString += "<tr><td><div id=\"vumeter\" style=\"width: 300px; height: 200px; margin: 10px;\"></div></td>\n";
	webString += "<tr><td><div style=\"margin: 15px;\">Terminal<br /><textarea id=\"raw_txt\" name=\"raw_txt\" rows=\"50\" cols=\"80\" /></textarea></div></td></tr>\n";
	webString += "</table>\n";

	webString += "</body></html>\n";
	request->send(200, "text/html", webString); // send to someones browser when asked

	delay(100);
	webString.clear();
}

void handle_about(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	char strCID[50];
	uint64_t chipid = ESP.getEfuseMac();
	sprintf(strCID, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);

	webString.clear();
	webString += "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;\"><td width=\"49%\" style=\"border:unset;\">";

	webString += "<table>";
	webString += "<th colspan=\"2\"><span><b>System Information</b></span></th>\n";
	// webString += "<tr><th width=\"200\"><span><b>Name</b></span></th><th><span><b>Information</b></span></th></tr>";
	webString += "<tr><td align=\"right\"><b>Hardware Version: </b></td><td align=\"left\">";
#if defined(CONFIG_IDF_TARGET_ESP32)
	webString += "ESP32-WROOM,ESP32 DoIt DevKit";
#elif defined(ESP32C3_MINI)
	webString += "ESP32C3-Mini,ESP32-C3 DIY";
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
	webString += "ESP32C3,ESP32-C3 DIY";
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
	webString += "ESP32C6,ESP32-C6 DIY";
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
	webString += "ESP32-S3-DevKit,ESP32-S3-WROOM";
#else
	webString += "UNKNOWN,ESP32 DIY";
#endif
	webString += "</td></tr>";
	webString += "<tr><td align=\"right\"><b>Firmware Version: </b></td><td align=\"left\"> V" + String(VERSION) + String(VERSION_BUILD) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>RF LoRa Chip: </b></td><td align=\"left\"> " + String(RF_TYPE[config.rf_type]) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>ESP32 Model: </b></td><td align=\"left\"> " + String(ESP.getChipModel()) + "</td></tr>";
	webString += "<tr><td align=\"right\"><b>Revision: </b></td><td align=\"left\"> " + String(ESP.getChipRevision()) + "</td></tr>";
	webString += "<tr><td align=\"right\"><b>Chip ID: </b></td><td align=\"left\"> " + String(strCID) + "</td></tr>";
	webString += "<tr><td align=\"right\"><b>Flash: </b></td><td align=\"left\">" + String(ESP.getFlashChipSize() / 1024) + " KByte</td></tr>";
	webString += "<tr><td align=\"right\"><b>PSRAM: </b></td><td align=\"left\">" + String((float)ESP.getFreePsram() / 1024, 1) + "/" + String((float)ESP.getPsramSize() / 1024, 1) + " KByte</td></tr>";
	webString += "<tr><td align=\"right\"><b>FILE SYSTEM: </b></td><td align=\"left\">" + String((float)LITTLEFS.usedBytes() / 1024, 1) + "/" + String((float)LITTLEFS.totalBytes() / 1024, 1) + " KByte</td></tr>";
	webString += "</table>";
	webString += "</td><td width=\"2%\" style=\"border:unset;\"></td>";
	webString += "<td width=\"49%\" style=\"border:unset;\">";

	webString += "<table>";
	webString += "<th colspan=\"2\"><span><b>Developer/Support Information</b></span></th>\n";
	webString += "<tr><td align=\"right\"><b>Author: </b></td><td align=\"left\">Mr.Somkiat Nakhonthai </td></tr>";
	webString += "<tr><td align=\"right\"><b>Callsign: </b></td><td align=\"left\">HS5TQA,Atten,Nakhonthai</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>Country: </b></td><td align=\"left\">Bangkok,Thailand</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>Github: </b></td><td align=\"left\"><a href=\"https://github.com/nakhonthai\" target=\"_github\">https://github.com/nakhonthai</a></td></tr>";
	webString += "<tr><td align=\"right\"><b>Youtube: </b></td><td align=\"left\"><a href=\"https://www.youtube.com/@HS5TQA\" target=\"_youtube\">https://www.youtube.com/@HS5TQA</a></td></tr>";
	webString += "<tr><td align=\"right\"><b>Facebook: </b></td><td align=\"left\"><a href=\"https://www.facebook.com/atten\" target=\"_facebook\">https://www.facebook.com/atten</a></td></tr>";
	webString += "<tr><td align=\"right\"><b>Chat: </b></td><td align=\"left\">Telegram:<a href=\"https://t.me/HS5TQA\" target=\"_line\">@HS5TQA</a> , WeChat:HS5TQA</td></tr>";
	webString += "<tr><td align=\"right\"><b>Sponsors: </b></td><td align=\"left\"><a href=\"https://github.com/sponsors/nakhonthai\" target=\"_sponsor\">https://github.com/sponsors/nakhonthai</a></td></tr>";
	webString += "<tr><td align=\"right\"><b>Donate: </b></td><td align=\"left\"><a href=\"https://www.paypal.me/0hs5tqa0\" target=\"_sponsor\">https://www.paypal.me/0hs5tqa0</a></td></tr>";

	webString += "</table>";
	webString += "</td></tr></table><br />";

	webString += "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;\"><td width=\"49%\" style=\"border:unset;\">";

	webString += "<table>\n";
	webString += "<th colspan=\"2\"><span><b>WiFi Status</b></span></th>\n";
	webString += "<tr><td align=\"right\"><b>Mode:</b></td>\n";
	webString += "<td align=\"left\">";
	if (config.wifi_mode == WIFI_AP_FIX)
	{
		webString += "AP";
	}
	else if (config.wifi_mode == WIFI_STA_FIX)
	{
		webString += "STA";
	}
	else if (config.wifi_mode == WIFI_AP_STA_FIX)
	{
		webString += "AP+STA";
	}
	else
	{
		webString += "OFF";
	}

	wifi_power_t wpr = WiFi.getTxPower();
	String wifipower = "";
	if (wpr < 8)
	{
		wifipower = "-1 dBm";
	}
	else if (wpr < 21)
	{
		wifipower = "2 dBm";
	}
	else if (wpr < 29)
	{
		wifipower = "5 dBm";
	}
	else if (wpr < 35)
	{
		wifipower = "8.5 dBm";
	}
	else if (wpr < 45)
	{
		wifipower = "11 dBm";
	}
	else if (wpr < 53)
	{
		wifipower = "13 dBm";
	}
	else if (wpr < 61)
	{
		wifipower = "15 dBm";
	}
	else if (wpr < 69)
	{
		wifipower = "17 dBm";
	}
	else if (wpr < 75)
	{
		wifipower = "18.5 dBm";
	}
	else if (wpr < 77)
	{
		wifipower = "19 dBm";
	}
	else if (wpr < 80)
	{
		wifipower = "19.5 dBm";
	}
	else
	{
		wifipower = "20 dBm";
	}

	webString += "</td></tr>\n";
	webString += "<tr><td align=\"right\" width=\"30%\"><b>MAC:</b></td>\n";
	webString += "<td align=\"left\">" + String(WiFi.macAddress()) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>Channel:</b></td>\n";
	webString += "<td align=\"left\">" + String(WiFi.channel()) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>TX Power:</b></td>\n";
	webString += "<td align=\"left\">" + wifipower + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>SSID:</b></td>\n";
	webString += "<td align=\"left\">" + String(WiFi.SSID()) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>Local IP:</b></td>\n";
	webString += "<td align=\"left\">" + WiFi.localIP().toString() + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>Gateway IP:</b></td>\n";
	webString += "<td align=\"left\">" + WiFi.gatewayIP().toString() + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>DNS:</b></td>\n";
	webString += "<td align=\"left\">" + WiFi.dnsIP().toString() + "</td></tr>\n";
	webString += "</table>\n";

	webString += "</td><td width=\"2%\" style=\"border:unset;\"></td>";
	webString += "<td width=\"49%\" style=\"border:unset;\">";
	webString += "<table>\n";
	webString += "<th colspan=\"2\"><span><b>PPPoS Status</b></span></th>\n";
	webString += "<tr><td align=\"right\" width=\"30%\"><b>Manufacturer:</b></td>\n";
	webString += "<td align=\"left\">" + String(pppStatus.manufacturer) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>Model:</b></td>\n";
	webString += "<td align=\"left\">" + String(pppStatus.model) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>IMEI:</b></td>\n";
	webString += "<td align=\"left\">" + String(pppStatus.imei) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>IMSI:</b></td>\n";
	webString += "<td align=\"left\">" + String(pppStatus.imsi) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>Operator:</b></td>\n";
	webString += "<td align=\"left\">" + String(pppStatus.oper) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>RSSI:</b></td>\n";
	webString += "<td align=\"left\">" + String(pppStatus.rssi) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>IP:</b></td>\n";
	webString += "<td align=\"left\">" + String(IPAddress(pppStatus.ip)) + "</td></tr>\n";
	webString += "<tr><td align=\"right\"><b>Gateway:</b></td>\n";
	webString += "<td align=\"left\">" + String(IPAddress(pppStatus.gateway)) + "</td></tr>\n";
	// webString += "<tr><td align=\"right\"><b>DNS:</b></td>\n";
	// webString += "<td align=\"left\">" + String(IPAddress(pppStatus.dns)) + "</td></tr>\n";
	webString += "</table>\n";
	webString += "</td></tr></table><br />";

	// webString += "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;\"><td width=\"96%\" style=\"border:unset;\">";

	webString += "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form' class=\"form-horizontal\">\n";
	webString += "<table>";
	webString += "<th colspan=\"2\"><span><b>Firmware Update</b></span></th>\n";
	webString += "<tr><td align=\"right\"><b>File:</b></td><td align=\"left\"><input id=\"file\" name=\"update\" type=\"file\" onchange='sub(this)' /></td></tr>\n";
	webString += "<tr><td align=\"right\"><b>Progress:</b></td><td><div id='prgbar'><div id='bar' style=\"width: 0px;\"><label id='prg'></label></div></div></td></tr>\n";
	webString += "<tr><td align=\"right\"><b>Support Firmware:</b></td><td align=\"left\"><a target=\"_download\" href=\"https://github.com/nakhonthai/ESP32APRS_Audio/releases\">https://github.com/nakhonthai/ESP32APRS_Audio/releases</a></td></tr>\n";
	webString += "</table><br />\n";
	webString += "<div class=\"col-sm-3 col-xs-4\"><input type='submit' class=\"btn btn-danger\" id=\"update_sumbit\" value='Firmware Update'></div>\n";

	webString += "</form>\n";
	// webString += "</td></tr></table><br />";

	webString += "<script>"
				 "function sub(obj){"
				 "var fileName = obj.value.split('\\\\');"
				 "document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
				 "};"
				 "$('form').submit(function(e){"
				 "e.preventDefault();"
				 "var form = $('#upload_form')[0];"
				 "var data = new FormData(form);"
				 "document.getElementById('update_sumbit').disabled = true;"
				 "$.ajax({"
				 "url: '/update',"
				 "type: 'POST',"
				 "data: data,"
				 "contentType: false,"
				 "processData:false,"
				 "xhr: function() {"
				 "var xhr = new window.XMLHttpRequest();"
				 "xhr.upload.addEventListener('progress', function(evt) {"
				 "if (evt.lengthComputable) {"
				 "var per = evt.loaded / evt.total;"
				 "$('#prg').html(Math.round(per*100) + '%');"
				 "$('#bar').css('width',Math.round(per*100) + '%');"
				 "}"
				 "}, false);"
				 "return xhr;"
				 "},"
				 "success:function(d, s) {"
				 "alert('Wait for system reboot 10sec') "
				 "},"
				 "error: function (a, b, c) {"
				 "}"
				 "});"
				 "});"
				 "</script>";

	webString += "</body></html>\n";
	request->send(200, "text/html", webString); // send to someones browser when asked
}

void handle_gnss(AsyncWebServerRequest *request)
{
	webString = "<html>\n<head>\n";
	webString += "<script src=\"https://apps.bdimg.com/libs/jquery/2.1.4/jquery.min.js\"></script>\n";
	webString += "<script src=\"https://code.highcharts.com/highcharts.js\"></script>\n";
	webString += "<script src=\"https://code.highcharts.com/highcharts-more.js\"></script>\n";
	webString += "<script language=\"JavaScript\">";

	// Add some life
	webString += "function gnss() { \n"; // the chart may be destroyed
	webString += "var raw=\"\";var timeStamp;\n";
	webString += "var host='ws://'+location.hostname+':81/ws_gnss'\n";
	webString += "const ws = new WebSocket(host);\n";
	webString += "ws.onopen = function() { console.log('Connection opened');};\n ws.onclose = function() { console.log('Connection closed');};\n";
	webString += "ws.onmessage = function(event) {\n  console.log(event.data);\n";
	webString += "const jsonR=JSON.parse(event.data);\n";
	webString += "document.getElementById(\"en\").innerHTML=parseInt(jsonR.en);\n";
	webString += "document.getElementById(\"lat\").innerHTML=parseFloat(jsonR.lat);\n";
	webString += "document.getElementById(\"lng\").innerHTML=parseFloat(jsonR.lng);\n";
	webString += "document.getElementById(\"alt\").innerHTML=parseFloat(jsonR.alt);\n";
	webString += "document.getElementById(\"spd\").innerHTML=parseFloat(jsonR.spd);\n";
	webString += "document.getElementById(\"csd\").innerHTML=parseFloat(jsonR.csd);\n";
	webString += "document.getElementById(\"hdop\").innerHTML=parseFloat(jsonR.hdop);\n";
	webString += "document.getElementById(\"sat\").innerHTML=parseInt(jsonR.sat);\n";
	webString += "document.getElementById(\"time\").innerHTML=parseInt(jsonR.time);\n";
	webString += "raw=jsonR.RAW;\n";
	webString += "timeStamp=Number(jsonR.timeStamp);\n";
	webString += "var textArea=document.getElementById(\"raw_txt\");\n";
	webString += "textArea.value+=atob(raw)+\"\\n\";\n";
	webString += "textArea.scrollTop = textArea.scrollHeight;\n";
	webString += "}\n";
	webString += "};\n</script>\n";
	webString += "</head><body onload=\"gnss()\">\n";

	webString += "<table width=\"200\" border=\"1\">";
	webString += "<th colspan=\"2\" style=\"background-color: #00BCD4;\"><span><b>GNSS Information</b></span></th>\n";
	// webString += "<tr><th width=\"200\"><span><b>Name</b></span></th><th><span><b>Information</b></span></th></tr>";
	webString += "<tr><td align=\"right\"><b>Enable: </b></td><td align=\"left\"> <label id=\"en\">" + String(config.gnss_enable) + "</label></td></tr>";
	webString += "<tr><td align=\"right\"><b>Latitude: </b></td><td align=\"left\"> <label id=\"lat\">" + String(gps.location.lat(), 5) + "</label></td></tr>";
	webString += "<tr><td align=\"right\"><b>Longitude: </b></td><td align=\"left\"> <label id=\"lng\">" + String(gps.location.lng(), 5) + "</label></td></tr>";
	webString += "<tr><td align=\"right\"><b>Altitude: </b></td><td align=\"left\"> <label id=\"alt\">" + String(gps.altitude.meters(), 2) + "</label> m.</td></tr>";
	webString += "<tr><td align=\"right\"><b>Speed: </b></td><td align=\"left\"> <label id=\"spd\">" + String(gps.speed.kmph(), 2) + "</label> km/h</td></tr>";
	webString += "<tr><td align=\"right\"><b>Course: </b></td><td align=\"left\"> <label id=\"csd\">" + String(gps.course.deg(), 1) + "</label></td></tr>";
	webString += "<tr><td align=\"right\"><b>HDOP: </b></td><td align=\"left\"> <label id=\"hdop\">" + String(gps.hdop.hdop(), 2) + "</label> </td></tr>";
	webString += "<tr><td align=\"right\"><b>SAT: </b></td><td align=\"left\"> <label id=\"sat\">" + String(gps.satellites.value()) + "</label> </td></tr>";
	webString += "<tr><td align=\"right\"><b>Time: </b></td><td align=\"left\"> <label id=\"time\">" + String(gps.time.value()) + "</label> </td></tr>";
	webString += "</table><table>";
	webString += "<tr><td><b>Terminal:</b><br /><textarea id=\"raw_txt\" name=\"raw_txt\" rows=\"30\" cols=\"80\" /></textarea></td></tr>\n";
	webString += "</table>\n";

	webString += "</body></html>\n";
	request->send(200, "text/html", webString); // send to someones browser when asked

	delay(100);
	webString.clear();
}

void handle_default()
{
	defaultSetting = true;
	defaultConfig();
	defaultSetting = false;
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{

	if (type == WS_EVT_CONNECT)
	{

		log_d("Websocket client connection received");
	}
	else if (type == WS_EVT_DISCONNECT)
	{

		log_d("Client disconnected");
	}
}

bool webServiceBegin = true;
void webService()
{
	if (webServiceBegin)
	{
		webServiceBegin = false;
	}
	else
	{
		return;
	}
	ws.onEvent(onWsEvent);

	// web client handlers
	async_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
					{ setMainPage(request); });
	async_server.on("/symbol", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_symbol(request); });
	// async_server.on("/symbol2", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
	// 				{ handle_symbol2(request); });
	async_server.on("/logout", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_logout(request); });
	async_server.on("/radio", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_radio(request); });
	async_server.on("/vpn", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_vpn(request); });
#ifdef MQTT
	async_server.on("/mqtt", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_mqtt(request); });
#endif
	async_server.on("/msg", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_msg(request); });					
	async_server.on("/mod", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_mod(request); });
	async_server.on("/default", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_default(); });
	async_server.on("/igate", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_igate(request); });
	async_server.on("/digi", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_digi(request); });
	async_server.on("/tracker", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_tracker(request); });
	async_server.on("/wx", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_wx(request); });
	async_server.on("/tlm", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_tlm(request); });
	async_server.on("/sensor", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_sensor(request); });
	async_server.on("/system", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_system(request); });
	async_server.on("/wireless", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_wireless(request); });
	async_server.on("/tnc2", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_test(request); });
	async_server.on("/gnss", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_gnss(request); });
	// async_server.on("/realtime", HTTP_GET, [](AsyncWebServerRequest *request)
	// 				{ handle_realtime(request); });
	async_server.on("/about", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_about(request); });
	async_server.on("/dashboard", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_dashboard(request); });
	async_server.on("/sidebarInfo", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_sidebar(request); });
	async_server.on("/sysinfo", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_sysinfo(request); });
	// async_server.on("/lastHeard", HTTP_GET, [](AsyncWebServerRequest *request)
	// 				{ handle_lastHeard(request); });
	async_server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_css(request); });
	async_server.on("/jquery-3.7.1.js", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_jquery(request); });
	async_server.on("/storage", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_storage(request); });
	async_server.on("/download", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_download(request); });
	async_server.on("/delete", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_delete(request); });
	async_server.on("/format", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_format(request); });
	async_server.on(
		"/update", HTTP_POST, [](AsyncWebServerRequest *request)
		{
  		bool espShouldReboot = !Update.hasError();
  		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", espShouldReboot ? "<h1><strong>Update DONE</strong></h1><br><a href='/'>Return Home</a>" : "<h1><strong>Update FAILED</strong></h1><br><a href='/updt'>Retry?</a>");
  		response->addHeader("Connection", "close");
  		request->send(response); },
		[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
		{
			if (!index)
			{
				log_d("Update Start: %s\n", filename.c_str());
				if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000))
				{
					Update.printError(Serial);
				}
				else
				{
					adcEn=-1;
					dacEn=-1;
					delay(500);
					// disableLoopWDT();
					// disableCore0WDT();
					// disableCore1WDT();
					//  vTaskSuspend(taskAPRSPollHandle);
					//  vTaskSuspend(taskAPRSHandle);
					//  vTaskSuspend(taskSensorHandle);
					//  vTaskSuspend(taskSerialHandle);
					//  vTaskSuspend(taskGPSHandle);
					//  vTaskSuspend(taskSensorHandle);
				}
			}
			if (!Update.hasError())
			{
				if (Update.write(data, len) != len)
				{
					Update.printError(Serial);
				}
			}
			if (final)
			{
				if (Update.end(true))
				{
					log_d("Update Success: %uByte\n", index + len);
					delay(1000);
					esp_restart();
				}
				else
				{
					Update.printError(Serial);
				}
			}
		});

	lastheard_events.onConnect([](AsyncEventSourceClient *client)
							   {
    if(client->lastId()){
      log_d("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    String html = event_lastHeard(true);
    client->send(html.c_str(), "lastHeard", time(NULL), 5000); });
	async_server.addHandler(&lastheard_events);

	message_events.onConnect([](AsyncEventSourceClient *client)
							 {
    if(client->lastId()){
      log_d("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
	String html = event_chatMessage(true);
    client->send(html.c_str(), "chatMsg", time(NULL), 5000); });
	async_server.addHandler(&message_events);
	
	async_server.onNotFound(notFound);
	async_server.begin();
	async_websocket.addHandler(&ws);
	async_websocket.addHandler(&ws_gnss);
	async_websocket.begin();
}
