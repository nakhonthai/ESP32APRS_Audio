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
#include <ESPCPUTemp.h>
#include "esp_wifi.h"
#include "esp_heap_caps.h"

// Helper function to allocate memory with PSRAM support
char *allocateStringMemory(size_t size)
{
#ifdef BOARD_HAS_PSRAM
	// Try to allocate in PSRAM first
	size*=2; // overallocation to reduce fragmentation
	char *ptr = (char *)ps_calloc(size, sizeof(char));
	if (ptr != NULL)
	{
		return ptr;
	}
	// If PSRAM allocation fails, fall back to regular heap
#endif
	// Regular heap allocation
	return (char *)calloc(size, sizeof(char));
}

// Helper function to format integers to string using allocateStringMemory
char *intToString(int value)
{
	char *str = allocateStringMemory(12); // Enough for a 32-bit integer + null terminator
	if (str != NULL)
	{
		sprintf(str, "%d", value);
	}
	return str;
}

// Helper function to format floats to string using allocateStringMemory
char *floatToString(float value, int decimals)
{
	char *str = allocateStringMemory(20); // Enough for most float values
	if (str != NULL)
	{
		switch (decimals)
		{
		case 0:
			sprintf(str, "%.0f", value);
			break;
		case 1:
			sprintf(str, "%.1f", value);
			break;
		case 2:
			sprintf(str, "%.2f", value);
			break;
		case 3:
			sprintf(str, "%.3f", value);
			break;
		default:
			sprintf(str, "%.2f", value);
			break;
		}
	}
	return str;
}

// Helper function to convert Arduino String to char* using allocateStringMemory
char *StringToCharPtr(const String &str)
{
	size_t len = str.length() + 1; // +1 for null terminator
	char *charPtr = allocateStringMemory(len);
	if (charPtr != NULL)
	{
		strcpy(charPtr, str.c_str());
	}
	return charPtr;
}

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

#ifdef PPPOS
extern pppType pppStatus;
#endif

// Create an Event Source on /events
AsyncEventSource lastheard_events("/eventHeard");
AsyncEventSource message_events("/eventMsg");

char *webString;

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
	char *webString = allocateStringMemory(64); // Small buffer for "Log out"
	if (!webString)
	{
		return; // Memory allocation failed
	}
	strcpy(webString, "Log out");
	request->send(200, "text/html", webString);
	free(webString); // Free the allocated memory
}

void setMainPage(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}

	// Using dynamic memory allocation instead of String
	char *webString = allocateStringMemory(12000); // Initial buffer size, adjust as needed
	if (!webString)
	{
		return; // Memory allocation failed
	}

	strcpy(webString, "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n");
	strcat(webString, "<meta name=\"robots\" content=\"index\" />\n");
	strcat(webString, "<meta name=\"robots\" content=\"follow\" />\n");
	strcat(webString, "<meta name=\"language\" content=\"English\" />\n");
	strcat(webString, "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\" />\n");
	strcat(webString, "<meta name=\"GENERATOR\" content=\"configure 20230924\" />\n");
	strcat(webString, "<meta name=\"Author\" content=\"Mr.Somkiat Nakhonthai (HS5TQA)\" />\n");
	strcat(webString, "<meta name=\"Description\" content=\"Web Embedded Configuration\" />\n");
	strcat(webString, "<meta name=\"KeyWords\" content=\"ESP32,ESP32C3,AFSK,APRS\" />\n");
	strcat(webString, "<meta http-equiv=\"Cache-Control\" content=\"no-cache, no-store, must-revalidate\" />\n");
	strcat(webString, "<meta http-equiv=\"pragma\" content=\"no-cache\" />\n");
	strcat(webString, "<link rel=\"shortcut icon\" href=\"http://aprs.dprns.com/favicon.ico\" type=\"image/x-icon\" />\n");
	strcat(webString, "<meta http-equiv=\"Expires\" content=\"0\" />\n");

	char temp_buffer[512];
	if (strlen(config.host_name) > 0)
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<title>%s</title>\n", config.host_name);
		strcat(webString, temp_buffer);
	}
	else
	{
		strcat(webString, "<title>ESP32APRS_Audio</title>\n");
	}

	strcat(webString, "<link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\" />\n");
	strcat(webString, "<script src=\"/jquery-3.7.1.js\"></script>\n");
	strcat(webString, "<script type=\"text/javascript\">\n");
	strcat(webString, "function selectTab(evt, tabName) {\n");
	strcat(webString, "var i, tabcontent, tablinks;\n");
	strcat(webString, "tablinks = document.getElementsByClassName(\"nav-tabs\");\n");
	strcat(webString, "for (i = 0; i < tablinks.length; i++) {\n");
	strcat(webString, "tablinks[i].className = tablinks[i].className.replace(\" active\", \"\");\n");
	strcat(webString, "}\n");
	strcat(webString, "\n");
	strcat(webString, "//document.getElementById(tabName).style.display = \"block\";\n");
	strcat(webString, "if (tabName == 'DashBoard') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/dashboard\");\n");
	strcat(webString, "} else if (tabName == 'Radio') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/radio\");\n");
	strcat(webString, "} else if (tabName == 'IGATE') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/igate\");\n");
	strcat(webString, "} else if (tabName == 'DIGI') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/digi\");\n");
	strcat(webString, "} else if (tabName == 'TRACKER') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/tracker\");\n");
	strcat(webString, "} else if (tabName == 'WX') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/wx\");\n");
	strcat(webString, "} else if (tabName == 'TLM') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/tlm\");\n");
	strcat(webString, "} else if (tabName == 'SENSOR') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/sensor\");\n");
	strcat(webString, "} else if (tabName == 'VPN') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/vpn\");\n");
#ifdef MQTT
	strcat(webString, "} else if (tabName == 'MQTT') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/mqtt\");\n");
#endif
	strcat(webString, "} else if (tabName == 'MSG') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/msg\");\n");
	strcat(webString, "} else if (tabName == 'WiFi') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/wireless\");\n");
	strcat(webString, "} else if (tabName == 'MOD') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/mod\");\n");
	strcat(webString, "} else if (tabName == 'System') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/system\");\n");
	strcat(webString, "} else if (tabName == 'File') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/storage\");\n");
	strcat(webString, "} else if (tabName == 'About') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/about\");\n");
	strcat(webString, "}\n");
	strcat(webString, "\n");
	strcat(webString, "if (evt != null) evt.currentTarget.className += \" active\";\n");
	strcat(webString, "}\n");
	strcat(webString, "if (!!window.EventSource) {");
	strcat(webString, "var source = new EventSource('/eventHeard');");

	strcat(webString, "source.addEventListener('open', function(e) {");
	strcat(webString, "console.log(\"Events Connected\");");
	strcat(webString, "}, false);");
	strcat(webString, "source.addEventListener('error', function(e) {");
	strcat(webString, "if (e.target.readyState != EventSource.OPEN) {");
	strcat(webString, "console.log(\"Events Disconnected\");");
	strcat(webString, "}\n}, false);");
	strcat(webString, "source.addEventListener('lastHeard', function(e) {");
	// strcat(webString, "console.log(\"lastHeard\", e.data);");
	strcat(webString, "var lh=document.getElementById(\"aprsTable\");");
	strcat(webString, "if(lh != null) {renderTable(e.data);}");
	strcat(webString, "}, false);\n}");
	strcat(webString, "if (!!window.EventSource) {");
	strcat(webString, "var source = new EventSource('/eventMsg');");

	strcat(webString, "source.addEventListener('open', function(e) {");
	strcat(webString, "console.log(\"Events MSG Connected\");");
	strcat(webString, "}, false);");
	strcat(webString, "source.addEventListener('error', function(e) {");
	strcat(webString, "if (e.target.readyState != EventSource.OPEN) {");
	strcat(webString, "console.log(\"Events MSG Disconnected\");");
	strcat(webString, "}\n}, false);");
	strcat(webString, "source.addEventListener('chatMsg', function(e) {");
	// strcat(webString, "console.log(\"lastHeard\", e.data);");
	strcat(webString, "var lh=document.getElementById(\"chatMsg\");");
	strcat(webString, "if(lh != null) {lh.innerHTML = e.data;}");
	strcat(webString, "}, false);\n}\n");
	//strcat(webString, "</script>\n");

	strcat(webString, "let sortDirection = {};\n");
	strcat(webString, "let currentSortKey = \"time\";\n\n");
	strcat(webString, "function renderTable(raw) {\n");
	// strcat(webString, "const tableBody = document.getElementById(\"aprsTableBody\");\n");
	// //strcat(webString, "const tableBody = document.querySelector(\"#aprsTable tbody\");\n");
	// strcat(webString, "if(tableBody == null) {return;}\n");
	strcat(webString, "var data=JSON.parse(raw);\n");	
	strcat(webString, "lastHeardSort(data);\n");
	strcat(webString, "document.querySelectorAll(\"#aprsTable th[data-sort]\")\n");
	strcat(webString, ".forEach(header => {\n\n");
	strcat(webString, "header.addEventListener(\"click\", () => {\n\n");
	strcat(webString, "const key = header.dataset.sort;\n\n");
	strcat(webString, "sortDirection[key] = !sortDirection[key];\n");
	strcat(webString, "currentSortKey = key;\n\n");	
	strcat(webString, "clearArrows();\n\n");
	strcat(webString, "const arrowSpan = header.querySelector(\".arrow\");\n");
	strcat(webString, "arrowSpan.textContent = sortDirection[key] ? \"▲\" : \"▼\";\n\n");
	strcat(webString, "lastHeardSort(data);\n");
	strcat(webString, "printLastHeard(data);\n");
	strcat(webString, "});\n\n");
	strcat(webString, "});\n\n");
	strcat(webString, "printLastHeard(data);\n");
	strcat(webString, "}\n\n");

	strcat(webString, "function printLastHeard(data) {\n");
	strcat(webString, "const tableBody = document.getElementById(\"aprsTableBody\");\n");
	//strcat(webString, "const tableBody = document.querySelector(\"#aprsTable tbody\");\n");
	strcat(webString, "if(tableBody == null) {return;}\n");
	strcat(webString, "tableBody.innerHTML = \"\";\n");
	strcat(webString, "data.forEach(row => {\n");
	strcat(webString, "const tr = document.createElement(\"tr\");\n");
	strcat(webString, "tr.innerHTML = `\n");
	strcat(webString, "<td>${row.time}</td>\n");
	strcat(webString, "<td><img src=\"http://aprs.nakhonthai.net/symbols/icons/${row.icon}\"></td>\n");
	strcat(webString, "<td>${row.callsign}</td>\n");
	strcat(webString, "<td align=\"left\">${row.path}</td>\n");
	strcat(webString, "<td>${row.dx !== null ? row.dx : \"-\"}</td>\n");
	strcat(webString, "<td>${row.packet}</td>\n");
	strcat(webString, "<td style=\"color:green;\">${row.audio !== '-' ? row.audio + \"dBV\" : \"-\"}</td>\n");
	strcat(webString, "`;\n");
	strcat(webString, "tableBody.appendChild(tr);\n");
	strcat(webString, "});\n");
	strcat(webString, "}\n\n");

	strcat(webString, "function clearArrows() {\n");
	strcat(webString, "document.querySelectorAll(\".arrow\").forEach(a => a.textContent = \"\");\n");
	strcat(webString, "}\n\n");
	strcat(webString, "function lastHeardSort(data) {\nvar key=currentSortKey;\n");
	strcat(webString, "data.sort((a, b) => {\n\n");
	strcat(webString, "let valA = a[key];\n");
	strcat(webString, "let valB = b[key];\n\n");
	strcat(webString, "if (key === \"time\") {\n");
	//strcat(webString, "// Parse time in dd hh:mm:ss format\n");
	strcat(webString, "const [dayTime, timePart] = valA.split(' ');\n");
	strcat(webString, "const [hours, minutes, seconds] = timePart.split(':');\n");
	strcat(webString, "valA = parseInt(dayTime) * 86400 + parseInt(hours) * 3600 + parseInt(minutes) * 60 + parseInt(seconds);\n");
	//strcat(webString, "                \n");
	strcat(webString, "const [dayTimeB, timePartB] = valB.split(' ');\n");
	strcat(webString, "const [hoursB, minutesB, secondsB] = timePartB.split(':');\n");
	strcat(webString, "valB = parseInt(dayTimeB) * 86400 + parseInt(hoursB) * 3600 + parseInt(minutesB) * 60 + parseInt(secondsB);\n");
	strcat(webString, "}\n\n");
	strcat(webString, "if (valA === null) return 1;\n");
	strcat(webString, "if (valB === null) return -1;\n\n");
	strcat(webString, "if (valA < valB) return sortDirection[key] ? -1 : 1;\n");
	strcat(webString, "if (valA > valB) return sortDirection[key] ? 1 : -1;\n");
	strcat(webString, "return 0;\n");
	strcat(webString, "});\n\n");
	strcat(webString, "}\n\n");
	strcat(webString, "</script>\n");
	strcat(webString, "</head>\n");
	//strcat(webString, "\n");
	strcat(webString, "<body onload=\"selectTab(event, 'DashBoard')\">\n");
	strcat(webString, "\n");
	strcat(webString, "<div class=\"container\">\n");
	strcat(webString, "<div class=\"header\">\n");
	// strcat(webString, "<div style=\"font-size: 8px; text-align: right; padding-right: 8px;\">ESP32IGate Firmware V" + String(VERSION) + "</div>\n");
	// strcat(webString, "<div style=\"font-size: 8px; text-align: right; padding-right: 8px;\"><a href=\"/logout\">[LOG OUT]</a></div>\n");
	if (strlen(config.host_name) > 0)
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<h1>%s</h1>\n", config.host_name);
		strcat(webString, temp_buffer);
	}
	else
	{
		strcat(webString, "<h1>ESP32APRS_Audio</h1>\n");
	}
	strcat(webString, "<div style=\"font-size: 8px; text-align: right; padding-right: 8px;\"><a href=\"/logout\">[LOG OUT]</a></div>\n");
	strcat(webString, "<div class=\"row\">\n");
	strcat(webString, "<ul class=\"nav nav-tabs\" style=\"margin: 5px;\">\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'DashBoard')\">DashBoard</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'Radio')\" id=\"btnRadio\">Radio</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'IGATE')\">IGATE</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'DIGI')\">DIGI</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'TRACKER')\">TRACKER</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'WX')\">WX</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'TLM')\">TLM</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'SENSOR')\">SENSOR</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'VPN')\">VPN</button>\n");
#ifdef MQTT
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'MQTT')\">MQTT</button>\n");
#endif
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'MSG')\">MSG</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'WiFi')\">WiFi</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'MOD')\">MOD</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'System')\">System</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'File')\">File</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'About')\">About</button>\n");
	strcat(webString, "</ul>\n");
	strcat(webString, "</div>\n");
	strcat(webString, "</div>\n");
	strcat(webString, "\n");

	strcat(webString, "<div class=\"contentwide\" id=\"contentmain\"  style=\"font-size: 2pt;\">\n");
	strcat(webString, "\n");
	strcat(webString, "</div>\n");
	strcat(webString, "<br />\n");
	strcat(webString, "<div class=\"footer\">\n");
	strcat(webString, "ESP32APRS_Audio Web Configuration<br />Copy right ©2023.\n");
	strcat(webString, "<br />\n");
	strcat(webString, "</div>\n");
	strcat(webString, "</div>\n");
	strcat(webString, "<!-- <script type=\"text/javascript\" src=\"/nice-select.min.js\"></script> -->\n");
	strcat(webString, "<script type=\"text/javascript\">\n");
	strcat(webString, "var selectize = document.querySelectorAll('select')\n");
	strcat(webString, "var options = { searchable: true };\n");
	strcat(webString, "selectize.forEach(function (select) {\n");
	strcat(webString, "if (select.length > 30 && null === select.onchange && !select.name.includes(\"ExtendedId\")) {\n");
	strcat(webString, "select.classList.add(\"small\", \"selectize\");\n");
	strcat(webString, "tabletd = select.closest('td');\n");
	strcat(webString, "tabletd.style.cssText = 'overflow-x:unset';\n");
	strcat(webString, "NiceSelect.bind(select, options);\n");
	strcat(webString, "}\n");
	strcat(webString, "});\n");
	strcat(webString, "</script>\n");
	strcat(webString, "</body>\n");
	strcat(webString, "</html>");


	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)webString);
	response->addHeader("Sensor", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	free(webString);
	lastHeardTimeout = 0;
	lastHeard_Flag = true;
}

////////////////////////////////////////////////////////////
// handler for web server request: http://IpAddress/      //
////////////////////////////////////////////////////////////

void handle_css(AsyncWebServerRequest *request)
{
	const char *css = ".container{width:900px;text-align:left;margin:auto;border-radius:10px 10px 10px 10px;-moz-border-radius:10px 10px 10px 10px;-webkit-border-radius:10px 10px 10px 10px;-khtml-border-radius:10px 10px 10px 10px;-ms-border-radius:10px 10px 10px 10px;box-shadow:3px 3px 3px #707070;background:#fff;border-color: #2194ec;padding: 0px;border-width: 5px;border-style:solid;}body,font{font:12px verdana,arial,sans-serif;color:#fff}.header{background:#2194ec;text-decoration:none;color:#fff;font-family:verdana,arial,sans-serif;text-align:left;padding:5px 0;border-radius:10px 10px 0 0;-moz-border-radius:10px 10px 0 0;-webkit-border-radius:10px 10px 0 0;-khtml-border-radius:10px 10px 0 0;-ms-border-radius:10px 10px 0 0}.content{margin:0 0 0 166px;padding:1px 5px 5px;color:#000;background:#fff;text-align:center;font-size: 8pt;}.contentwide{padding:50px 5px 5px;color:#000;background:#fff;text-align:center}.contentwide h2{color:#000;font:1em verdana,arial,sans-serif;text-align:center;font-weight:700;padding:0;margin:0;font-size: 12pt;}.footer{background:#2194ec;text-decoration:none;color:#fff;font-family:verdana,arial,sans-serif;font-size:9px;text-align:center;padding:10px 0;border-radius:0 0 10px 10px;-moz-border-radius:0 0 10px 10px;-webkit-border-radius:0 0 10px 10px;-khtml-border-radius:0 0 10px 10px;-ms-border-radius:0 0 10px 10px;clear:both}#tail{height:450px;width:805px;overflow-y:scroll;overflow-x:scroll;color:#0f0;background:#000}table{vertical-align:middle;text-align:center;empty-cells:show;padding-left:3;padding-right:3;padding-top:3;padding-bottom:3;border-collapse:collapse;border-color:#0f07f2;border-style:solid;border-spacing:0px;border-width:3px;text-decoration:none;color:#fff;background:#000;font-family:verdana,arial,sans-serif;font-size : 12px;width:100%;white-space:nowrap}table th{cursor: pointer;user-select: none;font-size: 10pt;font-family:lucidia console,Monaco,monospace;text-shadow:1px 1px #0e038c;text-decoration:none;background:#0525f7;border:1px solid silver}table tr:nth-child(even){background:#f7f7f7}table tr:nth-child(odd){background:#eeeeee}table td{color:#000;font-family:lucidia console,Monaco,monospace;text-decoration:none;border:1px solid #010369}body{background:#edf0f5;color:#000}a{text-decoration:none}a:link,a:visited{text-decoration:none;color:#0000e0;font-weight:400}th:last-child a.tooltip:hover span{left:auto;right:0}ul{padding:5px;margin:10px 0;list-style:none;float:left}ul li{float:left;display:inline;margin:0 10px}ul li a{text-decoration:none;float:left;color:#999;cursor:pointer;font:900 14px/22px arial,Helvetica,sans-serif}ul li a span{margin:0 10px 0 -10px;padding:1px 8px 5px 18px;position:relative;float:left}h1{text-shadow:2px 2px #303030;text-align:center}.toggle{position:absolute;margin-left:-9999px;visibility:hidden}.toggle+label{display:block;position:relative;cursor:pointer;outline:none}input.toggle-round-flat+label{padding:1px;width:33px;height:18px;background-color:#ddd;border-radius:10px;transition:background .4s}input.toggle-round-flat+label:before,input.toggle-round-flat+label:after{display:block;position:absolute;}input.toggle-round-flat+label:before{top:1px;left:1px;bottom:1px;right:1px;background-color:#fff;border-radius:10px;transition:background .4s}input.toggle-round-flat+label:after{top:2px;left:2px;bottom:2px;width:16px;background-color:#ddd;border-radius:12px;transition:margin .4s,background .4s}input.toggle-round-flat:checked+label{background-color:#dd4b39}input.toggle-round-flat:checked+label:after{margin-left:14px;background-color:#dd4b39}@-moz-document url-prefix(){select,input{margin:0;padding:0;border-width:1px;font:12px verdana,arial,sans-serif}input[type=button],button,input[type=submit]{padding:0 3px;border-radius:3px 3px 3px 3px;-moz-border-radius:3px 3px 3px 3px}}.nice-select.small,.nice-select-dropdown li.option{height:24px!important;min-height:24px!important;line-height:24px!important}.nice-select.small ul li:nth-of-type(2){clear:both}.nav{margin-bottom:0;padding-left:10;list-style:none}.nav>li{position:relative;display:block}.nav>li>a{position:relative;display:block;padding:5px 10px}.nav>li>a:hover,.nav>li>a:focus{text-decoration:none;background-color:#eee}.nav>li.disabled>a{color:#999}.nav>li.disabled>a:hover,.nav>li.disabled>a:focus{color:#999;text-decoration:none;background-color:initial;cursor:not-allowed}.nav .open>a,.nav .open>a:hover,.nav .open>a:focus{background-color:#eee;border-color:#428bca}.nav .nav-divider{height:1px;margin:9px 0;overflow:hidden;background-color:#e5e5e5}.nav>li>a>img{max-width:none}.nav-tabs{border-bottom:1px solid #ddd}.nav-tabs>li{float:left;margin-bottom:-1px}.nav-tabs>li>a{margin-right:0;line-height:1.42857143;border:1px solid #ddd;border-radius:10px 10px 0 0}.nav-tabs>li>a:hover{border-color:#eee #eee #ddd}.nav-tabs>button{margin-right:0;line-height:1.42857143;border:2px solid #ddd;border-radius:10px 10px 0 0}.nav-tabs>button:hover{background-color:#25bbfc;border-color:#428bca;color:#eaf2f9;border-bottom-color:transparent;}.nav-tabs>button.active,.nav-tabs>button.active:hover,.nav-tabs>button.active:focus{color:#f7fdfd;background-color:#1aae0d;border:1px solid #ddd;border-bottom-color:transparent;cursor:default}.nav-tabs>li.active>a,.nav-tabs>li.active>a:hover,.nav-tabs>li.active>a:focus{color:#428bca;background-color:#e5e5e5;border:1px solid #ddd;border-bottom-color:transparent;cursor:default}.nav-tabs.nav-justified{width:100%;border-bottom:0}.nav-tabs.nav-justified>li{float:none}.nav-tabs.nav-justified>li>a{text-align:center;margin-bottom:5px}.nav-tabs.nav-justified>.dropdown .dropdown-menu{top:auto;left:auto}.nav-status{float:left;margin:0;padding:3px;width:160px;font-weight:400;min-height:600}#bar,#prgbar {background-color: #f1f1f1;border-radius: 14px}#bar {background-color: #3498db;width: 0%;height: 14px}.switch{position:relative;display:inline-block;width:34px;height:16px}.switch input{opacity:0;width:0;height:0}.slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:#f55959;-webkit-transition:.4s;transition:.4s}.slider:before{position:absolute;content:\"\";height:12px;width:12px;left:2px;bottom:2px;background-color:#fff;-webkit-transition:.4s;transition:.4s}input:checked+.slider{background-color:#5ca30a}input:focus+.slider{box-shadow:0 0 1px #5ca30a}input:checked+.slider:before{-webkit-transform:translateX(16px);-ms-transform:translateX(16px);transform:translateX(16px)}.slider.round{border-radius:34px}.slider.round:before{border-radius:50%}.button{border:1px solid #06c;background-color:#09c;color:#fff;padding:5px 10px;border-radius: 3px}.button:hover{border:1px solid #09c;background-color:#0ac;color:#fff}.button:disabled,button[disabled]{border:1px solid #999;background-color:#ccc;color:#666}.arrow {margin-left: 5px;font-size: 12px;}\n";
	request->send_P(200, "text/css", css);
}

void handle_jquery(AsyncWebServerRequest *request)
{
#if defined(CONFIG_IDF_TARGET_ESP32)
	adcEn = -1;
	dacEn = -1;
	delay(100);
#endif
	AsyncWebServerResponse *response = request->beginResponse_P(200, "application/javascript", (const uint8_t *)jquery_3_7_1_min_js_gz, jquery_3_7_1_min_js_gz_len);
	response->addHeader("Content-Encoding", "gzip");
	response->addHeader("Cache-Control", "no-cache");
	response->setContentLength(jquery_3_7_1_min_js_gz_len);
	request->send(response);
#if defined(CONFIG_IDF_TARGET_ESP32)
	delay(200);
	adcEn = 1;
	dacEn = 0;
#endif
}

void handle_dashboard(AsyncWebServerRequest *request)
{
	char temp_buffer[200];
	// if (!request->authenticate(config.http_username, config.http_password))
	// {
	// 	return request->requestAuthentication();
	// }
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	// Using dynamic memory allocation instead of String
	char *webString = allocateStringMemory(4096); // Initial buffer size, adjust as needed
	if (!webString)
	{
		return; // Memory allocation failed
	}

	strcpy(webString, "<script type=\"text/javascript\">\n");
	strcat(webString, "function reloadSysInfo() {\n");
	strcat(webString, "$(\"#sysInfo\").load(\"/sysinfo\", function () { setTimeout(reloadSysInfo, 60000) });\n");
	strcat(webString, "}\n");
	strcat(webString, "setTimeout(reloadSysInfo(), 100);\n");
	strcat(webString, "function reloadSidebarInfo() {\n");
	strcat(webString, "$(\"#sidebarInfo\").load(\"/sidebarInfo\", function () { setTimeout(reloadSidebarInfo, 10000) });\n");
	strcat(webString, "}\n");
	strcat(webString, "setTimeout(reloadSidebarInfo, 1000);\n");
	strcat(webString, "$(window).trigger('resize');\n");

	strcat(webString, "</script>\n");

	//strcat(webString, "<script>\n");
	// strcat(webString, "let aprsData = [\n");
	// strcat(webString, "  { time: \"21:54:23\", icon: \"91-1.png\", callsign: \"HS5TQA-7\", path: \"RF: WIDE1-1\", dx: 0.0, packet: 2, audio: -19.6 },\n");
	// strcat(webString, "  { time: \"22:13:33\", icon: \"66-1.png\", callsign: \"HS5TQA-3\", path: \"RF: DIRECT\", dx: null, packet: 9, audio: -14.1 }\n");
	// strcat(webString, "];\n\n");
	// strcat(webString, "const tableBody = document.querySelector(\"#aprsTable tbody\");\n");
	// strcat(webString, "let sortDirection = {};\n");
	// strcat(webString, "let currentSortKey = null;\n\n");
	// strcat(webString, "function renderTable(data) {\n");
	// strcat(webString, "    tableBody.innerHTML = \"\";\n");
	// strcat(webString, "    data.forEach(row => {\n");
	// strcat(webString, "        const tr = document.createElement(\"tr\");\n");
	// strcat(webString, "        tr.innerHTML = `\n");
	// strcat(webString, "            <td>${row.time}</td>\n");
	// strcat(webString, "            <td><img src=\\\"http://aprs.dprns.com/symbols/icons/${row.icon}\\\"></td>\n");
	// strcat(webString, "            <td>${row.callsign}</td>\n");
	// strcat(webString, "            <td>${row.path}</td>\n");
	// strcat(webString, "            <td>${row.dx !== null ? row.dx + \\\" km\\\" : \\\"-\\\"}</td>\n");
	// strcat(webString, "            <td>${row.packet}</td>\n");
	// strcat(webString, "            <td style=\\\"color:green;\\\">${row.audio !== null ? row.audio + \\\" dBV\\\" : \\\"-\\\"}</td>\n");
	// strcat(webString, "        `;\n");
	// strcat(webString, "        tableBody.appendChild(tr);\n");
	// strcat(webString, "    });\n");
	// strcat(webString, "}\n\n");
	// strcat(webString, "function clearArrows() {\n");
	// strcat(webString, "    document.querySelectorAll(\\\".arrow\\\").forEach(a => a.textContent = \\\"\\\");\n");
	// strcat(webString, "}\n\n");
	// strcat(webString, "document.querySelectorAll(\\\"#aprsTable th[data-sort]\\\")\n");
	// strcat(webString, ".forEach(header => {\n\n");
	// strcat(webString, "    header.addEventListener(\\\"click\\\", () => {\n\n");
	// strcat(webString, "        const key = header.dataset.sort;\n\n");
	// strcat(webString, "        sortDirection[key] = !sortDirection[key];\n");
	// strcat(webString, "        currentSortKey = key;\n\n");
	// strcat(webString, "        aprsData.sort((a, b) => {\n\n");
	// strcat(webString, "            let valA = a[key];\n");
	// strcat(webString, "            let valB = b[key];\n\n");
	// strcat(webString, "            if (key === \\\"time\\\") {\n");
	// strcat(webString, "                valA = new Date(\\\"1970-01-01T\\\" + valA);\n");
	// strcat(webString, "                valB = new Date(\\\"1970-01-01T\\\" + valB);\n");
	// strcat(webString, "            }\n\n");
	// strcat(webString, "            if (valA === null) return 1;\n");
	// strcat(webString, "            if (valB === null) return -1;\n\n");
	// strcat(webString, "            if (valA < valB) return sortDirection[key] ? -1 : 1;\n");
	// strcat(webString, "            if (valA > valB) return sortDirection[key] ? 1 : -1;\n");
	// strcat(webString, "            return 0;\n");
	// strcat(webString, "        });\n\n");
	// strcat(webString, "        clearArrows();\n\n");
	// strcat(webString, "        const arrowSpan = header.querySelector(\\\".arrow\\\");\n");
	// strcat(webString, "        arrowSpan.textContent = sortDirection[key] ? \\\"▲\\\" : \\\"▼\\\";\n\n");
	// strcat(webString, "        renderTable(aprsData);\n");
	// strcat(webString, "    });\n\n");
	// strcat(webString, "});\n\n");
	// strcat(webString, "renderTable(aprsData);\n");
	// strcat(webString, "</script>\n");


	strcat(webString, "<div id=\"sysInfo\">\n");
	strcat(webString, "</div>\n");

	strcat(webString, "<br />\n");
	strcat(webString, "<div class=\"nav-status\">\n");
	strcat(webString, "<div id=\"sidebarInfo\">\n");
	strcat(webString, "</div>\n");
	strcat(webString, "<br />\n");

	strcat(webString, "<table>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<th colspan=\"2\">Radio Info</th>\n");
	strcat(webString, "</tr>\n");
	if (config.rf_en)
	{
		strcat(webString, "<tr>\n");
		strcat(webString, "<td>Freq.TX</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%.4f MHz</td>\n", config.freq_tx);
		strcat(webString, temp_buffer);

		strcat(webString, "</tr>\n");
		strcat(webString, "<tr>\n");
		strcat(webString, "<td>Freq.RX</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%.4f MHz</td>\n", config.freq_rx);
		strcat(webString, temp_buffer);

		strcat(webString, "</tr>\n");
		strcat(webString, "<tr>\n");
		strcat(webString, "<td>TX PWR</td>\n");
		if (config.rf_power)
			strcat(webString, "<td>HIGH</td>\n");
		else
			strcat(webString, "<td>LOW</td>\n");
		strcat(webString, "</tr>\n");
	}
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>MODEM</td>\n");

	
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", MODEM_TYPE[config.modem_type]);
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>FX.25</td>\n");

	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", FX25_MODE[config.fx25_mode]);
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "</table>\n");
	strcat(webString, "\n");
	if (config.igate_en)
	{
		strcat(webString, "<br />\n");
		strcat(webString, "<table>\n");
		strcat(webString, "<tr>\n");
		strcat(webString, "<th colspan=\"2\">APRS-IS SERVER</th>\n");
		strcat(webString, "</tr>\n");
		strcat(webString, "<tr>\n");
		strcat(webString, "<td>HOST</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", config.aprs_host);
		strcat(webString, temp_buffer);

		strcat(webString, "</tr>\n");
		strcat(webString, "<tr>\n");
		strcat(webString, "<td>PORT</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%d</td>\n", config.aprs_port);
		strcat(webString, temp_buffer);

		strcat(webString, "</tr>\n");
		strcat(webString, "</table>\n");
	}
	strcat(webString, "<br />\n");
	strcat(webString, "<table>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<th colspan=\"2\">WiFi</th>\n");
	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>MODE</td>\n");
	const char *strWiFiMode = "OFF";
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

	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", strWiFiMode);
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>SSID</td>\n");

	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", WiFi.SSID().c_str());
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>RSSI</td>\n");
	if (WiFi.isConnected())
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%d dBm</td>\n", WiFi.RSSI());
		strcat(webString, temp_buffer);
	}
	else
		strcat(webString, "<td style=\"background:#606060; color:#b0b0b0;\" aria-disabled=\"true\">Disconnect</td>\n");
	strcat(webString, "</tr>\n");
	strcat(webString, "</table>\n");
	strcat(webString, "<br />\n");
#ifdef BLUETOOTH
	strcat(webString, "<table>\n");
	strcat(webString, "<tr>\n");

	strcat(webString, "<th colspan=\"2\">Bluetooth</th>\n");
	strcat(webString, "</tr>\n");
	strcat(webString, "<td>Master</td>\n");
	if (config.bt_master)
		strcat(webString, "<td style=\"background:#0b0; color:#030; width:50%;\">Enabled</td>\n");
	else
		strcat(webString, "<td style=\"background:#606060; color:#b0b0b0;\" aria-disabled=\"true\">Disabled</td>\n");
	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>NAME</td>\n");

	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", config.bt_name);
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>MODE</td>\n");
	const char *btMode = "";
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

	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", btMode);
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "</table>\n");
#endif
	strcat(webString, "</div>\n");

	strcat(webString, "</div>\n");
	strcat(webString, "\n");
	strcat(webString, "<div class=\"content\">\n");
	strcat(webString, "<div id=\"lastHeard\">\n");
	//String lastHeardString = event_lastHeard(true);
	//strcat(webString, lastHeardString.c_str());
	//lastHeardString.clear();
	strcat(webString, "<table id=\"aprsTable\">\n<thread>\n");
	strcat(webString, "<th colspan=\"7\" style=\"background-color: #070ac2;\">LAST HEARD <a href=\"/tnc2\" target=\"_tnc2\" style=\"color: yellow;font-size:8pt\">[RAW]</a></th>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<th data-sort=\"time\" style=\"min-width:10ch\"><span><b>Time (");
	if (config.timeZone >= 0)
		strcat(webString, "+");
	// else
	//	strcat(webString, "-");

	if (config.timeZone == (int)config.timeZone)
	{
		sprintf(temp_buffer, "%d", (int)config.timeZone);
		strcat(webString, temp_buffer);
		strcat(webString, ")</b></span><span class=\"arrow\"></span></th>\n");
	}
	else
	{
		sprintf(temp_buffer, "%.1f", config.timeZone);
		strcat(webString, temp_buffer);
		strcat(webString, ")</b></span><span class=\"arrow\"></span></th>\n");
	}
	strcat(webString, "<th style=\"min-width:16px\">ICON</th>\n");
	strcat(webString, "<th data-sort=\"callsign\" style=\"min-width:10ch\">Callsign<span class=\"arrow\"></span></th>\n");
	strcat(webString, "<th>VIA LAST PATH</th>\n");
	strcat(webString, "<th data-sort=\"dx\" style=\"min-width:5ch\">DX<span class=\"arrow\"></span></th>\n");
	strcat(webString, "<th data-sort=\"packet\" style=\"min-width:5ch\">PACKET<span class=\"arrow\"></span></th>\n");
	strcat(webString, "<th data-sort=\"audio\" style=\"min-width:5ch\">AUDIO<span class=\"arrow\"></span></th>\n");
	strcat(webString, "</tr></thread>\n<tbody id=\"aprsTableBody\"></tbody>\n");
	strcat(webString, "</table>\n");
	strcat(webString, "</div>\n");

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)webString);
	response->addHeader("dashboard", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	free(webString);
	lastHeardTimeout = millis() + 500;
	lastHeard_Flag = true;
}

void handle_sidebar(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}

	// Using dynamic memory allocation instead of String
	char *html = allocateStringMemory(4096); // Initial buffer size, adjust as needed
	if (!html)
	{
		return; // Memory allocation failed
	}

	strcpy(html, "<table style=\"background:white;border-collapse: unset;\">\n");
	strcat(html, "<tr>\n");
	strcat(html, "<th colspan=\"2\">Modes Enabled</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	if (config.igate_en)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">IGATE</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">IGATE</th>\n");

	if (config.digi_en)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">DIGI</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">DIGI</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	if (config.wx_en)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">WX</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">WX</th>\n");
	if (config.trk_en)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">TRACKER</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">TRACKER</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "</table>\n");
	strcat(html, "<br />\n");
	strcat(html, "<table style=\"background:white;border-collapse: unset;\">\n");
	strcat(html, "<tr>\n");
	strcat(html, "<th colspan=\"2\">Network Status</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	if (aprsClient.connected() == true)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">APRS-IS</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">APRS-IS</th>\n");
	if (wireguard_active() == true)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">VPN</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">VPN</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
// strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">4G LTE</th>\n");
#ifdef PPPOS
	if (PPP.connected())
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">PPPoS</th>\n");
	else
#endif
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">PPPoS</th>\n");
	if (config.fx25_mode > 0)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">FX.25</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">FX.25</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "</table>\n");
	strcat(html, "<br />\n");
	strcat(html, "<table>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<th colspan=\"2\">STATISTICS</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">RADIO RX:</td>\n");

	// Convert numeric values to strings using temporary buffers
	char temp_buffer[64];
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.rxCount);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">PACKET RX:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.allCount);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">PACKET TX:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.txCount);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">RF2INET:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.rf2inet);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">INET2RF:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.inet2rf);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">DIGI:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.digiCount);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">DROP/ERR:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu/", status.dropCount);
	strcat(html, temp_buffer);
	snprintf(temp_buffer, sizeof(temp_buffer), "%lu</td>\n", status.errorCount);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "</table>\n");
	strcat(html, "<br />\n");
	if (config.gnss_enable)
	{
		strcat(html, "<table>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<th colspan=\"2\">GPS Info <a href=\"/gnss\" target=\"_gnss\" style=\"color: yellow;font-size:8pt\">[View]</a></th>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td>LAT:</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;text-align: left;\">%f</td>\n", gps.location.lat());
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td>LON:</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;text-align: left;\">%f</td>\n", gps.location.lng());
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td>ALT:</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;text-align: left;\">%f</td>\n", gps.altitude.meters());
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td>SAT:</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;text-align: left;\">%d</td>\n", gps.satellites.value());
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "</table>\n");
	}
	strcat(html, "<script>\n");
	strcat(html, "$(window).trigger('resize');\n");
	strcat(html, "</script>\n");

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
	response->addHeader("Sidebar", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	free(html); // Free the allocated memory
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

	char *web = allocateStringMemory(22000); // Initial buffer size, adjust as needed
	if (web)
	{
		memset(web, 0, 22000);
		strcat(web, "<table border=\"1\" align=\"center\">\n");
		strcat(web, "<tr><th colspan=\"16\">Table '/'</th></tr>\n");
		strcat(web, "<tr>\n");
		char lnk[200];
		for (i = 33; i < 129; i++)
		{
			memset(lnk, 0, sizeof(lnk));
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
			memset(lnk, 0, sizeof(lnk));
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
		size_t len = strlen(web);
		AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)web, len);
		response->addHeader("Symbol", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(web);
	}
}

void handle_sysinfo(AsyncWebServerRequest *request)
{
	// Using dynamic memory allocation instead of String
	char *html = allocateStringMemory(1024); // Initial buffer size, adjust as needed
	if (!html)
	{
		return; // Memory allocation failed
	}

	strcpy(html, "<table style=\"table-layout: fixed;border-collapse: unset;border-radius: 10px;border-color: #ee800a;border-style: ridge;border-spacing: 1px;border-width: 4px;background: #ee800a;\">\n");
	strcat(html, "<tr>\n");
	strcat(html, "<th><span><b>Up Time</b></span></th>\n");
	strcat(html, "<th><span>RAM(KByte)</span></th>\n");
#ifdef BOARD_HAS_PSRAM
	strcat(html, "<th><span>PSRAM(KByte)</span></th>\n");
#endif
	strcat(html, "<th><span>SPIFFS(KByte)</span></th>\n");
	if (VBat_Flag)
		strcat(html, "<th><span>VBat(V)</span></th>\n");
	strcat(html, "<th><span>CPU(Mhz)</span></th>\n");
	strcat(html, "<th><span>CPU.Temp(°C)</span></th>\n");

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	// time_t tn = time(NULL) - systemUptime;
	//  char* uptime = String(day(tn) - 1, DEC) + "D " + String(hour(tn), DEC) + ":" + String(minute(tn), DEC) + ":" + String(second(tn), DEC);
	// String uptime = String(day(tn) - 1, DEC) + "D " + String(hour(tn), DEC) + ":" + String(minute(tn), DEC);
	char strTime[20];
	convertSecondsToDHMS(strTime, (millis() / 1000) - upTimeStamp);

	char temp_buffer[512];
	snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%s</b></td>\n", strTime);
	strcat(html, temp_buffer);

	snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%.1f/%.1f</b></td>\n", (float)ESP.getFreeHeap() / 1000, (float)ESP.getHeapSize() / 1000);
	strcat(html, temp_buffer);

#ifdef BOARD_HAS_PSRAM
	snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%.1f/%.1f</b></td>\n", (float)ESP.getFreePsram() / 1000, (float)ESP.getPsramSize() / 1000);
	strcat(html, temp_buffer);
#endif

	unsigned long cardTotal = LITTLEFS.totalBytes();
	unsigned long cardUsed = LITTLEFS.usedBytes();
	snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%.1f/%.1f</b></td>\n", (double)cardUsed / 1024, (double)cardTotal / 1024);
	strcat(html, temp_buffer);

	if (VBat_Flag)
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%.2f</b></td>\n", VBat);
		strcat(html, temp_buffer);
	}

	snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%d</b></td>\n", ESP.getCpuFreqMHz());
	strcat(html, temp_buffer);

	ESPCPUTemp tempSensor;
	if (tempSensor.begin())
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%.1f</b></td>\n", tempSensor.getTemp());
		strcat(html, temp_buffer);
	}
	else
	{
		strcat(html, "<td><b>N/A</b></td>\n");
	}
	// html += "<td style=\"background: #f00\"><b>" + String(ESP.getCycleCount()) + "</b></td>\n";
	strcat(html, "</tr>\n");
	strcat(html, "</table>\n");

	// request->send(200, "text/html", html); // send to someones browser when asked
	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
	response->addHeader("Sysinfo", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	free(html); // Free the allocated memory
}

String event_lastHeard(bool gethtml)
{
	// log_d("Event count: %d",lastheard_events.count());
	// if (lastheard_events.count() == 0)
	//	return;

	struct pbuf_t aprs;
	ParseAPRS aprsParse;
	struct tm tmstruct, tmNow;

	// adcEn=-1;
	// dacEn=-1;
	// delay(20);

	// Using dynamic memory allocation instead of String
	char *html = allocateStringMemory(8192); // Initial buffer size, adjust as needed
	char *line = allocateStringMemory(1024); // Buffer for individual lines
	char temp_buffer[1024];					 // Temporary buffer for string operations
	if (!html || !line)
	{
		if (html)
			free(html);
		if (line)
			free(line);
		return ""; // Memory allocation failed
	}

	// strcpy(html, "<table id=\"aprsTable\">\n");
	// strcat(html, "<th colspan=\"7\" style=\"background-color: #070ac2;\">LAST HEARD <a href=\"/tnc2\" target=\"_tnc2\" style=\"color: yellow;font-size:8pt\">[RAW]</a></th>\n");
	// strcat(html, "<tr>\n");
	// strcat(html, "<th data-sort=\"time\" style=\"min-width:10ch\"><span><b>Time (");
	// if (config.timeZone >= 0)
	// 	strcat(html, "+");
	// // else
	// //	strcat(html, "-");

	// if (config.timeZone == (int)config.timeZone)
	// {
	// 	sprintf(temp_buffer, "%d", (int)config.timeZone);
	// 	strcat(html, temp_buffer);
	// 	strcat(html, ")</b></span></th>\n");
	// }
	// else
	// {
	// 	sprintf(temp_buffer, "%.1f", config.timeZone);
	// 	strcat(html, temp_buffer);
	// 	strcat(html, ")</b></span></th>\n");
	// }
	// strcat(html, "<th style=\"min-width:16px\">ICON</th>\n");
	// strcat(html, "<th data-sort=\"callsign\" style=\"min-width:10ch\">Callsign</th>\n");
	// strcat(html, "<th>VIA LAST PATH</th>\n");
	// strcat(html, "<th data-sort=\"dx\" style=\"min-width:5ch\">DX</th>\n");
	// strcat(html, "<th data-sort=\"packet\" style=\"min-width:5ch\">PACKET</th>\n");
	// strcat(html, "<th data-sort=\"audio\" style=\"min-width:5ch\">AUDIO</th>\n");
	// strcat(html, "</tr>\n");

	// sort(pkgList, PKGLISTSIZE);
	time_t timeNow = time(NULL);

	// log_d("Create html last heard");
	localtime_r(&timeNow, &tmNow);
//strcat(webString, "  { time: \"21:54:23\", icon: \"91-1.png\", callsign: \"HS5TQA-7\", path: \"RF: WIDE1-1\", dx: 0.0, packet: 2, audio: -19.6 },\n");
	strcpy(html, "[");
	for (int i = 0; i < PKGLISTSIZE; i++)
	{
		if (i >= PKGLISTSIZE)
			break;
		pkgListType pkg = getPkgList(i);
		if (pkg.time > 0)
		{
			snprintf(line, 1024, "%s", pkg.raw);
			// log_d("IDX=%d RAW:%s",i,line);
			int packet = pkg.pkg;
			char *pos_gt = strchr(line, '>'); // Find first position of '>'
			int start_val = pos_gt ? (pos_gt - line) : -1;
			if (start_val > 3)
			{
				// Extract src_call substring
				char src_call[64];
				strncpy(src_call, line, start_val);
				src_call[start_val] = '\0';

				memset(&aprs, 0, sizeof(pbuf_t));
				aprs.buf_len = 300;
				aprs.packet_len = strlen(line);
				strncpy((char *)aprs.data, line, aprs.packet_len < sizeof(aprs.data) ? aprs.packet_len : sizeof(aprs.data) - 1);

				char *pos_colon = strchr(line, ':');
				char *pos_comma = strchr(line, ',');
				char *pos_gt2 = strstr(line + 2, ">"); // Find '>' starting from position 2
				char *pos_dash = pos_gt2 ? strchr(pos_gt2, '-') : NULL;

				int start_info = pos_colon ? (pos_colon - line) : -1;
				int end_ssid = pos_comma ? (pos_comma - line) : -1;
				int start_dst = pos_gt2 ? (pos_gt2 - line) : -1;
				int start_dstssid = pos_dash ? (pos_dash - line) : -1;

				char path[256] = "";

				if ((end_ssid > start_dst) && (end_ssid < start_info) && (end_ssid < (int)strlen(line)))
				{
					int path_len = start_info - end_ssid - 1;
					strncpy(path, line + end_ssid + 1, path_len);
					path[path_len] = '\0';
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
					char strTime[20];
					//if (tmNow.tm_mday == tmstruct.tm_mday)
					//	sprintf(strTime, "%02d:%02d:%02d", tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
					//else
						sprintf(strTime, "%02d %02d:%02d:%02d", tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
					// String str = String(tmstruct.tm_hour, DEC) + ":" + String(tmstruct.tm_min, DEC) + ":" + String(tmstruct.tm_sec, DEC);

					// Append to html
					//strcat(html, "  { time: \"21:54:23\", icon: \"91-1.png\", callsign: \"HS5TQA-7\", path: \"RF: WIDE1-1\", dx: 0.0, packet: 2, audio: -19.6 },\n");
					char temp_html[512];
					snprintf(temp_html, sizeof(temp_html), "{\"time\":\"%s\",", strTime);
					strcat(html, temp_html);
					char fileImg[64] = "";
					uint8_t sym = (uint8_t)aprs.symbol[1];
					if (sym > 31 && sym < 127)
					{
						if (aprs.symbol[0] > 64 && aprs.symbol[0] < 91) // table A-Z
						{
							snprintf(fileImg, sizeof(fileImg), "%d", sym);
							// if (aprs.symbol[0] == 92)
							// {
							// 	strcat(fileImg, "-2.png");
							// }
							// else if (aprs.symbol[0] == 47)
							// {
								strcat(fileImg, "-1.png");
							//}

							//snprintf(temp_html, sizeof(temp_html), "<td><b>%c</b></td>", aprs.symbol[0]);
							snprintf(temp_html, sizeof(temp_html), "\"icon\":\"%s\",", fileImg);
							strcat(html, temp_html);
						}
						else
						{
							snprintf(fileImg, sizeof(fileImg), "%d", sym);
							if (aprs.symbol[0] == 92)
							{
								strcat(fileImg, "-2.png");
							}
							else if (aprs.symbol[0] == 47)
							{
								strcat(fileImg, "-1.png");
							}
							else
							{
								strcpy(fileImg, "dot.png");
							}
							//snprintf(temp_html, sizeof(temp_html), "<td><img src=\"http://aprs.dprns.com/symbols/icons/%s\"></td>", fileImg);
							snprintf(temp_html, sizeof(temp_html), "\"icon\":\"%s\",", fileImg);
							strcat(html, temp_html);
						}
					}
					else
					{
						//strcat(html, "<td><img src=\"http://aprs.dprns.com/symbols/icons/dot.png\"></td>");
						strcat(html, "\"icon\":\"dot.png\",");
					}
					
					if (aprs.srcname_len > 0 && aprs.srcname_len < 10) // Get Item/Object
					{
						char itemname[10];
						memset(&itemname, 0, 10);
						memcpy(&itemname, aprs.srcname, aprs.srcname_len);
						snprintf(temp_html, sizeof(temp_html), "\"callsign\":\"%s(%s)\",", itemname, src_call);
						strcat(html, temp_html);
					}else{
						snprintf(temp_html, sizeof(temp_html), "\"callsign\":\"%s\",", src_call);
						strcat(html, temp_html);
					}
					//strcat(html, "</td>");
					if (strlen(path) == 0)
					{
						strcat(html, "\"path\":\"RF: DIRECT\",");
					}
					else
					{
						// Find last occurrence of ','
						char *last_comma = strrchr(path, ',');
						char LPath[256] = "";
						if (last_comma != NULL)
						{
							strncpy(LPath, last_comma + 1, sizeof(LPath) - 1);
							LPath[sizeof(LPath) - 1] = '\0';
						}
						else
						{
							strncpy(LPath, path, sizeof(LPath) - 1);
							LPath[sizeof(LPath) - 1] = '\0';
						}
						// if(path.indexOf("qAR")>=0 || path.indexOf("qAS")>=0 || path.indexOf("qAC")>=0){ //Via from Internet Server
						if (strstr(path, "qA") != NULL || strstr(path, "TCPIP") != NULL)
						{
							snprintf(temp_html, sizeof(temp_html), "\"path\":\"INET:%s\",", LPath);
							strcat(html, temp_html);
						}
						else
						{
							if (strchr(path, '*') != NULL)
							{
								snprintf(temp_html, sizeof(temp_html), "\"path\":\"DIGI: %s\",", path);
								strcat(html, temp_html);
							}
							else
							{
								snprintf(temp_html, sizeof(temp_html), "\"path\":\"RF: %s\",", path);
								strcat(html, temp_html);
							}
						}
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
						snprintf(temp_html, sizeof(temp_html), "\"dx\":\"%.1fkm/%.0f°\",", dist, dtmp);
						strcat(html, temp_html);
					}
					else
					{
						strcat(html, "\"dx\":\"-\",");
					}
					snprintf(temp_html, sizeof(temp_html), "\"packet\":\"%d\",", packet);
					strcat(html, temp_html);
					if (pkg.audio_level == 0)
					{
						strcat(html, "\"audio\":\"-\"},");
					}
					else
					{
						double Vrms = (double)pkg.audio_level / 1000;
						double audBV = 20.0F * log10(Vrms);
						// if (audBV < -20.0F)
						// {
						// 	strcat(html, " audio:\"");
						// }
						// else if (audBV > -5.0F)
						// {
						// 	strcat(html, "<td style=\"color: #f00000;\">");
						// }
						// else
						// {
						// 	strcat(html, "<td style=\"color: #008000;\">");
						// }
						snprintf(temp_html, sizeof(temp_html), "\"audio\":\"%.1f\"},", audBV);
						strcat(html, temp_html);
						//strcat(html, "dBV</td></tr>\n");
					}
				}
			}
		}
	}
	html[strlen(html) - 1] = '\0'; // Remove the last comma
	strcat(html, "]");
	//strcat(html, "</table>\n");
	// log_d("HTML Length=%d Byte", strlen(html));
	// if (gethtml)
	// {
	// 	String result = String(html); // Convert back to String for return
	// 	free(html);
	// 	free(line);
	// 	return result;
	// }

	//size_t len = strlen(html);
	// char *info = (char *)calloc(len + 1, sizeof(char));
	// if (info)
	// {
	// 	strcpy(info, html);
		lastheard_events.send(html, "lastHeard", millis() / 1000, 1000);
	// 	free(info);
	// }

	free(html);
	free(line);
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

	// Using dynamic memory allocation instead of String
	char *html = allocateStringMemory(4096); // Initial buffer size, adjust as needed
	if (!html)
	{
		return String(""); // Memory allocation failed
	}

	time_t timen = time(NULL);
	localtime_r(&timen, &tmNow);

	strcpy(html, "<tr>\n");
	strcat(html, "<th style=\"width:60pt\"><span><b>Time (");
	if (config.timeZone >= 0)
		strcat(html, "+");

	// Convert timezone to string
	char temp_buffer[64];
	if (config.timeZone == (int)config.timeZone)
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "%d", (int)config.timeZone);
		strcat(html, temp_buffer);
		strcat(html, ")</b></span></th>\n");
	}
	else
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "%.1f", config.timeZone);
		strcat(html, temp_buffer);
		strcat(html, ")</b></span></th>\n");
	}
	// strcat(html, "<th style=\"min-width:16px\">ICON</th>\n");

	strcat(html, "<th style=\"width:70pt\">Callsign</th>\n");
	strcat(html, "<th>Message</th>\n");
	strcat(html, "<th style=\"width:10pt\">ACK</th>\n");
	strcat(html, "<th style=\"width:20pt\">msgID</th>\n");
	strcat(html, "</tr>\n");

	pkgMsgSort(msgQueue);
	for (int i = 0; i < PKGLISTSIZE; i++)
	{
		if (i >= PKGLISTSIZE)
			break;
		msgType pkg = getMsgList(i);
		if (pkg.time > 0)
		{
			// String line = String(pkg.text); // Not needed anymore

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
				strcat(html, "<tr style=\"background-color: #f1697dff;\">\n");
			}
			else if (pkg.ack == -1)
			{
				strcat(html, "<tr style=\"background-color: #7ff1c5ff;\">\n");
			}
			else if (pkg.ack == -2)
			{
				strcat(html, "<tr style=\"background-color: #73caf0ff;\">\n");
			}
			else
			{
				strcat(html, "<tr style=\"background-color: #f55353ff;\">\n");
			}

			strcat(html, "<td>");
			strcat(html, strTime);
			strcat(html, "</td>");

			strcat(html, "<td>");
			strcat(html, pkg.callsign);
			strcat(html, "</td>");

			strcat(html, "<td style=\"text-align: left;\">");
			strcat(html, pkg.text);
			strcat(html, "</td>");

			if (pkg.ack > 0)
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<td>%d/%d</td>", pkg.ack, config.msg_retry);
				strcat(html, temp_buffer);
			}
			else if (pkg.ack == -1)
			{
				strcat(html, "<td>RX</td>");
			}
			else if (pkg.ack == -2)
			{
				strcat(html, "<td>TX</td>");
			}
			else
			{
				strcat(html, "<td>TF</td>");
			}

			snprintf(temp_buffer, sizeof(temp_buffer), "<td>%d</td></tr>\n", pkg.msgID);
			strcat(html, temp_buffer);
		}
	}

	size_t html_len = strlen(html);
	log_d("HTML Length=%d Byte gethtml:%d event_cnt:%d", html_len, gethtml, message_events.count());

	if (gethtml)
	{
		String result = String(html); // Convert back to String for return
		free(html);					  // Free the allocated memory
		return result;
	}

	if (message_events.count() > 0)
	{
		char *info = (char *)calloc(html_len + 1, sizeof(char)); // +1 for null terminator
		if (info)
		{
			strcpy(info, html);
			message_events.send(info, "chatMsg", time(NULL), 5000);
			free(info);
		}
	}

	free(html); // Free the allocated memory
	return String("");
}

void handle_storage(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	adcEn = -1;
	dacEn = -1;
	delay(100);	

	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	const char *dirname = "/";
	char strTime[100];

	unsigned long cardTotal = LITTLEFS.totalBytes();
	unsigned long cardUsed = LITTLEFS.usedBytes();

	if (request->hasArg("delete"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "FILE")
			{
				String path = request->arg(i);
#ifdef DEBUG
				Serial.println("Deleting file: " + path);
#endif
				if (LITTLEFS.remove("/" + path))
				{
					//html = "File deleted";
#ifdef DEBUG
					Serial.println("File deleted");
#endif
				}
				else
				{
					//html = "Delete failed";
#ifdef DEBUG
					Serial.println("Delete failed");
#endif
				}
				break;
			}
		}
	}else if (request->hasArg("download"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "FILE")
			{
				String path = request->arg(i);
				String dataType = "";
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
					dataType = "plain/text";
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
				}else{
					dataType = "application/octet-stream";
					path = path.substring(0, path.lastIndexOf("."));
					//html = "File type not support";
					//request->send_P(404, PSTR("text/plain"), PSTR("File type not support"));
					//break;
				}

				if (path != "" && dataType != "")
				{
					String file = "/" + path;
					//request->send(LITTLEFS, file, dataType, true);
					AsyncWebServerResponse *response = request->beginResponse(LITTLEFS, file, dataType, true);
					//response->addHeader("Content-Disposition","attachment");
					request->send(response);
				}
				else
				{
					if (dataType != "")
						request->send_P(404, PSTR("text/plain"), PSTR("ContentType Not Support"));
					else
						request->send_P(404, PSTR("text/plain"), PSTR("File Not found"));
				}
				return;
			}
		}
	}

	// Using dynamic memory allocation instead of String
	char *webString = allocateStringMemory(8192); // Initial buffer size, adjust as needed
	if (!webString)
	{
		return; // Memory allocation failed
	}

	// strcpy(webString, "<script type=\"text/javascript\">\n");
	// 	strcat(webString, "$('form').submit(function (e) {\n");
	// 	strcat(webString, "e.preventDefault();\n");
	// 	strcat(webString, "var data = new FormData(e.currentTarget);\n");
	// 	strcat(webString, "if(e.currentTarget.id===\"formDownload\") document.getElementById(\"btnDownload\").disabled=true;\n");
	// 	strcat(webString, "if(e.currentTarget.id===\"formDelete\") document.getElementById(\"btnDelete\").disabled=true;\n");
	// 	strcat(webString, "$.ajax({\n");
	// 	strcat(webString, "url: '/storage',\n");
	// 	strcat(webString, "type: 'POST',\n");
	// 	strcat(webString, "data: data,\n");
	// 	strcat(webString, "contentType: false,\n");
	// 	strcat(webString, "processData: false,\n");
	// 	strcat(webString, "success: function (data) {\n");
	// 	strcat(webString, "$(\"#contentmain\").load(\"/storage\");\n");
	// 	//strcat(webString, "alert(\"Submited Successfully\");\n");
	// 	strcat(webString, "},\n");
	// 	strcat(webString, "error: function (data) {\n");
	// 	strcat(webString, "alert(\"An error occurred.\");\n");
	// 	strcat(webString, "}\n");
	// 	strcat(webString, "});\n");
	// 	strcat(webString, "});\n");
		//strcat(webString, "</script>\n");

		strcat(webString, "<script type=\"text/javascript\">\n"
					  "function sub(obj){"
					  "var fileName = obj.value.split('\\\\');"
					  "document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
					  "}\n"
					  //"var form = document.getElementById('upload_form');"
					  "$('form').submit(function(e){"
					  "e.preventDefault();"
					  "var data = new FormData(e.currentTarget);\n"
					  
					  "if(e.currentTarget.id === 'upload_form'){ document.getElementById('upload_sumbit').disabled = true;"
					  "var formUp = $('#upload_form')[0];"
					  "var dataUp = new FormData(formUp);"					  
					  //"document.getElementById('upload_sumbit').disabled = true;"
					  "$.ajax({"
					  "url: '/upload',"
					  "type: 'POST',"
					  "data: dataUp,"
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
					  "alert('Upload Success');"
					  "$(\"#contentmain\").load(\"/storage\");\n"
					  "},"
					  "error: function (a, b, c) {"
					  "}"
					  "});"
					  //"});"
					  "}else{"					  
					  "$.ajax({"
					  "url: '/storage',"
					  "type: 'POST',"
					  "data: data,"
					  "contentType: false,"
					  "processData:false,"					  
					  "success:function(d, s) {"
					  //"alert('Upload Success');"
					  "if(e.currentTarget.id===\"formDelete\") $(\"#contentmain\").load(\"/storage\");\n"
					  "},"
					  "error: function (a, b, c) {"
					  "}"
					  "});"
					  //"});"
					  "}"
					  "});"
					  "</script>");

	strcat(webString, "<div style=\"font-size: 8pt;text-align:left;\">");
	strcat(webString, "<b>Total space: </b>");

	char temp_buffer[512];
	if (cardTotal > 1000000)
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "%.2f MByte ,", (double)cardTotal / 1048576);
		strcat(webString, temp_buffer);
	}
	else
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "%.2f KByte ,", (double)cardTotal / 1024);
		strcat(webString, temp_buffer);
	}

	strcat(webString, "<b>Used space: </b>");
	snprintf(temp_buffer, sizeof(temp_buffer), "%.2f KByte", (double)cardUsed / 1024);
	strcat(webString, temp_buffer);

	snprintf(temp_buffer, sizeof(temp_buffer), "</br>Listing directory: </b>%s</div>\n", dirname);
	strcat(webString, temp_buffer);

	File root = LITTLEFS.open(dirname);
	if (!root)
	{
		strcat(webString, "Failed to open directory\n");
		// return;
	}
	if (!root.isDirectory())
	{
		strcat(webString, "Not a directory");
		// return;
	}

	File file = root.openNextFile();
	strcat(webString, "<table border=\"1\"><tr align=\"center\" bgcolor=\"#03DDFC\"><th width=\"100\"><b>DIRECTORY</b></th><th><b>FILE NAME</b></th><th width=\"100\"><b>SIZE(Byte)</b></th><th width=\"170\"><b>DATE TIME</b></th><th width=\"50\"><b>DELETE</b></th><th width=\"100\"><b>DOWNLOAD</b></th></tr>");
	while (file)
	{
		if (file.isDirectory())
		{
			// webString += "<tr><td>DIR : ");
			snprintf(temp_buffer, sizeof(temp_buffer), "<tr><td>%s</td>", file.name());
			strcat(webString, temp_buffer);
			time_t t = file.getLastWrite();
			struct tm *tmstruct = localtime(&t);
			sprintf(strTime, "<td></td><td></td><td align=\"right\">%d-%02d-%02d %02d:%02d:%02d</td>", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
			strcat(webString, strTime);
			// if (levels) {
			//	listDir(fs, file.name(), levels - 1);
			// }
			strcat(webString, "<td></td></tr>\n");
		}
		else
		{
			/*Serial.print("  FILE: ");
			Serial.print(file.name());*/
			// char *fName = String(file.name()).substring(1).c_str(); // Not needed for full path
			const char *fName = file.name();
			snprintf(temp_buffer, sizeof(temp_buffer), "<tr><td>/</td><td align=\"left\"><a href=\"/download?FILE=%s\" target=\"_blank\">%s</a></td>", fName, fName);
			strcat(webString, temp_buffer);
			// Serial.print("  SIZE: ");
			snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"right\">%d</td>", file.size());
			strcat(webString, temp_buffer);
			time_t t = file.getLastWrite();
			struct tm *tmstruct = localtime(&t);
			sprintf(strTime, "<td align=\"center\">%d-%02d-%02d %02d:%02d:%02d</td>", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
			strcat(webString, strTime);
			snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"center\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formDelete\" method=\"post\"><input name=\"delete\" type=\"hidden\" /><input name=\"FILE\" type=\"hidden\" value=\"%s\" /><button name=\"commit\" id=\"btnDelete\" type=\"submit\" style=\"background-color:red;color:white\">X</button></form></td>\n", fName);
			strcat(webString, temp_buffer);
			snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"center\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formDownload\" method=\"post\"><input name=\"download\" type=\"hidden\" /><input name=\"FILE\" type=\"hidden\" value=\"%s\" /><button name=\"commit\" id=\"btnDownload\" type=\"submit\" style=\"background-color:green;color:white\">DOWNLOAD</button></form></td></tr>\n", fName);
			strcat(webString, temp_buffer);
		}
		file = root.openNextFile();
	}
	strcat(webString, "</table>\n");
	// strcat(webString, "<form accept-charset=\"UTF-8\" action=\"/format\" class=\"form-horizontal\" id=\"format_form\" method=\"post\">\n");
	// strcat(webString, "<br><div><button class=\"button\" type='submit' id='format_form_sumbit'  name=\"commit\"> FORMAT </button></div>\n");
	// strcat(webString, "</form><br/>\n");

	// UPLOAD CONFIGURATION FILE
	strcat(webString, "<br><form accept-charset=\"UTF-8\" action=\"#\" "
					  "class=\"form-horizontal\" id=\"upload_form\" method=\"post\" "
					  "enctype=\"multipart/form-data\">\n");

	strcat(webString, "<table border=\"1\" style=\"margin-top:10px;width:100%;\">\n");
	strcat(webString, "<tr align=\"center\" bgcolor=\"#03DDFC\">"
					  "<th colspan=\"3\"><b>UPLOAD FILE</b></th>"
					  "</tr>\n");

	strcat(webString, "<tr align=\"center\">"
					  "<td width=\"60\" align=\"right\"><b>File:</b></td>"
					  "<td align=\"left\"><input type=\"file\" name=\"data\" required></td>"
					  "<td width=\"120\"><button class=\"button\" type='submit' id='upload_sumbit'>UPLOAD</button></td>"
					  "</tr>\n");

	strcat(webString, "</table>\n");
	strcat(webString, "</form><br/>\n");

	strcat(webString, "</body>\n</html>\n");

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)webString);
	response->addHeader("Sensor", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	free(webString); // Free the allocated memory
	adcEn = 1;
	dacEn = 0;
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
					// Compare the argument directly without converting to String
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
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
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(64); // Small buffer for "OK"
		if (html)
		{
			strcpy(html, "OK");
			request->send(200, "text/html", html); // send to someones browser when asked
			free(html);							   // Free the allocated memory
		}
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
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
					{
						hpf = true;
					}
				}
			}
			if (request->argName(i) == "LPF")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
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
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(64); // Small buffer for "OK"
		if (html)
		{
			strcpy(html, "OK");
			request->send(200, "text/html", html); // send to someones browser when asked
			free(html);							   // Free the allocated memory
		}
		saveConfiguration("/default.cfg", config);
		afskSetModem(config.modem_type, config.audio_lpf, config.tx_timeslot, config.preamble * 100, config.fx25_mode);
	}
	else
	{
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(12000); // Initial buffer size, adjust as needed
		if (!html)
		{
			return; // Memory allocation failed
		}

		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "var sliderVol = document.getElementById(\"sliderVolume\");\n");
		strcat(html, "var outputVol = document.getElementById(\"volShow\");\n");
		strcat(html, "var sliderSql = document.getElementById(\"sliderSql\");\n");
		strcat(html, "var outputSql = document.getElementById(\"sqlShow\");\n");
		strcat(html, "outputVol.innerHTML = sliderVol.value;\n");
		strcat(html, "outputSql.innerHTML = sliderSql.value;\n");
		strcat(html, "\n");
		strcat(html, "sliderVol.oninput = function () {\n");
		strcat(html, "outputVol.innerHTML = this.value;\n");
		strcat(html, "}\n");
		strcat(html, "sliderSql.oninput = function () {\n");
		strcat(html, "outputSql.innerHTML = this.value;\n");
		strcat(html, "}\n");
		strcat(html, "\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formRadio\") document.getElementById(\"submitRadio\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formTNC\") document.getElementById(\"submitTNC\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/radio',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "function rfType(){\n");
		strcat(html, "var type = document.getElementById(\"rf_type\").value;\n");
		strcat(html, "if(type==1||type==4||type==7){document.getElementById(\"tx_freq\").setAttribute(\"max\",174);document.getElementById(\"rx_freq\").setAttribute(\"max\",174);};\n");
		strcat(html, "if(type==1){document.getElementById(\"tx_freq\").setAttribute(\"min\",134);document.getElementById(\"rx_freq\").setAttribute(\"min\",134);};\n");
		strcat(html, "if(type==4||type==7){document.getElementById(\"tx_freq\").setAttribute(\"min\",136);document.getElementById(\"rx_freq\").setAttribute(\"min\",136);};\n");
		strcat(html, "if(type==2||type==5||type==8){document.getElementById(\"tx_freq\").setAttribute(\"max\",470);document.getElementById(\"rx_freq\").setAttribute(\"max\",470);};\n");
		strcat(html, "if(type==2||type==5||type==8){document.getElementById(\"tx_freq\").setAttribute(\"min\",400);document.getElementById(\"rx_freq\").setAttribute(\"min\",400);};\n");
		strcat(html, "if(type==3){document.getElementById(\"tx_freq\").setAttribute(\"min\",320);document.getElementById(\"rx_freq\").setAttribute(\"min\",320);};\n");
		strcat(html, "if(type==3){document.getElementById(\"tx_freq\").setAttribute(\"max\",400);document.getElementById(\"rx_freq\").setAttribute(\"max\",400);};\n");
		strcat(html, "if(type==6){document.getElementById(\"tx_freq\").setAttribute(\"min\",350);document.getElementById(\"rx_freq\").setAttribute(\"min\",350);};\n");
		strcat(html, "if(type==6){document.getElementById(\"tx_freq\").setAttribute(\"max\",390);document.getElementById(\"rx_freq\").setAttribute(\"max\",390);};\n");
		strcat(html, "if(type==1||type==4||type==7){document.getElementById(\"tx_freq\").setAttribute(\"value\",144.390);document.getElementById(\"rx_freq\").setAttribute(\"value\",144.390);};\n");
		strcat(html, "if(type==2||type==5||type==8){document.getElementById(\"tx_freq\").setAttribute(\"value\",432.5);document.getElementById(\"rx_freq\").setAttribute(\"value\",432.5);};\n");
		strcat(html, "\n");
		strcat(html, "}\n");
		strcat(html, "</script>\n");
		strcat(html, "<form id='formRadio' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>RF Analog Module</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");

		// Handle radio enable flag
		char temp_buffer[256];
		if (config.rf_en)
		{
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"radioEnable\" value=\"OK\" checked><span class=\"slider round\"></span></label></td>\n");
		}
		else
		{
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"radioEnable\" value=\"OK\" ><span class=\"slider round\"></span></label></td>\n");
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Module Type:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"rf_type\" id=\"rf_type\" onchange=\"rfType()\">\n");
		for (int i = 0; i < 10; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.rf_type == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%s</option>\n", RF_TYPE[i]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
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
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX Frequency:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input type=\"number\" id=\"tx_freq\" name=\"tx_freq\" min=\"%.4f\" max=\"%.4f\"\n", freqMin, freqMax);
		strcat(html, temp_buffer);
		snprintf(temp_buffer, sizeof(temp_buffer), "step=\"0.0001\" value=\"%.4f\" /> MHz</td>\n", config.freq_tx);
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RX Frequency:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input type=\"number\" id=\"rx_freq\" name=\"rx_freq\" min=\"%.4f\" max=\"%.4f\"\n", freqMin, freqMax);
		strcat(html, temp_buffer);
		snprintf(temp_buffer, sizeof(temp_buffer), "step=\"0.0001\" value=\"%.4f\" /> Mhz</td>\n", config.freq_rx);
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX CTCSS:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"tx_ctcss\" id=\"tx_ctcss\">\n");
		for (int i = 0; i < 39; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.tone_tx == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%.1f</option>\n", ctcss[i]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select> Hz\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RX CTCSS:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"rx_ctcss\" id=\"rx_ctcss\">\n");
		strcat(html, "<option value=\"0\" selected>0.0</option>\n");
		for (int i = 0; i < 39; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.tone_rx == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%.1f</option>\n", ctcss[i]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select> Hz\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Narrow/Wide:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"nw_band\" id=\"nw_band\">\n");

		if (config.band)
		{
			strcat(html, "<option value=\"0\" >12.5KHz</option>\n");
			strcat(html, "<option value=\"1\" selected>25.0KHz</option>\n");
		}
		else
		{
			strcat(html, "<option value=\"0\" selected>12.5KHz</option>\n");
			strcat(html, "<option value=\"1\" >25.0KHz</option>\n");
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX Power:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"rf_power\" id=\"rf_power\">\n");

		if (config.rf_power)
		{
			strcat(html, "<option value=\"1\" selected>HIGH</option>\n");
			strcat(html, "<option value=\"0\" >LOW</option>\n");
		}
		else
		{
			strcat(html, "<option value=\"1\" >HIGH</option>\n");
			strcat(html, "<option value=\"0\" selected>LOW</option>\n");
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>VOLUME:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"sliderVolume\" name=\"volume\" type=\"range\"\nmin=\"1\" max=\"8\" value=\"%d\" /><b><span style=\"font-size: 14pt;\" id=\"volShow\">%d</span></b></td>\n", config.volume, config.volume);
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SQL Level:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"sliderSql\" name=\"sql_level\" type=\"range\"\nmin=\"0\" max=\"8\" value=\"%d\" /><b><span style=\"font-size: 14pt;\" id=\"sqlShow\">%d</span></b></td>\n", config.sql_level, config.sql_level);
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitRadio'  name=\"commitRadio\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitRadio\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form>");

		// AFSK,TNC Configuration
		strcat(html, "<form id='formTNC' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>AFSK/TNC Configuration</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Modem Type:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"modem_type\" id=\"modem_type\" \">\n");
		for (int i = 0; i < 3; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.modem_type == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%s</option>\n", MODEM_TYPE[i]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>FX.25 Mode:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"fx25_mode\" id=\"fx25_mode\" \">\n");
		for (int i = 0; i < 3; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.fx25_mode == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%s</option>\n", FX25_MODE[i]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>  (FX.25 = AX.25 + FEC)\n");
		strcat(html, "</td>\n");
		strcat(html, "<tr>\n");
		// strcat(html, "<td align=\"right\"><b>Audio HPF:</b></td>\n");
		// char strFlag[32] = "";
		// if (config.audio_hpf)
		// 	strcpy(strFlag, "checked");
		// snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"HPF\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Audio high pass filter >1KHz cutoff 10Khz</i></label></td>\n", strFlag);
		// strcat(html, temp_buffer);
		// strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Deemphasis Audio:</b></td>\n");
		if (config.audio_lpf)
		{
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"LPF\" value=\"OK\" checked><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Audio low pass filter 1hz-2.5KHz</i></label></td>\n");
		}
		else
		{
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"LPF\" value=\"OK\" ><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Audio low pass filter 1hz-2.5KHz</i></label></td>\n");
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX Time Slot:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input type=\"number\" name=\"timeSlot\" min=\"0\" max=\"99999\"\nstep=\"100\" value=\"%d\" /> mSec.</td>\n", config.tx_timeslot);
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Preamble:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"preamble\">\n");
		for (int i = 1; i < 11; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.preamble == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%d</option>\n", i * 100);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select> mSec.\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitTNC'  name=\"commitTNC\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitTNC\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form>");
		// request->send(200, "text/html", html); // send to someones browser when asked
		//request->send_P(200, "text/html", html);

		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
		response->addHeader("Sysinfo", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(html); // Free the allocated memory
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
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						vpnEn = true;
				}
			}

			// if (request->argName(i) == "taretime") {
			//	if (request->arg(i) != "")
			//	{
			//		//if (isValidNumber(request->arg(i)))
			//		if (strcmp(request->arg(i).c_str(), "OK") == 0)
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
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(256); // Buffer for response message
		if (html)
		{
			if (saveConfiguration("/default.cfg", config))
			{
				strcpy(html, "Setup completed successfully");
				request->send(200, "text/html", html); // send to someones browser when asked
			}
			else
			{
				strcpy(html, "Save config failed.");
				request->send(501, "text/html", html); // Not Implemented
			}
			free(html); // Free the allocated memory
		}
	}
	else
	{
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(8192); // Initial buffer size, adjust as needed
		if (!html)
		{
			return; // Memory allocation failed
		}

		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formVPN\") document.getElementById(\"submitVPN\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/vpn',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");

		// Get MAC address and remove colons
		String ESP32_ID = WiFi.macAddress();
		ESP32_ID.replace(":", "");
		char temp_buffer[512];
		snprintf(temp_buffer, sizeof(temp_buffer), "function loadVPNConfig() {\nconst url = \"http://hs1.hs5tqa.ampr.org:81/wg/create\";\nconst espID = {'name': '%s'};\n", ESP32_ID.c_str());
		strcat(html, temp_buffer);
		strcat(html, "fetch(url,{\n");
		strcat(html, "method: 'POST',\n");
		strcat(html, "body: JSON.stringify(espID),\n");
		strcat(html, "headers: { 'Content-Type': 'application/json', 'Access-Control-Allow-Headers': 'Content-Type', 'Access-Control-Allow-Origin': '*','Access-Control-Allow-Methods': 'POST,GET,OPTIONS'}\n");
		strcat(html, "})\n");
		strcat(html, ".then(response => response.json())\n");
		strcat(html, ".then(data => {\n");
		strcat(html, "console.log(\"VPN Data:\", data);\n");
		strcat(html, "document.getElementById(\"wg_enable\").checked = true;\n");
		strcat(html, "document.getElementById(\"wg_peer_address\").value = data.Enpoint.split(\":\")[0];\n");
		strcat(html, "document.getElementById(\"wg_port\").value = data.Enpoint.split(\":\")[1];\n");
		strcat(html, "document.getElementById(\"wg_local_address\").value = data.Address;\n");
		strcat(html, "document.getElementById(\"wg_netmask_address\").value = \"255.255.255.0\";\n");
		strcat(html, "document.getElementById(\"wg_gw_address\").value = data.Gateway;\n");
		strcat(html, "document.getElementById(\"wg_public_key\").value = data.PublicKey;\n");
		strcat(html, "document.getElementById(\"wg_private_key\").value = data.PrivateKey;\n");
		strcat(html, "})\n");
		strcat(html, ".catch(err => console.error(\"VPN API Error:\", err));\n}\n");
		strcat(html, "</script>\n");
		// ===== JavaScript AJAX =====
		// strcat(html, "<script>\n");
		// strcat(html, "function loadVPNConfig() {\n");
		// strcat(html, "  $.ajax({\n");
		// strcat(html, "    url: '/api/vpnreq',\n");
		// strcat(html, "    method: 'GET',\n");
		// strcat(html, "    dataType: 'json',\n");
		// strcat(html, "    success: function(data) {\n");
		// strcat(html, "       console.log(data);\n");
		// strcat(html, "       let ep = data.Enpoint.split(':');\n");
		// strcat(html, "       $('#wg_peer_address').val(ep[0]);\n");
		// strcat(html, "       $('#wg_port').val(ep[1]);\n");
		// strcat(html, "       $('#wg_local_address').val(data.Address);\n");
		// strcat(html, "       $('#wg_public_key').val(data.PublicKey);\n");
		// strcat(html, "       $('#wg_private_key').val(data.PrivateKey);\n");
		// strcat(html, "    },\n");
		// strcat(html, "    error: function(e) {\n");
		// strcat(html, "       alert('โหลดข้อมูล VPN ไม่สำเร็จ');\n");
		// strcat(html, "       console.log(e);\n");
		// strcat(html, "    }\n");
		// strcat(html, "  });\n");
		// strcat(html, "}\n");
		// strcat(html, "</script>");

		// strcat(html, "<h2>System Setting</h2>\n");
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromVPN\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Wireguard Configuration</b></span></th>\n");
		strcat(html, "<tr>");

		// Handle sync flag
		if (config.vpn)
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"wg_enable\" name=\"vpnEnable\" value=\"OK\" checked><span class=\"slider round\"></span></label></td>\n");
		}
		else
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"wg_enable\" name=\"vpnEnable\" value=\"OK\" ><span class=\"slider round\"></span></label></td>\n");
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Address</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  size=\"20\" maxlength=\"32\" id=\"wg_peer_address\" name=\"wg_peer_address\" type=\"text\" value=\"%s\" /></td>\n", config.wg_peer_address);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Port</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"wg_port\" size=\"5\" name=\"wg_port\" type=\"number\" value=\"%d\" /></td>\n", config.wg_port);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Local Address</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"wg_local_address\" name=\"wg_local_address\" type=\"text\" value=\"%s\" /></td>\n", config.wg_local_address);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Netmask</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"wg_netmask_address\" name=\"wg_netmask_address\" type=\"text\" value=\"%s\" /></td>\n", config.wg_netmask_address);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Gateway</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"wg_gw_address\" name=\"wg_gw_address\" type=\"text\" value=\"%s\" /></td>\n", config.wg_gw_address);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Public Server Key</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"44\" id=\"wg_public_key\" name=\"wg_public_key\" type=\"text\" value=\"%s\" /></td>\n", config.wg_public_key);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Private Client Key</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"44\" id=\"wg_private_key\" name=\"wg_private_key\" type=\"text\" value=\"%s\" /></td>\n", config.wg_private_key);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitVPN'  name=\"commitVPN\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitVPN\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br /><br />");

		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromGetVPN\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Helper: Free VPN Wireguard for Web Service</b></span></th>\n");
		strcat(html, "<tr><td align=\"left\">1. Click New Register button to get VPN config from web service.</td></tr>\n");
		strcat(html, "<tr><td align=\"left\">2. The VPN config will fill in the form automatically.</td></tr>\n");
		strcat(html, "<tr><td align=\"left\">3. Click Apply Change and reboot again.</td></tr>\n");
		strcat(html, "<tr><td align=\"left\">4. Enjoy your free VPN service!</td></tr>\n");

		// Check if local address starts with "10.44."
		String wg_local_addr = String(config.wg_local_address);
		if (wg_local_addr.startsWith("10.44."))
		{
			int lastoct = wg_local_addr.substring(wg_local_addr.lastIndexOf('.') + 1).toInt();
			// String url="http://"+String(config.wg_peer_address)+":"+String(8000+lastoct);
			// strcat(html, "<tr><td>Your External Host IP: <a href=\"");
			// snprintf(temp_buffer, sizeof(temp_buffer), "%s\">%s</a></td></tr>\n", url.c_str(), url.c_str());
			// strcat(html, temp_buffer);
			int thirdoct = wg_local_addr.substring(wg_local_addr.indexOf('.', wg_local_addr.indexOf('.') + 1) + 1, wg_local_addr.lastIndexOf('.')).toInt();
			String url_base = "http://hs" + String(thirdoct) + ".hs5tqa.ampr.org:" + String((thirdoct * 10000) + 8000 + lastoct);
			snprintf(temp_buffer, sizeof(temp_buffer), "<tr><td>Your External by AMPR URL: <a href=\"%s\" target=\"_blank\">%s</a></td></tr>\n", url_base.c_str(), url_base.c_str());
			strcat(html, temp_buffer);
			url_base = "http://vpn.nakhonthai.net:" + String((thirdoct * 10000) + 8000 + lastoct);
			snprintf(temp_buffer, sizeof(temp_buffer), "<tr><td>Fast Direct URL: <a href=\"%s\" target=\"_blank\">%s</a></td></tr>\n", url_base.c_str(), url_base.c_str());
			strcat(html, temp_buffer);
		}
		strcat(html, "<tr><td><button type=\"button\" onclick=\"loadVPNConfig()\">New Register</button></td></tr>\n");
		strcat(html, "</table><br />\n");
		strcat(html, "</form>");

		// request->send(200, "text/html", html); // send to someones browser when asked
		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
		response->addHeader("VPN", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(html); // Free the allocated memory
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
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
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
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_TOPIC_TNC;
				}
			}
			if (request->argName(i) == "TopicSts")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_TOPIC_STATUS;
				}
			}
			if (request->argName(i) == "TopicTlm")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_TOPIC_TELEMETRY;
				}
			}
			if (request->argName(i) == "TopicWX")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_TOPIC_WX;
				}
			}
			if (request->argName(i) == "TopicSensor")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_TOPIC_SENSOR;
				}
			}

			if (request->argName(i) == "subCMD")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_SUBSCRIBE_CMD;
				}
			}
			if (request->argName(i) == "subTNC")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_SUBSCRIBE_TNC;
				}
			}
			if (request->argName(i) == "subMsg")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_SUBSCRIBE_MESSAGE;
				}
			}
		}

		config.en_mqtt = mqttEn;
		clientMQTT.disconnect();
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(256); // Buffer for response message
		if (html)
		{
			if (saveConfiguration("/default.cfg", config))
			{
				strcpy(html, "Setup completed successfully");
				request->send(200, "text/html", html); // send to someones browser when asked
			}
			else
			{
				strcpy(html, "Save config failed.");
				request->send(501, "text/html", html); // Not Implemented
			}
			free(html); // Free the allocated memory
		}
	}
	else
	{
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(8192); // Initial buffer size, adjust as needed
		if (!html)
		{
			return; // Memory allocation failed
		}

		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formVPN\") document.getElementById(\"submitMQTT\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/mqtt',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");

		// strcat(html, "<h2>System Setting</h2>\n");
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromMQTT\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>MQTT Configuration</b></span></th>\n");
		strcat(html, "<tr>");

		// Handle sync flag
		if (config.en_mqtt)
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"enable\" value=\"OK\" checked><span class=\"slider round\"></span></label></td>\n");
		}
		else
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"enable\" value=\"OK\" ><span class=\"slider round\"></span></label></td>\n");
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Address:</b></td>\n");
		char temp_buffer[512];
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  size=\"30\" maxlength=\"32\" name=\"host\" type=\"text\" value=\"%s\" /></td>\n", config.mqtt_host);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Port:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"5\"  maxlength=\"5\"  name=\"port\" type=\"number\" value=\"%d\" /></td>\n", config.mqtt_port);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>User:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input maxlength=\"32\" name=\"user\" type=\"text\" value=\"%s\" /></td>\n", config.mqtt_user);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Password:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"40\" maxlength=\"63\" name=\"pass\" type=\"password\" value=\"%s\" /></td>\n", config.mqtt_pass);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Topic:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"32\" id=\"topic\" name=\"topic\" type=\"text\" value=\"%s\" /></td>\n", config.mqtt_topic);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Topic Flag:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<fieldset id=\"TopicGrp\">\n");
		strcat(html, "<legend>Topic Flags Send out MQTT</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">");
		strcat(html, "<tr style=\"background:unset;\">");

		// Handle topic flags
		if (config.mqtt_topic_flag & MQTT_TOPIC_TNC)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicTNC\" type=\"checkbox\" value=\"OK\" checked/>TNC</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicTNC\" type=\"checkbox\" value=\"OK\" />TNC</td>\n");
		}

		if (config.mqtt_topic_flag & MQTT_TOPIC_STATUS)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicSts\" type=\"checkbox\" value=\"OK\" checked/>Status</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicSts\" type=\"checkbox\" value=\"OK\" />Status</td>\n");
		}

		if (config.mqtt_topic_flag & MQTT_TOPIC_TELEMETRY)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicTlm\" type=\"checkbox\" value=\"OK\" checked/>Telemetry</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicTlm\" type=\"checkbox\" value=\"OK\" />Telemetry</td>\n");
		}

		if (config.mqtt_topic_flag & MQTT_TOPIC_WX)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicWX\" type=\"checkbox\" value=\"OK\" checked/>Weather</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicWX\" type=\"checkbox\" value=\"OK\" />Weather</td>\n");
		}

		if (config.mqtt_topic_flag & MQTT_TOPIC_SENSOR)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicSensor\" type=\"checkbox\" value=\"OK\" checked/>Sensor</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicSensor\" type=\"checkbox\" value=\"OK\" />Sensor</td>\n");
		}

		strcat(html, "<td style=\"border:unset;\"></td>");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Subscription:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"32\" name=\"subscribe\" type=\"text\" value=\"%s\" /></td>\n", config.mqtt_subscribe);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Subscription Flag:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<fieldset id=\"SubGrp\">\n");
		strcat(html, "<legend>Subscription Flags Receive</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">");
		strcat(html, "<tr style=\"background:unset;\">");

		// Handle subscription flags
		if (config.mqtt_subscribe_flag & MQTT_SUBSCRIBE_CMD)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subCMD\" type=\"checkbox\" value=\"OK\" checked/>AT-Command</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subCMD\" type=\"checkbox\" value=\"OK\" />AT-Command</td>\n");
		}

		if (config.mqtt_subscribe_flag & MQTT_SUBSCRIBE_TNC)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subTNC\" type=\"checkbox\" value=\"OK\" checked/>TNC</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subTNC\" type=\"checkbox\" value=\"OK\" />TNC</td>\n");
		}

		if (config.mqtt_subscribe_flag & MQTT_SUBSCRIBE_MESSAGE)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subMsg\" type=\"checkbox\" value=\"OK\" checked/>Message</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subMsg\" type=\"checkbox\" value=\"OK\" />Message</td>\n");
		}

		strcat(html, "<td style=\"border:unset;\"></td>");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "</table><br />\n");
		strcat(html, "<td><input class=\"button\" id=\"submitMQTT\" name=\"commitMQTT\" type=\"submit\" value=\"Save Config\" maxlength=\"80\"/></td>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitMQTT\"/>\n");
		strcat(html, "</form>\n");

		// request->send(200, "text/html", html); // send to someones browser when asked
		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
		response->addHeader("MQTT", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(html); // Free the allocated memory
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
		// Using char arrays instead of String
		char toCall[10];
		char msg[256];
		memset(toCall, 0, sizeof(toCall));
		memset(msg, 0, sizeof(msg));

		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "toCall")
			{
				if (request->arg(i) != "")
				{
					strncpy(toCall, request->arg(i).c_str(), sizeof(toCall) - 1);
				}
			}
			if (request->argName(i) == "msg")
			{
				if (request->arg(i) != "")
				{
					strncpy(msg, request->arg(i).c_str(), sizeof(msg) - 1);
				}
			}
		}
		log_d("Chat to %s | msg %s", toCall, msg);
		sendAPRSMessage(String(toCall), String(msg), config.msg_encrypt);
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(64); // Small buffer for "Send completed"
		if (html)
		{
			strcpy(html, "Send completed");
			request->send(200, "text/html", html); // send to someones browser when asked
			free(html);							   // Free the allocated memory
		}
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
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						msgEn = true;
				}
			}
			if (request->argName(i) == "msgRf")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						msgRf = true;
				}
			}
			if (request->argName(i) == "msgInet")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						msgInet = true;
				}
			}
			if (request->argName(i) == "encrypt")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
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

		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(256); // Buffer for response message
		if (html)
		{
			if (saveConfiguration("/default.cfg", config))
			{
				strcpy(html, "Setup completed successfully");
				request->send(200, "text/html", html); // send to someones browser when asked
			}
			else
			{
				strcpy(html, "Save config failed.");
				request->send(501, "text/html", html); // Not Implemented
			}
			free(html); // Free the allocated memory
		}
	}
	else
	{
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(8192); // Initial buffer size, adjust as needed
		if (!html)
		{
			return; // Memory allocation failed
		}

		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formMSG\") document.getElementById(\"submitMSG\").disabled=true;\n");
		// strcat(html, "if(e.currentTarget.id===\"formChat\") document.getElementById(\"submitI2C0\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/msg',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "if(e.currentTarget.id===\"formMSG\") alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "if(e.currentTarget.id===\"formMSG\") alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");

		// strcat(html, "if (!!window.EventSource) {";
		// strcat(html, "var source = new EventSource('/eventMsg');");

		// strcat(html, "source.addEventListener('open', function(e) {";
		// strcat(html, "console.log(\"Events MSG Connected\");";
		// strcat(html, "}, false);";
		// strcat(html, "source.addEventListener('error', function(e) {";
		// strcat(html, "if (e.target.readyState != EventSource.OPEN) {";
		// strcat(html, "console.log(\"Events MSG Disconnected\");";
		// strcat(html, "}\n}, false);";
		// strcat(html, "source.addEventListener('chatMsg', function(e) {";
		// // strcat(html, "console.log(\"lastHeard\", e.data);";
		// strcat(html, "var lh=document.getElementById(\"chatMsg\");";
		// strcat(html, "if(lh != null) {lh.innerHTML = e.data;}";
		// strcat(html, "}, false);\n}";
		strcat(html, "</script>\n");

		// strcat(html, "<h2>System Setting</h2>\n");
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formMSG\" method=\"post\">\n");
		strcat(html, "<table width=\"90%\">\n");
		strcat(html, "<th colspan=\"2\"><span><b>Message Configuration</b></span></th>\n");
		strcat(html, "<tr>");

		// Handle sync flag
		if (config.msg_enable)
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"enable\" value=\"OK\" checked><span class=\"slider round\"></span></label></td>\n");
		}
		else
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"enable\" value=\"OK\" ><span class=\"slider round\"></span></label></td>\n");
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>My Callsign:</b></td>\n");
		char temp_buffer[512];
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  size=\"20\" maxlength=\"9\" name=\"mycall\" type=\"text\" value=\"%s\" /> *<i>Callsign with SSID (Ex. HS5TQA-12)</i></td>\n", config.msg_mycall);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		// Handle RF and Internet flags
		if (config.msg_rf && config.msg_inet)
		{
			strcat(html, "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"msgRf\" value=\"OK\" checked/>RF <input type=\"checkbox\" name=\"msgInet\" value=\"OK\" checked/>Internet </td></tr>\n");
		}
		else if (config.msg_rf)
		{
			strcat(html, "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"msgRf\" value=\"OK\" checked/>RF <input type=\"checkbox\" name=\"msgInet\" value=\"OK\" />Internet </td></tr>\n");
		}
		else if (config.msg_inet)
		{
			strcat(html, "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"msgRf\" value=\"OK\" />RF <input type=\"checkbox\" name=\"msgInet\" value=\"OK\" checked/>Internet </td></tr>\n");
		}
		else
		{
			strcat(html, "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"msgRf\" value=\"OK\" />RF <input type=\"checkbox\" name=\"msgInet\" value=\"OK\" />Internet </td></tr>\n");
		}

		strcat(html, "<tr>");
		if (config.msg_encrypt)
		{
			strcat(html, "<td align=\"right\"><b>Encryption</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"encrypt\" value=\"OK\" checked><span class=\"slider round\"></span></label></td>\n");
		}
		else
		{
			strcat(html, "<td align=\"right\"><b>Encryption</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"encrypt\" value=\"OK\" ><span class=\"slider round\"></span></label></td>\n");
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>AES Key:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  size=\"40\" maxlength=\"33\" name=\"key\" type=\"text\" value=\"%s\" /> *<i>ASCII HEX 16Byte</i></td>\n", config.msg_key);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Send Retry:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  min=\"0\" max=\"99\"   name=\"retry\" type=\"number\" value=\"%d\" /></td>\n", config.msg_retry);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Send Timeout:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  min=\"1\" max=\"9999\"   name=\"timeout\" type=\"number\" value=\"%d\" /> Sec.</td>\n", config.msg_interval);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"path\" id=\"path\">\n");
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", pthIdx);
			strcat(html, temp_buffer);
			if (config.msg_path == pthIdx)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%s</option>\n", PATH_NAME[pthIdx]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select></td>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitMSG'  name=\"commitMSG\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitMSG\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br /><br />");

		strcat(html, "<table width=\"90%\">\n");
		strcat(html, "<th style=\"background-color: #070ac2;\">CHAT MESSAGE</th>\n");

		strcat(html, "<tr><td>\n");
		strcat(html, "<table id=\"chatMsg\">\n");
		strcat(html, event_chatMessage(true).c_str());
		strcat(html, "</table>\n");

		strcat(html, "</td></tr><tr><td colspan=\"5\">");

		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formChat\" method=\"post\">\n");
		strcat(html, "<table>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"left\"><b>TO:</b><input size=\"10\" name=\"toCall\" id=\"toCall\" type=\"text\" value=\"\" oninput=\"this.value=this.value.toUpperCase();\" /> <b>MSG:</b><input size=\"80\" name=\"msg\" id=\"msg\" type=\"text\" value=\"\" /></td>\n");
		strcat(html, "<td align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitChat\" name=\"commitChat\" type=\"submit\" value=\"Send\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitChat\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form><br />\n");

		strcat(html, "</td></tr></table>");

		// request->send(200, "text/html", html); // send to someones browser when asked
		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
		response->addHeader("MSG", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(html); // Free the allocated memory
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
	}
	else if (request->hasArg("commitCMD"))
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
		bool pppNapt = false;
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

			if (request->argName(i) == "pppNapt")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						pppNapt = true;
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
		config.ppp_napt = pppNapt;
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
		char *html = allocateStringMemory(256); // Small buffer for success/failure message
		if (html != NULL)
		{
			if (saveConfiguration("/default.cfg", config))
			{
				strcpy(html, "Setup completed successfully");
				request->send(200, "text/html", html); // send to someones browser when asked
			}
			else
			{
				strcpy(html, "Save config failed.");
				request->send(501, "text/html", html); // Not Implemented
			}
			free(html);
		}
	}
#endif
	else
	{
		// Allocate memory for the HTML string
		char *html = allocateStringMemory(22000); // Start with 8KB, adjust as needed
		if (html == NULL)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}

		// Initialize the HTML string with the JavaScript code
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formUART0\") document.getElementById(\"submitURAT0\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formUART1\") document.getElementById(\"submitURAT1\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formUART1\") document.getElementById(\"submitURAT1\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formGNSS\") document.getElementById(\"submitGNSS\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formMODBUS\") document.getElementById(\"submitMODBUS\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formTNC\") document.getElementById(\"submitTNC\").disabled=true;\n");
		// strcat(html, "if(e.currentTarget.id===\"formONEWIRE\") document.getElementById(\"submitONEWIRE\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formRF\") document.getElementById(\"submitRF\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formI2C0\") document.getElementById(\"submitI2C0\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formI2C1\") document.getElementById(\"submitI2C1\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formCOUNT0\") document.getElementById(\"submitCOUNT0\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formCOUNT1\") document.getElementById(\"submitCOUNT1\").disabled=true;\n");
#ifdef PPPOS
		strcat(html, "if(e.currentTarget.id===\"formPPPoS\") document.getElementById(\"submitPPPoS\").disabled=true;\n");
#endif
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/mod',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\\nRequire hardware RESET!\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");

		strcat(html, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"32%\" style=\"border:unset;\">\n");
		// strcat(html, "<h2>System Setting</h2>\n");
		/**************UART0(USB) Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromUART0\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>UART0 Modify</b></span></th>\n");
		strcat(html, "<tr>\n");

		char enFlage[20] = "";
		if (config.uart0_enable)
			strcpy(enFlage, "checked");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(enFlage) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", enFlage);
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rx_gpio_str = StringToCharPtr(String(config.uart0_rx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"rx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"rx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rx_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *tx_gpio_str = StringToCharPtr(String(config.uart0_tx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"tx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(tx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"tx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, tx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(tx_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RTS/DE GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rts_gpio_str = StringToCharPtr(String(config.uart0_rts_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"rts\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rts_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"rts\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rts_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rts_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Baudrate:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"baudrate\" id=\"baudrate\">\n");
		for (int i = 0; i < 13; i++)
		{
			char *baudrate_str = StringToCharPtr(String(baudrate[i]));
			if (config.uart0_baudrate == baudrate[i])
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" selected>  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" selected>%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			else
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" >  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" >%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			free(baudrate_str);
		}
		strcat(html, "</select> bps\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitUART0\" name=\"commitUART0\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitUART0\"/>\n");
		strcat(html, "</td></tr></table>\n");

		strcat(html, "</form><br />\n");
		strcat(html, "</td><td width=\"32%\" style=\"border:unset;\">");

		/**************UART1 Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromUART1\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>UART1 Modify</b></span></th>\n");
		strcat(html, "<tr>");

		if (config.uart1_enable)
			strcpy(enFlage, "checked");
		else
			strcpy(enFlage, "");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(enFlage) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", enFlage);
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rx_gpio_str = StringToCharPtr(String(config.uart1_rx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"rx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"rx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rx_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *tx_gpio_str = StringToCharPtr(String(config.uart1_tx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"tx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(tx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"tx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, tx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(tx_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RTS/DE GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rts_gpio_str = StringToCharPtr(String(config.uart1_rts_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"rts\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rts_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"rts\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rts_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rts_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Baudrate:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"baudrate\" id=\"baudrate\">\n");
		for (int i = 0; i < 13; i++)
		{
			char *baudrate_str = StringToCharPtr(String(baudrate[i]));
			if (config.uart1_baudrate == baudrate[i])
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" selected>  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" selected>%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			else
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" >  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" >%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			free(baudrate_str);
		}
		strcat(html, "</select> bps\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitUART1\" name=\"commitUART1\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitUART1\"/>\n");
		strcat(html, "</td></tr></table>\n");

		strcat(html, "</form><br />\n");
		// strcat(html, "</td><td width=\"32%\" style=\"border:unset;\">");

		/**************UART2 Modify******************/
		// strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromUART2\" method=\"post\">\n");
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

		strcat(html, "</td><td width=\"32%\" style=\"border:unset;\">");

		/**************1-Wire Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromONEWIRE\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>1-Wire Bus Modify</b></span></th>\n");
		strcat(html, "<tr>");

		String syncFlage = "";
		if (config.onewire_enable)
			syncFlage = "checked";
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(syncFlage.c_str()) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", syncFlage.c_str());
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *onewire_gpio_str = StringToCharPtr(String(config.onewire_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"data\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(onewire_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"data\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, onewire_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(onewire_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitONEWIRE\" name=\"commitONEWIRE\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitONEWIRE\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form><br />\n");

		strcat(html, "</td></tr></table>\n");

		strcat(html, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">");
		/**************RF GPIO******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromRF\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>RF GPIO Modify</b></span></th>\n");
		strcat(html, "<tr>");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>ADC Attenuation:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"atten\" id=\"atten\">\n");
		for (int i = 0; i < 5; i++)
		{
			char *i_str = StringToCharPtr(String(i));
			char *atten_str = StringToCharPtr(String(ADC_ATTEN[i]));
			if (config.adc_atten == i)
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" selected>  </option>\n") + strlen(i_str) + strlen(atten_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" selected>%s </option>\n", i_str, atten_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			else
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" >  </option>\n") + strlen(i_str) + strlen(atten_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" >%s </option>\n", i_str, atten_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			free(i_str);
			free(atten_str);
		}
		{
			char *offset_str = StringToCharPtr(String(offset));
			char *temp_html = allocateStringMemory(strlen("</select> DC-Offset:  mV\n") + strlen(offset_str) + 1);
			sprintf(temp_html, "</select> DC-Offset: %s mV\n", offset_str);
			strcat(html, temp_html);
			free(temp_html);
			free(offset_str);
		}
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UART2 Baudrate:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"baudrate\" id=\"baudrate\">\n");
		for (int i = 0; i < 13; i++)
		{
			char *baudrate_str = StringToCharPtr(String(baudrate[i]));
			if (config.rf_baudrate == baudrate[i])
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" selected>  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" selected>%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			else
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" >  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" >%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			free(baudrate_str);
		}
		strcat(html, "</select> bps\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>ADC DC OFFSET:</b></td>\n";
		// html += "<td style=\"text-align: left;\"><input min=\"100\" max=\"2500\" name=\"offset\" type=\"number\" value=\"" + String(config.adc_dc_offset) + "\" /> mV     (Current: " + String(offset) + " mV)</td>\n";
		// strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UART2 RX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_rx_gpio_str = StringToCharPtr(String(config.rf_rx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"rx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rf_rx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"rx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rf_rx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_rx_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UART2 TX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_tx_gpio_str = StringToCharPtr(String(config.rf_tx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"tx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rf_tx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"tx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rf_tx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_tx_gpio_str);
		}
		strcat(html, "</tr>\n");

		char LowFlag[20] = "", HighFlag[20] = "";
		strcpy(LowFlag, "");
		strcpy(HighFlag, "");
		if (config.rf_pd_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PD GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_pd_gpio_str = StringToCharPtr(String(config.rf_pd_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"pd\" type=\"number\" value=\"\" /> Active:<input type=\"radio\" name=\"pd_active\" value=\"0\"  />LOW <input type=\"radio\" name=\"pd_active\" value=\"1\"  />HIGH </td>\n") + strlen(gpio_max_str) + strlen(rf_pd_gpio_str) + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"pd\" type=\"number\" value=\"%s\" /> Active:<input type=\"radio\" name=\"pd_active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"pd_active\" value=\"1\" %s/>HIGH </td>\n", gpio_max_str, rf_pd_gpio_str, LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_pd_gpio_str);
		}
		strcat(html, "</tr>\n");

		sprintf(LowFlag, "");
		sprintf(HighFlag, "");
		if (config.rf_pwr_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>H/L GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_pwr_gpio_str = StringToCharPtr(String(config.rf_pwr_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"pwr\" type=\"number\" value=\"\" /> Active:<input type=\"radio\" name=\"pwr_active\" value=\"0\"  />LOW <input type=\"radio\" name=\"pwr_active\" value=\"1\"  />HIGH </td>\n") + strlen(gpio_max_str) + strlen(rf_pwr_gpio_str) + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"pwr\" type=\"number\" value=\"%s\" /> Active:<input type=\"radio\" name=\"pwr_active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"pwr_active\" value=\"1\" %s/>HIGH </td>\n", gpio_max_str, rf_pwr_gpio_str, LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_pwr_gpio_str);
		}
		strcat(html, "</tr>\n");

		sprintf(LowFlag, "");
		sprintf(HighFlag, "");
		if (config.rf_sql_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SQL GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_sql_gpio_str = StringToCharPtr(String(config.rf_sql_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"sql\" type=\"number\" value=\"\" /> Active:<input type=\"radio\" name=\"sql_active\" value=\"0\"  />LOW <input type=\"radio\" name=\"sql_active\" value=\"1\"  />HIGH </td>\n") + strlen(gpio_max_str) + strlen(rf_sql_gpio_str) + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"sql\" type=\"number\" value=\"%s\" /> Active:<input type=\"radio\" name=\"sql_active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"sql_active\" value=\"1\" %s/>HIGH </td>\n", gpio_max_str, rf_sql_gpio_str, LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_sql_gpio_str);
		}
		strcat(html, "</tr>\n");

		sprintf(LowFlag, "");
		sprintf(HighFlag, "");
		if (config.rf_ptt_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PTT GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_ptt_gpio_str = StringToCharPtr(String(config.rf_ptt_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"ptt\" type=\"number\" value=\"\" /> Active:<input type=\"radio\" name=\"ptt_active\" value=\"0\"  />LOW <input type=\"radio\" name=\"ptt_active\" value=\"1\"  />HIGH </td>\n") + strlen(gpio_max_str) + strlen(rf_ptt_gpio_str) + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"ptt\" type=\"number\" value=\"%s\" /> Active:<input type=\"radio\" name=\"ptt_active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"ptt_active\" value=\"1\" %s/>HIGH </td>\n", gpio_max_str, rf_ptt_gpio_str, LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_ptt_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitRF\" name=\"commitRF\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitRF\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		strcat(html, "</td><td width=\"23%\" style=\"border:unset;\">");

		/**************I2C_0 Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromI2C0\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>I2C_0(OLED) Modify</b></span></th>\n");
		strcat(html, "<tr>");

		syncFlage = "";
		if (config.i2c_enable)
			syncFlage = "checked";
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(syncFlage.c_str()) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", syncFlage.c_str());
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SDA GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *i2c_sda_pin_str = StringToCharPtr(String(config.i2c_sda_pin));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"sda\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(i2c_sda_pin_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"sda\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, i2c_sda_pin_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(i2c_sda_pin_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SCK GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *i2c_sck_pin_str = StringToCharPtr(String(config.i2c_sck_pin));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"sck\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(i2c_sck_pin_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"sck\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, i2c_sck_pin_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(i2c_sck_pin_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Frequency:</b></td>\n");
		{
			char *freq_str = StringToCharPtr(String(config.i2c_freq));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"1000\" max=\"800000\" name=\"freq\" type=\"number\" value=\"\" /></td>\n") + strlen(freq_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"1000\" max=\"800000\" name=\"freq\" type=\"number\" value=\"%s\" /></td>\n", freq_str);
			strcat(html, temp_html);
			free(temp_html);
			free(freq_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitI2C0\" name=\"commitI2C0\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitI2C0\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		/**************Counter_0 Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromCOUNTER0\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Counter_0 Modify</b></span></th>\n");
		strcat(html, "<tr>");

		syncFlage = "";
		if (config.counter0_enable)
			syncFlage = "checked";
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(syncFlage.c_str()) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", syncFlage.c_str());
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>INPUT GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *counter0_gpio_str = StringToCharPtr(String(config.counter0_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"gpio\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(counter0_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"gpio\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, counter0_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(counter0_gpio_str);
		}
		strcat(html, "</tr>\n");

		sprintf(LowFlag, "");
		sprintf(HighFlag, "");
		if (config.counter0_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Active</td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input type=\"radio\" name=\"active\" value=\"0\"  />LOW <input type=\"radio\" name=\"active\" value=\"1\"  />HIGH </td>\n") + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input type=\"radio\" name=\"active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"active\" value=\"1\" %s/>HIGH </td>\n", LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitCOUNTER0\" name=\"commitCOUNTER0\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitCOUNTER0\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		strcat(html, "</td><td width=\"23%\" style=\"border:unset;\">");
		/**************I2C_1 Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromI2C1\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>I2C_1 Modify</b></span></th>\n");
		strcat(html, "<tr>");

		syncFlage = "";
		if (config.i2c1_enable)
			syncFlage = "checked";
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(syncFlage.c_str()) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", syncFlage.c_str());
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SDA GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *i2c1_sda_pin_str = StringToCharPtr(String(config.i2c1_sda_pin));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"sda\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(i2c1_sda_pin_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"sda\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, i2c1_sda_pin_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(i2c1_sda_pin_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SCK GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *i2c1_sck_pin_str = StringToCharPtr(String(config.i2c1_sck_pin));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"sck\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(i2c1_sck_pin_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"sck\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, i2c1_sck_pin_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(i2c1_sck_pin_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Frequency:</b></td>\n");
		{
			char *freq_str = StringToCharPtr(String(config.i2c1_freq));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"1000\" max=\"800000\" name=\"freq\" type=\"number\" value=\"\" /></td>\n") + strlen(freq_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"1000\" max=\"800000\" name=\"freq\" type=\"number\" value=\"%s\" /></td>\n", freq_str);
			strcat(html, temp_html);
			free(temp_html);
			free(freq_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitI2C1\" name=\"commitI2C1\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitI2C1\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		/**************Counter_1 Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromCOUNTER1\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Counter_1 Modify</b></span></th>\n");
		strcat(html, "<tr>");

		syncFlage = "";
		if (config.counter1_enable)
			syncFlage = "checked";
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(syncFlage.c_str()) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", syncFlage.c_str());
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>INPUT GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *counter1_gpio_str = StringToCharPtr(String(config.counter1_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"gpio\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(counter1_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"gpio\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, counter1_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(counter1_gpio_str);
		}
		strcat(html, "</tr>\n");

		sprintf(LowFlag, "");
		sprintf(HighFlag, "");
		if (config.counter1_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Active</td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input type=\"radio\" name=\"active\" value=\"0\"  />LOW <input type=\"radio\" name=\"active\" value=\"1\"  />HIGH </td>\n") + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input type=\"radio\" name=\"active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"active\" value=\"1\" %s/>HIGH </td>\n", LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitCOUNTER1\" name=\"commitCOUNTER1\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitCOUNTER1\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		strcat(html, "</td></tr></table>\n");
		strcat(html, "<br />\n");

		//******************
		strcat(html, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">");
		/**************GNSS Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromGNSS\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>GNSS Modify</b></span></th>\n");
		strcat(html, "<tr>\n");

		strcpy(enFlage, "");
		if (config.gnss_enable)
			strcpy(enFlage, "checked");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ");
		strcat(html, enFlage);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PORT:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"channel\" id=\"channel\">\n");
		for (int i = 0; i < 5; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr2 = intToString(i);
			strcat(html, iStr2);
			strcat(html, "\" ");
			if (config.gnss_channel == i)
			{
				strcat(html, "selected>");
				strcat(html, GNSS_PORT[i]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, GNSS_PORT[i]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr2);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<td align=\"right\"><b>AT Command:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"30\" size=\"20\" id=\"atc\" name=\"atc\" type=\"text\" value=\"");
		strcat(html, config.gnss_at_command);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<td align=\"right\"><b>TCP Host:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"20\" size=\"15\" id=\"Host\" name=\"Host\" type=\"text\" value=\"");
		strcat(html, config.gnss_tcp_host);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TCP Port:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"1024\" max=\"65535\"  id=\"Port\" name=\"Port\" type=\"number\" value=\"");
		char *gnssTcpPortStr = intToString(config.gnss_tcp_port);
		strcat(html, gnssTcpPortStr);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitGNSS\" name=\"commitGNSS\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitGNSS\"/>\n");
		strcat(html, "</td></tr></table>\n");

		strcat(html, "</form><br />\n");

		strcat(html, "</td><td width=\"23%\" style=\"border:unset;\">\n");

		/**************MODBUS Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromMODBUS\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>MODBUS Modify</b></span></th>\n");
		strcat(html, "<tr>\n");

		strcpy(enFlage, "");
		if (config.modbus_enable)
			strcpy(enFlage, "checked");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ");
		strcat(html, enFlage);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PORT:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"channel\" id=\"channel\">\n");
		for (int i = 0; i < 5; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr3 = intToString(i);
			strcat(html, iStr3);
			strcat(html, "\" ");
			if (config.modbus_channel == i)
			{
				strcat(html, "selected>");
				strcat(html, GNSS_PORT[i]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, GNSS_PORT[i]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr3);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Address:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"");
		char *gpioMaxStr20 = intToString(GPIO_NUM_MAX);
		strcat(html, gpioMaxStr20);
		strcat(html, "\" name=\"address\" type=\"number\" value=\"");
		char *modbusAddrStr = intToString(config.modbus_address);
		strcat(html, modbusAddrStr);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>DE:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"");
		char *gpioMaxStr21 = intToString(GPIO_NUM_MAX);
		strcat(html, gpioMaxStr21);
		strcat(html, "\" name=\"de\" type=\"number\" value=\"");
		char *modbusDeStr = intToString(config.modbus_de_gpio);
		strcat(html, modbusDeStr);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitMODBUS\" name=\"commitMODBUS\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitMODBUS\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		strcat(html, "</td><td width=\"23%\" style=\"border:unset;\">\n");

		/**************External TNC Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromTNC\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>External TNC Modify</b></span></th>\n");
		strcat(html, "<tr>\n");

		strcpy(enFlage, "");
		if (config.ext_tnc_enable)
			strcpy(enFlage, "checked");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ");
		strcat(html, enFlage);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PORT:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"channel\" id=\"channel\">\n");
		for (int i = 0; i < 5; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr4 = intToString(i);
			strcat(html, iStr4);
			strcat(html, "\" ");
			if (config.ext_tnc_channel == i)
			{
				strcat(html, "selected>");
				strcat(html, TNC_PORT[i]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, TNC_PORT[i]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr4);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>MODE:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mode\" id=\"mode\">\n");
		for (int i = 0; i < 4; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr5 = intToString(i);
			strcat(html, iStr5);
			strcat(html, "\" ");
			if (config.ext_tnc_mode == i)
			{
				strcat(html, "selected>");
				strcat(html, TNC_MODE[i]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, TNC_MODE[i]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr5);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitTNC\" name=\"commitTNC\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitTNC\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "<br />\n");

		strcat(html, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">\n");

		/************************ AT-COMMAND **************************/
		strcat(html, "<form id='formATCommand' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>AT-COMMAND CHANNEL</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td width=\"150\" align=\"right\"><b>MQTT:</b></td>\n");
		char cmdFlag[20] = "";
		if (config.at_cmd_mqtt)
			strcpy(cmdFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"mqtt\" value=\"OK\" ");
		strcat(html, cmdFlag);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>MESSAGE:</b></td>\n");
		strcpy(cmdFlag, "");
		if (config.at_cmd_msg)
			strcpy(cmdFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"msg\" value=\"OK\" ");
		strcat(html, cmdFlag);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>BLUETOOTH:</b></td>\n");
		strcpy(cmdFlag, "");
		if (config.at_cmd_bluetooth)
			strcpy(cmdFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"bluetooth\" value=\"OK\" ");
		strcat(html, cmdFlag);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UART PORT:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"uart\" id=\"cmdUart\">\n");
		for (int i = 0; i < 5; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr6 = intToString(i);
			strcat(html, iStr6);
			strcat(html, "\" ");
			if (config.at_cmd_uart == i)
			{
				strcat(html, "selected>");
				strcat(html, TNC_PORT[i]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, TNC_PORT[i]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr6);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitCMD'  name=\"commit\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitCMD\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />\n");

#ifdef PPPOS
		strcat(html, "<br />\n");

		strcat(html, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">\n");

		/************************ PPPoS **************************/

		strcat(html, "<form id='formPPPoS' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>PPP Over Serial (GSM/4G-LTE)</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
		char pppEnFlag[20] = "";
		if (config.ppp_enable)
			strcpy(pppEnFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"pppEn\" value=\"OK\" ");
		strcat(html, pppEnFlag);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<td align=\"right\"><b>GNSS:</b></td>\n");
		strcpy(pppEnFlag, "");
		if (config.ppp_gnss)
			strcpy(pppEnFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"pppGnss\" value=\"OK\" ");
		strcat(html, pppEnFlag);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>NAPT:</b></td>\n");
		strcpy(pppEnFlag, "");
		if (config.ppp_napt)
			strcpy(pppEnFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"pppNapt\" value=\"OK\" ");
		strcat(html, pppEnFlag);
		strcat(html, "><span class=\"slider round\"></span></label> *WiFi NAT</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>APN:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"20\" name=\"pppAPN\" type=\"text\" value=\"");
		strcat(html, config.ppp_apn);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		strcat(html, "<td align=\"right\"><b>PIN:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"0\" max=\"999999\" name=\"pppPin\" type=\"number\" value=\"");
		char *pppPinStr = intToString(atoi(config.ppp_pin));
		strcat(html, pppPinStr);
		strcat(html, "\" /> <i>*PIN of SIM</i></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		strcat(html, "<td align=\"right\"><b>RX GPIO:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\" name=\"rx\" type=\"number\" value=\"");
		char *pppRxStr = intToString(config.ppp_rx_gpio);
		strcat(html, pppRxStr);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX GPIO:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\" name=\"tx\" type=\"number\" value=\"");
		char *pppTxStr = intToString(config.ppp_tx_gpio);
		strcat(html, pppTxStr);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");

		strcpy(LowFlag, "");
		strcpy(HighFlag, "");
		if (config.ppp_rst_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RESET GPIO:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\"  name=\"rst\" type=\"number\" value=\"");
		char *pppRstStr = intToString(config.ppp_rst_gpio);
		strcat(html, pppRstStr);
		strcat(html, "\" /> Active:<input type=\"radio\" name=\"rst_active\" value=\"0\" ");
		strcat(html, LowFlag);
		strcat(html, "/>LOW <input type=\"radio\" name=\"rst_active\" value=\"1\" ");
		strcat(html, HighFlag);
		strcat(html, "/>HIGH </td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<td align=\"right\"><b>RESET DELAY:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"0\" max=\"999999\" name=\"rstDly\" type=\"number\" value=\"");
		char *pppRstDelayStr = intToString(config.ppp_rst_delay);
		strcat(html, pppRstDelayStr);
		strcat(html, "\" /> mSec.</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PORT:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"port\" id=\"port\">\n");
		for (int i = 0; i < 2; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr7 = intToString(i);
			strcat(html, iStr7);
			strcat(html, "\" ");
			if (config.ppp_serial == i)
			{
				strcat(html, "selected>");
				strcat(html, GNSS_PORT[i + 1]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, GNSS_PORT[i + 1]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr7);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Baudrate:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"baudrate\" id=\"baudrate\">\n");
		for (int i = 0; i < 13; i++)
		{
			strcat(html, "<option value=\"");
			char *baudrateStr3 = intToString(baudrate[i]);
			strcat(html, baudrateStr3);
			strcat(html, "\" ");
			if (config.ppp_serial_baudrate == baudrate[i])
			{
				strcat(html, "selected>");
				strcat(html, baudrateStr3);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, baudrateStr3);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(baudrateStr3);
		}
		strcat(html, "</select> bps\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitPPPoS'  name=\"commitPPPoS\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitPPPoS\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form>\n");

		strcat(html, "</td></tr></table>\n");
#endif

		const char* dataType = "text/html";
		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
		response->addHeader("MOD", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(html); // Free the allocated memory	
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
		const char *html = "Setup completed successfully";
		if (saveConfiguration("/default.cfg", config))
		{
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else if (request->hasArg("updateHostName"))
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
		const char *html = "Setup completed successfully";
		if (saveConfiguration("/default.cfg", config))
		{
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
		const char *html = "Setup completed successfully";
		if (saveConfiguration("/default.cfg", config))
		{
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else if (request->hasArg("updateAutoReset"))
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
		const char *html = "Setup completed successfully";
		if (saveConfiguration("/default.cfg", config))
		{
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
		const char *html = "Setup completed successfully";
		if (saveConfiguration("/default.cfg", config))
		{
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
			const char *html = "OK";
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
		const char *html = "Setup completed successfully";
		if (saveConfiguration("/default.cfg", config))
		{
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
		const char *html = "Setup completed successfully";
		if (saveConfiguration("/default.cfg", config))
		{
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
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
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
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_TELEMETRY;
				}
			}

			if (request->argName(i) == "FilterStatus")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_STATUS;
				}
			}

			if (request->argName(i) == "FilterWeather")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_WX;
				}
			}

			if (request->argName(i) == "FilterTracker")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_TRACKER;
				}
			}

			if (request->argName(i) == "FilterIGate")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_IGATE;
				}
			}

			if (request->argName(i) == "FilterDigi")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_DIGI;
				}
			}

			if (request->argName(i) == "FilterQuery")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_QUERY;
				}
			}

			if (request->argName(i) == "FilterWifi")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_WIFI;
				}
			}
		}
		config.pwr_en = PwrEn;
		const char *html = "Setup completed successfully";
		if (saveConfiguration("/default.cfg", config))
		{
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
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.log |= LOG_STATUS;
				}
			}

			if (request->argName(i) == "logWeather")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.log |= LOG_WX;
				}
			}

			if (request->argName(i) == "logTracker")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.log |= LOG_TRACKER;
				}
			}

			if (request->argName(i) == "logIgate")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.log |= LOG_IGATE;
				}
			}

			if (request->argName(i) == "logDigi")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.log |= LOG_DIGI;
				}
			}
		}
		const char *html = "Setup completed successfully";
		if (saveConfiguration("/default.cfg", config))
		{
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
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
					{
						oledEN = true;
					}
				}
			}
			if (request->argName(i) == "dispFlip")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
					{
						dispFlip = true;
					}
				}
			}
			if (request->argName(i) == "filterMessage")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_MESSAGE;
				}
			}

			if (request->argName(i) == "filterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_TELEMETRY;
				}
			}

			if (request->argName(i) == "filterStatus")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_STATUS;
				}
			}

			if (request->argName(i) == "filterWeather")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_WX;
				}
			}

			if (request->argName(i) == "filterObject")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_OBJECT;
				}
			}

			if (request->argName(i) == "filterItem")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_ITEM;
				}
			}

			if (request->argName(i) == "filterQuery")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_QUERY;
				}
			}
			if (request->argName(i) == "filterBuoy")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_BUOY;
				}
			}
			if (request->argName(i) == "filterPosition")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_POSITION;
				}
			}

			if (request->argName(i) == "dispRF")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						dispRF = true;
				}
			}

			if (request->argName(i) == "dispINET")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						dispINET = true;
				}
			}
			if (request->argName(i) == "txdispEnable")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						dispTX = true;
				}
			}
			if (request->argName(i) == "rxdispEnable")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
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
		if (oledEN && !config.oled_enable)
		{
			// display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false); // initialize with the I2C addr 0x3C (for the 128x64)
			//  Initialising the UI will init the display too.
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
		const char *html = "Setup completed successfully";
		if (saveConfiguration("/default.cfg", config))
		{
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

		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(22000); // Initial buffer size, adjust as needed
		if (!html)
		{
			return; // Memory allocation failed
		}

		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formHostName\") document.getElementById(\"updateHostName\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formTime\") document.getElementById(\"updateTime\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formNTP\") document.getElementById(\"updateTimeNtp\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formTimeZone\") document.getElementById(\"updateTimeZone\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formReboot\") document.getElementById(\"REBOOT\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formDisp\") document.getElementById(\"submitDISP\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formWebAuth\") document.getElementById(\"submitWebAuth\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formPWR\") document.getElementById(\"submitPWR\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formPath\") document.getElementById(\"submitPath\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/system',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");

		// html += "<h2>System Setting</h2>\n";
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>System Setting</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\">Host Name:</td>\n");

		// Building form with snprintf to avoid string concatenation
		char temp_buffer[512];
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formHostName\" method=\"post\"><input name=\"SetHostName\" type=\"text\" value=\"%s\" />\n", config.host_name);
		strcat(html, temp_buffer);
		strcat(html, "<button type='submit' id='updateHostName'  name=\"commit\"> Apply </button>\n");
		strcat(html, "<input type=\"hidden\" name=\"updateHostName\"/></form>\n</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>");
		// html += "<form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formTime\" method=\"post\">\n";
		strcat(html, "<td style=\"text-align: right;\">LOCAL DATE/TIME </td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formTime\" method=\"post\">\n<input name=\"SetTime\" type=\"text\" value=\"%s\" />\n", strTime);
		strcat(html, temp_buffer);
		strcat(html, "<span class=\"input-group-addon\">\n<span class=\"glyphicon glyphicon-calendar\">\n</span></span>\n");
		// html += "<div class=\"col-sm-3 col-xs-6\"><button class=\"btn btn-primary\" data-args=\"[true]\" data-method=\"getDate\" type=\"button\" data-related-target=\"#SetTime\" />Get Date</button></div>\n");
		strcat(html, "<button type='submit' id='updateTime'  name=\"commit\"> Time Update </button>\n");
		strcat(html, "<input type=\"hidden\" name=\"updateTime\"/></form>\n</td>\n");
		// html += "<input class=\"btn btn-primary\" id=\"updateTime\" name=\"updateTime\" type=\"submit\" value=\"Time Update\" maxlength=\"80\"/></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\">NTP Host </td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formNTP\" method=\"post\"><input name=\"SetTimeNtp\" type=\"text\" value=\"%s\" />\n", config.ntp_host);
		strcat(html, temp_buffer);
		strcat(html, "<button type='submit' id='updateTimeNtp'  name=\"commit\"> NTP Update </button>\n");
		strcat(html, "<input type=\"hidden\" name=\"updateTimeNtp\"/></form>\n</td>\n");
		// html += "<input class=\"btn btn-primary\" id=\"updateTimeNtp\" name=\"updateTimeNtp\" type=\"submit\" value=\"NTP Update\" maxlength=\"80\"/></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\">Auto REBOOT:</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formAutoReset\" method=\"post\"><input  min=\"0\" max=\"65535\"  name=\"SetAutoReset\" type=\"number\" value=\"%d\" /> Minutes\n", config.reset_timeout);
		strcat(html, temp_buffer);
		strcat(html, "<button type='submit' id='updateAutoReset'  name=\"commit\"> Update </button> *<i>0=No reset</i>\n");
		strcat(html, "<input type=\"hidden\" name=\"updateAutoReset\"/></form>\n</td>\n");
		// html += "<input class=\"button\" id=\"updateTimeNtp\" name=\"updateTimeNtp\" type=\"submit\" value=\"NTP Update\" maxlength=\"80\"/></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\">Time Zone </td>\n");
		strcat(html, "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formTimeZone\" method=\"post\">\n");
		strcat(html, "<select name=\"SetTimeZone\" id=\"SetTimeZone\">\n");
		for (int i = 0; i < 40; i++)
		{
			if (config.timeZone == tzList[i].tz)
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%.1f\" selected>%s Sec</option>\n", tzList[i].tz, tzList[i].name);
			}
			else
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%.1f\" >%s Sec</option>\n", tzList[i].tz, tzList[i].name);
			}
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>");
		strcat(html, "<button type='submit' id='updateTimeZone'  name=\"commit\"> TZ Update </button>\n");
		strcat(html, "<input type=\"hidden\" name=\"updateTimeZone\"/></form>\n</td>\n");
		// html += "<input class=\"btn btn-primary\" id=\"updateTimeZone\" name=\"updateTimeZone\" type=\"submit\" value=\"TZ Update\" maxlength=\"80\"/></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\">SYSTEM CONTROL </td>\n");
		strcat(html, "<td style=\"text-align: left;\"><table><tr><td width=\"100\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formReboot\" method=\"post\"> <button type='submit' id='REBOOT'  name=\"commit\" style=\"background-color:red;color:white\"> REBOOT </button>\n");
		strcat(html, " <input type=\"hidden\" name=\"REBOOT\"/></form></td><td width=\"100\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formFactory\" method=\"post\"> <button type='submit' id='Factory'  name=\"commit\" style=\"background-color:orange;color:white\"> Factory Reset </button>\n");
		strcat(html, " <input type=\"hidden\" name=\"Factory\"/></form></td><td width=\"100\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formLoad\" method=\"post\"> <button type='submit' id='LoadCFG'  name=\"commit\" style=\"background-color:green;color:white\"> Load Default </button>\n");
		strcat(html, " <input type=\"hidden\" name=\"LoadCFG\"/></form></td></tr></table></td>\n");
		// html += "<td style=\"text-align: left;\"><input type='submit' class=\"btn btn-danger\" id=\"REBOOT\" name=\"REBOOT\" value='REBOOT'></td>\n");
		strcat(html, "</tr></table><br /><br />\n");

		/************************ WEB AUTH **************************/
		strcat(html, "<form id='formWebAuth' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Web Authentication</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Web USER:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"32\" maxlength=\"32\" class=\"form-control\" name=\"webauth_user\" type=\"text\" value=\"%s\" /></td>\n", config.http_username);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Web PASSWORD:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"63\" maxlength=\"63\" class=\"form-control\" name=\"webauth_pass\" type=\"password\" value=\"%s\" /></td>\n", config.http_password);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitWebAuth'  name=\"commit\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitWebAuth\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br /><br />");

		/**************Power Mode******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formPWR\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Power Save Mode</b></span></th>\n");
		strcat(html, "<tr>");

		char enFlage[20] = "";
		if (config.pwr_en)
			strcpy(enFlage, "checked");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", enFlage);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		char LowFlag[30] = "", HighFlag[30] = "";
		strcpy(LowFlag, "");
		strcpy(HighFlag, "");
		if (config.pwr_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PWR GPIO:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\"  name=\"pwr\" type=\"number\" value=\"%d\" /> Output Active:<input type=\"radio\" name=\"pwr_active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"pwr_active\" value=\"1\" %s/>HIGH </td>\n", config.pwr_gpio, LowFlag, HighFlag);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Sleep Interval:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input min=\"0\" max=\"9999\" name=\"sleep\" type=\"number\" value=\"%d\" /></td>\n", config.pwr_sleep_interval);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>StandBy Delay:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input min=\"0\" max=\"9999\" name=\"stb\" type=\"number\" value=\"%d\" /></td>\n", config.pwr_stanby_delay);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Power Mode:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mode\" id=\"mode\">\n");
		for (int i = 0; i < 3; i++)
		{
			if (config.pwr_mode == i)
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" selected>%s </option>\n", i, PWR_MODE[i]);
			}
			else
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" >%s </option>\n", i, PWR_MODE[i]);
			}
			strcat(html, temp_buffer);
		}
		strcat(html, "</select> A=Reduce Speed(PWR Off),B=Light Sleep(WiFi/PWR Off),C=Deep Sleep(All Off)\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Event Activate:</b><br/>(For Mode C)</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<fieldset id=\"FilterGrp\">\n");
		strcat(html, "<legend>Events</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">\n");
		strcat(html, "<tr style=\"background:unset;\">");

		char filterFlageEn[20] = "";
		if (config.pwr_sleep_activate & ACTIVATE_TRACKER)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterTracker\" type=\"checkbox\" value=\"OK\" %s/>Tracker</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_STATUS)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterStatus\" type=\"checkbox\" value=\"OK\" %s/>Status</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_TELEMETRY)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterTelemetry\" type=\"checkbox\" value=\"OK\" %s/>Telemetry</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_WX)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_IGATE)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterIGate\" type=\"checkbox\" value=\"OK\" %s/>IGate</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_DIGI)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterDigi\" type=\"checkbox\" value=\"OK\" %s/>Digi</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_QUERY)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterQuery\" type=\"checkbox\" value=\"OK\" %s/>Query</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_WIFI)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterWifi\" type=\"checkbox\" value=\"OK\" %s/>WiFi</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		strcat(html, "<td style=\"border:unset;\"></td>\n");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitPWR'  name=\"commitPWR\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitPWR\"/>\n");
		strcat(html, "</td></tr></table>\n");

		strcat(html, "</form><br /><br />\n");

		/**************Log File******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formLOG\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Log File</b></span></th>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Activate:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<fieldset id=\"FilterGrp\">\n");
		strcat(html, "<legend>Events</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">\n");
		strcat(html, "<tr style=\"background:unset;\">");

		if (config.log & LOG_TRACKER)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logTracker\" type=\"checkbox\" value=\"OK\" %s/>Tracker</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.log & LOG_IGATE)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logIgate\" type=\"checkbox\" value=\"OK\" %s/>IGate</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.log & LOG_DIGI)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logDigi\" type=\"checkbox\" value=\"OK\" %s/>DIGI</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.log & LOG_WX)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		strcat(html, "<td style=\"border:unset;\"></td>\n");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitLOG'  name=\"commitLOG\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitLOG\"/>\n");
		strcat(html, "</td></tr></table>\n");

		strcat(html, "</form><br /><br />\n");

		/************************ PATH USER define **************************/
		strcat(html, "<form id='formPath' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>PATH USER Define</b></span></th>\n");
		strcat(html, "<tr>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"right\"><b>PATH_1:</b></td>\n<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path1\" type=\"text\" value=\"%s\" /></td>\n", config.path[0]);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"right\"><b>PATH_2:</b></td>\n<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path2\" type=\"text\" value=\"%s\" /></td>\n", config.path[1]);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"right\"><b>PATH_3:</b></td>\n<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path3\" type=\"text\" value=\"%s\" /></td>\n", config.path[2]);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"right\"><b>PATH_4:</b></td>\n<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path4\" type=\"text\" value=\"%s\" /></td>\n", config.path[3]);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitPath'  name=\"commitPath\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitPath\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form><br /><br />");
		// delay(1);
// log_d("%s",html.c_str());
// log_d("Length: %d",html.length());
#if defined OLED || defined ST7735_160x80
		strcat(html, "<form id='formDisp' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		// html += "<h2>Display Setting</h2>\n";
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Display Setting</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>OLED/TFT Enable</b></td>\n");
		char oledFlageEn[20] = "";
		if (config.oled_enable == true)
			strcpy(oledFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"oledEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", oledFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcpy(oledFlageEn, "");
		if (config.disp_flip == true)
			strcpy(oledFlageEn, "checked");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>Flip Rotate</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"dispFlip\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", oledFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>TX Display</b></td>\n");
		char txdispFlageEn[20] = "";
		if (config.tx_display == true)
			strcpy(txdispFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"txdispEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*All TX Packet for display affter filter.</i></label></td>\n", txdispFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>RX Display</b></td>\n");
		char rxdispFlageEn[20] = "";
		if (config.rx_display == true)
			strcpy(rxdispFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"rxdispEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*All RX Packet for display affter filter.</i></label></td>\n", rxdispFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>Head Up</b></td>\n");
		char hupFlageEn[20] = "";
		if (config.h_up == true)
			strcpy(hupFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"hupEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*The compass will rotate in the direction of movement.</i></label></td>\n", hupFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>TFT Brightness</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"dispBright\" id=\"dispBright\">\n");
		for (int i = 0; i < 255; i += 25)
		{
			if (config.disp_brightness == i)
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" selected>%d</option>\n", i, i);
			}
			else
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" >%d</option>\n", i, i);
			}
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>Popup Delay</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"dispDelay\" id=\"dispDelay\">\n");
		for (int i = 0; i < 16; i += 1)
		{
			if (config.dispDelay == i)
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" selected>%d Sec</option>\n", i, i);
			}
			else
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" >%d Sec</option>\n", i, i);
			}
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td></tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>OLED/TFT Sleep</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"oled_timeout\" id=\"oled_timeout\">\n");
		for (int i = 0; i <= 600; i += 30)
		{
			if (config.oled_timeout == i)
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" selected>%d Sec</option>\n", i, i);
			}
			else
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" >%d Sec</option>\n", i, i);
			}
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td></tr>\n");
		char rfFlageEn[20] = "";
		if (config.dispRF == true)
			strcpy(rfFlageEn, "checked");
		char inetFlageEn[20] = "";
		if (config.dispINET == true)
			strcpy(inetFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<tr><td style=\"text-align: right;\"><b>RX Channel</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"dispRF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"dispINET\" value=\"OK\" %s/>Internet </td></tr>\n", rfFlageEn, inetFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Filter DX:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input type=\"number\" name=\"filterDX\" min=\"0\" max=\"9999\"\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "step=\"1\" value=\"%d\" /> Km.  <label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*Value 0 is all distant allow.</i></label></td>\n", config.filterDistant);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Filter:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<fieldset id=\"filterDispGrp\">\n");
		strcat(html, "<legend>Filter popup display</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">\n");
		strcat(html, "<tr style=\"background:unset;\">");

		// html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"dispTNC\" name=\"dispTNC\" type=\"checkbox\" value=\"OK\" " + rfFlageEn + "/>From RF</td>\n";

		// html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"dispINET\" name=\"dispINET\" type=\"checkbox\" value=\"OK\" " + inetFlageEn + "/>From INET</td>\n";

		char filterMessageFlageEn[20] = "";
		if (config.dispFilter & FILTER_MESSAGE)
			strcpy(filterMessageFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterMessage\" name=\"filterMessage\" type=\"checkbox\" value=\"OK\" %s/>Message</td>\n", filterMessageFlageEn);
		strcat(html, temp_buffer);

		char filterStatusFlageEn[20] = "";
		if (config.dispFilter & FILTER_STATUS)
			strcpy(filterStatusFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterStatus\" name=\"filterStatus\" type=\"checkbox\" value=\"OK\" %s/>Status</td>\n", filterStatusFlageEn);
		strcat(html, temp_buffer);

		char filterTelemetryFlageEn[20] = "";
		if (config.dispFilter & FILTER_TELEMETRY)
			strcpy(filterTelemetryFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterTelemetry\" name=\"filterTelemetry\" type=\"checkbox\" value=\"OK\" %s/>Telemetry</td>\n", filterTelemetryFlageEn);
		strcat(html, temp_buffer);

		char filterWeatherFlageEn[20] = "";
		if (config.dispFilter & FILTER_WX)
			strcpy(filterWeatherFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterWeather\" name=\"filterWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n", filterWeatherFlageEn);
		strcat(html, temp_buffer);

		char filterObjectFlageEn[20] = "";
		if (config.dispFilter & FILTER_OBJECT)
			strcpy(filterObjectFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterObject\" name=\"filterObject\" type=\"checkbox\" value=\"OK\" %s/>Object</td>\n", filterObjectFlageEn);
		strcat(html, temp_buffer);

		char filterItemFlageEn[20] = "";
		if (config.dispFilter & FILTER_ITEM)
			strcpy(filterItemFlageEn, "checked");
		strcat(html, "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterItem\" name=\"filterItem\" type=\"checkbox\" value=\"OK\" ");
		strcat(html, filterItemFlageEn);
		strcat(html, "/>Item</td>\n");

		char filterQueryFlageEn[20] = "";
		if (config.dispFilter & FILTER_QUERY)
			strcpy(filterQueryFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterQuery\" name=\"filterQuery\" type=\"checkbox\" value=\"OK\" %s/>Query</td>\n", filterQueryFlageEn);
		strcat(html, temp_buffer);

		char filterBuoyFlageEn[20] = "";
		if (config.dispFilter & FILTER_BUOY)
			strcpy(filterBuoyFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterBuoy\" name=\"filterBuoy\" type=\"checkbox\" value=\"OK\" %s/>Buoy</td>\n", filterBuoyFlageEn);
		strcat(html, temp_buffer);

		char filterPositionFlageEn[20] = "";
		if (config.dispFilter & FILTER_POSITION)
			strcpy(filterPositionFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterPosition\" name=\"filterPosition\" type=\"checkbox\" value=\"OK\" %s/>Position</td>\n", filterPositionFlageEn);
		strcat(html, temp_buffer);

		strcat(html, "<td style=\"border:unset;\"></td>\n");
		strcat(html, "</tr></table></fieldset>\n");

		strcat(html, "</td></tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitDISP'  name=\"commitDISP\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitDISP\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");
#endif

		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
		response->addHeader("System", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(html);
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

		// waitISRetry = millis() + 10000; // Retry connect 5Sec
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
		aprsClient.stop();
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
		// Allocate initial memory for HTML content
		char *html = allocateStringMemory(25000); // Start with 8KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		// strcat(html, "document.getElementById(\"submitIGATE\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formIgate\") document.getElementById(\"submitIGATE\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formIgateFilter\") document.getElementById(\"submitIGATEfilter\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/igate',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n<script type=\"text/javascript\">\n");

		strcat(html, "function openWindowSymbol() {\n");
		strcat(html, "var i, l, options = [{\n");
		strcat(html, "value: 'first',\n");
		strcat(html, "text: 'First'\n");
		strcat(html, "}, {\n");
		strcat(html, "value: 'second',\n");
		strcat(html, "text: 'Second'\n");
		strcat(html, "}],\n");
		strcat(html, "newWindow = window.open(\"/symbol\", null, \"height=400,width=400,status=no,toolbar=no,menubar=no,location=no\");\n");
		strcat(html, "}\n");

		strcat(html, "function setValue(symbol,table) {\n");
		strcat(html, "document.getElementById('igateSymbol').value = String.fromCharCode(symbol);\n");
		strcat(html, "if(table==1){\n document.getElementById('igateTable').value='/';\n");
		strcat(html, "}else if(table==2){\n document.getElementById('igateTable').value='\\\\';\n}\n");
		strcat(html, "document.getElementById('igateImgSymbol').src = \"http://aprs.dprns.com/symbols/icons/\"+symbol.toString()+'-'+table.toString()+'.png';\n");
		strcat(html, "\n}\n");
		strcat(html, "function calculatePHGR(){document.forms.formIgate.texttouse.value=\"PHG\"+calcPower(document.forms.formIgate.power.value)+calcHeight(document.forms.formIgate.haat.value)+calcGain(document.forms.formIgate.gain.value)+calcDirection(document.forms.formIgate.direction.selectedIndex)}function Log2(e){return Math.log(e)/Math.log(2)}function calcPerHour(e){return e<10?e:String.fromCharCode(65+(e-10))}function calcHeight(e){return String.fromCharCode(48+Math.round(Log2(e/10),0))}function calcPower(e){if(e<1)return 0;if(e>=1&&e<4)return 1;if(e>=4&&e<9)return 2;if(e>=9&&e<16)return 3;if(e>=16&&e<25)return 4;if(e>=25&&e<36)return 5;if(e>=36&&e<49)return 6;if(e>=49&&e<64)return 7;if(e>=64&&e<81)return 8;if(e>=81)return 9}function calcDirection(e){if(e==\"0\")return\"0\";if(e==\"1\")return\"1\";if(e==\"2\")return\"2\";if(e==\"3\")return\"3\";if(e==\"4\")return\"4\";if(e==\"5\")return\"5\";if(e==\"6\")return\"6\";if(e==\"7\")return\"7\";if(e==\"8\")return\"8\"}function calcGain(e){return e>9?\"9\":e<0?\"0\":Math.round(e,0)}\n");
		strcat(html, "function onRF2INETCheck() {\n");
		strcat(html, "if (document.querySelector('#rf2inetEnable').checked) {\n");
		// Checkbox has been checked
		strcat(html, "document.getElementById(\"rf2inetFilterGrp\").disabled=false;\n");
		strcat(html, "} else {\n");
		// Checkbox has been unchecked
		strcat(html, "document.getElementById(\"rf2inetFilterGrp\").disabled=true;\n");
		strcat(html, "}\n}\n");
		strcat(html, "function onINET2RFCheck() {\n");
		strcat(html, "if (document.querySelector('#inet2rfEnable').checked) {\n");
		// Checkbox has been checked
		strcat(html, "document.getElementById(\"inet2rfFilterGrp\").disabled=false;\n");
		strcat(html, "} else {\n");
		// Checkbox has been unchecked
		strcat(html, "document.getElementById(\"inet2rfFilterGrp\").disabled=true;\n");
		strcat(html, "}\n}\n");

		strcat(html, "function selPrecision(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "x = document.getElementsByName(\"precision\"+idx)[0].value;\n");
		strcat(html, "document.getElementsByName(\"eqns\"+idx+\"b\")[0].value=1/Math.pow(10,x);\n");
		strcat(html, "}\n");
		strcat(html, "function selOffset(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "x = document.getElementsByName(\"offset\"+idx)[0].value;\n");
		strcat(html, "document.getElementsByName(\"eqns\"+idx+\"c\")[0].value=x*(-1);\n");
		strcat(html, "}\n");
		strcat(html, "</script>\n");
		delay(1);
		/************************ IGATE Mode **************************/
		strcat(html, "<form id='formIgate' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		// strcat(html, "<h2>[IGATE] Internet Gateway Mode</h2>\n");
		strcat(html, "<table>\n");
		// strcat(html, "<tr>\n");
		// strcat(html, "<th width=\"200\"><span><b>Setting</b></span></th>\n");
		// strcat(html, "<th><span><b>Value</b></span></th>\n");
		// strcat(html, "</tr>\n");
		strcat(html, "<th colspan=\"2\"><span><b>[IGATE] Internet Gateway Mode</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
		String igateEnFlag = "";
		if (config.igate_en)
			igateEnFlag = "checked";
		{
			char *temp_flag = allocateStringMemory(512);
			if (temp_flag)
			{
				snprintf(temp_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"igateEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", igateEnFlag);
				strcat(html, temp_flag);
				free(temp_flag);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station Callsign:</b></td>\n");
		{
			char *temp_value = allocateStringMemory(256);
			if (temp_value)
			{
				snprintf(temp_value, 256, "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"%s\" /></td>\n", config.aprs_mycall);
				strcat(html, temp_value);
				free(temp_value);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station SSID:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mySSID\" id=\"mySSID\">\n");
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			char *temp_option = allocateStringMemory(256);
			if (temp_option)
			{
				if (config.aprs_ssid == ssid)
				{
					snprintf(temp_option, 256, "<option value=\"%d\" selected>%d</option>\n", ssid, ssid);
				}
				else
				{
					snprintf(temp_option, 256, "<option value=\"%d\">%d</option>\n", ssid, ssid);
				}
				strcat(html, temp_option);
				free(temp_option);
			}
		}
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station Symbol:</b></td>\n");
		const char *table = "1";
		if (config.igate_symbol[0] == 47)
			table = "1";
		if (config.igate_symbol[0] == 92)
			table = "2";
		{
			char *temp_symbol = allocateStringMemory(512);
			if (temp_symbol)
			{
				snprintf(temp_symbol, 512, "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"igateTable\" name=\"igateTable\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"igateSymbol\" name=\"igateSymbol\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"igateImgSymbol\" onclick=\"openWindowSymbol();\" src=\"http://aprs.dprns.com/symbols/icons/%d-%s.png\"> <i>*Click icon for select symbol</i></td>\n",
						 config.igate_symbol[0], config.igate_symbol[1], (int)config.igate_symbol[1], table);
				strcat(html, temp_symbol);
				free(temp_symbol);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Item/Obj Name:</b></td>\n");
		{
			char *temp_obj = allocateStringMemory(256);
			if (temp_obj)
			{
				snprintf(temp_obj, 256, "<td style=\"text-align: left;\"><input maxlength=\"9\" size=\"9\" id=\"igateObject\" name=\"igateObject\" type=\"text\" value=\"%s\" /><i> *If not used, leave it blank.In use 3-9 charactor</i></td>\n", config.igate_object);
				strcat(html, temp_obj);
				free(temp_obj);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"igatePath\" id=\"igatePath\">\n");
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			char *temp_path = allocateStringMemory(256);
			if (temp_path)
			{
				if (config.igate_path == pthIdx)
				{
					snprintf(temp_path, 256, "<option value=\"%d\" selected>%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
				}
				else
				{
					snprintf(temp_path, 256, "<option value=\"%d\">%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
				}
				strcat(html, temp_path);
				free(temp_path);
			}
		}
		strcat(html, "</select></td>\n");
		// strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" id=\"igatePath\" name=\"igatePath\" type=\"text\" value=\"" + String(config.igate_path) + "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Host:</b></td>\n");
		{
			char *temp_host = allocateStringMemory(512);
			if (temp_host)
			{
				snprintf(temp_host, 512, "<td style=\"text-align: left;\"><input maxlength=\"20\" size=\"20\" id=\"aprsHost\" name=\"aprsHost\" type=\"text\" value=\"%s\" /> *APRS-IS by T2THAI at <a href=\"http://aprs.dprns.com:14501\" target=\"_t2thai\">aprs.dprns.com:14580</a>,CBAPRS at <a href=\"http://aprs.dprns.com:24501\" target=\"_t2thai\">aprs.dprns.com:24580</a></td>\n", config.aprs_host);
				strcat(html, temp_host);
				free(temp_host);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Port:</b></td>\n");
		{
			char *temp_port = allocateStringMemory(512);
			if (temp_port)
			{
				snprintf(temp_port, 512, "<td style=\"text-align: left;\"><input min=\"1\" max=\"65535\" step=\"1\" id=\"aprsPort\" name=\"aprsPort\" type=\"number\" value=\"%d\" /> *AMPR Host at <a href=\"http://aprs.hs5tqa.ampr.org:14501\" target=\"_t2thai\">aprs.hs5tqa.ampr.org:14580</a></td>\n", config.aprs_port);
				strcat(html, temp_port);
				free(temp_port);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Filter:</b></td>\n");
		{
			char *temp_filter = allocateStringMemory(512);
			if (temp_filter)
			{
				snprintf(temp_filter, 512, "<td style=\"text-align: left;\"><input maxlength=\"30\" size=\"30\" id=\"aprsFilter\" name=\"aprsFilter\" type=\"text\" value=\"%s\" /> *Filter: <a target=\"_blank\" href=\"http://www.aprs-is.net/javAPRSFilter.aspx\">http://www.aprs-is.net/javAPRSFilter.aspx</a></td>\n", config.aprs_filter);
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Text Comment:</b></td>\n");
		{
			char *temp_comment = allocateStringMemory(256);
			if (temp_comment)
			{
				snprintf(temp_comment, 256, "<td style=\"text-align: left;\"><input maxlength=\"25\" size=\"30\" id=\"igateComment\" name=\"igateComment\" type=\"text\" value=\"%s\" /></td>\n", config.igate_comment);
				strcat(html, temp_comment);
				free(temp_comment);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Text Status:</b></td>\n");
		{
			char *temp_status = allocateStringMemory(512);
			if (temp_status)
			{
				snprintf(temp_status, 512, "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"60\" id=\"igateStatus\" name=\"igateStatus\" type=\"text\" value=\"%s\" />  Interval:<input min=\"0\" max=\"3600\" step=\"1\" name=\"igateSTSInv\" type=\"number\" value=\"%d\" />Sec.</td>\n", config.igate_status, config.igate_sts_interval);
				strcat(html, temp_status);
				free(temp_status);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		{
			const char *rf2inetEnFlag = config.rf2inet ? "checked" : "";
			char *temp_rf2inet = allocateStringMemory(512);
			if (temp_rf2inet)
			{
				snprintf(temp_rf2inet, 512, "<td align=\"right\"><b>RF2INET:</b></td>\n<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"rf2inetEnable\" name=\"rf2inetEnable\" onclick=\"onRF2INETCheck()\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch RF to Internet gateway</i></label></td>\n", rf2inetEnFlag);
				strcat(html, temp_rf2inet);
				free(temp_rf2inet);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		{
			const char *inet2rfEnFlag = config.inet2rf ? "checked" : "";
			char *temp_inet2rf = allocateStringMemory(512);
			if (temp_inet2rf)
			{
				snprintf(temp_inet2rf, 512, "<td align=\"right\"><b>INET2RF:</b></td>\n<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"inet2rfEnable\" name=\"inet2rfEnable\" onclick=\"onINET2RFCheck()\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch Internet to RF gateway</i></label></td>\n", inet2rfEnFlag);
				strcat(html, temp_inet2rf);
				free(temp_inet2rf);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		{
			const char *timeStampFlag = config.igate_timestamp ? "checked" : "";
			char *temp_timestamp = allocateStringMemory(512);
			if (temp_timestamp)
			{
				snprintf(temp_timestamp, 512, "<td align=\"right\"><b>Time Stamp:</b></td>\n<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"igateTimeStamp\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", timeStampFlag);
				strcat(html, temp_timestamp);
				free(temp_timestamp);
			}
		}
		strcat(html, "</tr>\n<tr>");

		strcat(html, "<td align=\"right\"><b>POSITION:</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");
		{
			const char *igateBcnEnFlag = config.igate_bcn ? "checked" : "";
			char *temp_beacon = allocateStringMemory(512);
			if (temp_beacon)
			{
				snprintf(temp_beacon, 512, "<tr><td style=\"text-align: right;\">Beacon:</td><td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"igateBcnEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\">  Interval:<input min=\"0\" max=\"3600\" step=\"1\" id=\"igatePosInv\" name=\"igatePosInv\" type=\"number\" value=\"%d\" />Sec.</label></td></tr>", igateBcnEnFlag, config.igate_interval);
				strcat(html, temp_beacon);
				free(temp_beacon);
			}
		}
		const char *igatePosFixFlag = config.igate_gps ? "" : "checked=\"checked\"";
		const char *igatePosGPSFlag = config.igate_gps ? "checked=\"checked\"" : "";
		const char *igatePos2RFFlag = config.igate_loc2rf ? "checked" : "";
		const char *igatePos2INETFlag = config.igate_loc2inet ? "checked" : "";

		{
			char *temp_pos = allocateStringMemory(512);
			if (temp_pos)
			{
				snprintf(temp_pos, 512, "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"igatePosSel\" value=\"0\" %s/>Fix <input type=\"radio\" name=\"igatePosSel\" value=\"1\" %s/>GPS </td></tr>\n", igatePosFixFlag, igatePosGPSFlag);
				strcat(html, temp_pos);
				free(temp_pos);
			}
		}
		{
			char *temp_channel = allocateStringMemory(512);
			if (temp_channel)
			{
				snprintf(temp_channel, 512, "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"igatePos2RF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"igatePos2INET\" value=\"OK\" %s/>Internet </td></tr>\n", igatePos2RFFlag, igatePos2INETFlag);
				strcat(html, temp_channel);
				free(temp_channel);
			}
		}
		{
			char *temp_lat = allocateStringMemory(512);
			if (temp_lat)
			{
				snprintf(temp_lat, 512, "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" id=\"igatePosLat\" name=\"igatePosLat\" type=\"number\" value=\"%.5f\" />degrees (positive for North, negative for South)</td></tr>\n", config.igate_lat);
				strcat(html, temp_lat);
				free(temp_lat);
			}
		}
		{
			char *temp_lon = allocateStringMemory(512);
			if (temp_lon)
			{
				snprintf(temp_lon, 512, "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" id=\"igatePosLon\" name=\"igatePosLon\" type=\"number\" value=\"%.5f\" />degrees (positive for East, negative for West)</td></tr>\n", config.igate_lon);
				strcat(html, temp_lon);
				free(temp_lon);
			}
		}
		{
			char *temp_alt = allocateStringMemory(512);
			if (temp_alt)
			{
				snprintf(temp_alt, 512, "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" id=\"igatePosAlt\" name=\"igatePosAlt\" type=\"number\" value=\"%.2f\" /> meter. *Value 0 is not send height</td></tr>\n", config.igate_alt);
				strcat(html, temp_alt);
				free(temp_alt);
			}
		}
		strcat(html, "</table></td>");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PHG:</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Radio TX Power</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"power\" id=\"power\">\n");
		strcat(html, "<option value=\"1\" selected>1</option>\n");
		strcat(html, "<option value=\"5\">5</option>\n");
		strcat(html, "<option value=\"10\">10</option>\n");
		strcat(html, "<option value=\"15\">15</option>\n");
		strcat(html, "<option value=\"25\">25</option>\n");
		strcat(html, "<option value=\"35\">35</option>\n");
		strcat(html, "<option value=\"50\">50</option>\n");
		strcat(html, "<option value=\"65\">65</option>\n");
		strcat(html, "<option value=\"80\">80</option>\n");
		strcat(html, "</select> Watts</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td style=\"text-align: right;\">Antenna Gain</td><td style=\"text-align: left;\"><input size=\"3\" min=\"0\" max=\"100\" step=\"0.1\" id=\"gain\" name=\"gain\" type=\"number\" value=\"6\" /> dBi</td></tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Height</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"haat\" id=\"haat\">\n");
		int k = 10;
		for (uint8_t w = 0; w < 10; w++)
		{
			char *temp_opt = allocateStringMemory(256);
			if (temp_opt)
			{
				if (w == 0)
				{
					snprintf(temp_opt, 256, "<option value=\"%d\" selected>%d</option>\n", k, k);
				}
				else
				{
					snprintf(temp_opt, 256, "<option value=\"%d\">%d</option>\n", k, k);
				}
				strcat(html, temp_opt);
				free(temp_opt);
			}
			k += k;
		}
		strcat(html, "</select> Feet</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Antenna/Direction</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"direction\" id=\"direction\">\n");
		strcat(html, "<option>Omni</option><option>NE</option><option>E</option><option>SE</option><option>S</option><option>SW</option><option>W</option><option>NW</option><option>N</option>\n");
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");

		{
			char *temp_phg = allocateStringMemory(512);
			if (temp_phg)
			{
				snprintf(temp_phg, 512, "<tr><td align=\"right\"><b>PHG Text</b></td><td align=\"left\"><input name=\"texttouse\" type=\"text\" size=\"6\" style=\"background-color: rgb(97, 239, 170);\" value=\"%s\"/> <input type=\"button\" value=\"Calculate PHG\" onclick=\"javascript:calculatePHGR()\" /></td></tr>\n", config.igate_phg);
				strcat(html, temp_phg);
				free(temp_phg);
			}
		}
		strcat(html, "</table></td>");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Telemetry:</b><br />(v=0->8280)</td>\n");
		strcat(html, "<td align=\"center\"><table>\n");
		{
			char *temp_intv = allocateStringMemory(512);
			if (temp_intv)
			{
				snprintf(temp_intv, 512, "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"1000\" step=\"1\" id=\"igateTlmInv\" name=\"igateTlmInv\" type=\"number\" value=\"%d\" /> *Number of packets interval,<i>Example: 0 not send,1 send every packet</i></label></td></tr>", config.igate_tlm_interval);
				strcat(html, temp_intv);
				free(temp_intv);
			}
		}
		for (int ax = 0; ax < 5; ax++)
		{
			{
				char *temp_ch = allocateStringMemory(256);
				if (temp_ch)
				{
					snprintf(temp_ch, 256, "<tr><td align=\"right\"><b>CH A%d:</b></td>\n", ax + 1);
					strcat(html, temp_ch);
					free(temp_ch);
				}
			}
			strcat(html, "<td align=\"center\">\n");
			strcat(html, "<table>");

			strcat(html, "<tr><td style=\"text-align: right;\">Sensor:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">CH: ");

			{
				char *temp_select = allocateStringMemory(256);
				if (temp_select)
				{
					snprintf(temp_select, 256, "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax, ax);
					strcat(html, temp_select);
					free(temp_select);
				}
			}

			for (uint8_t idx = 0; idx < 11; idx++)
			{
				char *temp_opt = allocateStringMemory(256);
				if (temp_opt)
				{
					if (idx == 0)
					{
						if (config.igate_tlm_sensor[ax] == idx)
						{
							snprintf(temp_opt, 256, "<option value=\"%d\" selected>NONE</option>\n", idx);
						}
						else
						{
							snprintf(temp_opt, 256, "<option value=\"%d\">NONE</option>\n", idx);
						}
					}
					else
					{
						if (config.igate_tlm_sensor[ax] == idx)
						{
							snprintf(temp_opt, 256, "<option value=\"%d\" selected>SENSOR#%d</option>\n", idx, idx);
						}
						else
						{
							snprintf(temp_opt, 256, "<option value=\"%d\">SENSOR#%d</option>\n", idx, idx);
						}
					}
					strcat(html, temp_opt);
					free(temp_opt);
				}
			}
			strcat(html, "</select></td>\n");

			{
				char *temp_param = allocateStringMemory(512);
				if (temp_param)
				{
					snprintf(temp_param, 512, "<td style=\"text-align: left;\">Name: <input maxlength=\"10\" size=\"8\" name=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.igate_tlm_PARM[ax]);
					strcat(html, temp_param);
					free(temp_param);
				}
			}

			{
				char *temp_unit = allocateStringMemory(512);
				if (temp_unit)
				{
					snprintf(temp_unit, 512, "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.igate_tlm_UNIT[ax]);
					strcat(html, temp_unit);
					free(temp_unit);
				}
			}

			{
				char *temp_prec = allocateStringMemory(512);
				if (temp_prec)
				{
					snprintf(temp_prec, 512, "<td style=\"text-align: left;\">Precision: <input min=\"0\" max=\"5\" step=\"1\" type=\"number\" style=\"width: 2em\" name=\"precision%d\" type=\"text\" value=\"%d\" onchange=\"selPrecision(%d)\"/></td></tr>\n", ax, config.igate_tlm_precision[ax], ax);
					strcat(html, temp_prec);
					free(temp_prec);
				}
			}

			{
				char *temp_eqns = allocateStringMemory(1024);
				if (temp_eqns)
				{
					snprintf(temp_eqns, 1024, "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%da\" type=\"number\" value=\"%.5f\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%db\" type=\"number\" value=\"%.5f\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%dc\" type=\"number\" value=\"%.5f\" /> (av<sup>2</sup>+bv+c) </td>\n",
							 ax, config.igate_tlm_EQNS[ax][0], ax, config.igate_tlm_EQNS[ax][1], ax, config.igate_tlm_EQNS[ax][2]);
					strcat(html, temp_eqns);
					free(temp_eqns);
				}
			}

			{
				char *temp_offset = allocateStringMemory(512);
				if (temp_offset)
				{
					snprintf(temp_offset, 512, "<td style=\"text-align: left;\">Offset: <input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" type=\"number\" name=\"offset%d\" type=\"text\" value=\"%.5f\" onchange=\"selOffset(%d)\"/></td></tr>\n", ax, config.igate_tlm_offset[ax], ax);
					strcat(html, temp_offset);
					free(temp_offset);
				}
			}

			strcat(html, "</table></td>");
			strcat(html, "</tr>\n");
		}
		strcat(html, "</table></td></tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitIGATE'  name=\"commitIGATE\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitIGATE\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br /><br />");

		strcat(html, "<form id='formIgateFilter' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>[IGATE] Filter</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RF2INET Filter:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		if (config.rf2inet)
			strcat(html, "<fieldset id=\"rf2inetFilterGrp\">\n");
		else
			strcat(html, "<fieldset id=\"rf2inetFilterGrp\" disabled>\n");
		strcat(html, "<legend>Filter RF to Internet</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">");
		strcat(html, "<tr style=\"background:unset;\">");

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterMessage\" type=\"checkbox\" value=\"OK\" %s/>Message</td>\n",
						 (config.rf2inetFilter & FILTER_MESSAGE) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterStatus\" type=\"checkbox\" value=\"OK\" %s/>Status</td>\n",
						 (config.rf2inetFilter & FILTER_STATUS) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterTelemetry\" type=\"checkbox\" value=\"OK\" %s/>Telemetry</td>\n",
						 (config.rf2inetFilter & FILTER_TELEMETRY) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n",
						 (config.rf2inetFilter & FILTER_WX) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterObject\" type=\"checkbox\" value=\"OK\" %s/>Object</td>\n",
						 (config.rf2inetFilter & FILTER_OBJECT) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterItem\" type=\"checkbox\" value=\"OK\" %s/>Item</td>\n",
						 (config.rf2inetFilter & FILTER_ITEM) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterQuery\" type=\"checkbox\" value=\"OK\" %s/>Query</td>\n",
						 (config.rf2inetFilter & FILTER_QUERY) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterBuoy\" type=\"checkbox\" value=\"OK\" %s/>Buoy</td>\n",
						 (config.rf2inetFilter & FILTER_BUOY) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterPosition\" type=\"checkbox\" value=\"OK\" %s/>Position</td>\n",
						 (config.rf2inetFilter & FILTER_POSITION) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		strcat(html, "<td style=\"border:unset;\"></td>");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>INET2RF Filter:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		if (config.inet2rf)
			strcat(html, "<fieldset id=\"inet2rfFilterGrp\">\n");
		else
			strcat(html, "<fieldset id=\"inet2rfFilterGrp\" disabled>\n");
		strcat(html, "<legend>Filter Internet to RF</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">");
		strcat(html, "<tr style=\"background:unset;\">");

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterMessage\" type=\"checkbox\" value=\"OK\" %s/>Message</td>\n",
						 (config.inet2rfFilter & FILTER_MESSAGE) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterStatus\" type=\"checkbox\" value=\"OK\" %s/>Status</td>\n",
						 (config.inet2rfFilter & FILTER_STATUS) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterTelemetry\" type=\"checkbox\" value=\"OK\" %s/>Telemetry</td>\n",
						 (config.inet2rfFilter & FILTER_TELEMETRY) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n",
						 (config.inet2rfFilter & FILTER_WX) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterObject\" type=\"checkbox\" value=\"OK\" %s/>Object</td>\n",
						 (config.inet2rfFilter & FILTER_OBJECT) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterItem\" type=\"checkbox\" value=\"OK\" %s/>Item</td>\n",
						 (config.inet2rfFilter & FILTER_ITEM) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterQuery\" type=\"checkbox\" value=\"OK\" %s/>Query</td>\n",
						 (config.inet2rfFilter & FILTER_QUERY) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterBuoy\" type=\"checkbox\" value=\"OK\" %s/>Buoy</td>\n",
						 (config.inet2rfFilter & FILTER_BUOY) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterPosition\" type=\"checkbox\" value=\"OK\" %s/>Position</td>\n",
						 (config.inet2rfFilter & FILTER_POSITION) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		strcat(html, "<td style=\"border:unset;\"></td>");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitIGATEfilter'  name=\"commitIGATEfilter\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitIGATEfilter\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");

		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
		response->addHeader("IGATE", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(html); // Free the allocated memory
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
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						digiEn = true;
				}
			}
			if (request->argName(i) == "digiAuto")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
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
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						pos2RF = true;
				}
			}
			if (request->argName(i) == "digiPos2INET")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						pos2INET = true;
				}
			}
			if (request->argName(i) == "digiBcnEnable")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						bcnEN = true;
				}
			}
			// Filter
			if (request->argName(i) == "FilterMessage")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_MESSAGE;
				}
			}

			if (request->argName(i) == "FilterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_TELEMETRY;
				}
			}

			if (request->argName(i) == "FilterStatus")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_STATUS;
				}
			}

			if (request->argName(i) == "FilterWeather")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_WX;
				}
			}

			if (request->argName(i) == "FilterObject")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_OBJECT;
				}
			}

			if (request->argName(i) == "FilterItem")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_ITEM;
				}
			}

			if (request->argName(i) == "FilterQuery")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_QUERY;
				}
			}
			if (request->argName(i) == "FilterBuoy")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_BUOY;
				}
			}
			if (request->argName(i) == "FilterPosition")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_POSITION;
				}
			}
			if (request->argName(i) == "digiTimeStamp")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
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

			for (int x = 0; x < 5; x++)
			{
				{
					char *arg = allocateStringMemory(32);
					if (arg)
					{
						snprintf(arg, 32, "sensorCH%d", x);
						if (request->argName(i) == String(arg))
						{
							if (isValidNumber(request->arg(i)))
								config.digi_tlm_sensor[x] = request->arg(i).toInt();
						}
						free(arg);
					}
				}
				{
					char *arg = allocateStringMemory(32);
					if (arg)
					{
						snprintf(arg, 32, "param%d", x);
						if (request->argName(i) == String(arg))
						{
							if (request->arg(i) != "")
							{
								strcpy(config.digi_tlm_PARM[x], request->arg(i).c_str());
							}
						}
						free(arg);
					}
				}
				{
					char *arg = allocateStringMemory(32);
					if (arg)
					{
						snprintf(arg, 32, "unit%d", x);
						if (request->argName(i) == String(arg))
						{
							if (request->arg(i) != "")
							{
								strcpy(config.digi_tlm_UNIT[x], request->arg(i).c_str());
							}
						}
						free(arg);
					}
				}
				{
					char *arg = allocateStringMemory(32);
					if (arg)
					{
						snprintf(arg, 32, "precision%d", x);
						if (request->argName(i) == String(arg))
						{
							if (isValidNumber(request->arg(i)))
								config.digi_tlm_precision[x] = request->arg(i).toInt();
						}
						free(arg);
					}
				}
				{
					char *arg = allocateStringMemory(32);
					if (arg)
					{
						snprintf(arg, 32, "offset%d", x);
						if (request->argName(i) == String(arg))
						{
							if (isValidNumber(request->arg(i)))
								config.digi_tlm_offset[x] = request->arg(i).toFloat();
						}
						free(arg);
					}
				}
				for (int y = 0; y < 3; y++)
				{
					{
						char *arg = allocateStringMemory(32);
						if (arg)
						{
							snprintf(arg, 32, "eqns%d%c", x, (char)(y + 'a'));
							if (request->argName(i) == String(arg))
							{
								if (isValidNumber(request->arg(i)))
									config.digi_tlm_EQNS[x][y] = request->arg(i).toFloat();
							}
							free(arg);
						}
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
		String html_msg;
		if (saveConfiguration("/default.cfg", config))
		{
			html_msg = "Setup completed successfully";
			request->send(200, "text/html", html_msg); // send to someones browser when asked
		}
		else
		{
			html_msg = "Save config failed.";
			request->send(501, "text/html", html_msg); // Not Implemented
		}
	}
	else
	{
		// Allocate initial memory for HTML content
		char *html = allocateStringMemory(20000); // Start with 12KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		memset(html, 0, 20000);
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "document.getElementById(\"submitDIGI\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/digi',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n<script type=\"text/javascript\">\n");
		strcat(html, "function openWindowSymbol() {\n");
		strcat(html, "var i, l, options = [{\n");
		strcat(html, "value: 'first',\n");
		strcat(html, "text: 'First'\n");
		strcat(html, "}, {\n");
		strcat(html, "value: 'second',\n");
		strcat(html, "text: 'Second'\n");
		strcat(html, "}],\n");
		strcat(html, "newWindow = window.open(\"/symbol\", null, \"height=400,width=400,status=no,toolbar=no,menubar=no,titlebar=no,location=no\");\n");
		strcat(html, "}\n");

		strcat(html, "function setValue(symbol,table) {\n");
		strcat(html, "document.getElementById('digiSymbol').value = String.fromCharCode(symbol);\n");
		strcat(html, "if(table==1){\n document.getElementById('digiTable').value='/';\n");
		strcat(html, "}else if(table==2){\n document.getElementById('digiTable').value='\\\\';\n}\n");
		strcat(html, "document.getElementById('digiImgSymbol').src = \"http://aprs.dprns.com/symbols/icons/\"+symbol.toString()+'-'+table.toString()+'.png';\n");
		strcat(html, "\n}\n");
		strcat(html, "function calculatePHGR(){document.forms.formDIGI.texttouse.value=\"PHG\"+calcPower(document.forms.formDIGI.power.value)+calcHeight(document.forms.formDIGI.haat.value)+calcGain(document.forms.formDIGI.gain.value)+calcDirection(document.forms.formDIGI.direction.selectedIndex)}function Log2(e){return Math.log(e)/Math.log(2)}function calcPerHour(e){return e<10?e:String.fromCharCode(65+(e-10))}function calcHeight(e){return String.fromCharCode(48+Math.round(Log2(e/10),0))}function calcPower(e){if(e<1)return 0;if(e>=1&&e<4)return 1;if(e>=4&&e<9)return 2;if(e>=9&&e<16)return 3;if(e>=16&&e<25)return 4;if(e>=25&&e<36)return 5;if(e>=36&&e<49)return 6;if(e>=49&&e<64)return 7;if(e>=64&&e<81)return 8;if(e>=81)return 9}function calcDirection(e){if(e==\"0\")return\"0\";if(e==\"1\")return\"1\";if(e==\"2\")return\"2\";if(e==\"3\")return\"3\";if(e==\"4\")return\"4\";if(e==\"5\")return\"5\";if(e==\"6\")return\"6\";if(e==\"7\")return\"7\";if(e==\"8\")return\"8\"}function calcGain(e){return e>9?\"9\":e<0?\"0\":Math.round(e,0)}\n");
		strcat(html, "function selPrecision(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "x = document.getElementsByName(\"precision\"+idx)[0].value;\n");
		strcat(html, "document.getElementsByName(\"eqns\"+idx+\"b\")[0].value=1/Math.pow(10,x);\n");
		strcat(html, "}\n");
		strcat(html, "function selOffset(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "x = document.getElementsByName(\"offset\"+idx)[0].value;\n");
		strcat(html, "document.getElementsByName(\"eqns\"+idx+\"c\")[0].value=x*(-1);\n");
		strcat(html, "}\n");
		strcat(html, "</script>\n");

		/************************ DIGI Mode **************************/
		strcat(html, "<form id='formDIGI' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		// strcat(html, "<h2>[DIGI] Digital Repeater Mode</h2>\n");
		strcat(html, "<table>\n");
		// strcat(html, "<tr>\n");
		// strcat(html, "<th width=\"200\"><span><b>Setting</b></span></th>\n");
		// strcat(html, "<th><span><b>Value</b></span></th>\n");
		// strcat(html, "</tr>\n");
		strcat(html, "<th colspan=\"2\"><span><b>[DIGI] Digital Repeater Mode</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
		String digiEnFlag = "";
		if (config.digi_en)
			digiEnFlag = "checked";
		{
			char *temp_flag = allocateStringMemory(512);
			if (temp_flag)
			{
				snprintf(temp_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", digiEnFlag);
				strcat(html, temp_flag);
				free(temp_flag);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Auto Enable:</b></td>\n");
		String digiAutoFlag = "";
		if (config.digi_auto)
			digiAutoFlag = "checked";
		{
			char *temp_flag = allocateStringMemory(512);
			if (temp_flag)
			{
				snprintf(temp_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiAuto\" value=\"OK\" %s><span class=\"slider round\"></span></label> <i>*Automatic enable when APRS-IS disconnected</i></td>\n", digiAutoFlag);
				strcat(html, temp_flag);
				free(temp_flag);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station Callsign:</b></td>\n");
		{
			char *temp_value = allocateStringMemory(256);
			if (temp_value)
			{
				snprintf(temp_value, 256, "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"%s\" /></td>\n", config.digi_mycall);
				strcat(html, temp_value);
				free(temp_value);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station SSID:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mySSID\" id=\"mySSID\">\n");
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			char *temp_option = allocateStringMemory(256);
			if (temp_option)
			{
				if (config.digi_ssid == ssid)
				{
					snprintf(temp_option, 256, "<option value=\"%d\" selected>%d</option>\n", ssid, ssid);
				}
				else
				{
					snprintf(temp_option, 256, "<option value=\"%d\">%d</option>\n", ssid, ssid);
				}
				strcat(html, temp_option);
				free(temp_option);
			}
		}
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station Symbol:</b></td>\n");
		String table = "1";
		if (config.digi_symbol[0] == 47)
			table = "1";
		if (config.digi_symbol[0] == 92)
			table = "2";
		{
			char *temp_symbol = allocateStringMemory(512);
			if (temp_symbol)
			{
				snprintf(temp_symbol, 512, "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"digiTable\" name=\"digiTable\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"digiSymbol\" name=\"digiSymbol\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"digiImgSymbol\" onclick=\"openWindowSymbol();\" src=\"http://aprs.dprns.com/symbols/icons/%d-%s.png\"> <i>*Click icon for select symbol</i></td>\n",
						 config.digi_symbol[0], config.digi_symbol[1], (int)config.digi_symbol[1], table);
				strcat(html, temp_symbol);
				free(temp_symbol);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"digiPath\" id=\"digiPath\">\n");
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			char *temp_path = allocateStringMemory(256);
			if (temp_path)
			{
				if (config.digi_path == pthIdx)
				{
					snprintf(temp_path, 256, "<option value=\"%d\" selected>%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
				}
				else
				{
					snprintf(temp_path, 256, "<option value=\"%d\">%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
				}
				strcat(html, temp_path);
				free(temp_path);
			}
		}
		strcat(html, "</select></td>\n");
		// strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" id=\"digiPath\" name=\"digiPath\" type=\"text\" value=\"" + String(config.digi_path) + "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Text Comment:</b></td>\n");
		{
			char *temp_comment = allocateStringMemory(256);
			if (temp_comment)
			{
				snprintf(temp_comment, 256, "<td style=\"text-align: left;\"><input maxlength=\"25\" size=\"30\" id=\"digiComment\" name=\"digiComment\" type=\"text\" value=\"%s\" /></td>\n", config.digi_comment);
				strcat(html, temp_comment);
				free(temp_comment);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Text Status:</b></td>\n");
		{
			char *temp_status = allocateStringMemory(512);
			if (temp_status)
			{
				snprintf(temp_status, 512, "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"60\" id=\"digiStatus\" name=\"digiStatus\" type=\"text\" value=\"%s\" />  Interval:<input min=\"0\" max=\"3600\" step=\"1\" name=\"digiSTSInv\" type=\"number\" value=\"%d\" />Sec.</td>\n", config.digi_status, config.digi_sts_interval);
				strcat(html, temp_status);
				free(temp_status);
			}
		}
		strcat(html, "</tr>\n");

		{
			char *temp_delay = allocateStringMemory(512);
			if (temp_delay)
			{
				snprintf(temp_delay, 512, "<tr><td style=\"text-align: right;\"><b>Repeat Delay:</b></td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"100\" id=\"digiDelay\" name=\"digiDelay\" type=\"number\" value=\"%d\" /> mSec. <i>*0 is auto,Other random of delay time</i></td></tr>", config.digi_delay);
				strcat(html, temp_delay);
				free(temp_delay);
			}
		}

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Time Stamp:</b></td>\n");
		char timeStampFlag[10];
		if (config.digi_timestamp)
			strcpy(timeStampFlag, "checked");
		else
			strcpy(timeStampFlag, "");
		{
			char *temp_flag = allocateStringMemory(512);
			if (temp_flag)
			{
				snprintf(temp_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiTimeStamp\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", timeStampFlag);
				strcat(html, temp_flag);
				free(temp_flag);
			}
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td align=\"right\"><b>POSITION:</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");
		String digiBcnEnFlag = "";
		if (config.digi_bcn)
			digiBcnEnFlag = "checked";

		{
			char *temp_beacon = allocateStringMemory(512);
			if (temp_beacon)
			{
				snprintf(temp_beacon, 512, "<tr><td style=\"text-align: right;\">Beacon:</td><td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiBcnEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\">  Interval:<input min=\"0\" max=\"3600\" step=\"1\" id=\"digiPosInv\" name=\"digiPosInv\" type=\"number\" value=\"%d\" />Sec.</label></td></tr>", digiBcnEnFlag, config.digi_interval);
				strcat(html, temp_beacon);
				free(temp_beacon);
			}
		}
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
		{
			char *temp_pos = allocateStringMemory(512);
			if (temp_pos)
			{
				snprintf(temp_pos, 512, "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"digiPosSel\" value=\"0\" %s/>Fix <input type=\"radio\" name=\"digiPosSel\" value=\"1\" %s/>GPS </td></tr>\n",
						 digiPosFixFlag, digiPosGPSFlag);
				strcat(html, temp_pos);
				free(temp_pos);
			}
		}
		{
			char *temp_channel = allocateStringMemory(512);
			if (temp_channel)
			{
				snprintf(temp_channel, 512, "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"digiPos2RF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"digiPos2INET\" value=\"OK\" %s/>Internet </td></tr>\n",
						 digiPos2RFFlag, digiPos2INETFlag);
				strcat(html, temp_channel);
				free(temp_channel);
			}
		}
		{
			char *temp_lat = allocateStringMemory(512);
			if (temp_lat)
			{
				snprintf(temp_lat, 512, "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" id=\"digiPosLat\" name=\"digiPosLat\" type=\"number\" value=\"%.5f\" />degrees (positive for North, negative for South)</td></tr>\n", config.digi_lat);
				strcat(html, temp_lat);
				free(temp_lat);
			}
		}
		{
			char *temp_lon = allocateStringMemory(512);
			if (temp_lon)
			{
				snprintf(temp_lon, 512, "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" id=\"digiPosLon\" name=\"digiPosLon\" type=\"number\" value=\"%.5f\" />degrees (positive for East, negative for West)</td></tr>\n", config.digi_lon);
				strcat(html, temp_lon);
				free(temp_lon);
			}
		}
		{
			char *temp_alt = allocateStringMemory(512);
			if (temp_alt)
			{
				snprintf(temp_alt, 512, "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" id=\"digiPosAlt\" name=\"digiPosAlt\" type=\"number\" value=\"%.2f\" /> meter. *Value 0 is not send height</td></tr>\n", config.digi_alt);
				strcat(html, temp_alt);
				free(temp_alt);
			}
		}
		strcat(html, "</table></td>");
		strcat(html, "</tr>\n");
		delay(1);
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PHG:</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Radio TX Power</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"power\" id=\"power\">\n");
		strcat(html, "<option value=\"1\" selected>1</option>\n");
		strcat(html, "<option value=\"5\">5</option>\n");
		strcat(html, "<option value=\"10\">10</option>\n");
		strcat(html, "<option value=\"15\">15</option>\n");
		strcat(html, "<option value=\"25\">25</option>\n");
		strcat(html, "<option value=\"35\">35</option>\n");
		strcat(html, "<option value=\"50\">50</option>\n");
		strcat(html, "<option value=\"65\">65</option>\n");
		strcat(html, "<option value=\"80\">80</option>\n");
		strcat(html, "</select> Watts</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td style=\"text-align: right;\">Antenna Gain</td><td style=\"text-align: left;\"><input size=\"3\" min=\"0\" max=\"100\" step=\"0.1\" id=\"gain\" name=\"gain\" type=\"number\" value=\"6\" /> dBi</td></tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Height</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"haat\" id=\"haat\">\n");
		int k = 10;
		for (uint8_t w = 0; w < 10; w++)
		{
			{
				char *temp_opt = allocateStringMemory(256);
				if (temp_opt)
				{
					if (w == 0)
					{
						snprintf(temp_opt, 256, "<option value=\"%d\" selected>%d</option>\n", k, k);
					}
					else
					{
						snprintf(temp_opt, 256, "<option value=\"%d\">%d</option>\n", k, k);
					}
					strcat(html, temp_opt);
					free(temp_opt);
				}
			}
			k += k;
		}
		strcat(html, "</select> Feet</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Antenna/Direction</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"direction\" id=\"direction\">\n");
		strcat(html, "<option>Omni</option><option>NE</option><option>E</option><option>SE</option><option>S</option><option>SW</option><option>W</option><option>NW</option><option>N</option>\n");
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");
		{
			char *temp_phg = allocateStringMemory(512);
			if (temp_phg)
			{
				snprintf(temp_phg, 512, "<tr><td align=\"right\"><b>PHG Text</b></td><td align=\"left\"><input name=\"texttouse\" type=\"text\" size=\"6\" style=\"background-color: rgb(97, 239, 170);\" value=\"%s\"/> <input type=\"button\" value=\"Calculate PHG\" onclick=\"javascript:calculatePHGR()\" /></td></tr>\n", config.digi_phg);
				strcat(html, temp_phg);
				free(temp_phg);
			}
		}

		strcat(html, "</table></tr>");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Filter:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<fieldset id=\"FilterGrp\">\n");
		strcat(html, "<legend>Filter repeater</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">");
		strcat(html, "<tr style=\"background:unset;\">");

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterMessage\" type=\"checkbox\" value=\"OK\" %s/>Message</td>\n",
						 (config.digiFilter & FILTER_MESSAGE) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterStatus\" type=\"checkbox\" value=\"OK\" %s/>Status</td>\n",
						 (config.digiFilter & FILTER_STATUS) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterTelemetry\" type=\"checkbox\" value=\"OK\" %s/>Telemetry</td>\n",
						 (config.digiFilter & FILTER_TELEMETRY) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n",
						 (config.digiFilter & FILTER_WX) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterObject\" type=\"checkbox\" value=\"OK\" %s/>Object</td>\n",
						 (config.digiFilter & FILTER_OBJECT) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterItem\" type=\"checkbox\" value=\"OK\" %s/>Item</td>\n",
						 (config.digiFilter & FILTER_ITEM) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterQuery\" type=\"checkbox\" value=\"OK\" %s/>Query</td>\n",
						 (config.digiFilter & FILTER_QUERY) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterBuoy\" type=\"checkbox\" value=\"OK\" %s/>Buoy</td>\n",
						 (config.digiFilter & FILTER_BUOY) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		{
			char *temp_filter = allocateStringMemory(256);
			if (temp_filter)
			{
				snprintf(temp_filter, 256, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterPosition\" type=\"checkbox\" value=\"OK\" %s/>Position</td>\n",
						 (config.digiFilter & FILTER_POSITION) ? "checked" : "");
				strcat(html, temp_filter);
				free(temp_filter);
			}
		}

		strcat(html, "<td style=\"border:unset;\"></td>");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Telemetry:</b><br />(v=0->8280)</td>\n");
		strcat(html, "<td align=\"center\"><table>\n");
		{
			char *temp_intv = allocateStringMemory(512);
			if (temp_intv)
			{
				snprintf(temp_intv, 512, "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"1000\" step=\"1\" id=\"digiTlmInv\" name=\"digiTlmInv\" type=\"number\" value=\"%d\" /> *Number of packets interval,<i>Example: 0 not send,1 send every packet</i></label></td></tr>", config.digi_tlm_interval);
				strcat(html, temp_intv);
				free(temp_intv);
			}
		}
		for (int ax = 0; ax < 5; ax++)
		{
			{
				char *temp_ch = allocateStringMemory(256);
				if (temp_ch)
				{
					snprintf(temp_ch, 256, "<tr><td align=\"right\"><b>CH A%d:</b></td>\n", ax + 1);
					strcat(html, temp_ch);
					free(temp_ch);
				}
			}
			strcat(html, "<td align=\"center\">\n");
			strcat(html, "<table>");

			strcat(html, "<tr><td style=\"text-align: right;\">Sensor:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">CH: ");

			{
				char *temp_select = allocateStringMemory(256);
				if (temp_select)
				{
					snprintf(temp_select, 256, "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax, ax);
					strcat(html, temp_select);
					free(temp_select);
				}
			}

			for (uint8_t idx = 0; idx < 11; idx++)
			{
				char *temp_opt = allocateStringMemory(256);
				if (temp_opt)
				{
					if (idx == 0)
					{
						if (config.digi_tlm_sensor[ax] == idx)
						{
							snprintf(temp_opt, 256, "<option value=\"%d\" selected>NONE</option>\n", idx);
						}
						else
						{
							snprintf(temp_opt, 256, "<option value=\"%d\">NONE</option>\n", idx);
						}
					}
					else
					{
						if (config.digi_tlm_sensor[ax] == idx)
						{
							snprintf(temp_opt, 256, "<option value=\"%d\" selected>SENSOR#%d</option>\n", idx, idx);
						}
						else
						{
							snprintf(temp_opt, 256, "<option value=\"%d\">SENSOR#%d</option>\n", idx, idx);
						}
					}
					strcat(html, temp_opt);
					free(temp_opt);
				}
			}
			strcat(html, "</select></td>\n");

			{
				char *temp_param = allocateStringMemory(512);
				if (temp_param)
				{
					snprintf(temp_param, 512, "<td style=\"text-align: left;\">Name: <input maxlength=\"10\" size=\"8\" name=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.digi_tlm_PARM[ax]);
					strcat(html, temp_param);
					free(temp_param);
				}
			}

			{
				char *temp_unit = allocateStringMemory(512);
				if (temp_unit)
				{
					snprintf(temp_unit, 512, "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.digi_tlm_UNIT[ax]);
					strcat(html, temp_unit);
					free(temp_unit);
				}
			}

			{
				char *temp_prec = allocateStringMemory(512);
				if (temp_prec)
				{
					snprintf(temp_prec, 512, "<td style=\"text-align: left;\">Precision: <input min=\"0\" max=\"5\" step=\"1\" type=\"number\" style=\"width: 2em\" name=\"precision%d\" type=\"text\" value=\"%d\" onchange=\"selPrecision(%d)\"/></td></tr>\n", ax, config.digi_tlm_precision[ax], ax);
					strcat(html, temp_prec);
					free(temp_prec);
				}
			}

			{
				char *temp_eqns = allocateStringMemory(1024);
				if (temp_eqns)
				{
					snprintf(temp_eqns, 1024, "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%da\" type=\"number\" value=\"%.5f\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%db\" type=\"number\" value=\"%.5f\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%dc\" type=\"number\" value=\"%.5f\" /> (av<sup>2</sup>+bv+c) </td>\n",
							 ax, config.digi_tlm_EQNS[ax][0], ax, config.digi_tlm_EQNS[ax][1], ax, config.digi_tlm_EQNS[ax][2]);
					strcat(html, temp_eqns);
					free(temp_eqns);
				}
			}

			{
				char *temp_offset = allocateStringMemory(512);
				if (temp_offset)
				{
					snprintf(temp_offset, 512, "<td style=\"text-align: left;\">Offset: <input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" type=\"number\" name=\"offset%d\" type=\"text\" value=\"%.5f\" onchange=\"selOffset(%d)\" /></td></tr>\n", ax, config.digi_tlm_offset[ax], ax);
					strcat(html, temp_offset);
					free(temp_offset);
				}
			}

			strcat(html, "</table></td>");
			strcat(html, "</tr>\n");
		}
		strcat(html, "</table></td></tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitDIGI'  name=\"commitDIGI\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitDIGI\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");

		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
		response->addHeader("digi", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(html); // Free the allocated memory
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
		String html_msg;
		if (saveConfiguration("/default.cfg", config))
		{
			html_msg = "Setup completed successfully";
			request->send(200, "text/html", html_msg); // send to someones browser when asked
		}
		else
		{
			html_msg = "Save config failed.";
			request->send(501, "text/html", html_msg); // Not Implemented
		}
	}
	else
	{
		// Allocate initial memory for HTML content
		char *html = allocateStringMemory(26000); // Start with 8KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "document.getElementById(\"submitWX\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/wx',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");

		/************************ WX Mode **************************/
		strcat(html, "<form id='formWX' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>[WX] Weather Station</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
		String EnFlag = "";
		if (config.wx_en)
			EnFlag = "checked";
		{
			char *temp_flag = allocateStringMemory(512);
			if (temp_flag)
			{
				snprintf(temp_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", EnFlag);
				strcat(html, temp_flag);
				free(temp_flag);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station Callsign:</b></td>\n");
		{
			char *temp_callsign = allocateStringMemory(512);
			if (temp_callsign)
			{
				snprintf(temp_callsign, 512, "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"%s\" /></td>\n", config.wx_mycall);
				strcat(html, temp_callsign);
				free(temp_callsign);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station SSID:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mySSID\" id=\"mySSID\">\n");
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			{
				char *temp_option = allocateStringMemory(256);
				if (temp_option)
				{
					if (config.wx_ssid == ssid)
					{
						snprintf(temp_option, 256, "<option value=\"%d\" selected>%d</option>\n", ssid, ssid);
					}
					else
					{
						snprintf(temp_option, 256, "<option value=\"%d\">%d</option>\n", ssid, ssid);
					}
					strcat(html, temp_option);
					free(temp_option);
				}
			}
		}
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Object Name:</b></td>\n");
		{
			char *temp_object = allocateStringMemory(512);
			if (temp_object)
			{
				snprintf(temp_object, 512, "<td style=\"text-align: left;\"><input maxlength=\"9\" size=\"9\" name=\"Object\" type=\"text\" value=\"%s\" /><i> *If not used, leave it blank.In use 3-9 charactor</i></td>\n", config.wx_object);
				strcat(html, temp_object);
				free(temp_object);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"Path\" id=\"Path\">\n");
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			{
				char *temp_path = allocateStringMemory(256);
				if (temp_path)
				{
					if (config.wx_path == pthIdx)
					{
						snprintf(temp_path, 256, "<option value=\"%d\" selected>%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
					}
					else
					{
						snprintf(temp_path, 256, "<option value=\"%d\">%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
					}
					strcat(html, temp_path);
					free(temp_path);
				}
			}
		}
		strcat(html, "</select></td>\n");
		// strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" name=\"Path\" type=\"text\" value=\"" + String(config.wx_path) + "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Text Comment:</b></td>\n");
		{
			char *temp_comment = allocateStringMemory(512);
			if (temp_comment)
			{
				snprintf(temp_comment, 512, "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"50\" name=\"Comment\" type=\"text\" value=\"%s\" /></td>\n", config.wx_comment);
				strcat(html, temp_comment);
				free(temp_comment);
			}
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Time Stamp:</b></td>\n");
		char timeStampFlag[10];
		if (config.wx_timestamp)
			strcpy(timeStampFlag, "checked");
		else
			strcpy(timeStampFlag, "");
		{
			char *temp_flag = allocateStringMemory(512);
			if (temp_flag)
			{
				snprintf(temp_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wxTimeStamp\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", timeStampFlag);
				strcat(html, temp_flag);
				free(temp_flag);
			}
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td align=\"right\"><b>POSITION:</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");
		{
			char *temp_interval = allocateStringMemory(512);
			if (temp_interval)
			{
				snprintf(temp_interval, 512, "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" id=\"PosInv\" name=\"PosInv\" type=\"number\" value=\"%d\" />Sec.</td></tr>", config.wx_interval);
				strcat(html, temp_interval);
				free(temp_interval);
			}
		}
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
		{
			char *temp_location = allocateStringMemory(1024);
			if (temp_location)
			{
				snprintf(temp_location, 1024, "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"PosSel\" value=\"0\" %s/>Fix <input type=\"radio\" name=\"PosSel\" value=\"1\" %s/>GPS </td></tr>\n", PosFixFlag, PosGPSFlag);
				strcat(html, temp_location);
				free(temp_location);
			}
		}
		{
			char *temp_channel = allocateStringMemory(1024);
			if (temp_channel)
			{
				snprintf(temp_channel, 1024, "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"Pos2RF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"Pos2INET\" value=\"OK\" %s/>Internet </td></tr>\n", Pos2RFFlag, Pos2INETFlag);
				strcat(html, temp_channel);
				free(temp_channel);
			}
		}
		{
			char *temp_lat = allocateStringMemory(512);
			if (temp_lat)
			{
				snprintf(temp_lat, 512, "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" name=\"PosLat\" type=\"number\" value=\"%.5f\" />degrees (positive for North, negative for South)</td></tr>\n", config.wx_lat);
				strcat(html, temp_lat);
				free(temp_lat);
			}
		}
		{
			char *temp_lon = allocateStringMemory(512);
			if (temp_lon)
			{
				snprintf(temp_lon, 512, "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" name=\"PosLon\" type=\"number\" value=\"%.5f\" />degrees (positive for East, negative for West)</td></tr>\n", config.wx_lon);
				strcat(html, temp_lon);
				free(temp_lon);
			}
		}
		{
			char *temp_alt = allocateStringMemory(512);
			if (temp_alt)
			{
				snprintf(temp_alt, 512, "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" name=\"PosAlt\" type=\"number\" value=\"%.2f\" /> meter. *The altitude in meters(m) above sea level</td></tr>\n", config.wx_alt);
				strcat(html, temp_alt);
				free(temp_alt);
			}
		}
		strcat(html, "</table></td>");
		strcat(html, "</tr>\n");

		// strcat(html, "<tr>\n");
		// strcat(html, "<td align=\"right\"><b>PORT:</b></td>\n");
		// strcat(html, "<td style=\"text-align: left;\">\n");
		// strcat(html, "<select name=\"channel\" id=\"channel\">\n");
		// for (int i = 0; i < 5; i++)
		// {
		// 	if (config.wx_channel == i)
		// 		strcat(html, "<option value=\"" + String(i) + "\" selected>" + String(WX_PORT[i]) + " </option>\n");
		// 	else
		// 		strcat(html, "<option value=\"" + String(i) + "\" >" + String(WX_PORT[i]) + " </option>\n");
		// }
		// strcat(html, "</select>\n");
		// strcat(html, "</td>\n");
		// strcat(html, "</tr>\n");
		/************************ Sensor Config Mode **************************/
		// strcat(html, "<form id='formSENSOR' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		// strcat(html, "<table>\n");
		// strcat(html, "<th colspan=\"2\"><span><b>Sensor Config</b></span></th>\n");

		strcat(html, "<tr><td align=\"right\"><b>SENSOR:<br />Selection</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");

		for (int ax = 0; ax < WX_SENSOR_NUM; ax++)
		{
			{
				char *temp_sensor = allocateStringMemory(512);
				if (temp_sensor)
				{
					snprintf(temp_sensor, 512, "<tr><td align=\"right\"><b>%s:</b> \n", WX_SENSOR[ax]);
					strcat(html, temp_sensor);
					free(temp_sensor);
				}
			}
			EnFlag = "";
			if (config.wx_sensor_enable[ax])
				EnFlag = "checked";
			{
				char *temp_checkbox = allocateStringMemory(512);
				if (temp_checkbox)
				{
					snprintf(temp_checkbox, 512, "<label class=\"switch\"><input type=\"checkbox\" name=\"senEn%d\" value=\"OK\" %s><span class=\"slider round\"></span></label>", ax, EnFlag);
					strcat(html, temp_checkbox);
					free(temp_checkbox);
				}
			}
			strcat(html, "</td>\n");

			// strcat(html, "<td style=\"text-align: lefe;\">Sensor:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">Sensor Channel: ");
			{
				char *temp_select = allocateStringMemory(256);
				if (temp_select)
				{
					snprintf(temp_select, 256, "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax, ax);
					strcat(html, temp_select);
					free(temp_select);
				}
			}
			for (uint8_t idx = 0; idx < 11; idx++)
			{
				{
					char *temp_option = allocateStringMemory(256);
					if (temp_option)
					{
						if (idx == 0)
						{
							if (config.wx_sensor_ch[ax] == idx)
							{
								snprintf(temp_option, 256, "<option value=\"%d\" selected>NONE</option>\n", idx);
							}
							else
							{
								snprintf(temp_option, 256, "<option value=\"%d\">NONE</option>\n", idx);
							}
						}
						else
						{
							if (config.wx_sensor_ch[ax] == idx)
							{
								snprintf(temp_option, 256, "<option value=\"%d\" selected>SENSOR#%d</option>\n", idx, idx);
							}
							else
							{
								snprintf(temp_option, 256, "<option value=\"%d\">SENSOR#%d</option>\n", idx, idx);
							}
						}
						strcat(html, temp_option);
						free(temp_option);
					}
				}
			}
			strcat(html, "</select>\n");
			String avgFlag = "";
			String sampleFlag = "";
			if (config.wx_sensor_avg[ax])
				avgFlag = "checked=\"checked\"";
			else
				sampleFlag = "checked=\"checked\"";
			{
				char *temp_radio = allocateStringMemory(512);
				if (temp_radio)
				{
					snprintf(temp_radio, 512, "<input type=\"radio\" name=\"avgSel%d\" value=\"0\" %s/>Sample <input type=\"radio\" name=\"avgSel%d\" value=\"1\" %s/>Average", ax, sampleFlag, ax, avgFlag);
					strcat(html, temp_radio);
					free(temp_radio);
				}
			}
			strcat(html, "</td></tr>");
		}
		strcat(html, "</table></td></tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitWX'  name=\"commitWX\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitWX\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");

		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
		response->addHeader("Weather", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(html); // Free the allocated memory
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
		String html_msg;
		if (saveConfiguration("/default.cfg", config))
		{
			html_msg = "Setup completed successfully";
			request->send(200, "text/html", html_msg); // send to someones browser when asked
		}
		else
		{
			html_msg = "Save config failed.";
			request->send(501, "text/html", html_msg); // Not Implemented
		}
	}
	else
	{
		// Allocate initial memory for HTML content
		char *html = allocateStringMemory(18000); // Start with 8KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "document.getElementById(\"submitTLM\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/tlm',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");

		/************************ TLM Mode **************************/
		strcat(html, "<form id='formTLM' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>System Telemetry</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
		char EnFlag[10] = "";
		if (config.tlm0_en)
			strcpy(EnFlag, "checked");
		{
			char *temp_flag = allocateStringMemory(512);
			if (temp_flag)
			{
				snprintf(temp_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", EnFlag);
				strcat(html, temp_flag);
				free(temp_flag);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		{
			char *temp_callsign = allocateStringMemory(512);
			if (temp_callsign)
			{
				snprintf(temp_callsign, 512, "<td align=\"right\"><b>Station Callsign:</b></td>\n<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"%s\" /></td>\n", config.tlm0_mycall);
				strcat(html, temp_callsign);
				free(temp_callsign);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station SSID:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mySSID\" id=\"mySSID\">\n");
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			char *temp_option = allocateStringMemory(256);
			if (temp_option)
			{
				if (config.tlm0_ssid == ssid)
				{
					snprintf(temp_option, 256, "<option value=\"%d\" selected>%d</option>\n", ssid, ssid);
				}
				else
				{
					snprintf(temp_option, 256, "<option value=\"%d\">%d</option>\n", ssid, ssid);
				}
				strcat(html, temp_option);
				free(temp_option);
			}
		}
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"Path\" id=\"Path\">\n");
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			{
				char *temp_path = allocateStringMemory(256);
				if (temp_path)
				{
					if (config.tlm0_path == pthIdx)
					{
						snprintf(temp_path, 256, "<option value=\"%d\" selected>%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
					}
					else
					{
						snprintf(temp_path, 256, "<option value=\"%d\">%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
					}
					strcat(html, temp_path);
					free(temp_path);
				}
			}
		}
		strcat(html, "</select></td>\n");
		// strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" name=\"Path\" type=\"text\" value=\"" + String(config.tlm0_path) + "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		{
			char *temp_comment = allocateStringMemory(512);
			if (temp_comment)
			{
				snprintf(temp_comment, 512, "<td align=\"right\"><b>Text Comment:</b></td>\n<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"50\" name=\"Comment\" type=\"text\" value=\"%s\" /></td>\n", config.tlm0_comment);
				strcat(html, temp_comment);
				free(temp_comment);
			}
		}
		strcat(html, "</tr>\n");

		{
			char *temp_intervals = allocateStringMemory(1024);
			if (temp_intervals)
			{
				snprintf(temp_intervals, 1024, "<tr><td style=\"text-align: right;\">Info Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" name=\"infoInv\" type=\"number\" value=\"%d\" />Sec.</td></tr>", config.tlm0_info_interval);
				strcat(html, temp_intervals);
				snprintf(temp_intervals, 1024, "<tr><td style=\"text-align: right;\">Data Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" name=\"dataInv\" type=\"number\" value=\"%d\" />Sec.</td></tr>", config.tlm0_data_interval);
				strcat(html, temp_intervals);
				free(temp_intervals);
			}
		}

		char Pos2RFFlag[10] = "";
		char Pos2INETFlag[10] = "";
		if (config.tlm0_2rf)
			strcpy(Pos2RFFlag, "checked");
		if (config.tlm0_2inet)
			strcpy(Pos2INETFlag, "checked");
		{
			char *temp_channels = allocateStringMemory(1024);
			if (temp_channels)
			{
				snprintf(temp_channels, 1024, "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"Pos2RF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"Pos2INET\" value=\"OK\" %s/>Internet </td></tr>\n", Pos2RFFlag, Pos2INETFlag);
				strcat(html, temp_channels);
				free(temp_channels);
			}
		}

		// strcat(html, "<tr>\n");
		// strcat(html, "<td align=\"right\"><b>Time Stamp:</b></td>\n");
		// String timeStampFlag = "";
		// if (config.wx_timestamp)
		// 	timeStampFlag = "checked";
		// strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wxTimeStamp\" value=\"OK\" " + timeStampFlag + "><span class=\"slider round\"></span></label></td>\n");
		// strcat(html, "</tr>\n");
		for (int ax = 0; ax < 5; ax++)
		{
			{
				char *temp_channel_a = allocateStringMemory(512);
				if (temp_channel_a)
				{
					snprintf(temp_channel_a, 512, "<tr><td align=\"right\"><b>Channel A%d:</b></td>\n", ax + 1);
					strcat(html, temp_channel_a);
					free(temp_channel_a);
				}
			}
			strcat(html, "<td align=\"center\">\n");
			strcat(html, "<table>");

			// strcat(html, "<tr><td style=\"text-align: right;\">Name:</td><td style=\"text-align: center;\"><i>Sensor Type</i></td><td style=\"text-align: center;\"><i>Parameter</i></td><td style=\"text-align: center;\"><i>Unit</i></td></tr>\n");

			strcat(html, "<tr><td style=\"text-align: right;\">Type/Name:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">Sensor Type: ");
			{
				char *temp_select = allocateStringMemory(256);
				if (temp_select)
				{
					snprintf(temp_select, 256, "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax, ax);
					strcat(html, temp_select);
					free(temp_select);
				}
			}
			for (uint8_t idx = 0; idx < SYSTEM_LEN; idx++)
			{
				{
					char *temp_option = allocateStringMemory(256);
					if (temp_option)
					{
						if (config.tml0_data_channel[ax] == idx)
						{
							snprintf(temp_option, 256, "<option value=\"%d\" selected>%s</option>\n", idx, SYSTEM_NAME[idx]);
						}
						else
						{
							snprintf(temp_option, 256, "<option value=\"%d\">%s</option>\n", idx, SYSTEM_NAME[idx]);
						}
						strcat(html, temp_option);
						free(temp_option);
					}
				}
			}
			strcat(html, "</select></td>\n");

			{
				char *temp_param = allocateStringMemory(512);
				if (temp_param)
				{
					snprintf(temp_param, 512, "<td style=\"text-align: left;\">Parameter: <input maxlength=\"10\" size=\"8\" name=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.tlm0_PARM[ax]);
					strcat(html, temp_param);
					free(temp_param);
				}
			}
			{
				char *temp_unit = allocateStringMemory(512);
				if (temp_unit)
				{
					snprintf(temp_unit, 512, "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.tlm0_UNIT[ax]);
					strcat(html, temp_unit);
					free(temp_unit);
				}
			}
			strcat(html, "</tr>\n");
			{
				char *temp_eqns = allocateStringMemory(1024);
				if (temp_eqns)
				{
					snprintf(temp_eqns, 1024, "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.0001\" name=\"eqns%da\" type=\"number\" value=\"%.3f\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.0001\" name=\"eqns%db\" type=\"number\" value=\"%.3f\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.0001\" name=\"eqns%dc\" type=\"number\" value=\"%.3f\" /> (av<sup>2</sup>+bv+c)</td></tr>\n",
							 ax, config.tlm0_EQNS[ax][0], ax, config.tlm0_EQNS[ax][1], ax, config.tlm0_EQNS[ax][2]);
					strcat(html, temp_eqns);
					free(temp_eqns);
				}
			}
			strcat(html, "</table></td>");
			strcat(html, "</tr>\n");
		}

		uint8_t b = 1;
		for (int ax = 0; ax < 8; ax++)
		{
			{
				char *temp_channel_b = allocateStringMemory(512);
				if (temp_channel_b)
				{
					snprintf(temp_channel_b, 512, "<tr><td align=\"right\"><b>Channel B%d:</b></td>\n", ax + 1);
					strcat(html, temp_channel_b);
					free(temp_channel_b);
				}
			}
			strcat(html, "<td align=\"center\">\n");
			strcat(html, "<table>");

			// strcat(html, "<tr><td style=\"text-align: right;\">Type/Name:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">Type: ");
			{
				char *temp_select_b = allocateStringMemory(256);
				if (temp_select_b)
				{
					snprintf(temp_select_b, 256, "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax + 5, ax);
					strcat(html, temp_select_b);
					free(temp_select_b);
				}
			}
			for (uint8_t idx = 0; idx < SYSTEM_BIT_LEN; idx++)
			{
				{
					char *temp_option_b = allocateStringMemory(256);
					if (temp_option_b)
					{
						if (config.tml0_data_channel[ax + 5] == idx)
						{
							snprintf(temp_option_b, 256, "<option value=\"%d\" selected>%s</option>\n", idx, SYSTEM_BITS_NAME[idx]);
						}
						else
						{
							snprintf(temp_option_b, 256, "<option value=\"%d\">%s</option>\n", idx, SYSTEM_BITS_NAME[idx]);
						}
						strcat(html, temp_option_b);
						free(temp_option_b);
					}
				}
			}
			strcat(html, "</select></td>\n");

			{
				char *temp_param_b = allocateStringMemory(512);
				if (temp_param_b)
				{
					snprintf(temp_param_b, 512, "<td style=\"text-align: left;\">Parameter: <input maxlength=\"10\" size=\"8\" name=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax + 5, config.tlm0_PARM[ax + 5]);
					strcat(html, temp_param_b);
					free(temp_param_b);
				}
			}
			{
				char *temp_unit_b = allocateStringMemory(512);
				if (temp_unit_b)
				{
					snprintf(temp_unit_b, 512, "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax + 5, config.tlm0_UNIT[ax + 5]);
					strcat(html, temp_unit_b);
					free(temp_unit_b);
				}
			}
			char LowFlag[20] = "", HighFlag[20] = "";
			if (config.tlm0_BITS_Active & b)
				strcpy(HighFlag, "checked=\"checked\"");
			else
				strcpy(LowFlag, "checked=\"checked\"");
			{
				char *temp_radio_b = allocateStringMemory(512);
				if (temp_radio_b)
				{
					snprintf(temp_radio_b, 512, "<td style=\"text-align: left;\"> Active:<input type=\"radio\" name=\"bitact%d\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"bitact%d\" value=\"1\" %s/>HIGH </td>\n", ax, LowFlag, ax, HighFlag);
					strcat(html, temp_radio_b);
					free(temp_radio_b);
				}
			}
			strcat(html, "</tr>\n");
			// strcat(html, "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax + 1) + "a\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][0], 3) + "\" />  b:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax + 1) + "b\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][1], 3) + "\" /> c:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax + 1) + "c\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][2], 3) + "\" /> (av<sup>2</sup>+bv+c)</td></tr>\n";
			strcat(html, "</table></td>");
			strcat(html, "</tr>\n");
			b <<= 1;
		}

		// strcat(html, "<tr><td align=\"right\"><b>Parameter Name:</b></td>\n";
		// strcat(html, "<td align=\"center\">\n";
		// strcat(html, "<table>";

		// // strcat(html, "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" name=\"PosLat\" type=\"number\" value=\"" + String(config.wx_lat, 5) + "\" />degrees (positive for North, negative for South)</td></tr>\n";
		// // strcat(html, "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" name=\"PosLon\" type=\"number\" value=\"" + String(config.wx_lon, 5) + "\" />degrees (positive for East, negative for West)</td></tr>\n";
		// // strcat(html, "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" name=\"PosAlt\" type=\"number\" value=\"" + String(config.wx_alt, 2) + "\" /> meter. *Value 0 is not send height</td></tr>\n";
		// strcat(html, "</table></td>";
		// strcat(html, "</tr>\n";
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitTLM'  name=\"commitTLM\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitTLM\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");

		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
		response->addHeader("Telemetry", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(html); // Free the allocated memory
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
		// vTaskSuspend(taskSensorHandle);
		for (int x = 0; x < SENSOR_NUMBER; x++)
		{
			config.sensor[x].enable = false;
		}
		for (int i = 0; i < request->args(); i++)
		{
			// log_d("Arg %s: %s", request->argName(i).c_str(), request->arg(i).c_str());
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

		log_d("Sensor Config Updated.");
		String html_msg;
		if (saveConfiguration("/default.cfg", config))
		{
			html_msg = "Setup completed successfully";
			request->send(200, "text/html", html_msg); // send to someones browser when asked
		}
		else
		{
			html_msg = "Save config failed.";
			request->send(501, "text/html", html_msg); // Not Implemented
		}
		// vTaskResume(taskSensorHandle);
	}
	else
	{
		// Allocate initial memory for HTML content
		char *html = allocateStringMemory(25000); // Start with 8KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "document.getElementById(\"submitSENSOR\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/sensor',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "function setElm(name,val) {\n");
		strcat(html, "document.getElementById(name).value=val;\n");
		strcat(html, "};\n");
		strcat(html, "function selSensorType(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "var parm=\"param\"+idx;\n");
		strcat(html, "var unit=\"unit\"+idx;\n");
		strcat(html, "x = document.getElementById(\"sensorCH\"+idx).value;\n");
		strcat(html, "if (x==1) {\n");
		strcat(html, "setElm(parm,\"Co2\");");
		strcat(html, "setElm(unit,\"ppm\");\n");
		strcat(html, "}else if (x==2) {\n");
		strcat(html, "setElm(parm,\"CH2O\");");
		strcat(html, "setElm(unit,\"μg/m³\");\n");
		strcat(html, "}else if (x==3) {\n");
		strcat(html, "setElm(parm,\"TVOC\");");
		strcat(html, "setElm(unit,\"μg/m³\");\n");
		strcat(html, "}else if (x==4) {\n");
		strcat(html, "setElm(parm,\"PM2.5\");");
		strcat(html, "setElm(unit,\"μg/m³\");\n");
		strcat(html, "}else if (x==5) {\n");
		strcat(html, "setElm(parm,\"PM10.0\");");
		strcat(html, "setElm(unit,\"μg/m³\");\n");
		strcat(html, "}else if (x==6) {\n");
		strcat(html, "setElm(parm,\"Temperature\");");
		strcat(html, "setElm(unit,\"°C\");\n");
		strcat(html, "}else if (x==7) {\n");
		strcat(html, "setElm(parm,\"Humidity\");");
		strcat(html, "setElm(unit,\"%RH\");\n");
		strcat(html, "}else if (x==8) {\n");
		strcat(html, "setElm(parm,\"Pressure\");");
		strcat(html, "setElm(unit,\"hPa\");\n");
		strcat(html, "}else if (x==9) {\n");
		strcat(html, "setElm(parm,\"WindSpeed\");");
		strcat(html, "setElm(unit,\"kPh\");\n");
		strcat(html, "}else if (x==10) {\n");
		strcat(html, "setElm(parm,\"WindCourse\");");
		strcat(html, "setElm(unit,\"°\");\n");
		strcat(html, "}else if (x==11) {\n");
		strcat(html, "setElm(parm,\"Rain\");");
		strcat(html, "setElm(unit,\"mm\");\n");
		strcat(html, "}else if (x==12) {\n");
		strcat(html, "setElm(parm,\"Luminosity\");");
		strcat(html, "setElm(unit,\"W/m³\");\n");
		strcat(html, "}else if (x==13) {\n");
		strcat(html, "setElm(parm,\"SoilTemp\");");
		strcat(html, "setElm(unit,\"°C\");\n");
		strcat(html, "}else if (x==14) {\n");
		strcat(html, "setElm(parm,\"SoilMoisture\");");
		strcat(html, "setElm(unit,\"%VWC\");\n");
		strcat(html, "}else if (x==15) {\n");
		strcat(html, "setElm(parm,\"WaterTemp\");");
		strcat(html, "setElm(unit,\"°C\");\n");
		strcat(html, "}else if (x==16) {\n");
		strcat(html, "setElm(parm,\"WaterTDS\");");
		strcat(html, "setElm(unit,\" \");\n");
		strcat(html, "}else if (x==17) {\n");
		strcat(html, "setElm(parm,\"WaterLevel\");");
		strcat(html, "setElm(unit,\"mm\");\n");
		strcat(html, "}else if (x==18) {\n");
		strcat(html, "setElm(parm,\"WaterFlow\");");
		strcat(html, "setElm(unit,\"L/min\");\n");
		strcat(html, "}else if (x==19) {\n");
		strcat(html, "setElm(parm,\"Voltage\");");
		strcat(html, "setElm(unit,\"V\");\n");
		strcat(html, "}else if (x==20) {\n");
		strcat(html, "setElm(parm,\"Current\");");
		strcat(html, "setElm(unit,\"A\");\n");
		strcat(html, "}else if (x==21) {\n");
		strcat(html, "setElm(parm,\"Power\");");
		strcat(html, "setElm(unit,\"W\");\n");
		strcat(html, "}else if (x==22) {\n");
		strcat(html, "setElm(parm,\"Energy\");");
		strcat(html, "setElm(unit,\"Wh\");\n");
		strcat(html, "}else if (x==23) {\n");
		strcat(html, "setElm(parm,\"Frequency\");");
		strcat(html, "setElm(unit,\"Hz\");\n");
		strcat(html, "}else if (x==24) {\n");
		strcat(html, "setElm(parm,\"PF\");");
		strcat(html, "setElm(unit,\" \");\n");
		strcat(html, "}else if (x==25) {\n");
		strcat(html, "setElm(parm,\"Satellite\");");
		strcat(html, "setElm(unit,\" \");\n");
		strcat(html, "}else if (x==26) {\n");
		strcat(html, "setElm(parm,\"HDOP\");");
		strcat(html, "setElm(unit,\" \");\n");
		strcat(html, "}else if (x==27) {\n");
		strcat(html, "setElm(parm,\"Battery\");");
		strcat(html, "setElm(unit,\"V\");\n");
		strcat(html, "}else if (x==28) {\n");
		strcat(html, "setElm(parm,\"BattLevel\");");
		strcat(html, "setElm(unit,\"%\");\n");
		strcat(html, "}\n}\n");

		strcat(html, "function selSensor(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "x = document.getElementById(\"sensorP\"+idx).value;\n");
		strcat(html, "if (x>=10 && x<=13) {\n");
#ifdef TTGO_T_Beam_S3_SUPREME_V3
		strcat(html, "document.getElementById(\"address\"+idx).value=119;\n");
#else
		strcat(html, "document.getElementById(\"address\"+idx).value=118;\n");
#endif
		strcat(html, "}else if (x==16 || x==17) {\n");
		strcat(html, "document.getElementById(\"address\"+idx).value=90;\n");
		strcat(html, "}else if (x==23) {\n");
		strcat(html, "document.getElementById(\"address\"+idx).value=1;\n");
		strcat(html, "}else if (x==24 || x==25) {\n");
		strcat(html, "document.getElementById(\"address\"+idx).value=1000;\n");
		strcat(html, "}else{\n");
		strcat(html, "document.getElementById(\"address\"+idx).value=0;\n");
		strcat(html, "}\n}\n");
		strcat(html, "</script>\n");

		/************************ Sensor Monitor **************************/
		// strcat(html, "<form id='formSENSOR' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"5\"><span><b>Sensor Monitor</b></span></th>\n");
		int ax = 0;
		for (int r = 0; r < 3; r++)
		{
			strcat(html, "<tr>\n");
			for (int c = 0; c < 5; c++)
			{
				strcat(html, "<td align=\"center\">\n");
				if (config.sensor[ax].enable)
				{
					{
						char *temp_fieldset = allocateStringMemory(256);
						if (temp_fieldset)
						{
							snprintf(temp_fieldset, 256, "<fieldset id=\"SenGrp%d\">\n", ax + 1);
							strcat(html, temp_fieldset);
							free(temp_fieldset);
						}
					}
				}
				else
				{
					{
						char *temp_fieldset = allocateStringMemory(256);
						if (temp_fieldset)
						{
							snprintf(temp_fieldset, 256, "<fieldset id=\"SenGrp%d\" disabled>\n", ax + 1);
							strcat(html, temp_fieldset);
							free(temp_fieldset);
						}
					}
				}

				{
					char *temp_legend = allocateStringMemory(512);
					if (temp_legend)
					{
						snprintf(temp_legend, 512, "<legend>SEN#%d-%s</legend>\n", ax + 1, config.sensor[ax].parm);
						strcat(html, temp_legend);
						free(temp_legend);
					}
				}
				{
					char *temp_input = allocateStringMemory(512);
					if (temp_input)
					{
						snprintf(temp_input, 512, "<input id=\"sVal%d\" style=\"text-align:right;\" size=\"5\" type=\"text\" value=\"%.2f\" readonly/> %s\n", ax, sen[ax].sample, config.sensor[ax].unit);
						strcat(html, temp_input);
						free(temp_input);
					}
				}
				strcat(html, "</td>\n");
				ax++;
				if (ax >= SENSOR_NUMBER)
					break;
			}
			strcat(html, "</tr>\n");
			if (ax >= SENSOR_NUMBER)
				break;
		}
		strcat(html, "</table>< /br>\n");

		/************************ Sensor Config Mode **************************/
		strcat(html, "<form id='formSENSOR' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Sensor Config</b></span></th>\n");
		String EnFlag = "";

		for (int ax = 0; ax < SENSOR_NUMBER; ax++)
		{
			{
				char *temp_sensor = allocateStringMemory(256);
				if (temp_sensor)
				{
					snprintf(temp_sensor, 256, "<tr><td align=\"right\"><b>SENSOR#%d:</b><br />\n", ax + 1);
					strcat(html, temp_sensor);
					free(temp_sensor);
				}
			}
			EnFlag = "";
			if (config.sensor[ax].enable)
				EnFlag = "checked";
			{
				char *temp_checkbox = allocateStringMemory(256);
				if (temp_checkbox)
				{
					snprintf(temp_checkbox, 256, "<label class=\"switch\"><input type=\"checkbox\" name=\"En%d\" value=\"OK\" %s><span class=\"slider round\"></span></label>", ax, EnFlag);
					strcat(html, temp_checkbox);
					free(temp_checkbox);
				}
			}
			strcat(html, "</td><td align=\"center\">\n");
			strcat(html, "<table>");

			strcat(html, "<tr><td style=\"text-align: right;\">Type:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">");
			{
				char *temp_select = allocateStringMemory(256);
				if (temp_select)
				{
					snprintf(temp_select, 256, "<select name=\"sensorCH%d\" id=\"sensorCH%d\" onchange=\"selSensorType(%d)\">\n", ax, ax, ax);
					strcat(html, temp_select);
					free(temp_select);
				}
			}
			// for (uint8_t idx = 0; idx < SENSOR_NAME_NUM; idx++)
			// {
			// 	if (config.sensor[ax].type == idx)
			// 	{
			// 		strcat(html, "<option value=\"" + String(idx) + "\" selected>" + String(SENSOR_NAME[idx]) + "</option>\n");
			// 	}
			// 	else
			// 	{
			// 		strcat(html, "<option value=\"" + String(idx) + "\">" + String(SENSOR_NAME[idx]) + "</option>\n");
			// 	}
			// }
			strcat(html, "</select></td>\n");

			{
				char *temp_name = allocateStringMemory(512);
				if (temp_name)
				{
					snprintf(temp_name, 512, "<td style=\"text-align: left;\">Name: <input maxlength=\"15\" size=\"15\" name=\"param%d\" id=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax, ax, config.sensor[ax].parm);
					strcat(html, temp_name);
					free(temp_name);
				}
			}
			{
				char *temp_unit = allocateStringMemory(512);
				if (temp_unit)
				{
					snprintf(temp_unit, 512, "<td style=\"text-align: left;\">Unit: <input maxlength=\"10\" size=\"5\" name=\"unit%d\" id=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax, ax, config.sensor[ax].unit);
					strcat(html, temp_unit);
					free(temp_unit);
				}
			}
			strcat(html, "</tr>\n");
			// strcat(html, "<tr><td style=\"text-align: right;\">Port:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax) + "a\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[0], 3) + "\" />  b:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax) + "b\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[1], 3) + "\" /> c:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax) + "c\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[2], 3) + "\" /> (av<sup>2</sup>+bv+c)</td></tr>\n";
			{
				char *temp_port = allocateStringMemory(256);
				if (temp_port)
				{
					snprintf(temp_port, 256, "<tr><td style=\"text-align: right;\">PORT:</td>\n<td style=\"text-align: left;\">\n<select name=\"sensorP%d\" id=\"sensorP%d\" onchange=\"selSensor(%d)\">\n", ax, ax, ax);
					strcat(html, temp_port);
					free(temp_port);
				}
			}
			// for (uint8_t idx = 0; idx < SENSOR_PORT_NUM; idx++)
			// {
			// 	if (config.sensor[ax].port == idx)
			// 	{
			// 		strcat(html, "<option value=\"" + String(idx) + "\" selected>" + String(SENSOR_PORT[idx]) + "</option>\n";
			// 	}
			// 	else
			// 	{
			// 		strcat(html, "<option value=\"" + String(idx) + "\">" + String(SENSOR_PORT[idx]) + "</option>\n";
			// 	}
			// }
			strcat(html, "</select></td>\n");
			{
				char *temp_addr = allocateStringMemory(512);
				if (temp_addr)
				{
					snprintf(temp_addr, 512, "<td style=\"text-align: left;\">Addr/Reg/GPIO: <input style=\"text-align:right;\" min=\"0\" max=\"6500\" step=\"1\" name=\"address%d\" id=\"address%d\" type=\"number\" value=\"%d\" /></td>\n", ax, ax, config.sensor[ax].address);
					strcat(html, temp_addr);
					free(temp_addr);
				}
			}
			{
				char *temp_sample = allocateStringMemory(512);
				if (temp_sample)
				{
					snprintf(temp_sample, 512, "<td style=\"text-align: left;\">Sample: <input style=\"text-align:right;\" min=\"0\" max=\"9999\" step=\"1\" name=\"sample%d\" type=\"number\" value=\"%d\" />Sec.\n", ax, config.sensor[ax].samplerate);
					strcat(html, temp_sample);
					snprintf(temp_sample, 512, "Average: <input style=\"text-align:right;\" min=\"0\" max=\"999\" step=\"1\" name=\"avg%d\" type=\"number\" value=\"%d\" />Sec.</td></tr>\n", ax, config.sensor[ax].averagerate);
					strcat(html, temp_sample);
					free(temp_sample);
				}
			}
			{
				char *temp_eqns = allocateStringMemory(1024);
				if (temp_eqns)
				{
					snprintf(temp_eqns, 1024, "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-999\" max=\"999\" step=\"0.00001\" name=\"eqns%da\" type=\"number\" value=\"%.5f\" />  b:<input min=\"-999\" max=\"999\" step=\"0.00001\" name=\"eqns%db\" type=\"number\" value=\"%.5f\" /> c:<input min=\"-999\" max=\"999\" step=\"0.00001\" name=\"eqns%dc\" type=\"number\" value=\"%.5f\" /> (av<sup>2</sup>+bv+c)</td></tr>\n",
							 ax, config.sensor[ax].eqns[0], ax, config.sensor[ax].eqns[1], ax, config.sensor[ax].eqns[2]);
					strcat(html, temp_eqns);
					free(temp_eqns);
				}
			}
			strcat(html, "</table></td>");
			strcat(html, "</tr>\n");
		}

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitSENSOR'  name=\"commitSENSOR\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitSENSOR\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");

		strcat(html, "<script type=\"text/javascript\">\n");
		strcat(html, "if (typeof typeArry === 'undefined'){let typeArry = [];};\n");
		strcat(html, "typeArry = new Array(");
		for (uint8_t idx = 0; idx < SENSOR_NAME_NUM; idx++)
		{
			{
				char *temp_array_item = allocateStringMemory(256);
				if (temp_array_item)
				{
					snprintf(temp_array_item, 256, "'%s'", SENSOR_NAME[idx]);
					strcat(html, temp_array_item);
					if (idx < SENSOR_NAME_NUM - 1)
						strcat(html, ",");
					free(temp_array_item);
				}
			}
		}
		strcat(html, ");\n");
		strcat(html, "if (typeof portArry === 'undefined'){let portArry = [];};\n");
		strcat(html, "portArry = new Array(");
		for (uint8_t idx = 0; idx < SENSOR_PORT_NUM; idx++)
		{
			{
				char *temp_port_item = allocateStringMemory(256);
				if (temp_port_item)
				{
					snprintf(temp_port_item, 256, "'%s'", SENSOR_PORT[idx]);
					strcat(html, temp_port_item);
					if (idx < SENSOR_PORT_NUM - 1)
						strcat(html, ",");
					free(temp_port_item);
				}
			}
		}
		strcat(html, ");\n");
		// strcat(html, "delete typeSel;delete listType;delete portSel;delete listPort;\n";
		strcat(html, "if (typeof typeSel === 'undefined'){var typeSel = [];};\n");
		strcat(html, "if (typeof listType === 'undefined'){var listType = [];};\n");
		strcat(html, "if (typeof portSel === 'undefined'){var portSel = [];};\n");
		strcat(html, "if (typeof listPort === 'undefined'){var listPort = [];};\n");
		for (int i = 0; i < 10; i++)
		{
			{
				char *temp_list = allocateStringMemory(512);
				if (temp_list)
				{
					snprintf(temp_list, 512, "listType[%d] = document.querySelector('#sensorCH%d');typeSel[%d]=%d;\n", i, i, i, config.sensor[i].type);
					strcat(html, temp_list);
					snprintf(temp_list, 512, "listPort[%d] = document.querySelector('#sensorP%d');portSel[%d]=%d;\n", i, i, i, config.sensor[i].port);
					strcat(html, temp_list);
					free(temp_list);
				}
			}
		}

		strcat(html, "for (let n = 0; n < 10; n++){\n");
		strcat(html, "for (let i = 0; i < typeArry.length; i++) {\n");
		strcat(html, "const optionType = new Option(typeArry[i], i);\n");
		strcat(html, "listType[n].add(optionType, undefined);\n");
		strcat(html, "};\n");
		strcat(html, "listType[n].options[typeSel[n]].selected = true;\n");
		strcat(html, "for (let p = 0; p < portArry.length; p++) {\n");
		strcat(html, "const optionPort = new Option(portArry[p], p);\n");
		strcat(html, "listPort[n].add(optionPort, undefined);\n");
		strcat(html, "};\n");
		strcat(html, "listPort[n].options[portSel[n]].selected = true;\n");
		strcat(html, "};\n");

		strcat(html, "</script>\n");

		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
		response->addHeader("Sensor", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(html); // Free the allocated memory
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
		String html_msg;
		if (saveConfiguration("/default.cfg", config))
		{
			html_msg = "Setup completed successfully";
			request->send(200, "text/html", html_msg); // send to someones browser when asked
		}
		else
		{
			html_msg = "Save config failed.";
			request->send(501, "text/html", html_msg); // Not Implemented
		}
	}

	// Allocate initial memory for HTML content
	char *html = allocateStringMemory(22000); // Start with 8KB buffer
	if (!html)
	{
		request->send(500, "text/html", "Memory allocation failed");
		return;
	}
	strcpy(html, "<script type=\"text/javascript\">\n");
	strcat(html, "$('form').submit(function (e) {\n");
	strcat(html, "e.preventDefault();\n");
	strcat(html, "var data = new FormData(e.currentTarget);\n");
	strcat(html, "document.getElementById(\"submitTRACKER\").disabled=true;\n");
	strcat(html, "$.ajax({\n");
	strcat(html, "url: '/tracker',\n");
	strcat(html, "type: 'POST',\n");
	strcat(html, "data: data,\n");
	strcat(html, "contentType: false,\n");
	strcat(html, "processData: false,\n");
	strcat(html, "success: function (data) {\n");
	strcat(html, "alert(\"Submited Successfully\");\n");
	strcat(html, "},\n");
	strcat(html, "error: function (data) {\n");
	strcat(html, "alert(\"An error occurred.\");\n");
	strcat(html, "}\n");
	strcat(html, "});\n");
	strcat(html, "});\n");
	strcat(html, "</script>\n<script type=\"text/javascript\">\n");
	strcat(html, "function openWindowSymbol(sel) {\n");
	strcat(html, "var i, l, options = [{\n");
	strcat(html, "value: 'first',\n");
	strcat(html, "text: 'First'\n");
	strcat(html, "}, {\n");
	strcat(html, "value: 'second',\n");
	strcat(html, "text: 'Second'\n");
	strcat(html, "}],\n");
	strcat(html, "newWindow = window.open(\"/symbol?sel=\"+sel.toString(), null, \"height=400,width=400,status=no,toolbar=no,menubar=no,location=no\");\n");
	strcat(html, "}\n");

	strcat(html, "function setValue(sel,symbol,table) {\n");
	strcat(html, "var txtsymbol=document.getElementById('trackerSymbol');\n");
	strcat(html, "var txttable=document.getElementById('trackerTable');\n");
	strcat(html, "var imgicon=document.getElementById('trackerImgSymbol');\n");
	strcat(html, "if(sel==1){\n");
	strcat(html, "txtsymbol=document.getElementById('moveSymbol');\n");
	strcat(html, "txttable=document.getElementById('moveTable');\n");
	strcat(html, "imgicon= document.getElementById('moveImgSymbol');\n");
	strcat(html, "}else if(sel==2){\n");
	strcat(html, "txtsymbol=document.getElementById('stopSymbol');\n");
	strcat(html, "txttable=document.getElementById('stopTable');\n");
	strcat(html, "imgicon= document.getElementById('stopImgSymbol');\n");
	strcat(html, "}\n");
	strcat(html, "txtsymbol.value = String.fromCharCode(symbol);\n");
	strcat(html, "if(table==1){\n txttable.value='/';\n");
	strcat(html, "}else if(table==2){\n txttable.value='\\\\';\n}\n");
	strcat(html, "imgicon.src = \"http://aprs.dprns.com/symbols/icons/\"+symbol.toString()+'-'+table.toString()+'.png';\n");
	strcat(html, "\n}\n");
	strcat(html, "function onSmartCheck() {\n");
	strcat(html, "if (document.querySelector('#smartBcnEnable').checked) {\n");
	// Checkbox has been checked
	strcat(html, "document.getElementById(\"smartbcnGrp\").disabled=false;\n");
	strcat(html, "} else {\n");
	// Checkbox has been unchecked
	strcat(html, "document.getElementById(\"smartbcnGrp\").disabled=true;\n");
	strcat(html, "}\n}\n");

	strcat(html, "function selPrecision(idx) {\n");
	strcat(html, "var x=0;\n");
	strcat(html, "x = document.getElementsByName(\"precision\"+idx)[0].value;\n");
	strcat(html, "document.getElementsByName(\"eqns\"+idx+\"b\")[0].value=1/Math.pow(10,x);\n");
	strcat(html, "}\n");
	strcat(html, "function selOffset(idx) {\n");
	strcat(html, "var x=0;\n");
	strcat(html, "x = document.getElementsByName(\"offset\"+idx)[0].value;\n");
	strcat(html, "document.getElementsByName(\"eqns\"+idx+\"c\")[0].value=x*(-1);\n");
	strcat(html, "}\n");
	strcat(html, "</script>\n");

	delay(1);
	/************************ tracker Mode **************************/
	strcat(html, "<form id='formtracker' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
	// strcat(html, "<h2>[TRACKER] Tracker Position Mode</h2>\n");
	strcat(html, "<table>\n");
	// strcat(html, "<tr>\n");
	// strcat(html, "<th width=\"200\"><span><b>Setting</b></span></th>\n");
	// strcat(html, "<th><span><b>Value</b></span></th>\n");
	// strcat(html, "</tr>\n");
	strcat(html, "<th colspan=\"2\"><span><b>[TRACKER] Tracker Position Mode</b></span></th>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
	char trackerEnFlag[10] = "";
	if (config.trk_en)
		strcpy(trackerEnFlag, "checked");
	{
		char *temp_flag = allocateStringMemory(512);
		if (temp_flag)
		{
			snprintf(temp_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"trackerEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", trackerEnFlag);
			strcat(html, temp_flag);
			free(temp_flag);
		}
	}
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Station Callsign:</b></td>\n");
	{
		char *temp_call = allocateStringMemory(512);
		if (temp_call)
		{
			snprintf(temp_call, 512, "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"%s\" /></td>\n", config.trk_mycall);
			strcat(html, temp_call);
			free(temp_call);
		}
	}
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Station SSID:</b></td>\n");
	strcat(html, "<td style=\"text-align: left;\">\n");
	strcat(html, "<select name=\"mySSID\" id=\"mySSID\">\n");
	for (uint8_t ssid = 0; ssid <= 15; ssid++)
	{
		{
			char *temp_option = allocateStringMemory(256);
			if (temp_option)
			{
				if (config.trk_ssid == ssid)
				{
					snprintf(temp_option, 256, "<option value=\"%d\" selected>%d</option>\n", ssid, ssid);
				}
				else
				{
					snprintf(temp_option, 256, "<option value=\"%d\">%d</option>\n", ssid, ssid);
				}
				strcat(html, temp_option);
				free(temp_option);
			}
		}
	}
	strcat(html, "</select></td>\n");
	strcat(html, "</tr>\n");

	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Item/Obj Name:</b></td>\n");
	{
		char *temp_item = allocateStringMemory(512);
		if (temp_item)
		{
			snprintf(temp_item, 512, "<td style=\"text-align: left;\"><input maxlength=\"9\" size=\"9\" id=\"trackerObject\" name=\"trackerObject\" type=\"text\" value=\"%s\" /><i> *If not used, leave it blank.In use 3-9 charactor</i></td>\n", config.trk_item);
			strcat(html, temp_item);
			free(temp_item);
		}
	}
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
	strcat(html, "<td style=\"text-align: left;\">\n");
	strcat(html, "<select name=\"trackerPath\" id=\"trackerPath\">\n");
	for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
	{
		{
			char *temp_path = allocateStringMemory(256);
			if (temp_path)
			{
				if (config.trk_path == pthIdx)
				{
					snprintf(temp_path, 256, "<option value=\"%d\" selected>%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
				}
				else
				{
					snprintf(temp_path, 256, "<option value=\"%d\">%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
				}
				strcat(html, temp_path);
				free(temp_path);
			}
		}
	}
	strcat(html, "</select></td>\n");
	// strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" id=\"trackerPath\" name=\"trackerPath\" type=\"text\" value=\"" + String(config.trk_path) + "\" /></td>\n";
	strcat(html, "</tr>\n");

	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Text Comment:</b></td>\n");
	{
		char *temp_comment = allocateStringMemory(512);
		if (temp_comment)
		{
			snprintf(temp_comment, 512, "<td style=\"text-align: left;\"><input maxlength=\"25\" size=\"30\" id=\"trackerComment\" name=\"trackerComment\" type=\"text\" value=\"%s\" /></td>\n", config.trk_comment);
			strcat(html, temp_comment);
			free(temp_comment);
		}
	}
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Text Status:</b></td>\n");
	{
		char *temp_status = allocateStringMemory(512);
		if (temp_status)
		{
			snprintf(temp_status, 512, "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"60\" id=\"trkStatus\" name=\"trkStatus\" type=\"text\" value=\"%s\" />  Interval:<input min=\"0\" max=\"3600\" step=\"1\" name=\"trkSTSInv\" type=\"number\" value=\"%d\" />Sec.</td>\n", config.trk_status, config.trk_sts_interval);
			strcat(html, temp_status);
			free(temp_status);
		}
	}
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Smart Beacon:</b></td>\n");
	char smartBcnEnFlag[10] = "";
	if (config.trk_smartbeacon)
		strcpy(smartBcnEnFlag, "checked");
	{
		char *temp_smart = allocateStringMemory(512);
		if (temp_smart)
		{
			snprintf(temp_smart, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"smartBcnEnable\" name=\"smartBcnEnable\" onclick=\"onSmartCheck()\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch use to smart beacon mode</i></label></td>\n", smartBcnEnFlag);
			strcat(html, temp_smart);
			free(temp_smart);
		}
	}
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Compress:</b></td>\n");
	char compressEnFlag[10] = "";
	if (config.trk_compress)
		strcpy(compressEnFlag, "checked");
	{
		char *temp_compress = allocateStringMemory(512);
		if (temp_compress)
		{
			snprintf(temp_compress, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"compressEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch compress packet</i></label></td>\n", compressEnFlag);
			strcat(html, temp_compress);
			free(temp_compress);
		}
	}
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Mic-E Type:</b></td>\n");
	strcat(html, "<td style=\"text-align: left;\">\n");
	strcat(html, "<select name=\"trkMicEType\" id=\"trkMicEType\">\n");
	for (uint8_t micEIdx = 0; micEIdx < 8; micEIdx++)
	{
		{
			char *temp_mice = allocateStringMemory(256);
			if (temp_mice)
			{
				if (config.trk_mice_type == micEIdx)
				{
					snprintf(temp_mice, 256, "<option value=\"%d\" selected>%s</option>\n", micEIdx, MIC_E_MSG[micEIdx]);
				}
				else
				{
					snprintf(temp_mice, 256, "<option value=\"%d\">%s</option>\n", micEIdx, MIC_E_MSG[micEIdx]);
				}
				strcat(html, temp_mice);
				free(temp_mice);
			}
		}
	}
	strcat(html, "</select><label style=\"vertical-align: bottom;font-size: 8pt;\"><i>*Support if Compress is enabled and not use Item/Obj,Time Stamp</i></label></td>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Time Stamp:</b></td>\n");
	char timeStampFlag[10] = "";
	if (config.trk_timestamp)
		strcpy(timeStampFlag, "checked");
	{
		char *temp_time = allocateStringMemory(512);
		if (temp_time)
		{
			snprintf(temp_time, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"trackerTimeStamp\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", timeStampFlag);
			strcat(html, temp_time);
			free(temp_time);
		}
	}
	strcat(html, "</tr>\n");
	char trackerPos2RFFlag[10] = "";
	char trackerPos2INETFlag[10] = "";
	if (config.trk_loc2rf)
		strcpy(trackerPos2RFFlag, "checked");
	if (config.trk_loc2inet)
		strcpy(trackerPos2INETFlag, "checked");
	{
		char *temp_channels = allocateStringMemory(512);
		if (temp_channels)
		{
			snprintf(temp_channels, 512, "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"trackerPos2RF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"trackerPos2INET\" value=\"OK\" %s/>Internet </td></tr>\n", trackerPos2RFFlag, trackerPos2INETFlag);
			strcat(html, temp_channels);
			free(temp_channels);
		}
	}
	char trackerOptBatFlag[10] = "";
	char trackerOptSatFlag[10] = "";
	char trackerOptAltFlag[10] = "";
	char trackerOptCSTFlag[10] = "";
	if (config.trk_rssi)
		strcpy(trackerOptBatFlag, "checked");
	if (config.trk_sat)
		strcpy(trackerOptSatFlag, "checked");
	if (config.trk_altitude)
		strcpy(trackerOptAltFlag, "checked");
	if (config.trk_log)
		strcpy(trackerOptCSTFlag, "checked");
	{
		char *temp_options = allocateStringMemory(1024);
		if (temp_options)
		{
			snprintf(temp_options, 1024, "<tr><td style=\"text-align: right;\"><b>Option:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"trackerOptCST\" value=\"OK\" %s/>Telemetry <input type=\"checkbox\" name=\"trackerOptAlt\" value=\"OK\" %s/>Altutude <input type=\"checkbox\" name=\"trackerOptBat\" value=\"OK\" %s/>RSSI Request </td></tr>\n",
					 trackerOptCSTFlag, trackerOptAltFlag, trackerOptBatFlag);
			strcat(html, temp_options);
			free(temp_options);
		}
	}

	strcat(html, "<tr>");
	strcat(html, "<td align=\"right\"><b>POSITION:</b></td>\n");
	strcat(html, "<td align=\"center\">\n");
	strcat(html, "<table>");
	{
		char *temp_interval = allocateStringMemory(512);
		if (temp_interval)
		{
			snprintf(temp_interval, 512, "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" id=\"trackerPosInv\" name=\"trackerPosInv\" type=\"number\" value=\"%d\" />Sec.</label></td></tr>", config.trk_interval);
			strcat(html, temp_interval);
			free(temp_interval);
		}
	}
	char trackerPosFixFlag[20] = "";
	char trackerPosGPSFlag[20] = "";

	if (config.trk_gps)
		strcpy(trackerPosGPSFlag, "checked=\"checked\"");
	else
		strcpy(trackerPosFixFlag, "checked=\"checked\"");

	{
		char *temp_location = allocateStringMemory(1024);
		if (temp_location)
		{
			snprintf(temp_location, 1024, "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"trackerPosSel\" value=\"0\" %s/>Fix <input type=\"radio\" name=\"trackerPosSel\" value=\"1\" %s/>GPS </td></tr>\n",
					 trackerPosFixFlag, trackerPosGPSFlag);
			strcat(html, temp_location);
			free(temp_location);
		}
	}
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\">Symbol Icon:</td>\n");
	char table[5] = "1";
	if (config.trk_symbol[0] == 47)
		strcpy(table, "1");
	if (config.trk_symbol[0] == 92)
		strcpy(table, "2");
	{
		char *temp_symbol = allocateStringMemory(1024);
		if (temp_symbol)
		{
			snprintf(temp_symbol, 1024, "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"trackerTable\" name=\"trackerTable\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"trackerSymbol\" name=\"trackerSymbol\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"trackerImgSymbol\" onclick=\"openWindowSymbol(0);\" src=\"http://aprs.dprns.com/symbols/icons/%d-%s.png\"> <i>*Click icon for select symbol</i></td>\n",
					 config.trk_symbol[0], config.trk_symbol[1], (int)config.trk_symbol[1], table);
			strcat(html, temp_symbol);
			free(temp_symbol);
		}
	}
	strcat(html, "</tr>\n");
	{
		char *temp_lat = allocateStringMemory(512);
		if (temp_lat)
		{
			snprintf(temp_lat, 512, "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" id=\"trackerPosLat\" name=\"trackerPosLat\" type=\"number\" value=\"%.5f\" />degrees (positive for North, negative for South)</td></tr>\n", config.trk_lat);
			strcat(html, temp_lat);
			free(temp_lat);
		}
	}
	{
		char *temp_lon = allocateStringMemory(512);
		if (temp_lon)
		{
			snprintf(temp_lon, 512, "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" id=\"trackerPosLon\" name=\"trackerPosLon\" type=\"number\" value=\"%.5f\" />degrees (positive for East, negative for West)</td></tr>\n", config.trk_lon);
			strcat(html, temp_lon);
			free(temp_lon);
		}
	}
	{
		char *temp_alt = allocateStringMemory(512);
		if (temp_alt)
		{
			snprintf(temp_alt, 512, "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" id=\"trackerPosAlt\" name=\"trackerPosAlt\" type=\"number\" value=\"%.2f\" /> meter. *Value 0 is not send height</td></tr>\n", config.trk_alt);
			strcat(html, temp_alt);
			free(temp_alt);
		}
	}
	strcat(html, "</table></td>");
	strcat(html, "</tr>\n");

	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Smart Beacon:</b></td>\n");
	strcat(html, "<td align=\"center\">\n");
	if (config.trk_smartbeacon)
		strcat(html, "<fieldset id=\"smartbcnGrp\">\n");
	else
		strcat(html, "<fieldset id=\"smartbcnGrp\" disabled>\n");
	strcat(html, "<legend>Smart beacon configuration</legend>\n<table>");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\">Move Symbol:</td>\n");
	strcpy(table, "1");
	if (config.trk_symmove[0] == 47)
		strcpy(table, "1");
	if (config.trk_symmove[0] == 92)
		strcpy(table, "2");
	{
		char *temp_move = allocateStringMemory(1024);
		if (temp_move)
		{
			snprintf(temp_move, 1024, "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"moveTable\" name=\"moveTable\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"moveSymbol\" name=\"moveSymbol\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"moveImgSymbol\" onclick=\"openWindowSymbol(1);\" src=\"http://aprs.dprns.com/symbols/icons/%d-%s.png\"> <i>*Click icon for select MOVE symbol</i></td>\n",
					 config.trk_symmove[0], config.trk_symmove[1], (int)config.trk_symmove[1], table);
			strcat(html, temp_move);
			free(temp_move);
		}
	}
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\">Stop Symbol:</td>\n");
	strcpy(table, "1");
	if (config.trk_symstop[0] == 47)
		strcpy(table, "1");
	if (config.trk_symstop[0] == 92)
		strcpy(table, "2");
	{
		char *temp_stop = allocateStringMemory(1024);
		if (temp_stop)
		{
			snprintf(temp_stop, 1024, "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"stopTable\" name=\"stopTable\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"stopSymbol\" name=\"stopSymbol\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"stopImgSymbol\" onclick=\"openWindowSymbol(2);\" src=\"http://aprs.dprns.com/symbols/icons/%d-%s.png\"> <i>*Click icon for select STOP symbol</i></td>\n",
					 config.trk_symstop[0], config.trk_symstop[1], (int)config.trk_symstop[1], table);
			strcat(html, temp_stop);
			free(temp_stop);
		}
	}
	strcat(html, "</tr>\n");
	{
		char *temp_speed = allocateStringMemory(512);
		if (temp_speed)
		{
			snprintf(temp_speed, 512, "<tr><td style=\"text-align: right;\">High Speed:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"10\" max=\"1000\" step=\"1\" id=\"hspeed\" name=\"hspeed\" type=\"number\" value=\"%d\" /> km/h</td></tr>\n", config.trk_hspeed);
			strcat(html, temp_speed);
			free(temp_speed);
		}
	}
	{
		char *temp_lspeed = allocateStringMemory(512);
		if (temp_lspeed)
		{
			snprintf(temp_lspeed, 512, "<tr><td style=\"text-align: right;\">Low Speed:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"1\" max=\"250\" step=\"1\" id=\"lspeed\" name=\"lspeed\" type=\"number\" value=\"%d\" /> km/h</td></tr>\n", config.trk_lspeed);
			strcat(html, temp_lspeed);
			free(temp_lspeed);
		}
	}
	{
		char *temp_slow = allocateStringMemory(512);
		if (temp_slow)
		{
			snprintf(temp_slow, 512, "<tr><td style=\"text-align: right;\">Slow Interval:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"60\" max=\"3600\" step=\"1\" id=\"slowInterval\" name=\"slowInterval\" type=\"number\" value=\"%d\" /> Sec.</td></tr>\n", config.trk_slowinterval);
			strcat(html, temp_slow);
			free(temp_slow);
		}
	}
	{
		char *temp_max = allocateStringMemory(512);
		if (temp_max)
		{
			snprintf(temp_max, 512, "<tr><td style=\"text-align: right;\">Max Interval:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"10\" max=\"255\" step=\"1\" id=\"maxInterval\" name=\"maxInterval\" type=\"number\" value=\"%d\" /> Sec.</td></tr>\n", config.trk_maxinterval);
			strcat(html, temp_max);
			free(temp_max);
		}
	}
	{
		char *temp_min = allocateStringMemory(512);
		if (temp_min)
		{
			snprintf(temp_min, 512, "<tr><td style=\"text-align: right;\">Min Interval:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"1\" max=\"100\" step=\"1\" id=\"minInterval\" name=\"minInterval\" type=\"number\" value=\"%d\" /> Sec.</td></tr>\n", config.trk_mininterval);
			strcat(html, temp_min);
			free(temp_min);
		}
	}
	{
		char *temp_angle = allocateStringMemory(512);
		if (temp_angle)
		{
			snprintf(temp_angle, 512, "<tr><td style=\"text-align: right;\">Min Angle:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"1\" max=\"359\" step=\"1\" id=\"minAngle\" name=\"minAngle\" type=\"number\" value=\"%d\" /> Degree.</td></tr>\n", config.trk_minangle);
			strcat(html, temp_angle);
			free(temp_angle);
		}
	}

	strcat(html, "</table></fieldset></tr>");

	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Telemetry:</b><br />(v=0->8280)</td>\n");
	strcat(html, "<td align=\"center\"><table>\n");
	{
		char *temp_tlm = allocateStringMemory(512);
		if (temp_tlm)
		{
			snprintf(temp_tlm, 512, "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"1000\" step=\"1\" id=\"trkTlmInv\" name=\"trkTlmInv\" type=\"number\" value=\"%d\" /> *Number of packets interval,<i>Example: 0 not send,1 send every packet</i></label></td></tr>", config.trk_tlm_interval);
			strcat(html, temp_tlm);
			free(temp_tlm);
		}
	}
	for (int ax = 0; ax < 5; ax++)
	{
		{
			char *temp_ch = allocateStringMemory(256);
			if (temp_ch)
			{
				snprintf(temp_ch, 256, "<tr><td align=\"right\"><b>CH A%d:</b></td>\n", ax + 1);
				strcat(html, temp_ch);
				free(temp_ch);
			}
		}
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");

		strcat(html, "<tr><td style=\"text-align: right;\">Sensor:</td>\n");
		strcat(html, "<td style=\"text-align: left;\">CH: ");
		{
			char *temp_select = allocateStringMemory(256);
			if (temp_select)
			{
				snprintf(temp_select, 256, "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax, ax);
				strcat(html, temp_select);
				free(temp_select);
			}
		}
		for (uint8_t idx = 0; idx < 11; idx++)
		{
			{
				char *temp_opt = allocateStringMemory(256);
				if (temp_opt)
				{
					if (idx == 0)
					{
						if (config.trk_tlm_sensor[ax] == idx)
						{
							snprintf(temp_opt, 256, "<option value=\"%d\" selected>NONE</option>\n", idx);
						}
						else
						{
							snprintf(temp_opt, 256, "<option value=\"%d\">NONE</option>\n", idx);
						}
					}
					else
					{
						if (config.trk_tlm_sensor[ax] == idx)
						{
							snprintf(temp_opt, 256, "<option value=\"%d\" selected>SENSOR#%d</option>\n", idx, idx);
						}
						else
						{
							snprintf(temp_opt, 256, "<option value=\"%d\">SENSOR#%d</option>\n", idx, idx);
						}
					}
					strcat(html, temp_opt);
					free(temp_opt);
				}
			}
		}
		strcat(html, "</select></td>\n");

		{
			char *temp_name = allocateStringMemory(512);
			if (temp_name)
			{
				snprintf(temp_name, 512, "<td style=\"text-align: left;\">Name: <input maxlength=\"10\" size=\"8\" name=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.trk_tlm_PARM[ax]);
				strcat(html, temp_name);
				free(temp_name);
			}
		}
		{
			char *temp_unit = allocateStringMemory(512);
			if (temp_unit)
			{
				snprintf(temp_unit, 512, "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.trk_tlm_UNIT[ax]);
				strcat(html, temp_unit);
				free(temp_unit);
			}
		}
		{
			char *temp_prec = allocateStringMemory(512);
			if (temp_prec)
			{
				snprintf(temp_prec, 512, "<td style=\"text-align: left;\">Precision: <input min=\"0\" max=\"5\" step=\"1\" type=\"number\" style=\"width: 2em\" name=\"precision%d\" type=\"text\" value=\"%d\" onchange=\"selPrecision(%d)\" /></td></tr>\n", ax, config.trk_tlm_precision[ax], ax);
				strcat(html, temp_prec);
				free(temp_prec);
			}
		}

		{
			char *temp_eqns = allocateStringMemory(1024);
			if (temp_eqns)
			{
				snprintf(temp_eqns, 1024, "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%da\" type=\"number\" value=\"%.5f\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%db\" type=\"number\" value=\"%.5f\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%dc\" type=\"number\" value=\"%.5f\" /> (av<sup>2</sup>+bv+c) </td>\n",
						 ax, config.trk_tlm_EQNS[ax][0], ax, config.trk_tlm_EQNS[ax][1], ax, config.trk_tlm_EQNS[ax][2]);
				strcat(html, temp_eqns);
				free(temp_eqns);
			}
		}
		{
			char *temp_offset = allocateStringMemory(512);
			if (temp_offset)
			{
				snprintf(temp_offset, 512, "<td style=\"text-align: left;\">Offset: <input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" type=\"number\" name=\"offset%d\" type=\"text\" value=\"%.5f\"  onchange=\"selOffset(%d)\" /></td></tr>\n", ax, config.trk_tlm_offset[ax], ax);
				strcat(html, temp_offset);
				free(temp_offset);
			}
		}
		strcat(html, "</table></td>");
		strcat(html, "</tr>\n");
	}
	strcat(html, "</table></td></tr>\n");
	strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
	strcat(html, "<div><button class=\"button\" type='submit' id='submitTRACKER'  name=\"commitTRACKER\"> Apply Change </button></div>\n");
	strcat(html, "<input type=\"hidden\" name=\"commitTRACKER\"/>\n");
	strcat(html, "</td></tr></table><br />\n");
	strcat(html, "</form><br />");

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
	response->addHeader("Tracker", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	free(html); // Free the allocated memory	
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
		String html_msg = "";
		if (saveConfiguration("/default.cfg", config))
		{
			html_msg = "Setup completed successfully";
			request->send(200, "text/html", html_msg); // send to someones browser when asked
		}
		else
		{
			html_msg = "Save config failed.";
			request->send(501, "text/html", html_msg); // Not Implemented
		}
	}
	else if (request->hasArg("commitWiFiClient"))
	{
		bool wifiSTA = false;
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
				String nameSSID = "wifiStation" + String(n);
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
				String namePASS = "wifi_pass" + String(n);
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
					{
						config.wifi_power = (int8_t)request->arg(i).toInt();
						WiFi.setTxPower((wifi_power_t)config.wifi_power);
					}
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
		String html_msg2 = "";
		if (saveConfiguration("/default.cfg", config))
		{
			html_msg2 = "Setup completed successfully";
			request->send(200, "text/html", html_msg2); // send to someones browser when asked
		}
		else
		{
			html_msg2 = "Save config failed.";
			request->send(501, "text/html", html_msg2); // Not Implemented
		}
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
		String html_msg3 = "";
		if (saveConfiguration("/default.cfg", config))
		{
			html_msg3 = "Setup completed successfully";
			request->send(200, "text/html", html_msg3); // send to someones browser when asked
		}
		else
		{
			html_msg3 = "Save config failed.";
			request->send(501, "text/html", html_msg3); // Not Implemented
		}
	}
	else
	{
		// Allocate initial memory for HTML content
		char *html = allocateStringMemory(12000); // Start with 12KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formBluetooth\") document.getElementById(\"submitBluetooth\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formWiFiAP\") document.getElementById(\"submitWiFiAP\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formWiFiClient\") document.getElementById(\"submitWiFiClient\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/wireless',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");
		/************************ WiFi AP **************************/
		strcat(html, "<form id='formWiFiAP' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		// strcat(html, "<h2>WiFi Access Point</h2>\n");
		strcat(html, "<table>\n");
		// strcat(html, "<tr>\n");
		// strcat(html, "<th width=\"200\"><span><b>Setting</b></span></th>\n");
		// strcat(html, "<th><span><b>Value</b></span></th>\n");
		// strcat(html, "</tr>\n");
		strcat(html, "<th colspan=\"2\"><span><b>WiFi Access Point</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\" width=\"120\"><b>Enable:</b></td>\n");
		const char *wifiAPEnFlag = (config.wifi_mode & WIFI_AP_FIX) ? "checked" : "";
		{
			char *temp_flag = allocateStringMemory(512);
			if (temp_flag)
			{
				snprintf(temp_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wifiAP\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", wifiAPEnFlag);
				strcat(html, temp_flag);
				free(temp_flag);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>WiFi AP SSID:</b></td>\n");
		{
			char *temp_ssid = allocateStringMemory(512);
			if (temp_ssid)
			{
				snprintf(temp_ssid, 512, "<td style=\"text-align: left;\"><input size=\"32\" maxlength=\"32\" class=\"form-control\" id=\"wifi_ssidAP\" name=\"wifi_ssidAP\" type=\"text\" value=\"%s\" /></td>\n", config.wifi_ap_ssid);
				strcat(html, temp_ssid);
				free(temp_ssid);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>WiFi AP PASSWORD:</b></td>\n");
		{
			char *temp_pass = allocateStringMemory(512);
			if (temp_pass)
			{
				snprintf(temp_pass, 512, "<td style=\"text-align: left;\"><input size=\"63\" maxlength=\"63\" class=\"form-control\" id=\"wifi_passAP\" name=\"wifi_passAP\" type=\"password\" value=\"%s\" /></td>\n", config.wifi_ap_pass);
				strcat(html, temp_pass);
				free(temp_pass);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitWiFiAP'  name=\"commitWiFiAP\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitWiFiAP\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");
		/************************ WiFi Client **************************/
		strcat(html, "<br />\n");
		strcat(html, "<form id='formWiFiClient' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>WiFi Multi Station</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>WiFi STA Enable:</b></td>\n");
		String wifiClientEnFlag = "";
		if (config.wifi_mode & WIFI_STA_FIX)
			wifiClientEnFlag = "checked";
		{
			char *temp_wifi_flag = allocateStringMemory(512);
			if (temp_wifi_flag)
			{
				snprintf(temp_wifi_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wificlient\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", wifiClientEnFlag);
				strcat(html, temp_wifi_flag);
				free(temp_wifi_flag);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>WiFi RF Power:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"wifi_pwr\" id=\"wifi_pwr\">\n");
		for (int i = 0; i < 12; i++)
		{
			{
				char *temp_option = allocateStringMemory(256);
				if (temp_option)
				{
					if (config.wifi_power == (int8_t)wifiPwr[i][0])
						snprintf(temp_option, 256, "<option value=\"%d\" selected>%.1f dBm</option>\n", (int8_t)wifiPwr[i][0], wifiPwr[i][1]);
					else
						snprintf(temp_option, 256, "<option value=\"%d\" >%.1f dBm</option>\n", (int8_t)wifiPwr[i][0], wifiPwr[i][1]);
					strcat(html, temp_option);
					free(temp_option);
				}
			}
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		for (int n = 0; n < 5; n++)
		{
			strcat(html, "<tr>\n");
			{
				char *temp_station = allocateStringMemory(256);
				if (temp_station)
				{
					snprintf(temp_station, 256, "<td align=\"right\"><b>Station #%d:</b></td>\n", n + 1);
					strcat(html, temp_station);
					free(temp_station);
				}
			}
			strcat(html, "<td align=\"center\">\n");
			{
				char *temp_fieldset = allocateStringMemory(256);
				if (temp_fieldset)
				{
					snprintf(temp_fieldset, 256, "<fieldset id=\"filterDispGrp%d\">\n", n + 1);
					strcat(html, temp_fieldset);
					free(temp_fieldset);
				}
			}
			{
				char *temp_legend = allocateStringMemory(256);
				if (temp_legend)
				{
					snprintf(temp_legend, 256, "<legend>WiFi Station #%d</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">", n + 1);
					strcat(html, temp_legend);
					free(temp_legend);
				}
			}
			strcat(html, "<tr style=\"background:unset;\">");
			// strcat(html, "<tr>\n";
			strcat(html, "<td align=\"right\" width=\"120\"><b>Enable:</b></td>\n");
			const char *wifiClientEnFlag = config.wifi_sta[n].enable ? "checked" : "";
			{
				char *temp_wifi_station = allocateStringMemory(256);
				if (temp_wifi_station)
				{
					snprintf(temp_wifi_station, 256, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wifiStation%d\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", n, wifiClientEnFlag);
					strcat(html, temp_wifi_station);
					free(temp_wifi_station);
				}
			}
			strcat(html, "</tr>\n");
			strcat(html, "<tr>\n");
			strcat(html, "<td align=\"right\"><b>WiFi SSID:</b></td>\n");
			{
				char *temp_wifi_ssid = allocateStringMemory(512);
				if (temp_wifi_ssid)
				{
					snprintf(temp_wifi_ssid, 512, "<td style=\"text-align: left;\"><input size=\"32\" maxlength=\"32\" name=\"wifi_ssid%d\" type=\"text\" value=\"%s\" /></td>\n", n, config.wifi_sta[n].wifi_ssid);
					strcat(html, temp_wifi_ssid);
					free(temp_wifi_ssid);
				}
			}
			strcat(html, "</tr>\n");
			strcat(html, "<tr>\n");
			strcat(html, "<td align=\"right\"><b>WiFi PASSWORD:</b></td>\n");
			{
				char *temp_wifi_pass = allocateStringMemory(512);
				if (temp_wifi_pass)
				{
					snprintf(temp_wifi_pass, 512, "<td style=\"text-align: left;\"><input size=\"63\" maxlength=\"63\" name=\"wifi_pass%d\" type=\"password\" value=\"%s\" /></td>\n", n, config.wifi_sta[n].wifi_pass);
					strcat(html, temp_wifi_pass);
					free(temp_wifi_pass);
				}
			}
			strcat(html, "</tr>\n");
			strcat(html, "</tr></table></fieldset>\n");
			strcat(html, "</td></tr>\n");
		}
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitWiFiClient'  name=\"commitWiFiClient\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitWiFiClient\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");
		/************************ Bluetooth **************************/
#ifdef BLUETOOTH
		strcat(html, "<br />\n");
		strcat(html, "<form id='formBluetooth' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		// strcat(html, "<h2>Bluetooth Master (BLE)</h2>\n";
		strcat(html, "<table>\n");
		// strcat(html, "<tr>\n";
		// strcat(html, "<th width=\"200\"><span><b>Setting</b></span></th>\n";
		// strcat(html, "<th><span><b>Value</b></span></th>\n";
		// strcat(html, "</tr>\n";
		strcat(html, "<th colspan=\"2\"><span><b>Bluetooth Master (BLE)</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
		const char* btEnFlag = config.bt_master ? "checked" : "";
		{
			char *temp_bt_flag = allocateStringMemory(256);
			if (temp_bt_flag)
			{
				snprintf(temp_bt_flag, 256, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"btMaster\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", btEnFlag);
				strcat(html, temp_bt_flag);
				free(temp_bt_flag);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>NAME:</b></td>\n");
		{
			char *temp_bt_name = allocateStringMemory(256);
			if (temp_bt_name)
			{
				snprintf(temp_bt_name, 256, "<td style=\"text-align: left;\"><input maxlength=\"20\" id=\"bt_name\" name=\"bt_name\" type=\"text\" value=\"%s\" /></td>\n", config.bt_name);
				strcat(html, temp_bt_name);
				free(temp_bt_name);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PIN:</b></td>\n");
		{
			char *temp_bt_pin = allocateStringMemory(256);
			if (temp_bt_pin)
			{
				snprintf(temp_bt_pin, 256, "<td style=\"text-align: left;\"><input min=\"0\" max=\"999999\" id=\"bt_pin\" name=\"bt_pin\" type=\"number\" value=\"%d\" /></td> <i>*Value 0 is no auth.</i>\n", config.bt_pin);
				strcat(html, temp_bt_pin);
				free(temp_bt_pin);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UUID:</b></td>\n");
		{
			char *temp_bt_uuid = allocateStringMemory(512);
			if (temp_bt_uuid)
			{
				snprintf(temp_bt_uuid, 512, "<td style=\"text-align: left;\"><input maxlength=\"37\" size=\"38\" id=\"bt_uuid\" name=\"bt_uuid\" type=\"text\" value=\"%s\" /></td>\n", config.bt_uuid);
				strcat(html, temp_bt_uuid);
				free(temp_bt_uuid);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UUID RX:</b></td>\n");
		{
			char *temp_bt_uuid_rx = allocateStringMemory(512);
			if (temp_bt_uuid_rx)
			{
				snprintf(temp_bt_uuid_rx, 512, "<td style=\"text-align: left;\"><input maxlength=\"37\" size=\"38\" id=\"bt_uuid_rx\" name=\"bt_uuid_rx\" type=\"text\" value=\"%s\" /></td>\n", config.bt_uuid_rx);
				strcat(html, temp_bt_uuid_rx);
				free(temp_bt_uuid_rx);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UUID TX:</b></td>\n");
		{
			char *temp_bt_uuid_tx = allocateStringMemory(512);
			if (temp_bt_uuid_tx)
			{
				snprintf(temp_bt_uuid_tx, 512, "<td style=\"text-align: left;\"><input maxlength=\"37\" size=\"38\" id=\"bt_uuid_tx\" name=\"bt_uuid_tx\" type=\"text\" value=\"%s\" /></td>\n", config.bt_uuid_tx);
				strcat(html, temp_bt_uuid_tx);
				free(temp_bt_uuid_tx);
			}
		}
		strcat(html, "</tr>\n");

		strcat(html, "<td align=\"right\"><b>MODE:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"bt_mode\" id=\"bt_mode\">\n");
		const char* btModeOff = (config.bt_mode == 0) ? "selected" : "";
		const char* btModeTNC2 = (config.bt_mode == 1) ? "selected" : "";
		const char* btModeKISS = (config.bt_mode == 2) ? "selected" : "";
		{
			char *temp_bt_options = allocateStringMemory(512);
			if (temp_bt_options)
			{
				snprintf(temp_bt_options, 512, "<option value=\"0\" %s>NONE</option>\n<option value=\"1\" %s>TNC2</option>\n<option value=\"2\" %s>KISS</option>\n", btModeOff, btModeTNC2, btModeKISS);
				strcat(html, temp_bt_options);
				free(temp_bt_options);
			}
		}
		strcat(html, "</select>\n");

		strcat(html, "<label style=\"font-size: 8pt;text-align: right;\">*See the following for generating UUIDs: <a href=\"https://www.uuidgenerator.net\" target=\"_blank\">https://www.uuidgenerator.net</a></label></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitBluetooth'  name=\"commitBluetooth\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitBluetooth\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form>");
#endif

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)html);
	response->addHeader("wifi", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	free(html); // Free the allocated memory

	}
}

// extern String lastPkgRaw;
// extern float dBV;
// extern int mVrms;
//  void handle_realtime(AsyncWebServerRequest *request)
//  {
//  	// char jsonMsg[1000];
//  	char *jsonMsg;
//  	time_t timeStamp;
//  	time(&timeStamp);

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

// void handle_ws(String Raw,uint16_t mVrms)
void handle_ws(char *Raw, size_t len, uint16_t mVrms)
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
			if (mVrms > 0)
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
	if (ws_gnss.count() < 1)
		return;

	time_t timeStamp;
	time(&timeStamp);
	// unsigned int output_length = encode_base64_length(size);
	// unsigned char nmea_enc[output_length];
	// char jsonMsg[output_length + 100];
	// encode_base64((unsigned char *)nmea, size, (unsigned char *)nmea_enc);
	// sprintf(jsonMsg, "{\"en\":\"%d\",\"lat\":\"%.5f\",\"lng\":\"%.5f\",\"alt\":\"%.2f\",\"spd\":\"%.2f\",\"csd\":\"%.1f\",\"hdop\":\"%.2f\",\"sat\":\"%d\",\"time\":\"%d\",\"timeStamp\":\"%li\",\"RAW\":\"", (int)config.gnss_enable, gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.speed.kmph(), gps.course.deg(), gps.hdop.hdop(), gps.satellites.value(), gps.time.value(), timeStamp);
	// strncat(jsonMsg, (const char *)nmea_enc, output_length);
	// strcat(jsonMsg, "\"}");
	unsigned int output_length = encode_base64_length(size);
	unsigned char *nmea_enc = (unsigned char *)calloc(output_length + 2, sizeof(unsigned char));
	char *jsonMsg = (char *)calloc(output_length + 200, sizeof(char));
	if (nmea_enc && jsonMsg)
	{
		encode_base64((unsigned char *)nmea, size, (unsigned char *)nmea_enc);
		sprintf(jsonMsg, "{\"en\":\"%d\",\"lat\":\"%.5f\",\"lng\":\"%.5f\",\"alt\":\"%.2f\",\"spd\":\"%.2f\",\"csd\":\"%.1f\",\"hdop\":\"%.2f\",\"sat\":\"%d\",\"time\":\"%d\",\"timeStamp\":\"%li\",\"RAW\":\"", (int)config.gnss_enable, gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.speed.kmph(), gps.course.deg(), gps.hdop.hdop(), gps.satellites.value(), gps.time.value(), timeStamp);
		strncat(jsonMsg, (const char *)nmea_enc, output_length);
		strcat(jsonMsg, "\"}");
		ws_gnss.textAll(jsonMsg);
		free(nmea_enc);
		free(jsonMsg);
	}
	
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

	// Allocate memory for HTML content
	char *webString = allocateStringMemory(8192); // Start with 8KB buffer
	if (!webString)
	{
		request->send(500, "text/html", "Memory allocation failed");
		return;
	}
	strcpy(webString, "<html>\n<head>\n");
	strcat(webString, "<script src=\"https://apps.bdimg.com/libs/jquery/2.1.4/jquery.min.js\"></script>\n");
	strcat(webString, "<script src=\"https://code.highcharts.com/highcharts.js\"></script>\n");
	strcat(webString, "<script src=\"https://code.highcharts.com/highcharts-more.js\"></script>\n");
	strcat(webString, "<script language=\"JavaScript\">");
	strcat(webString, "$(document).ready(function() {\nvar chart = {\ntype: 'gauge',plotBorderWidth: 1,plotBackgroundColor: {linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },stops: [[0, '#FFFFC6'],[0.3, '#FFFFFF'],[1, '#FFF4C6']]},plotBackgroundImage: null,height: 200};\n");
	strcat(webString, "var credits = {enabled: false};\n");
	strcat(webString, "var title = {text: 'RX/AUDIO VU Meter'};\n");
	strcat(webString, "var pane = [{startAngle: -45,endAngle: 45,background: null,center: ['50%', '145%'],size: 300}];\n");
	strcat(webString, "var yAxis = [{min: -40,max: 1,minorTickPosition: 'outside',tickPosition: 'outside',labels: {rotation: 'auto',distance: 20},\n");
	strcat(webString, "plotBands: [{from: -10,to: 1,color: '#C02316',innerRadius: '100%',outerRadius: '105%'},{from: -20,to: -10,color: '#00C000',innerRadius: '100%',outerRadius: '105%'},{from: -30,to: -20,color: '#AFFF0F',innerRadius: '100%',outerRadius: '105%'},{from: -40,to: -30,color: '#C0A316',innerRadius: '100%',outerRadius: '105%'}],\n");
	strcat(webString, "pane: 0,title: {text: '<span style=\"font-size:12px\">dBV</span>',y: -40}}];\n");
	strcat(webString, "var plotOptions = {gauge: {dataLabels: {enabled: false},dial: {radius: '100%'}}};\n");
	strcat(webString, "var series= [{data: [-40],yAxis: 0}];\n");
	strcat(webString, "var json = {};\n json.chart = chart;\n json.credits = credits;\n json.title = title;\n json.pane = pane;\n json.yAxis = yAxis;\n json.plotOptions = plotOptions;\n json.series = series;\n");
	// Add some life
	strcat(webString, "var chartFunction = function (chart) { \n"); // the chart may be destroyed
	strcat(webString, "var Vrms=0;\nvar dBV=-40;\nvar active=0;var raw=\"\";var timeStamp;\n");
	strcat(webString, "if (chart.series) {\n");
	strcat(webString, "var left = chart.series[0].points[0];\n");
	strcat(webString, "var host='ws://'+location.hostname+':81/ws'\n");
	strcat(webString, "const ws = new WebSocket(host);\n");
	strcat(webString, "ws.onopen = function() { console.log('Connection opened');};\n ws.onclose = function() { console.log('Connection closed');};\n");
	strcat(webString, "ws.onmessage = function(event) {\n  console.log(event.data);\n");
	strcat(webString, "const jsonR=JSON.parse(event.data);\n");
	strcat(webString, "active=parseInt(jsonR.Active);\n");
	strcat(webString, "Vrms=parseFloat(jsonR.mVrms)/1000;\n");
	strcat(webString, "dBV=20.0*Math.log10(Vrms);\n");
	strcat(webString, "if(dBV<-40) dBV=-40;\n");
	strcat(webString, "raw=jsonR.RAW;\n");
	strcat(webString, "timeStamp=Number(jsonR.timeStamp);\n");
	strcat(webString, "if(active==1){\nleft.update(dBV,false);\nchart.redraw();\n");
	strcat(webString, "var date=new Date(timeStamp * 1000);\n");
	strcat(webString, "var head=date+\"[\"+Vrms.toFixed(3)+\"Vrms,\"+dBV.toFixed(1)+\"dBV]\\n\";\n");
	// strcat(webString, "document.getElementById(\"raw_txt\").value+=head+atob(raw)+\"\\n\";\n");
	strcat(webString, "var textArea=document.getElementById(\"raw_txt\");\n");
	strcat(webString, "textArea.value+=head+atob(raw)+\"\\n\";\n");
	strcat(webString, "textArea.scrollTop = textArea.scrollHeight;\n");
	strcat(webString, "}\n");
	strcat(webString, "}\n");
	strcat(webString, "}};\n");
	strcat(webString, "$('#vumeter').highcharts(json, chartFunction);\n");
	strcat(webString, "});\n</script>\n");
	strcat(webString, "</head><body>\n<table>\n");
	// strcat(webString, "<tr><td><form accept-charset=\"UTF-8\" action=\"/test\" class=\"form-horizontal\" id=\"test_form\" method=\"post\">\n");
	// strcat(webString, "<div style=\"margin-left: 20px;\"><input type='submit' class=\"btn btn-danger\" name=\"sendBeacon\" value='SEND BEACON'></div><br />\n");
	// strcat(webString, "<div style=\"margin-left: 20px;\">TNC2 RAW: <input id=\"raw\" name=\"raw\" type=\"text\" size=\"60\" value=\"" + String(config.aprs_mycall) + ">APE32I,WIDE1-1:>Test Status\"/></div>\n");
	// strcat(webString, "<div style=\"margin-left: 20px;\"><input type='submit' class=\"btn btn-primary\" name=\"sendRaw\" value='SEND RAW'></div> <br />\n");
	// strcat(webString, "</form></td></tr>\n");
	// strcat(webString, "<tr><td><hr width=\"80%\" /></td></tr>\n");
	strcat(webString, "<tr><td><div id=\"vumeter\" style=\"width: 300px; height: 200px; margin: 10px;\"></div></td>\n");
	strcat(webString, "<tr><td><div style=\"margin: 15px;\">Terminal<br /><textarea id=\"raw_txt\" name=\"raw_txt\" rows=\"50\" cols=\"80\" /></textarea></div></td></tr>\n");
	strcat(webString, "</table>\n");

	strcat(webString, "</body></html>\n");

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)webString);
	response->addHeader("Test", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	free(webString); // Free the allocated memory
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

	// Allocate memory for HTML content
	char *webString = allocateStringMemory(8192); // Start with 8KB buffer
	if (!webString)
	{
		request->send(500, "text/html", "Memory allocation failed");
		return;
	}
	strcpy(webString, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;\"><td width=\"49%\" style=\"border:unset;\">");

	strcat(webString, "<table>");
	strcat(webString, "<th colspan=\"2\"><span><b>System Information</b></span></th>\n");
	// strcat(webString, "<tr><th width=\"200\"><span><b>Name</b></span></th><th><span><b>Information</b></span></th></tr>";
	strcat(webString, "<tr><td align=\"right\"><b>Hardware Version: </b></td><td align=\"left\">");
#if defined(CONFIG_IDF_TARGET_ESP32)
	strcat(webString, "ESP32-WROOM,ESP32 DoIt DevKit");
#elif defined(ESP32C3_MINI)
	strcat(webString, "ESP32C3-Mini,ESP32-C3 DIY");
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
	strcat(webString, "ESP32C3,ESP32-C3 DIY");
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
	strcat(webString, "ESP32C6,ESP32-C6 DIY");
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
	strcat(webString, "ESP32-S3-DevKit,ESP32-S3-WROOM");
#else
	strcat(webString, "UNKNOWN,ESP32 DIY");
#endif
	strcat(webString, "</td></tr>");

	char *temp_str = allocateStringMemory(512);
	if (temp_str)
	{
		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>Firmware Version: </b></td><td align=\"left\"> V%s%c</td></tr>\n", VERSION, VERSION_BUILD);
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>RF Module: </b></td><td align=\"left\"> %s</td></tr>\n", RF_TYPE[config.rf_type]);
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>ESP32 Model: </b></td><td align=\"left\"> %s</td></tr>", ESP.getChipModel());
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>Revision: </b></td><td align=\"left\"> %d</td></tr>", ESP.getChipRevision());
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>Chip ID: </b></td><td align=\"left\"> %s</td></tr>", strCID);
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>Flash: </b></td><td align=\"left\">%d KByte</td></tr>", ESP.getFlashChipSize() / 1024);
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>PSRAM: </b></td><td align=\"left\">%.1f/%.1f KByte</td></tr>",
				 (float)ESP.getFreePsram() / 1024, (float)ESP.getPsramSize() / 1024);
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>FILE SYSTEM: </b></td><td align=\"left\">%.1f/%.1f KByte</td></tr>",
				 (float)LITTLEFS.usedBytes() / 1024, (float)LITTLEFS.totalBytes() / 1024);
		strcat(webString, temp_str);

		free(temp_str);
	}

	strcat(webString, "</table>");
	strcat(webString, "</td><td width=\"2%\" style=\"border:unset;\"></td>");
	strcat(webString, "<td width=\"49%\" style=\"border:unset;\">");

	strcat(webString, "<table>");
	strcat(webString, "<th colspan=\"2\"><span><b>Developer/Support Information</b></span></th>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Author: </b></td><td align=\"left\">Mr.Somkiat Nakhonthai </td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Callsign: </b></td><td align=\"left\">HS5TQA,Atten,Nakhonthai</td></tr>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Country: </b></td><td align=\"left\">Bangkok,Thailand</td></tr>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Github: </b></td><td align=\"left\"><a href=\"https://github.com/nakhonthai\" target=\"_github\">https://github.com/nakhonthai</a></td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Youtube: </b></td><td align=\"left\"><a href=\"https://www.youtube.com/@HS5TQA\" target=\"_youtube\">https://www.youtube.com/@HS5TQA</a></td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Facebook: </b></td><td align=\"left\"><a href=\"https://www.facebook.com/atten\" target=\"_facebook\">https://www.facebook.com/atten</a></td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Chat: </b></td><td align=\"left\">Telegram:<a href=\"https://t.me/HS5TQA\" target=\"_line\">@HS5TQA</a> , WeChat:HS5TQA</td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Sponsors: </b></td><td align=\"left\"><a href=\"https://github.com/sponsors/nakhonthai\" target=\"_sponsor\">https://github.com/sponsors/nakhonthai</a></td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Donate: </b></td><td align=\"left\"><a href=\"https://www.paypal.me/0hs5tqa0\" target=\"_sponsor\">https://www.paypal.me/0hs5tqa0</a></td></tr>");

	strcat(webString, "</table>");
	strcat(webString, "</td></tr></table><br />");

	strcat(webString, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;\"><td width=\"49%\" style=\"border:unset;\">");

	strcat(webString, "<table>\n");
	strcat(webString, "<th colspan=\"2\"><span><b>WiFi Status</b></span></th>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Mode:</b></td>\n");
	strcat(webString, "<td align=\"left\">");
	if (config.wifi_mode == WIFI_AP_FIX)
	{
		strcat(webString, "AP");
	}
	else if (config.wifi_mode == WIFI_STA_FIX)
	{
		strcat(webString, "STA");
	}
	else if (config.wifi_mode == WIFI_AP_STA_FIX)
	{
		strcat(webString, "AP+STA");
	}
	else
	{
		strcat(webString, "OFF");
	}
	uint8_t proto = 0;
	esp_wifi_get_protocol(WIFI_IF_STA, &proto);
	strcat(webString, " (802.11");
	if (proto & WIFI_PROTOCOL_11B)
		strcat(webString, "b");
	if (proto & WIFI_PROTOCOL_11G)
		strcat(webString, "g");
	if (proto & WIFI_PROTOCOL_11N)
		strcat(webString, "n");
	if (proto & WIFI_PROTOCOL_LR)
		strcat(webString, "lr");
	strcat(webString, ")");

	wifi_power_t wpr = WiFi.getTxPower();
	char wifipower[20] = "";
	if (wpr < 8)
	{
		strcpy(wifipower, "-1 dBm");
	}
	else if (wpr < 21)
	{
		strcpy(wifipower, "2 dBm");
	}
	else if (wpr < 29)
	{
		strcpy(wifipower, "5 dBm");
	}
	else if (wpr < 35)
	{
		strcpy(wifipower, "8.5 dBm");
	}
	else if (wpr < 45)
	{
		strcpy(wifipower, "11 dBm");
	}
	else if (wpr < 53)
	{
		strcpy(wifipower, "13 dBm");
	}
	else if (wpr < 61)
	{
		strcpy(wifipower, "15 dBm");
	}
	else if (wpr < 69)
	{
		strcpy(wifipower, "17 dBm");
	}
	else if (wpr < 75)
	{
		strcpy(wifipower, "18.5 dBm");
	}
	else if (wpr < 77)
	{
		strcpy(wifipower, "19 dBm");
	}
	else if (wpr < 80)
	{
		strcpy(wifipower, "19.5 dBm");
	}
	else
	{
		strcpy(wifipower, "20 dBm");
	}

	strcat(webString, "</td></tr>\n");

	char *temp_str1 = allocateStringMemory(512);
	if (temp_str1)
	{
		snprintf(temp_str1, 512, "<tr><td align=\"right\" width=\"30%%\"><b>MAC:</b></td>\n<td align=\"left\">%s</td></tr>\n", WiFi.macAddress().c_str());
		strcat(webString, temp_str1);
		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>Channel:</b></td>\n<td align=\"left\">%d</td></tr>\n", WiFi.channel());
		strcat(webString, temp_str1);

		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>TX Power:</b></td>\n<td align=\"left\">%s</td></tr>\n", wifipower);
		strcat(webString, temp_str1);

		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>SSID:</b></td>\n<td align=\"left\">%s</td></tr>\n", WiFi.SSID().c_str());
		strcat(webString, temp_str1);

		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>Local IP:</b></td>\n<td align=\"left\">%s</td></tr>\n", WiFi.localIP().toString().c_str());
		strcat(webString, temp_str1);

		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>Gateway IP:</b></td>\n<td align=\"left\">%s</td></tr>\n", WiFi.gatewayIP().toString().c_str());
		strcat(webString, temp_str1);
		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>DNS:</b></td>\n<td align=\"left\">%s</td></tr>\n", WiFi.dnsIP().toString().c_str());
		strcat(webString, temp_str1);

		free(temp_str1);
	}

	strcat(webString, "</table>\n");

	strcat(webString, "</td><td width=\"2%\" style=\"border:unset;\"></td>");
	strcat(webString, "<td width=\"49%\" style=\"border:unset;\">");
	strcat(webString, "<table>\n");
#ifdef PPPOS
	strcat(webString, "<th colspan=\"2\"><span><b>PPPoS Status</b></span></th>\n");

	char *temp_str2 = allocateStringMemory(512);
	if (temp_str2)
	{
		snprintf(temp_str2, 512, "<tr><td align=\"right\" width=\"30%%\"><b>Manufacturer:</b></td>\n<td align=\"left\">%s</td></tr>\n", pppStatus.manufacturer);
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>Model:</b></td>\n<td align=\"left\">%s</td></tr>\n", pppStatus.model);
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>IMEI:</b></td>\n<td align=\"left\">%s</td></tr>\n", pppStatus.imei);
		strcat(webString, temp_str2);
		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>IMSI:</b></td>\n<td align=\"left\">%s</td></tr>\n", pppStatus.imsi);
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>Operator:</b></td>\n<td align=\"left\">%s</td></tr>\n", pppStatus.oper);
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>RSSI:</b></td>\n<td align=\"left\">%d dBm</td></tr>\n", pppStatus.rssi);
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>IP:</b></td>\n<td align=\"left\">%s</td></tr>\n", IPAddress(pppStatus.ip).toString().c_str());
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>Gateway:</b></td>\n<td align=\"left\">%s</td></tr>\n", IPAddress(pppStatus.gateway).toString().c_str());
		strcat(webString, temp_str2);
		free(temp_str2);
	}

// strcat(webString, "<tr><td align=\"right\"><b>DNS:</b></td>\n");
// strcat(webString, "<td align=\"left\">%s</td></tr>\n", IPAddress(pppStatus.dns).toString().c_str());
#endif
	strcat(webString, "</table>\n");
	strcat(webString, "</td></tr></table><br />");

	// strcat(webString, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;\"><td width=\"96%\" style=\"border:unset;\">");
	#ifndef NO_OTA
	strcat(webString, "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form' class=\"form-horizontal\">\n");
	strcat(webString, "<table>");
	strcat(webString, "<th colspan=\"2\"><span><b>Firmware Update</b></span></th>\n");
	strcat(webString, "<tr><td align=\"right\"><b>File:</b></td><td align=\"left\"><input id=\"file\" name=\"update\" type=\"file\" onchange='sub(this)' /></td></tr>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Progress:</b></td><td><div id='prgbar'><div id='bar' style=\"width: 0px;\"><label id='prg'></label></div></div></td></tr>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Support Firmware:</b></td><td align=\"left\"><a target=\"_download\" href=\"https://github.com/nakhonthai/ESP32APRS_Audio/releases\">https://github.com/nakhonthai/ESP32APRS_Audio/releases</a></td></tr>\n");
	strcat(webString, "</table><br />\n");
	strcat(webString, "<div class=\"col-sm-3 col-xs-4\"><input type='submit' class=\"btn btn-danger\" id=\"update_sumbit\" value='Firmware Update'></div>\n");

	strcat(webString, "</form>\n");
	// strcat(webString, "</td></tr></table><br />");

	strcat(webString, "<script>"
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
					  "alert('Wait for system reboot 10sec');"
					  "},"
					  "error: function (a, b, c) {"
					  "}"
					  "});"
					  "});"
					  "</script>");
	#endif
	strcat(webString, "</body></html>\n");

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)webString);
	response->addHeader("About", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	free(webString);
}

void handle_gnss(AsyncWebServerRequest *request)
{
	// Allocate initial memory for HTML content
	char *webString = allocateStringMemory(8192); // Start with 8KB buffer
	if (!webString)
	{
		request->send(500, "text/html", "Memory allocation failed");
		return;
	}
	strcpy(webString, "<html>\n<head>\n");
	strcat(webString, "<script src=\"https://apps.bdimg.com/libs/jquery/2.1.4/jquery.min.js\"></script>\n");
	strcat(webString, "<script src=\"https://code.highcharts.com/highcharts.js\"></script>\n");
	strcat(webString, "<script src=\"https://code.highcharts.com/highcharts-more.js\"></script>\n");
	strcat(webString, "<script language=\"JavaScript\">");

	// Add some life
	strcat(webString, "function gnss() { \n"); // the chart may be destroyed
	strcat(webString, "var raw=\"\";var timeStamp;\n");
	strcat(webString, "var host='ws://'+location.hostname+':81/ws_gnss'\n");
	strcat(webString, "const ws = new WebSocket(host);\n");
	strcat(webString, "ws.onopen = function() { console.log('Connection opened');};\n ws.onclose = function() { console.log('Connection closed');};\n");
	strcat(webString, "ws.onmessage = function(event) {\n  console.log(event.data);\n");
	strcat(webString, "const jsonR=JSON.parse(event.data);\n");
	strcat(webString, "document.getElementById(\"en\").innerHTML=parseInt(jsonR.en);\n");
	strcat(webString, "document.getElementById(\"lat\").innerHTML=parseFloat(jsonR.lat);\n");
	strcat(webString, "document.getElementById(\"lng\").innerHTML=parseFloat(jsonR.lng);\n");
	strcat(webString, "document.getElementById(\"alt\").innerHTML=parseFloat(jsonR.alt);\n");
	strcat(webString, "document.getElementById(\"spd\").innerHTML=parseFloat(jsonR.spd);\n");
	strcat(webString, "document.getElementById(\"csd\").innerHTML=parseFloat(jsonR.csd);\n");
	strcat(webString, "document.getElementById(\"hdop\").innerHTML=parseFloat(jsonR.hdop);\n");
	strcat(webString, "document.getElementById(\"sat\").innerHTML=parseInt(jsonR.sat);\n");
	strcat(webString, "document.getElementById(\"time\").innerHTML=parseInt(jsonR.time);\n");
	strcat(webString, "raw=jsonR.RAW;\n");
	strcat(webString, "timeStamp=Number(jsonR.timeStamp);\n");
	strcat(webString, "var textArea=document.getElementById(\"raw_txt\");\n");
	strcat(webString, "textArea.value+=atob(raw)+\"\\n\";\n");
	strcat(webString, "textArea.scrollTop = textArea.scrollHeight;\n");
	strcat(webString, "}\n");
	strcat(webString, "};\n</script>\n");
	strcat(webString, "</head><body onload=\"gnss()\">\n");

	strcat(webString, "<table width=\"200\" border=\"1\">");
	strcat(webString, "<th colspan=\"2\" style=\"background-color: #00BCD4;\"><span><b>GNSS Information</b></span></th>\n");
	// strcat(webString, "<tr><th width=\"200\"><span><b>Name</b></span></th><th><span><b>Information</b></span></th></tr>");

	{
		char *temp_en = allocateStringMemory(512);
		if (temp_en)
		{
			snprintf(temp_en, 512, "<tr><td align=\"right\"><b>Enable: </b></td><td align=\"left\"> <label id=\"en\">%d</label></td></tr>", config.gnss_enable);
			strcat(webString, temp_en);
			free(temp_en);
		}
	}

	{
		char *temp_lat = allocateStringMemory(512);
		if (temp_lat)
		{
			snprintf(temp_lat, 512, "<tr><td align=\"right\"><b>Latitude: </b></td><td align=\"left\"> <label id=\"lat\">%.5f</label></td></tr>", gps.location.lat());
			strcat(webString, temp_lat);
			free(temp_lat);
		}
	}

	{
		char *temp_lng = allocateStringMemory(512);
		if (temp_lng)
		{
			snprintf(temp_lng, 512, "<tr><td align=\"right\"><b>Longitude: </b></td><td align=\"left\"> <label id=\"lng\">%.5f</label></td></tr>", gps.location.lng());
			strcat(webString, temp_lng);
			free(temp_lng);
		}
	}

	{
		char *temp_alt = allocateStringMemory(512);
		if (temp_alt)
		{
			snprintf(temp_alt, 512, "<tr><td align=\"right\"><b>Altitude: </b></td><td align=\"left\"> <label id=\"alt\">%.2f</label> m.</td></tr>", gps.altitude.meters());
			strcat(webString, temp_alt);
			free(temp_alt);
		}
	}

	{
		char *temp_spd = allocateStringMemory(512);
		if (temp_spd)
		{
			snprintf(temp_spd, 512, "<tr><td align=\"right\"><b>Speed: </b></td><td align=\"left\"> <label id=\"spd\">%.2f</label> km/h</td></tr>", gps.speed.kmph());
			strcat(webString, temp_spd);
			free(temp_spd);
		}
	}

	{
		char *temp_csd = allocateStringMemory(512);
		if (temp_csd)
		{
			snprintf(temp_csd, 512, "<tr><td align=\"right\"><b>Course: </b></td><td align=\"left\"> <label id=\"csd\">%.1f</label></td></tr>", gps.course.deg());
			strcat(webString, temp_csd);
			free(temp_csd);
		}
	}

	{
		char *temp_hdop = allocateStringMemory(512);
		if (temp_hdop)
		{
			snprintf(temp_hdop, 512, "<tr><td align=\"right\"><b>HDOP: </b></td><td align=\"left\"> <label id=\"hdop\">%.2f</label> </td></tr>", gps.hdop.hdop());
			strcat(webString, temp_hdop);
			free(temp_hdop);
		}
	}

	{
		char *temp_sat = allocateStringMemory(512);
		if (temp_sat)
		{
			snprintf(temp_sat, 512, "<tr><td align=\"right\"><b>SAT: </b></td><td align=\"left\"> <label id=\"sat\">%d</label> </td></tr>", gps.satellites.value());
			strcat(webString, temp_sat);
			free(temp_sat);
		}
	}

	{
		char *temp_time = allocateStringMemory(512);
		if (temp_time)
		{
			snprintf(temp_time, 512, "<tr><td align=\"right\"><b>Time: </b></td><td align=\"left\"> <label id=\"time\">%d</label> </td></tr>", gps.time.value());
			strcat(webString, temp_time);
			free(temp_time);
		}
	}

	strcat(webString, "</table><table>");
	strcat(webString, "<tr><td><b>Terminal:</b><br /><textarea id=\"raw_txt\" name=\"raw_txt\" rows=\"30\" cols=\"80\" /></textarea></td></tr>\n");
	strcat(webString, "</table>\n");

	strcat(webString, "</body></html>\n");

	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const char *)webString);
	response->addHeader("GNSS", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	free(webString);
}

void handle_default()
{
	defaultSetting = true;
	defaultConfig();
	defaultSetting = false;
}

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index) {
        // Open the file in write mode on the first chunk
        request->_tempFile = LITTLEFS.open("/" + filename, "w");
    }
    if (len < (LITTLEFS.totalBytes()-LITTLEFS.usedBytes())) {
        // Write the data chunk to the file
        request->_tempFile.write(data, len);
    }else{
		// Not enough space to write the file
		request->_tempFile.close();
		LITTLEFS.remove("/" + filename); // Remove the incomplete file
		request->send(500, "text/plain", "Not enough space to upload the file");
		return;
	}
    if (final) {
        // Close the file on the last chunk and redirect
        request->_tempFile.close();
        request->redirect("/"); // Redirect back to the main page
    }
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

// void handle_vpn_request(AsyncWebServerRequest *request) {
//     HTTPClient http;

//     String url = "http://vpn.nakhonthai.net:82/wg/create";

//     String mac = WiFi.macAddress();
//     mac.replace(":", "");

//     String payload = "{\"name\":\"" + mac + "\"}";

//     http.begin(url);
//     http.addHeader("Content-Type", "application/json");

//     int httpCode = http.POST(payload);

//     if (httpCode > 0) {
//         String res = http.getString();
//         request->send(200, "application/json", res);
//     } else {
//         request->send(500, "text/plain", "Error contacting VPN server");
//     }

//     http.end();
// }

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
	// Route to handle the file upload
    async_server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "File uploaded successfully!");
    }, handleUpload); // Pass the handleUpload function as the upload handler

	// async_server.on("/api/vpnreq", HTTP_GET, handle_vpn_request);
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
					adcEn = -1;
					dacEn = -1;
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
    //String html = event_lastHeard(true);
    //client->send(html.c_str(), "lastHeard", time(NULL), 5000); 
});
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
