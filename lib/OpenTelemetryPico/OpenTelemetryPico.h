// lib/OpenTelemetryPico/OpenTelemetryPico.h
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

#ifndef OTEL_COLLECTOR_HOST
#define OTEL_COLLECTOR_HOST "192.168.1.100"
#endif

#ifndef OTEL_COLLECTOR_PORT
#define OTEL_COLLECTOR_PORT 4318
#endif

namespace OTel {

  enum LogLevel {
    DEBUG = 5,
    INFO = 9,
    WARN = 13,
    ERROR = 17
  };

  class Logger {
   public:
    static void begin(const char* service = "pico-app", const char* host = "", const char* custom = "") {
      strncpy(serviceName, service, sizeof(serviceName) - 1);
      strncpy(hostName, host, sizeof(hostName) - 1);
      strncpy(customLabel, custom, sizeof(customLabel) - 1);
      configTime(0, 0, "pool.ntp.org", "time.nist.gov");
      Serial.print("[OTel] Waiting for NTP sync");
      time_t now = time(nullptr);
      unsigned long start = millis();
      while (time(nullptr) < 100000 && millis() - start < 10000) {
        delay(100);
        Serial.print(".");
      }
      Serial.println((time(nullptr) >= 100000) ? " done." : " failed.");
    }

    static uint64_t currentTimeUnixNano() {
      struct timeval tv;
      gettimeofday(&tv, nullptr);
      return static_cast<uint64_t>(tv.tv_sec) * 1000000000ULL + static_cast<uint64_t>(tv.tv_usec) * 1000ULL;
    }

    static void log(const char* name, LogLevel level, const char* message) {
      HTTPClient http;

      JsonDocument doc;
      JsonArray resource_logs = doc["resource_logs"].to<JsonArray>();
      JsonObject resource_log = resource_logs.add<JsonObject>();
      JsonObject resource = resource_log["resource"].to<JsonObject>();
      JsonArray resource_attrs = resource["attributes"].to<JsonArray>();

      JsonObject attr1 = resource_attrs.add<JsonObject>();
      attr1["key"] = "service.name";
      attr1["value"].to<JsonObject>()["string_value"] = serviceName;

      JsonObject attr2 = resource_attrs.add<JsonObject>();
      attr2["key"] = "host.name";
      attr2["value"].to<JsonObject>()["string_value"] = hostName;

      JsonObject attr3 = resource_attrs.add<JsonObject>();
      attr3["key"] = "custom.label";
      attr3["value"].to<JsonObject>()["string_value"] = customLabel;

      JsonArray scope_logs = resource_log["scope_logs"].to<JsonArray>();
      JsonObject scope_log = scope_logs.add<JsonObject>();
      JsonObject scope = scope_log["instrumentation_scope"].to<JsonObject>();
      scope["name"] = "otel-embedded";
      scope["version"] = "1.0";
      scope_log["schema_url"] = "https://opentelemetry.io/schemas/1.11.0";

      JsonArray log_records = scope_log["log_records"].to<JsonArray>();
      JsonObject record = log_records.add<JsonObject>();

      uint64_t now = currentTimeUnixNano();
      record["time_unix_nano"] = now;
      record["observed_time_unix_nano"] = now;
      record["severity_number"] = level;
      record["severity_text"] =
        level == DEBUG ? "DEBUG" :
        level == INFO ? "INFO" :
        level == WARN ? "WARN" : "ERROR";
      record["body"].to<JsonObject>()["string_value"] = message;
      record["dropped_attributes_count"] = 0;

      String output;
      serializeJson(doc, output);
      Serial.print("[OTel JSON] ");
      Serial.println(output);

      String url = String("http://") + OTEL_COLLECTOR_HOST + ":" + OTEL_COLLECTOR_PORT + "/v1/logs";
      http.begin(url);
      http.addHeader("Content-Type", "application/json");

      int httpCode = http.POST(output);
      Serial.printf("[OTel] HTTP POST code: %d\n", httpCode);
      String payload = http.getString();
      Serial.println(payload);
      http.end();
    }

    static void logInfo(const char* message) { log("info", INFO, message); }
    static void logWarn(const char* message) { log("warn", WARN, message); }
    static void logError(const char* message) { log("error", ERROR, message); }

    static inline char serviceName[64] = "pico-app";
    static inline char hostName[64] = "";
    static inline char customLabel[64] = "";
  };

}  // namespace OTel

