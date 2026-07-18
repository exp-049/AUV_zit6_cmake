#include "ConfigService.hpp"
#include "AppContext.hpp"
#include "SoftWatchdog.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "cJSON.h"
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace auv {
namespace component {

static char g_json_build_buf[4096];
static char g_path_walk_buf[128];

class JsonWriter {
  char *buf;
  size_t size;
  size_t &pos;
  bool first = true;

public:
  JsonWriter(char *b, size_t s, size_t &p) : buf(b), size(s), pos(p) {
    pos = 0;
  }
  void startObj() {
    append("{");
    first = true;
  }
  void endObj() { append("}"); }
  void pair(const char *key, float v, int p = 4) {
    pre();
    append("\"");
    append(key);
    append("\":");
    ConfigService::append_float_fixed(buf, size, pos, v, p);
  }
  void pair(const char *key, bool v) {
    pre();
    append("\"");
    append(key);
    append("\":");
    append(v ? "true" : "false");
  }
  void pair(const char *key, const char *v) {
    pre();
    append("\"");
    append(key);
    append("\":\"");
    append(v);
    append("\"");
  }

private:
  void pre() {
    if (!first)
      append(",");
    first = false;
  }
  void append(const char *s) { ConfigService::append_str(buf, size, pos, s); }
};

static void copyParamString(void *ptr, const char *value) {
  if (!ptr || !value)
    return;
  // Convention: STRING parameters must point to a writable buffer >= 64 bytes.
  char *dst = static_cast<char *>(ptr);
  strncpy(dst, value, 63);
  dst[63] = '\0';
}

static void toLowerCopy(const char *src, char *dst, size_t dst_size) {
  if (!dst || dst_size == 0)
    return;
  if (!src) {
    dst[0] = '\0';
    return;
  }
  size_t i = 0;
  for (; i + 1 < dst_size && src[i]; ++i) {
    dst[i] = (char)std::tolower((unsigned char)src[i]);
  }
  dst[i] = '\0';
}

static bool startsWith(const char *s, const char *prefix) {
  if (!s || !prefix)
    return false;
  return strncmp(s, prefix, strlen(prefix)) == 0;
}

static auv::config::ParamType
resolveParamType(const auv::config::ParamMeta &p) {
  using auv::config::ParamType;
  if (!p.path)
    return p.type;
  if (strcmp(p.path, "soft_watchdog.timeout_ms") == 0)
    return ParamType::UINT32;
  if (p.type == ParamType::UINT32) {
    if (startsWith(p.path, "chassis.") || startsWith(p.path, "simulation.") ||
        startsWith(p.path, "ins.")) {
      return ParamType::FLOAT;
    }
  }
  return p.type;
}

static bool dispatchUpdate(const char *path, const char *value, bool &updated) {
  using namespace auv::config;
  for (size_t i = 0; i < SYSTEM_PARAMS_COUNT; ++i) {
    const auto &p = SYSTEM_PARAMS[i];
    if (p.path && strcmp(p.path, path) == 0) {
#ifdef USE_DEPTH_CALC_BOARD
      // The production depth-calculation board owns navigation z. Do not
      // accept a runtime request that would select an INS-based z source.
      if (p.type == ParamType::ENUM_Z) {
        *(ZDataSource *)p.ptr = ZDataSource::USE_MS5837_Z;
        return false;
      }
#endif
      const auto effective_type = resolveParamType(p);
      switch (effective_type) {
      case ParamType::FLOAT:
        *(float *)p.ptr = strtof(value, nullptr);
        break;
      case ParamType::UINT32:
        *(uint32_t *)p.ptr = (uint32_t)strtoul(value, nullptr, 10);
        break;
      case ParamType::INT32:
        *(int32_t *)p.ptr = (int32_t)strtol(value, nullptr, 10);
        break;
      case ParamType::BOOL:
        *(bool *)p.ptr =
            (strcmp(value, "true") == 0 || strcmp(value, "1") == 0);
        break;
      case ParamType::STRING:
        copyParamString(p.ptr, value);
        break;
      case ParamType::ENUM_Z: {
        char lower[32];
        toLowerCopy(value, lower, sizeof(lower));
        if (strstr(lower, "ms5837"))
          *(ZDataSource *)p.ptr = ZDataSource::USE_MS5837_Z;
        else if (strstr(lower, "manometer") || strstr(lower, "pressure"))
          *(ZDataSource *)p.ptr = ZDataSource::USE_INS_PRESSURE_Z;
        else
          *(ZDataSource *)p.ptr = ZDataSource::USE_INS_INTEGRATED_Z;
      } break;
      default:
        return false;
      }
      updated = true;
      return true;
    }
  }
  return false;
}

static void walkJson(cJSON *item, char *path_buf, size_t depth, bool &updated) {
  while (item) {
    size_t len = strlen(path_buf);
    if (depth > 0)
      strcat(path_buf, ".");
    strcat(path_buf, item->string);
    if (item->type == cJSON_Object) {
      walkJson(item->child, path_buf, depth + 1, updated);
    } else {
      char val_str[32] = {0};
      if (cJSON_IsNumber(item)) {
        size_t vpos = 0;
        ConfigService::append_float_fixed(val_str, 32, vpos,
                                          (float)item->valuedouble, 6);
      } else if (cJSON_IsBool(item))
        strcpy(val_str, cJSON_IsTrue(item) ? "true" : "false");
      else if (cJSON_IsString(item))
        strncpy(val_str, item->valuestring, 31);
      dispatchUpdate(path_buf, val_str, updated);
    }
    path_buf[len] = '\0';
    item = item->next;
  }
}

bool ConfigService::updateParams(const char *json, const char **paths,
                                 const char **values, size_t count,
                                 char *out_buf, size_t out_size) {
  bool updated = false;
  if (json && json[0]) {
    cJSON *root = cJSON_Parse(json);
    if (root) {
      g_path_walk_buf[0] = '\0';
      walkJson(root, g_path_walk_buf, 0, updated);
      cJSON_Delete(root);
    }
  }
  for (size_t i = 0; i < count; ++i) {
    if (paths[i] && values[i]) {
      dispatchUpdate(paths[i], values[i], updated);
    }
  }
  if (updated) {
    // 移除临界区：避免在 Micro-ROS 回调中死锁。简单的内存赋值在 32
    // 位系统上大部分是原子的。
    auv::system::g_app_ctx.chassis->applyConfig(
        auv::config::sys_config.chassis);
    auv::system::g_app_ctx.watchdog->init(
        auv::config::sys_config.system.soft_watchdog);
    auv::system::system_context.planner_replan_flag = true;
    if (out_buf)
      strncpy(out_buf, "ok", out_size - 1);
  } else {
    if (out_buf)
      strncpy(out_buf, "not found", out_size - 1);
  }
  return updated;
}

const char *ConfigService::getParamsJson(const char **req_paths,
                                         size_t req_count) {
  using namespace auv::config;
  size_t pos = 0;
  JsonWriter jw(g_json_build_buf, sizeof(g_json_build_buf), pos);
  jw.startObj();
  bool any_match = false;
  for (size_t i = 0; i < SYSTEM_PARAMS_COUNT; ++i) {
    const auto &p = SYSTEM_PARAMS[i];
    if (!p.path)
      continue;
    bool include = (req_count == 0);
    if (!include) {
      for (size_t j = 0; j < req_count; ++j) {
        if (req_paths[j] &&
            strncmp(p.path, req_paths[j], strlen(req_paths[j])) == 0) {
          include = true;
          break;
        }
      }
    }
    if (include) {
      any_match = true;
      const auto effective_type = resolveParamType(p);
      switch (effective_type) {
      case ParamType::FLOAT:
        jw.pair(p.path, *(float *)p.ptr);
        break;
      case ParamType::UINT32:
        jw.pair(p.path, (float)*(uint32_t *)p.ptr, 0);
        break;
      case ParamType::INT32:
        jw.pair(p.path, (float)*(int32_t *)p.ptr, 0);
        break;
      case ParamType::BOOL:
        jw.pair(p.path, *(bool *)p.ptr);
        break;
      case ParamType::STRING:
        jw.pair(p.path, (const char *)p.ptr);
        break;
      case ParamType::ENUM_Z:
        if (*(ZDataSource *)p.ptr == ZDataSource::USE_MS5837_Z) {
          jw.pair(p.path, "use_ms5837_z");
        } else if (*(ZDataSource *)p.ptr == ZDataSource::USE_INS_PRESSURE_Z) {
          jw.pair(p.path, "use_ins_pressure_z");
        } else {
          jw.pair(p.path, "use_ins_integrated_z");
        }
        break;
      default:
        break;
      }
    }
  }
  jw.endObj();
  return any_match ? g_json_build_buf : "{}";
}

char *ConfigService::append_str(char *buf, size_t size, size_t &pos,
                                const char *str) {
  if (!str)
    return buf + pos;
  size_t len = strlen(str);
  if (pos + len < size) {
    memcpy(buf + pos, str, len);
    pos += len;
    buf[pos] = '\0';
  }
  return buf + pos;
}

char *ConfigService::append_float_fixed(char *buf, size_t size, size_t &pos,
                                        float v, int prec) {
  if (!buf || size == 0 || pos >= size - 1)
    return buf + pos;
  if (std::isnan(v))
    return append_str(buf, size, pos, "nan");
  if (std::isinf(v))
    return append_str(buf, size, pos, (v < 0) ? "-inf" : "inf");
  bool neg = false;
  if (v < 0.0f) {
    neg = true;
    v = -v;
  }
  uint32_t scale = 1;
  for (int i = 0; i < prec; ++i)
    scale *= 10;
  uint32_t scaled = (uint32_t)(v * (float)scale + 0.5f);
  uint32_t intpart = scaled / scale;
  uint32_t frac = scaled % scale;
  char tmp[32];
  sprintf(tmp, "%s%lu", neg ? "-" : "", (unsigned long)intpart);
  append_str(buf, size, pos, tmp);
  if (prec > 0 && pos < size - 1) {
    buf[pos++] = '.';
    buf[pos] = '\0';
    char digits[16];
    int d = 0;
    uint32_t t = frac;
    while (d < prec) {
      digits[d++] = (char)('0' + (t % 10));
      t /= 10;
    }
    for (int i = prec - 1; i >= 0 && pos < size - 1; --i)
      buf[pos++] = digits[i];
    buf[pos] = '\0';
  }
  return buf + pos;
}

} // namespace component
} // namespace auv
