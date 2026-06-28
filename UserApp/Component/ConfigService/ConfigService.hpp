#ifndef __CONFIG_SERVICE_COMPONENT_HPP
#define __CONFIG_SERVICE_COMPONENT_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace auv {
namespace component {

class ConfigService {
public:
  /**
   * @brief 更新系统参数 (零分配接口)
   * @param json JSON 格式的更新请求 (可选)
   * @param paths 路径数组指针
   * @param values 值数组指针
   * @param count 数组长度
   * @param out_buf 结果说明输出缓冲区
   * @param out_size 缓冲区大小
   */
  static bool updateParams(const char *json, const char **paths,
                           const char **values, size_t count, char *out_buf,
                           size_t out_size);

  /**
   * @brief 获取系统参数 (使用内部静态缓冲区返回)
   * @param req_paths 请求的前缀或路径数组指针
   * @param req_count 数组长度
   * @return 返回指向内部静态缓冲区的指针，内容为 JSON 字符串
   */
  static const char *getParamsJson(const char **req_paths, size_t req_count);

  // 辅助函数
  static char *append_str(char *buf, size_t size, size_t &pos, const char *str);
  static char *append_float_fixed(char *buf, size_t size, size_t &pos, float v,
                                  int prec = 4);
};

} // namespace component
} // namespace auv

#endif
