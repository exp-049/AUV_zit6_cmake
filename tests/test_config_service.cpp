/**
 * @file test_config_service.cpp
 * @brief ConfigService 辅助函数主机端单元测试
 *
 * 测试策略：
 * - append_str：字符串追加、缓冲区溢出保护
 * - append_float_fixed：浮点数格式化、精度、负数
 *
 * 注意：getParamsJson/updateParams 深度耦合硬件驱动，不在本文件测试。
 */

#include <gtest/gtest.h>
#include <cstring>
#include <cstddef>
#include <cstdio>

// ============================================================================
// 辅助函数实现（从 ConfigService.cpp 提取，纯字符串操作，无硬件依赖）
// ============================================================================

static char* append_str(char* buf, size_t size, size_t& pos, const char* str) {
  if (!buf || !str || pos >= size) return buf;
  while (pos < size - 1 && *str) {
    buf[pos++] = *str++;
  }
  buf[pos] = '\0';
  return buf;
}

static char* append_float_fixed(char* buf, size_t size, size_t& pos,
                                float v, int prec = 4) {
  if (!buf || pos >= size) return buf;
  int n = snprintf(buf + pos, size - pos, "%.*f", prec, (double)v);
  if (n > 0) {
    size_t written = (size_t)(n < (int)(size - pos) ? n : (int)(size - pos - 1));
    pos += written;
  }
  return buf;
}

// ============================================================================
// append_str 测试
// ============================================================================

TEST(ConfigServiceHelpers, AppendStr) {
  char buf[64] = {0};
  size_t pos = 0;
  char *ret = append_str(buf, sizeof(buf), pos, "hello");
  EXPECT_EQ(ret, buf);
  EXPECT_EQ(pos, 5u);
  EXPECT_STREQ(buf, "hello");
}

TEST(ConfigServiceHelpers, AppendStrFullBuffer) {
  char buf[4] = {'a', 'b', 0, 0};  // 已有前缀 "ab"
  size_t pos = 2;
  char *ret = append_str(buf, sizeof(buf), pos, "hello");
  EXPECT_EQ(ret, buf);
  EXPECT_EQ(pos, 3u);  // 写入 'h' 后 pos=3，剩余空间不足
  EXPECT_STREQ(buf, "abh");  // "ab" + 'h' + '\0'
}

TEST(ConfigServiceHelpers, AppendStrNullInput) {
  char buf[16] = "test";
  size_t pos = 4;
  append_str(buf, sizeof(buf), pos, nullptr);
  EXPECT_EQ(pos, 4u);  // 不改变
}

TEST(ConfigServiceHelpers, AppendStrEmptyString) {
  char buf[16] = {0};
  size_t pos = 0;
  append_str(buf, sizeof(buf), pos, "");
  EXPECT_EQ(pos, 0u);
  EXPECT_STREQ(buf, "");
}

// ============================================================================
// append_float_fixed 测试
// ============================================================================

TEST(ConfigServiceHelpers, AppendFloatFixed) {
  char buf[32] = {0};
  size_t pos = 0;
  append_float_fixed(buf, sizeof(buf), pos, 3.14159f, 3);
  EXPECT_STREQ(buf, "3.142");
  EXPECT_EQ(pos, 5u);
}

TEST(ConfigServiceHelpers, AppendFloatFixedInteger) {
  char buf[32] = {0};
  size_t pos = 0;
  append_float_fixed(buf, sizeof(buf), pos, 42.0f, 2);
  EXPECT_STREQ(buf, "42.00");
}

TEST(ConfigServiceHelpers, AppendFloatNegative) {
  char buf[32] = {0};
  size_t pos = 0;
  append_float_fixed(buf, sizeof(buf), pos, -0.5f, 1);
  EXPECT_STREQ(buf, "-0.5");
}

TEST(ConfigServiceHelpers, AppendFloatZero) {
  char buf[32] = {0};
  size_t pos = 0;
  append_float_fixed(buf, sizeof(buf), pos, 0.0f, 2);
  EXPECT_STREQ(buf, "0.00");
}

TEST(ConfigServiceHelpers, AppendFloatDefaultPrecision) {
  char buf[32] = {0};
  size_t pos = 0;
  append_float_fixed(buf, sizeof(buf), pos, 1.234567f);
  EXPECT_STREQ(buf, "1.2346");
}

// ============================================================================
// 组合测试：多次追加
// ============================================================================

TEST(ConfigServiceHelpers, MultipleAppends) {
  char buf[64] = {0};
  size_t pos = 0;
  append_str(buf, sizeof(buf), pos, "x=");
  append_float_fixed(buf, sizeof(buf), pos, 3.14f, 2);
  append_str(buf, sizeof(buf), pos, ", y=");
  append_float_fixed(buf, sizeof(buf), pos, -0.5f, 1);
  EXPECT_STREQ(buf, "x=3.14, y=-0.5");
}
