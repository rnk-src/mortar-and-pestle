#include <unity.h>

#include "core/version.h"

void setUp() {}
void tearDown() {}

static void test_protocol_version_is_one() { TEST_ASSERT_EQUAL_UINT8(1, core::protocol_version()); }

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_protocol_version_is_one);
  return UNITY_END();
}
