// V3.0 smoke test. Confirms the native test toolchain (Unity + PlatformIO)
// is wired correctly. Runs on the host, no hardware required. Real module
// tests will join this directory as src/<module>/ lands.
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

void test_toolchain_alive(void) {
    TEST_ASSERT_EQUAL(2, 1 + 1);
}

void test_string_compare(void) {
    TEST_ASSERT_EQUAL_STRING("hello", "hello");
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_toolchain_alive);
    RUN_TEST(test_string_compare);
    return UNITY_END();
}
