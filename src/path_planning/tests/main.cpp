// Copyright (C) 2026 by RoboTeam Twente

#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>

uint64_t fib(uint64_t n) { return n < 2 ? 1 : fib(n - 1) + fib(n - 2); }

TEST_CASE("hello world", "[path_planning]") {
    REQUIRE(2 + 2 == 4);
    REQUIRE_FALSE(2 + 2 == 5);

    BENCHMARK("fib(20)") { return fib(20); };
}
