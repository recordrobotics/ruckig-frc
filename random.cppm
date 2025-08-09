module;

#include <random>
#include <limits>
#include <cstdint>
#include <type_traits>

export module random;

namespace rnd::detail
{
    inline std::mt19937 &engine()
    {
        thread_local std::mt19937 eng{std::random_device{}()};
        return eng;
    }
}

namespace rnd
{
    // Seed the RNG for deterministic sequences (thread-local engine).
    export inline void seed(std::uint32_t value)
    {
        detail::engine().seed(value);
    }

    // Inclusive integer range [min, max]. Automatically swaps if min > max.
    export inline int random_int(int min, int max)
    {
        if (min > max)
            std::swap(min, max);
        std::uniform_int_distribution<int> dist(min, max);
        return dist(detail::engine());
    }

    // Generic integral version (excludes bool).
    export template <typename I>
        requires(std::is_integral_v<I> && !std::is_same_v<I, bool>)
    inline I random_integral(I min, I max)
    {
        if (min > max)
            std::swap(min, max);
        std::uniform_int_distribution<I> dist(min, max);
        return dist(detail::engine());
    }

    // Half-open floating range [min, max). Swaps if min > max.
    export inline double random_double(double min, double max)
    {
        if (min > max)
            std::swap(min, max);
        std::uniform_real_distribution<double> dist(min, max);
        return dist(detail::engine());
    }

    // Generic floating version.
    export template <typename F>
        requires std::is_floating_point_v<F>
    inline F random_floating(F min, F max)
    {
        if (min > max)
            std::swap(min, max);
        std::uniform_real_distribution<F> dist(min, max);
        return dist(detail::engine());
    }
}
