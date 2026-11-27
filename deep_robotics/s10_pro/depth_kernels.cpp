#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <vector>

namespace {
constexpr int W = 640, H = 480, N = W * H;
struct Surface {
    std::vector<int> depth, low, high;
    std::vector<unsigned char> rejected;
    Surface() : depth(N), low(N), high(N), rejected(N) {}
};

Surface bridge(const Surface& source, bool rows, int gap, int jump, double relative) {
    Surface result = source;
    const int lines = rows ? H : W, length = rows ? W : H;
    const int step = rows ? 1 : W;
    for (int line = 0; line < lines; ++line) {
        const int base = rows ? line * W : line;
        int left = -1;
        for (int right = 0; right < length; ++right) {
            const int last = base + right * step;
            if (!source.depth[last]) continue;
            if (left >= 0 && right - left - 1 <= gap) {
                const int first = base + left * step;
                const int low = std::min(source.low[first], source.low[last]);
                const int high = std::max(source.high[first], source.high[last]);
                const bool consistent = high - low <= jump && high - low <= relative * low;
                const int span = right - left;
                for (int column = left + 1; column < right; ++column) {
                    const int pixel = base + column * step;
                    if (!consistent) result.rejected[pixel] = 1;
                    if (!consistent || source.rejected[pixel]) continue;
                    result.depth[pixel] = (source.depth[first] * (right - column)
                        + source.depth[last] * (column - left) + span / 2) / span;
                    result.low[pixel] = low;
                    result.high[pixel] = high;
                }
            }
            left = right;
        }
    }
    return result;
}
}

extern "C" int s10_fill_holes(const unsigned char* raw, std::size_t bytes,
        unsigned char* output, int* counts, int gap, int jump, double relative) {
    if (!raw || !output || !counts || bytes != N * 2 || gap < 0 || gap > W
            || jump < 0 || !(relative >= 0)) return 2;
    try {
        Surface source;
        for (int i = 0; i < N; ++i) {
            const int value = raw[2 * i] | (int(raw[2 * i + 1]) << 8);
            source.depth[i] = source.low[i] = source.high[i] = value;
        }
        const auto horizontal = bridge(bridge(source, true, gap, jump, relative),
                                       false, gap, jump, relative);
        const auto vertical = bridge(bridge(source, false, gap, jump, relative),
                                     true, gap, jump, relative);
        counts[0] = counts[1] = 0;
        for (int i = 0; i < N; ++i) {
            const int a = horizontal.depth[i], b = vertical.depth[i];
            const bool both = a != 0 && b != 0;
            const int low = std::min(horizontal.low[i], vertical.low[i]);
            const int high = std::max(horizontal.high[i], vertical.high[i]);
            const bool disagreement = both && std::abs(a - b) > std::min(60., .02 * std::min(a, b));
            const bool support_conflict = both && (high - low > jump || high - low > relative * low);
            const bool candidate = !source.depth[i] && (a || b);
            const bool conflict = candidate && (horizontal.rejected[i] || vertical.rejected[i]
                                                || disagreement || support_conflict);
            int value = both ? (a + b + 1) / 2 : std::max(a, b);
            if (conflict) value = 0;
            if (source.depth[i]) value = source.depth[i];
            counts[0] += !source.depth[i] && value != 0;
            counts[1] += conflict;
            output[2 * i] = value & 255;
            output[2 * i + 1] = (value >> 8) & 255;
        }
        return 0;
    } catch (const std::bad_alloc&) {
        return 1;
    }
}

extern "C" int s10_window_sum(const double* values, std::size_t count, double* output) {
    if (!values || !output || count != N) return 2;
    try {
        constexpr int stride = W + 17;
        std::vector<double> integral((H + 17) * stride, 0.);
        std::vector<double> columns(W + 16, 0.);
        for (int y = 0; y < H + 16; ++y) {
            double row = 0.;
            for (int x = 0; x < W + 16; ++x) {
                if (y >= 8 && y < H + 8 && x >= 8 && x < W + 8)
                    columns[x] += values[(y - 8) * W + x - 8];
                row += columns[x];
                integral[(y + 1) * stride + x + 1] = row;
            }
        }
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x)
                output[y * W + x] = ((integral[(y + 17) * stride + x + 17]
                    - integral[y * stride + x + 17]) - integral[(y + 17) * stride + x])
                    + integral[y * stride + x];
        return 0;
    } catch (const std::bad_alloc&) {
        return 1;
    }
}
