#include <iostream>
#include <vector>
#include <assert.h>
#include <algorithm>
#include <unordered_set>

namespace gsy {
    template<unsigned Dim, typename T>
    struct Line;

    template<typename T>
    struct Line<1, T> {
        T pmin, pmax;
    };

    template<typename T>
    T choose_split_number(const std::vector<Line<1, T>>& lines) {
        assert(lines.size() > 0);

        struct _Sort_Term{
            T value;
            unsigned index;

            bool operator<(const _Sort_Term& other) const {
                return value < other.value;
            }
            bool operator==(const _Sort_Term& other) const {
                return value == other.value;
            }
        };

        std::vector<_Sort_Term> points;
        points.reserve(lines.size() * 2);

        for(unsigned i = 0 ; i < lines.size() ; ++i) {
            points.emplace_back(lines[i].pmin, i);
            points.emplace_back(lines[i].pmax, i);
        }

        std::sort(points.begin(), points.end());

        std::unordered_set<unsigned> vis;
        unsigned left_size = 1;
        unsigned right_size = lines.size();
        unsigned opt_delta = right_size - left_size;
        T  opt_split = (points[1].value + points[0].value) / static_cast<T>(2);
        vis.insert(points[0].index);

        for(unsigned i = 2 ; i < points.size() ; ++i) {
            T curr_split = (points[i-1].value + points[i].value) / static_cast<T>(2);
            unsigned curr_index = points[i-1].index;
            if(vis.find(curr_index) != vis.end()) {
                right_size -= 1;
                vis.erase(curr_index);
            } else {
                vis.insert(curr_index);
                left_size += 1;
            }
            unsigned curr_delta = std::abs((long)left_size - (long)right_size);

            if(curr_delta < opt_delta) {
                opt_delta = curr_delta;
                opt_split = curr_split;
                if(opt_delta == 0) {
                    break;
                }
            }
        }

        return opt_split;
    }
}

using namespace gsy;

int main() {
    std::vector<Line<1, float>> lines;
    lines.emplace_back(0.f, 1.f);
    lines.emplace_back(2.f, 3.f);
    lines.emplace_back(4.f, 6.f);
    float opt_split = choose_split_number(lines);
    std::cout << opt_split << std::endl;
    return 0;
}