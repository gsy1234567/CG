#include <algorithm>
#include <iostream>
#include <cstdint>
#include <array>
#include <vector>
#include <cstddef>
#include <unordered_set>
#include <queue>

template<std::size_t LeafCapa>
struct ResultNode {
    std::vector<std::size_t> indices;
};

namespace std {
    template<typename Container,size_t Dim, size_t OrderStart, size_t OrderCurr>
    struct orderLess {
        constexpr bool operator()(const Container&l, const Container& r) const {
            if constexpr ((OrderCurr + 1) % Dim == OrderStart) {
                return l[OrderCurr]<r[OrderCurr];
            } else {
                return l[OrderCurr] != r[OrderCurr] ? l[OrderCurr] < r[OrderCurr] : orderLess<Container, Dim, OrderStart, (OrderCurr + 1) % Dim>()(l, r);
            }
        }
    };
}

template<std::size_t CurrAxis, std::size_t LastAxis>
inline void sort_data(const auto& v, auto& ordered_indices) {
    if constexpr (CurrAxis < LastAxis) {
        auto tmp_v = v;
        std::sort(tmp_v.begin(), tmp_v.end(), [&tmp_v](const std::tuple<Point, std::size_t>&l, const std::tuple<Point, std::size_t>& r)->bool {
            return std::orderLess<Point, Dim, CurrAxis, CurrAxis>()(std::get<0>(l), std::get<0>(r));
        });
        for(std::size_t i = 0 ; i < tmp_v.size() ; ++i) {
            ordered_indices[CurrAxis].emplace_back(std::get<1>(tmp_v[i]));
        }
        sort_data<CurrAxis + 1, LastAxis>(v, ordered_indices);
    }
}


template<std::size_t Dim, std::size_t LeafCapa, typename Point>
std::vector<ResultNode<LeafCapa>> buildKDTree(const std::vector<Point>& points) {
    using element_type = decltype(points[0]);

    std::array<std::vector<std::size_t>, Dim> ordered_indices;
    std::vector<std::tuple<Point, std::size_t>> v;

    for(std::size_t i = 0 ; i < points.size() ; ++i) {
        v.emplace_back(points[i], i);
    }

    sort_data<0, Dim>(v, ordered_indices);

    struct BuildTask {
        std::size_t start, end;
        std::size_t dim;
    };

    std::queue<BuildTask> unFinished;
    unFinished.emplace(0, points.size(), 0);
    std::vector<ResultNode<LeafCapa>> ans;

    while(!unFinished.empty()) {
        BuildTask currTask = unFinished.front();
        unFinished.pop();
        std::size_t currLen = currTask.end - currTask.start;
        if(currLen < LeafCapa) {
            std::vector<std::size_t> ans_indices {ordered_indices[currTask.dim].begin() + currTask.start, ordered_indices[currTask.dim].begin() + currTask.end};
            ans.emplace_back(ans_indices);
        } else {
            std::size_t currMid = (currTask.start + currTask.end) / 2;

            unFinished.emplace(currTask.start, currMid, (currTask.dim + 1) % Dim);
            unFinished.emplace(currMid, currTask.end, (currTask.dim + 1) % Dim);

            std::unordered_set<std::size_t> min {
                ordered_indices[currTask.dim].begin() + currTask.start, 
                ordered_indices[currTask.dim].begin() + currMid
            };

            for(std::size_t j = 0 ; j < Dim - 1 ; ++j) {
                std::vector<std::size_t> tmp (currLen, 0);
                std::size_t minStart = 0;
                std::size_t maxStart = currMid;
                std::size_t currDim = (currTask.dim + j) % Dim;
                for(std::size_t i = currTask.start ; i != currTask.end ; ++i) {
                    std::size_t currIdx = ordered_indices[currDim][i];
                    if(min.find(currIdx) != min.end()) {
                        tmp[minStart++] = currIdx;
                    }
                    else {
                        tmp[maxStart++] = currIdx;
                    }
                    std::copy_n(tmp.begin(), currLen, ordered_indices[currDim].begin() + currTask.start);
                }
            }
        }
    }

    return ans;
}


int main() {
    std::vector<std::array<int, 3>> input{
        {2, 3, 3}, 
        {5, 4, 2}, 
        {9, 6, 7},
        {4, 7, 9},
        {8, 1, 5}, 
        {7, 2, 6}, 
        {9, 4, 1}, 
        {8, 4, 2}, 
        {9, 7, 8}, 
        {6, 3, 1}, 
        {3, 4, 5}, 
        {1, 6, 8}, 
        {9, 5, 3}, 
        {2, 1, 3}, 
        {8, 7, 6}
    };
    auto output = buildKDTree<3, 3>(input);
    return 0;
}