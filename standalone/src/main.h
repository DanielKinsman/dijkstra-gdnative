#ifndef MAIN_H
#define MAIN_H

#include <unordered_map>
#include <unordered_set>

using namespace std;

struct CellCoord {
    int row;
    int column;

    bool operator==(const CellCoord& other) const {
        return row == other.row && column == other.column;
    }

    CellCoord() {
        this->row = 0;
        this->column = 0;
    }

    CellCoord(int row, int column) {
        this->row = row;
        this->column = column;
    }
};

typedef pair<CellCoord, int> CellDistance;
bool compare_distance(CellDistance left, CellDistance right);

struct CompareDistance {
    bool operator()(const CellDistance& left, const CellDistance& right) const {
        return left.second > right.second;  // > because in the priority queue, we want top() to be the smallest
    }
};

pair<unordered_map<CellCoord, int>, unordered_map<CellCoord, CellCoord>> dijkstra_test(int rows, int columns, CellCoord source);
int main();

namespace std {
    template<> struct hash<CellCoord> {
        size_t operator()(CellCoord const& coord) const noexcept {
        size_t row_hash = hash<int>{}(coord.row);
        size_t column_hash = hash<int>{}(coord.column);
        return row_hash ^ (column_hash << 1); // TODO better hash combine function, e.g. boost hash_combine       
        }
    };
}

#endif
