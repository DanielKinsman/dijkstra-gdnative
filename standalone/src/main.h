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

CellCoord min_distance(unordered_map<CellCoord, int> const& distances, unordered_set<CellCoord> const& vertex_set);
void dijkstra_test();
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
