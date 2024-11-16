#ifndef DATA_H
#define DATA_H

#include <string>
#include "Point.h"

class Data {
private:
    Point embedding;
    std::string path;

public:
    Data(const Point& embedding, const std::string& imagePath)
        : embedding(embedding), path(imagePath) {}

    // Getters
    const Point& getEmbedding() const { return embedding; }
    const std::string& getPath() const { return path; }

    // Operators
    bool operator==(const Data& other) const {
        return path == other.path;
    }
};

#endif // DATA_H
