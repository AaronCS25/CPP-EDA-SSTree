#include "Point.h"

// Constructor
Point::Point(const Eigen::VectorXf& coordinates) : coordinates_(coordinates) {
    if (coordinates.size() != DIM) {
        throw std::invalid_argument("Dimensionalidad incorrecta :c");
    }
}

// Operadores
Point Point::operator+(const Point& other) const {
    return Point(coordinates_ + other.coordinates_);
}

Point& Point::operator+=(const Point& other) {
    coordinates_ += other.coordinates_;
    return *this;
}

Point Point::operator-(const Point& other) const {
    return Point(coordinates_ - other.coordinates_);
}

Point& Point::operator-=(const Point& other) {
    coordinates_ -= other.coordinates_;
    return *this;
}

Point Point::operator*(float scalar) const {
    return Point(coordinates_ * scalar);
}

Point& Point::operator*=(float scalar) {
    coordinates_ *= scalar;
    return *this;
}

Point Point::operator/(float scalar) const {
    if (std::abs(scalar) < EPSILON) {
        throw std::invalid_argument("División por cero (o casi cero).");
    }
    return Point(coordinates_ / scalar);
}

Point& Point::operator/=(float scalar) {
    if (std::abs(scalar) < EPSILON) {
        throw std::invalid_argument("División por cero (o casi cero).");
    }
    coordinates_ /= scalar;
    return *this;
}

// Punto aleatorio
Point Point::random(float min, float max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);

    Eigen::VectorXf coordinates(DIM);
    for (std::size_t i = 0; i < DIM; ++i) {
        coordinates[i] = dis(gen);
    }

    return Point(coordinates);
}

// Distancia entre dos puntos
float Point::distance(const Point& a, const Point& b) {
    return (a - b).norm();
}

// Imprimir el punto
void Point::print() const {
    std::cout << "Point(";
    for (std::size_t i = 0; i < DIM; ++i) {
        std::cout << coordinates_[i];
        if (i < DIM - 1) std::cout << ", ";
    }
    std::cout << ")" << std::endl;
}
