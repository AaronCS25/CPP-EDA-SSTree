#ifndef POINT_H
#define POINT_H

#include <Eigen/Dense>
#include <stdexcept>
#include <random>
#include <iostream>

constexpr std::size_t DIM = 768;
constexpr float EPSILON = 1e-8f;

class Point {
public:
    // Constructores
    Point() : coordinates_(Eigen::VectorXf::Zero(DIM)) {}
    explicit Point(const Eigen::VectorXf& coordinates);

    // Operadores
    Point  operator+ (const Point& other) const;
    Point& operator+=(const Point& other);
    Point  operator- (const Point& other) const;
    Point& operator-=(const Point& other);
    Point  operator* (float scalar) const;
    Point& operator*=(float scalar);
    Point  operator/ (float scalar) const;
    Point& operator/=(float scalar);

    // Métodos adicionales
    float norm() const { return coordinates_.norm(); }
    float normSquared() const { return coordinates_.squaredNorm(); }
    float distance(const Point& other) const { return (*this - other).norm(); }
    float distanceSquared(const Point& other) const { return (*this - other).normSquared(); }

    // Operadores de acceso
    float  operator[](std::size_t index) const { return coordinates_(index); }
    float& operator[](std::size_t index) { return coordinates_(index); }

    // Puntos aleatorios
    static Point random(float min = 0.0f, float max = 1.0f);

    // Distancia entre dos puntos
    static float distance(const Point& a, const Point& b);

    // Print!
    void print() const;

private:
    Eigen::VectorXf coordinates_;
};

#endif // POINT_H
