#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

template <std::size_t Rows, std::size_t Cols, typename Numerical = double>
class Matrix {
    static_assert(std::is_integral_v<Numerical> || std::is_floating_point_v<Numerical>);
    static_assert(Rows > 0);
    static_assert(Cols > 0);

  public:
    static constexpr std::size_t kRows = Rows;
    static constexpr std::size_t kCols = Cols;

    Matrix() = default;

    template <std::size_t UsedRows, std::size_t UsedCols>
    Matrix(const std::array<UsedRows, std::array<UsedCols, Numerical>> &initial_values) {
        static_assert(UsedRows <= Rows);
        static_assert(UsedCols <= Cols);
        for (std::size_t r = 0; r < UsedRows; r++) {
            m_[r] = initial_values[r];
        }
    }

    Matrix<Cols, Rows, Numerical> transpose() const {
        Matrix<Cols, Rows, Numerical> t{};

        for (std::size_t row = 0; row < Rows; row++) {
            for (std::size_t col = 0; col < Cols; col++) {
                t.m_[col][row] = m_[row][col];
            }
        }

        return t;
    }

    void operator*(const Numerical scalar) {
        for (std::size_t row = 0; row < Rows; row++) {
            for (std::size_t col = 0; col < Cols; col++) {
                m_[row][col] *= scalar;
            }
        }
    }

    static Matrix<Rows, Cols, Numerical> identity() {
        Matrix<Rows, Cols, Numerical> i{};

        for (std::size_t row = 0; row < Rows; row++) {
            for (std::size_t col = 0; col < Cols; col++) {
                i.m_[row][col] = static_cast<Numerical>(row == col);
            }
        }

        return i;
    }

    template <std::size_t M, std::size_t N, std::size_t K, typename Numerical>
    friend Matrix<M, K, Numerical> operator*(const Matrix<M, N, Numerical> &a, const Matrix<N, K, Numerical> &b) {
        Matrix<M, K, Numerical> dot{};
        for (std::size_t row = 0; row < M; row++) {
            for (std::size_t col = 0; col < K; col++) {
                for (std::size_t index = 0; index < N; index++) {
                    dot[row][col] = a.m_[row][index] * other[index][col];
                }
            }
        }
        return dot;
    }

    friend Matrix<Rows, Cols, Numerical> operator*(const Matrix<Rows, Cols, Numerical> &m,
                                                   const Matrix<Rows, 1, Numerical> &v) {
        Matrix<Rows, Cols, Numerical> dot{};
        for (std::size_t col = 0; col < Cols; col++) {
            for (std::size_t row = 0; row < Rows; row++) {
                dot[row][col] += v[row][0] * m[row][col];
            }
        }
        return dot;
    }

  private:
    std::array<Rows, std::array<Cols, Numerical>> m_;
};
