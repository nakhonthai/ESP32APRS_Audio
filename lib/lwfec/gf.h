/*
This file is part of LwFEC.

LwFEC is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

LwFEC is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with LwFEC.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef GF_H_
#define GF_H_

#include <stdint.h>

extern const uint8_t GfExp[255];
extern const uint8_t GfLog[256];

/**
 * @brief Add in Galois field
 * @param x Term 1
 * @param y Term 2
 * @return Sum
 */
static inline uint8_t GfAdd(uint8_t x, uint8_t y)
{
    return x ^ y; //addition is done by XOR in GF(2^n)
}

/**
 * @brief Subtract in Galois field
 * @param x Minuend
 * @param y Subtrahend
 * @return Difference
 */
static inline uint8_t GfSub(uint8_t x, uint8_t y)
{
    return x ^ y; //subtraction is done by XOR in GF(2^n) - same as addition
}

/**
 * @brief Multiply in Galois field
 * @param x Multiplicand
 * @param y Multiplier
 * @return Multiplication result
 */
static inline uint8_t GfMul(uint8_t x, uint8_t y)
{
    if((x == 0) || (y == 0)) //trivial multiplication by 0
        return 0;
    //fast multiplication using lookup tables
    //we know that log(x)+log(y)=log(x*y), and b^log(a)=a, when b is the logarithm base
    //so b^(log(x)+log(y))=b^log(x*y)=x*y, where b is the logarithm base
    return GfExp[(GfLog[x] + GfLog[y]) % 255];
}

/**
 * @brief Divide in Galois field
 * @param dividend Dividend
 * @param divisor Divisor
 * @return Division result. 0 is returned when dividing by 0.
 */
static inline uint8_t GfDiv(uint8_t dividend, uint8_t divisor)
{
    if(divisor == 0) 
        return 0; //illegal division by 0, but for now just return 0
    if(dividend == 0)
        return 0; //trivial division of 0
    //similarly to multiplication, x/y=b^(log(x)-log(y)), where b is the logarithm base
    return GfExp[(255 + GfLog[dividend] - GfLog[divisor]) % 255];
}

/**
 * @brief Exponentiate in Galois field
 * @param x Base
 * @param exponent Exponent
 * @return Result
 */
static inline uint8_t GfPow(uint8_t x, uint8_t exponent)
{
    //since a*log(x)=log(x^a) and b^log(x)=x, b^(a*log(x))=b^(log(x^a))=x^a, where b is the logarithm base
    return GfExp[(exponent * GfLog[x]) % 255];
}

/**
 * @brief Calculate 2^x in Galois field
 * @param exponent Exponent (x)
 * @return Result
 */
static inline uint8_t GfPow2(uint8_t exponent)
{
    return GfExp[exponent];
}

/**
 * @brief Invert in Galois field
 * @param x Number to calculate the inverse of
 * @return 1/x
 */
static inline uint8_t GfInv(uint8_t x)
{
    return GfExp[255 - GfLog[x]];
}

/**
 * @brief Multiply polynomial by a scalar
 * @param *p Input polynomial
 * @param o Length of polynomial buffer (degree of a poly + 1)
 * @param s Scalar multiplicand
 * @param out Ouput polynomial. Has the same degree as input polynomial
 */
void GfPolyScale(uint8_t *p, uint8_t o, uint8_t s, uint8_t *out);

/**
 * @brief Add two polynomials
 * @param *p1 1st polynomial
 * @param o1 1st polynomial buffer length (degree of a poly + 1)
 * @param *p2 2nd polynomial
 * @param o2 2nd polynomial buffer length (degree of a poly + 1)
 * @param *out Output polynomial buffer
 * @warning This function uses higher degree polynomial buffer as output buffer
 * @return Output polynomial length
 */
uint8_t GfPolyAdd(uint8_t *p1, uint8_t o1, uint8_t *p2, uint8_t o2, uint8_t *out);

/**
 * @brief Multiply two polynomials
 * @param *p1 1st polynomial
 * @param o1 1st polynomial buffer length (degree of a poly + 1)
 * @param *p2 2nd polynomial
 * @param o2 2nd polynomial buffer length (degree of a poly + 1)
 * @param out Output polynomial. Has a length of o1 + o2 - 1
 * @warning Output buffer must be separate from input buffers
 */
void GfPolyMul(uint8_t *p1, uint8_t o1, uint8_t *p2, uint8_t o2, uint8_t *out);

/**
 * @brief Evaluate the polynomial at given x
 * @param *p Polynomial
 * @param o Polynomial buffer length (degree of a poly + 1)
 * @param x Value to evaluate the polynomial at
 * @return Evaluated value
 */
uint8_t GfPolyEval(uint8_t *p, uint8_t o, uint8_t x);

/**
 * @brief Divide two polynomials in Galois field
 * @param *p1 Divident polynomial
 * @param o1 Divident polynomial buffer length
 * @param *p2 Divisor polynomial
 * @param o2 Divisor polynomial buffer length
 * @param *out Output polynomial quotient and remainder. Size o1 + o2 - 1. Must be preallocated.
 * @warning This function works on polynomials orderder highest-degree-term-first
 * @return Pointer to the first element of the remainder
 */
uint8_t *GfPolyDiv(uint8_t *p1, uint8_t o1, uint8_t *p2, uint8_t o2, uint8_t *out);

/**
 * @brief Reverse the order of elements in a polynomial (in-place)
 * @param *p Polynomial input/output
 * @param o Polynomial buffer length (order + 1)
 */
void GfPolyInv(uint8_t *p, uint8_t o);

#endif