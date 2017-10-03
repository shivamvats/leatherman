/*
 * Copyright (c) 2010, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LEATHERMAN_BRESENHAM_H
#define LEATHERMAN_BRESENHAM_H

namespace leatherman {

/// Stores the state of the 3D Bresenham line-filling algorithm.
struct BresenhamState3D
{
    int x1, y1, z1;
    int x2, y2, z2;
    int index_x, index_y, index_z;
    int use_index;
    int inc_x, inc_y, inc_z;
    int dx, dy, dz;
    int dx2, dy2, dz2;
    int err1, err2;
};

void InitBresenhamState(
    BresenhamState3D& params,
    int p1x, int p1y, int p1z,
    int p2x, int p2y, int p2z);

/// Advance to the next cell to be filled. Return false if at the end of the
/// line.
bool AdvanceBresenham(BresenhamState3D& params);

/// Get the current filled cell.
void GetCurrentPoint(const BresenhamState3D& params, int* xyz);

/// Return all the cells filled by rasterizing a 3D line.
/// \param a An array containing the start cell coordinates
/// \param b An array containing the goal cell coordinates
/// \param out An output iterator to store the filled cell coordinates
template <class OutputIt>
void GetBresenhamLine(
    const int* a,
    const int* b,
    OutputIt out)
{
    BresenhamState3D params;
    InitBresenhamState(params, a[0], a[1], a[2], b[0], b[1], b[2]);
    do {
        int xyz[3];
        GetCurrentPoint(params, xyz);
        *out++ = xyz[0];
        *out++ = xyz[1];
        *out++ = xyz[2];
    } while (AdvanceBresenham(params));
}

} // namespace leatherman

#endif
