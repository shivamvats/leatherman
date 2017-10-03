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

#include <leatherman/bresenham.h>

#include <cmath>

namespace leatherman {

void InitBresenhamState(
    BresenhamState3D& params,
    int p1x, int p1y, int p1z,
    int p2x, int p2y, int p2z)
{
    params.x1 = p1x;
    params.y1 = p1y;
    params.z1 = p1z;
    params.x2 = p2x;
    params.y2 = p2y;
    params.z2 = p2z;

    params.index_x = params.x1;
    params.index_y = params.y1;
    params.index_z = params.z1;

    params.dx = std::abs(p2x - p1x);
    params.dy = std::abs(p2y - p1y);
    params.dz = std::abs(p2z - p1z);
    params.dx2 = params.dx << 1;
    params.dy2 = params.dy << 1;
    params.dz2 = params.dz << 1;

    // get direction of slope
    if (p2x - p1x < 0) {
        params.inc_x = -1;
    } else {
        params.inc_x = 1;
    }

    if (p2y - p1y < 0) {
        params.inc_y = -1;
    } else {
        params.inc_y = 1;
    }

    if (p2z - p1z < 0) {
        params.inc_z = -1;
    } else {
        params.inc_z = 1;
    }

    // choose which axis to use as the index
    if (params.dx >= params.dy && params.dx >= params.dz) {
        params.use_index = 0;
        params.err1 = params.dy2 - params.dx;
        params.err2 = params.dz2 - params.dx;
    } else if (params.dy >= params.dx && params.dy >= params.dz) {
        params.use_index = 1;
        params.err1 = params.dx2 - params.dy;
        params.err2 = params.dz2 - params.dy;
    } else {
        params.use_index = 2;
        params.err1 = params.dy2 - params.dz;
        params.err2 = params.dx2 - params.dz;
    }
}

void GetCurrentPoint(const BresenhamState3D& params, int* xyz)
{
    xyz[0] = params.index_x;
    xyz[1] = params.index_y;
    xyz[2] = params.index_z;
}

bool AdvanceBresenham(BresenhamState3D& params)
{
    // check to see if at end of line
    if (params.index_x == params.x2 &&
        params.index_y == params.y2 &&
        params.index_z == params.z2)
    {
        return false;
    }

    if (params.use_index == 0) {
        if (params.err1 > 0) {
            params.index_y += params.inc_y;
            params.err1 -= params.dx2;
        }
        if (params.err2 > 0) {
            params.index_z += params.inc_z;
            params.err2 -= params.dx2;
        }
        params.err1 += params.dy2;
        params.err2 += params.dz2;
        params.index_x += params.inc_x;
    } else if (params.use_index == 1) {
        if (params.err1 > 0) {
            params.index_x += params.inc_x;
            params.err1 -= params.dy2;
        }
        if (params.err2 > 0) {
            params.index_z += params.inc_z;
            params.err2 -= params.dy2;
        }
        params.err1 += params.dx2;
        params.err2 += params.dz2;
        params.index_y += params.inc_y;
    } else {
        if (params.err1 > 0) {
            params.index_y += params.inc_y;
            params.err1 -= params.dz2;
        }
        if (params.err2 > 0) {
            params.index_x += params.inc_x;
            params.err2 -= params.dz2;
        }
        params.err1 += params.dy2;
        params.err2 += params.dx2;
        params.index_z += params.inc_z;
    }
    return true;
}

} // namespace leatherman
