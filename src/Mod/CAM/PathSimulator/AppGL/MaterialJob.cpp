// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2026 The FreeCAD project association AISBL              *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#include "MaterialJob.h"

#include <cstdio>
#include <cstring>
#include <memory>

namespace CAMSimulator::Material
{

namespace
{

constexpr char Magic[8] = "CAMJOB2";

using File = std::unique_ptr<FILE, int (*)(FILE*)>;

}  // namespace

bool SaveJob(const std::string& path, const Job& job)
{
    File f(std::fopen(path.c_str(), "wb"), &std::fclose);
    if (!f) {
        return false;
    }
    bool ok = true;
    auto write = [&](const void* p, size_t n) {
        ok = ok && std::fwrite(p, 1, n, f.get()) == n;
    };
    auto u32 = [&](uint32_t v) {
        write(&v, 4);
    };
    auto i32 = [&](int32_t v) {
        write(&v, 4);
    };
    write(Magic, sizeof(Magic));
    write(&job.quality, 4);
    u32((uint32_t)job.profiles.size());
    for (const auto& p : job.profiles) {
        u32((uint32_t)p.size());
        write(p.data(), 4 * p.size());
    }
    u32((uint32_t)job.segments.size());
    for (const Segment& s : job.segments) {
        i32(s.tool);
        i32((s.arc ? 1 : 0) | (s.rapid ? 2 : 0));
        write(s.start, 24);
        write(s.end, 24);
        write(s.center, 16);
        write(&s.radius, 8);
        write(&s.startAngle, 8);
        write(&s.sweep, 8);
        write(s.axis0, 24);
        write(s.axis1, 24);
        i32(s.steps);
        i32(s.sweepSteps);
        i32(s.operation);
    }
    u32((uint32_t)job.stockXyz.size());
    write(job.stockXyz.data(), 4 * job.stockXyz.size());
    u32((uint32_t)job.stockTriangles.size());
    write(job.stockTriangles.data(), 4 * job.stockTriangles.size());
    return ok;
}

bool LoadJob(const std::string& path, Job& job)
{
    File f(std::fopen(path.c_str(), "rb"), &std::fclose);
    if (!f) {
        return false;
    }
    bool ok = true;
    auto read = [&](void* p, size_t n) {
        ok = ok && std::fread(p, 1, n, f.get()) == n;
    };
    auto u32 = [&]() {
        uint32_t v = 0;
        read(&v, 4);
        return v;
    };
    auto i32 = [&]() {
        int32_t v = 0;
        read(&v, 4);
        return v;
    };
    // counts are checked against what is left of the file before allocating
    std::fseek(f.get(), 0, SEEK_END);
    const long size = std::ftell(f.get());
    std::fseek(f.get(), 0, SEEK_SET);
    auto count = [&](size_t itemBytes) {
        const uint32_t n = u32();
        const long left = size - std::ftell(f.get());
        if (!ok || (double)n * itemBytes > (double)left) {
            ok = false;
            return (size_t)0;
        }
        return (size_t)n;
    };

    char magic[sizeof(Magic)] = {};
    read(magic, sizeof(magic));
    if (!ok || std::memcmp(magic, Magic, sizeof(Magic)) != 0) {
        return false;
    }
    Job j;
    read(&j.quality, 4);
    j.profiles.resize(count(4));
    for (auto& p : j.profiles) {
        p.resize(count(4));
        read(p.data(), 4 * p.size());
    }
    j.segments.resize(count(156));
    for (Segment& s : j.segments) {
        s.tool = i32();
        const int flags = i32();
        s.arc = (flags & 1) != 0;
        s.rapid = (flags & 2) != 0;
        read(s.start, 24);
        read(s.end, 24);
        read(s.center, 16);
        read(&s.radius, 8);
        read(&s.startAngle, 8);
        read(&s.sweep, 8);
        read(s.axis0, 24);
        read(s.axis1, 24);
        s.steps = i32();
        s.sweepSteps = i32();
        s.operation = i32();
    }
    j.stockXyz.resize(count(4));
    read(j.stockXyz.data(), 4 * j.stockXyz.size());
    j.stockTriangles.resize(count(4));
    read(j.stockTriangles.data(), 4 * j.stockTriangles.size());
    if (!ok) {
        return false;
    }
    for (const Segment& s : j.segments) {
        if (s.tool < 0 || s.tool >= (int)j.profiles.size()) {
            return false;
        }
    }
    for (uint32_t v : j.stockTriangles) {
        if (v >= j.stockXyz.size() / 3) {
            return false;
        }
    }
    job = std::move(j);
    return true;
}

}  // namespace CAMSimulator::Material
