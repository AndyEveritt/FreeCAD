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

#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace CAMSimulator
{

// A fixed set of threads that run one function in parallel, once per worker.
class ThreadPool
{
public:
    // threads <= 0 uses one thread per hardware thread. Low priority threads yield to other work,
    // like rendering.
    explicit ThreadPool(int threads = 0, bool lowPriority = false);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    int Size() const
    {
        return (int)mThreads.size() + 1;
    }

    // Run fn(worker) for every worker in [0, Size()) in parallel and wait until all are done.
    // The calling thread runs worker 0.
    void Run(const std::function<void(int)>& fn);

    // Lower the scheduling priority of the calling thread
    static void LowerCurrentThreadPriority();

private:
    void WorkerLoop(int worker);

    bool mLowPriority = false;

    std::vector<std::thread> mThreads;
    std::mutex mMutex;
    std::condition_variable mStart;
    std::condition_variable mDone;
    const std::function<void(int)>* mTask = nullptr;
    unsigned mGeneration = 0;
    int mRunning = 0;
    bool mStop = false;
};

}  // namespace CAMSimulator
