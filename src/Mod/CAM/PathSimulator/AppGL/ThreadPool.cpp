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

#include "ThreadPool.h"

#include <algorithm>

#if defined(__linux__)
# include <sys/resource.h>
# include <sys/syscall.h>
# include <unistd.h>
#elif defined(_WIN32)
# ifndef NOMINMAX
#  define NOMINMAX
# endif
# include <windows.h>
#endif

namespace CAMSimulator
{

void ThreadPool::LowerCurrentThreadPriority()
{
#if defined(__linux__)
    // on Linux the nice value applies to the thread
    setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), 10);
#elif defined(_WIN32)
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
}

ThreadPool::ThreadPool(int threads, bool lowPriority)
    : mLowPriority(lowPriority)
{
    if (threads <= 0) {
        threads = std::max(1, (int)std::thread::hardware_concurrency());
    }
    for (int i = 1; i < threads; i++) {
        mThreads.emplace_back([this, i] { WorkerLoop(i); });
    }
}

ThreadPool::~ThreadPool()
{
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mStop = true;
    }
    mStart.notify_all();
    for (auto& thread : mThreads) {
        thread.join();
    }
}

void ThreadPool::Run(const std::function<void(int)>& fn)
{
    if (mThreads.empty()) {
        fn(0);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mTask = &fn;
        mRunning = (int)mThreads.size();
        mGeneration++;
    }
    mStart.notify_all();
    fn(0);
    std::unique_lock<std::mutex> lock(mMutex);
    mDone.wait(lock, [this] { return mRunning == 0; });
    mTask = nullptr;
}

void ThreadPool::WorkerLoop(int worker)
{
    if (mLowPriority) {
        LowerCurrentThreadPriority();
    }
    unsigned generation = 0;
    while (true) {
        const std::function<void(int)>* task;
        {
            std::unique_lock<std::mutex> lock(mMutex);
            mStart.wait(lock, [&] { return mStop || mGeneration != generation; });
            if (mStop) {
                return;
            }
            generation = mGeneration;
            task = mTask;
        }
        (*task)(worker);
        {
            std::lock_guard<std::mutex> lock(mMutex);
            mRunning--;
        }
        mDone.notify_one();
    }
}

}  // namespace CAMSimulator
