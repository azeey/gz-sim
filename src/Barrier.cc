/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <chrono>
#include "Barrier.hh"
#include "tracy/Tracy.hpp"
#ifdef GZ_PROFILE
#undef GZ_PROFILE
#endif
#ifdef GZ_PROFILE_THREAD_NAME
#undef GZ_PROFILE_THREAD_NAME
#endif

#define GZ_PROFILE ZoneScopedN
#define GZ_PROFILE_THREAD_NAME tracy::SetThreadName

class gz::sim::BarrierPrivate
{
  /// \brief Mutex for syncronization
  public: std::mutex mutex;

  /// \brief Condition Variable for signaling
  public: std::condition_variable cv;

  /// \brief Flag to indicate if the barrier was cancelled
  public: std::atomic<bool> cancelled { false };

  /// \brief Number of participating threads
  public: unsigned int threadCount;

  /// \brief Current remaining thread count (decrements from threadCount)
  public: unsigned int count;

  /// \brief Barrier generation, incremented when all threads report
  public: unsigned int generation{0};
};

using namespace gz::sim;

//////////////////////////////////////////////////
Barrier::Barrier(unsigned int _threadCount)
  : dataPtr(std::make_unique<BarrierPrivate>())
{
  this->dataPtr->threadCount = _threadCount;
  this->dataPtr->count = _threadCount;
}

//////////////////////////////////////////////////
Barrier::~Barrier() = default;

//////////////////////////////////////////////////
Barrier::ExitStatus Barrier::Wait()
{
  if (this->dataPtr->cancelled)
  {
    return Barrier::ExitStatus::CANCELLED;
  }

  std::unique_lock<std::mutex> lock(this->dataPtr->mutex);
  unsigned int gen = this->dataPtr->generation;

  if (--this->dataPtr->count == 0)
  {
    GZ_PROFILE("Barrier notify");
    // All threads have reached the wait, so reset the barrier.
    this->dataPtr->generation++;
    this->dataPtr->count = this->dataPtr->threadCount;
    this->dataPtr->cv.notify_all();
    return Barrier::ExitStatus::DONE_LAST;
  }

  while (gen == this->dataPtr->generation && !this->dataPtr->cancelled)
  {
    GZ_PROFILE("Barrier wait_for");
    using namespace std::chrono_literals;
    // All threads haven't reached, so wait until generation is reached
    // or a cancel occurs
    this->dataPtr->cv.wait_for(lock, 500us);
  }

  if (this->dataPtr->cancelled)
  {
    return Barrier::ExitStatus::CANCELLED;
  }
  else
  {
    return Barrier::ExitStatus::DONE;
  }
}

//////////////////////////////////////////////////
void Barrier::Cancel()
{
  std::unique_lock<std::mutex> lock(this->dataPtr->mutex);
  // This forces pending threads to release
  this->dataPtr->generation++;
  this->dataPtr->cancelled = true;
  this->dataPtr->cv.notify_all();
}
