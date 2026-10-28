// Copyright (c) 2025 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef MULTISENSORCALIBRATION_LATESTMESSAGESYNC_H
#define MULTISENSORCALIBRATION_LATESTMESSAGESYNC_H

// Std
#include <functional>
#include <mutex>

// ROS
#include <message_filters/connection.hpp>

namespace multisensor_calibration
{

/**
 * @ingroup calibration
 * @brief Message pairing without time synchronization: each message of the source sensor is
 * passed to the callback together with the latest message received from the reference sensor.
 *
 * This is used if the clocks of the sensors are not synchronized, i.e. if neither exact nor
 * approximate time synchronization finds matching message pairs. It requires the target and the
 * sensors to be static while a target is captured.
 */
template <class SrcMsgT, class RefMsgT>
class LatestMessageSync
{
  public:
    using Callback = std::function<void(const typename SrcMsgT::ConstSharedPtr&,
                                        const typename RefMsgT::ConstSharedPtr&)>;

    /**
     * @brief Initialization constructor, registering at the given message filters.
     *
     * @param[in] ioSrcFilter Message filter (e.g. subscriber) of the source sensor messages.
     * @param[in] ioRefFilter Message filter (e.g. subscriber) of the reference sensor messages.
     * @param[in] iCallback Callback called for each source message once a reference message
     * has been received.
     */
    template <class SrcFilterT, class RefFilterT>
    LatestMessageSync(SrcFilterT& ioSrcFilter, RefFilterT& ioRefFilter, Callback iCallback) :
      callback_(std::move(iCallback))
    {
        refConnection_ = ioRefFilter.registerCallback(
          [this](const typename RefMsgT::ConstSharedPtr& ipRefMsg)
          {
              std::lock_guard<std::mutex> guard(mutex_);
              pLatestRefMsg_ = ipRefMsg;
          });
        srcConnection_ = ioSrcFilter.registerCallback(
          [this](const typename SrcMsgT::ConstSharedPtr& ipSrcMsg)
          {
              typename RefMsgT::ConstSharedPtr pRefMsg;
              {
                  std::lock_guard<std::mutex> guard(mutex_);
                  pRefMsg = pLatestRefMsg_;
              }
              if (pRefMsg)
                  callback_(ipSrcMsg, pRefMsg);
          });
    }

    /**
     * @brief Destructor, disconnecting from the message filters.
     */
    ~LatestMessageSync()
    {
        srcConnection_.disconnect();
        refConnection_.disconnect();
    }

    LatestMessageSync(const LatestMessageSync&)            = delete;
    LatestMessageSync& operator=(const LatestMessageSync&) = delete;

  private:
    /// Callback called with each source message and the latest reference message.
    Callback callback_;

    /// Mutex guarding the latest reference message.
    std::mutex mutex_;

    /// Latest message received from the reference sensor.
    typename RefMsgT::ConstSharedPtr pLatestRefMsg_;

    /// Connection to the source message filter.
    message_filters::Connection srcConnection_;

    /// Connection to the reference message filter.
    message_filters::Connection refConnection_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_LATESTMESSAGESYNC_H
