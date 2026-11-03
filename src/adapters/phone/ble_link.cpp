#include "adapters/phone/ble_link.h"

#include <algorithm>

namespace dlidar::phone
{
    // The server's view of the phone. Close() only marks it: OnClosed comes
    // on the next wake, so a Send that closes never pulls it from under the
    // server (as for the socket).
    class BleLink::Conn : public Connection
    {
    public:
        explicit Conn(BleLink &link) : link_(link) {}

        void Send(const std::string &bytes) override
        {
            if (closed)
                return;
            if (link_.queued_ + bytes.size() > link_.options_.cap_bytes)
            {
                Close(); // the phone does not take its notifications
                return;
            }
            link_.Queue(bytes);
        }

        size_t Pending() const override { return link_.queued_; }

        void Close() override { link_.RequestClose(); }

        bool closed = false;

    private:
        BleLink &link_;
    };

    BleLink::BleLink(ConnectionHandler &handler, BlePeripheral &peripheral, ServerClock &clock, Options options)
        : handler_(handler), peripheral_(peripheral), clock_(clock), options_(options)
    {
    }

    BleLink::~BleLink() { CloseNow(); }

    void BleLink::BluezUp()
    {
        up_ = true;
        UpdateAdvertising();
    }

    void BleLink::BluezDown()
    {
        CloseNow();
        up_ = false;
        advertising_ = false; // gone with bluetoothd, nothing to unregister
        notifying_ = false;
        connected_.clear();
        phone_.clear();
        mtu_ = kMinMtu;
    }

    void BleLink::DeviceConnected(const std::string &device)
    {
        if (!connected_.insert(device).second)
            return;
        if (phone_.empty())
        {
            phone_ = device;
            mtu_ = kMinMtu;
        }
        else if (device != phone_)
            peripheral_.Disconnect(device); // one phone at a time
        UpdateAdvertising();
        TryOpen();
    }

    void BleLink::DeviceDisconnected(const std::string &device)
    {
        if (!connected_.erase(device))
            return;
        if (device == phone_)
        {
            CloseNow();
            phone_.clear();
            mtu_ = kMinMtu;
            // A second device we disconnected is not promoted; ask again in
            // case it is still there.
            for (const std::string &other : connected_)
                peripheral_.Disconnect(other);
        }
        UpdateAdvertising();
    }

    void BleLink::NotifyStarted()
    {
        notifying_ = true;
        TryOpen();
    }

    void BleLink::NotifyStopped()
    {
        notifying_ = false;
        CloseNow();
    }

    WriteResult BleLink::Write(const std::string &device, std::string_view bytes, uint16_t mtu, uint16_t offset)
    {
        if (phone_.empty())
            DeviceConnected(device); // the write came before its Connected signal
        if (device != phone_)
        {
            connected_.insert(device);
            peripheral_.Disconnect(device);
            UpdateAdvertising();
            return WriteResult::NotPermitted;
        }
        if (offset != 0)
            return WriteResult::InvalidOffset; // no long writes: pieces are at most MTU-3
        if (mtu >= kMinMtu)
            mtu_ = mtu;
        if (!conn_ || conn_->closed)
            return WriteResult::NotPermitted; // its answer could not be delivered
        handler_.OnBytes(*conn_, bytes);
        return WriteResult::Ok;
    }

    void BleLink::OnWake()
    {
        if (close_pending_)
            CloseNow();
        Drain();
    }

    void BleLink::UpdateAdvertising()
    {
        bool want = up_ && connected_.empty();
        if (want != advertising_)
        {
            advertising_ = want;
            peripheral_.SetAdvertising(want);
        }
    }

    void BleLink::TryOpen()
    {
        if (conn_ || !notifying_ || phone_.empty())
            return;
        conn_ = std::make_unique<Conn>(*this);
        tokens_ = Burst(); // the first message goes at once
        refilled_ = clock_.Now();
        handler_.OnOpen(*conn_);
    }

    void BleLink::CloseNow()
    {
        close_pending_ = false;
        queue_.clear();
        queued_ = 0;
        if (!conn_)
            return;
        std::unique_ptr<Conn> c = std::move(conn_);
        c->closed = true;
        handler_.OnClosed(*c);
    }

    void BleLink::RequestClose()
    {
        if (!conn_ || conn_->closed)
            return;
        conn_->closed = true;
        queue_.clear();
        queued_ = 0;
        if (!phone_.empty())
            peripheral_.Disconnect(phone_);
        close_pending_ = true;
        clock_.WakeAt(clock_.Now());
    }

    double BleLink::Burst() const
    {
        return options_.rate_bytes_per_s * std::chrono::duration<double>(options_.tick).count();
    }

    void BleLink::Queue(const std::string &message)
    {
        size_t piece = std::max<size_t>(mtu_ - 3, 1);
        for (size_t i = 0; i < message.size(); i += piece)
            queue_.push_back(message.substr(i, piece)); // each message starts a fresh piece
        queued_ += message.size();
        clock_.WakeAt(clock_.Now());
    }

    void BleLink::Drain()
    {
        if (!conn_ || conn_->closed || queue_.empty())
            return;
        ServerClock::TimePoint now = clock_.Now();
        double burst = Burst();
        double elapsed = std::chrono::duration<double>(now - refilled_).count();
        tokens_ = std::min(burst, tokens_ + options_.rate_bytes_per_s * std::max(elapsed, 0.0));
        refilled_ = now;
        // A piece may go when the bucket is not empty; it may run into debt,
        // which the next refills pay off, so the average stays at the rate.
        while (!queue_.empty() && tokens_ > 0)
        {
            const std::string &p = queue_.front();
            peripheral_.Notify(p);
            tokens_ -= (double)p.size();
            queued_ -= p.size();
            queue_.pop_front();
        }
        if (!queue_.empty())
        {
            // Wake when the debt is paid, at the earliest one tick on.
            double wait = tokens_ < 0 ? -tokens_ / options_.rate_bytes_per_s : 0.0;
            auto delay = std::max(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double>(wait)),
                                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(options_.tick));
            clock_.WakeAt(now + delay);
            return;
        }
        handler_.OnWritable(*conn_); // the server's held events may follow
    }
}
