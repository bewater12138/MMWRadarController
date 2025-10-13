#include "gui.h"
#include <ImGui/imgui.h>
#include "code_convert.h"
#include "implot/implot.h"
#include <serial/serial.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <ranges>
#include <cmath>
#include <thread>
#include <memory>
#include <queue>
#include <mutex>
#include <optional>
#include <functional>
#include <concepts>
#include <atomic>
#include <future>

template<class Container>
std::string ToString(const Container& c)
{
    std::stringstream ss;
    ss << "[";
    for (auto it = c.begin(); it != c.end(); ++it)
    {
        ss << std::format("{}", *it);
        if (std::next(it) != c.end())
        {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
}

using namespace std::literals;
using namespace std::chrono_literals;
using byte = uint8_t;

template<class It>
concept byte_Iterator = 
    std::is_same_v<std::decay_t<decltype(*It())>, byte> &&
    requires(It beg, It end)
{
    std::vector<byte>(beg, end);
};

/*
控制帧协议：
    SYNC0           : 1
    SYNC1           : 1
    <command>       : 1
    <payload_len>   : 2
    <payload>       : payload_len
    <crc>           : 1
*/
#define CTRL_SYNC0 0X55
#define CTRL_SYNC1 0Xaa
#define CTRL_CMD_SET_OFFSET 0X01    //payload为新窗口偏移
#define CTRL_CMD_FULL_SCAN 0X02     //payload为空
#define MINIMAL_CTRL_FRAME_LEN 6
#define MAX_CTRL_PAYLOAD_LEN 64

template<byte_Iterator It>
std::vector<byte> MakeCtrlFrame(byte cmd, It payload_beg, It payload_end)
{
    std::vector<byte> frame( 6 + (payload_end - payload_beg) );
    frame[0] = CTRL_SYNC0;
    frame[1] = CTRL_SYNC1;
    frame[2] = cmd;
    reinterpret_cast<uint16_t&>(frame[3]) = (uint16_t)(payload_end - payload_beg);
    frame.resize(5);
    frame.insert(frame.end(), payload_beg, payload_end);
    byte crc = std::accumulate(frame.begin(), frame.end(), 0);
    frame.push_back(crc);
    return frame;
}

#define SEND_BIN_COUNT 10
#define TOTAL_BIN_COUNT 40
#define CHANNEL_COUNT 8

#define FRAME_TOTAL_CRC_SIZE 4
#define FRAME_HEAD_SIZE 12
#define FRAMETYPE_OFFSET_FFTDATA 1
#define FRAMETYPE_FULLSCAN_FFTDATA 2
#define FRAMETYPE_SETOFFSET_ACK 3
#define FRAMETYPE_INFO 4
/*
数据帧协议:
    <head>              : 12
        0x55
        0xaa
        0x00
        0xff
        <type>          : 2
        <arg>           : 2
        <payload_len>   : 2
        <head_crc>      : 2
    <body>              : payload_len
    <crc>               : 4
        crc(head+body)  : 1
        crc + 1         : 1
        crc + 2         : 1
        crc + 3         : 1
*/

#define FULLSCAN_COUNT_PER_CMD  5
#define MINAMAL_FULLSCAN_COUNT_PER_CMD  3

nlohmann::json g_Config;

struct Complex16
{
	int16_t Imag;
	int16_t Real;
};

struct Frame
{
    Frame() = default;

    template<byte_Iterator It>
    Frame(uint16_t type, uint16_t arg, It payload_beg, It payload_end)
        :Head{
            .Type = type,
            .Arg = arg,
        }, Payload(payload_beg, payload_end)
    {
        
    }

    struct
    {
        byte Sync[4];
        uint16_t Type;
        uint16_t Arg;
        uint16_t PayloadLen;
        uint16_t HeadCRC;
    }Head;
    std::vector<byte> Payload{};
};

struct MonitorFrame
{
    MonitorFrame() = default;
    MonitorFrame(const Frame& frame)
    {
        assert(frame.Head.Type == FRAMETYPE_OFFSET_FFTDATA);
        Offset = frame.Head.Arg;
        memcpy_s(Data, sizeof(Data), frame.Payload.data(), frame.Payload.size());
        for (size_t bin = 0; bin < SEND_BIN_COUNT; bin++)
        {
            EnergyCurve[bin] = 0.0;
            for (size_t ch = 0; ch < CHANNEL_COUNT; ch++)
            {
                EnergyCurve[bin] += std::sqrt(
                    Data[ch][bin].Imag * Data[ch][bin].Imag +
                    Data[ch][bin].Real * Data[ch][bin].Real
                );
            }
        }
    }

    uint16_t Offset;
    Complex16 Data[CHANNEL_COUNT][SEND_BIN_COUNT];
    double EnergyCurve[SEND_BIN_COUNT];
};

struct InfoFrame
{
    InfoFrame() = default;
    InfoFrame(const Frame& frame)
    {
        assert(frame.Head.Type == FRAMETYPE_INFO);
        Text.resize(frame.Payload.size());
        memcpy_s(Text.data(), Text.size(), frame.Payload.data(), frame.Payload.size());
    }

    std::string Text;
};

struct ScanFrame
{
    ScanFrame() = default;
    ScanFrame(const Frame& frame)
    {
        assert(frame.Head.Type == FRAMETYPE_FULLSCAN_FFTDATA);
        memcpy_s(Data, sizeof(Data), frame.Payload.data(), frame.Payload.size());
        for (size_t bin = 0; bin < TOTAL_BIN_COUNT; bin++)
        {
            EnergyCurve[bin] = 0.0;
            for (size_t ch = 0; ch < CHANNEL_COUNT; ch++)
            {
                EnergyCurve[bin] += std::sqrt(
                    Data[ch][bin].Imag * Data[ch][bin].Imag +
                    Data[ch][bin].Real * Data[ch][bin].Real
                );
            }
        }
    }

    Complex16 Data[CHANNEL_COUNT][TOTAL_BIN_COUNT];
    double EnergyCurve[TOTAL_BIN_COUNT];
};

class UartReader
{
public:
    using LockGuard = std::lock_guard<std::mutex>;

    UartReader(const std::string& port_name, uint32_t baudrate)
    {
        try
        {
            _Uart.emplace(port_name, baudrate);
            _IsOpen = _Uart->isOpen();
        }
        catch (const serial::PortNotOpenedException& expt)
        {
            _ErrorMessage = expt.what();
        }
        catch (const serial::IOException& expt)
        {
            _ErrorMessage = expt.what();
        }
        catch (const std::invalid_argument& expt)
        {
            _ErrorMessage = expt.what();
        }
        catch (const std::exception& unknow_expt)
        {
            _ErrorMessage = unknow_expt.what();
        }

        if (IsOpen())
        {
            _ReaderThread = std::jthread([this](std::stop_token stoken) {
                _StopToken = stoken;
                _Run();
                });
        }
    }

    ~UartReader()
    {
        if (_IsOpen)
        {
            if (_ReaderThread.joinable())
            {
                _ReaderThread.request_stop();
                _ReaderThread.join();
            }
        }
    }

    bool IsOpen()const
    {
        return _IsOpen;
    }

    void RequestStop()
    {
        _ReaderThread.request_stop();
    }

    std::optional<std::string> GetErrorMessage()const
    {
        return _ErrorMessage;
    }

    size_t FrameCount()
    {
        LockGuard lg(_FrameQueueMutex);
        return _Frames.size();
    }

    Frame PopFrame()
    {
        LockGuard lg(_FrameQueueMutex);
        Frame frame = _Frames.front();
        _Frames.pop();
        return frame;
    }

    std::vector<Frame> PopFrame(size_t count)
    {
        std::vector<Frame> frames;
        frames.reserve(count);
        LockGuard lg(_FrameQueueMutex);
        assert(_Frames.size() >= count);
        for (size_t i = 0; i < count; i++)
        {
            frames.emplace_back(_Frames.front());
            _Frames.pop();
        }
        return frames;
    }

    std::vector<Frame> PopAllFrames()
    {
        LockGuard lg(_FrameQueueMutex);
        std::vector<Frame> frames;
        while (!_Frames.empty())
        {
            frames.push_back(_Frames.front());
            _Frames.pop();
        }
        return frames;
    }

    void ClearFrames()
    {
        LockGuard lg(_FrameQueueMutex);
        while (!_Frames.empty())
        {
            _Frames.pop();
        }
    }

protected:
    std::optional<std::string> _ErrorMessage;

    void _IgnoreBytes(size_t count)
    {
        _RawData.erase(_RawData.begin(), _RawData.begin() + count);
    }

    virtual void _OnUpdate()
    {

    }

private:
    bool _IsOpen = false;

    std::optional<Frame> _TempFrame;

    void _TryParseFrame()
    {
        BEGIN_PARSE:

        //检查头部
        if (!_TempFrame.has_value())
        {
            if (_RawData.size() < FRAME_HEAD_SIZE)return;

            if (_RawData[0] == 0x55 &&
                _RawData[1] == 0xaa &&
                _RawData[2] == 0x00 &&
                _RawData[3] == 0xff)
            {
                std::vector<byte> head_buf;
                head_buf.clear();
                head_buf.insert(head_buf.begin(), _RawData.begin(), _RawData.begin() + FRAME_HEAD_SIZE - sizeof(uint16_t));
                uint16_t* p = reinterpret_cast<uint16_t*>(head_buf.data());
                uint16_t crc = 0;
                for (size_t i = 0; i < (FRAME_HEAD_SIZE - sizeof(uint16_t))/2; i++)
                {
                    crc += p[i];
                }

                union {
                    struct 
                    {
                        byte CRC_LowByte;
                        byte CRC_HighByte;
                    };
                    uint16_t CRC;
                };
                CRC_LowByte = _RawData[FRAME_HEAD_SIZE - sizeof(uint16_t)];
                CRC_HighByte = _RawData[FRAME_HEAD_SIZE - sizeof(uint16_t) + 1];
                if (CRC == crc)
                {
                    _TempFrame.emplace();
                    std::vector<byte> head(_RawData.begin(), _RawData.begin() + FRAME_HEAD_SIZE);
                    memcpy_s(&_TempFrame->Head, FRAME_HEAD_SIZE, head.data(), FRAME_HEAD_SIZE);
                    _IgnoreBytes(FRAME_HEAD_SIZE);
                }
                else
                {
                    _IgnoreBytes(4);
                }
            }
            else
            {
                _IgnoreBytes(1);
            }
        }

        //如果已经读取了完整的头部，则等待接收足够的负载数据
        if (_TempFrame.has_value())
        {
            if (_RawData.size() < _TempFrame->Head.PayloadLen + FRAME_TOTAL_CRC_SIZE)
            {
                return;
            }

            auto& payload = _TempFrame->Payload;
            size_t payload_size = _TempFrame->Head.PayloadLen;
            payload.insert(payload.begin(), _RawData.begin(), _RawData.begin() + payload_size);
            _IgnoreBytes(payload_size);

            //检查crc
            byte crc = std::accumulate((byte*)&_TempFrame->Head, (byte*)&_TempFrame->Head + FRAME_HEAD_SIZE, 0);
            crc = std::accumulate(_TempFrame->Payload.begin(), _TempFrame->Payload.end(), crc);
            if (_RawData[0] == crc &&
                _RawData[1] == crc + 1 &&
                _RawData[2] == crc + 2 &&
                _RawData[3] == crc + 3)
            {
                LockGuard lg(_FrameQueueMutex);
                _Frames.emplace(std::move(_TempFrame.value()));
            }

            //重置接收状态
            _TempFrame.reset();
            _IgnoreBytes(FRAME_TOTAL_CRC_SIZE);
        }

        goto BEGIN_PARSE;
    }

    void _Run()
    {
        constexpr size_t buffer_size = 1024;
        std::array<byte, buffer_size> buffer;
        buffer.fill(0);

        while (!_StopToken.stop_requested())
        {
            _UartMutex.lock();
            if (_Uart->available() > 0)
            {
                auto count = _Uart->read(buffer.data(), std::min(buffer.size(), _Uart->available()));
                _RawData.insert(_RawData.end(), buffer.begin(), buffer.begin() + count);
            }
            else
            {
                std::this_thread::sleep_for(10us);
            }
            _UartMutex.unlock();
            _TryParseFrame();
            _OnUpdate();

            //防止挤压过多帧
            LockGuard lg(_FrameQueueMutex);
            while(_Frames.size() > 20)
            {
                _Frames.pop();
            }
        }
    }

protected:
    std::mutex _UartMutex;
    std::optional<serial::Serial> _Uart;
    std::jthread _ReaderThread;
    std::stop_token _StopToken;
    std::deque<byte> _RawData;
    std::queue<Frame> _Frames;
    std::mutex _FrameQueueMutex;
};

class Uart0Monitor:public UartReader
{
public:
    Uart0Monitor(const std::string& port_name, uint32_t baudrate)
        :UartReader(port_name, baudrate)
    {

    }

    MonitorFrame PopMonitorFrame()
    {
        return MonitorFrame(PopFrame());
    }
};

class Uart1Controller :public UartReader
{
public:
    using LockGuard = std::lock_guard<std::mutex>;
    using AfterScanCallback = std::function<void(const std::vector<ScanFrame>&)>;

    Uart1Controller(const std::string& port_name, uint32_t baudrate)
        :UartReader(port_name, baudrate)
    {

    }

    bool SetOffset(uint16_t offset)
    {
        if (_SetOffsetMutex.try_lock())
        {
            _NeedSetOffsetACK = true;
            for (size_t i = 0; i < 3; i++)
            {
                auto frame = MakeCtrlFrame((byte)CTRL_CMD_SET_OFFSET, (byte*)&offset, (byte*)&offset + 2);
                _UartMutex.lock();
                _Uart->write(frame);
                _Uart->flushOutput();
                _UartMutex.unlock();

                auto deadline = std::chrono::steady_clock::now() + 100ms;
                while (std::chrono::steady_clock::now() < deadline)
                {
                    if (!_NeedSetOffsetACK) //已经接收到了ack
                    {
                        _SetOffsetMutex.unlock();
                        return true;
                    }
                }
            }
            _SetOffsetMutex.unlock();
            return false;
        }
        return false;
    }

    void RequestFullScan(AfterScanCallback after_scan, std::chrono::steady_clock::duration retry_time)
    {
        _ScanMutex.lock();
        _Scanning = true;
        _NeedScanFrameCount = MINAMAL_FULLSCAN_COUNT_PER_CMD;
        //_NeedScanFrameCount = 1;
        _BeginScanTime = std::chrono::steady_clock::now();
        _RetryTime = retry_time;
        _ScanFrames.clear();
        _AfterScan = after_scan;
        _ScanMutex.unlock();

        _SendFullScanFrame();
    }

    uint16_t GetOffset()const
    {
        return _Offset;
    }

    bool Scanning()
    {
        LockGuard lg(_ScanMutex);
        return _Scanning;
    }

protected:
    void _OnUpdate()override
    {
        while (FrameCount() > 0)
        {
            Frame frame = PopFrame();
            if (frame.Head.Type == FRAMETYPE_INFO)
            {
                InfoFrame info(frame);
                if (!info.Text.ends_with('\n'))
                {
                    info.Text.append(1, '\n');
                }
                std::cout << std::format("[uart1 info]:{}", info.Text);
            }
            else if (frame.Head.Type == FRAMETYPE_FULLSCAN_FFTDATA)
            {
                LockGuard lg(_ScanMutex);
                if (!_Scanning)continue;

                _ScanFrames.emplace_back(ScanFrame(frame));

                std::cout << "接收到了全局帧，已接收" << _ScanFrames.size() << "帧\n";

                if (_ScanFrames.size() == _NeedScanFrameCount)  //扫描了足够的帧数
                {
                    if(_AfterScan)
                        _AfterScan(_ScanFrames);
                    _Scanning = false;
                    _ScanFrames.clear();
                }
            }
            else if (frame.Head.Type == FRAMETYPE_SETOFFSET_ACK)
            {
                _NeedSetOffsetACK = false;
            }
            else
            {
                std::cerr << "uart1接收到预期外的数据帧类型:" << frame.Head.Type << std::endl;
                assert(false);
            }
        }

        //检测全局扫描是否超时，如果超时，则重试
        LockGuard lg(_ScanMutex);
        if (_Scanning && _BeginScanTime + _RetryTime < std::chrono::steady_clock::now())
        {
            _BeginScanTime = std::chrono::steady_clock::now();
            _ScanFrames.clear();
            _SendFullScanFrame();
        }
    }

    void _SendFullScanFrame()
    {
        auto frame = MakeCtrlFrame((byte)CTRL_CMD_FULL_SCAN, (byte*)nullptr, (byte*)nullptr);
        LockGuard lg(_UartMutex);
        //_Uart->read(_Uart->available());
        _Uart->write(frame);
        _Uart->flushOutput();
    }

private:
    uint16_t _Offset = 5;
    std::mutex _ScanMutex;
    std::mutex _SetOffsetMutex;
    bool _Scanning = false;
    int _NeedScanFrameCount = 0;
    std::chrono::steady_clock::time_point _BeginScanTime{};
    std::chrono::steady_clock::duration _RetryTime{};
    AfterScanCallback _AfterScan;
    std::vector<ScanFrame> _ScanFrames;
    std::atomic_bool _NeedSetOffsetACK = false;
};

std::mutex g_UserOffsetMutex;
bool g_UseUserOffset = false;  //是否由用户指定偏移
uint16_t g_UserOffset = 5;
bool g_ShowAllBins = false;

std::mutex g_CurveMutex;
std::array<double, TOTAL_BIN_COUNT> g_MonitorCurveX;
std::array<double, TOTAL_BIN_COUNT> g_MonitorCurveY;
std::mutex g_GlobalCurveMutex;
std::vector<double> g_GlobalCurveY(TOTAL_BIN_COUNT);
void DataThread(std::stop_token stoken, Uart0Monitor& u0, Uart1Controller& u1)
{
    auto monitor_frame_count=                               g_Config["monitor-frame-count"].get<size_t>();
    auto read_timeout       =   std::chrono::milliseconds(  g_Config["read-timeout"].get<size_t>());
    //size_t total_bin_count  =                               g_Config["total-bin-count"].get<size_t>();
    auto peak_threshold_min =                               g_Config["peak-threshold-min"].get<double>();
    auto peak_threshold_max =                               g_Config["peak-threshold-max"].get<double>();
    auto scan_interval      =   std::chrono::milliseconds(  g_Config["scan-interval"].get<int>());

    u1.SetOffset(5);
    std::vector<MonitorFrame> recv_frames;
    for (size_t i = 0; i < TOTAL_BIN_COUNT; i++)
    {
        g_MonitorCurveX[i] = 5.0 * i;
    }
    auto last_read_time = std::chrono::high_resolution_clock::now();
    auto last_scan_time = std::chrono::high_resolution_clock::now();

    std::optional<size_t> last_peak_offset;
    std::optional<double> last_peak_value;

    auto set_offset_by_peak = [&](size_t peak_offset, size_t old_offset) -> bool {
        uint16_t new_offset = (uint16_t)(peak_offset - std::min(peak_offset, (size_t)SEND_BIN_COUNT / 2));
        new_offset = std::min((uint16_t)(TOTAL_BIN_COUNT - SEND_BIN_COUNT), new_offset);
        if (new_offset != old_offset)
        {
            bool success = u1.SetOffset(new_offset);
            return success;
        }
        return false;
        };

    //收集全局扫描数据
    std::atomic_bool fullscan_over = false; //标识是否完成了一次全局扫描
    auto process_fullscan_data = [&](const std::vector<ScanFrame>& data){
        //在uart1读线程中回调
        assert(!data.empty());
        auto task = std::async(
            [&](){
                auto& fake_peak = g_Config["fake-signal"];
                g_GlobalCurveMutex.lock();
                for (size_t bin = 0; bin < TOTAL_BIN_COUNT; bin++)
                {
                    g_GlobalCurveY[bin] = 0.0;
                    for (auto& frame : data)
                    {
                        g_GlobalCurveY[bin] += frame.EnergyCurve[bin];
                    }
                    g_GlobalCurveY[bin] /= data.size();

                    //手动去除假信号
                    if (bin < fake_peak.size())
                    {
                        g_GlobalCurveY[bin] = std::max(
                            g_GlobalCurveY[bin] - fake_peak[bin].get<double>(),
                            0.0
                        );
                    }
                }
                fullscan_over = true;

                std::cout << "完成全局扫描\n";
                auto peak_offset = std::max_element(g_GlobalCurveY.begin(), g_GlobalCurveY.end()) - g_GlobalCurveY.begin();
                auto peak_value = g_GlobalCurveY[peak_offset];
                set_offset_by_peak(peak_offset, u1.GetOffset());
                g_GlobalCurveMutex.unlock();
            }
        );
    };

    //启动后进行首次全局扫描
    std::cout << "正在等待首次全局扫描...\n";
    u1.RequestFullScan(process_fullscan_data, 1000ms);
    while (!stoken.stop_requested() && !fullscan_over)
    {
        std::this_thread::sleep_for(10ms);
    }

    //进入监测周期
    while (!stoken.stop_requested())
    {
        //读取监测数据
        while (recv_frames.size() < monitor_frame_count
            && std::chrono::steady_clock::now() < last_read_time + read_timeout)
        {
            if (u0.FrameCount())
            {
                auto frame = u0.PopMonitorFrame();
                if (frame.Offset < TOTAL_BIN_COUNT - SEND_BIN_COUNT)
                {
                    recv_frames.emplace_back(std::move(frame));
                }
                else
                {
                    std::cerr << std::format("收到异常偏移的帧:Offset={}\n", frame.Offset);
                }
            }
        }
        last_read_time = std::chrono::high_resolution_clock::now();
        if (recv_frames.empty())
        {
            std::cerr << "读取帧超时\n";
            continue;
        }

        //根据监测数据填充折线图
        g_CurveMutex.lock();
        g_MonitorCurveY.fill(0.0);
        std::array<int, TOTAL_BIN_COUNT> frame_count_per_bin;
        frame_count_per_bin.fill(0);
        for (auto& f : recv_frames)
        {
            for (size_t i = 0; i < SEND_BIN_COUNT; i++)
            {
                g_MonitorCurveY[f.Offset + i] += f.EnergyCurve[i];
                frame_count_per_bin[f.Offset + i]++;
            }
        }
        auto& fake_peak = g_Config["fake-signal"];
        for (size_t i = 0; i < TOTAL_BIN_COUNT; i++)
        {
            if (frame_count_per_bin[i] == 0)continue;
            g_MonitorCurveY[i] /= frame_count_per_bin[i];

            //手动去除伪高峰
            if (i < fake_peak.size())
            {
                g_MonitorCurveY[i] = std::max(
                    g_MonitorCurveY[i] - fake_peak[i].get<double>(),
                    0.0
                );
            }
        }
        g_CurveMutex.unlock();
        recv_frames.clear();

        //计算偏移
        auto peak_idx = std::max_element(g_MonitorCurveY.begin(), g_MonitorCurveY.end()) - g_MonitorCurveY.begin();
        double peak_value = g_MonitorCurveY[peak_idx];
        size_t peak_offset = peak_idx;
        bool need_rescan = false;
        if (last_peak_value.has_value())
        {
            //新旧峰值的大小相近和距离，则认为是峰值的迁移
            if (peak_value > last_peak_value.value() * peak_threshold_min &&
                peak_value < last_peak_value.value() * peak_threshold_max &&
                std::max(peak_offset, last_peak_offset.value()) - std::min(peak_offset, last_peak_offset.value()) < 2)
            {
                set_offset_by_peak(peak_offset, u1.GetOffset());
            }
            //否则，重扫
            else
            {
                need_rescan = true;
            }
        }

        //重新全量扫描
        if (g_ShowAllBins || need_rescan || last_scan_time + scan_interval < std::chrono::high_resolution_clock::now())
        {
            if (!g_UseUserOffset || g_ShowAllBins)
            {
                if (!u1.Scanning())
                {
                    std::cout << "开始全局扫描\n";
                    u1.RequestFullScan(process_fullscan_data, 1000ms);
                    last_scan_time = std::chrono::high_resolution_clock::now();
                }
            }
        }

        //手动指定偏移
        {
            std::lock_guard<std::mutex> lg(g_UserOffsetMutex);
            if (g_UseUserOffset && u1.GetOffset() != g_UserOffset)
            {
                u1.SetOffset(g_UserOffset);
            }
        }

        last_peak_offset = peak_offset;
        last_peak_value = peak_value;

        if (u0.FrameCount() > 100)  //防止积压过多未读取的帧
        {
            assert(monitor_frame_count < 100);
            u0.PopFrame(u0.FrameCount() - monitor_frame_count);
        }
    }
}

bool ReadConfig()
{
    std::ifstream fconfig("config.json");
    if (!fconfig.is_open())
    {
        std::cerr << "无法打开配置文件config.json" << std::endl;
        return false;
    }
    fconfig.seekg(0, std::ios::end);
    size_t file_length = fconfig.tellg();
    std::vector<char> config_content(file_length + 1);
    fconfig.seekg(0);
    fconfig.read(config_content.data(), file_length);
    config_content.back() = 0;
    g_Config = nlohmann::json::parse(config_content.data());
    return true;
}

int main()
{
    if (!ReadConfig())
    {
        return -1;
    }
    auto& config = g_Config;

    Uart1Controller controller(
        config["uart1"].get<std::string>(),
        config["uart1-baudrate"].get<uint32_t>()
    );
    if (!controller.IsOpen())
    {
        std::cerr << "uart1 打开失败\n";
        return -1;
    }

    //controller.RequestFullScan([](const std::vector<ScanFrame>&) {
    //    std::cout << "full scan over!\n";
    //    }, 200ms);
    //while (true)
    //{
    //    std::this_thread::sleep_for(100ms);
    //}

    Uart0Monitor monitor(
        config["uart0"].get<std::string>(), 
        config["uart0-baudrate"].get<uint32_t>()
	);
    if (!monitor.IsOpen())
    {
        std::cerr << "uart0 打开失败\n";
        return -1;
    }

    std::jthread data_thread(DataThread, std::ref(monitor),std::ref(controller));

	if (!ImGuiInit())
	{
		return -1;
	}

	RunGuiThread([&]() {
		//ImGui::ShowDemoWindow();
		//ImPlot::ShowDemoWindow();

        ImGui::Begin((const char*)u8"距离曲线");
        {
            ImGui::Checkbox((const char*)u8"显示所有bin", &g_ShowAllBins);
            ImGui::Text((const char*)u8"当前偏移:%d", (int)controller.GetOffset());
            ImGui::Checkbox((const char*)u8"手动指定偏移", &g_UseUserOffset);
            {
                std::lock_guard<std::mutex> lg(g_UserOffsetMutex);
                ImGui::SameLine();
                static int p_step = 5;
                ImGui::InputScalar((const char*)u8"偏移", ImGuiDataType_U16, &g_UserOffset, &p_step);
                g_UserOffset = std::clamp(g_UserOffset, (uint16_t)0, (uint16_t)TOTAL_BIN_COUNT);
            }
            if (ImPlot::BeginPlot((const char*)u8"距离曲线", ImGui::GetWindowSize() - ImGui::GetCursorPos()))
            {
                {
                    std::lock_guard<std::mutex> lg(g_CurveMutex);
                    ImPlot::SetupAxes((const char*)u8"距离(cm)", (const char*)u8"信号强度");
                    auto max_energy = *std::max_element(g_MonitorCurveY.begin(), g_MonitorCurveY.end());
                    auto min_energy = *std::min_element(g_MonitorCurveY.begin(), g_MonitorCurveY.end());
                    ImPlot::SetupAxesLimits(0.0, 5.0 * TOTAL_BIN_COUNT + 5.0, 0.0, std::max(1000.0, max_energy) * 1.8);

                    ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
                    ImPlot::PlotShaded((const char*)u8"监测曲线", g_MonitorCurveX.data(), g_MonitorCurveY.data(), TOTAL_BIN_COUNT);
                    ImPlot::PlotLine((const char*)u8"监测曲线", g_MonitorCurveX.data(), g_MonitorCurveY.data(), TOTAL_BIN_COUNT);
                    ImPlot::PopStyleVar();
                }

                if(g_ShowAllBins)
                {
                    std::lock_guard<std::mutex> lg(g_GlobalCurveMutex);
                    ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.15f);
                    ImPlot::PlotShaded((const char*)u8"全局曲线", g_MonitorCurveX.data(), g_GlobalCurveY.data(), TOTAL_BIN_COUNT);
                    ImPlot::PlotLine((const char*)u8"全局曲线", g_MonitorCurveX.data(), g_GlobalCurveY.data(), TOTAL_BIN_COUNT);
                    ImPlot::PopStyleVar();
                }

                ImPlot::EndPlot();
            }
        }
        ImGui::End();

		});

	ImGuiClose();

    data_thread.request_stop();
	return 0;
}

