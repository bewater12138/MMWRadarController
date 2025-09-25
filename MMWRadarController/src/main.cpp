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

constexpr byte SYNC0 = 0X55;
constexpr byte SYNC1 = 0XAA;
constexpr byte CMD_SET_OFFSET = 0x01;
constexpr byte RSP_ACK = 0x81;
constexpr size_t SendBinCount = 10;
constexpr size_t SendCount = SendBinCount + 1;
constexpr size_t CHANNEL_BYTES = SendCount * 4;
constexpr size_t CHANNEL_COUNT = 8;
constexpr size_t FRAME_LENGTH = CHANNEL_COUNT * CHANNEL_BYTES;
nlohmann::json g_Config;

struct Complex16
{
	int16_t Imag;
	int16_t Real;
};

struct Frame
{
#pragma warning(disable: 26495)
    template<class It>
        requires requires(It beg, It end)
    {
        std::vector<byte>(beg, end);
    }
    Frame(It beg, It end)
        :RawData(beg, end)
    {
        Offset = reinterpret_cast<uint16_t&>(RawData[2]);
        for (size_t ch = 0; ch < CHANNEL_COUNT; ch++)
        {
            void* channel_ptr = &RawData[ch * CHANNEL_BYTES + 4];
            memcpy_s(Complex16Datas[ch].data(), SendBinCount * 4, channel_ptr, SendBinCount * 4);
        }

        double mag[SendBinCount][CHANNEL_COUNT];
        for (size_t ch = 0; ch < CHANNEL_COUNT; ch++)
        {
            for (size_t bin = 0; bin < SendBinCount; bin++)
            {
                mag[bin][ch] = std::sqrt(
                    Complex16Datas[ch][bin].Imag * Complex16Datas[ch][bin].Imag +
                    Complex16Datas[ch][bin].Real * Complex16Datas[ch][bin].Real
                );
            }
        }

        for (size_t bin = 0; bin < SendBinCount; bin++)
        {
            EnergyCurve[bin] = std::accumulate(&mag[bin][0], &mag[bin][CHANNEL_COUNT], 0.0);
        }

        //修复伪高峰
        auto& fake_signal = g_Config["fake-signal"];
        for (size_t i = 0; i < SendBinCount; i++)
        {
            if (i + Offset < fake_signal.size())
            {
                EnergyCurve[i] -= fake_signal[i + Offset];
                EnergyCurve[i] = std::max(0.0, EnergyCurve[i]);
            }
        }
    }
    Frame(const Frame&) = default;
    Frame(Frame&&) = default;
	Frame& operator=(const Frame&) = default;
	Frame& operator=(Frame&&) = default;

    uint16_t Offset;
	std::vector<byte> RawData;
    std::array<
        std::array<Complex16, SendBinCount>,
        CHANNEL_COUNT
    > Complex16Datas;
    std::array<double, SendBinCount> EnergyCurve;
};

byte CalCRC(uint32_t cmd, const std::vector<byte>& payloads)
{
    byte payload_sum = std::accumulate(payloads.begin(), payloads.end(), 0);
    byte crc = (byte)(cmd + payloads.size() + payload_sum) & 0xff;
    return crc;
}

class Uart0Reader
{
public:
    Uart0Reader(std::string_view port_name, uint32_t baudrate)
		:_Uart(port_name.data(), baudrate, serial::Timeout::simpleTimeout(1000))
    {
        if (_Uart.isOpen())
        {
            _ReaderThread = std::jthread([this](std::stop_token stoken) {
                _StopToken = stoken;
                _Run();
                });
        }
	}

    ~Uart0Reader()
    {
        if (_ReaderThread.joinable())
        {
            _ReaderThread.request_stop();
			_ReaderThread.join();
        }
        if(_Uart.isOpen())
            _Uart.close();
    }

    bool IsOpen()
    {
        return _Uart.isOpen();
    }

    void RequestStop()
    {
		_ReaderThread.request_stop();
    }

    size_t FrameCount()
    {
        auto lg = std::lock_guard<std::mutex>(_FrameQueueMutex);
		return _Frames.size();
    }

    Frame PopFrame()
    {
        auto lg = std::lock_guard<std::mutex>(_FrameQueueMutex);
		Frame frame = _Frames.front();
		_Frames.pop();
        return frame;
    }

    std::vector<Frame> PopFrame(size_t count)
    {
        std::vector<Frame> frames;
        frames.reserve(count);
        auto lg = std::lock_guard<std::mutex>(_FrameQueueMutex);
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
        auto lg = std::lock_guard<std::mutex>(_FrameQueueMutex);
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
        auto lg = std::lock_guard<std::mutex>(_FrameQueueMutex);
        while (!_Frames.empty())
        {
            _Frames.pop();
        }
    }

private:
    void _Run()
    {
		constexpr size_t buffer_size = 1024;
        static std::vector<byte> buffer(buffer_size);
        while (!_StopToken.stop_requested())
        {
            auto read_count = _Uart.read(buffer.data(),std::min(_Uart.available(), buffer_size));
			_RawData.insert(_RawData.end(), buffer.begin(), buffer.begin() + read_count);
            
			//寻找数据帧起始位置
            while(_RawData.size() >= FRAME_LENGTH)
            {
                auto start_pos = _FindFrameStart();
                if (start_pos.has_value())
                {
                    auto lg = std::lock_guard<std::mutex>(_FrameQueueMutex);
                    _RawData.erase(_RawData.begin(), _RawData.begin() + start_pos.value());
                    _Frames.emplace(Frame(_RawData.begin(), _RawData.begin() + FRAME_LENGTH));
                    _RawData.erase(_RawData.begin(), _RawData.begin() + FRAME_LENGTH);

					//防止队列无限增长
                    if(_Frames.size() > 100)
                    {
                        _Frames.pop();
					}
                }
				else if(_RawData.size() >= FRAME_LENGTH) //无法找到帧起始位置，丢弃无效部分数据
                {
                    _RawData.erase(_RawData.begin(), _RawData.end() - FRAME_LENGTH);
                    break;
                }
            }
        }
    }

    std::optional<size_t> _FindFrameStart()
    {
        for (size_t i = 0; i <= _RawData.size() - FRAME_LENGTH; i++)
        {
            bool ok = true;
            for (size_t ch = 0; ch < CHANNEL_COUNT; ch++)
            {
				size_t pos = i + ch * CHANNEL_BYTES;
                if (not (_RawData[pos] == 0x0a && _RawData[pos + 1] == 0x00))
                {
                    ok = false;
					break;
                }

                if (ch == 0)
                {
                    
                }
                else
                {
                    if (not (_RawData[pos + 2] == ch && _RawData[pos + 3] == 0x00))
                    {
                        ok = false;
						break;
                    }
                }
            }

            if (ok)
            {
                return i;
            }
        }

		std::cerr << "无法对数据流进行同步" << std::endl;
		return std::nullopt;
    }

private:
	std::jthread _ReaderThread;
	std::stop_token _StopToken;
	serial::Serial _Uart;
	std::deque<byte> _RawData;
	std::queue<Frame> _Frames;
	std::mutex _FrameQueueMutex;
};

class Uart1Controller
{
public:
    Uart1Controller(std::string_view port_name, uint32_t baudrate)
        :_Uart(port_name.data(), baudrate, serial::Timeout::simpleTimeout(200))
    {

    }

    bool IsOpen()
    {
        return _Uart.isOpen();
    }

    bool CmdSetOffset(uint16_t offset)
    {
        if (_Offset == offset)return true;

        //最多重试两次
        for (size_t i = 0; i < 3; i++)
        {
            _Uart.read(_Uart.available());
            auto frame = BuildSetOffsetFrame(offset);
            _Uart.write(frame);
            _Uart.flushOutput();
            auto start_time = std::chrono::high_resolution_clock::now();
            auto deadline = start_time + 200ms;
            while (std::chrono::high_resolution_clock::now() < deadline)
            {
                if (auto ack = _TryParseACKFrame())
                {
                    if (ack.value() == 0)
                    {
                        _Offset = offset;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    std::vector<byte> BuildSetOffsetFrame(uint16_t offset)
    {
        return {
            SYNC0,
            SYNC1,
            CMD_SET_OFFSET,
            0x02,
            (byte)offset,
            0,
            CalCRC(CMD_SET_OFFSET, {(byte)offset, 0})
        };
    }

    uint16_t Offset()
    {
        return _Offset;
    }

private:
    std::optional<byte> _TryParseACKFrame()
    {
        //ACK 帧: 55 AA 81 01 <status> <crc>
        constexpr size_t ack_frame_length = 6;

        //先从串口读出数据
        static byte recvbuf[64];
        auto count = _Uart.read(recvbuf, std::min(_Uart.available(), sizeof(recvbuf)));
        _RecvRawData.insert(_RecvRawData.end(), recvbuf + 0, recvbuf + count);

        while (_RecvRawData.size() >= ack_frame_length)
        {
            if (_RecvRawData.size() < ack_frame_length)continue;

            if (_RecvRawData[0] == 0x55 &&
                _RecvRawData[1] == 0xaa &&
                _RecvRawData[2] == 0x81 &&
                _RecvRawData[3] == 0x01)
            {
                auto cmd = _RecvRawData[2];
                auto length = _RecvRawData[3];
                auto payload = _RecvRawData[4];
                auto crc = (cmd + length + payload);
                if (crc == _RecvRawData[5]) //成功读取到确认帧
                {
                    _IgnoreBytes(ack_frame_length);
                    return payload;
                }
                else
                {
                    _IgnoreBytes(4);
                }
            }
            _IgnoreBytes(1);
        }

        return std::nullopt;
    }

    void _IgnoreBytes(size_t count)
    {
        _RecvRawData.erase(_RecvRawData.begin(), _RecvRawData.begin() + count);
    }

private:
    serial::Serial _Uart;
    std::deque<byte> _RecvRawData;
    uint16_t _Offset = 5;
};

std::mutex g_UserOffsetMutex;
bool g_UseUserOffset = false;  //是否由用户指定偏移
uint16_t g_UserOffset = 5;
bool g_ShowAllBins = false;

std::mutex g_CurveMutex;
std::array<double, SendBinCount> g_CurveX;
std::array<double, SendBinCount> g_CurveY;
std::mutex g_GlobalCurveMutex;
std::vector<double> g_GlobalCurveY;
void DataThread(std::stop_token stoken, Uart0Reader& u0, Uart1Controller& u1)
{
    auto monitor_frame_count=                               g_Config["monitor-frame-count"].get<size_t>();
    auto read_timeout       =   std::chrono::milliseconds(  g_Config["read-timeout"].get<size_t>());
    size_t total_bin_count  =                               g_Config["total-bin-count"].get<size_t>();
    auto peak_threshold_min =                               g_Config["peak-threshold-min"].get<double>();
    auto peak_threshold_max =                               g_Config["peak-threshold-max"].get<double>();
    auto scan_interval      =   std::chrono::milliseconds(  g_Config["scan-interval"].get<int>());

    u1.CmdSetOffset(5);
    uint16_t recvd_offset = u1.Offset();
    std::vector<Frame> recv_frames;
    auto last_read_time = std::chrono::high_resolution_clock::now();
    auto last_scan_time = std::chrono::high_resolution_clock::now();

    std::optional<size_t> last_peak_offset;
    std::optional<double> last_peak_value;

    auto set_offset_by_peak = [&](size_t peak_offset, size_t old_offset) {
        uint16_t new_offset = (uint16_t)(peak_offset - std::min(peak_offset, SendBinCount / 2));
        new_offset = std::min((uint16_t)(total_bin_count - SendBinCount), new_offset);
        if (new_offset != old_offset)
        {
            bool success = u1.CmdSetOffset(new_offset);
        }
        };

    auto full_scan = [&]()->bool {
        std::cout << "正在执行全量扫描...\n";

        std::vector<double> curve(total_bin_count);

        //扫描所有bin
        uint16_t cur_offset = 0;
        for (; cur_offset + SendBinCount <= total_bin_count; cur_offset+=SendBinCount)
        {
            u0.ClearFrames();
            if (!u1.CmdSetOffset(cur_offset))
            {
                std::cout << "全量扫描失败\n";
                return false;
            }

            for (size_t i = 0; i < monitor_frame_count; i++)
            {
                auto start_time = std::chrono::high_resolution_clock::now();
                while (start_time + read_timeout > std::chrono::high_resolution_clock::now())
                {
                    if (u0.FrameCount() == 0)continue;
                    Frame frame = u0.PopFrame();
                    if (frame.Offset == cur_offset)
                    {
                        for (size_t j = 0; j < SendBinCount; j++)
                        {
                            curve[cur_offset + j] += frame.EnergyCurve[j];
                        }
                        break;
                    }
                }
            }
            for (size_t j = 0; j < SendBinCount; j++)
            {
                curve[cur_offset + j] /= (double)monitor_frame_count;
            }
        }

        g_GlobalCurveMutex.lock();
        g_GlobalCurveY = curve;
        g_GlobalCurveMutex.unlock();

        //找出峰值

        auto peak_idx = std::max_element(curve.begin(), curve.end()) - curve.begin();
        last_peak_offset = peak_idx;
        last_peak_value = curve[peak_idx];
        set_offset_by_peak(peak_idx, cur_offset);

        std::cout << std::format("全量扫描结束, peak_offset = {}\n", peak_idx);
        return true;
        };

    //启动后进行首次全量扫描
    while (!full_scan())
    {
        std::cerr << "重试全量扫描\n";
    }

    while (!stoken.stop_requested())
    {
        uint16_t offset = u1.Offset();
        if (recvd_offset != offset)
        {
            recv_frames.clear();
        }

        while (recv_frames.size() < monitor_frame_count
            && std::chrono::high_resolution_clock::now() < last_read_time + read_timeout)
        {
            if (u0.FrameCount())
            {
                auto frame = u0.PopFrame();
                if (frame.Offset != offset)continue;

                recv_frames.emplace_back(frame);
                recvd_offset = frame.Offset;
            }
        }

        if (recv_frames.empty())
        {
            std::cerr << "读取帧超时\n";
            last_read_time = std::chrono::high_resolution_clock::now();
            continue;
        }

        //读取了足够的帧数或者到达指定时间，则开始填充折线图数据
        g_CurveMutex.lock();
        g_CurveY.fill(0.0);
        for (auto& f : recv_frames)
        {
            for (size_t i = 0; i < SendBinCount; i++)
            {
                g_CurveY[i] += f.EnergyCurve[i];
            }
        }
        for (auto& e : g_CurveY)
        {
            e /= (double)recv_frames.size();
        }
        for (size_t i = 0; i < SendBinCount; i++)
        {
            g_CurveX[i] = offset * 5.0 + i * 5.0;
        }
        g_CurveMutex.unlock();

        recv_frames.clear();
        last_read_time = std::chrono::high_resolution_clock::now();

        //计算偏移
        auto peak_idx = std::max_element(g_CurveY.begin(), g_CurveY.end()) - g_CurveY.begin();
        double peak_value = g_CurveY[peak_idx];
        size_t peak_offset = peak_idx + offset;
        bool need_rescan = false;
        if (last_peak_value.has_value())
        {
            //新旧峰值的大小相近，则认为是峰值的迁移
            if (peak_value > last_peak_value.value() * peak_threshold_min &&
                peak_value < last_peak_value.value() * peak_threshold_max)
            {
                set_offset_by_peak(peak_offset, offset);
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
                full_scan();
                last_scan_time = std::chrono::high_resolution_clock::now();
            }
        }

        //手动指定偏移
        {
            std::lock_guard<std::mutex> lg(g_UserOffsetMutex);
            if (g_UseUserOffset && u1.Offset() != g_UserOffset)
            {
                u1.CmdSetOffset(g_UserOffset);
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

    Uart0Reader reader(
        config["uart0"].get<std::string>(), 
        config["uart0-baudrate"].get<uint32_t>()
	);
    if (!reader.IsOpen())
    {
        std::cerr << "uart0 打开失败\n";
        return -1;
    }

    Uart1Controller controller(
        config["uart1"].get<std::string>(),
        config["uart1-baudrate"].get<uint32_t>()
    );
    if (!controller.IsOpen())
    {
        std::cerr << "uart1 打开失败\n";
        return -1;
    }

    std::jthread data_thread(DataThread, std::ref(reader),std::ref(controller));

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
            if (g_ShowAllBins)
            {
                if (ImPlot::BeginPlot((const char*)u8"距离曲线", ImGui::GetWindowSize() - ImGui::GetCursorPos()))
                {
                    std::lock_guard<std::mutex> lg(g_GlobalCurveMutex);
                    ImPlot::SetupAxes((const char*)u8"距离(cm)", (const char*)u8"信号强度");
                    std::vector<double> curvex(g_GlobalCurveY.size());
                    for (size_t i = 0; i < curvex.size(); i++)
                    {
                        curvex[i] = 5.0 * i;
                    }
                    auto max_energy = *std::max_element(g_GlobalCurveY.begin(), g_GlobalCurveY.end());
                    auto min_energy = *std::min_element(g_GlobalCurveY.begin(), g_GlobalCurveY.end());
                    ImPlot::SetupAxesLimits(0.0, 5.0 * g_Config["total-bin-count"].get<size_t>() + 5.0, 0.0, std::max(1000.0, max_energy) * 1.8);

                    ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
                    ImPlot::PlotShaded((const char*)u8"距离曲线", curvex.data(), g_GlobalCurveY.data(), curvex.size());
                    ImPlot::PlotLine((const char*)u8"距离曲线", curvex.data(), g_GlobalCurveY.data(), curvex.size());
                    ImPlot::PopStyleVar();
                    ImPlot::EndPlot();
                }
            }
            else
            {
                ImGui::Text((const char*)u8"当前偏移:%d", (int)controller.Offset());
                ImGui::Checkbox((const char*)u8"手动指定偏移", &g_UseUserOffset);
                {
                    std::lock_guard<std::mutex> lg(g_UserOffsetMutex);
                    ImGui::SameLine();
                    static int p_step = 5;
                    ImGui::InputScalar((const char*)u8"偏移", ImGuiDataType_U16, &g_UserOffset, &p_step);
                }
                if (ImPlot::BeginPlot((const char*)u8"距离曲线", ImGui::GetWindowSize() - ImGui::GetCursorPos()))
                {
                    std::lock_guard<std::mutex> lg(g_CurveMutex);
                    ImPlot::SetupAxes((const char*)u8"距离(cm)", (const char*)u8"信号强度");
                    auto max_energy = *std::max_element(g_CurveY.begin(), g_CurveY.end());
                    auto min_energy = *std::min_element(g_CurveY.begin(), g_CurveY.end());
                    ImPlot::SetupAxesLimits(0.0, 5.0 * g_Config["total-bin-count"].get<size_t>() + 5.0, 0.0, std::max(1000.0, max_energy) * 1.8);

                    ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
                    ImPlot::PlotShaded((const char*)u8"距离曲线", g_CurveX.data(), g_CurveY.data(), SendBinCount);
                    ImPlot::PlotLine((const char*)u8"距离曲线", g_CurveX.data(), g_CurveY.data(), SendBinCount);
                    ImPlot::PopStyleVar();
                    ImPlot::EndPlot();
                }
            }
        }
        ImGui::End();

		});

	ImGuiClose();

    data_thread.request_stop();
	return 0;
}

