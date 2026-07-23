#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <parameters/param.h>
#include <uORB/topics/vehicle_air_water_status.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>
#include <drivers/drv_hrt.h>

// ================================================================================
// PX4 自定义通信协议说明 (SBUS Bridge -> PX4)  【BDDB 加扰版】
// ================================================================================
// 帧格式 (发射端 24 字节):
//   00 | AD AD AD AD AD | FE | 10 | CRC | Data'[12] | BD DB BD
//                                    |<-------- 16 bytes -------->|
//   其中 0x10 = 后续载荷长度 16 = CRC(1) + Data'(12) + pad(3)
//   Data' = Data XOR (BD DB 交替)；CRC 仅覆盖 Data'；pad 不参与 CRC
//
// 接收同步:
//   连续 ≥3 个 AD，再接 FE 10，然后读 16 字节载荷
//   前导 00 / pad 不参与同步；不要求必须凑满 5 个 AD
//
// 载荷 (16 Bytes):
//     [0]     CRC-8 (多项式 0x31, 仅对 Data'[12])
//     [1-12]  Data'：解扰后再按位域解析
//     [13-15] pad BD DB BD (不参与 CRC)
//
// 数据位压缩逻辑 (Data Block, 96 bits):
//     * 通道 0-3 (摇杆) : 11位精度 (0-2047)  => 4 * 11 = 44 bits
//     * 通道 4-9 (辅助) :  6位精度 (0-63)    => 6 * 6  = 36 bits
//     * 帧计数          : 11位精度 (0-2047)  => 11 bits
//     * 预留            :  5位精度 (0-31)    =>  5 bits
//     * 总计: 96 bits = 12 Bytes
//
// 位映射 (Data Block 内, 从低位到高位):
//     [Bits 00-10] Ch0  (Roll)
//     [Bits 11-21] Ch1  (Pitch)
//     [Bits 22-32] Ch2  (Throttle)
//     [Bits 33-43] Ch3  (Yaw)
//     [Bits 44-49] Ch4
//     [Bits 50-55] Ch5
//     [Bits 56-61] Ch6
//     [Bits 62-67] Ch7
//     [Bits 68-73] Ch8
//     [Bits 74-79] Ch9
//     [Bits 80-90] Frame Count -> 填入 rc.values[13]
//     [Bits 91-95] Reserved    -> 填入 rc.values[14]
// ================================================================================

extern "C" __EXPORT int uart_telem3_main(int argc, char *argv[]);

namespace
{

#if defined(CONFIG_ARCH_BOARD_PX4_FMU_V6C)
	static const char *UART_DEVICE = "/dev/ttyS3";
#elif defined(CONFIG_ARCH_BOARD_PX4_FMU_V6X)
	static const char *UART_DEVICE = "/dev/ttyS4";
#else
	static const char *UART_DEVICE = "/dev/ttyS1";
#endif

static const int UART_DEFAULT_BAUD = 9600;

using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;

static uint16_t map_11bit_to_pwm(u32 val)
{
	return static_cast<uint16_t>(1000u + (val * 1000u) / 2047u);
}

static uint16_t map_6bit_to_pwm(u32 val)
{
	return static_cast<uint16_t>(1000u + (val * 1000u) / 63u);
}

static u32 extract_bits(const u8 *data, int start_bit, int num_bits)
{
	const int byte_idx = start_bit / 8;
	const int bit_offset = start_bit % 8;

	u32 raw = data[byte_idx];

	if (byte_idx + 1 < 12) {
		raw |= (static_cast<u32>(data[byte_idx + 1]) << 8);
	}

	if (byte_idx + 2 < 12) {
		raw |= (static_cast<u32>(data[byte_idx + 2]) << 16);
	}

	return (raw >> bit_offset) & ((1u << num_bits) - 1);
}

static constexpr u8 LEN_BYTE = 0x10;
static constexpr u8 PAYLOAD_LEN = 16; // CRC(1) + Data'(12) + pad(3)
static constexpr u8 STATE_PAYLOAD_BASE = 5;

static constexpr u8 TELEM_WHITEN[12] = {
	0xBD, 0xDB, 0xBD, 0xDB, 0xBD, 0xDB,
	0xBD, 0xDB, 0xBD, 0xDB, 0xBD, 0xDB
};

static volatile uint32_t _rx_bytes = 0;
static volatile uint32_t _frames_ok = 0;
static volatile uint32_t _frames_crc_err = 0;
static volatile uint32_t _frames_sum_match = 0;
static volatile uint32_t _frames_synced = 0;
static volatile uint32_t _sync_lost = 0;
static volatile uint32_t _seq_lost = 0;
static volatile uint32_t _last_seq = 0;
static volatile bool     _seq_inited = false;
static volatile uint32_t _last_ok_seq = 0;
static volatile hrt_abstime _last_ok_time = 0;
static volatile int32_t  _active_baud = UART_DEFAULT_BAUD;
static u8 _last_fail_frame[16] {};
static volatile bool _last_fail_valid = false;

static void reset_stats()
{
	_rx_bytes = 0;
	_frames_ok = 0;
	_frames_crc_err = 0;
	_frames_sum_match = 0;
	_frames_synced = 0;
	_sync_lost = 0;
	_seq_lost = 0;
	_last_seq = 0;
	_seq_inited = false;
	_last_ok_seq = 0;
	_last_ok_time = 0;
	_last_fail_valid = false;
}

static uint8_t Get_CRC8(const uint8_t *ptr, uint16_t len)
{
	uint8_t crc = 0x00;

	while (len--) {
		crc ^= *ptr++;

		for (uint8_t i = 0; i < 8; i++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x31;
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

static u8 recover_sync_state_from_buf(const u8 *buf, u8 len)
{
	for (int start = 0; start + 5 <= static_cast<int>(len); start++) {
		int ad = 0;

		while (start + ad < static_cast<int>(len) && buf[start + ad] == 0xAD) {
			ad++;
		}

		if (ad < 3) {
			continue;
		}

		const int fe_pos = start + ad;

		if (fe_pos + 1 < static_cast<int>(len) && buf[fe_pos] == 0xFE && buf[fe_pos + 1] == LEN_BYTE) {
			if (fe_pos + 2 == static_cast<int>(len)) {
				return STATE_PAYLOAD_BASE;
			}
		}
	}

	if (len >= 4 && buf[len - 1] == 0xFE) {
		u8 ad = 0;

		for (int i = static_cast<int>(len) - 2; i >= 0 && buf[i] == 0xAD; i--) {
			ad++;
		}

		if (ad >= 3) {
			return 4;
		}
	}

	u8 trailing_ad = 0;

	for (int i = static_cast<int>(len) - 1; i >= 0; i--) {
		if (buf[i] == 0xAD) {
			trailing_ad++;
		} else {
			break;
		}
	}

	if (trailing_ad >= 3) {
		return 3;
	}

	return trailing_ad;
}

#ifndef UART_TELEM3_IGNORE_CRC
# define UART_TELEM3_IGNORE_CRC 0
#endif

static bool Receive_Command(u8 *packet_buf,
			    uORB::Subscription &sub_status,
			    uORB::PublicationMulti<input_rc_s> &pub_input_rc)
{
	(void)sub_status;
	_frames_synced++;

	u8 checksum_rcv = packet_buf[0];
	const u8 *wire = &packet_buf[1];
	u8 checksum_cal = Get_CRC8(wire, 12);

	if (checksum_cal != checksum_rcv) {
		_frames_crc_err++;

		u8 sum = 0;

		for (int i = 0; i < 12; i++) {
			sum = static_cast<u8>(sum + wire[i]);
		}

		if (sum == checksum_rcv) {
			_frames_sum_match++;
		}

		for (int i = 0; i < 16; i++) {
			_last_fail_frame[i] = packet_buf[i];
		}

		_last_fail_valid = true;

#if !UART_TELEM3_IGNORE_CRC
		return false;
#endif
	}

	u8 data[12];

	for (int i = 0; i < 12; i++) {
		data[i] = static_cast<u8>(wire[i] ^ TELEM_WHITEN[i]);
	}

	uint16_t pwm[10];
	pwm[0] = map_11bit_to_pwm(extract_bits(data,  0, 11));
	pwm[1] = map_11bit_to_pwm(extract_bits(data, 11, 11));
	pwm[2] = map_11bit_to_pwm(extract_bits(data, 22, 11));
	pwm[3] = map_11bit_to_pwm(extract_bits(data, 33, 11));
	pwm[4] = map_6bit_to_pwm(extract_bits(data,  44, 6));
	pwm[5] = map_6bit_to_pwm(extract_bits(data,  50, 6));
	pwm[6] = map_6bit_to_pwm(extract_bits(data,  56, 6));
	pwm[7] = map_6bit_to_pwm(extract_bits(data,  62, 6));
	pwm[8] = map_6bit_to_pwm(extract_bits(data,  68, 6));
	pwm[9] = map_6bit_to_pwm(extract_bits(data,  74, 6));

	u32 frame_count = extract_bits(data, 80, 11);
	u32 reserved    = extract_bits(data, 91, 5);

	if (_seq_inited) {
		const uint32_t expected = (_last_seq + 1u) & 0x7FFu;

		if (frame_count != expected) {
			const uint32_t lost_this = (frame_count - expected) & 0x7FFu;

			if (lost_this > 0 && lost_this < 512u) {
				_seq_lost += lost_this;
			}
		}
	}

	_last_seq = frame_count;
	_seq_inited = true;

	input_rc_s rc{};
	const hrt_abstime now = hrt_absolute_time();
	rc.timestamp             = now;
	rc.timestamp_last_signal = now;
	rc.channel_count         = 15;
	rc.rssi                  = input_rc_s::RSSI_MAX;
	rc.rssi_dbm              = 0.0f;
	rc.rc_failsafe           = false;
	rc.rc_lost               = false;
	rc.rc_lost_frame_count   = static_cast<uint16_t>(_seq_lost > 0xFFFFu ? 0xFFFFu : _seq_lost);
	rc.rc_total_frame_count  = static_cast<uint16_t>(frame_count);
	rc.rc_ppm_frame_length   = 0;
	rc.input_source          = input_rc_s::RC_INPUT_SOURCE_PX4FMU_CRSF;
	rc.link_quality          = input_rc_s::RSSI_MAX;

	for (int i = 0; i < 10; ++i) {
		rc.values[i] = pwm[i];
	}

	for (int i = 10; i < 18; i++) {
		rc.values[i] = 1500;
	}

	rc.values[13] = static_cast<uint16_t>(frame_count);
	rc.values[14] = static_cast<uint16_t>(reserved);

	pub_input_rc.publish(rc);

	_frames_ok++;
	_last_ok_seq = frame_count;
	_last_ok_time = now;
	return true;
}

static void Ux_Receive(u8 *num, u8 *buf, u8 val,
		       uORB::Subscription &sub_status,
		       uORB::PublicationMulti<input_rc_s> &pub_input_rc)
{
	switch (*num) {
	case 0:
		if (val == 0xAD) { (*num)++; }
		break;

	case 1:
	case 2:
		if (val == 0xAD) { (*num)++; }
		else { _sync_lost++; *num = 0; }
		break;

	case 3:
		if (val == 0xFE) { (*num)++; }
		else if (val == 0xAD) { /* keep */ }
		else { _sync_lost++; *num = 0; }
		break;

	case 4:
		if (val == LEN_BYTE) { (*num)++; }
		else if (val == 0xAD) {
			_sync_lost++;
			*num = 1;
		} else {
			_sync_lost++;
			*num = 0;
		}
		break;

	default: {
		u8 payload_idx = *num - STATE_PAYLOAD_BASE;

		if (payload_idx < 32) {
			buf[payload_idx] = val;
		}

		(*num)++;

		if (payload_idx == (PAYLOAD_LEN - 1)) {
			const bool ok = Receive_Command(buf, sub_status, pub_input_rc);

			if (ok) {
				*num = 0;
			} else {
				*num = recover_sync_state_from_buf(buf, PAYLOAD_LEN);
			}
		}

		break;
	}
	}
}

static int open_uart(int baudrate)
{
	int fd = ::open(UART_DEVICE, O_RDWR | O_NOCTTY);

	if (fd < 0) {
		PX4_ERR("open %s failed: %d", UART_DEVICE, errno);
		return -1;
	}

	struct termios uart_config {};

	if (tcgetattr(fd, &uart_config) < 0) {
		PX4_ERR("tcgetattr failed on %s", UART_DEVICE);
		::close(fd);
		return -1;
	}

	cfmakeraw(&uart_config);
	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
	uart_config.c_cflag |= CS8;
	uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);
	uart_config.c_oflag &= ~ONLCR;

	speed_t speed;

	switch (baudrate) {
	case 4800: speed = B4800; break;
	case 9600: speed = B9600; break;
	case 57600: speed = B57600; break;
	case 115200: speed = B115200; break;
	default:
		speed = B9600;
		PX4_WARN("Unsupported baud %d, using 9600", baudrate);
		break;
	}

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("cfsetispeed/cfsetospeed failed on %s", UART_DEVICE);
		::close(fd);
		return -1;
	}

	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr failed on %s", UART_DEVICE);
		::close(fd);
		return -1;
	}

	int flags = fcntl(fd, F_GETFL, 0);

	if (flags >= 0) {
		fcntl(fd, F_SETFL, flags | O_NONBLOCK);
	}

	tcflush(fd, TCIOFLUSH);
	return fd;
}

static volatile bool _task_should_exit = false;
static volatile bool _is_running = false;
static int _task_handle = -1;

int task_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	_is_running = true;
	_task_should_exit = false;
	reset_stats();

	int32_t baud_param = UART_DEFAULT_BAUD;
	param_t ph_baud = param_find("U3_BAUD");

	if (ph_baud != PARAM_INVALID) {
		param_get(ph_baud, &baud_param);
	}

	_active_baud = baud_param;

	PX4_INFO("uart_telem3 BDDB RC on %s @ %ld", UART_DEVICE, (long)baud_param);

	int uart_fd = open_uart((int)baud_param);

	if (uart_fd < 0) {
		_is_running = false;
		return -1;
	}

	uORB::Subscription vehicle_air_water_status_sub{ORB_ID(vehicle_air_water_status)};
	uORB::PublicationMulti<input_rc_s> input_rc_pub{ORB_ID(input_rc)};

	u8 num_state = 0;
	u8 rcv_buf[32] {};
	uint8_t rx_byte;

	while (!_task_should_exit) {
		ssize_t nread = ::read(uart_fd, &rx_byte, 1);

		if (nread > 0) {
			_rx_bytes++;
			Ux_Receive(&num_state, rcv_buf, rx_byte,
				   vehicle_air_water_status_sub, input_rc_pub);
		} else {
			px4_usleep(1000);
		}
	}

	PX4_INFO("Exiting uart_telem3");
	::close(uart_fd);
	_is_running = false;
	return 0;
}

static void print_status()
{
	const uint32_t ok = _frames_ok;
	const uint32_t crc_err = _frames_crc_err;
	const uint32_t synced = _frames_synced;
	const uint32_t seq_lost = _seq_lost;
	const uint32_t sync_lost = _sync_lost;
	const uint32_t rx_bytes = _rx_bytes;

	float crc_err_pct = 0.f;

	if (synced > 0) {
		crc_err_pct = 100.f * static_cast<float>(crc_err) / static_cast<float>(synced);
	}

	const uint32_t denom = ok + crc_err + seq_lost;
	float loss_pct = 0.f;

	if (denom > 0) {
		loss_pct = 100.f * static_cast<float>(crc_err + seq_lost) / static_cast<float>(denom);
	}

	int age_ms = -1;

	if (_last_ok_time != 0) {
		age_ms = static_cast<int>((hrt_absolute_time() - _last_ok_time) / 1000);
	}

	PX4_INFO("uart_telem3 status");
	PX4_INFO("  running     : %s", _is_running ? "yes" : "no");
	PX4_INFO("  device      : %s", UART_DEVICE);
	PX4_INFO("  baud        : %ld", (long)_active_baud);
	PX4_INFO("  rx_bytes    : %lu", (unsigned long)rx_bytes);
	PX4_INFO("  frames_ok   : %lu", (unsigned long)ok);
	PX4_INFO("  frames_crc  : %lu (fail)", (unsigned long)crc_err);
	PX4_INFO("  sum_match   : %lu (crc-fail but old-sum ok)", (unsigned long)_frames_sum_match);
	PX4_INFO("  frames_sync : %lu (ok+crc)", (unsigned long)synced);
	PX4_INFO("  seq_lost    : %lu (from frame_count gaps)", (unsigned long)seq_lost);
	PX4_INFO("  sync_lost   : %lu (header desync)", (unsigned long)sync_lost);
	PX4_INFO("  crc_err_rate: %.2f %%", (double)crc_err_pct);
	PX4_INFO("  loss_rate   : %.2f %%  (crc_fail + seq_lost)", (double)loss_pct);
	PX4_INFO("  last_ok_seq : %lu", (unsigned long)_last_ok_seq);
	PX4_INFO("  last_ok_age : %d ms", age_ms);

	if (_last_fail_valid) {
		PX4_INFO("  last_fail   : %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
			 _last_fail_frame[0], _last_fail_frame[1], _last_fail_frame[2], _last_fail_frame[3],
			 _last_fail_frame[4], _last_fail_frame[5], _last_fail_frame[6], _last_fail_frame[7],
			 _last_fail_frame[8], _last_fail_frame[9], _last_fail_frame[10], _last_fail_frame[11],
			 _last_fail_frame[12], _last_fail_frame[13], _last_fail_frame[14], _last_fail_frame[15]);
	}
}

static u8 s_dump_buf[200];
static volatile unsigned s_dump_want = 200;
static volatile bool s_dump_running = false;

static int dump_raw_bytes(unsigned want_len)
{
	if (want_len == 0 || want_len > sizeof(s_dump_buf)) {
		want_len = sizeof(s_dump_buf);
	}

	if (_is_running) {
		PX4_WARN("dump: stopping reader for exclusive UART access");
		_task_should_exit = true;

		int i = 0;

		while (_is_running && i++ < 50) {
			px4_usleep(10000);
		}

		if (_is_running) {
			px4_task_delete(_task_handle);
			_is_running = false;
		}
	}

	int32_t baud_param = UART_DEFAULT_BAUD;
	param_t ph_baud = param_find("U3_BAUD");

	if (ph_baud != PARAM_INVALID) {
		param_get(ph_baud, &baud_param);
	}

	_active_baud = baud_param;

	int fd = open_uart((int)baud_param);

	if (fd < 0) {
		s_dump_running = false;
		return -1;
	}

	memset(s_dump_buf, 0, sizeof(s_dump_buf));
	unsigned got = 0;
	const hrt_abstime t0 = hrt_absolute_time();
	const hrt_abstime timeout_us = 3000000;

	PX4_INFO("dump %u bytes from %s @ %ld", want_len, UART_DEVICE, (long)baud_param);

	while (got < want_len) {
		ssize_t n = ::read(fd, &s_dump_buf[got], want_len - got);

		if (n > 0) {
			got += static_cast<unsigned>(n);
		} else {
			if ((hrt_absolute_time() - t0) > timeout_us) {
				break;
			}

			px4_usleep(1000);
		}
	}

	::close(fd);

	PX4_INFO("dump got %u/%u bytes", got, want_len);

	for (unsigned i = 0; i < got; i += 16) {
		const unsigned nline = (got - i > 16) ? 16 : (got - i);

		if (nline == 16) {
			PX4_INFO("%03u: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
				 i,
				 s_dump_buf[i], s_dump_buf[i + 1], s_dump_buf[i + 2], s_dump_buf[i + 3],
				 s_dump_buf[i + 4], s_dump_buf[i + 5], s_dump_buf[i + 6], s_dump_buf[i + 7],
				 s_dump_buf[i + 8], s_dump_buf[i + 9], s_dump_buf[i + 10], s_dump_buf[i + 11],
				 s_dump_buf[i + 12], s_dump_buf[i + 13], s_dump_buf[i + 14], s_dump_buf[i + 15]);
		} else {
			PX4_INFO("%03u: (partial %u)", i, nline);
		}

		px4_usleep(30000);
	}

	PX4_INFO("scan header AD*N(N>=3) FE 10:");
	unsigned found = 0;

	for (unsigned i = 0; i + 5 <= got;) {
		unsigned ad = 0;

		while (i + ad < got && s_dump_buf[i + ad] == 0xAD) {
			ad++;
		}

		if (ad >= 3) {
			const unsigned fe = i + ad;

			if (fe + 1 < got && s_dump_buf[fe] == 0xFE && s_dump_buf[fe + 1] == LEN_BYTE) {
				const unsigned crc_i = fe + 2;
				const u8 crc = (crc_i < got) ? s_dump_buf[crc_i] : 0;
				const u8 d0 = (crc_i + 1 < got) ? s_dump_buf[crc_i + 1] : 0;
				PX4_INFO("  hdr@%u AD=%u CRC=%02X DATA0=%02X", i, ad, crc, d0);
				found++;
				i = fe + 2;
				px4_usleep(20000);
				continue;
			}
		}

		i++;
	}

	if (found == 0) {
		PX4_INFO("  (no full header found)");
	}

	PX4_INFO("dump done. run: uart_telem3 start");
	s_dump_running = false;
	return 0;
}

static int dump_task_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	return dump_raw_bytes(s_dump_want);
}

} // namespace

int uart_telem3_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("usage: uart_telem3 {start|stop|status|dump [n]}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (_is_running) {
			PX4_WARN("already running");
			return 1;
		}

		_task_handle = px4_task_spawn_cmd("uart_telem3_task",
						  SCHED_DEFAULT,
						  SCHED_PRIORITY_DEFAULT,
						  2048,
						  (px4_main_t)&task_main,
						  nullptr);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (!_is_running) {
			PX4_WARN("not running");
			return 1;
		}

		_task_should_exit = true;
		int i = 0;

		while (_is_running && i++ < 10) {
			px4_usleep(10000);
		}

		if (_is_running) {
			PX4_WARN("Task didn't stop cleanly, forcing...");
			px4_task_delete(_task_handle);
			_is_running = false;
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		print_status();
		return 0;
	}

	if (!strcmp(argv[1], "dump")) {
		if (s_dump_running) {
			PX4_WARN("dump already running");
			return 1;
		}

		unsigned n = 200;

		if (argc >= 3) {
			n = static_cast<unsigned>(strtoul(argv[2], nullptr, 0));

			if (n == 0) {
				n = 200;
			}

			if (n > sizeof(s_dump_buf)) {
				n = sizeof(s_dump_buf);
			}
		}

		s_dump_want = n;
		s_dump_running = true;

		const int ret = px4_task_spawn_cmd("uart_telem3_dump",
						   SCHED_DEFAULT,
						   SCHED_PRIORITY_DEFAULT,
						   4096,
						   (px4_main_t)&dump_task_main,
						   nullptr);

		if (ret < 0) {
			s_dump_running = false;
			PX4_ERR("failed to spawn dump task");
			return 1;
		}

		PX4_INFO("dump task started, wait for hex lines...");
		return 0;
	}

	return 1;
}
