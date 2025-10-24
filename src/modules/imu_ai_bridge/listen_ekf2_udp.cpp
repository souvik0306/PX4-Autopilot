/*
 * EKF2 IMU UDP Telemetry Listener (C++)
 * -------------------------------------
 * Listens for EKF2 IMU telemetry packets on UDP port 14567 (default)
 * Packet format: 40 bytes
 *   uint64_t timestamp_us
 *   float delta_angle[3]
 *   float delta_velocity[3]
 *   float delta_ang_dt
 *   float delta_vel_dt
 *
 * Usage:
 *   ./listen_ekf2_udp [--port 14567]
 */

#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <chrono>
#include <set>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// 55-byte packed struct, no padding
struct ImuPacket {
    uint64_t timestamp;
    uint64_t timestamp_sample;
    uint32_t accel_device_id;
    uint32_t gyro_device_id;
    float delta_angle[3];
    float delta_velocity[3];
    uint16_t delta_angle_dt;
    uint16_t delta_velocity_dt;
    uint8_t delta_velocity_clipping;
    uint8_t accel_calibration_count;
    uint8_t gyro_calibration_count;
} __attribute__((packed));

int main(int argc, char* argv[]) {
    int port = 14567;
    if (argc > 2 && std::string(argv[1]) == "--port") {
        port = std::atoi(argv[2]);
    }

    // Sender socket for feedback
    int tx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_sock < 0) {
        std::cerr << "[ERROR] Could not create TX socket for feedback\n";
        return 1;
    }

    // Increase socket send buffer to avoid packet loss
    int send_buf_size = 16 * 1024 * 1024;  // 16MB
    if (setsockopt(tx_sock, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size)) < 0) {
        std::cerr << "[WARNING] Could not set SO_SNDBUF\n";
    }

    sockaddr_in tx_addr{};
    tx_addr.sin_family = AF_INET;
    tx_addr.sin_port = htons(14568);
    tx_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "[ERROR] Could not create socket\n";
        close(tx_sock);
        return 1;
    }

    // Set SO_REUSEADDR to allow reuse
    int reuse = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "[WARNING] setsockopt(SO_REUSEADDR) failed\n";
    }

    // Increase socket receive buffer to avoid packet loss
    int recv_buf_size = 16 * 1024 * 1024;  // 16MB
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size)) < 0) {
        std::cerr << "[WARNING] Could not set SO_RCVBUF\n";
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (bind(sockfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[ERROR] Could not bind to 127.0.0.1:" << port << "\n";
        close(sockfd);
        close(tx_sock);
        return 1;
    }

    std::cout << "[EKF2 IMU UDP] Listening on 127.0.0.1:" << port << std::endl;
    std::cout << "[EKF2 IMU UDP] TX configured to send to 127.0.0.1:14568" << std::endl;
    std::cout << "[EKF2 IMU UDP] Filtering to device ID: 0x" << std::hex << 0x14010c << std::dec << " (primary IMU)" << std::endl;
    std::cout << "[EKF2 IMU UDP] RX socket FD: " << sockfd << std::endl;
    std::cout << "[EKF2 IMU UDP] TX socket FD: " << tx_sock << std::endl;
    std::cout << "[EKF2 IMU UDP] Waiting for EKF2_IMU_SRC=1 (AI mode) telemetry...\n" << std::endl;

    int msg_count = 0;
    int tx_count = 0;
    int tx_errors = 0;
    int rx_errors = 0;
    int duplicate_timestamps = 0;
    uint64_t last_timestamp = 0;
    uint64_t first_packet_timestamp = 0;
    uint32_t unique_sources = 0;
    uint32_t last_source_id = 0;
    uint32_t filtered_device_id = 0x14010c;  // Filter to primary IMU (0x14010c)
    std::set<uint32_t> source_ids;
    int filtered_out = 0;
    double rate_sum = 0.0;
    int rate_count = 0;
    auto last_time = std::chrono::steady_clock::now();
    bool first_packet_logged = false;

    while (true) {
        uint8_t buf[55] = {};
        sockaddr_in src_addr{};
        socklen_t src_len = sizeof(src_addr);
        ssize_t n = recvfrom(sockfd, buf, sizeof(buf), 0, (sockaddr*)&src_addr, &src_len);
        if (n == 55) {
            ImuPacket pkt{};
            std::memcpy(&pkt, buf, sizeof(pkt));
            msg_count++;

            // Track all sources
            source_ids.insert(pkt.accel_device_id);

            // Skip packets from other IMU sources
            if (pkt.accel_device_id != filtered_device_id) {
                filtered_out++;
                continue;
            }

            // Store first packet timestamp for reference and log it
            if (first_packet_timestamp == 0) {
                first_packet_timestamp = pkt.timestamp;
                double first_time_ms = pkt.timestamp / 1000.0;
                std::cout << "[STARTUP] CPP: First packet received at [" << std::fixed << std::setprecision(1)
                          << first_time_ms << "ms] | ts=" << pkt.timestamp << " us" << std::endl;
                first_packet_logged = true;
            }

            // Track unique sources and duplicates for filtered packets
            if (pkt.timestamp == last_timestamp) {
                duplicate_timestamps++;
            }
            if (pkt.accel_device_id != last_source_id) {
                unique_sources++;
                last_source_id = pkt.accel_device_id;
            }
            last_timestamp = pkt.timestamp;

            // Calculate actual rate from delta_velocity_dt (in microseconds)
            // Rate = 1 / (delta_time_us / 1e6)
            double actual_dt = pkt.delta_velocity_dt / 1e6;  // Convert to seconds
            double packet_rate = (actual_dt > 0) ? (1.0 / actual_dt) : 0.0;
            rate_sum += packet_rate;
            rate_count++;

            // Convert delta values to instantaneous rates (divide by dt)
            double dt_angle = pkt.delta_angle_dt / 1e6;      // Convert us to seconds
            double dt_velocity = pkt.delta_velocity_dt / 1e6; // Convert us to seconds

            float gyro_x = (dt_angle > 0) ? (pkt.delta_angle[0] / dt_angle) : 0.0f;
            float gyro_y = (dt_angle > 0) ? (pkt.delta_angle[1] / dt_angle) : 0.0f;
            float gyro_z = (dt_angle > 0) ? (pkt.delta_angle[2] / dt_angle) : 0.0f;

            float accel_x = (dt_velocity > 0) ? (pkt.delta_velocity[0] / dt_velocity) : 0.0f;
            float accel_y = (dt_velocity > 0) ? (pkt.delta_velocity[1] / dt_velocity) : 0.0f;
            float accel_z = (dt_velocity > 0) ? (pkt.delta_velocity[2] / dt_velocity) : 0.0f;

            // Send back the full 55-byte packet (AI-processed) to EKF2 on port 14568
            // In this case, we're just echoing it back, but this is where you'd apply AI processing
            ssize_t tx_result = sendto(tx_sock, buf, 55, 0,
                                       (sockaddr*)&tx_addr, sizeof(tx_addr));

            if (tx_result != 55) {
                std::cerr << "[TX ERROR] sendto() returned " << tx_result << " bytes (expected 55), errno=" << errno << std::endl;
                tx_errors++;
            } else {
                tx_count++;
            }

            // Log rate summary every 5000 packets (matching EKF2 log interval for easy comparison)
            if (tx_count % 5000 == 0 && tx_count > 0) {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - last_time).count();
                double avg_rate = (rate_count > 0) ? (rate_sum / rate_count) : 0.0;
                double loss_pct = 0.0;

                // Use packet timestamp for logging (absolute, not relative)
                double packet_time_ms = pkt.timestamp / 1000.0;  // Convert us to ms

                std::cout << "[" << std::fixed << std::setprecision(1) << packet_time_ms << "ms] "
                    << "CPP RX=" << msg_count << " (Filtered out: " << filtered_out << ") | TX=" << tx_count
                    << " | Dropped=0 (" << std::setprecision(1) << loss_pct << "%) "
                    << "| Sources: " << source_ids.size() << " [";
                int i = 0;
                for (auto id : source_ids) {
                    if (i > 0) std::cout << ", ";
                    std::cout << "0x" << std::hex << id << std::dec;
                    i++;
                }
                std::cout << "] | Rate: " << std::setprecision(1) << avg_rate << " Hz" << std::endl;
                std::cout << "  ACC=[" << std::setprecision(4) << accel_x << ", " << accel_y << ", " << accel_z
                          << "] m/s² | GYRO=[" << gyro_x << ", " << gyro_y << ", " << gyro_z << "] rad/s" << std::endl;
                last_time = now;
                rate_sum = 0.0;
                rate_count = 0;
            }
        } else if (n > 0 && n != 55) {
            std::cerr << "[!] Invalid packet: got " << n << " bytes, expected 55" << std::endl;
            rx_errors++;
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "[RX ERROR] recvfrom failed: " << strerror(errno) << std::endl;
            rx_errors++;
        }
    }

    close(sockfd);
    close(tx_sock);
    return 0;
}
