// tx_ultimate_easy.h

#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/uart/uart_component.h"
#include <array>

namespace esphome {
    namespace tx_ultimate_easy {
        // Touch Max Position
        constexpr uint8_t TOUCH_MAX_POSITION = 10;

        // Touch State Constants
        constexpr uint8_t TOUCH_STATE_RELEASE = 0x01;
        constexpr uint8_t TOUCH_STATE_PRESS = 0x02;
        constexpr uint8_t TOUCH_STATE_SWIPE = 0x03;
        constexpr uint8_t TOUCH_STATE_MULTI_TOUCH = 0x0B;
        constexpr uint8_t TOUCH_STATE_SWIPE_RIGHT = 0x0C;
        constexpr uint8_t TOUCH_STATE_SWIPE_LEFT = 0x0D;

        // UART Constants
        constexpr int UART_RECEIVED_BYTES_SIZE = 15;
        constexpr int HEADER_BYTE_1 = 0xAA;
        constexpr int HEADER_BYTE_2 = 0x55;
        constexpr int VALID_DATA_BYTE_2 = 0x01;
        constexpr int VALID_DATA_BYTE_3 = 0x02;

        // Log tag
        static const char *TAG = "tx_ultimate_easy";

        struct TouchPoint {
            uint8_t button = 0;
            int8_t x = -1;
            int8_t state = -1;
            std::string state_str = "Unknown";
        };

        class TxUltimateEasy : public uart::UARTDevice, public Component {
        public:
            Trigger<TouchPoint> *get_trigger_touch_event() { return &this->trigger_touch_event_; }
            Trigger<TouchPoint> *get_trigger_touch() { return &this->trigger_touch_; }
            Trigger<TouchPoint> *get_trigger_release() { return &this->trigger_release_; }
            Trigger<TouchPoint> *get_trigger_swipe_left() { return &this->trigger_swipe_left_; }
            Trigger<TouchPoint> *get_trigger_swipe_right() { return &this->trigger_swipe_right_; }
            Trigger<TouchPoint> *get_trigger_multi_touch_release() { return &this->trigger_multi_touch_release_; }
            Trigger<TouchPoint> *get_trigger_long_touch_release() { return &this->trigger_long_touch_release_; }
            
            // Custom touch detection triggers
            Trigger<TouchPoint> *get_trigger_custom_click() { return &this->trigger_custom_click_; }
            Trigger<TouchPoint> *get_trigger_custom_long_click() { return &this->trigger_custom_long_click_; }

            void set_uart_component(esphome::uart::UARTComponent *uart_component) { this->set_uart_parent(uart_component); }

            void setup() override;
            void loop() override;
            void dump_config() override;

            uint8_t get_gang_count() { return this->gang_count_; }
            bool set_gang_count(const uint8_t gang_count);
            uint8_t get_button_from_position(const uint8_t position);
            
            uint32_t get_custom_long_click_threshold() { return this->custom_long_click_threshold_; }
            void set_custom_long_click_threshold(const uint32_t threshold);

        protected:
            void send_touch_(TouchPoint tp);
            void handle_touch(const std::array<int, UART_RECEIVED_BYTES_SIZE> &bytes);
            void reset_custom_touch_state_();

            TouchPoint get_touch_point(const std::array<int, UART_RECEIVED_BYTES_SIZE> &bytes);
            bool is_valid_data(const std::array<int, UART_RECEIVED_BYTES_SIZE> &bytes);
            int get_touch_position_x(const std::array<int, UART_RECEIVED_BYTES_SIZE> &bytes);
            int get_touch_state(const std::array<int, UART_RECEIVED_BYTES_SIZE> &bytes);

            Trigger<TouchPoint> trigger_touch_event_;
            Trigger<TouchPoint> trigger_touch_;
            Trigger<TouchPoint> trigger_release_;
            Trigger<TouchPoint> trigger_swipe_left_;
            Trigger<TouchPoint> trigger_swipe_right_;
            Trigger<TouchPoint> trigger_multi_touch_release_;
            Trigger<TouchPoint> trigger_long_touch_release_;
            
            // Custom touch detection
            Trigger<TouchPoint> trigger_custom_click_;
            Trigger<TouchPoint> trigger_custom_long_click_;

            uint8_t gang_count_ = 1;
            uint32_t custom_long_click_threshold_ = 500;  // Default 500ms
            
            // Touch state tracking
            bool touch_pressed_ = false;
            uint32_t touch_press_time_ = 0;
            uint8_t touch_press_position_ = 0;
            uint8_t touch_press_button_ = 0;

        }; // class TxUltimateEasy

    } // namespace tx_ultimate_easy
} // namespace esphome
